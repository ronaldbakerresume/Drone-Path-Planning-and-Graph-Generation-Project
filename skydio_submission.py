import pyproj
import pydeck as pdk
import pandas as pd
import itertools
import math
import random
from heapq import heappush, heappop
from OSMPythonTools.nominatim import Nominatim
from OSMPythonTools.overpass import overpassQueryBuilder, Overpass

class GenerateGraph:
  """
    GenerateGraph creates a graph between the coordinates in feasible travel
    range of the drone.
  """

  def __init__ (self, location : str, elementType : str,  selector : str):
    """
    __init__ Enables the generation of the graph required for path planning
    :param location: The bounding box created to collect information
    :param elementType: The type of data being queried
    :param selector: The descriptor of the data in question the graph is
    querying for.
    """

    self.drone = Drone()
    self.graph = Graph()
    self.nominatim = Nominatim()
    self.overpass = Overpass()
    self.geodesic = pyproj.Geod(ellps='WGS84')
    self.location = location
    self.elementType = elementType
    self.selector = selector
    self.noNameMetaData = {}
    self.metaData = {}
    self.debug = True
    self.result = None
    self.query = None
    self.count = None

    """
    Generate graph during object initialization.
    """
    self._generate_graph()

  def _collect_regional_data(self) -> int:
    """
    _collect_regional_data Queries data from overpassQueryBuilder based on set
    member variable attributes
    :return: The number of elements queried
    """

    self.query = overpassQueryBuilder(bbox=self.location,
                                      elementType=self.elementType,
                                      selector=self.selector,
                                      out='body')
    self.result = self.overpass.query(self.query)
    self.count = self.result.countElements()

    if self.debug:

      print('\n\nLightHouse Total count -> %s \n\n\n' % self.count)


    return self.count

  def _generate_graph(self) -> None:
    """
    _generate_graph Produces graph vertex nodes based on the ability of the
    drone being able to make it to the next location without running out of
    energy. Since the drone must recharge once reaching its final destination,
    it is assumed the drone will wait at next location until fully charged
    """

    count = self._collect_regional_data()
    unknownCount = 0

    """
      Iterate through each of the nodes and conduct comparison between all
      possible vertex.
    """
    for pIndex in range(0, count):
      elementPrime = self.result.nodes()[pIndex]
      pName = elementPrime.tag("name")
      pLon = elementPrime.lon()
      pLat = elementPrime.lat()

      """
      Return to the top of the loop if the name has not been set.
      """
      if pName is None:
        self.noNameMetaData["Unknown: " + str(unknownCount)] = [pLon,pLat]
        unknownCount += 1
        continue

      self.metaData[pName] = [pLon,pLat]
      vertex = []

      """
      Iterate through each of the nodes collected and check if there is a
      possibility if the drone can make it to the next location directly.
      """
      for sIndex in range(0, count):
        elementSecondary = self.result.nodes()[sIndex]
        sName = elementSecondary.tag("name")
        sLon = elementSecondary.lon()
        sLat = elementSecondary.lat()

        """
        Bypass the algorithm check if the primary and secondary elements are the
        same and if the name isn't set to a specific number.
        """
        if elementPrime is elementSecondary or sName is None:
          continue

        """
        Calculate the distance between the two geo-locations.
        """
        fwd_azimuth,back_azimuth,distance = self.geodesic.inv(sLon,
                                                              sLat,
                                                              pLon,
                                                              pLat)

        """
        Add to list of vertex if the drone can make it to this next location.
        """
        if self.drone.can_drone_make_it(distance):
            edge = Edge()
            edge.distance = distance
            edge.vertex = sName
            vertex.append(edge)

      """
      Store all collected vertex to the proper main node.
      """
      self.graph.children[pName] = vertex

    if self.debug:
      print("\nResults of children nodes of each node: \n")
      for entry in self.graph.children:
        vCount = str(len(self.graph.children[entry]))
        print("\t\t Lighthouse Name : " + entry +
              "\n\t\t\t [children count -> " + vCount + "]")

class Graph:
  """
    Graph stores a dictionary correlated to the vertex associated with a
    particular data point
  """

  def __init__ (self):
    self.children = {}

class Edge:
  """
    Edge stores the time to travel and rest from the primary coordinate to the
    secondary cooridinate. The name of the vertex is also stored.
  """

  def __init__(self):
    self.distance = None
    self.vertex = None

class Drone:
  """
    Edge stores the time to travel and rest from the primary coordinate to the
    secondary cooridinate. The name of the vertex is also stored.
  """

  def __init__ (self):
    """
    __init__ initialize battery life and charge time of the drone. The speed at
    which the drone moves is also stored.
    """
    self.batteryLifeInSeconds = 30 * 60
    self.speedMetersPerSecond = 12
    self.chargingTimeInSeconds = 45 * 60
    self._calculate_drone_max_distance()

  def _calculate_drone_max_distance(self) -> None:
    """
    _calculate_drone_max_distance Calculates max distance in meters
    """
    self.max_disance = self.batteryLifeInSeconds * self.speedMetersPerSecond


  def can_drone_make_it(self,distance : float) -> bool:
    """
    can_drone_make_it Calculates whether it is possible for the drone to travel
    the passed distance without running out of battery.
    :param distance: The range of travel in meters
    """

    if distance <= self.max_disance:
      return True
    return False

class PriorityQueue:
  """
  PriorityQueue Generates a queue used of the dijkstra algorithm for adding,
  updating, and removing elements necessary for properly designing the
  traversal graph.
  """

  def __init__(self):
    """
    __init__ Initialize priority queue and additional information required for
    maintaining the queue.
    """
    self.pq = []
    self.entry_finder = {}
    self.counter = itertools.count()

  def __len__(self) -> int:
    """
    __len__ Provides the current size of the priority queue.
    :return: Priority Queue current size.
    """

    return len(self.pq)

  def add_task(self, priority : float , task : str):
    """
    add_task Add a new task or update the priority of an existing task.
    :param priority: The value associated with importance of task.
    :param task: The name of the task being stored.
    """

    if task in self.entry_finder:
      self.update_priority(priority,task)
      return self

    count = next(self.counter)
    entry = [priority, count, task]
    self.entry_finder[task] = entry
    heappush(self.pq, entry)

  def update_priority(self, priority : float , task : str) -> None:
    """
    update_priority Update the priority of a task. Raise KeyError if not found.
    :param priority: The value associated with importance of task
    :param task: The name of the task being stored.
    """

    entry = self.entry_finder[task]
    count = next(self.counter)
    entry[0], entry[1] = priority, count

  def pop_task(self) -> None:
    """
    pop_task Remove and return the lowest priority task, Raise KeyError if
    empty.
    """

    """
    Enter if there is any data stored in the priority queue.
    """
    while self.pq:
      priority, count, task = heappop(self.pq)
      del self.entry_finder[task]
      return priority, task

class Algorithms:
  """
  Algorithms Provide an interface for working with necessary algorithms of path
  planning.
  """

  def dijkstra(self, graph : Graph, start: str ):
    """
    update_priority Update the priority of a task. Raise KeyError if not found.
    :param graph: The completely generated graph between all the collected
    coordinate points.
    :param start: The name of first location of the algorithm.
    :return: Dictionary correlated to the previous traversal points of each of
    the coordinate points.
    """

    previous = {v: None for v in graph.children.keys()}
    visited = {v: False for v in graph.children.keys()}
    distance = {v: float("inf") for v in graph.children.keys()}
    distance[start] = 0
    queue = PriorityQueue()
    queue.add_task(0,start)

    """
    Iterate while there's elements held within the queue.
    """
    while queue:
      removed_time, removed = queue.pop_task()
      visited[removed] = True

      """
        Iterate through all the edges of the graph
      """
      for edge in graph.children[removed]:
        if visited[edge.vertex]:
          continue

        new_distance = removed_time + edge.distance

        """
        Change the time required to get to new location if a better route option
        is found.
        """
        if new_distance < distance[edge.vertex]:
          distance[edge.vertex] = new_distance
          previous[edge.vertex] = removed
          queue.add_task(new_distance, edge.vertex)

    return previous, distance


class DrawGraph:
  """
  DrawGraph Provides an interface for displaying collect graph data to html
  page.
  """

  def draw_path(self, path : list,
                points : list,
                noNamePoints : list,
                viewportLat : float,
                viewportLon : float,
                zoom : int ) -> None:
    """
    draw_path Creates a randomized line graph that is transposed over the map.
    """

    def hex_to_rgb(h : str) -> tuple:
      """
      hex_to_rgb Generates a randomized hex value correlated to a particular
      rgb value.
      :return: Tuple correlated to randomly generated RGB value.
      """
      h = h.lstrip("#")
      return tuple(int(h[i : i + 2], 16) for i in (0, 2, 4))

    df = pd.DataFrame(path)
    df["color"] = df["color"].apply(hex_to_rgb)

    """
    Generate viewport.
    """
    view_state = pdk.ViewState(latitude=viewportLat,
                               longitude=viewportLon,
                               zoom=zoom)

    """
    Generate layers over viewport.
    """

    """
    Line Plot
    """
    layer1 = pdk.Layer(
      type="PathLayer",
      data=df,
      pickable=True,
      get_color="color",
      width_scale=20,
      width_min_pixels=5,
      get_path="path",
      get_width=5,
      )

    df = pd.DataFrame(points)

    """
    name Scatter Plot
    """
    layer2 = pdk.Layer(
      "ScatterplotLayer",
      df,
      pickable=True,
      opacity=0.8,
      stroked=True,
      filled=True,
      radius_scale=6,
      radius_min_pixels=1,
      radius_max_pixels=100,
      line_width_min_pixels=1,
      get_position="coordinates",
      get_radius=100,
      get_fill_color=[255, 140, 0],
      get_line_color=[0, 0, 0],
    )

    df = pd.DataFrame(noNamePoints)

    """
    Unnamed Scatter Plot
    """
    layer3 = pdk.Layer(
      "ScatterplotLayer",
      df,
      pickable=True,
      opacity=0.4,
      stroked=True,
      filled=True,
      radius_scale=6,
      radius_min_pixels=1,
      radius_max_pixels=100,
      line_width_min_pixels=1,
      get_position="coordinates",
      get_radius=200,
      get_fill_color=[0, 140, 230],
      get_line_color=[0, 0, 0],
    )

    """
    Render layers to html page
    """
    render = pdk.Deck(layers=[layer1,layer2, layer3],
                      initial_view_state=view_state)
    render.to_html("path_layer.html")


class Submission:
  """
  Submission handles submission formats
  """

  def __init__(self):
    """
    __init__ Initialize submission data
    """
    self.checkPoints = { "Stockholm -> Vindö -> Torö":
     ["Bottenholmen", "Branten", "Blockhusudden"],
                         "Stockholm -> Torö": ["Bottenholmen", "Blockhusudden"]}
    self.boundingBox = [58.588299,16.795349,59.505061,20.223083]
    self.viewportLat = 59.088299
    self.viewportLon = 18.795349
    self.zoom = 8
    self.selector = "man_made=lighthouse"
    self.elementType = "node"
    self.generateGraph = GenerateGraph(self.boundingBox,
                                       self.elementType,
                                       self.selector)

  def draw_UI(self) -> None:
    """
    draw_UI Initialize submission data
    """

    knownPoints = []
    noNamePoints = []
    fullPath = []

    for path in self.checkPoints:
      pattern = self.checkPoints[path]
      stopCount = 0
      timeCount = 0
      for point in range(0, len(pattern) - 1 ):
        path = []
        pathSequence = []
        start = pattern[point]
        end = pattern[point + 1]
        color = "%06x" % random.randint(0, 0xFFFFFF)
        fullPath.append({"name" : ("From: " + start +  " To: " + end),
                          "color": color,
                          "path" : path})

        result, times  = Algorithms().dijkstra(self.generateGraph.graph,start)

        if result[end] != None:
          while end != start:
            pathSequence.append(end)
            end = result[end]
          pathSequence.append(start)

        stopCount += len(pathSequence)

        for lighthouse in pathSequence:
          timeCount += times[lighthouse]
          path.append(self.generateGraph.metaData[lighthouse])

    for point in self.generateGraph.metaData:
      lat = self.generateGraph.metaData[point][0]
      lon = self.generateGraph.metaData[point][1]
      knownPoints.append({"name" : point, "coordinates" : [lat, lon]})

    for point in self.generateGraph.noNameMetaData:
      lat = self.generateGraph.noNameMetaData[point][0]
      lon = self.generateGraph.noNameMetaData[point][1]
      noNamePoints.append({"name" : point, "coordinates" : [lat, lon]})

    DrawGraph().draw_path(fullPath,
                          knownPoints,
                          noNamePoints,
                          self.viewportLat,
                          self.viewportLon,
                          self.zoom)


  def print_results(self) -> None:
    """
    print_results Prints data supporting submission
    """

    gg = self.generateGraph
    drone = gg.drone
    data = gg.metaData
    graph = gg.graph

    fullPath = []
    logger = "\n\n\n___________________Submission Report___________________\n\n"
    logger += "Drone Stats:\n\n "
    logger += "\tMax Flight Distance %d meters\n" % (drone.max_disance)
    logger += "\tMax Speed %d meters per second\n" % (drone.speedMetersPerSecond)
    logger += "\tCharge Time %d seconds\n\n" % (drone.chargingTimeInSeconds)

    for path in self.checkPoints:
      logger += "Path Log: %s \n\n" % (path)
      pattern = self.checkPoints[path]
      stopCount = 0
      timeCount = 0

      totalDistance = 0
      totalTime = 0
      numberOfStops = 0

      for point in range(0, len(pattern) - 1 ):
        path = []
        pathSequence = []
        start = pattern[point]
        end = pattern[point + 1]
        logger += "\tRoute: %s -> %s \n\n" % (start,end)
        result, times  = Algorithms().dijkstra(graph,start)
        if result[end] != None:
          while end != start:
            pathSequence.append(end)
            end = result[end]
          pathSequence.append(start)
          for index in range(0, len(pathSequence) - 1):
            start = pathSequence[index]
            end = pathSequence[index + 1]
            startLat = data[start][1]
            startLon = data[start][0]
            endLat = data[end][1]
            endLon = data[end][0]
            fwd_azimuth,back_azimuth,distance = self.generateGraph.geodesic.inv(
                                              startLon,
                                              startLat,
                                              endLon,
                                              endLat)
            totalDistance += distance
            numberOfStops += 1
            timeInFlight = distance * drone.speedMetersPerSecond
            logger += "\t\tSub Route: %s -> %s \n\n" % (start,end)
            logger += "\t\t\tDistance: %d meters \n\n" % (distance)
            logger += "\t\t\tTime In flight: %d seconds \n\n" % (timeInFlight)

      totalRestTime = numberOfStops * drone.chargingTimeInSeconds
      logger += "\tNumber of checkpoints: %s \n" % (numberOfStops)
      logger += "\tTotal Distance: %s meters\n" % (totalDistance)
      logger += "\tTotal Time In Flight: %s seconds\n" % (timeInFlight)
      logger += "\tTotal Rest Time: %s seconds\n" % (totalRestTime)
      logger += "\tTotal Trip Time: %s seconds\n\n" % (timeInFlight +
                                                        totalRestTime)
    print(logger)

if __name__ == "__main__":
  """
  Implementation Notes:

   The project is essentially a simple Dijkstra algorithm with a graphs derived
   from the ability of the drone to reach one point to another to collect is
   edges.

   In short, if the drone is able to reach one lighthouse from another, they are
   both added to the graph as possible connect. We know the max distance of how
   far the drone can go based on the battery life time and the rate at which the
   drone can travel.

   Once the graph is properly constructed, it is run through the Dijkstra
   algorithm.

   In the plot you will see 4 main layers plotted on top of each other for
   readability.

  Graph Description:

    - Orange dots are lighthouses with names.
    - Blue dots are lighthouse without names.
    - Path one (Randomized Color):

          Cities:

              "Stockholm -> Vindö -> Torö"

          lighthouses in city:

               "Bottenholmen -> Branten -> Blockhusudden"

    - Path two (Randomized Color):

          Cities:

              "Stockholm -> Torö" (This skips Vindö completely)

          lighthouses in city:

               "Bottenholmen -> Blockhusudden"

      Data printout is in next prompt. Includes total route distances.
  """

  s = Submission()
  s.draw_UI()