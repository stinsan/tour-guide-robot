#!/usr/bin/env python
from shapely.geometry.polygon import Polygon

#  A tour hightlight to be visited by the robot
class TourHighlight:
    def __init__(self, id, coord, blurb):
        """
        :param coord: a tuple (x, y) of the coordinate location of a
            tour highlight on the RViz map (maps/map.pgm).
        :param blurb: a string containing what the robot should say
            upon arriving at this tour highlight.
        """
        self.id = id
        self.coord = coord
        self.blurb = blurb

        
#  An edge connecting the nodes
class Edge:
    def __init__(self, start, destination, gates=None):
        self.start = start
        self.destination = destination
        self.gates = gates

        
#  A gate node representing the entrance/exit of doors
class Gate:
    def __init__(self, id, coord):
        self.id = id
        self.coord = coord

#  Tour Map consting of nodes (highlights and gates) and connecting edges
class TourMap:
    def __init__(self):
        # 1. DEH east entrance: (3.6, -8.3)
        # 2. Atrium center: (-3.6, -12.5)
        # 3. Atrium SW: (-5.0, -15.8)
        # 4. End Hallway: (-3.25, -2.6)
        # 5. Devon 115 near door: (-4.2, -7.8)
        # 6. Devon 115 west: (-8.8, -9.3)

        self.highlights = [
            TourHighlight(id=0, coord=(3.6, -8.3), blurb='This is the entryway to Devon Energy Hall!'),
            TourHighlight(id=1, coord=(-5.0, -15.8), blurb='These tables are where CS/ECE students study!'),
            TourHighlight(id=2, coord=(-3.6, -12.5), blurb='This is the atrium!'),
            TourHighlight(id=3, coord=(-3.3, -2.6), blurb='This hallway leads to various lecture halls and offices!'),
            TourHighlight(id=4, coord=(-4.2, -7.8), blurb='This room is Devon 115 and is where CS students go to work on projects!'),
            TourHighlight(id=5, coord=(-8.8, -9.3), blurb='Computer science students are able to use these computers for their homeworks and projects.'),
        ]

        GATE0 = Gate(0, (3.0, -8.5)) # East DEH Entrance (Entering DEH).
        GATE1 = Gate(1, (2.4, -9.1)) # East DEH Entrance (Exiting DEH).
        GATE2 = Gate(2, (-1.8, -6.9)) # Doorway to DEH 115 (Entering DEH 115).
        GATE3 = Gate(3, (-2.9, -7.6)) # Doorway to DEH 115 (Exiting DEH 115).
        self.edges = {
            0: [Edge(0, 1, gates=[GATE0]), Edge(0, 2, gates=[GATE0]), Edge(0, 3, gates=[GATE0]), Edge(0, 4, gates=[GATE0, GATE2]), Edge(0, 5, gates=[GATE0, GATE2])],
            1: [Edge(1, 0, gates=[GATE1]), Edge(1, 2), Edge(1, 3), Edge(1, 4, gates=[GATE2]), Edge(1, 5, gates=[GATE2])],
            2: [Edge(2, 0, gates=[GATE1]), Edge(2, 1), Edge(2, 3), Edge(2, 4, gates=[GATE2]), Edge(2, 5, gates=[GATE2])],
            3: [Edge(3, 0, gates=[GATE1]), Edge(3, 1), Edge(3, 2), Edge(3, 4, gates=[GATE2]), Edge(3, 5, gates=[GATE2])],
            4: [Edge(4, 0, gates=[GATE3, GATE1]), Edge(4, 1, gates=[GATE3]), Edge(4, 2, gates=[GATE3]), Edge(4, 3, gates=[GATE3]), Edge(4, 5)],
            5: [Edge(5, 0, gates=[GATE3, GATE1]), Edge(5, 1, gates=[GATE3]), Edge(5, 2, gates=[GATE3]), Edge(5, 3, gates=[GATE3]), Edge(5 , 4)]
        }

        ZONE1 = Polygon([(3.6, -9.9), (2.0, -7.8), (3.5, -6.7), (5.1, -8.7)]) # Entrance.
        ZONE2 = Polygon([(-8.8, -13.3), (-4.1, -19.3), (2.9, -14.1), (1.3, -11.7), (3.6, -9.9), (0.6, -5.9)]) # Atrium
        ZONE3 = Polygon([(-1.7, -7.8), (0.6, -5.9), (-2.8, -1.2), (-5.2, -2.8)]) # Hallway
        ZONE4 = Polygon([(-1.7, -7.8), (-5.1, -2.9), (-12.3, -8.4), (-8.8, -13.2)]) # DEH 115.
        self.zones = [ZONE1, ZONE2, ZONE3, ZONE4]
    
    def get_edge(self, start, destination):
        edge_list = self.edges[start]

        for e in edge_list:
            if e.destination == destination:
                return e

        return None

    def remove_edge(self, start, destination):
        edge_list = self.edges[start]

        for e in edge_list:
            if e.destination == destination:
                self.edges[start].remove(e)

    #  Pruning edges and nodes when a door is not opened
    def prune(self, gate_id):
        """ Removes the edges that are blocked by the given gate_id,
        also removes blocked tour highlights.
        """
        edges_to_remove = list()
        for index in self.edges:
            for edge in self.edges[index]:
                if edge.gates is None: 
                    continue

                for gate in edge.gates:
                    if gate.id == gate_id:
                        edges_to_remove.append(edge)
        
        for edge in edges_to_remove:
            self.remove_edge(edge.start, edge.destination)
            self.remove_highlight(edge.destination)

    def get_highlight(self, id):
        for h in self.highlights:
            if h.id == id:
                return h
        
        return None
        
    def remove_highlight(self, id):
        h = self.get_highlight(id)

        if h is not None:
            self.highlights.remove(h)
        
