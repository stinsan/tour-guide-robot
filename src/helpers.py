#!/usr/bin/env python
import math
from shapely.geometry import Point

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def gazebo_to_rviz_coords(coord):
    return (coord[0] - 1.54, coord[1] - 7.80)

def find_start(curr_coord, highlights, zones):
    # Find the current zone.
    curr_point = Point(curr_coord[0], curr_coord[1])
    curr_zone =  None
    for z in zones:
        if z.contains(curr_point):
            curr_zone = z

    # Find the closest highlight in our current zone.
    closest_highlight = None
    min_distance = float('inf')

    for highlight in highlights:
        highlight_point = Point(highlight.coord[0], highlight.coord[1])
        d = euclidean_distance(curr_coord, highlight.coord)
        if d < min_distance and curr_zone.contains(highlight_point):
            min_distance = d
            closest_highlight = highlight
    
    return closest_highlight

def find_closest_highlight(tour_map, curr_node):
    closest_highlight = None
    min_distance = float('inf')

    # Find current zone.
    curr_zone =  None
    for z in tour_map.zones:
        if z.contains(Point(curr_node.coord[0], curr_node.coord[1])):
            curr_zone = z

    for highlight in tour_map.highlights:
        highlight_point = Point(highlight.coord[0], highlight.coord[1])

        if curr_zone.contains(highlight_point):
            return highlight
            
        d = euclidean_distance(curr_node.coord, highlight.coord)
        if d < min_distance and tour_map.get_edge(curr_node.id, highlight.id) is not None:
            min_distance = d
            closest_highlight = highlight
    
    return closest_highlight
