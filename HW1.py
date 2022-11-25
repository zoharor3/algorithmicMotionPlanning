import argparse
import os
from typing import List, Tuple
from numpy.linalg import norm

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString

from collections import defaultdict
import heapq as heap
from itertools import combinations


# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """
    tmpObstaclePoints = [0] * (len(original_shape.exterior.coords)-1)*4
    for cSpacetmpIndex in range(len(original_shape.exterior.coords)-1):
        tmpObstaclePoints[cSpacetmpIndex*4] =tuple(map(lambda i, j: i + j, original_shape.exterior.coords[cSpacetmpIndex] , (0, r)))
        tmpObstaclePoints[cSpacetmpIndex*4 + 1] = tuple(map(lambda i, j: i + j, original_shape.exterior.coords[cSpacetmpIndex] , (r, 0)))
        tmpObstaclePoints[cSpacetmpIndex*4 + 2] = tuple(map(lambda i, j: i + j, original_shape.exterior.coords[cSpacetmpIndex] , (0, -r)))
        tmpObstaclePoints[cSpacetmpIndex*4 + 3] = tuple(map(lambda i, j: i + j, original_shape.exterior.coords[cSpacetmpIndex] , (-r, 0)))

    tmpObstaclePolygon = Polygon(tmpObstaclePoints)
    # print(tmpObstaclePoints)
    # print(tmpObstaclePolygon.convex_hull)

    return tmpObstaclePolygon.convex_hull

def check_crossed_polygon(obstacles: List[Polygon], cords1,cords2):
    line = LineString([cords1, cords2])
    for obstacle in obstacles:
        # if obstacle.contains(line):
        #     return True
        # elif obstacle.covers(line):
        #     return False
        if obstacle.intersects(line):
            if not obstacle.touches(line):
                return True
    return False



# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    line_string_list = []
    edge_list = []
    for obstacle in obstacles:
        for cords in obstacle.exterior.coords:
            edge_list.append(cords)

    for cords1, cords2 in combinations(edge_list, 2):
        if check_crossed_polygon(obstacles,cords1,cords2) is False:
            line_string_list.append(LineString([cords1,cords2]))
    return line_string_list

    # for object1, object2 in combinations(obstacles, 2):
    #    for cords1 in object1.exterior.coords:
    #        for cords2 in object2.exterior.coords:
    #            if check_intersection(obstacles,cords1,cords2) is False:
    #                line_string_list.append(LineString([cords1,cords2]))
    # return line_string_list


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist

# def find_shortest_path(lines: List[LineString], startingNode: tuple) -> tuple[LineString, float]:
#     """
#     find shortest path for given graph
#     :param lines: A list of the graph edges
#     :return: A LineStrings holding the edges of the shortest path and cost of that path
#     """
#     distances = []
#     for line in lines:
#         distances.append(norm(line))
#
#     visited = set()
#     parentsMap = {}
#     PriorityQueue = []
#     nodeCosts = defaultdict(lambda: float('inf'))
#     nodeCosts[startingNode] = 0
#     heap.heappush(PriorityQueue, (0, startingNode))
#
#     while PriorityQueue:
#         # go greedily by always extending the shorter cost nodes first
#         _, node = heap.heappop(PriorityQueue)
#         visited.add(node)
#         print(PriorityQueue)
#         for adjNode, weight in lines[node].items():
#             if adjNode in visited:    continue
#
#             newCost = nodeCosts[node] + weight
#             if nodeCosts[adjNode] > newCost:
#                 parentsMap[adjNode] = node
#                 nodeCosts[adjNode] = newCost
#                 heap.heappush(PriorityQueue, (newCost, adjNode))
#
#     return parentsMap, nodeCosts



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()




    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    # lines = get_visibility_graph(c_space_obstacles, source, dest)
    lines = {"a" : (0.0, 0.5), "b" : (0.5, 0.0),"c" : (0.0, -0.5), "d":(-0.5, 0.0), "e":(1.0, 0.5),"f": (1.5, 0.0),"g": (1.0, -0.5), "h":(0.5, 1.5)} # only until visibility graph will be ready

    #TODO: fill in the next line
    shortest_path, cost = find_shortest_path(lines, (0.5,0.0))

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    # plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))


    plotter3.show_graph()