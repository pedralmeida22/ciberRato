from heapq import *
from itertools import permutations
from math import dist
from sys import maxsize


def heuristic(a, b):
    return dist(a, b)


def astar(start, goal, array, walls):
    neighbors = [(0, 2), (0, -2), (2, 0), (-2, 0)]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            # data.append(start)
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)

            if neighbor not in array:
                continue

            # Make sure walkable terrain
            if i == 2 and ((current[0] + 1, current[1]) in walls):
                continue
            if i == -2 and ((current[0] - 1, current[1]) in walls):
                continue
            if j == 2 and ((current[0], current[1] + 1) in walls):
                continue
            if j == -2 and ((current[0], current[1] - 1) in walls):
                continue

            if neighbor in close_set and tentative_g_score > gscore.get(neighbor, 0):
                continue

            if tentative_g_score <= gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return None


def smallPath(graph, start):
    points = []
    print("START->", start)

    for p in graph.keys():
        if p != start:
            points.append(p)

    path = []
    min_path = maxsize
    permutacoes = permutations(points)

    for i in permutacoes:
        cost = 0

        k = start
        temp = []
        temp.append(k)

        for j in i:
            # print("Debug " + str(k) + "-> " + str(graph[k]))
            p = [x for x in graph[k] if x[0] == j]
            cost += p[0][1]
            k = j
            temp.append(p[0][0])
        p = [x for x in graph[k] if x[0] == start]
        cost += p[0][1]

        if min_path > cost:
            min_path = cost
            path = temp
    return path
