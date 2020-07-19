from collections import defaultdict
from math import sqrt

class Graph:

    # --- Build the data structure ---
    # Create empty graph, add edge between two vertexes, manage blocked tiles and build the graph itself

    def __init__(self, x, y):
        self.graph = defaultdict(list)
        self.blocked = set()
        self.x = x
        self.y = y

    def __str__(self):
        return str(dict(self.graph))

    def addEdge(self, start, dest):
        self.graph[start].append(dest)

    def addBlocked(self, id):
        self.blocked.add(id)

    def removeBlocked(self, id):
        self.blocked.discard(id)

    def clearBlocked(self):
        self.blocked.clear()

    def buildGraph(self):
        for first in range(self.x * self.y):
            if first in self.blocked:
                continue
            firstCoords = getCoords(first, self.x)
            for second in range(self.x * self.y):
                if first == second or second in self.blocked:
                    continue
                secondCoords = getCoords(second, self.x)
                if abs(firstCoords[0] - secondCoords[0]) <= 1 and abs(firstCoords[1] - secondCoords[1]) <= 1:
                    self.addEdge(first, second)

    # --- Shortest path algorithms ---

    # Universal function for creating the path after the main algorithm
    def result(self, parent, dest):
        route = list([dest])
        previous = parent[dest]
        while previous != -1:
            route.append(previous)
            previous = parent[previous]

        return route

    # Breadth First Search algorithm
    def BFS(self, start, dest):
        numOfCells = self.x * self.y
        visited = [False] * (numOfCells)
        parent = [-1 for i in range(numOfCells)]
        queue = []
        for idx in range(numOfCells):
            if idx in self.blocked:
                visited[idx] = True;

        queue.append(start)
        visited[start] = True

        while queue:
            s = queue.pop(0)
            for i in self.graph[s]:
                if visited[i] == False:
                    visited[i] = True
                    queue.append(i)
                    parent[i] = s
                    if i == dest:
                        break

        return self.result(parent, dest)

    # Dijkstra algorithm
    def minIdx(self, dist, queue) -> int:
        minValue = float('inf')
        minIdx = -1
        for i in range(len(dist)):
            if i in queue and dist[i] < minValue:
                minValue = dist[i]
                minIdx = i
        return minIdx

    def Dijkstra(self, start, dest):
        numOfCells = self.x * self.y
        distance = [float('inf')] * numOfCells
        parent = [-1] * numOfCells

        distance[start] = 0
        queue = []
        for idx in range(numOfCells):
            if idx not in self.blocked:
                queue.append(idx)

        while queue:
            s = self.minIdx(distance, queue)
            if s == dest:
                break
            try:
                queue.remove(s)
            except ValueError:
                return [dest]
            for idx in self.graph[s]:
                if idx in queue and distance[s] + 1 < distance[idx]:
                    distance[idx] = distance[s] + 1
                    parent[idx] = s

        return self.result(parent, dest)

    # A* algorithm
    def distanceTo(self, current, dest):
        currentCoords = getCoords(current, self.x)
        destCoords = getCoords(dest, self.x)
        return sqrt(pow(currentCoords[0] - destCoords[0], 2) + pow(currentCoords[1] - destCoords[1], 2))

    def minIdxAStar(self, open, distance, start, dest):
        minIdx = open[0]
        minValue = self.distanceTo(start, open[0]) + self.distanceTo(open[0], dest)
        for i in open:
            dist_temp = self.distanceTo(start, i) + self.distanceTo(i, dest)
            if dist_temp < minValue:
                minIdx = i
                minValue = dist_temp
        return minIdx

    def AStar(self, start, dest):
        numOfCells = self.x * self.y
        distance = [float('inf')] * numOfCells
        parent = [-1] * numOfCells

        open = [start]
        closed = []
        distance[start] = 0

        while len(open) > 0:
            s = self.minIdxAStar(open, distance, start, dest)
            open.remove(s)
            if s == dest:
                break
            closed.append(s)
            for neighbor in self.graph[s]:
                if neighbor in closed:
                    continue
                gValue = distance[s] + self.distanceTo(s, neighbor)
                fValue = gValue + self.distanceTo(neighbor, dest)
                canAdd = True
                for node in open:
                    if neighbor == node and fValue > distance[s] + self.distanceTo(s, dest):
                        canAdd = False
                if canAdd:
                    open.append(neighbor)
                    distance[neighbor] = gValue
                    parent[neighbor] = s

        return self.result(parent, dest)

    # Bidirectional search


def getCoords(idx, maxX):
    x = idx % maxX
    y = idx // maxX
    return [x, y]

def main():
    start = 0
    end = 8
    graph = Graph(3, 3)
    graph.buildGraph()
    print(graph)
    route = graph.BFS(start, end)
    print(route)
    route = graph.Dijkstra(start, end)
    print(route)
    route = graph.AStar(start, end)
    print(route)

if __name__ == '__main__':
    main()