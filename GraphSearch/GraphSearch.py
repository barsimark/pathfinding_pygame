from collections import defaultdict
from math import sqrt
import pygame, sys, time

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
        self.graph.clear()
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

        return [item for item in reversed(route)]

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
            if s != start:
                drawOneCell(s)

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
            if s != start:
                drawOneCell(s)

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
        for node in self.blocked:
            closed.append(node)

        while len(open) > 0:
            s = self.minIdxAStar(open, distance, start, dest)
            open.remove(s)
            if s == dest:
                break
            closed.append(s)
            for neighbor in self.graph[s]:
                if neighbor in closed:
                    continue
                gValue = distance[s] + 1
                fValue = gValue + self.distanceTo(s, dest)
                canAdd = True
                for node in open:
                    if neighbor == node and fValue > distance[s] + self.distanceTo(s, dest):
                        canAdd = False
                if canAdd:
                    open.append(neighbor)
                    distance[neighbor] = gValue
                    parent[neighbor] = s
            if s != start:
                drawOneCell(s)

        return self.result(parent, dest)

    # Bidirectional BFS
    def intersectingNode(self, first, second):
        for idx in range(self.x * self.y):
            if idx not in self.blocked and first[idx] and second[idx]:
                return idx
        return -1

    def BiDirBFS(self, start, dest):
        if start == dest:
            return [dest]
        if start in self.graph[dest] or dest in self.graph[start]:
            return [start, dest]
        numOfCells = self.x * self.y
        sVisited = [False] * numOfCells
        dVisited = [False] * numOfCells
        sParent = [-1] * numOfCells
        dParent = [-1] * numOfCells
        sQueue = [start]
        dQueue = [dest]

        for idx in range(numOfCells):
            if idx in self.blocked:
                sVisited[idx] = True
                dVisited[idx] = True

        while sQueue and dQueue:
            sCurrent = sQueue.pop(0)
            dCurrent = dQueue.pop(0)
            for neighbor in self.graph[sCurrent]:
                if not sVisited[neighbor]:
                    sParent[neighbor] = sCurrent
                    sVisited[neighbor] = True
                    sQueue.append(neighbor)
                    if neighbor != start and neighbor != dest:
                        drawOneCell(neighbor)
            for neighbor in self.graph[dCurrent]:
                if not dVisited[neighbor]:
                    dParent[neighbor] = dCurrent
                    dVisited[neighbor] = True
                    dQueue.append(neighbor)
                    if neighbor != start and neighbor != dest:
                        drawOneCell(neighbor)

            intersect = self.intersectingNode(sVisited, dVisited)
            if intersect != -1:
                route = [intersect]
                i = intersect
                while i != start:
                    route.append(sParent[i])
                    i = sParent[i]
                route = [item for item in reversed(route)]
                i = intersect
                while i != dest:
                    route.append(dParent[i])
                    i = dParent[i]
                return route

        return [dest]


def getCoords(idx, maxX):
    x = idx % maxX
    y = idx // maxX
    return [x, y]


CELLSIZE = 30
XCELLS = 30
YCELLS = 20

XSIZE = CELLSIZE * XCELLS
YSIZE = CELLSIZE * YCELLS

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 40, 0)
GREEN = (0, 128, 0)
YELLOW = (235, 186, 26)
LIGHTBLUE = (115, 194, 251)

def drawOneCell(cell):
    pos = getLeftTop(cell)
    pygame.draw.rect(BOARD, LIGHTBLUE, [pos[0], pos[1], CELLSIZE, CELLSIZE])
    drawGrid()
    time.sleep(0.01)

def main():
    start = 0
    end = XCELLS * YCELLS - 1
    pygame.init()
    global BOARD
    BOARD = pygame.display.set_mode((XSIZE, YSIZE))
    pygame.display.set_caption("Pathfinding Visualizer")
    drawBoard(start, end)
    drawGrid()
    graph = Graph(XCELLS, YCELLS)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.display.quit()
                pygame.quit()
                try:
                    sys.exit()
                except SystemExit:
                    exit(0)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                cellidx = getCellIdx(pygame.mouse.get_pos())
                if cellidx != start and cellidx != end:
                    if cellidx in graph.blocked:
                        graph.removeBlocked(cellidx)
                    else:
                        graph.addBlocked(cellidx)
                    drawBoard(start, end)
                    drawGraph(graph)
                    drawGrid()
            elif event.type == pygame.KEYDOWN:
                draw = True
                graph.buildGraph()
                if event.key == pygame.K_1:
                    route = graph.BFS(start, end)
                elif event.key == pygame.K_2:
                    route = graph.Dijkstra(start, end)
                elif event.key == pygame.K_3:
                    route = graph.AStar(start, end)
                elif event.key == pygame.K_4:
                    route = graph.BiDirBFS(start, end)
                elif event.key == pygame.K_DELETE:
                    graph.clearBlocked()
                    drawBoard(start, end)
                    drawGraph(graph)
                    drawGrid()
                    draw = False
                elif event.key == pygame.K_n:
                    drawBoard(start, end)
                    drawGraph(graph)
                    drawGrid()
                    draw = False
                else:
                    draw = False
                if draw:
                    drawBoard(start, end, False)
                    for idx in route:
                        if idx != start and idx != end:
                            pos = getLeftTop(idx)
                            pygame.draw.rect(BOARD, YELLOW, [pos[0], pos[1], CELLSIZE, CELLSIZE])
                    drawGraph(graph)
                    drawGrid()
                    
def getLeftTop(idx):
    cellCoords = getCoords(idx, XCELLS)
    return [cellCoords[0] * CELLSIZE, cellCoords[1] * CELLSIZE]

def getCellIdx(coords):
    x = coords[0] // CELLSIZE
    y = coords[1] // CELLSIZE
    return y * XCELLS + x

def drawGrid():
    for x in range(0, XSIZE, CELLSIZE):
        pygame.draw.line(BOARD, BLACK, (x, 0), (x, YSIZE))
    for y in range(0, YSIZE, CELLSIZE):
        pygame.draw.line(BOARD, BLACK, (0, y), (XSIZE, y))
    pygame.display.update()

def drawBoard(start, end, fill = True):
    if fill:
        BOARD.fill(WHITE)
    startPos = getLeftTop(start)
    endPos = getLeftTop(end)
    pygame.draw.rect(BOARD, RED, [startPos[0], startPos[1], CELLSIZE, CELLSIZE])
    pygame.draw.rect(BOARD, GREEN, [endPos[0], endPos[1], CELLSIZE, CELLSIZE])
    pygame.display.update()

def drawGraph(graph):
    for blocked in graph.blocked:
        pos = getLeftTop(blocked)
        pygame.draw.rect(BOARD, GRAY, [pos[0], pos[1], CELLSIZE, CELLSIZE])
    pygame.display.update()

if __name__ == '__main__':
    main()