from collections import defaultdict

class Graph:
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

    def BFS(self, start, dest):
        visited = [False] * (self.x * self.y)
        table = [-1 for i in range(self.x * self.y)]
        queue = []
        for idx in range(self.x * self.y):
            if idx in self.blocked:
                visited[idx] = True;

        queue.append(start)
        visited[start] = True

        previous = start
        while queue:
            s = queue.pop(0)
            for i in self.graph[s]:
                if visited[i] == False:
                    visited[i] = True
                    queue.append(i)
                    table[i] = s
                    if i == dest:
                        previous = s
                        break
        route = list()
        route.append(dest)
        while previous != -1:
            route.append(previous)
            previous = table[previous]

        return route

def getCoords(idx, maxX):
    x = idx % maxX
    y = idx // maxX
    return [x, y]

def main():
    graph = Graph(3, 3)
    graph.addBlocked(4)
    graph.buildGraph()
    print(graph)
    route = graph.BFS(0, 8)
    print(route)

if __name__ == '__main__':
    main()