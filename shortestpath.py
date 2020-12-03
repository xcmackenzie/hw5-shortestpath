# Implementation of Bellman Ford and Dijkstra's Algorithm for HW5

from pqdict import PQDict

# Converts results of BellmanFord/Dijkstra into actual shortest path
# rather than just minimum distances
def shortestPath(graph, source, end, alg):
    # Initialize path and current node
    node = end
    path = [node]
    # Run whichever algorithm specified: 1 = BellmanFord, 2 = Dijkstra
    try:
        if alg == 1:
            dis, prev = BellmanFord(graph, source)
        if alg == 2:
            dis, prev = Dijkstra(graph, source)
    except AssertionError:
        print("Algorithm failed.")
    # Working from end node to source
    while node != source:
        # Check previous visited node and add to path
        node = prev[node]
        path.append(node)
    # Reverse direction of path
    path.reverse()
    return path

def BellmanFord(graph, source):
    # Initialize distance to node, previous node for each node in graph
    distance = {}
    prevNode = {}
    for node in graph:
        distance[node] = float('Inf')
        prevNode[node] = None
    # Set distance from source to itself = 0
    distance[source] = 0

    # Relax each edge
    for i in range(len(graph) - 1):
        for node in graph:
            for neighbor in graph[node]:
                # If the distance between current node and neighbor is lower than current
                # minimum distance, update distance
                if distance[neighbor] > distance[node] + graph[node][neighbor]:
                    distance[neighbor] = distance[node] + graph[node][neighbor]
                    prevNode[neighbor] = node

    # Check if there are any negative weighted cycles in the graph.
    # If there are, algorithm will not work, so send error
    for node in graph:
        for neighbor in graph[node]:
            assert distance[neighbor] <= distance[node] + graph[node][neighbor]

    return distance, prevNode

def BFTest():
    # Graph with positive and negative edges
    # For source a, distances should be: a: 0, b: 3, c: 5, d: 3, e: -1
    g1 = {
    'a': {'b': 3, 'c': 5},
    'b': {'d': 1},
    'c': {'d': -2},
    'd': {'e': -4},
    'e': {}
    }
    # Graph with positive edges only
    # For source a, distances should be: a: 0, b: 5, c: 7, d: 4
    g2 = {
    'a': {'b': 5, 'd': 4},
    'b': {'c': 4},
    'c': {},
    'd': {'c': 3}
    }
    # Graph with negative cycle
    # Should throw an error
    g3 = {
    'a': {'b': 2},
    'b': {'c': -4, 'd': 5},
    'c': {'a': 1},
    'd': {}
    }

    graphs = [g1, g2, g3]

    for graph in graphs:
        try:
            print(BellmanFord(graph, 'a'))
        except AssertionError:
            print("Negative Cycle")
    # Print shortest path in g1 from a to e
    # Should read a, c, d, e
    print(shortestPath(g1, 'a', 'e', 1))

def Dijkstra(graph, source):
    # Initialize distance to node, queue for tracking minimum shortest path,
    # previous node visited, list of nodes that have not yet been visited
    distance = {source: 0}
    queue = PQDict(distance)
    prevNode = {}
    unexplored = set(graph.keys())

    # While there are still nodes that haven't been explored
    while unexplored:
        # Pop node with minimum distance that hasn't yet been explored
        (node, dis) = queue.popitem()
        # Update distance
        distance[node] = dis
        # Remove newly explored node from unexplored
        unexplored.remove(node)

        # For each of exploring node's neighbors
        for neighbor in graph[node]:
            if neighbor in unexplored:
                # Find new distance if neighbor hasn't yet been explored
                dis = distance[node] + graph[node][neighbor]
                # If newly discovered path shorter than currently stored path
                # Update queue and previous node
                if dis < queue.get(neighbor, float('Inf')):
                    queue[neighbor] = dis
                    prevNode[neighbor] = node

    # Check if there are any negative weight edges in the graph
    # If there are, algorithm will not work, so send error
    for node in graph:
        for neighbor in graph[node]:
            assert graph[node][neighbor] > 0

    return distance, prevNode



def DijkstraTest():
    # Directed graph
    # For source a, distances should be: a: 0, b: 3, c: 5, d: 4, e: 8
    g1 = {
    'a': {'b': 3, 'c': 5},
    'b': {'d': 1},
    'c': {'d': 2},
    'd': {'e': 4},
    'e': {}
    }
    # Undirected graph
    # For source a, distances should be: a: 0,
    g2 = {
    'a': {'b': 3, 'c': 5},
    'b': {'a': 3, 'd': 5},
    'c': {'a': 5, 'd': 2},
    'd': {'c': 5, 'e': 4},
    'e': {'d': 4}
    }
    # Graph with negative edge
    # Should throw an error
    g3 = {
    'a': {'b': -3, 'c': 5},
    'b': {'a': -3, 'd': 5},
    'c': {'a': 5, 'd': 2},
    'd': {'c': 5, 'e': 4},
    'e': {'d': 4}
    }

    graphs = [g1, g2, g3]
    for graph in graphs:
        try:
            print(Dijkstra(graph, 'a'))
        except AssertionError:
            print("Negative Edge")
    # Print shortest path in g1 from a to e
    # Should read a, b, d, e
    print(shortestPath(g1, 'a', 'e', 2))

BFTest()
DijkstraTest()
