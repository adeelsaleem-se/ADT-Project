import random


def generate_geometric_graph(n, r):
    # Define a set of vertices V such that |V| = n
    V = set(range(1, n + 1))

    # Initialize an empty set of edges E
    E = set()

    # Assign random (x, y) coordinates to each vertex
    coordinates = {vertex: (random.uniform(0, 1), random.uniform(0, 1)) for vertex in V}

    # Add edges (u, v) and (v, u) for each pair of vertices within distance r
    for u in V:
        for v in V:
            if u != v:
                # Calculate Euclidean distance between vertices u and v
                distance = ((coordinates[u][0] - coordinates[v][0]) ** 2 + (
                            coordinates[u][1] - coordinates[v][1]) ** 2)
                # If distance is less than or equal to r, add edge (u, v) and (v, u)
                if distance <= r ** 2:
                    E.add((min(u, v), max(u, v)))

    return E, coordinates

def store_graph_to_file(edges, coordinates, filename):
    """
    Store the graph in an EDGES file augmented with position information.
    """
    with open(filename, 'w') as f:
        for edge in edges:
            u, v = edge
            x1, y1 = coordinates[u]
            x2, y2 = coordinates[v]
            f.write(f"{u} {x1} {y1} {v} {x2} {y2}\n")

def dfs(graph, visited, vertex, component):
    """
    Depth-first search (DFS) traversal starting from a given vertex,
    storing the vertices of the connected component.
    """
    # Mark the current vertex as visited
    visited.add(vertex)

    # Add the current vertex to the connected component
    component.append(vertex)

    # Traverse all neighbors of the current vertex
    for neighbor in graph[vertex]:
        if neighbor not in visited:
            # Recursively visit the neighbor
            dfs(graph, visited, neighbor, component)

def largest_connected_component(graph):
    """
    Find the largest connected component in the graph using DFS.
    """
    visited = set()
    largest_component = []
    for vertex in graph:
        if vertex not in visited:
            component = []
            dfs(graph, visited, vertex, component)
            if len(component) > len(largest_component):
                largest_component = component

    # Create a new graph containing only edges within the largest connected component
    lcc_graph = {}
    for vertex in largest_component:
        lcc_graph[vertex] = [v for v in graph[vertex] if v in largest_component]

    return lcc_graph

# Binary search to find the maximum distance r for the given constraints
def binary_search_r(n, min_ratio, max_ratio):
    """
    Perform binary search to find the maximum distance r for the given constraints.
    """
    low = 0.0
    high = 2 ** 0.5 # Assuming coordinates range from 0 to 10
    while low <= high:
        mid = (low + high) / 2
        vlcc_min = int(min_ratio * n)
        vlcc_max = int(max_ratio * n)
        E, _ = generate_geometric_graph(n, mid)
        graph = edges_to_adjacency_list(E)
        lcc = largest_connected_component(graph)
        lcc_size = len(lcc)
        if vlcc_min <= lcc_size <= vlcc_max:
            return mid
        elif lcc_size < vlcc_min:
            high = mid - 0.00001
        else:
            low = mid + 0.00001
    return None  # No suitable r found

def edges_to_adjacency_list(edges):
    """
    Convert a graph represented as edges into an adjacency list.
    """
    adjacency_list = {}
    for u, v in edges:
        adjacency_list.setdefault(u, []).append(v)
        adjacency_list.setdefault(v, []).append(u)  # Assuming undirected graph
    return adjacency_list



graph = {
    1: [2],
    2: [1],
    3: [7, 4],
    4: [3, 5],
    5: [4],
    6: [],
    7: [3]
}

# Call the largest_connected_component function
lcc = largest_connected_component(graph)
print("Largest connected component:", lcc)


# Generate graphs and find suitable r for each set of characteristics
n_values = [300, 400, 500]
min_ratios = [0.9, 0.8, 0.7]
max_ratios = [0.95, 0.9, 0.8]
for n, min_ratio, max_ratio in zip(n_values, min_ratios, max_ratios):
    r = binary_search_r(n, min_ratio, max_ratio)
    if r is not None:
        E, coordinates = generate_geometric_graph(n, r)
        store_graph_to_file(E, coordinates, f"graph_n{n}_r{r}.txt")
        print(f"Graph with n={n} and r={r} stored successfully.")
    else:
        print(f"No suitable r found for n={n}.")