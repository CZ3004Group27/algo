"""Given a FULL graph:
    + nodes: represent the starting pose and the poses to view the images
        The starting pose is represented as node 0 by default.
    + edges: represent the distance the car needs to move between the nodes
Compute a list of sequences of nodes for a Hamiltonian path
"""

import networkx as nx

from enum import Enum
class GraphPathPlannerType(Enum):
    GREEDY = "greedy"
    SHORTEST = "shortest"

def get_graph_path_planner(planner_type: GraphPathPlannerType):
    if planner_type is GraphPathPlannerType.GREEDY:
        return GreedyHamiltonianPathPlanner
    elif planner_type is GraphPathPlannerType.SHORTEST:
        pass

class GreedyHamiltonianPathPlanner:
    def __init__(self, graph: nx.Graph, starting_node: int = 0):
        self.graph = graph
        self.starting_node = starting_node
    def find_path(self):
        unvisited = set(self.graph.nodes)

        # the start node is already visited
        current_node = self.starting_node
        unvisited.remove(current_node)
        path = [current_node]
        path_length = 0

        while unvisited:
            next_node = min(unvisited, key=
                lambda node: self.graph[current_node][node]["weight"])

            path_length += self.graph[current_node][next_node]["weight"]
            current_node = next_node
            unvisited.remove(current_node)
            path.append(current_node)

        return path, path_length

# Unittest the algorithms on some small inputs
if __name__ == "__main__":
    G = nx.Graph()
    G.add_edges_from([
        (0, 1, {"weight": 7}),
        (0, 2, {"weight": 2}),
        (0, 3, {"weight": 10}),
        (0, 4, {"weight": 1}),
        (1, 2, {"weight": 8}),
        (1, 3, {"weight": 4}),
        (1, 4, {"weight": 5}),
        (2, 3, {"weight": 11}),
        (2, 4, {"weight": 3}),
        (3, 4, {"weight": 6}),
    ])

    greedy_planner = GreedyHamiltonianPathPlanner(G)
    path, path_length = greedy_planner.find_path()
    assert path == [0, 4, 2, 1, 3]
    assert path_length == 16
