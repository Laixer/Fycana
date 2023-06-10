import networkx as nx

from pyglonax.pathplan import PathFinding

from shapely import Point, LineString


class InterBufferPath(PathFinding):
    def __init__(self, bounds, obstacles):
        super().__init__(bounds, obstacles)

    def _strongly_connect(self, graph):
        for node_a in graph.nodes:
            for node_b in graph.nodes:
                if node_a == node_b:
                    continue

                if graph.has_edge(node_a, node_b):
                    continue

                effective_vector = LineString([node_a, node_b])

                obscross = effective_vector.crosses(self.obstacles)
                if not any(obscross):
                    graph.add_edge(node_a, node_b)

    def run(self, start, target):
        root_point = Point(start)
        target_point = Point(target)
        graph = nx.Graph()

        graph.add_node(root_point)
        graph.add_node(target_point)

        obstacles_maxy = self.obstacles_maxy()

        effective_vector = PathFinding.effective_vector(root_point, target_point)

        last_node = root_point

        obsint = effective_vector.intersection(self.obstacles)

        for intersection in obsint:
            for inter_point in intersection.coords:
                elevated_point = Point(inter_point[0], obstacles_maxy + 0.5)

                graph.add_node(elevated_point)
                graph.add_edge(last_node, elevated_point)
                last_node = elevated_point

        graph.add_edge(last_node, target_point)

        self._strongly_connect(graph)

        return nx.shortest_path(graph, source=root_point, target=target_point)
