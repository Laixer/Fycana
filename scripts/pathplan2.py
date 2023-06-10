import sys
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import networkx as nx

from shapely import Point, LineString, box

np.set_printoptions(formatter={"float": lambda x: "{0:0.2f}".format(x)})


class PathFinding:
    def __init__(self, bounds, obstacles):
        self.bounds = bounds
        self.obstacles = obstacles

    @staticmethod
    def effective_vector(start, target):
        return LineString([start, target])

    def obstacles_maxy(self):
        global_maxy = float("-inf")

        for obstacle in self.obstacles:
            (_, _, _, maxy) = obstacle.bounds
            if maxy > global_maxy:
                global_maxy = maxy

        return global_maxy


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
        print("obstacles_maxy ", obstacles_maxy)

        effective_vector = PathFinding.effective_vector(root_point, target_point)
        print("effective_vector ", effective_vector)

        last_node = root_point

        obsint = effective_vector.intersection(self.obstacles)

        for intersection in obsint:
            print(" intersection ", intersection)
            for inter_point in intersection.coords:
                print("  inter_point ", inter_point)

                elevated_point = Point(inter_point[0], obstacles_maxy + 0.5)
                print("  elevated_point ", elevated_point)

                graph.add_node(elevated_point)
                graph.add_edge(last_node, elevated_point)
                last_node = elevated_point

        graph.add_edge(last_node, target_point)

        self._strongly_connect(graph)

        # return nx.shortest_path(graph, source=root_point, target=target_point)
        return (nx.shortest_path(graph, source=root_point, target=target_point), graph)


start = np.array([1.5, 3])
bounds = np.array([[0, 15], [0, 15]])
target = np.array([12, 1.5])


obstacles = [
    box(4.0, 0.0, 6.0, 2.0),
    box(5.0, 2.0, 5.5, 5.0),
    box(8.0, 0.0, 10.5, 3.5),
    box(12.5, 0.0, 15.0, 5.0),
    box(6.0, 0.0, 9.0, 3.0),
]


# sys.exit()

rrt = InterBufferPath(bounds, obstacles)

path, graph = rrt.run(start, target)

matplotlib.use("TkAgg")


fig, ax = plt.subplots()

plt.axis([0, 15, 0, 15])
plt.suptitle("start: " + str(start) + ", target: " + str(target))


for obstacle in obstacles:
    import matplotlib.patches as patches

    bounds = obstacle.bounds

    rect = patches.Rectangle(
        (bounds[0], bounds[1]),
        bounds[2] - bounds[0],
        bounds[3] - bounds[1],
    )
    ax.add_patch(rect)


plt.plot(
    [start[0], target[0]],
    [start[1], target[1]],
    "-",
    color="purple",
)


# plt.pause(10)

for node in graph.nodes:
    plt.plot(node.x, node.y, "ro")


for edge in graph.edges:
    plt.plot(
        [edge[0].x, edge[1].x],
        [edge[0].y, edge[1].y],
        "-",
        color="black",
    )

    # plt.pause(0.1)

print("")


print("Path length ", len(path) - 1)

parent = None

for x in path:
    if parent is not None:
        plt.plot(
            [x.x, parent.x],
            [x.y, parent.y],
            "-",
            color="blue",
        )
    parent = x


plt.show()
