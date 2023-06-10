import sys
import time
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import networkx as nx
from shapely import Polygon, Point, LineString, box

np.set_printoptions(formatter={"float": lambda x: "{0:0.2f}".format(x)})


class RRT:
    def __init__(self, start, bounds, target, stepsize, obstacles):
        self.bounds = bounds

        self.target = Point(target)
        self.stepsize = stepsize
        self.root = Point(start)
        self.graph = nx.Graph()
        self.obstacles = obstacles

        self.graph.add_node(self.root)

    def sample_point(self):
        rng = np.random.default_rng()

        x = rng.integers(self.bounds[0][0], self.bounds[0][1])
        y = rng.integers(self.bounds[1][0], self.bounds[1][1])
        r = np.array([x, y])
        return r

    def calc_new_point(self, base_point, point):
        base_point_n = np.asarray(base_point.coords)
        point_n = np.asarray(point.coords)

        u_hat = (point_n - base_point_n) / base_point.distance(point)

        new_point = base_point_n + (u_hat * self.stepsize)
        return Point(new_point)

    def is_within_obstacle(self, point):
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return True
        return False

    def add_node(self, point):
        print("")

        new_point = Point(point)
        print("Selected point ", new_point)

        (nearest_node, dist) = self.find_nearest_node(new_point)

        print("Nearest node ", nearest_node)

        new_node = self.calc_new_point(nearest_node, new_point)

        ls = LineString([nearest_node, new_node])
        print("Line string ", ls)

        for obstacle in self.obstacles:
            ls = LineString([nearest_node, new_node])

            if ls.crosses(obstacle):
                print("Crossing obstacle")
                return

        if self.is_within_obstacle(new_node):
            print(f"Point {new_node.point} is within obstacle")
            return

        lowest_cost_node = nearest_node
        lowest_cost = float("inf")

        rrt.graph.add_node(new_node)
        self.graph.add_edge(lowest_cost_node, new_node)

        return new_node

    def find_nearest_node(self, point):
        nearest_node = None
        distance = float("inf")

        for node in self.graph.nodes:
            node_dist = node.distance(point)
            if node_dist < distance:
                distance = node_dist
                nearest_node = node

        return (nearest_node, distance)

    def reached_target(self, node):
        return node.distance(self.target) < self.stepsize


class InterBufferPath:
    def __init__(self, start, bounds, target, stepsize, obstacles):
        self.bounds = bounds

        self.target = Point(target)
        self.stepsize = stepsize
        self.root = Point(start)
        self.graph = nx.Graph()
        self.obstacles = obstacles

        self.graph.add_node(self.root)


start = np.array([1.5, 3])
bounds = np.array([[0, 15], [0, 15]])
target = np.array([12, 1.5])
stepsize = 1

obstacles = [
    # box(4.0, 0.0, 6.0, 2.0),
    # box(5.0, 2.0, 5.5, 5.0),
    # box(6.0, 0.0, 9.0, 1.5),
    # box(8.0, 0.0, 10.5, 3.5),
    # box(12.5, 0.0, 15.0, 5.0),
    box(6.0, 0.0, 9.0, 3.0),
]


# sys.exit()

rrt = RRT(start, bounds, target, stepsize, obstacles)

matplotlib.use("TkAgg")

last_node = None
last_node_idx = None

has_reached_target = False

for i in range(1500):
    point = rrt.sample_point()

    node = rrt.add_node(point)

    if node is not None:
        print("reached_target ", rrt.reached_target(node))
        print("distance to target ", node.distance(rrt.target))

        if rrt.reached_target(node) and not has_reached_target:
            last_node = rrt.target

            rrt.graph.add_node(last_node)
            rrt.graph.add_edge(node, last_node)

            has_reached_target = True
            break

fig, ax = plt.subplots()

plt.axis([0, 15, 0, 15])
plt.suptitle(
    "start: " + str(start) + ", target: " + str(target) + ", stepsize: " + str(stepsize)
)


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
    [rrt.root.x, rrt.target.x],
    [rrt.root.y, rrt.target.y],
    "-",
    color="purple",
)


# plt.pause(10)

for node in rrt.graph.nodes:
    plt.plot(node.x, node.y, "ro")


for edge in rrt.graph.edges:
    plt.plot(
        [edge[0].x, edge[1].x],
        [edge[0].y, edge[1].y],
        "-",
        color="black",
    )

    # plt.pause(0.01)

print("")


path_length = nx.shortest_path_length(rrt.graph, source=rrt.root, target=rrt.target)
print("Path length ", path_length)

parent = None

path = nx.shortest_path(rrt.graph, source=rrt.root, target=rrt.target)
for x in path:
    if parent is not None:
        plt.plot(
            [x.x, parent.x],
            [x.y, parent.y],
            "-",
            color="blue",
        )
    parent = x

    print(x)


tup = list()
for x in path:
    tup.append((x.x, x.y))

lines = LineString(tup)
print(lines)

s = lines.simplify(1.0, preserve_topology=False)
print(s)

parent = None

for x in s.coords:
    if parent is not None:
        plt.plot(
            [x[0], parent[0]],
            [x[1], parent[1]],
            "-",
            color="green",
        )
    parent = x


start_point = Point(start)
target_point = Point(target)

straight_line = LineString([start_point, target_point])

print("straight_line ", straight_line)

for obstacle in obstacles:
    obsint = straight_line.intersection(obstacles)

    (minx, miny, maxx, maxy) = obstacle.bounds

    print("obsint ", obsint)

    for intersection in obsint:
        print("intersection ", intersection)
        for inter_point in intersection.coords:
            print("inter_point ", inter_point)

            elevated_point = Point(inter_point[0], maxy + 0.5)
            print("elevated_point ", elevated_point)

    # new_point = Point(0.0, 0.5)
    # print("new_point ", new_point)

# nx.draw(rrt.graph, with_labels=True, node_color="lightblue", font_weight="bold")


plt.show()
