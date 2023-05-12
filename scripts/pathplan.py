import sys
import matplotlib.pyplot as plt
import matplotlib
import numpy as np


np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})


class RRTPath:
    def __init__(self, stepsize):
        self.stepsize = stepsize
        self.waypoints = []


class TreeNode:
    def __init__(self, point, parent, children=[]):
        self.point = point
        self.parent = parent
        self.children = children

    def add_child_point(self, point):
        child = TreeNode(point, self, [])
        self.children.append(child)
        return child

    def __str__(self):
        s = f"TreeNode(point={self.point}, parent={self.parent.point if self.parent else None})"
        for child in self.children:
            s += f"\n- {child}"
        return s


class RRT:
    def __init__(self, start, bounds, target, stepsize):
        self.start = start
        self.bounds = bounds
        self.target = TreeNode(target, None, children=[])
        self.stepsize = stepsize
        self.root = TreeNode(start, None)

    def sample_point(self):
        rng = np.random.default_rng()

        x = rng.integers(self.bounds[0][0], self.bounds[0][1])
        y = rng.integers(self.bounds[1][0], self.bounds[1][1])
        r = np.array([x, y])
        return r

    def calc_new_point(self, base_point, point):
        u_hat = (point - base_point) / RRT.distance(base_point, point)
        print("Unit vector ", u_hat)

        new_point = base_point + (u_hat * self.stepsize)

        # TODO: Check if new_point is colliding with obstacles

        return new_point

    def add_node(self, point):
        print("")

        print("Selected point ", point)

        (nearest_node, dist) = self.find_nearest_node(self.root, point)
        print("Distance from final ", dist, nearest_node.point)

        if dist > 0.1:
            n_point = self.calc_new_point(nearest_node.point, point)
            print("New point ", n_point)

            return nearest_node.add_child_point(n_point)

    @staticmethod
    def find_nearest_node(node, point, distance=None):
        dist = RRT.distance(node.point, point)

        if distance is None:
            distance = dist
        else:
            distance = min(distance, dist)

        for child in node.children:
            (child_node, child_dist) = RRT.find_nearest_node(child, point, distance)
            if child_dist < distance:
                return (child_node, child_dist)

        return (node, distance)

    @staticmethod
    def distance(p1, p2) -> float:
        return np.linalg.norm(p1 - p2)

    def reached_target(self, node):
        return RRT.distance(node.point, self.target.point) < self.stepsize


start = np.array([1, 1])
bounds = np.array([[0, 15], [0, 15]])
target = np.array([7, 14])
stepsize = 1

rrt = RRT(start, bounds, target, stepsize)

# di = rrt.distance(np.array([1, 1]), np.array([2, 0]))

# print("Distance ", di)

# sys.exit(0)

matplotlib.use('TkAgg')

last_node = None

while True:
    point = rrt.sample_point()
    node = rrt.add_node(point)

    if node is not None:
        print("reached_target ", rrt.reached_target(node))
        print("distance to target ", rrt.distance(node.point, rrt.target.point))

        if rrt.reached_target(node):
            rrt.target.parent = node
            node.children.append(rrt.target)
            last_node = rrt.target
            break

plt.axis([0, 15, 0, 15])
plt.suptitle('start: ' + str(start) + ', target: ' + str(target) + ', stepsize: ' + str(stepsize))

def add_to_plot(node):
    plt.plot(node.point[0], node.point[1], 'ro')
    for child in node.children:
        plt.plot([node.point[0], child.point[0]], [node.point[1], child.point[1]], '-', color='black')

        plt.pause(0.1)

        add_to_plot(child)


add_to_plot(rrt.root)

print("")

print(rrt.root)

print("")


path = RRTPath(stepsize)

while last_node:
    path.waypoints.insert(0, last_node.point)
    if last_node.parent is not None:
        plt.plot([last_node.point[0], last_node.parent.point[0]], [last_node.point[1], last_node.parent.point[1]], '-', color='blue')
    last_node = last_node.parent

print("Path points ", len(path.waypoints))
print("Path length ", len(path.waypoints) * path.stepsize)
print("Path", path.waypoints)

plt.show()
