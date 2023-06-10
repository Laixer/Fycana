from shapely import Point, LineString, box


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
