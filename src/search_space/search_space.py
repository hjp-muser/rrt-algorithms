# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import numpy as np
from rtree import index

from src.utilities.geometry import es_points_along_line
from src.utilities.obstacle_generation import obstacle_generator


class SearchSpace(object):
    def __init__(self, dimension_ranges, obstacles=None):
        """
        Initialize Search Space
        :param dimension_ranges: range of each dimension
        :param obstacles: list of obstacle
        """
        self.dimensions = len(dimension_ranges)  # number of dimensions
        self.dimension_ranges = dimension_ranges  # range of each dimension
        p = index.Property()
        p.dimension = self.dimensions
        if obstacles is None:
            self.obs = index.Index(interleaved=True, properties=p)
        else:
            # r-tree representation of obstacles
            self.obs = index.Index(obstacle_generator(obstacles), interleaved=True, properties=p)

    def obstacle_free(self, x):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
        return self.obs.count(x) == 0

    def sample_free(self):
        """
        Sample a location within X_free
        :return: random location within X_free
        """
        while True:  # sample until not inside of an obstacle
            x = self.sample()
            if self.obstacle_free(x):
                return x

    def collision_free(self, start, end, r):
        """
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :param r: resolution of points to sample along edge when checking for collisions
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        points = es_points_along_line(start, end, r)
        coll_free = all(map(self.obstacle_free, points))
        return coll_free

    def sample(self):
        """
        Return a random location within X
        :return: random location within X (not necessarily X_free)
        """
        x = np.random.randint(self.dimension_ranges[:, 0], self.dimension_ranges[:, 1])
        return tuple(x)
