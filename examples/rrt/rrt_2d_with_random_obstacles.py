# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import uuid

import numpy as np

from src.rrt.rrt import RRT
from src.search_space.search_space import SearchSpace
from src.utilities.obstacle_generation import generate_random_obstacles
from src.utilities.plotting import Plot

X_dimensions = np.array([(0, 32), (0, 24)])  # dimensions of Search Space
x_init = (0, 0)  # starting location
x_goal = (32, 24)  # goal location

Q = np.array([(8, 4)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 500  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create search space
X = SearchSpace(X_dimensions)
n = 20
obstacles = generate_random_obstacles(X_dimensions, x_init, x_goal, n)
# create rrt_search
for obstacle in obstacles:
    X.obs.add(uuid.uuid4(), tuple(obstacle), tuple(obstacle))
# rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)

cnt = 0
# while x_init != x_goal:
cnt += 1
rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
res, path = rrt.rrt_search()
# x_init = path[1]

# plot
plot = Plot("rrt_2d_with_random_obstacles")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=False)

print("yeah!!!")
print("cnt = ", cnt)
