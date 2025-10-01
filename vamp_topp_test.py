import vamp
import numpy as np


pos1 = [0.9941923543408971, -0.051116248331033905, 0.55644523090925, -2.204390838326673, 0.016857619708280643, 2.293574043590873, 0.702844622050357]
pos2 = [-1.4847849572216933, 0.013402851810183476, -0.16617785697236934, -2.1464311136674072, 0.017834601468841655, 2.245508108743958, 0.7687104453444691]
for i in range(14):
    pos1.append(0)
    pos2.append(0)
# set up planner
(
    vamp_module,
    planner_func,
    plan_settings,
    simp_settings,
) = vamp.configure_robot_and_planner_with_kwargs("pandatopp", "rrtctopp")

rng = vamp_module.halton()

plan_settings.max_iterations = 10000
plan_settings.max_samples = 10000
plan_settings.range = 1.25

cuboids_data = [
        # table
        #[[0.0, 0.0, -0.08], [0.0, 0.0, 0.0], [1.5, 1.5, 0.1]],
        # roof
        [[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [1.5, 1.5, 0.1]],
        # right wall
        # [[-0.4, 0.0, 0.91], [0.0, 0.0, 0.0], [0.01, 1.0, 1.0]],
        # # left wall
        # [[1.0, 0.0, 0.91], [0.0, 0.0, 0.0], [0.01, 1.0, 1.0]],
        # # front wall
        # [[0.2, 0.8, 0.91], [0.0, 0.0, 0.0], [1.0, 0.01, 1.0]],
        # back wall
        [[0.2, -0.7, 0.91], [0.0, 0.0, 0.0], [1.0, 0.01, 1.0]],
    ]
env = vamp.Environment()
cuboids = [vamp.Cuboid(*data) for data in cuboids_data]
for cuboid in cuboids:
    env.add_cuboid(cuboid)

result = planner_func(np.array(pos1), np.array(pos2), env, plan_settings, rng)
simple = vamp_module.simplify(result.path, env, simp_settings, rng)
print(simple.path.numpy())