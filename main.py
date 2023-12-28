from edi_gym import EdiEnv
import numpy as np


def reverse_kinematics(cart):
    # Placeholder function for reverse kinematics
    joint = [0, 0, 0, 0, 0, 0]
    return joint


def interpolate_joints(start, end, steps):
    # Linear interpolation between start and end positions
    interpolated = []
    for s, e in zip(start, end):
        interpolated.append(np.linspace(s, e, steps))
    return np.array(interpolated).T  # Transpose to get the correct shape


def run_one_episode(goal_cart):
    obs = env.reset()
    current_joint = obs["status"]["current_pos"]

    goal_joint = reverse_kinematics(goal_cart)

    num_steps = 100
    actions = interpolate_joints(current_joint, goal_joint, num_steps)

    for action in actions:
        obs, _, _, info = env.step(action)


# Initialize the environment
env = EdiEnv()

# Define the bounds of the 3D workspace for x, y, z (x_min, x_max, y_min, y_max, z_min, z_max)
workspace_bounds = [0, 200, 0, 200, 0, 200]

constant_rpy = [0, 0, 0]

max_episode = 10
for _ in range(max_episode):
    goal_xyz = [np.random.uniform(low, high) for low, high in zip(workspace_bounds[::2], workspace_bounds[1::2])]
    goal_cart = goal_xyz + constant_rpy
    run_one_episode(goal_cart)
