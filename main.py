from edi_gym import EdiEnv
import numpy as np
import cv2


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
    current_joint = obs["status"]["jt_cur_pos"][0]

    goal_joint = reverse_kinematics(goal_cart)

    num_steps = 100
    actions = interpolate_joints(current_joint, goal_joint, num_steps)

    print(actions)
    for action in actions:
        action_with_gripper = action + [0]
        obs, _, _, info = env.step(action_with_gripper)
        for camera_name, image in obs["images"].items():
            cv2.imshow(camera_name, image)
        cv2.waitKey(1)  # Display the window until a key is pressed


# Initialize the environment
env = EdiEnv()

# Define the bounds of the 3D workspace for x, y, z (x_min, x_max, y_min, y_max, z_min, z_max)
workspace_bounds = [500, 700, 80, 100, 150, 200]
rpy_bounds = [120, 240, -30, 30, -30, 30]

constant_rpy = [0, 0, 0]

max_episode = 100
for _ in range(max_episode):
    goal_xyz = [np.random.uniform(low, high) for low, high in zip(workspace_bounds[::2], workspace_bounds[1::2])]
    goal_rpy = [np.random.uniform(low, high) for low, high in zip(rpy_bounds[::2], rpy_bounds[1::2])]
    goal_cart = goal_xyz + constant_rpy
    run_one_episode(goal_cart)
