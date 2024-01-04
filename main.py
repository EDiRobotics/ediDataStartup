from edi_gym import EdiEnv
import numpy as np
import cv2
import rospy
from std_srvs.srv import Trigger
import time
import json
import os
import sys
from ctypes import cdll

records_bag_full_path = []

current_dir = os.path.dirname(os.path.abspath(__file__))
path = os.path.join(current_dir, "frrpc.so")

if not os.path.exists(path):
    print("Please put 'frrpc.so' into the file directory.")
    raise FileNotFoundError

# Add the directory containing frrpc.so to sys.path
sys.path.append(current_dir)
# Load the frrpc library
frrpc = cdll.LoadLibrary(path)
# Now you can import frrpc and use it
import frrpc

robot = frrpc.RPC('192.168.1.10')


def send_start_request():
    success = False

    rospy.wait_for_service('/record/ctrl/start_record_srv')
    try:
        # Check if '/env/info/instruct' is set and not empty
        if not rospy.has_param('/env/info/instruct') or rospy.get_param('/env/info/instruct') == "":
            rospy.logwarn("Warning: Parameter '/env/info/instruct' is not set or is empty.")

        start_record = rospy.ServiceProxy('/record/ctrl/start_record_srv', Trigger)
        response = start_record()
        if response.success:
            instruct = rospy.get_param('/env/info/instruct', "")
            rospy.loginfo(f"Recording started successfully, current instruction is \"{instruct}\".")
            success = True

        else:
            rospy.loginfo("Unable to start recording: " + response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: " + str(e))
    return success


def send_end_request():
    global last_end
    success = False
    rospy.wait_for_service('/record/ctrl/end_record_srv')
    try:
        end_record = rospy.ServiceProxy('/record/ctrl/end_record_srv', Trigger)
        response = end_record()
        if response.success:
            bag_full_path = response.message
            records_bag_full_path.append(bag_full_path)
            rospy.loginfo(f"Recording stopped successfully, save to {bag_full_path}.")
            success = True
        else:
            rospy.loginfo("Unable to stop recording: " + response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: " + str(e))
    last_end = time.time()
    return success


def reverse_kinematics(cart):
    # Placeholder function for reverse kinematics
    cart = [float(i) for i in cart]
    joint = robot.GetInverseKin(0, [float(i) for i in cart], -1)[1:]
    r_cart = robot.GetForwardKin(joint)[1:]
    # print(f"cart: {cart}, r_cart: {r_cart}")
    return joint


def forward_kinematics(joint):
    # Placeholder function for reverse kinematics
    cart = robot.GetForwardKin(joint)[1:]
    return cart


def interpolate_joints(start, end, steps):
    # Linear interpolation between start and end positions
    interpolated = []
    for s, e in zip(start, end):
        interpolated.append(np.linspace(s, e, steps))
    return np.array(interpolated).T  # Transpose to get the correct shape


def run_one_episode(goal_cart, vel=3):
    obs = env.reset()
    current_joint = obs["status"]["jt_cur_pos"][0]
    current_cart = forward_kinematics(current_joint)
    print(f"vel {vel}, goal_cart {goal_cart}", )
    goal_joint = reverse_kinematics(goal_cart)

    distance = np.linalg.norm(np.array(current_cart[:3]) - np.array(goal_cart[:3]))
    num_steps = max(int(distance / vel), 1)

    d = {"goal_joint": goal_joint, "goal_cart": goal_cart, "vel": vel,
         "current_joint": current_joint, "current_cart": current_cart}
    param_value = json.dumps(d)
    rospy.set_param('/env/info/instruct', param_value)

    actions = interpolate_joints(current_joint, goal_joint, num_steps)
    if not send_start_request():
        rospy.logerr("Can not start recording, return!")
        return
        # print(actions)
    for action in actions:
        action_with_gripper = action.tolist() + [0]
        obs, _, _, info = env.step(action_with_gripper)
        for camera_name, image in obs["images"].items():
            cv2.imshow(camera_name, image)
        cv2.waitKey(1)  # Display the window until a key is pressed
    while not rospy.is_shutdown():
        if send_end_request():
            break
        time.sleep(1)


# Initialize the environment
env = EdiEnv()

# Define the bounds of the 3D workspace for x, y, z (x_min, x_max, y_min, y_max, z_min, z_max)
workspace_bounds = [300, 800, -300, 60, 150, 350]
rpy_bounds = [120, 240, -30, 30, -30, 30]
rpy_fixed = [180, 0, 0]
vel_limits = [2, 5]
max_episode = 10000
first_end = send_end_request()
if first_end:
    rospy.logwarn("It was recording just now and it has been ended...")

for _ in range(max_episode):
    goal_xyz = [np.random.uniform(low, high) for low, high in zip(workspace_bounds[::2], workspace_bounds[1::2])]
    # goal_rpy = [np.random.uniform(low, high) for low, high in zip(rpy_bounds[::2], rpy_bounds[1::2])]
    goal_cart = goal_xyz + rpy_fixed

    vel = np.random.uniform(vel_limits[0], vel_limits[1])

    run_one_episode(goal_cart, vel)
