import sys
import os
# Add the local folder (the directory containing the 'control' package) to the Python path
sys.path.append(os.path.join('/home/ControlUser/legged_ctrl_ws/src/legged_mppi/scripts'))

import rospy
import mujoco
import numpy as np
import mujoco_viewer
from unitree_legged_msgs.msg import MotorState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R

from control.mppi_gait import MPPI_box_push



import rosbag
import os
import argparse

from threading import Thread
from time import sleep
from datetime import datetime
from PIL import Image

def apply_translation_to_com(com_world, quaternion, translation_local):
    """
    Apply a translation in the robot's local frame to the CoM in the world frame.
    
    Parameters:
    - com_world: 3D array, [x, y, z] coordinates of the CoM in the world frame
    - quaternion: 4D array, [q_w, q_x, q_y, q_z] quaternion representing the robot's orientation in the world
    - translation_local: 3D array, [x, y, z] translation in the robot's local frame (e.g., [0.3, 0, 0] for 30 cm along x-axis)
    
    Returns:
    - new_com_world: 3D array, new [x, y, z] coordinates of the CoM in the world frame
    """
    # Convert quaternion to rotation matrix
    rotation = R.from_quat(quaternion[[1, 2, 3, 0]])
    
    # Transform the local translation to the world frame
    translation_world = rotation.apply(translation_local)
    
    # Apply the translation in the world frame to the CoM
    new_com_world = np.array(com_world) + translation_world
    
    return new_com_world

class OwnJointState:
    def __init__(self, q=0.0, dq=0.0):
        self.q = q  # Position or angle
        self.dq = dq  # Velocity or rate of change of position

    def __repr__(self):
        return f"OwnJointState(q={self.q}, dq={self.dq})"
    
class QposVisualizerMuJoCo:
    def __init__(self, model_path="src/legged_mppi/scripts/models/go1/go1_scene_mppi_pyr_push_box_14in.xml", replay=False,
                 save_frames=False, frame_rate=50, save_dir='src/frames'):
        
        # Initialize the ROS node
        rospy.init_node('qpos_visualizer_mujoco', anonymous=True) 

        # Load the MuJoCo model and create the simulation
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.agent = MPPI_box_push(task='push_box_14in')

        # Initialize the viewer
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, hide_menus=True)

        self.joints = ["RL_hip", "RR_hip", "FL_hip", "FR_hip", 
                        "RL_thigh", "RR_thigh", "FL_thigh", "FR_thigh", 
                        "RL_calf", "RR_calf", "FL_calf", "FR_calf"]
        
        # Flag to enable/disable saving frames
        self.save_frames = save_frames
        self.frame_rate = frame_rate  # Frame capture rate in Hz (frames per second)
        self.save_dir = save_dir      # Directory to save the frames

        # Ensure the directory exists
        if self.save_frames and not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Start a separate thread to save frames if the flag is set

        # Visualizer logic
        self.joint_command_publishers = {}
        self.joint_states = {}
        dummy_state = OwnJointState(q=0, dq=0)
        self.joint_states = {"RL_hip":dummy_state, "RR_hip":dummy_state, "FL_hip":dummy_state, "FR_hip":dummy_state, 
                             "RL_thigh":dummy_state, "RR_thigh":dummy_state, "FL_thigh":dummy_state, "FR_thigh":dummy_state, 
                             "RL_calf":dummy_state, "RR_calf":dummy_state, "FL_calf":dummy_state, "FR_calf":dummy_state}
        
        for joint in self.joints:
            state_topic = f"/joint_controller_{joint}/state"
            rospy.Subscriber(state_topic, MotorState, self.joint_state_callback, joint)

        # Flag to check if rosbag replay mode is enabled
        self.replay = replay

        if self.replay:
            rospy.Subscriber("/unitree_hardware/joint_foot", JointState, self.unitree_hardware_callback)
            # Subscribe to real-time topics only if not replaying from rosbag
            for joint in self.joints:
                state_topic = f"/joint_controller_{joint}/state"
                rospy.Subscriber(state_topic, MotorState, self.joint_state_callback, joint)
        

        # Subscribers for mocap data
        rospy.Subscriber("/mocap_node/Box_body/Odom", Odometry, self.mocap_box_callback)
        rospy.Subscriber("/mocap_node/Go1_body/Odom", Odometry, self.mocap_robot_callback)

        # Initialize qpos storage (assuming a robot with 6 degrees of freedom for example)
        self.state = np.zeros(50)  # nq is the number of generalized coordinates (joints)
        # # Create a subscriber to the /robot/qpos topic (assuming qpos is published here as Float32MultiArray)
        # rospy.Subscriber("/mocap_node/Box_body/Odom", Odometry, self.mocap_box_callback)
        # rospy.Subscriber("/mocap_node/Go1_body/Odom", Odometry, self.mocap_robot_callback)

        # Set the desired update rate (e.g., 10 Hz)
        self.rate = rospy.Rate(100)  # 10 Hz update rate
        rospy.sleep(2)

    def replay_from_bag(self, bag_file):
        # Open the rosbag and replay the data
        rospy.loginfo(f"Replaying rosbag: {bag_file}")
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages():
            if topic == "/mocap_node/Box_body/Odom":
                self.mocap_box_callback(msg)
            elif topic == "/mocap_node/Go1_body/Odom":
                self.mocap_robot_callback(msg)
            elif topic == "/unitree_hardware/joint_foot" in topic:
                self.unitree_hardware_callback(msg)
            # Add more conditions if needed for additional topics
        bag.close()

    def capture_frame(self):
        # Capture the current frame from the viewer
        width, height = self.viewer.viewport.width, self.viewer.viewport.height
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Render the frame and store it in the array
        mujoco.mjr_readPixels(frame, None, self.viewer.viewport, self.viewer.ctx)

        # Save the frame as a PNG image
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        filename = os.path.join(self.save_dir, f"frame_{timestamp}.png")
        image = Image.fromarray(np.flipud(frame))
        image.save(filename)
        rospy.loginfo(f"Saved frame: {filename}")

    def mocap_box_callback(self, data):
        # Store the latest state data
        # print("odom_callback")
        #self.state[:2] = [data.pose.pose.position.x, data.pose.pose.position.y]
        self.state[:3] = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z/2]
        self.state[3:7] = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]

    def mocap_robot_callback(self, data):
        # Store the latest state data
        # print("odom_callback")
        self.state[7:10] = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z] + np.array([0, -0.05, 0])
        self.state[10:14] = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]

    def joint_state_callback(self, data, joint_name):
        # Store the latest state data
        self.joint_states[joint_name] = data
    
    def unitree_hardware_callback(self, data):
        self.joint_states["FL_hip"] = OwnJointState(q=data.position[0], dq=data.velocity[0])
        self.joint_states["FL_thigh"] = OwnJointState(q=data.position[1], dq=data.velocity[1])
        self.joint_states["FL_calf"] = OwnJointState(q=data.position[2], dq=data.velocity[2])
        self.joint_states["FR_hip"] = OwnJointState(q=data.position[3], dq=data.velocity[3])
        self.joint_states["FR_thigh"] = OwnJointState(q=data.position[4], dq=data.velocity[4])
        self.joint_states["FR_calf"] = OwnJointState(q=data.position[5], dq=data.velocity[5])
        self.joint_states["RL_hip"] = OwnJointState(q=data.position[6], dq=data.velocity[6])
        self.joint_states["RL_thigh"] = OwnJointState(q=data.position[7], dq=data.velocity[7])
        self.joint_states["RL_calf"] = OwnJointState(q=data.position[8], dq=data.velocity[8])
        self.joint_states["RR_hip"] = OwnJointState(q=data.position[9], dq=data.velocity[9])
        self.joint_states["RR_thigh"] = OwnJointState(q=data.position[10], dq=data.velocity[10])
        self.joint_states["RR_calf"] = OwnJointState(q=data.position[11], dq=data.velocity[11])

    def set_camera_parameters(self, azimuth=90, elevation=-45, distance=2.0, lookat=[0,0,0]):
        cam = self.viewer.cam
        cam.azimuth = azimuth
        cam.elevation = elevation
        cam.distance = distance
        cam.lookat[:] = lookat
        
    def print_camera_parameters(self):
        cam = self.viewer.cam

        # Print the current camera parameters
        print(f"Camera Azimuth: {cam.azimuth}")
        print(f"Camera Elevation: {cam.elevation}")
        print(f"Camera Distance: {cam.distance}")
        print(f"Camera Lookat: {cam.lookat}")

    def run(self):
        """
        Run the MuJoCo viewer and update it based on the incoming qpos data from ROS or rosbag
        """
        # if self.rosbag_file:
        #     self.replay_from_bag(self.rosbag_file)
        self.agent.body_ref[2] = 0.24

        print("Right goal: {}".format(self.agent.x_box_ref))
        
        while not rospy.is_shutdown():
            error = np.linalg.norm(np.array(self.agent.body_ref[:3]) - np.array(self.state[7:10]))
            box_error = np.linalg.norm(self.agent.box_state[:2] - self.agent.x_box_ref[:2])
            if error < 0.2 or box_error < 0.4:
                # print("move on to the next goal")
                self.agent.next_goal()
            self.state[14:17] = [self.joint_states["FR_hip"].q, self.joint_states["FR_thigh"].q, self.joint_states["FR_calf"].q]
            self.state[17:20] = [self.joint_states["FL_hip"].q, self.joint_states["FL_thigh"].q, self.joint_states["FL_calf"].q]
            self.state[20:23] = [self.joint_states["RR_hip"].q, self.joint_states["RR_thigh"].q, self.joint_states["RR_calf"].q]
            self.state[23:26] = [self.joint_states["RL_hip"].q, self.joint_states["RL_thigh"].q, self.joint_states["RL_calf"].q]
            self.state[26:29] = [0,0,0]
            self.state[29:32] = [0,0,0]
            self.state[32:35] = [0,0,0]
            self.state[35:38] = [0,0,0]
            self.state[38:41] = [self.joint_states["FR_hip"].dq, self.joint_states["FR_thigh"].dq, self.joint_states["FR_calf"].dq]
            self.state[41:44] = [self.joint_states["FL_hip"].dq, self.joint_states["FL_thigh"].dq, self.joint_states["FL_calf"].dq]
            self.state[44:47] = [self.joint_states["RR_hip"].dq, self.joint_states["RR_thigh"].dq, self.joint_states["RR_calf"].dq]
            self.state[47:50] = [self.joint_states["RL_hip"].dq, self.joint_states["RL_thigh"].dq, self.joint_states["RL_calf"].dq]

            self.data.qpos[:] = self.state[:26] 
            self.data.qvel[:] = self.state[26:50] 

            # Step the MuJoCo simulation forward
            mujoco.mj_step(self.model, self.data)

            self.viewer.add_marker(
                    pos=[-1.0, -0.7, 0.15],         # Position of the marker
                    size=[0.15, 0.15, 0.15],     # Size of the sphere
                    rgba=[1, 1, 0, 1],           # Color of the sphere (red)
                    type=mujoco.mjtGeom.mjGEOM_SPHERE, # Specify that this is a sphere
                    label=""
                )
            
            #self.state[:3] = self.state[:3][[1,0,2]]
            self.agent.update(self.state)

            # self.viewer.add_marker(
            #     pos=self.agent.body_ref[:3]*1,         # Position of the marker
            #     size=[0.15, 0.15, 0.15],     # Size of the sphere
            #     rgba=[1, 1, 0, 1],           # Color of the sphere (red)
            #     type=mujoco.mjtGeom.mjGEOM_SPHERE, # Specify that this is a sphere
            #     label=""
            # )
            
            
            #orientation = self.data.qpos[10:14]
            orientation = self.data.qpos[4:8]
            #print(f"Box mocap: {self.state[:3]}")
            #print(f"Box simulator: {self.agent.state_rollouts[0,0,:3]}")
            translation_local = [0, 0, 0.3]
            #translation_local = [0, 0, 0]
            for n in range(self.agent.n_samples):
                for i in range(self.agent.horizon):
                    #com = self.agent.state_rollouts[n,i,8:11]
                    com = self.agent.state_rollouts[n,i,1:4]
                    new_com_world = apply_translation_to_com(com, orientation, translation_local)
                    
                    self.viewer.add_marker(
                    pos=new_com_world,      # Position of the marker
                    size=[0.001, 0.001, 0.001],     # Size of the sphere
                    rgba=[0.5, 0.5, 0.5, 1],           # Color of the sphere (red)
                    type=mujoco.mjtGeom.mjGEOM_SPHERE, # Specify that this is a sphere
                    label=""
                )

            # Box
            mean_traj = self.agent.exp_weights.reshape(30, 1, 1)*self.agent.state_rollouts[:,:,1:4]/(np.sum(self.agent.exp_weights) + 1e-10)
            # Robot

            #mean_traj = self.agent.exp_weights.reshape(30, 1, 1)*self.agent.state_rollouts[:,:,8:11]/(np.sum(self.agent.exp_weights) + 1e-10)
            mean_traj = mean_traj.sum(axis=0)
            for i in range(self.agent.n_samples):
                com = mean_traj[i]
                new_com_world = apply_translation_to_com(com, orientation, translation_local)
                self.viewer.add_marker(
                pos=new_com_world,      # Position of the marker
                size=[0.005, 0.005, 0.005],     # Size of the sphere
                rgba=[1, 0, 1, 1],           # Color of the sphere (red)
                type=mujoco.mjtGeom.mjGEOM_SPHERE, # Specify that this is a sphere
                label=""
            )
                
            orientation = self.data.qpos[10:14]
            #print(f"Box mocap: {self.state[:3]}")
            #print(f"Box simulator: {self.agent.state_rollouts[0,0,:3]}")
            translation_local = [0, 0, 0.3]
            #translation_local = [0, 0, 0]
            for n in range(self.agent.n_samples):
                for i in range(self.agent.horizon):
                    com = self.agent.state_rollouts[n,i,8:11]
                    #com = self.agent.state_rollouts[n,i,1:4]
                    new_com_world = apply_translation_to_com(com, orientation, translation_local)
                    
                    self.viewer.add_marker(
                    pos=new_com_world,      # Position of the marker
                    size=[0.001, 0.001, 0.001],     # Size of the sphere
                    rgba=[0.5, 0.5, 0.5, 1],           # Color of the sphere (red)
                    type=mujoco.mjtGeom.mjGEOM_SPHERE, # Specify that this is a sphere
                    label=""
                )

            # Box
            #mean_traj = self.agent.exp_weights.reshape(30, 1, 1)*self.agent.state_rollouts[:,:,1:4]/(np.sum(self.agent.exp_weights) + 1e-10)
            # Robot

            mean_traj = self.agent.exp_weights.reshape(30, 1, 1)*self.agent.state_rollouts[:,:,8:11]/(np.sum(self.agent.exp_weights) + 1e-10)
            mean_traj = mean_traj.sum(axis=0)
            for i in range(self.agent.n_samples):
                com = mean_traj[i]
                new_com_world = apply_translation_to_com(com, orientation, translation_local)
                self.viewer.add_marker(
                pos=new_com_world,      # Position of the marker
                size=[0.005, 0.005, 0.005],     # Size of the sphere
                rgba=[1, 0, 1, 1],           # Color of the sphere (red)
                type=mujoco.mjtGeom.mjGEOM_SPHERE, # Specify that this is a sphere
                label=""
            )
            
            # Render the current state
            self.viewer.render()
            
            if self.save_frames:
                self.capture_frame()

            # Sleep to maintain the loop rate
            self.rate.sleep()

        # Close the viewer when the node is shut down
        self.viewer.close()

if __name__ == '__main__':
    try:
        # Parse command line arguments to determine whether to replay from rosbag or use live data
        parser = argparse.ArgumentParser(description="Qpos Visualizer MuJoCo with rosbag or live topic support")
        parser.add_argument('--replay', type=bool, help='Replay from rosbag file flag', default=False)
        parser.add_argument('--save_frames', type=bool, help='Replay from rosbag file flag', default=False)

        args = parser.parse_args()

        # Create and run the visualizer
        visualizer = QposVisualizerMuJoCo(replay=args.replay, save_frames=args.save_frames, frame_rate=50)
        visualizer.set_camera_parameters(azimuth=-90, elevation=-30, distance=2.5, lookat=[-2.03, -0.13, 0.34])
        visualizer.print_camera_parameters()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass