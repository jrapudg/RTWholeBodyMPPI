import rospy
import argparse
import numpy as np
from whole_body_mppi.control.controllers.mppi_locomanipulation import MPPI_box_push
from scipy.spatial.transform import Rotation as R

from unitree_legged_msgs.msg import MotorState, MotorCmd
from nav_msgs.msg import Odometry
from whole_body_mppi.utils.tasks import get_task

KP_HIP_GAIN, KD_HIP_GAIN = 55, 3
KP_THIGH_GAIN, KD_THIGH_GAIN = 55, 3
KP_CALF_GAIN, KD_CALF_GAIN = 55, 3
    
class Controller:
    def __init__(self, position_topic, velocity_topic, box_topic):
        self.position_topic = position_topic
        self.velocity_topic = velocity_topic
        self.box_topic = box_topic

        self.joint_command_publishers = {}
        self.joint_states = {j: None for j in [
                "FR_hip", "FR_thigh", "FR_calf",
                "FL_hip", "FL_thigh", "FL_calf",
                "RR_hip", "RR_thigh", "RR_calf",
                "RL_hip", "RL_thigh", "RL_calf"
            ]}

        self.controls = self.joint_states.copy()

        self.Kp_gains = {j: g for j, g in zip(self.joint_states.keys(), 
            [KP_HIP_GAIN]*4 + [KP_THIGH_GAIN]*4 + [KP_CALF_GAIN]*4)}
        self.Kd_gains = {j: g for j, g in zip(self.joint_states.keys(), 
            [KD_HIP_GAIN]*4 + [KD_THIGH_GAIN]*4 + [KD_CALF_GAIN]*4)}
        
        self.body_pos = [0, 0, 0]
        self.body_xy = None
        self.body_z = [0]
        self.body_ori = [1, 0, 0, 0]
        self.body_vel = [0, 0, 0]
        self.body_ang_vel = [0, 0, 0]
        self.joits_ref = None

        self.box_pos = [3, 0, 0]
        self.box_ori = [1, 0, 0, 0]

    def joint_state_callback(self, data, joint_name):
        self.joint_states[joint_name] = data
    
    def odom_callback(self, data):
        self.body_pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        self.body_ori = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]
        self.body_vel = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
        self.body_ang_vel = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]

    def pos_xy_callback(self, data):
        self.body_xy = [data.pose.pose.position.x, data.pose.pose.position.y]
    
    def pos_z_callback(self, data):
        self.body_z = [data.pose.pose.position.z]

    def ori_callback(self, data):
        self.body_ori = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]

    def lin_callback(self, data):
        self.body_vel = [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
    
    def ang_callback(self, data):
        self.body_ang_vel = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]

    def vel_callback(self, data):
        rot = R.from_quat(np.array(self.body_ori)[[1,2,3,0]])
        local_array = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
        self.body_vel = rot.apply(local_array)
        self.body_ang_vel = [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]

    def box_callback(self, data):
        self.box_pos = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        self.box_ori = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]

    def setup_ros(self):
        joints = self.joint_states.keys()
        for joint in joints:
            state_topic = f"/joint_controller_{joint}/state"
            cmd_topic = f"/joint_controller_{joint}/cmd"
            rospy.Subscriber(state_topic, MotorState, self.joint_state_callback, joint)
            self.joint_command_publishers[joint] = rospy.Publisher(cmd_topic, MotorCmd, queue_size=1)

        rospy.Subscriber(self.position_topic, Odometry, self.pos_xy_callback)
        rospy.Subscriber(self.position_topic, Odometry, self.pos_z_callback)
        rospy.Subscriber(self.position_topic, Odometry, self.ori_callback)
        rospy.Subscriber(self.velocity_topic, Odometry, self.vel_callback)
        rospy.Subscriber(self.box_topic, Odometry, self.box_callback)

    def loop(self, task):
        rospy.init_node('controller_quadruped', anonymous=True)
        rate = rospy.Rate(100) 

        self.setup_ros()

        rospy.sleep(3)

        print("Pos: {}".format(self.body_pos))
        print("Ori: {}".format(self.body_ori))
        print("Lin: {}".format(self.body_vel))
        print("Ang: {}".format(self.body_ang_vel))
        print("Box: {}".format(self.box_pos))

        if self.body_xy is None:
            rospy.logwarn("MOCAP not connected")
            rospy.signal_shutdown("No messages received, shutting down node")
            return

        mppi = MPPI_box_push(task)

        mppi.internal_ref = True
        mppi.body_ref[:2] = self.body_xy
        mppi.body_ref[2] = 0.24

        self.body_pos = [self.body_xy[0], self.body_xy[1], self.body_z[0]]
        rospy.loginfo(f"Position {self.body_xy}: Goal = {mppi.body_ref[:3]}")

        mppi.body_ref[:2] = self.body_xy
        mppi.body_ref[2] = 0.24
        state = np.zeros(50)
        print("Goal: {}".format(mppi.x_box_ref))

        while not rospy.is_shutdown():
            self.body_pos = [self.body_xy[0], self.body_xy[1], self.body_z[0]]
            error = np.linalg.norm(np.array(mppi.body_ref[:3]) - np.array(self.body_pos))
            
            box_error = np.linalg.norm(mppi.box_state[:2] - mppi.x_box_ref[:2])
            if error < 0.2 or box_error < 0.4:
                mppi.next_goal()

            state[:3] = self.box_pos
            state[3:7] = self.box_ori
            state[7:10] = self.body_pos
            state[10:14] = self.body_ori
            state[14:17] = [self.joint_states["FR_hip"].q, self.joint_states["FR_thigh"].q, self.joint_states["FR_calf"].q]
            state[17:20] = [self.joint_states["FL_hip"].q, self.joint_states["FL_thigh"].q, self.joint_states["FL_calf"].q]
            state[20:23] = [self.joint_states["RR_hip"].q, self.joint_states["RR_thigh"].q, self.joint_states["RR_calf"].q]
            state[23:26] = [self.joint_states["RL_hip"].q, self.joint_states["RL_thigh"].q, self.joint_states["RL_calf"].q]
            state[26:29] = [0,0,0]
            state[29:32] = [0,0,0]
            state[32:35] = self.body_vel
            state[35:38] = self.body_ang_vel
            state[38:41] = [self.joint_states["FR_hip"].dq, self.joint_states["FR_thigh"].dq, self.joint_states["FR_calf"].dq]
            state[41:44] = [self.joint_states["FL_hip"].dq, self.joint_states["FL_thigh"].dq, self.joint_states["FL_calf"].dq]
            state[44:47] = [self.joint_states["RR_hip"].dq, self.joint_states["RR_thigh"].dq, self.joint_states["RR_calf"].dq]
            state[47:50] = [self.joint_states["RL_hip"].dq, self.joint_states["RL_thigh"].dq, self.joint_states["RL_calf"].dq]

            control_effort = mppi.update(state)

            self.controls["FR_hip"] = control_effort[0]
            self.controls["FR_thigh"] = control_effort[1]
            self.controls["FR_calf"] = control_effort[2]

            self.controls["FL_hip"] = control_effort[3]
            self.controls["FL_thigh"] = control_effort[4]
            self.controls["FL_calf"] = control_effort[5]

            self.controls["RR_hip"] = control_effort[6]
            self.controls["RR_thigh"] = control_effort[7]
            self.controls["RR_calf"] = control_effort[8]

            self.controls["RL_hip"] = control_effort[9]
            self.controls["RL_thigh"] = control_effort[10]
            self.controls["RL_calf"] = control_effort[11]
            
            for joint_name, data in self.joint_states.items():                 
                command_msg = MotorCmd()
                command_msg.mode = 0x0A # Position control mode
                command_msg.q = self.controls[joint_name]
                command_msg.dq = 0 
                command_msg.tau = 0 # Control_effort
                command_msg.Kp = self.Kp_gains[joint_name] # Position gain
                command_msg.Kd = self.Kd_gains[joint_name] # Damping gain
                self.joint_command_publishers[joint_name].publish(command_msg)

            rate.sleep()

if __name__ == "__main__":

    task = "push_box_hw"
    position_topic = '/mocap_node/Go1_body/Odom'
    velocity_topic = '/odom'
    box_topic = "/mocap_node/Box_body/Odom"

    controller = Controller(position_topic=position_topic,
                            velocity_topic=velocity_topic,
                            box_topic=box_topic)
    controller.loop(task)