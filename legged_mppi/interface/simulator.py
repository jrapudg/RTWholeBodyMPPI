import mujoco
import os
import mujoco_viewer
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import tqdm
from PIL import Image

class Simulator:
    """
    A class representing a simulator for controlling and estimating the state of a system.
    
    Attributes:
        filter (object): The filter used for state estimation.
        agent (object): The agent used for control.
        model_path (str): The path to the XML model file.
        T (int): The number of time steps.
        dt (float): The time step size.
        viewer (bool): Flag indicating whether to enable the viewer.
        gravity (bool): Flag indicating whether to enable gravity.
        model (object): The MuJoCo model.
        data (object): The MuJoCo data.
        qpos (ndarray): The position trajectory.
        qvel (ndarray): The velocity trajectory.
        finite_diff_qvel (ndarray): The finite difference of velocity.
        ctrl (ndarray): The control trajectory.
        sensordata (ndarray): The sensor data trajectory.
        noisy_sensordata (ndarray): The noisy sensor data trajectory.
        time (ndarray): The time trajectory.
        state_estimate (ndarray): The estimated state trajectory.
        viewer (object): The MuJoCo viewer.
    """
    def __init__(self, filter=None, agent=None,
                 model_path = os.path.join(os.path.dirname(__file__), "../models/go1/task_simulate.xml"),
                T = 200, dt = 0.01, viewer = True, gravity = True,
                # stiff=False
                timeconst=0.02, dampingratio=1.0, ctrl_rate=100,
                save_dir="./frames", save_frames=False
                ):
        # filter
        self.filter = filter
        self.agent = agent
        self.ctrl_rate = ctrl_rate
        self.update_ratio = max(1, 1/(dt*ctrl_rate))
        self.interpolate_cam = False
        # model
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = dt
        self.model.opt.enableflags = 1 # to override contact settings
        self.model.opt.o_solref = np.array([timeconst, dampingratio])
        # data
        self.data = mujoco.MjData(self.model)
        self.T = T
        # save
        self.save_frames = save_frames
        self.save_dir = save_dir
        # rollout
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos = self.model.key_qpos[1]
        self.data.qvel = self.model.key_qvel[1]
        self.data.ctrl = self.model.key_ctrl[1]

        # turn off gravity
        if not gravity:
            self.model.opt.gravity[:] = 0
            self.filter.model.opt.gravity[:] = 0

        # viewer
        if viewer:
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, hide_menus=True)
        else:
            self.viewer = None

        # trajectories
        self.qpos = np.zeros((self.model.nq, self.T))
        self.qvel = np.zeros((self.model.nv, self.T))
        self.finite_diff_qvel = np.zeros((self.model.nv, self.T-1))
        self.ctrl = np.zeros((self.model.nu, self.T))
        self.sensordata = np.zeros((self.model.nsensordata, self.T))
        self.noisy_sensordata = np.zeros((self.model.nsensordata, self.T))
        self.time = np.zeros(self.T)
        self.cost = np.zeros((1, self.T))

        # Ensure the directory exists
        if self.save_frames and not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

    def get_sensor(self):
        return self.data.sensordata

    def step(self, ctrl=None):
        self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)
        return self.data.qpos, self.data.qvel
    
    def store_trajectory(self, t):
        self.qpos[:, t] = self.data.qpos
        self.qvel[:, t] = self.data.qvel
        self.ctrl[:, t] = self.data.ctrl
        self.sensordata[:, t] = self.data.sensordata
        self.time[t] = self.data.time
        self.cost[0, t] = self.agent.eval_best_trajectory()
        return None
    
    def state_difference(self, pos1, pos2):
        # computes the finite difference between two states
        vel = np.zeros(self.model.nv)
        mujoco.mj_differentiatePos(self.model, vel, self.model.opt.timestep, pos1, pos2)
        return vel
    
    def capture_frame(self):
        # Capture the current frame from the viewer
        width, height = self.viewer.viewport.width, self.viewer.viewport.height
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Render the frame and store it in the array
        mujoco.mjr_readPixels(frame, None, self.viewer.viewport, self.viewer.ctx)

        # Save the frame as a PNG image
        filename = os.path.join(self.save_dir, f"frame_{self.t}.png")
        image = Image.fromarray(np.flipud(frame))
        image.save(filename)

    def capture_frame_traj(self):
        # Capture the current frame from the viewer
        width, height = self.viewer.viewport.width, self.viewer.viewport.height
        frame = np.zeros((height, width, 3), dtype=np.uint8)

        # Render the frame and store it in the array
        mujoco.mjr_readPixels(frame, None, self.viewer.viewport, self.viewer.ctx)

        # Save the frame as a PNG image
        filename = os.path.join(self.save_dir, f"frame_{self.t}_{self.n}_{self.i}.png")
        image = Image.fromarray(np.flipud(frame))
        image.save(filename)

    def run(self):
        tqdm_range = tqdm.tqdm(range(self.T-1))
        for t in tqdm_range:
            self.t = t
            mujoco.mj_forward(self.model, self.data)
            self.store_trajectory(t)
            self.ctrl[:, t] = self.data.ctrl

            if self.agent is not None:
                if t % self.update_ratio == 0:
                    action = self.agent.update(np.concatenate([self.data.qpos, self.data.qvel], axis=0))
                self.data.ctrl = action

            mujoco.mj_step(self.model, self.data)
            mujoco.mj_forward(self.model, self.data)
            
            error = np.linalg.norm(np.array(self.agent.body_ref[:3]) - np.array(self.data.qpos[:3]))
            if error < self.agent.goal_thresh[self.agent.goal_index]:
                self.agent.next_goal()

            if self.viewer is not None and self.viewer.is_alive:
                self.viewer.add_marker(
                    pos=self.agent.body_ref[:3]*1,         # Position of the marker
                    size=[0.15, 0.15, 0.15],     # Size of the sphere
                    rgba=[1, 0, 1, 1],           # Color of the sphere (red)
                    type=mujoco.mjtGeom.mjGEOM_SPHERE, # Specify that this is a sphere
                    label=""
                )
                            
                self.viewer.render()
                if self.save_frames:
                    self.capture_frame()
            else:
                pass
        
        # store last state
        self.store_trajectory(self.T-1)
        
        if self.viewer is not None:
            self.viewer.close()
        return None

    def plot_trajectory(self):
        np.savetxt('analysis/mujoco_traj_param_rate_{}_h_{}_lam_{}_n_{}_T_{}_task_{}.tsv'.format(self.ctrl_rate, \
                                                                                                 self.agent.horizon, \
                                                                                                 self.agent.temperature, \
                                                                                                 self.agent.n_samples, \
                                                                                                 self.T,\
                                                                                                 self.agent.task),\
                                                                                                 self.cost, delimiter='\t')
        # position
        plt.figure()
        
        plt.plot(self.time, self.qpos[0, :], label="x (sim)", ls="--", color="blue")
        plt.plot(self.time, self.qpos[1, :], label="y (sim)", ls="--", color="orange")
        plt.plot(self.time, self.qpos[2, :], label="z (sim)", ls="--", color="magenta")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")

        # orientation plot
        fig = plt.figure()

        plt.plot(self.time, self.qpos[3, :], label="q0 (sim)", ls="--", color="blue")
        plt.plot(self.time, self.qpos[4, :], label="q1 (sim)", ls="--", color="orange")
        plt.plot(self.time, self.qpos[5, :], label="q2 (sim)", ls="--", color="magenta")
        plt.plot(self.time, self.qpos[6, :], label="q3 (sim)", ls="--", color="green")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Orientation")

        # plot sensor
        fig = plt.figure()
        plt.plot(self.time, self.sensordata[0, :], label="pos x", color="blue")
        plt.plot(self.time, self.noisy_sensordata[0, :], label="pos x (noisy)", ls="--", color="blue")
        plt.plot(self.time, self.sensordata[1, :], label="pos y", color="orange")
        plt.plot(self.time, self.noisy_sensordata[1, :], label="pos y (noisy)", ls="--", color="orange")
        plt.plot(self.time, self.sensordata[2, :], label="pos z", color="magenta")
        plt.plot(self.time, self.noisy_sensordata[2, :], label="pos z (noisy)", ls="--", color="magenta")
        plt.plot(self.time, self.sensordata[3, :], label="q0", color="blue")
        plt.plot(self.time, self.noisy_sensordata[3, :], label="q0 (noisy)", ls="--", color="blue")
        plt.plot(self.time, self.sensordata[4, :], label="q1", color="orange")
        plt.plot(self.time, self.noisy_sensordata[4, :], label="q1 (noisy)", ls="--", color="orange")
        plt.plot(self.time, self.sensordata[5, :], label="q2", color="magenta")
        plt.plot(self.time, self.noisy_sensordata[5, :], label="q2 (noisy)", ls="--", color="magenta")
        plt.plot(self.time, self.sensordata[6, :], label="q3", color="green")
        plt.plot(self.time, self.noisy_sensordata[6, :], label="q3 (noisy)", ls="--", color="green")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Sensor reading")

        # plot velocity
        fig = plt.figure()
        plt.plot(self.time, self.qvel[0, :], label="vx", color="blue", ls="--")
        plt.plot(self.time[1:], self.finite_diff_qvel[0, :], label="vx (finite diff)", color="blue", ls=":")

        plt.plot(self.time, self.qvel[1, :], label="vy", color="orange", ls="--")
        plt.plot(self.time[1:], self.finite_diff_qvel[1, :], label="vy (finite diff)", color="orange", ls=":")

        plt.plot(self.time, self.qvel[2, :], label="vz", color="magenta", ls="--")
        plt.plot(self.time[1:], self.finite_diff_qvel[2, :], label="vz (finite diff)", color="magenta", ls=":")
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")

        # plt angular velocity
        fig = plt.figure()
        plt.plot(self.time, self.qvel[3, :], label="wx", color="blue", ls="--")
        plt.plot(self.time[1:], self.finite_diff_qvel[3, :], label="wx (finite diff)", color="blue", ls=":")
        plt.plot(self.time, self.qvel[4, :], label="wy", color="orange", ls="--")
        plt.plot(self.time[1:], self.finite_diff_qvel[4, :], label="wy (finite diff)", color="orange", ls=":")
        plt.plot(self.time, self.qvel[5, :], label="wz", color="magenta", ls="--")
        plt.plot(self.time[1:], self.finite_diff_qvel[5, :], label="wz (finite diff)", color="magenta", ls=":")
        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")
        

        # plot controls
        fig = plt.figure()
        plt.plot(self.time[:], self.ctrl[0, :], label="FR_thigh", color="blue")
        plt.plot(self.time[:], self.ctrl[1, :], label="FR_hip", color="orange")
        plt.plot(self.time[:], self.ctrl[2, :], label="FR_knee", color="magenta")
        plt.plot(self.time[:], self.ctrl[3, :], label="FL_thigh", color="green")
        plt.plot(self.time[:], self.ctrl[4, :], label="FL_hip", color="red")
        plt.plot(self.time[:], self.ctrl[5, :], label="FL_knee", color="purple")
        plt.plot(self.time[:], self.ctrl[6, :], label="RR_thigh", color="black")
        plt.plot(self.time[:], self.ctrl[7, :], label="RR_hip", color="blue")
        plt.plot(self.time[:], self.ctrl[8, :], label="RR_knee", color="orange")
        plt.plot(self.time[:], self.ctrl[9, :], label="RL_thigh", color="magenta")
        plt.plot(self.time[:], self.ctrl[10, :], label="RL_hip", color="green")
        plt.plot(self.time[:], self.ctrl[11, :], label="RL_knee", color="red")

        plt.legend()
        plt.xlabel("Time (s)")
        plt.ylabel("Control (angles)")
        plt.show()
        
        return None

    def get_state(self):
        return np.concatenate([self.data.qpos, self.data.qvel], axis=0)

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
   
if __name__ == "__main__":
    model_path = os.path.join(os.path.dirname(__file__), "../models/go1/task_simulate.xml")

    simulator = Simulator(filter=None, T = 300, dt=0.002, viewer=True, gravity=True, model_path=model_path)
    simulator.run()
    simulator.plot_trajectory()



