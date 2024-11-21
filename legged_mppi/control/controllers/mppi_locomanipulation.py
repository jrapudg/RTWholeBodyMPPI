import os
import yaml
import mujoco
import numpy as np

# Local imports (ensure these are part of your package structure)
from utils.tasks import get_task
from control.controllers.base_controller import BaseMPPI
from control.gait_scheduler.scheduler import GaitScheduler
from utils.transforms import batch_world_to_local_velocity, calculate_orientation_quaternion

# Define base directory and paths for resource files
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
GAIT_DIR = os.path.join(BASE_DIR, "../gait_scheduler/gaits/")

# Paths for gait files
GAIT_INPLACE_PATH = os.path.join(GAIT_DIR, "FAST/walking_gait_raibert_FAST_0_0_10cm_100hz.tsv")
GAIT_TROT_PATH = os.path.join(GAIT_DIR, "MED/walking_gait_raibert_MED_0_5_15cm_100hz.tsv")
GAIT_WALK_PATH = os.path.join(GAIT_DIR, "MED/walking_gait_raibert_MED_0_1_10cm_100hz.tsv")

class MPPI_box_push(BaseMPPI):
    """
    Model Predictive Path Integral (MPPI) Controller for quadruped robots.

    Attributes:
        - Task-specific parameters and goals for locomanipulation.
        - Gait scheduler and configurations.
        - MPPI sampling and cost calculation configurations.
    """

    def __init__(self, task='box_push') -> None:
        """
        Initialize the MPPI controller with task-specific configurations.

        Args:
            task (str): The name of the task ('box_push'), only this task implemented.
        """
        print("Task: ", task)

        # Retrieve task-specific parameters
        self.task = task
        self.follow_box = False
        self.task_data = get_task(task)
        self.goal_pos = self.task_data['goal_pos']
        self.goal_ori = self.task_data['default_orientation'] 
        self.cmd_vel = self.task_data['cmd_vel']
        self.goal_thresh = self.task_data['goal_thresh']
        self.desired_gait = self.task_data['desired_gait']
        model_path = self.task_data['model_path'] 
        config_path = self.task_data['config_path']

        # Dynamically resolve paths for model and configuration files
        CONFIG_PATH = os.path.join(BASE_DIR, config_path)
        MODEL_PATH = os.path.join(BASE_DIR, "../..", model_path)

        # Initialize base MPPI
        super().__init__(MODEL_PATH, CONFIG_PATH)

        # load the configuration file
        with open(CONFIG_PATH, 'r') as file:
            params = yaml.safe_load(file)

        # Cost
        self.Q_robot = np.diag(np.array(params['Q_robot']))
        self.Q_box = np.diag(np.array(params['Q_box']))
        self.R = np.diag(np.array(params['R_diag']))
        self.x_box_ref = np.concatenate([np.array(params['box_pos_ref']), np.array(params['box_quat_ref'])])

        
        # Set initial parameters and state
        self.obs = None
        self.internal_ref = True
        self.exp_weights = np.ones(self.n_samples) / self.n_samples  # Initial MPPI weights

        # Initialize gait schedulers
        self.gaits = {
            'in_place': GaitScheduler(gait_path=GAIT_INPLACE_PATH, name='in_place'),
            'trot': GaitScheduler(gait_path=GAIT_TROT_PATH, name='trot'),
            'walk': GaitScheduler(gait_path=GAIT_WALK_PATH, name='walk')
        }

        self.gait_scheduler = self.gaits['in_place']

        # Initialize planner and goals
        self.reset_planner()
        self.goal_index = 0
        self.body_ref = np.concatenate((self.goal_pos[self.goal_index],
                                        self.goal_ori[self.goal_index],
                                        self.cmd_vel[self.goal_index],
                                        np.zeros(4)))
        
        self.gait_scheduler = self.gaits[self.desired_gait[self.goal_index]]
        self.task_success = False
        self.box_state = np.zeros(13) 
        self.robot_state = np.zeros(37) 

        # Debug information
        print(f"Initial goal {self.goal_index}: {self.goal_pos[self.goal_index] }")
        print(f"Initial gait {self.desired_gait[self.goal_index]}")

    def next_goal(self):
        """
        Progress to the next goal based on the task sequence.
        Updates the internal reference trajectory and gait scheduler.
        """
        if (self.goal_index == 0 and (len(self.goal_pos) - 1 > 0 and self.gait_scheduler.phase_time > 300)):
                if self.goal_index == 0:
                    self.follow_box = True
                self.goal_index = (self.goal_index + 1)
                self.body_ref[:2] = self.box_state[:2]
                self.body_ref[7:9] = self.cmd_vel[self.goal_index]
                self.gait_scheduler = self.gaits[self.desired_gait[self.goal_index]]
        elif self.follow_box:
            self.prev_body_ref = self.body_ref[:2]*1
            self.body_ref[:2] = self.box_state[:2]
            print("still working...")
            if np.linalg.norm(np.array(self.box_state[:2])-np.array(self.x_box_ref[:2])) < 0.3:
                print("Task succeed. Time to back up!")
                self.follow_box = False
                self.goal_index = (self.goal_index + 1)
                # Change Q box to 0
                self.Q_box = np.diag(np.zeros(3))
                self.body_ref[:2] = self.prev_body_ref
                self.body_ref[7:9] = [0,0]
                self.gait_scheduler = self.gaits['in_place']
        else:
            pass
        
    def update(self, obs):
        """
        Update the MPPI controller based on the current observation.

        Args:
            obs (np.ndarray): Current state observation.
        Returns:
            np.ndarray: Selected action based on the optimal trajectory.
        """
         # Generate perturbed actions for rollouts
        actions = self.perturb_action()
        self.obs = obs

        # Update the robot and box states
        self.box_state[:7] = obs[:7]
        self.robot_state[:19] = obs[7:26]
        self.box_state[7:] = obs[26:32]
        self.robot_state[19:] = obs[32:]

        # Calculate the direction and distance to the goal
        direction = self.body_ref[:3] - self.robot_state[:3]
        goal_delta = np.linalg.norm(direction)

        # Update desired orientation based on the goal position
        if (goal_delta > 0.1) and ((self.gait_scheduler == self.gaits['walk']) or (self.gait_scheduler == self.gaits['trot'])): 
            self.goal_ori = calculate_orientation_quaternion(self.robot_state[:3], self.body_ref[:3])
        elif self.gait_scheduler == self.gaits['in_place']:
            self.goal_ori = np.array([1,0,0,0])

        self.body_ref[3:7] = self.goal_ori
        self.rollout_func(self.state_rollouts, actions, np.repeat(np.array([np.concatenate([[0],self.obs])]), self.n_samples, axis=0), num_workers=self.num_workers, nstep=self.horizon)
        
        if self.internal_ref:
            self.joints_ref = self.gait_scheduler.gait[:, self.gait_scheduler.indices[:self.horizon]]
        
        costs_sum = self.cost_func(self.state_rollouts[:,:,np.r_[8:27, 33:51]],\
                                   self.state_rollouts[:,:,np.r_[1:8, 27:33]],\
                                   actions, 
                                   self.joints_ref, self.body_ref)

        # Update the gait scheduler
        self.gait_scheduler.roll()

        # Calculate MPPI weights for the samples
        min_cost = np.min(costs_sum)
        max_cost = np.max(costs_sum)
        self.exp_weights = np.exp(-1 / self.temperature * ((costs_sum - min_cost) / (max_cost - min_cost)))

        # Weighted average of action deltas
        weighted_delta_u = self.exp_weights.reshape(self.n_samples, 1, 1) * actions
        weighted_delta_u = np.sum(weighted_delta_u, axis=0) / (np.sum(self.exp_weights) + 1e-10)
        updated_actions = np.clip(weighted_delta_u, self.act_min, self.act_max)

        # Update the trajectory with the optimal action
        self.selected_trajectory = updated_actions
        self.trajectory = np.roll(updated_actions, shift=-1, axis=0)
        self.trajectory[-1] = updated_actions[-1]

        # Return the first action in the trajectory as the output action
        return updated_actions[0]
    
    def quaternion_distance_np(self, q1, q2):
        dot_products = np.einsum('ij,ij->i', q1, q2)
        return 1 - np.abs(dot_products)
    
    def quaternion_distance_1D_np(self, q1, q2):
        # Broadcasting q2 to match the shape of q1
        dot_products = np.einsum('ij,j->i', q1, q2)
        # Reshape to (N, 1) for the output
        return (1 - np.abs(dot_products)).reshape(-1, 1)

    def quadruped_cost_np(self, x_robot, x_box, u, x_robot_ref, x_box_ref):
        kp = 50
        kd = 3
        
        # ROBOT COST
        # Compute the error terms
        x_error = x_robot - x_robot_ref
        
        quat_robot_dist = self.quaternion_distance_np(x_robot[:, 3:7], x_robot_ref[:, 3:7])
        x_error[:, 3] = quat_robot_dist
        x_error[:, 4] = 0
        x_error[:, 5] = 0
        x_error[:, 6] = 0

        x_joint = x_robot[:, 7:19]
        v_joint = x_robot[:, 25:]
        u_error = kp * (u - x_joint) - kd * v_joint

        # Compute cost using einsum for precise matrix operations
        # Apply the matrix Q to x_error and R to u_error, sum over appropriate dimensions
        x_error[:, :3] = 0
        x_pos_error = x_robot[:,:3] - x_robot_ref[:,:3]
        L1_norm_pos_cost = np.abs(np.dot(x_pos_error, self.Q_robot[:3,:3])).sum(axis=1)
        
        # BOX COST
        #quat_box_dist = self.quaternion_distance_1D_np(x_box[:, 3:7], x_box_ref[3:7])
        pos_error_box = x_box[:, :3] - x_box_ref[:3]
        #x_error_box = np.concatenate([pos_error_box, quat_box_dist], axis=1)
        box_cost = np.abs(np.dot(pos_error_box, self.Q_box[:3,:3])).sum(axis=1)

        robot_cost = np.einsum('ij,ik,jk->i', x_error, x_error, self.Q_robot) + np.einsum('ij,ik,jk->i', u_error, u_error, self.R) + L1_norm_pos_cost
        return robot_cost + box_cost

    def calculate_total_cost(self, robot_state, box_state, actions, joints_ref, body_ref):
        num_samples = robot_state.shape[0]
        num_pairs = robot_state.shape[1]

        traj_body_ref = np.repeat(body_ref[np.newaxis,:], num_samples*num_pairs, axis=0)

        # Flatten robot_state and actions to two dimensions, treating all pairs per sample as a batch
        robot_state = robot_state.reshape(-1, robot_state.shape[2])
        box_state = box_state.reshape(-1, box_state.shape[2])
        actions = actions.reshape(-1, actions.shape[2])

        joints_ref = joints_ref.T
        joints_ref = np.tile(joints_ref, (num_samples, 1, 1))
        joints_ref = joints_ref.reshape(-1, joints_ref.shape[2])
        
        x_robot_ref = np.concatenate([traj_body_ref[:,:7], joints_ref[:,:12], 
                                traj_body_ref[:,7:], joints_ref[:,12:]], 
                                axis=1)
        # Compute batch costs
        rotated_ref = batch_world_to_local_velocity(robot_state[:,3:7], robot_state[:,19:22])
        robot_state[:,19:22] = rotated_ref
        costs = self.quadruped_cost_np(robot_state, box_state, actions, x_robot_ref, self.x_box_ref)
        # Sum costs for each sample
        total_costs = costs.reshape(num_samples, num_pairs).sum(axis=1)
        return total_costs
    
    def eval_best_trajectory(self):
        """
        Evaluate the cost of the best trajectory selected by MPPI.

        Returns:
            float: Cost of the best trajectory, or None if no observation is available.
        """
        if self.obs is None:
            # If no observation is available, return None
            return None
        else:
            # Create a rollout array for the best trajectory
            best_rollouts = np.zeros((1, self.horizon, mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_FULLPHYSICS.value)))
            # Perform rollout for the best trajectory
            self.rollout_func(best_rollouts, np.array([self.selected_trajectory]), np.repeat(np.array([np.concatenate([[0],self.obs])]), 1, axis=0), num_workers=self.num_workers, nstep=self.horizon)
        # Compute and return the cost of the best trajectory
        return (self.cost_func(best_rollouts[:,:,1:], np.array([self.selected_trajectory]), self.joints_ref, self.body_ref))[0]

    def __del__(self):
        self.shutdown()
    
if __name__ == "__main__":
    mppi = MPPI_box_push()