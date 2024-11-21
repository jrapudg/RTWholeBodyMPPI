import os
import yaml
import mujoco
import numpy as np

# Local imports (ensure these are part of your package structure)
from utils.tasks import get_task
from control.controllers.base_controller import BaseMPPI
from control.gait_scheduler.scheduler import GaitScheduler
from control.gait_scheduler.scheduler import Timer
from utils.transforms import batch_world_to_local_velocity, calculate_orientation_quaternion

# Define base directory and paths for resource files
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
GAIT_DIR = os.path.join(BASE_DIR, "../gait_scheduler/gaits/")

# Paths for gait files
GAIT_INPLACE_PATH = os.path.join(GAIT_DIR, "FAST/walking_gait_raibert_FAST_0_0_10cm_100hz.tsv")
GAIT_TROT_PATH = os.path.join(GAIT_DIR, "MED/walking_gait_raibert_MED_0_5_15cm_100hz.tsv")
GAIT_WALK_PATH = os.path.join(GAIT_DIR, "MED/walking_gait_raibert_MED_0_1_10cm_100hz.tsv")
GAIT_WALK_FAST_PATH = os.path.join(GAIT_DIR, "FAST/walking_gait_raibert_FAST_0_1_10cm_100hz.tsv")

class MPPI(BaseMPPI):
    """
    Model Predictive Path Integral (MPPI) Controller for quadruped robots.

    Attributes:
        - Task-specific parameters and goals.
        - Gait scheduler and configurations.
        - MPPI sampling and cost calculation configurations.
    """

    def __init__(self, task='stand') -> None:
        """
        Initialize the MPPI controller with task-specific configurations.

        Args:
            task (str): The name of the task ('stand', 'walk').
        """
        print("Task: ", task)

        # Retrieve task-specific parameters
        self.task = task
        self.task_data = get_task(task)

        self.goal_pos = self.task_data['goal_pos']
        self.goal_ori = self.task_data['default_orientation'] 
        self.cmd_vel = self.task_data['cmd_vel']
        self.goal_thresh = self.task_data['goal_thresh']
        self.desired_gait = self.task_data['desired_gait']
        model_path = self.task_data['model_path'] 
        config_path = self.task_data['config_path']
        waiting_times = self.task_data['waiting_times']

        # Dynamically resolve paths for model and configuration files
        CONFIG_PATH = os.path.join(BASE_DIR, config_path)
        MODEL_PATH = os.path.join(BASE_DIR, "../..", model_path)

        # Initialize base MPPI
        super().__init__(MODEL_PATH, CONFIG_PATH)

        # load the configuration file
        with open(CONFIG_PATH, 'r') as file:
            params = yaml.safe_load(file)
        # Cost weights
        self.Q = np.diag(np.array(params['Q_diag']))
        self.R = np.diag(np.array(params['R_diag']))

        # Set initial parameters and state
        self.obs = None
        self.internal_ref = True
        self.exp_weights = np.ones(self.n_samples) / self.n_samples  # Initial MPPI weights
        self.waiting_times = waiting_times
        self.timer = Timer(end_time=self.waiting_times[0])

        # Initialize gait schedulers
        self.gaits = {
            'in_place': GaitScheduler(gait_path=GAIT_INPLACE_PATH, name='in_place'),
            'trot': GaitScheduler(gait_path=GAIT_TROT_PATH, name='trot'),
            'walk': GaitScheduler(gait_path=GAIT_WALK_PATH, name='walk'),
            'walk_fast': GaitScheduler(gait_path=GAIT_WALK_FAST_PATH, name='walk_fast')
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

        # Debug information
        print(f"Initial goal {self.goal_index}: {self.goal_pos[self.goal_index] }")
        print(f"Initial gait {self.desired_gait[self.goal_index]}")
    
    def next_goal(self):
        """
        Progress to the next goal based on the task sequence.
        Updates the internal reference trajectory and gait scheduler.
        """
        self.timer.increment()

        if self.goal_index < len(self.goal_pos) - 1 and self.timer.done:
            # Move to the next goal
            self.goal_index += 1
            self.body_ref[:3] = self.goal_pos[self.goal_index]
            self.body_ref[7:9] = self.cmd_vel[self.goal_index]
            self.gait_scheduler = self.gaits[self.desired_gait[self.goal_index]]
            self.timer.reset()
            self.timer.end_time = self.waiting_times[self.goal_index]
            print(f"Moved to next goal {self.goal_index}: {self.goal_pos[self.goal_index]}")
            print(f"Gait: {self.desired_gait[self.goal_index]}")
            self.timer.waiting = False

        elif self.goal_index == len(self.goal_pos) - 1 and not self.task_success and self.timer.done:
            # Final goal reached
            print("Task succeeded.")
            self.task_success = True

        else:
            self.timer.waiting = True

        if not self.task_success:
            if self.desired_gait[self.goal_index] in ['in_place', 'walk', 'walk_fast']:
                self.noise_sigma = np.array([0.06, 0.1, 0.1] * 4)
            elif self.desired_gait[self.goal_index] in ['trot']:
                self.noise_sigma = np.array([0.06, 0.2, 0.2] * 4)
        
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

        # Calculate the direction and distance to the goal
        direction = self.body_ref[:3] - obs[:3]
        goal_delta = np.linalg.norm(direction)

        # Update desired orientation based on the goal position
        if goal_delta > 0.1 and not self.timer.waiting:
            self.goal_ori = calculate_orientation_quaternion(obs[:3], self.body_ref[:3])
        else:
            self.goal_ori = np.array([1, 0, 0, 0])

        self.body_ref[3:7] = self.goal_ori

        # Perform rollouts using threaded rollout function
        self.rollout_func(self.state_rollouts, actions, np.repeat(
            np.array([np.concatenate([[0], obs])]), self.n_samples, axis=0), 
            num_workers=self.num_workers, nstep=self.horizon)

        # Update joint references from the gait scheduler
        if self.internal_ref:
            self.joints_ref = self.gait_scheduler.gait[:, self.gait_scheduler.indices[:self.horizon]]

        # Calculate costs for each sampled trajectory
        costs_sum = self.cost_func(self.state_rollouts[:, :, 1:], actions, self.joints_ref, self.body_ref)

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
        """
        Compute the distance between two sets of quaternions.

        Args:
            q1 (np.ndarray): Array of quaternions (N x 4).
            q2 (np.ndarray): Array of quaternions (N x 4).

        Returns:
            np.ndarray: Array of distances between the quaternions.
        """
        # Compute dot product between corresponding quaternions
        dot_products = np.einsum('ij,ij->i', q1, q2)
        # Compute distance as 1 - absolute dot product
        return 1 - np.abs(dot_products)


    def quadruped_cost_np(self, x, u, x_ref):
        """
        Compute the cost for quadruped motion based on state and action errors.

        Args:
            x (np.ndarray): Current states (N x state_dim).
            u (np.ndarray): Current actions (N x action_dim).
            x_ref (np.ndarray): Reference states (N x state_dim).

        Returns:
            np.ndarray: Computed cost for each sample.
        """
        kp = 50  # Proportional gain for joint error
        kd = 3   # Derivative gain for joint velocity error

        # Compute state error relative to the reference
        x_error = x - x_ref

        # Compute quaternion distance for orientation error
        q_dist = self.quaternion_distance_np(x[:, 3:7], x_ref[:, 3:7])
        x_error[:, 3] = q_dist
        x_error[:, 4] = q_dist
        x_error[:, 5] = q_dist
        x_error[:, 6] = q_dist

        # Compute joint and velocity errors
        x_joint = x[:, 7:19]
        v_joint = x[:, 25:]
        u_error = kp * (u - x_joint) - kd * v_joint

        # Compute positional cost (L1 norm for positional error)
        x_error[:, :3] = 0  # Ignore positional error for simplicity
        x_pos_error = x[:, :3] - x_ref[:, :3]
        L1_norm_pos_cost = np.abs(np.dot(x_pos_error, self.Q[:3, :3])).sum(axis=1)

        # Compute total cost
        cost = (
            np.einsum('ij,ik,jk->i', x_error, x_error, self.Q) +
            np.einsum('ij,ik,jk->i', u_error, u_error, self.R) +
            L1_norm_pos_cost
        )
        return cost


    def calculate_total_cost(self, states, actions, joints_ref, body_ref):
        """
        Calculate the total cost for all rollouts.

        Args:
            states (np.ndarray): Rollout states (samples x time steps x state_dim).
            actions (np.ndarray): Rollout actions (samples x time steps x action_dim).
            joints_ref (np.ndarray): Reference joint positions (time steps x joint_dim).
            body_ref (np.ndarray): Reference body state (state_dim).

        Returns:
            np.ndarray: Total cost for each sample.
        """
        num_samples = states.shape[0]
        num_pairs = states.shape[1]

        # Repeat body reference for all samples and time steps
        traj_body_ref = np.repeat(body_ref[np.newaxis, :], num_samples * num_pairs, axis=0)

        # Flatten states and actions for batch processing
        states = states.reshape(-1, states.shape[2])
        actions = actions.reshape(-1, actions.shape[2])

        # Repeat and reshape joint references for batch processing
        joints_ref = joints_ref.T
        joints_ref = np.tile(joints_ref, (num_samples, 1, 1))
        joints_ref = joints_ref.reshape(-1, joints_ref.shape[2])

        # Concatenate body and joint references for full reference state
        x_ref = np.concatenate(
            [traj_body_ref[:, :7], joints_ref[:, :12], traj_body_ref[:, 7:], joints_ref[:, 12:]],
            axis=1
        )

        # Rotate velocity vectors to the local frame
        rotated_ref = batch_world_to_local_velocity(states[:, 3:7], states[:, 19:22])
        states[:, 19:22] = rotated_ref

        # Compute cost for each rollout
        costs = self.quadruped_cost_np(states, actions, x_ref)

        # Sum costs across time steps for each sample
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
    mppi = MPPI()