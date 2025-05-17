import numpy as np
import concurrent.futures
import threading
from concurrent.futures import ThreadPoolExecutor
import mujoco
from scipy.interpolate import CubicSpline
from mujoco import rollout
import yaml

class BaseMPPI:
    """
    Base class for Model Predictive Path Integral (MPPI) controllers.
    Provides shared functionality for all task-specific controllers.
    """

    def __init__(self, model_path, config_path):
        """
        Initialize common MPPI parameters and configurations.

        Args:
            params (dict): Dictionary of task parameters from configuration.
            model_path (str): Path to the MuJoCo model XML file.
            config_path (str): Path to the configuration file.
        """

        # Load task-specific configurations
        with open(config_path, 'r') as file:
            params = yaml.safe_load(file)

        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.model.opt.timestep = params['dt']
        self.model.opt.enableflags = 1  # Override contact settings
        self.model.opt.o_solref = np.array(params['o_solref'])

        # MPPI parameters
        self.temperature = params['lambda']
        self.horizon = params['horizon']
        self.n_samples = params['n_samples']
        self.noise_sigma = np.array(params['noise_sigma'])
        self.num_workers = params['n_workers']
        self.sampling_init = np.array([-0.3, 1.34, -2.83, 0.3, 1.34, -2.83] * 2)

        # Initialize rollouts and sampling configurations
        self.h = params['dt']
        self.sample_type = params['sample_type']
        self.n_knots = params['n_knots']
        self.random_generator = np.random.default_rng(params["seed"])
        self.rollout_func = self.threaded_rollout
        self.cost_func = self.calculate_total_cost

        # Threading
        self.thread_local = threading.local()
        self.executor = ThreadPoolExecutor(max_workers=self.num_workers, initializer=self.thread_initializer)

        # Initialize rollouts
        self.state_rollouts = np.zeros(
            (self.n_samples, self.horizon, mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_FULLPHYSICS.value))
        )
        self.selected_trajectory = None

        # Action limits
        self.act_dim = 12
        self.act_max = np.array([0.863, 4.501, -0.888] * 4)
        self.act_min = np.array([-0.863, -0.686, -2.818] * 4)

    def reset_planner(self):
        """Reset the action planner to its initial state."""
        self.trajectory = np.zeros((self.horizon, self.act_dim))
        self.trajectory += self.sampling_init

    def sample_delta_u(self):
        if self.sample_type == 'normal':
            size = (self.n_samples, self.horizon, self.act_dim)
            return self.generate_noise(size)
        elif self.sample_type == 'cubic':
            indices = np.arange(self.n_knots)*self.horizon//self.n_knots
            size = (self.n_samples, self.n_knots, self.act_dim)
            knot_points = self.generate_noise(size)
            cubic_spline = CubicSpline(indices, knot_points, axis=1)
            return cubic_spline(np.arange(self.horizon))
        
    def perturb_action(self):
        if self.sample_type == 'normal':
            size = (self.n_samples, self.horizon, self.act_dim)
            actions = self.trajectory + self.generate_noise(size)
            actions = np.clip(actions, self.act_min, self.act_max)
            return actions
        
        elif self.sample_type == 'cubic':
            indices_float = np.linspace(0, self.horizon - 1, num=self.n_knots)
            indices = np.round(indices_float).astype(int)
            size = (self.n_samples, self.n_knots, self.act_dim)
            noise = self.generate_noise(size)
            knot_points = self.trajectory[indices] + noise
            #knot_points[:, 0, :] = self.trajectory[0]
            cubic_spline = CubicSpline(indices, knot_points, axis=1)
            actions = cubic_spline(np.arange(self.horizon))
            actions = np.clip(actions, self.act_min, self.act_max)
            return actions
        
    def generate_noise(self, size):
        """
        Generate noise for sampling actions.

        Args:
            size (tuple): Shape of the noise array.

        Returns:
            np.ndarray: Generated noise scaled by `noise_sigma`.
        """
        return self.random_generator.normal(size=size) * self.noise_sigma

    def thread_initializer(self):
        """Initialize thread-local storage for MuJoCo data."""
        self.thread_local.data = mujoco.MjData(self.model)

    def shutdown(self):
        """Shutdown the thread pool executor."""
        self.executor.shutdown(wait=True)

    def call_rollout(self, initial_state, ctrl, state):
        """
        Perform a rollout of the model given the initial state and control actions.

        Args:
            initial_state (np.ndarray): Initial state of the model.
            ctrl (np.ndarray): Control actions to apply during the rollout.
            state (np.ndarray): State array to store the results of the rollout.
        """
        rollout.rollout(self.model, self.thread_local.data, skip_checks=True,
                        nroll=state.shape[0], nstep=state.shape[1],
                        initial_state=initial_state, control=ctrl, state=state)

    def threaded_rollout(self, state, ctrl, initial_state, num_workers=32, nstep=5):
        """
        Perform rollouts in parallel using a thread pool.

        Args:
            state (np.ndarray): Array to store the results of the rollouts.
            ctrl (np.ndarray): Control actions for the rollouts.
            initial_state (np.ndarray): Initial states for the rollouts.
            num_workers (int): Number of parallel threads to use.
            nstep (int): Number of steps in each rollout.
        """
        n = len(initial_state) // num_workers

        # Divide tasks into chunks for each worker
        chunks = [(initial_state[i * n:(i + 1) * n], ctrl[i * n:(i + 1) * n], state[i * n:(i + 1) * n])
                for i in range(num_workers - 1)]

        # Add remaining chunk
        chunks.append((initial_state[(num_workers - 1) * n:], ctrl[(num_workers - 1) * n:], state[(num_workers - 1) * n:]))

        # Submit tasks to thread pool
        futures = [self.executor.submit(self.call_rollout, *chunk) for chunk in chunks]
        for future in concurrent.futures.as_completed(futures):
            future.result()  # Ensure all threads complete execution

    def set_params(self, horizon, lambda_, N):
        """
        Update MPPI parameters and reset controller.

        Args:
            horizon (int): Time horizon.
            lambda_ (float): Temperature parameter for MPPI.
            N (int): Number of samples for MPPI rollouts.
        """
        self.horizon = horizon
        self.temperature = lambda_
        self.n_samples = N

        # Reset state rollouts with updated dimensions
        self.state_rollouts = np.zeros(
            (self.n_samples, self.horizon, mujoco.mj_stateSize(self.model, mujoco.mjtState.mjSTATE_FULLPHYSICS.value))
        )

        # Reset the planner to its initial state
        self.reset_planner()

    def __del__(self):
        self.shutdown()