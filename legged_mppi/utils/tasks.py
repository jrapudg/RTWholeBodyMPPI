"""
Task definitions for robot navigation and behavior scenarios.

Each task is represented as a dictionary containing key parameters:
- `goal_pos`: List of target positions in the format [x, y, z].
- `default_orientation`: Default orientation of the robot as a quaternion [w, x, y, z].
- `cmd_vel`: Commanded velocities in the format [linear x, linear y] in body frame.
- `goal_thresh`: Thresholds for achieving goals.
- `desired_gait`: Gait type for each phase of the task.
- `waiting_times`: Time in milliseconds to wait at each phase.
- `model_path`: Path to the robot's model file.
- `config_path`: Path to the robot's configuration file.
- `sim_path`: Path to the simulation file.
"""

DEFAULT_MODEL_PATH = 'models/go1/go1_scene_mppi_pyr.xml'
DEFAULT_CONFIG_PATH = 'configs/mppi_gait_config_walk.yml'
DEFAULT_SIM_PATH = 'models/go1/go1_scene_mppi.xml'
DEFAULT_ORIENTATION = [[1, 0, 0, 0]]


TASKS = {
    "walk_straight": {
        "goal_pos": [[0, 0, 0.27], 
                     [1, 0, 0.27], 
                     [1, 0, 0.27]],
        "default_orientation": DEFAULT_ORIENTATION,
        "cmd_vel": [[0.0, 0.0], 
                    [0.2, 0.0], 
                    [0.0, 0.0]],
        "goal_thresh": [0.2, 
                        0.2, 
                        0.2],
        "desired_gait": ['in_place', 
                         'walk_fast', 
                         'in_place'],
        "waiting_times": [0, 
                          0, 
                          0],
        "model_path": DEFAULT_MODEL_PATH,
        "config_path": DEFAULT_CONFIG_PATH,
        "sim_path": DEFAULT_SIM_PATH
    },
    "walk_octagon": {
        "goal_pos": [[0, 0, 0.27], 
                     [1, 0, 0.27], 
                     [2, 1, 0.27], 
                     [2, 2, 0.27],
                     [1, 3, 0.27], 
                     [0, 3, 0.27], 
                     [-1, 2, 0.27], 
                     [-1, 1, 0.27],
                     [0, 0, 0.27], 
                     [0, 0, 0.27]],
        "default_orientation": DEFAULT_ORIENTATION,
        "cmd_vel": [[0.0, 0.0]] + [[0.2, 0.0]] * 8 + [[0.0, 0.0]],
        "goal_thresh": [0.2] * 10,
        "desired_gait": ['in_place'] + 
                        ['walk_fast'] * 8 + 
                        ['in_place'],
        "waiting_times": [0] * 10,
        "model_path": DEFAULT_MODEL_PATH,
        "config_path": DEFAULT_CONFIG_PATH,
        "sim_path": DEFAULT_SIM_PATH
    },
    "big_box": {
        "goal_pos": [[0, 0, 0.27], 
                     [0.4, 0, 0.27], 
                     [0.7, 0, 0.7], 
                     [1, 0, 0.65], 
                     [1, 0, 0.65]],
        "default_orientation": DEFAULT_ORIENTATION,
        "cmd_vel": [[0.0, 0.0]] + [[0.5, 0.0]] * 3 + [[0.0, 0.0]],
        "goal_thresh": [0.2] * 5,
        "desired_gait": ['in_place', 
                         'walk', 
                         'trot', 
                         'trot', 
                         'in_place'],
        "waiting_times": [50,
                          0, 
                          0, 
                          0, 
                          200],
        "model_path": 'models/go1/go1_scene_mppi_pyr_big_box.xml',
        "config_path": 'configs/mppi_gait_config_big_box.yml',
        "sim_path": 'models/go1/go1_scene_mppi_pyr_big_box.xml'
    },
    "stairs": {
        "goal_pos": [[0.0, 0.0, 0.27], 
                     [0.8, 0.0, 0.27], 
                     [1.95, 0.0, 1.04],
                     [2.9, 0.0, 1.78], 
                     [3.2, 0.0, 1.78], 
                     [3.3, 0.0, 1.73]],
        "default_orientation": DEFAULT_ORIENTATION,
        "cmd_vel": [[0.0, 0.0]] + [[0.2, 0.0]] * 4 + [[0.0, 0.0]],
        "goal_thresh": [0.2] * 6,
        "desired_gait": ['in_place'] + ['walk'] * 4 + ['in_place'],
        "waiting_times": [50] + [0] * 5,
        "model_path": 'models/go1/go1_scene_mppi_stairs.xml',
        "config_path": 'configs/mppi_gait_config_stairs.yml',
        "sim_path": 'models/go1/go1_scene_mppi_stairs.xml'
    },
    "stand": {
        "goal_pos": [[0, 0, 0.27]],
        "default_orientation": DEFAULT_ORIENTATION,
        "cmd_vel": [[0.0, 0.0]],
        "goal_thresh": [0.2],
        "desired_gait": ['in_place'],
        "waiting_times": [0],
        "model_path": DEFAULT_MODEL_PATH,
        "config_path": DEFAULT_CONFIG_PATH,
        "sim_path": DEFAULT_SIM_PATH
    },
    "push_box": {
        "goal_pos": [[0, 0.0, 0.27], 
                     [1, 1, 0.27]],
        "default_orientation": DEFAULT_ORIENTATION,
        "cmd_vel": [[0.0, 0.0], 
                    [0.2, 0.0]],
        "goal_thresh": [0.2, 
                        0.2],
        "desired_gait": ['in_place', 
                         'walk'],
        "waiting_times": [None],
        "model_path": 'models/go1/go1_scene_mppi_pyr_push_box_14in.xml',
        "config_path": 'configs/mppi_gait_config_push_box.yml',
        "sim_path": 'models/go1/go1_scene_mppi_pyr_push_box_14in.xml'
    }
}

def get_task(task_name):
    """
    Retrieve task configuration by name.

    Args:
        task_name (str): Name of the task. Must be one of the keys in TASKS.

    Returns:
        dict: Task configuration dictionary.

    Raises:
        ValueError: If the task_name is not found in TASKS.
    """
    if task_name not in TASKS:
        raise ValueError(f"Task '{task_name}' not found. Available tasks: {list(TASKS.keys())}")
    return TASKS[task_name]