import numpy as np
import os
from whole_body_mppi.interface.simulator import Simulator
from whole_body_mppi.control.controllers.mppi_locomotion import MPPI
from whole_body_mppi.utils.tasks import get_task

import argparse

def main(task):
    T = 2000  # 20 seconds
    VIEWER = True

    SIMULATION_STEP = 0.01
    CTRL_UPDATE_RATE = 100
    CTRL_HORIZON = 40
    CTRL_LAMBDA = 0.1
    CTRL_N_SAMPLES = 30

    # Soft contact model paramters
    TIMECONST = 0.02
    DAMPINGRATIO = 1.0
    

    # Get task data
    task_data = get_task(task)
    sim_path = os.path.join(os.path.dirname(__file__), "../whole_body_mppi", task_data["sim_path"])

    # Initialize agent and simulator
    agent = MPPI(task=task)
    # agent.set_params(horizon=CTRL_HORIZON, lambda_=CTRL_LAMBDA, N=CTRL_N_SAMPLES)
    simulator = Simulator(agent=agent, viewer=VIEWER, T=T, dt=SIMULATION_STEP, timeconst=TIMECONST,
                          dampingratio=DAMPINGRATIO, model_path=sim_path, ctrl_rate=CTRL_UPDATE_RATE)
    
    # Run simulation
    simulator.run()
    simulator.plot_trajectory()

if __name__ == "__main__":
    # Define valid tasks
    VALID_TASKS = ['stairs', 'stand', 'walk_octagon', 'walk_straight', 'big_box',
                   'walk_octagon_hw', 'walk_straight_hw', 'stand_hw', 'climb_box_hw']

    # Parse arguments
    parser = argparse.ArgumentParser(description="Run simulation with a specified task.")
    parser.add_argument('--task', type=str, required=True, choices=VALID_TASKS, 
                        help=f"Name of the task. Must be one of {VALID_TASKS}.")
    args = parser.parse_args()

    # Run main with the provided task
    main(args.task)