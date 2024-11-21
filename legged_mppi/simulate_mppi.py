import numpy as np
from interface.simulator import Simulator
from control.controllers.mppi_locomotion import MPPI
from utils.tasks import get_task

import os

def main():
    T = 100000 # 20 seconds
    TIMECONST = 0.02
    VIEWER = True
    task = 'walk_octagon'

    SIMULATION_STEP = 0.01 #0.002
    CTRL_UPDATE_RATE = 100
    CTRL_HORIZON = 40
    CTRL_LAMBDA = 0.1
    CTRL_N_SAMPLES = 30

    task_data = get_task(task)
    sim_path = task_data["sim_path"]

    agent = MPPI(task=task)
    agent.set_params(horizon=CTRL_HORIZON, lambda_=CTRL_LAMBDA, N=CTRL_N_SAMPLES)
    simulator = Simulator(agent=agent, viewer=VIEWER, T=T, dt=SIMULATION_STEP, timeconst=TIMECONST,
                          model_path=sim_path, ctrl_rate=CTRL_UPDATE_RATE)
    
    simulator.run()
    simulator.plot_trajectory()
    pass

if __name__ == "__main__":
    main()