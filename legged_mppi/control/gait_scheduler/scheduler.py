import numpy as np

class GaitScheduler:
    """
    A class for managing gait schedules for a quadruped robot.

    Attributes:
    -----------
    gait : np.ndarray
        A 2D array representing the gait pattern loaded from a file.
        Each row corresponds to a leg, and each column represents a timestep.
    phase_length : int
        Number of phases in the gait pattern.
    phase_time : int
        The current phase time index.
    indices : np.ndarray
        An array of phase indices for reference during rolling.
    type : str
        The name/type of the gait (e.g., 'walk', 'trot').

    Methods:
    --------
    roll():
        Advances the gait to the next phase by incrementing the phase time
        and updating the phase indices.
    get_current_ref():
        Returns the current reference values (gait states) for all legs
        at the current phase.
    """

    def __init__(self, gait_path, name='walk', phase_time=0):
        """
        Initializes the GaitScheduler with a gait pattern file.

        Parameters:
        -----------
        gait_path : str
            Path to the file containing the gait pattern (tab-delimited).
        name : str, optional
            The name of the gait type. Default is 'walk'.
        phase_time : int, optional
            Initial phase time index. Default is 0.
        """
        # Load the configuration file
        with open(gait_path, 'r') as file:
            gait_array = np.loadtxt(file, delimiter='\t')
        
        # Initialize attributes
        self.gait = gait_array
        self.phase_length = gait_array.shape[1]
        self.phase_time = phase_time
        self.indices = np.arange(self.phase_length)
        self.type = name
        
    def roll(self):
        """
        Advances the gait to the next phase.
        Increments the phase time and rotates the phase indices.
        """
        self.phase_time += 1
        self.indices = np.roll(self.indices, -1)
    
    def get_current_ref(self):
        """
        Retrieves the current reference values for all legs
        based on the current phase time.

        Returns:
        --------
        np.ndarray:
            The current gait states for all legs at the current phase.
        """
        return self.gait[:, self.phase_time] 


class Timer:
    """
    A class to manage timing for tasks or phases in a simulation.

    Attributes:
    -----------
    elapsed_time : int
        The current elapsed time.
    end_time : int
        The time at which the timer completes.
    done : bool
        A flag indicating whether the timer has finished.
    waiting : bool
        A flag for indicating if the timer is paused or waiting.

    Methods:
    --------
    increment():
        Advances the timer by one time step.
    reset():
        Resets the timer to its initial state.
    """

    def __init__(self, init_time=0, end_time=300):
        """
        Initializes the Timer with optional start and end times.

        Parameters:
        -----------
        init_time : int, optional
            The initial time value. Default is 0.
        end_time : int, optional
            The end time value when the timer should stop. Default is 300.
        """
        self.elapsed_time = init_time
        self.end_time = end_time
        self.done = False
        self.waiting = False
        
    def increment(self):
        """
        Advances the timer by one time step.
        Marks the timer as 'done' if the elapsed time reaches the end time.
        """
        if self.elapsed_time < self.end_time:
            self.elapsed_time += 1
        else:
            self.done = True
    
    def reset(self):
        """
        Resets the timer to its initial state.
        """
        self.elapsed_time = 0 
        self.done = False