from scipy.spatial.transform import Rotation as R
import numpy as np

def batch_world_to_local_velocity(quaternions, world_velocities):
    """
    Transforms a batch of world frame velocities to local body frame using quaternion orientations.
    
    Parameters:
    quaternions (np.array): Array of shape (t, 4) where each quaternion is [q_w, q_x, q_y, q_z]
    world_velocities (np.array): Array of shape (t, 3) where each velocity is [v_x, v_y, v_z]
    
    Returns:
    local_velocities (np.array): Array of shape (t, 3) with local body frame velocities
    """
    # Create a rotation object from the batch of quaternions (scipy expects [q_x, q_y, q_z, q_w])
    rotation = R.from_quat(quaternions[:, [1, 2, 3, 0]])  # Reordering to [q_x, q_y, q_z, q_w]
    
    # Apply the inverse rotation to each velocity vector in the batch
    local_velocities = rotation.inv().apply(world_velocities)
    return local_velocities

def calculate_orientation_quaternion(current_point, goal_point):
    """
    Calculates the orientation quaternion that points from the current point to the goal point
    with counterclockwise rotations around the z-axis (yaw) and y-axis (pitch).
    
    Parameters:
    current_point (np.array): The current position as [x, y, z].
    goal_point (np.array): The goal position as [x, y, z].
    
    Returns:
    quaternion (np.array): The orientation quaternion [q_w, q_x, q_y, q_z].
    """
    
    # Calculate the direction vector from the current point to the goal point
    direction = goal_point - current_point
    length = np.linalg.norm(direction)
    #print("Error: ", length)
    direction_normalized = direction / length
    
    # Calculate yaw (rotation around the z-axis)
    yaw = np.arctan2(direction_normalized[1], direction_normalized[0])
    
    # Calculate pitch (rotation around the y-axis)
    pitch = -np.arctan2(direction_normalized[2], np.sqrt(direction_normalized[0]**2 + direction_normalized[1]**2))
    
    # Convert yaw and pitch to quaternions
    yaw_quat = R.from_euler('z', yaw).as_quat()  # Yaw quaternion
    pitch_quat = R.from_euler('y', pitch).as_quat()  # Pitch quaternion
    
    # Combine the quaternions by multiplying them
    combined_rotation = R.from_quat(yaw_quat) * R.from_quat(pitch_quat)
    quaternion = combined_rotation.as_quat()
    
    # Reorder the quaternion to [q_w, q_x, q_y, q_z]
    quaternion = np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])
    
    return quaternion