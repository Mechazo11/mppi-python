"""
Module implementing various utility functions.

Author:
Azmyin Md. Kamal,
Ph.D. student in MIE,
Louisiana State University,
Louisiana, USA

Date: November 30th, 2024
Version: 1.0

AI: ChatGPT 4.0
"""

# imports
import time
import numpy as np
import matplotlib.pyplot as plt

# Utility functions
def curr_time():
    """Return the time tick in milliseconds."""
    return time.monotonic() * 1000

def debug_lock():
    """Locks system in an infinite loop for debugging."""
    print("LOCK")
    while 1:
        pass

def results_inputs(u_optims:list[np.ndarray], matched_indices:list, step_val:int)-> None:
    """Compute averages and plot optimal inputs (linear and angular body velocities)."""
    vb_arr = u_optims[:, 0]  # First column linear velocity vb
    wb_arr = u_optims[:, 1]  # Second column, angular velocity omega_b
    t_steps = np.linspace(0,step_val,vb_arr.shape[0])
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 8))
    # First subplot
    ax1.plot(t_steps,vb_arr, 'b-', label='$V_b$', linewidth=2)
    #ax1.set_xlabel('Timesteps (s)')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.set_title('u(0): Linear velocity $V_b$')
    ax1.legend()
    ax1.grid(True)
    # Second subplot
    ax2.plot(t_steps,wb_arr, 'r-', label='$\omega_b$', linewidth=2)
    ax2.set_xlabel('Timesteps (s)')
    ax2.set_ylabel('Angular velocity (rad/s)')
    ax2.set_title('u(1): Angular velocity $\omega_b$')
    ax2.legend()
    ax2.grid(True)
    # Save plot
    plt.savefig("input_figs.pdf", format="pdf")

    print(f"Mean linear velocity: {np.mean(vb_arr)} m/s")
    print(f"Mean angular velocity: {np.mean(wb_arr)} rad/s")

def compute_rmse(actual:np.ndarray, predicted:np.ndarray)->float:
    """Compute Root Mean Squared error between two equally shaped np.ndarray."""
    # Calculate the mean squared error (MSE) by taking the mean of the squared differences
    meanSquaredError = ((predicted - actual) ** 2).mean()
    # Calculate the RMSE by taking the square root of the MSE
    rmse = np.sqrt(meanSquaredError)
    return rmse

# RMSE value
def compute_erros(ref_path:np.ndarray, matched_path:np.ndarray)->None:
    """Compute positional and rotational errors."""
    x_true = ref_path[:,0] # First column X positions
    x_tracked = matched_path[:,0] # First column, X positions
    y_true = ref_path[:,1] # Second column Y positions
    y_tracked = matched_path[:,1]
    yaw_true = ref_path[:,2] # Third column yaw positions
    yaw_tracked = matched_path[:,2]
    x_rmse = compute_rmse(x_true, x_tracked)
    y_rmse = compute_rmse(y_true, y_tracked)
    yaw_rmse = compute_rmse(yaw_true, yaw_tracked)
    # Print stats
    print(f"RMSE for X-position: {x_rmse} m")
    print(f"RMSE for Y-position: {y_rmse} m")
    print(f"RMSE for Yaw : {yaw_rmse} radians")

def plot_ref_vs_tracked_path(ref_path: np.ndarray, tracked_path: list, 
                             obstacle_arr:np.ndarray = np.array([]),
                             save_plot:bool = False) -> None:
    """
    Plot Reference vs Trajectory Paths.
    
    AI Generated Code: Claude 3.5 Sonnet
    """
    # Extract reference x, y coordinates
    ref_x = ref_path[:, 0]
    ref_y = ref_path[:, 1]
    
    # Convert tracked path to numpy array for easier processing
    # np.array([x,y,yaw])
    tracked_points = np.array([[point[0], point[1], point[2]] for point in tracked_path])
    
    # Find closest tracked points to each reference point
    matched_indices = []
    for ref_point in ref_path[:, :3]:
        distances = np.linalg.norm(tracked_points - ref_point, axis=1)
        closest_idx = np.argmin(distances)
        matched_indices.append(closest_idx)
    
    # Get matched tracked points
    matched_tracked = tracked_points[matched_indices]

    # Create plot
    # Plot paths first
    plt.figure(figsize=(10, 6))
    plt.plot(ref_x, ref_y, 'b-', label='Reference Path', linewidth=2)
    plt.plot(matched_tracked[:, 0], matched_tracked[:, 1], 'r--', 
             label='Tracked Path', linewidth=2)
    
    for obstacle in obstacle_arr:
        circle = plt.Circle((obstacle[0], obstacle[1]), obstacle[2], color='gray', alpha=0.5)
        plt.gca().add_patch(circle)

    # Find the y-value on the reference path at x=0
    start_y = ref_y[np.abs(ref_x).argmin()]
    plt.plot([0, 0], [start_y+0.1, start_y + 3], 'gold', linewidth=5, label='Start Position')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Reference vs Tracked Path')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    if save_plot:
        plt.savefig("./figs/ref_vs_traj.pdf", format="pdf")
    plt.show()

    return matched_tracked, matched_indices