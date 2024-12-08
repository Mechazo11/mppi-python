"""
Module that implements MPPI control of a 2W/4W differential drive robot.

Submitted as part of EE 7500 Model Preidctive Control final project.
MPPI code is based on Mizuho Aoki's simple_mppi project found here: https://github.com/MizuhoAOKI/python_simple_mppi

Author:
Azmyin Md. Kamal,
Ph.D. student in MIE,
Louisiana State University,
Louisiana, USA

Date: December 1st, 2024
Version: 1.0
"""
# Imports
import numpy as np
import matplotlib.pyplot as plt
import numpy as np

class DiffDriveVehicle:
    """
    Class to implement Forward Kinematics of AMR using Differential Drive model.
    
    vehicle state variables
        x: x-axis position in the global/inertial frame [m]
        y: y-axis position in the global/inertial frame [m]
        theta: orientation in the global/inertial frame [rad]
    Control input:
        omega_left_wheel: Left wheel angular velocity [rad/s] (positive in the counterclockwize direction)
        omega_right_wheel: Right wheel angular velocity [rad/s] (positive in the forward direction)
    
    Note 1: Dynamics of the vehicle is the Kinematic Differential Drive Model. 
    Note 2: All theory and implementation is based on the Forward Differential Drive model explained by Dr. Haber https://www.youtube.com/watch?v=fx6bxPJ6BEs
    Note 3: Input constraints will be in the form -u_abs <= u <= u_abs
    Note 4: negative omega b ==> clockwise rotation, positive omega_b ==> Counter-clockwise rotation
    """
    
    def __init__(self,
                delta_t: float = 0.05, # [s]
                ref_path: np.ndarray = np.array([[-100.0, 0.0], [100.0, 0.0]]), # [m, m]  # noqa: B008
                wheel_radius: float = 0.5, # Wheel radius [m]
                wheel_num: int = 4, # number of wheels [int]
                wheel_base: int = 2.5, # distance betwen front and back wheels, [m]
                vehicle_lenght: float = 4.0, # Lenght of vehicle [float]
                vehicle_width: float =  3.0, # Widht of vehicle [float]
                abs_vb_val: float = 2.0, # Maximum linear velocity of robot base [m/s]
                abs_omega_val: float = 1.0, # Maximum angular velocity of robot base [rad/s]
                obstacle_circles: np.ndarray = np.array([[-2.0, 1.0, 1.0], [2.0, -1.0, 1.0]]), # [obs_x, obs_y, obs_radius]
                init_state: np.ndarray = np.array([0.0, 0.0, 0.0])
                ) -> None:
        # Geometric paramters
        self.wheel_num = wheel_num
        self.obstacle_circles = obstacle_circles
        # Distance betweens centers of left and right side wheel/axle lenght 
        self.track_width_s = vehicle_width + wheel_radius  # |--r--|----body----|--r--| hence r + 2*(r/2) = wheelbase
        self.wheel_base_l = wheel_base # [m]
        self.wheel_radius = wheel_radius # [m]
        self.vehicle_length = vehicle_lenght #[m]
        self.vehicle_width = vehicle_width #[m]
        # Constraints
        self.abs_vb_val = abs_vb_val # [m/s]
        self.abs_omega_val = abs_omega_val # [rad/s]
        self.curr_body_vel = 0.0 # [m/s] Robot's base linear velocity at timestep t
        # Simulation parameters
        self.delta_t = delta_t #[s]
        self.ref_path = ref_path # np.ndarray([])
        self.out_arr = [] # List containing optimal u1, u2, q_t+1 optimal_trajectory and sample_trajectories at timestep t
        self.state = init_state # q_(t) [np.ndarray]
        self.u_history = np.empty((0,2)) # vb, omega_b

    def update(
            self,
            u: np.ndarray, # Optimal control
            x_t_prev: np.ndarray, # Previous good state
            delta_t: float = 0.0, # To support variable timestep
            optimal_traj: np.ndarray = np.empty(0), # predicted optimal trajectory from MPPI
            sampled_traj_list: np.ndarray = np.empty(0), # sampled trajectories from MPPI
        ) -> list:
        """
        Update q_(t+1) by executing Forward Kinematics.
        
        Based on Aoki's implementation modified to work with Differential drive robots
        """
        # Initialize work variables
        self.out_arr = []
        x_t =x_t_prev
        vb_t = 0.0
        omega_t = 0.0
        psi_dot_arr = [] # List[psi_dot_l, psi_dot_r]
        self.u_history=np.vstack([self.u_history,u])
        # Run forward kinematics to get x_(t+1), also applies input constraints
        vb_t, omega_t, psi_dot_arr = self.forward_kinematics(x_t= x_t, u1=u[0], u2=u[1], delta_t=delta_t)
        # Update state
        self.out_arr = [vb_t, omega_t, psi_dot_arr, self.state.copy(), optimal_traj, sampled_traj_list] # List[u1, u2, x(t+1), nd.ndarray, np.ndarray]

    def forward_kinematics(self, x_t: np.ndarray, u1: np.ndarray, u2:np.ndarray, delta_t: float = 0.5):
        """
        Compute q_(t+1), i.e. the next state.
        
        All vehicle geometric values are available when the class was initialized.
        Note: Inputs v_t must be constrained within limits
        """
        # Initialize work variables
        new_x, new_y, new_theta = 0.0,0.0,0.0
        x_dot, y_dot, theta_dot = 0.0,0.0,0.0
        psi_dot_l,psi_dot_r = 0.0, 0.0
        v_l, v_r = 0,0
        # Prepare params
        x, y, theta = x_t # Numpy unpacking directly works no need self.stat[:4]
        s = self.track_width_s
        r = self.wheel_radius
        dt = self.delta_t if delta_t == 0.0 else delta_t
        # Apply constraint limit control inputs (probably redundant)
        vb_t = np.clip(u1, -self.abs_vb_val, self.abs_vb_val)
        omega_t = np.clip(u2, -self.abs_omega_val, self.abs_omega_val)
        # Does not work
        #vb_t = np.clip(u1, self.abs_vb_val,0)
        #omega_t = np.clip(u2, 0, self.abs_omega_val)
        
        # Compute left and right linear wheel velocities
        v_l = vb_t - ((omega_t*s)/2) # [m/s]
        v_r = vb_t + ((omega_t*s)/2) # [m/s]

        # Compute left and right wheel angular velocities
        psi_dot_l = v_l/r # [rad/s]
        psi_dot_r = v_r/r # [rad/s]

        # Compute update
        x_dot = ((r*psi_dot_l)/2) * np.cos(theta) + ((r*psi_dot_r)/2) * np.cos(theta)
        y_dot = ((r*psi_dot_l)/2) * np.sin(theta) + ((r*psi_dot_r)/2) * np.sin(theta)
        theta_dot = (-r/s)*psi_dot_l + (r/s)*psi_dot_r

        # Compute next state
        new_x = x + x_dot * dt
        new_y = y + y_dot * dt
        new_theta = theta + theta_dot*dt
        
        self.curr_body_vel = np.asarray(vb_t) # Update velocity of robot base
        self.state = np.array([new_x, new_y, new_theta]) # Update state
        return u1, u2, [psi_dot_l, psi_dot_r]

    def get_state(self) -> np.ndarray:
        """Return state variables, q(t)."""
        return self.state.copy()