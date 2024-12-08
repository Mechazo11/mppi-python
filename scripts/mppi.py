"""

Module that implements Model Predictive Path Integral Controller

Based on the implementation by Mizuho Aoki, modified to work with Differential Drive robots
and the G. Williams et al. "Information-Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving"

Author:
Azmyin Md. Kamal,
Ph.D. student in MIE,
Louisiana State University,
Louisiana, USA

Date: December 1st, 2024
Version: 1.0
"""
# Imports
import math
import time
import numpy as np
from typing import Tuple
from scripts.diffdrive import DiffDriveVehicle
from scripts.utils import debug_lock, curr_time
from numba import njit

@njit
def affine_transform(xlist: np.ndarray, 
                     ylist: np.ndarray, angle: float, 
                     translation: np.ndarray) -> tuple:
   """Optimized affine transform."""
   cos_angle = np.cos(angle) 
   sin_angle = np.sin(angle)
   tx, ty = translation
   transformed_x = xlist * cos_angle - ylist * sin_angle + tx
   transformed_y = xlist * sin_angle + ylist * cos_angle + ty
   # Append first point to close shape
   return np.append(transformed_x, transformed_x[0]), np.append(transformed_y, transformed_y[0])

@njit
def find_nearest_waypoint(x, y, ref_path, prev_idx, search_len=200):
    """Optimized nearest waypoint search."""
    dx = ref_path[prev_idx:(prev_idx + search_len), 0] - x
    dy = ref_path[prev_idx:(prev_idx + search_len), 1] - y
    d = dx * dx + dy * dy
    nearest_idx = np.argmin(d) + prev_idx
    return (nearest_idx, 
            ref_path[nearest_idx, 0],  # ref_x
            ref_path[nearest_idx, 1],  # ref_y 
            ref_path[nearest_idx, 2],  # ref_yaw
            ref_path[nearest_idx, 3])  # ref_v

@njit
def check_collision(x_t: np.ndarray, 
                          vehicle_params: np.ndarray, 
                          obstacle_circles: np.ndarray) -> float:
    """Check if vehicle collides with an obstacle."""
    vw, vl, safety_margin_rate = vehicle_params
    vw, vl = vw * safety_margin_rate, vl * safety_margin_rate
    x, y, yaw = x_t
    vehicle_shape_x = np.array([-0.5*vl, -0.5*vl, 0.0, 0.5*vl, 0.5*vl, 0.5*vl, 0.0, -0.5*vl, -0.5*vl])
    vehicle_shape_y = np.array([0.0, 0.5*vw, 0.5*vw, 0.5*vw, 0.0, -0.5*vw, -0.5*vw, -0.5*vw, 0.0])
    
    rotated_x, rotated_y = affine_transform(vehicle_shape_x, 
                                            vehicle_shape_y, 
                                            yaw, np.array([x, y]))
    
    for obs in obstacle_circles:
        obs_x, obs_y, obs_r = obs
        for p in range(len(rotated_x)):
            if (rotated_x[p] - obs_x)**2 + (rotated_y[p] - obs_y)**2 < obs_r**2:
                return 1.0
                #return -1.0
    return 0.0

@njit
def calculate_stage_cost(x_t: np.ndarray, 
                         ref_point: np.ndarray, 
                         weights:np.ndarray, 
                         vehicle_vel:np.ndarray,
                         vehicle_params:np.ndarray,
                         obstacle_circles:np.ndarray) -> np.array:
    """JIT-compiled stage cost calculation."""
    x, y, yaw = x_t # numpy array
    yaw = ((yaw + 2.0*np.pi) % (2.0*np.pi)) # normalize angle between 0 to 2pi
    ref_x, ref_y, ref_yaw, ref_v = ref_point
    
    stage_cost = (weights[0] * (x-ref_x)**2 + 
                 weights[1] * (y-ref_y)**2 + 
                 weights[2] * (yaw-ref_yaw)**2 + 
                 weights[3] * (vehicle_vel-ref_v)**2)
    # Add collision penalty
    stage_cost += check_collision(x_t, vehicle_params, obstacle_circles) * 1.0e10
    return stage_cost

class MPPIControllerDiffDrive():
    def __init__(
            self,
            vehicle_model: DiffDriveVehicle,
            dim_x: int = 3, #[x,y,yaw]
            dim_u: int = 2,
            delta_t: float = 0.05,
            wheel_base: float = 2.5, # [m]
            abs_vb_val: float = 2.5, # Maximum linear speeds of robot base [m/s]
            abs_omega_val: float = 1.0, # Maximum angular speed of robot_base [rad/s]
            ref_path: np.ndarray = np.array([[0.0, 0.0, 0.0, 1.0], [10.0, 0.0, 0.0, 1.0]]),
            horizon_step_T: int = 30, # [int]
            number_of_samples_K: int = 1000, # [int]
            param_exploration: float = 0.0,
            param_lambda: float = 50.0,
            param_alpha: float = 1.0,
            sigma_matrix: np.ndarray = np.array([[0.5, 0.0], [0.0, 0.1]]), # Covariance matrix \uppercase{sigma}
            stage_cost_weight: np.ndarray = np.array([50.0, 50.0, 1.0, 20.0]), # weight for [x, y, yaw, v]
            terminal_cost_weight: np.ndarray = np.array([50.0, 50.0, 1.0, 20.0]), # weight for [x, y, yaw, v]
            visualize_optimal_traj = True,  # if True, optimal trajectory is visualized
            visualze_sampled_trajs = True, # if True, sampled trajectories are visualized
            obstacle_circles: np.ndarray = np.array([[-2.0, 1.0, 1.0], [2.0, -1.0, 1.0]]), # [obs_x, obs_y, obs_radius]
            collision_safety_margin_rate: float = 1.2, # safety margin for collision check
    ) -> None:
        """Initialize mppi controller for path-tracking."""
        # Class specific parameters
        self.vehicle_model = vehicle_model
        # MPPI parameters
        self.dim_x = dim_x # dimension of system state vector
        self.dim_u = dim_u # dimension of control input vector
        self.T = horizon_step_T # prediction horizon
        self.K = number_of_samples_K # number of sample trajectories
        self.param_exploration = param_exploration  # constant parameter of mppi
        self.param_lambda = param_lambda  # constant parameter of mppi
        self.param_alpha = param_alpha # constant parameter of mppi
        self.param_gamma = self.param_lambda * (1.0 - (self.param_alpha))  # constant parameter of mppi
        self.Sigma = sigma_matrix # deviation of noise
        self.stage_cost_weight = stage_cost_weight
        self.terminal_cost_weight = terminal_cost_weight
        self.visualize_optimal_traj = visualize_optimal_traj
        self.visualze_sampled_trajs = visualze_sampled_trajs
        # Simulation parameters
        self.delta_t = delta_t #[s]
        self.ref_path = ref_path
        self.flag_done = False
        # Vehicle parameters
        self.wheel_base = wheel_base #[m]
        self.vehicle_l = vehicle_model.vehicle_length #[m]
        self.vehicle_w = vehicle_model.vehicle_width #[m]
        self.abs_vb_val = abs_vb_val # [m/s]
        self.abs_omega_val = abs_omega_val # [rad/s]
        # Mppi variables
        self.u_prev = np.zeros((self.T, self.dim_u)) # 2D array with self.T rows and self.dimu_u columns
        # Obstacle parameters
        self.collision_safety_margin_rate = collision_safety_margin_rate
        self.obstacle_circles = obstacle_circles
        self.vehicle_params = np.array([self.vehicle_w, self.vehicle_l, self.collision_safety_margin_rate])
        # Ref_path info
        self.prev_waypoints_idx = 0

    def calc_control_input(self, observed_x: np.ndarray):
        """
        Calculate optimal control input.

        Note 1: Codeblocks matching the steps Aoki described in "controller_notes_mizuho.ipynb" notebook is indicated below
        Note 2: This function is mostly a one-to-one implementations of Algorithm 1 and Algorithm 2 from information-theoretic paper
        Note 3: Paper uses v_t, an input vector with added noise randomly sampled from a Gaussian distribution
        """

        # load previous control input sequence
        u = self.u_prev
        # Set initial x value from observation
        x0 = observed_x
        # time keeprs
        step1_arr = []
        step2_arr = []
        step3_arr = []
        step4_arr = []
        t0 = 0.0
        t1 = 0.0

        # Get the waypoint closest to current vehicle position 
        self._get_nearest_waypoint(x0[0], x0[1], update_prev_idx=True)
        
        if self.prev_waypoints_idx >= self.ref_path.shape[0]-1:
            # Did we reach the end of sequence?
            #print("Reached the end of the reference path.")
            self.flag_done = True
            return (), [], [], [], self.flag_done

        # Prepare buffer store cost of evaluating each sampled trajectory
        S = np.zeros(self.K) # state cost list for each number of sampled trajectory
        
        ## Step 1: Randomly sample input sequence ##
        # Sample noise epsilon_t in Step 1
        # size is self.K x self.T i.e. number of sampled traj x prediction horizon
        t0 = curr_time()
        epsilon = self._calc_epsilon(self.Sigma, self.K, self.T, self.dim_u) 
        t1 = curr_time()
        t_diff = t1 - t0
        step1_arr.append(t_diff)
        ## Step 1: Randomly sample input sequence ##

        ## Step 2: Predict future states and evaluate cost for each sample. ##
        # prepare buffer of sampled control input sequence
        v = np.zeros((self.K, self.T, self.dim_u)) # control input sequence with noise
        t0 = curr_time()
        # loop for 0 ~ K-1 samples
        for k in range(self.K):         
            x = x0 # set initial(t=0) state x i.e. observed state of the vehicle
            # loop for time step t = 1 ~ T
            for t in range(1, self.T+1):
                # get control input with noise
                if k < (1.0-self.param_exploration)*self.K:
                    v[k, t-1] = u[t-1] + epsilon[k, t-1] # sampling for exploitation
                else:
                    v[k, t-1] = epsilon[k, t-1] # sampling for exploration
                # update x
                x = self._F(x, self._g(v[k, t-1]), self.delta_t)
                # add stage cost
                S[k] += self._cost_jit(x) + self.param_gamma * u[t-1].T @ np.linalg.inv(self.Sigma) @ v[k, t-1]
            # add terminal cost
            # S[k] += self._phi(x)
            S[k] += self._cost_jit(x) # Both stage and terminal cost are the same quadratic function
        ## Step 2: Predict future states and evaluate cost for each sample. ##
        t1 = curr_time()
        t_diff = t1 - t0
        step2_arr.append(t_diff)

        ## Step 3 Compute information theoretic weights for each sample ##
        ## NOTE: According to the paper, this part was done in parallel in a GPU
        t0 = curr_time()
        w = self._compute_weights(S) ## Algorithm 2
        # calculate w_k * epsilon_k
        w_epsilon = np.zeros((self.T, self.dim_u)) # Same shape as u_prev
        for t in range(self.T): # loop for time step t = 0 ~ T-1
            # For each k in the self.K cost list
            for k in range(self.K):
                w_epsilon[t] += w[k] * epsilon[k, t]
        # apply moving average filter for smoothing input sequence
        w_epsilon = self._moving_average_filter(xx=w_epsilon, window_size=10)
        # update control input sequence
        u += w_epsilon
        ## Step 3 Compute information theoretic weights for each sample ##
        t1 = curr_time()
        t_diff = (t1 - t0)
        step3_arr.append(t_diff)

        ## Step 4 Calculate optimal trajectory ##
        t0 = curr_time()
        optimal_traj = np.zeros((self.T, self.dim_x))
        if self.visualize_optimal_traj:
            x = x0
            for t in range(self.T):
                x = self._F(x, self._g(u[t-1]), self.delta_t)
                optimal_traj[t] = x
        # calculate sampled trajectories
        sampled_traj_list = np.zeros((self.K, self.T, self.dim_x))
        sorted_idx = np.argsort(S) # sort samples by state cost, 0th is the best sample
        if self.visualze_sampled_trajs:
            for k in sorted_idx:
                x = x0
                for t in range(self.T):
                    x = self._F(x, self._g(v[k, t-1]), self.delta_t)
                    sampled_traj_list[k, t] = x
        # update previous control input sequence (shift 1 step to the left)
        self.u_prev[:-1] = u[1:]
        self.u_prev[-1] = u[-1]
        t1 = curr_time()
        t_diff = (t1 - t0)
        step4_arr.append(t_diff)
        ## Step 4 Calculate optimal trajectory ##

        # print(f"Step 1 avg: {np.mean(step1_arr):.3f} ms")
        # print(f"Step 2 avg: {np.mean(step2_arr):.3f} ms")
        # print(f"Step 3 avg: {np.mean(step3_arr):.3f} ms")
        # print(f"Step 4 avg: {np.mean(step4_arr):.3f} ms")
        # print()

        # return optimal control input and input sequence
        return u[0], u, optimal_traj, sampled_traj_list, self.flag_done

    def _calc_epsilon(self, sigma: np.ndarray, size_sample: int, size_time_step: int, size_dim_u: int) -> np.ndarray:
        """Sample epsilon_(t) in Step 1 for each of the control dimension."""
        # check if sigma row size == sigma col size == size_dim_u and size_dim_u > 0
        if sigma.shape[0] != sigma.shape[1] or sigma.shape[0] != size_dim_u or size_dim_u < 1:
            print("[ERROR] sigma / covariance matrix must be a square matrix with the size of size_dim_u.")
            raise ValueError
        # sample epsilon
        mu = np.zeros((size_dim_u)) # set average as a zero vector
        # For each of the control 
        epsilon = np.random.multivariate_normal(mu, sigma, (size_sample, size_time_step)) 
        return epsilon

    def _g(self, v: np.ndarray) -> float:
        """Clamp input signals. This is a hard-constraint."""
        if len(v) > 2:
            raise ValueError("Error: More than two inputs are not supported yet.")  # noqa: EM101
        # limit control inputs to the left and right wheel velocities
        v[0] = np.clip(v[0], -self.abs_vb_val, self.abs_vb_val) 
        v[1] = np.clip(v[1], -self.abs_omega_val, self.abs_omega_val) 
        #v[1] = np.clip(v[1], 0, self.abs_omega_val) 
        return v

    def _c(self, x_t: np.ndarray) -> float:
        """Calculate stage cost / running cost."""
        # parse x_t
        x, y, yaw = x_t
        yaw = ((yaw + 2.0*np.pi) % (2.0*np.pi)) # normalize theta to [0, 2*pi]
        v = self.vehicle_model.curr_body_vel
        # calculate stage cost
        _, ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        stage_cost = self.stage_cost_weight[0]*(x-ref_x)**2 + self.stage_cost_weight[1]*(y-ref_y)**2 + \
                     self.stage_cost_weight[2]*(yaw-ref_yaw)**2 + self.stage_cost_weight[3]*(v-ref_v)**2
        # Add penalty for collision with obstacles
        stage_cost += self._is_collided(x_t) * 1.0e10
        return stage_cost

    def _cost_jit(self, x_t: np.ndarray) -> float:
        """Calculate state cost using JIT-compiled function."""
        ref_arr = np.zeros(4)
        x, y, _ = x_t
        _, ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        ref_arr = np.array([ref_x, ref_y, ref_yaw, ref_v])
        v = self.vehicle_model.curr_body_vel
        # calculate stage cost
        stage_cost = calculate_stage_cost(x_t=x_t,ref_point=ref_arr,
                                       weights=self.stage_cost_weight,
                                       vehicle_vel=v,
                                       vehicle_params=self.vehicle_params,
                                       obstacle_circles=self.obstacle_circles)
        return stage_cost

    def _phi(self, x_T: np.ndarray) -> float:
        """Calculate terminal cost."""
        # parse x_T
        x, y, yaw = x_T # In diffdrive, only three states
        yaw = ((yaw + 2.0*np.pi) % (2.0*np.pi)) # normalize theta to [0, 2*pi]
        v = self.vehicle_model.curr_body_vel
        # calculate terminal cost
        _, ref_x, ref_y, ref_yaw, ref_v = self._get_nearest_waypoint(x, y)
        
        terminal_cost = self.terminal_cost_weight[0]*(x-ref_x)**2 + self.terminal_cost_weight[1]*(y-ref_y)**2 + \
                        self.terminal_cost_weight[2]*(yaw-ref_yaw)**2 + self.terminal_cost_weight[3]*(v-ref_v)**2
        
        # add penalty for collision with obstacles
        terminal_cost += self._is_collided(x_T) * 1.0e10
        return terminal_cost

    def _is_collided(self,  x_t: np.ndarray) -> bool:
        """Check if the vehicle is collided with obstacles."""
        # vehicle shape parameters
        vw, vl = self.vehicle_w, self.vehicle_l
        safety_margin_rate = self.collision_safety_margin_rate
        vw, vl = vw*safety_margin_rate, vl*safety_margin_rate
        # get current states
        x, y, yaw = x_t
        # key points for collision check
        vehicle_shape_x = [-0.5*vl, -0.5*vl, 0.0, +0.5*vl, +0.5*vl, +0.5*vl, 0.0, -0.5*vl, -0.5*vl]
        vehicle_shape_y = [0.0, +0.5*vw, +0.5*vw, +0.5*vw, 0.0, -0.5*vw, -0.5*vw, -0.5*vw, 0.0]
        rotated_vehicle_shape_x, rotated_vehicle_shape_y = \
            self._affine_transform(vehicle_shape_x, vehicle_shape_y, yaw, [x, y]) # make the vehicle be at the center of the figure
        # check if the key points are inside the obstacles
        for obs in self.obstacle_circles: # for each circular obstacles
            obs_x, obs_y, obs_r = obs # [m] x, y, radius
            for p in range(len(rotated_vehicle_shape_x)):
                if (rotated_vehicle_shape_x[p]-obs_x)**2 + (rotated_vehicle_shape_y[p]-obs_y)**2 < obs_r**2:
                    return 1.0 # collided
        return 0.0 # not collided

    def _affine_transform(self, xlist: list, ylist: list, angle: float, translation: list=[0.0, 0.0]) -> Tuple[list, list]:
        """Rotate shape and return location on the x-y plane."""
        transformed_x = []
        transformed_y = []
        if len(xlist) != len(ylist):
            print("[ERROR] xlist and ylist must have the same size.")
            raise AttributeError
        for i, xval in enumerate(xlist):
            transformed_x.append((xlist[i])*np.cos(angle)-(ylist[i])*np.sin(angle)+translation[0])
            transformed_y.append((xlist[i])*np.sin(angle)+(ylist[i])*np.cos(angle)+translation[1])
        transformed_x.append(transformed_x[0])
        transformed_y.append(transformed_y[0])
        return transformed_x, transformed_y
    
    def _get_nearest_waypoint(self, x: float, y: float, update_prev_idx: bool = False):
        """Search the closest waypoint to the vehicle on the reference path."""
        SEARCH_IDX_LEN = 200 # [points] forward search range
        prev_idx = self.prev_waypoints_idx
        dx = [x - ref_x for ref_x in self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 0]]
        dy = [y - ref_y for ref_y in self.ref_path[prev_idx:(prev_idx + SEARCH_IDX_LEN), 1]]
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        min_d = min(d)
        nearest_idx = d.index(min_d) + prev_idx
        # get reference values of the nearest waypoint
        ref_x = self.ref_path[nearest_idx,0]
        ref_y = self.ref_path[nearest_idx,1]
        ref_yaw = self.ref_path[nearest_idx,2]
        ref_v = self.ref_path[nearest_idx,3]
        # update nearest waypoint index if necessary
        if update_prev_idx:
            self.prev_waypoints_idx = nearest_idx 

        return nearest_idx, ref_x, ref_y, ref_yaw, ref_v

    def _F(self, x_t: np.ndarray, v_t: np.ndarray, dt: float) -> np.ndarray:  # noqa: N802
        """
        Calculate next state of the vehicle.
        
        x_t: Previous state, v_t = inputs
        x_t_plus_1: Next state
        """
        # Initialize
        x_t_plus_1 = np.zeros(3)
        # Limits on input are already available within the vehicle model.
        # v_t = v_t[::-1] # experimental
        self.vehicle_model.forward_kinematics(x_t = x_t, u1=v_t[0], u2=v_t[1], delta_t=dt)
        x_t_plus_1 = self.vehicle_model.get_state()
        return x_t_plus_1

    def _compute_weights(self, S: np.ndarray) -> np.ndarray:
        """Compute weights for each sample."""
        # prepare buffer
        w = np.zeros((self.K))
        # calculate rho
        rho = S.min()
        # calculate eta
        eta = 0.0
        for k in range(self.K):
            eta += np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )
        # calculate weight
        for k in range(self.K):
            w[k] = (1.0 / eta) * np.exp( (-1.0/self.param_lambda) * (S[k]-rho) )
        return w

    def _moving_average_filter(self, xx: np.ndarray, window_size: int) -> np.ndarray:
        """
        Apply moving average filter for smoothing input sequence.

        Ref. https://zenn.dev/bluepost/articles/1b7b580ab54e95
        """
        b = np.ones(window_size)/window_size
        dim = xx.shape[1]
        xx_mean = np.zeros(xx.shape)

        for d in range(dim):
            xx_mean[:,d] = np.convolve(xx[:,d], b, mode="same")
            n_conv = math.ceil(window_size/2)
            xx_mean[0,d] *= window_size/n_conv
            for i in range(1, n_conv):
                xx_mean[i,d] *= window_size/(i+n_conv)
                xx_mean[-i,d] *= window_size/(i + n_conv - (window_size % 2)) 
        return xx_mean
