"""
Module containing all animation functions.

A good chunk of this module is based on Mizuho Aoki's simple_mppi project found here: https://github.com/MizuhoAOKI/python_simple_mppi

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
from typing import Tuple
from matplotlib import patches
from matplotlib.animation import ArtistAnimation
from IPython import display
from scripts.diffdrive import DiffDriveVehicle
from scripts.utils import debug_lock
from matplotlib.lines import Line2D


class Animator:
    """Class to draw 2D animation frames."""

    def __init__(self, 
                 vehicle_model:DiffDriveVehicle,
                 abs_x_lim_max = 20.0,
                 abs_y_lim_max = 25.0):
        """
        Class constructor.
        
        Note 1: self.u1 = psi_dot_l and self.u2 = psi_dot_r
        """
        # Pointer to vehicle model
        self.vehicle_model = vehicle_model
        self.abs_vb_val = vehicle_model.abs_vb_val # [m/s]
        self.abs_omega_val = vehicle_model.abs_omega_val # [rad/s]
        self.ref_path = vehicle_model.ref_path # np.ndarray([x,y], [x,y],.....)
        self.obstacle_circles = vehicle_model.obstacle_circles
        self.frames = []
        
        # Visualization settings
        self.vehicle_l = vehicle_model.vehicle_length # Vehicle lenght[m]
        self.vehicle_w = vehicle_model.vehicle_width # Vehicle widht[m]
        self.view_x_lim_min, self.view_x_lim_max = -abs_x_lim_max, abs_x_lim_max
        self.view_y_lim_min, self.view_y_lim_max = -abs_y_lim_max, abs_y_lim_max
        
        # Prepare figure
        self.fig = plt.figure(figsize=(9,9))
        self.main_ax = plt.subplot2grid((3,4), (0,0), rowspan=3, colspan=3) # Main view window
        self.minimap_ax = plt.subplot2grid((3,4), (0,3))
        self.u1_ax = plt.subplot2grid((6,4), (2,3), rowspan=1)
        self.u2_ax = plt.subplot2grid((6,4), (3,3), rowspan=1)
        #self.u1_ax = plt.subplot2grid((3, 4), (1, 3), rowspan=1)  # Reduce the height by splitting the grid into more rows
        #self.u2_ax = plt.subplot2grid((3, 4), (2, 3), rowspan=2)  # Adjust u2_ax height accordingly

        # Graph layout settings
        ## main view
        self.main_ax.set_aspect('equal')
        self.main_ax.set_xlim(self.view_x_lim_min, self.view_x_lim_max)
        self.main_ax.set_ylim(self.view_y_lim_min, self.view_y_lim_max)
        self.main_ax.tick_params(labelbottom=False, labelleft=False, labelright=False, labeltop=False)
        self.main_ax.tick_params(bottom=False, left=False, right=False, top=False)
        ## mini map
        self.minimap_ax.set_aspect('equal')
        self.minimap_ax.axis('off')
        ## u(1) view
        self.u1_ax.set_title("Robot base: Linear Vel", fontsize="12")
        self.u1_ax.axis('off')
        ## u(2) view
        self.u2_ax.set_title("Robot base: Angular Vel", fontsize="12")
        self.u2_ax.axis('off')
        # apply tight layout
        self.fig.tight_layout()
    
    def draw_frame(self, in_arr: list) -> list:
        """
        Draw a frame of the animation, display it and push it to a global list.

        Based on Aoki's implementation modified for skid steering robots.
        """
        # self.out_arr = [vb_t, omega_t, psi_dot_arr, self.state.copy(), optimal_traj, sampled_traj_list]
        # Unpack variables
        vb_t = in_arr[0]
        omega_t = in_arr[1]
        psi_dot_l = in_arr[2][0]
        psi_dot_r = in_arr[2][1]
        x, y, yaw= in_arr[3].copy() # self.state.copy()
        optimal_traj = in_arr[4]
        sampled_traj_list = in_arr[5]
        
        ## Debug
        # print(f"psi_dot_l = {psi_dot_l}, psi_dot_r = {psi_dot_r}")
        # print(f"states x,y,theta: {x},{y},{yaw}")
        # print(f"Optimal trajectory points: {optimal_traj.size}")
        # print(f"Sampled trajectories from MPPI: {sampled_traj_list.size}")
        # Calculate vehicle speed

        ### main view ###
        # Draw vehicle's rectangular body
        vw, vl = self.vehicle_w, self.vehicle_l # vehicle width and vehicle height
        vehicle_shape_x = [-0.5*vl, -0.5*vl, +0.5*vl, +0.5*vl, -0.5*vl, -0.5*vl] # HARDCODED
        vehicle_shape_y = [0.0, +0.5*vw, +0.5*vw, -0.5*vw, -0.5*vw, 0.0] # HARDCODED
        rotated_vehicle_shape_x, rotated_vehicle_shape_y = \
            self._affine_transform(vehicle_shape_x, vehicle_shape_y, yaw, [0, 0]) # make the vehicle be at the center of the figure
        frame = self.main_ax.plot(rotated_vehicle_shape_x, rotated_vehicle_shape_y, color='black', linewidth=2.0, zorder=3)
        
        # Define wheel shapes (rectangles)
        ww, wl = 0.4, 0.7 #[m]
        wheel_shape_x = np.array([-0.5*wl, -0.5*wl, +0.5*wl, +0.5*wl, -0.5*wl, -0.5*wl])
        wheel_shape_y = np.array([0.0, +0.5*ww, +0.5*ww, -0.5*ww, -0.5*ww, 0.0])
        ## rear-left wheel
        wheel_shape_rl_x, wheel_shape_rl_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, 0.0, [-0.3*vl, 0.3*vw])
        wheel_rl_x, wheel_rl_y = \
            self._affine_transform(wheel_shape_rl_x, wheel_shape_rl_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_rl_x, wheel_rl_y, color='black', zorder=3)
        ## rear-right wheel
        wheel_shape_rr_x, wheel_shape_rr_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, 0.0, [-0.3*vl, -0.3*vw])
        wheel_rr_x, wheel_rr_y = \
            self._affine_transform(wheel_shape_rr_x, wheel_shape_rr_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_rr_x, wheel_rr_y, color='black', zorder=3)
        ## front-left wheel
        wheel_shape_fl_x, wheel_shape_fl_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, 0.0, [0.3*vl, 0.3*vw])
        wheel_fl_x, wheel_fl_y = \
            self._affine_transform(wheel_shape_fl_x, wheel_shape_fl_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_fl_x, wheel_fl_y, color='black', zorder=3)
        ## front-right wheel
        wheel_shape_fr_x, wheel_shape_fr_y = \
            self._affine_transform(wheel_shape_x, wheel_shape_y, 0.0, [0.3*vl, -0.3*vw])
        wheel_fr_x, wheel_fr_y = \
            self._affine_transform(wheel_shape_fr_x, wheel_shape_fr_y, yaw, [0, 0])
        frame += self.main_ax.fill(wheel_fr_x, wheel_fr_y, color='black', zorder=3)
        
        # Draw the vehicle center circle (point B on kinematic diagram)
        vehicle_center = patches.Circle([0, 0], radius=vw/20.0, fc='white', ec='black', linewidth=2.0, zorder=6)
        frame += [self.main_ax.add_artist(vehicle_center)]
        
        # Draw the reference path by shifting the X and Y coordinates of the reference path relative to robot's body
        ref_path_x = self.ref_path[:, 0] - np.full(self.ref_path.shape[0], x)
        ref_path_y = self.ref_path[:, 1] - np.full(self.ref_path.shape[0], y)
        frame += self.main_ax.plot(ref_path_x, ref_path_y, color='black', linestyle="dashed", linewidth=1.5)
        
        # Add a vertical red line at the reference trajectory's starting position (0, 0)
        # 12/06/24: Bad code
        # line = Line2D([-x, -x], [0, 1], color='red', linestyle='solid', linewidth=4.0, zorder=2)  # Fixed height from -10 to 10
        # frame += [self.main_ax.add_line(line)]

        # Draw the information text
        # text = f"linear velocity $V_b$ = {vb:>+6.1f} [m/s]".format(pos_e=x, head_e=np.rad2deg(yaw), v=vb)
        text = f"Robot base: $V_b$ = {vb_t:>+6.1f} [m/s]"
        frame += [self.main_ax.text(0.5, 0.09, text, ha='center', 
                                    transform=self.main_ax.transAxes, 
                                    fontsize=14, fontfamily='monospace')]
        # Draw the second line of information text
        text2 = f"Robot base: $\\omega_b$ = {omega_t:>+6.1f} [rad/s]"
        frame += [self.main_ax.text(0.5, 0.07, text2, ha='center', 
                                    transform=self.main_ax.transAxes, 
                                    fontsize=14, fontfamily='monospace')]
        # Draw u1 current value
        text3 = f"Left-Wheel Angular Velocity $\\dot{{\\psi}}_L$ = {psi_dot_l:>+6.1f} [rad/s]"
        frame += [self.main_ax.text(0.5, 0.04, text3, ha='center', 
                                    transform=self.main_ax.transAxes, 
                                    fontsize=14, fontfamily='monospace')]
        # Draw u2 current value
        text4 = f"Right-Wheel Angular Velocity $\\dot{{\\psi}}_R$ = {psi_dot_r:>+6.1f} [rad/s]"
        frame += [self.main_ax.text(0.5, 0.01, text4, ha='center', 
                                    transform=self.main_ax.transAxes, 
                                    fontsize=14, fontfamily='monospace')]

        # Draw the predicted optimal trajectory MPPI
        if optimal_traj.any():
            optimal_traj_x_offset = np.ravel(optimal_traj[:, 0]) - np.full(optimal_traj.shape[0], x)
            optimal_traj_y_offset = np.ravel(optimal_traj[:, 1]) - np.full(optimal_traj.shape[0], y)
            frame += self.main_ax.plot(optimal_traj_x_offset, optimal_traj_y_offset, color='#990099', 
                                       linestyle="solid", linewidth=2.0, zorder=5)
        ### Draw the sampled trajectories from mppi
        if sampled_traj_list.any():
            min_alpha_value = 0.25
            max_alpha_value = 0.35
            for idx, sampled_traj in enumerate(sampled_traj_list):
                # draw darker for better samples
                alpha_value = (1.0 - (idx+1)/len(sampled_traj_list)) * (max_alpha_value - min_alpha_value) + min_alpha_value
                sampled_traj_x_offset = np.ravel(sampled_traj[:, 0]) - np.full(sampled_traj.shape[0], x)
                sampled_traj_y_offset = np.ravel(sampled_traj[:, 1]) - np.full(sampled_traj.shape[0], y)
                frame += self.main_ax.plot(sampled_traj_x_offset, sampled_traj_y_offset, 
                                           color='gray', linestyle="solid", linewidth=0.2, zorder=4, alpha=alpha_value)

        ### Draw the circular obstacles in the main view
        for obs in self.obstacle_circles:
            obs_x, obs_y, obs_r = obs
            obs_circle = patches.Circle([obs_x-x, obs_y-y], radius=obs_r, fc='white', ec='black', linewidth=2.0, zorder=0)
            frame += [self.main_ax.add_artist(obs_circle)]

        ### mini map view ###
        frame += self.minimap_ax.plot(self.ref_path[:, 0], 
                                      self.ref_path[:,1], 
                                      color='black', 
                                      linestyle='dashed')
        rotated_vehicle_shape_x_minimap, rotated_vehicle_shape_y_minimap = \
            self._affine_transform(vehicle_shape_x, vehicle_shape_y, yaw, [x, y]) # make the vehicle be at the center of the figure
        frame += self.minimap_ax.plot(rotated_vehicle_shape_x_minimap, 
                                      rotated_vehicle_shape_y_minimap, 
                                      color='black', linewidth=2.0, zorder=3)
        frame += self.minimap_ax.fill(rotated_vehicle_shape_x_minimap, 
                                      rotated_vehicle_shape_y_minimap, 
                                      color='white', zorder=2)

        # draw the circular obstacles in the mini map view
        for obs in self.obstacle_circles:
            obs_x, obs_y, obs_r = obs
            obs_circle = patches.Circle([obs_x, obs_y], radius=obs_r, fc='white', ec='black', linewidth=2.0, zorder=0)
            frame += [self.minimap_ax.add_artist(obs_circle)]

        # Animation to show the direction and strenght of two wheel's input angular velocities
        ### control input view ###
        MAX_LINEAR_VEL = self.abs_vb_val  # Maximum linear velocity
        MAX_ANGULAR_VEL = self.abs_omega_val # Maximum angular velocity
        BAR_HEIGHT = 0.01  # Height of the bar

        ## u(1): Vb ##
        ## Draw the background bar (full range - inactive region)
        background_bar = self.u1_ax.barh(
            y=0, width=2 * MAX_LINEAR_VEL, height=BAR_HEIGHT,
            left=-MAX_LINEAR_VEL, color='0.8', edgecolor="1",
        )

        ## Draw the active bar (from the center to the left/right ==> reverse / forward)
        active_bar = self.u1_ax.barh(
            y=0, width=vb_t, height=BAR_HEIGHT,
            left=0, color='r', edgecolor="1",
        )

        # Add the bar components to the frame
        frame += background_bar
        frame += active_bar

        ## u(2): omega_b ##
        ## Draw the background bar (full range - inactive region)
        background_bar = self.u2_ax.barh(
            y=0, width=2 * MAX_ANGULAR_VEL, height=BAR_HEIGHT,
            left=-MAX_ANGULAR_VEL, color='0.8', edgecolor="1",
        )

        ## Draw the active bar (from the center to the left/right ==> counterclockwise/clockwise rotation)
        active_bar = self.u2_ax.barh(
            y=0, width=omega_t, height=BAR_HEIGHT,
            left=0, color='r', edgecolor="1",
        )

        # Add the bar components to the frame
        frame += background_bar
        frame += active_bar

        # Push frame to global frame list
        self.frames.append(frame)

    def _affine_transform(self, xlist: list, ylist: list, angle: float, translation: list=[0.0, 0.0]) -> Tuple[list, list]:
        """Rotate shape and return location on x-y plane."""
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

    def show_animation(self, interval_ms: int) -> None:
        """Show animation of the recorded frames."""
        ani = ArtistAnimation(self.fig, self.frames, interval=interval_ms) # blit=True
        html = display.HTML(ani.to_jshtml())
        display.display(html)
        plt.close()

    def save_animation(self, filename: str, interval: int, movie_writer: str="ffmpeg") -> None:
        """Save animation of the recorded frames (ffmpeg required)."""
        ani = ArtistAnimation(self.fig, self.frames, interval=interval)
        ani.save(filename, writer=movie_writer)