# -*- coding: utf-8 -*-
""" Module MCL
"""

__project__ = 'Exercise 5'
__module__  = 'MCL'
__author__  = 'Philipp Lohrer'
__date__    = '16.07.2015'

__version__ = '1.0'

# Standard library imports
import matplotlib.pyplot as plt
# Local imports

class MCL():
    """ class description:

    """

    def __init__(self, particle_cloud, robot_loc=None, draw=False):
        """
        :param particle_cloud:      (ref) particle_cloud
        :param robot_loc:           (ref) robot_location, default=None
        :param draw:                default=False
        :return:
        """

        self.particle_cloud = particle_cloud
        self.robot_loc = robot_loc
        self.draw = draw

        if self.draw and self.robot_loc is not None:
            # Error functions: current, min, max
            self.e_x = [0 for i in xrange(50)]
            self.e_y = [0 for i in xrange(50)]
            self.e_theta = [0 for i in xrange(50)]
            self.e_x_min = float('inf')
            self.e_y_min = float('inf')
            self.e_theta_min = float('inf')
            self.e_x_max = 0
            self.e_y_max = 0
            self.e_theta_max = 0
            # Time
            self.time_step = self.robot_loc.get_time_step()
            self.T = [-self.time_step * i for i in xrange(50,0,-1)]

            # Pyplot: enable interactive (= non-blocking) mode
            plt.ion()
            # Pyplot: create 3 subplots for error functions
            fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(nrows=3, ncols=1)
            self.plt_axes = [self.ax1, self.ax2, self.ax3]
            # for axes in self.plt_axes:
            #     axes.relim()
            #     axes.autoscale_view(True,True,True)
            self.ax1.set_title('Error x_est - x_real')
            self.ax2.set_title('Error y_est - y_real')
            self.ax3.set_title('Error theta_est - theta_real')
            # Create plot-objects for continuous data updates
            self.plt_plot_e_x = None
            self.plt_plot_e_y = None
            self.plt_plot_e_theta = None

            # self.plt_plot_ax1_zero = None
            # self.plt_plot_ax2_zero = None
            # self.plt_plot_ax3_zero = None

            self.plt_plot_e_x_min = None
            self.plt_plot_e_y_min = None
            self.plt_plot_e_theta_min = None
            self.plt_plot_e_x_max = None
            self.plt_plot_e_y_max = None
            self.plt_plot_e_theta_max = None
            self.plt_plots = [self.plt_plot_e_x, self.plt_plot_e_y, self.plt_plot_e_theta]

    def init_plots(self):

        self.plt_plot_e_x = self.ax1.plot(self.T, self.e_x)
        self.plt_plot_e_y = self.ax2.plot(self.T, self.e_y)
        self.plt_plot_e_theta = self.ax3.plot(self.T, self.e_theta)

        # self.plt_plot_ax1_zero = self.ax1.plot((self.T[0], self.T[-1]), (0, 0), '--')
        # self.plt_plot_ax2_zero = self.ax2.plot((self.T[0], self.T[-1]), (0, 0), '--')
        # self.plt_plot_ax3_zero = self.ax3.plot((self.T[0], self.T[-1]), (0, 0), '--')

        self.plt_plot_e_x_min = self.ax1.plot((self.T[0], self.T[-1]), (self.e_x_min, self.e_x_min), 'g--')
        self.plt_plot_e_y_min = self.ax2.plot((self.T[0], self.T[-1]), (self.e_y_min, self.e_y_min), 'g--')
        self.plt_plot_e_theta_min = self.ax3.plot((self.T[0], self.T[-1]), (self.e_theta_min, self.e_theta_min), 'g--')
        self.plt_plot_e_x_max = self.ax1.plot((self.T[0], self.T[-1]), (self.e_x_max, self.e_x_max), 'r--')
        self.plt_plot_e_y_max = self.ax2.plot((self.T[0], self.T[-1]), (self.e_y_max, self.e_y_max), 'r--')
        self.plt_plot_e_theta_max = self.ax3.plot((self.T[0], self.T[-1]), (self.e_theta_max, self.e_theta_max), 'r--')

        # for i, plt_plot in enumerate(self.plt_plots):
        #     plt_plot = self.plt_axes[i].plot(self.T, self.e_fcns[i])

    def draw_plots(self):

        if not all([self.plt_plot_e_x, self.plt_plot_e_y, self.plt_plot_e_theta]):
            self.init_plots()
        else:
            self.plt_plot_e_x[0].set_data(self.T, self.e_x)
            self.plt_plot_e_y[0].set_data(self.T, self.e_y)
            self.plt_plot_e_theta[0].set_data(self.T, self.e_theta)

            # self.plt_plot_ax1_zero[0].set_data((self.T[0], self.T[-1]), (0, 0))
            # self.plt_plot_ax2_zero[0].set_data((self.T[0], self.T[-1]), (0, 0))
            # self.plt_plot_ax3_zero[0].set_data((self.T[0], self.T[-1]), (0, 0))

            self.plt_plot_e_x_min[0].set_data((self.T[0], self.T[-1]), (self.e_x_min, self.e_x_min))
            self.plt_plot_e_y_min[0].set_data((self.T[0], self.T[-1]), (self.e_y_min, self.e_y_min))
            self.plt_plot_e_theta_min[0].set_data((self.T[0], self.T[-1]), (self.e_theta_min, self.e_theta_min))
            self.plt_plot_e_x_max[0].set_data((self.T[0], self.T[-1]), (self.e_x_max, self.e_x_max))
            self.plt_plot_e_y_max[0].set_data((self.T[0], self.T[-1]), (self.e_y_max, self.e_y_max))
            self.plt_plot_e_theta_max[0].set_data((self.T[0], self.T[-1]), (self.e_theta_max, self.e_theta_max))

            for axes in self.plt_axes:
                axes.relim()
                axes.autoscale_view(True,True,True)
            plt.draw()
        plt.show()

    def mcl_landmark(self, movement, landmark_positions, sensor_data, debug=True):
        """
        :param movement:            [v, omega]
        :param landmark_positions:  list([x1, y1], [x2, y2], ..., [x_n, y_n])
        :param sensor_data:         [indexes, distances, angles]
        :param debug:               default=False
        :return:                    estimated position [x, y, theta]
        """

        # 1. Move particles
        self.particle_cloud.move_particles(movement)

        # 2. Calculate weight of each particle and save it as list
        self.particle_cloud.weight_particles(landmark_positions=landmark_positions, sensor_data=sensor_data, debug=False)

        if debug:
            for particle in self.particle_cloud:
                number = particle.get_number()
                p_x, p_y, p_theta = particle.get_pos()
                print 'Particle %d: x=%0.2f y=%0.2f theta=%0.2f' % (number, p_x, p_y, p_theta),
                print 'weight=%0.2f' % particle.get_weight()

        # 3. Resampling
        self.particle_cloud.resample(debug=debug)

        if debug:
            for particle in self.particle_cloud:
                number = particle.get_number()
                p_x, p_y, p_theta = particle.get_pos()
                print 'Particle %d: x=%0.2f y=%0.2f theta=%0.2f' % (number, p_x, p_y, p_theta)

            print '########################################################################'

        # 4. Get estimated robot location and return it
        est_robot_location = self.particle_cloud.get_est_location()

        # 5. Draw
        if self.draw and self.robot_loc is not None:
            # Update plots with current errors
            real_x, real_y, real_theta = self.robot_loc.get_robot_position(est=False)
            est_x, est_y, est_theta = est_robot_location
            # print 'Real robot location: x=%0.2f y=%0.2f theta=%0.2f' % (real_x, real_y, real_theta * 180.0 / pi)
            # print 'Estimated robot location: x=%0.2f y=%0.2f theta=%0.2f' % (est_x, est_y, est_theta * 180.0 / pi)
            # print '###############################################################################################'
            # Append new error and shift error_lists 1 element to the left
            self.e_x.append(real_x - est_x)
            self.e_y.append(real_y - est_y)
            self.e_theta.append(real_theta - est_theta)
            self.e_x = self.e_x[1:]
            self.e_y = self.e_y[1:]
            self.e_theta = self.e_theta[1:]
            t_old = self.T[len(self.T)-1] if self.T else 0
            self.T.append(self.time_step + t_old)
            self.T = self.T[1:]

            # Update upper/lower errors
            self.e_x_min = self.e_x[-1] if self.e_x[-1] < self.e_x_min else self.e_x_min
            self.e_y_min = self.e_y[-1] if self.e_y[-1] < self.e_y_min else self.e_y_min
            self.e_theta_min = self.e_theta[-1] if self.e_theta[-1] < self.e_theta_min else self.e_theta_min
            self.e_x_max = self.e_x[-1] if self.e_x[-1] > self.e_x_max else self.e_x_max
            self.e_y_max = self.e_y[-1] if self.e_y[-1] > self.e_y_max else self.e_y_max
            self.e_theta_max = self.e_theta[-1] if self.e_theta[-1] > self.e_theta_max else self.e_theta_max

            self.draw_plots()

        return est_robot_location

