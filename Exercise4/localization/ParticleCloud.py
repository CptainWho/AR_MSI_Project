# -*- coding: utf-8 -*-
""" Module ParticleCloud
"""

__project__ = 'Exercise 4'
__module__  = 'ParticleCloud'
__author__  = 'Philipp Lohrer'
__date__    = '25.06.2015'

__version__ = '0.1'

# Standard library imports
from math import pi, sqrt, sin, cos
import random as rnd
# Local imports
from Exercise4.util import Calculations as Calc


class ParticleCloud:
    """ class description:

    """

    def __init__(self, world_ref, robot_ref, localization='landmark', draw=False):
        """
        :param world_ref: reference of World
        :param robot_ref: reference of Robot
        :return:
        """
        self.particles = []

        self.world_ref = world_ref
        self.robot_ref = robot_ref

        if localization == 'distance_field':
            self.localization = 'distance_field'
        else:
            self.localization = 'landmark'

        self.draw = draw

        # Robot parameters [v_max, omega_max, noise_d, noise_theta, noise_drift, time_step]
        self.motion_params = self.robot_ref.getMotionParams()

        self.sum_weight_particles = 0
        self.sum_weight_particles_normed = 0

    def __iter__(self):
        return iter(self.particles)

    def __len__(self):
        return len(self.particles)

    def __contains__(self, particle):
        return True if particle in self.particles else False

    def add_particle(self, x, y, theta, number=None):
        if number is None or number < len(self.particles):
            number = len(self.particles)
        particle = Particle(number, x, y, theta, self.world_ref)
        self.append(particle)

    def append(self, particle):
        self.particles.append(particle)
        # Draw Particle in world
        if self.draw:
                # Draw particle in world
                particle.draw()

    def remove(self, particle):
        self.particles.remove(particle)
        # Undraw particle in world
        if self.draw:
                # Undraw particle in world
                particle.undraw()

    def update(self, particles):
        """ Delete all elements in current particle_cloud and set up new particle_cloud
        :param particles: (list) particles
        :return:
        """

        self.particles = particles


    def create_particles(self, amount):
        """ Create and randomly place a given amount of particles in given world's boundaries
        :param amount:  (int) amount of particles to create
        :return:        -
        """

        size_world = self.world_ref.get_size()

        for n in xrange(int(amount)):
            # Random particle position
            p_x = round(rnd.random() * size_world[0], 2)
            p_y = round(rnd.random() * size_world[1], 2)
            p_theta = rnd.random() * 2 * pi
            if p_theta > pi:
                p_theta -= 2 * pi
            particle = Particle(len(self.particles), p_x, p_y, p_theta, self.world_ref)

            # Append particle to particle_cloud
            self.append(particle)

    def move_particles(self, motion):
        """ Move all particles
        :param motion:  [v, omega]
        :return:        -
        """

        for particle in self.particles:
            particle.move(motion, self.motion_params)
            if self.draw:
                # Redraw particle in world
                particle.undraw()
                particle.draw()

    def weight_particles(self, landmark_positions=None, sensor_data=None):
        """ Calculate weight for each particle
        :param landmark_positions:
        :param sensor_data:
        :return:
        """

        if self.localization == 'landmark' and landmark_positions is not None and sensor_data is not None:
            landmark_distances, landmark_angles = sensor_data
            for particle in self.particles:
                weight = particle.calculate_weight(landmark_positions, landmark_distances, landmark_angles)
                self.sum_weight_particles += weight

    def get_weight_particles(self):
        """ Returns the weight of all particles normed between [0...1]
        Sum of all weights is 1.0
        :return: (list) weights [w1, w2, ..., w_n]
        """

        weight_list = []
        best_weight = 0
        best_weight_index = 0

        for particle in self.particles:
            weight = particle.get_weight()
            # Scale weight
            weight /= self.sum_weight_particles
            # Change weight order
            weight = 1 - weight

            if weight > best_weight:
                best_weight = weight
                best_weight_index = len(weight_list)

            self.sum_weight_particles_normed += weight

            # Append weight to weight_list
            weight_list.append(weight)

        # Close gap to 1.0
        if self.sum_weight_particles_normed != 1:
            offset = 1 - self.sum_weight_particles_normed
            # Add offset to particle with highest weight
            weight_list[best_weight_index] += offset

    def shuffle_particles(self, n):
        """ Shuffle n particles
        :param n:   (int) amount of particles to shuffle
        :return:    -
        """

        pass


class Particle:
    """ class description:

    """

    def __init__(self, number, x, y, theta, world_ref):
        """ Initialize particle
        :param x:           coord
        :param y:           coord
        :param theta:       angle (orientation)
        :param world_ref:   world reference
        :return:
        """

        self.number = number
        self.x, self.y, self.theta = x, y, theta
        self.world_ref = world_ref

        self.particle_weight = 1

    def get_pos(self):
        return [self.x, self.y, self.theta]

    def set_pos(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta

    def get_theta(self):
        return self.theta

    def get_number(self):
        return self.number

    def get_weight(self):
        return self.particle_weight

    def set_weight(self, weight):
        self.particle_weight = weight

    def draw(self):
        self.world_ref.draw_particle(self, color='black', number=self.number)

    def undraw(self):
        self.world_ref.undraw_particle(self)

    def move(self, motion, motion_params):
        """ Move particle in a way similar to Robot.move() and save new particle position
        :param motion:          [v, omega]
        :param motion_params    [v_max, omega_max, noise_d, noise_theta, noise_drift, time_step]
        :return:        -
        """

        v = motion[0]
        omega = motion[1]
        v_max, omega_max, noise_d, noise_theta, noise_drift, time_step = motion_params

        # translational and rotational speed is limited:
        if omega > omega_max:
            omega = omega_max
        if omega < -omega_max:
            omega = -omega_max
        if v > v_max:
            v = v_max
        if v < -v_max:
            v = -v_max

        # Add noise to v:
        sigma_v_2 = (noise_d / time_step) * abs(v)
        v_noisy = v + rnd.gauss(0.0, sqrt(sigma_v_2))

        # Add noise to omega:
        sigma_omega_tr_2 = (noise_theta / time_step) * abs(omega)  # turning rate noise
        sigma_omega_drift_2 = (noise_drift / time_step) * abs(v)  # drift noise
        omega_noisy = omega + rnd.gauss(0.0, sqrt(sigma_omega_tr_2))
        omega_noisy += rnd.gauss(0.0, sqrt(sigma_omega_drift_2))

        # Move robot in the world (with noise):
        d_noisy = v_noisy * time_step
        d_theta_noisy = omega_noisy * time_step

        # Get current particle position
        x, y, theta = self.get_pos()
        dx = d_noisy * cos(theta + 0.5 * d_theta_noisy)
        dy = d_noisy * sin(theta + 0.5 * d_theta_noisy)
        theta_new = (theta + d_theta_noisy) % (2 * pi)

        # Update particle position
        self.set_pos(x+dx, y+dy, theta_new)

    def calculate_weight(self, landmark_positions, landmark_distances, landmark_angles, debug=False):
        """
        :param landmark_positions:  landmark positions [[x1, y1], [x2, y2], ..., [x_n, y_n]]
        :param landmark_distances:  measured distance from robot to landmark [dist1, dist2, ..., dist_n]
        :param landmark_angles:     measured angle from robot to landmark [theta1, theta2, ..., theta_n]
        :return:                    (float) particle_weight
        """

        if debug:
            print 'Particle %d:' % self.number
            print '\tp_theta = %0.2f' % (self.theta / pi * 180.0)

        for i in xrange(len(landmark_positions)):
            # Calculate estimated distance and angle from particle to given landmark
            est_dist_to_landmark = Calc.get_dist_from_point_to_point([self.x, self.y], landmark_positions[i])
            est_angle_to_landmark = Calc.get_angle_from_point_to_point([self.x, self.y], landmark_positions[i])
            rel_angle_to_landmark = Calc.diff(est_angle_to_landmark, self.theta)

            # Calculate particle_weight
            self.particle_weight = abs(self.particle_weight * (landmark_distances[i] - est_dist_to_landmark) * \
                                   Calc.add_angles(landmark_angles[i], -rel_angle_to_landmark))

            if debug:
                print '\test dist to landmark %d: %0.2f' % (i, est_dist_to_landmark),
                print '--> diff = %0.2f' % (landmark_distances[i] - est_dist_to_landmark)
                print '\test angle to landmark %d: %0.2f' % (i, rel_angle_to_landmark / pi * 180.0),
                print '--> diff = %0.2f' % (Calc.add_angles(landmark_angles[i], -rel_angle_to_landmark) / pi * 180)
                print '\t\t--> resulting weight: %0.2f' % self.particle_weight

        return self.particle_weight
