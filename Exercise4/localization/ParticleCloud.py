# -*- coding: utf-8 -*-
""" Module ParticleCloud
"""

__project__ = 'Exercise 4'
__module__  = 'ParticleCloud'
__author__  = 'Philipp Lohrer'
__date__    = '014.07.2015'

__version__ = '0.1'

# Standard library imports
from math import pi, sqrt, sin, cos
import bisect
import random
# Local imports
from Exercise4.util import Calculations as Calc


class ParticleCloud:
    """ class description:

    """

    def __init__(self, world_ref, robot_ref, localization='landmark', draw=None):
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

        self.draw_particle = False
        self.draw_number = False
        self.draw_estimation = False
        if draw == 'particle':
            self.draw_particle = True
        elif draw == 'particle_number':
            self.draw_particle = True
            self.draw_number = True
        elif draw == 'estimation':
            self.draw_estimation = True
        elif draw == 'particle_estimation':
            self.draw_particle = True
            self.draw_estimation = True

        # Robot parameters [v_max, omega_max, noise_d, noise_theta, noise_drift, time_step]
        self.motion_params = self.robot_ref.getMotionParams()

        self.sum_weight_particles = 0
        self.sum_weight_particles_normed = 0

        # Estimated robot location
        self.est_robot_pos = None
        self.theta_est_old = 0

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
        if self.draw_particle:
            # Draw particle in world
            particle.draw(draw_number=self.draw_number)

    def remove(self, particle):
        self.particles.remove(particle)
        # Undraw particle in world
        if self.draw_particle:
                # Undraw particle in world
                particle.undraw()

    def update(self, particles):
        """ Delete all elements in current particle_cloud and set up new particle_cloud
        :param particles: (list) particles
        :return:
        """
        if self.draw_particle:
            for particle in self.particles:
                particle.undraw()

        self.particles = particles

        if self.draw_particle:
            for particle in self.particles:
                particle.draw(draw_number=self.draw_number)

    def create_particles(self, amount, position=None):
        """ Create and randomly place a given amount of particles in given world's boundaries
        If given position is not None, place particles at give position.
        :param amount:      (int) amount of particles to create
        :param position:    [x, y, theta]
        :return:        -
        """

        size_world = self.world_ref.get_size()

        for n in xrange(int(amount)):
            if position is None:
                # Random particle position
                p_x = round(random.random() * size_world[0], 2)
                p_y = round(random.random() * size_world[1], 2)
                p_theta = random.random() * 2 * pi
                if p_theta > pi:
                    p_theta -= 2 * pi
            else:
                # Place particle at given position
                p_x, p_y, p_theta = position
            particle = Particle(len(self.particles), p_x, p_y, p_theta, self.world_ref)

            # Append particle to particle_cloud
            self.append(particle)

    def move_particles(self, motion):
        """ Move all particles
        :param motion:  [v, omega]
        :return:        -
        """

        # counter = 0

        for particle in self.particles:
            # rnd = True if counter % 50 == 0 else False
            # counter += 1
            particle.move(motion, self.motion_params, random_movement=False)
            if self.draw_particle:
                # Redraw particle in world
                particle.undraw()
                particle.draw(draw_number=self.draw_number)

    def weight_particles(self, landmark_positions=None, sensor_data=None, laser_sensor_data=None, likelihoodfield=None, debug=False):
        """ Calculate weight for each particle
        :param landmark_positions:
        :param sensor_data:
        :return:
        """

        self.sum_weight_particles = 0

        if self.localization == 'landmark' and landmark_positions is not None and sensor_data is not None:
            landmark_numbers, landmark_distances, landmark_angles = sensor_data
            for particle in self.particles:
                weight = particle.calculate_weight_landmarks(landmark_positions, landmark_distances, landmark_angles, debug=debug)
                self.sum_weight_particles += weight
        elif self.localization == 'distance_field' and laser_sensor_data is not None and laser_sensor_data is not None:
            for particle in self.particles:
                weight = particle.calculate_weight_beams(laser_sensor_data, likelihoodfield)
                self.sum_weight_particles += weight

    def get_weight_particles(self):
        """ Returns the weight of all particles normed between [0...1]
        Sum of all weights is 1.0
        :return: (list) weights [w1, w2, ..., w_n]
        """

        weight_list = []
        # best_weight = 0
        # best_weight_index = 0
        self.sum_weight_particles_normed = 0

        for particle in self.particles:
            weight = particle.get_weight()
            # Change weight order
            weight = (1 - weight)

            weight_list.append(weight)

        return weight_list

    def resample(self, debug=False):
        """
        :param debug:   default=False
        :return:
        """

        # 1. Set up empty particle_cloud
        particles_resampled = []

        # 2. Calculate weight for each particle
        weights = self.get_weight_particles()
        if debug:
            for i, weight in enumerate(weights):
                print '\tWeight particle %d: %0.5f' % (i, weight)

        # 3. Initialize WeightedRandomGenerator with weights
        wrg = WeightedRandomGenerator(weights)

        indexes = []
        # 4. Resample particles
        for i in xrange(len(self)):
            # 4.1 Pick one particle randomly via weighted choice
            # particle = np.random.choice(self.particles, p=weights)
            index = wrg()
            indexes.append(index)
            particle = self.particles[index]
            if debug:
                print '\tResampling: selected particle: %d' % particle.get_number()
            # 4.2 Create new particle with parameters of selected particle
            p_x, p_y, p_theta = particle.get_pos()
            particle_new = Particle(i, p_x, p_y, p_theta, self.world_ref)
            # 4.3 Append new particle to resampled particle_cloud
            particles_resampled.append(particle_new)

        # 4. Update particles
        self.update(particles_resampled)

    def get_est_location(self):
        """ Calculate estimated robot location via mean values of particle locations
        :return:    estimated location of robot due to particle locations
        """

        x_est, y_est, theta_est = [0, 0, 0]
        list_angle = []

        for particle in self.particles:
            p_x, p_y, p_theta = particle.get_pos()
            x_est += p_x
            y_est += p_y
            list_angle.append(p_theta)

        # Calculate average angle
        theta_est = Calc.get_average_angle(list_angle)
        # If average_angle is None (= not defined), use old theta_est
        if theta_est is None:
            theta_est = self.theta_est_old

        x_est /= float(len(self.particles))
        y_est /= float(len(self.particles))

        if self.draw_estimation:
            self.world_ref.draw_estimated_position(x_est, y_est, theta_est)

        return [x_est, y_est, theta_est]


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

        # noises
        self.likelihood_dist_noise = 0.01

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

    def draw(self, draw_number=False):
        number = None
        if draw_number:
            number = self.get_number()
        self.world_ref.draw_particle(self, color='black', number=number)

    def undraw(self):
        self.world_ref.undraw_particle(self)

    def move(self, motion, motion_params, random_movement=False):
        """ Move particle in a way similar to Robot.move() and save new particle position
        :param motion:          [v, omega]
        :param motion_params    [v_max, omega_max, noise_d, noise_theta, noise_drift, time_step]
        :param random_movement: default=False
        :return:        -
        """

        v_max, omega_max, noise_d, noise_theta, noise_drift, time_step = motion_params

        # Add gain to noise
        noise_d *= 2.0
        noise_theta *= 2.0
        noise_drift *= 2.0

        if random_movement:
            v_noisy = random.random() * v_max
            omega_noisy = random.random() * omega_max
            if random.random() < 0.5:
                omega_noisy *= -1.0
        else:
            v = motion[0]
            omega = motion[1]

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
            v_noisy = v + random.gauss(0.0, sqrt(sigma_v_2))

            # Add noise to omega:
            sigma_omega_tr_2 = (noise_theta / time_step) * abs(omega)  # turning rate noise
            sigma_omega_drift_2 = (noise_drift / time_step) * abs(v)  # drift noise
            omega_noisy = omega + random.gauss(0.0, sqrt(sigma_omega_tr_2))
            omega_noisy += random.gauss(0.0, sqrt(sigma_omega_drift_2))

        # Move particle in the world (with noise):
        d_noisy = v_noisy * time_step
        d_theta_noisy = omega_noisy * time_step

        # Get current particle position
        x, y, theta = self.get_pos()
        dx = d_noisy * cos(theta + 0.5 * d_theta_noisy)
        dy = d_noisy * sin(theta + 0.5 * d_theta_noisy)
        theta_new = (theta + d_theta_noisy) % (2 * pi)

        ### Test
        # rnd_x = random.random() * 0.05
        # dx += rnd_x if random.random() > 0.5 else (- 1.0 * rnd_x)
        # rnd_y = random.random() * 0.01
        # dy += rnd_y if random.random() > 0.5 else (- 1.0 * rnd_y)

        # Update particle position
        self.set_pos(x+dx, y+dy, theta_new)

        # self.world_ref.move_particle(self, d_noisy, d_theta_noisy, time_step)

    def calculate_weight_landmarks(self, landmark_positions, landmark_distances, landmark_angles, debug=False):
        """
        :param landmark_positions:  landmark positions [[x1, y1], [x2, y2], ..., [x_n, y_n]]
        :param landmark_distances:  measured distance from robot to landmark [dist1, dist2, ..., dist_n]
        :param landmark_angles:     measured angle from robot to landmark [theta1, theta2, ..., theta_n]
        :return:                    (float) particle_weight
        """

        # Reset weight
        self.particle_weight = 1

        if debug:
            print 'Particle %d:' % self.number
            print '\tp_theta = %0.2f' % (self.theta / pi * 180.0)

        for i in xrange(len(landmark_positions)):
            # Calculate estimated distance and angle from particle to given landmark
            est_dist_to_landmark = Calc.get_dist_from_point_to_point([self.x, self.y], landmark_positions[i])
            est_angle_to_landmark = Calc.get_angle_from_point_to_point([self.x, self.y], landmark_positions[i])
            rel_angle_to_landmark = Calc.diff(self.theta, est_angle_to_landmark)

            # Calculate particle_weight
            dist = abs(landmark_distances[i] - est_dist_to_landmark)
            dist += random.gauss(0.0, sqrt(0.01 ** 2 * dist))
            d_theta = abs(Calc.add_angles(landmark_angles[i], -rel_angle_to_landmark))
            d_theta += random.gauss(0.0, sqrt(0.01 ** 2 * d_theta))
            self.particle_weight = self.particle_weight * dist * d_theta

            if debug:
                print '\test dist to landmark %d: %0.2f' % (i, est_dist_to_landmark),
                print '--> diff = %0.2f' % (landmark_distances[i] - est_dist_to_landmark)
                print '\test angle to landmark %d: %0.2f' % (i, rel_angle_to_landmark / pi * 180.0),
                print '--> diff = %0.2f' % (Calc.add_angles(landmark_angles[i], -rel_angle_to_landmark) / pi * 180)
                print '\t\t--> resulting weight: %0.8f' % self.particle_weight

        return self.particle_weight

    def calculate_weight_beams(self, laser_sensor_data, likelihoodfield):
        """

        :param laser_sensor_data:   data from the laser sensors (distance and angle)
        :param likelihoodfield:     the brushfire grid that is used for the distance to next wall
        :return:                    calculated weight in reverse (0 => highest weight)
        """
        p=1
        # go through all data points
        for [distance, angle] in laser_sensor_data:
            # obstacle point from particle position
            lokal_obstacle_point = Calc.polar_2_cartesian(distance, angle)
            x_z = lokal_obstacle_point[0] + self.x
            y_z = lokal_obstacle_point[1] + self.y
            obstacle_point = [x_z, y_z]
            # get distance value from likelihoodfield
            dist_val = 1.0 - likelihoodfield.getValue(obstacle_point)
            # apply gauss curve
            sigma2 = self.likelihood_dist_noise ** 2 * dist_val
            dist_val += random.gauss(0.0, sqrt(sigma2))
            p *= dist_val

        self.particle_weight = 1 - p
        return self.particle_weight


class WeightedRandomGenerator(object):
    """
    source: http://eli.thegreenplace.net/2010/01/22/weighted-random-generation-in-python
    """

    def __init__(self, weights):
        self.totals = []
        running_total = 0

        for w in weights:
            running_total += w
            self.totals.append(running_total)

    def next(self):
        rnd = random.random() * self.totals[-1]
        index = bisect.bisect_right(self.totals, rnd)
        return index  # len(self.totals) - 1 if index >= len(self.totals) else index

    def __call__(self):
        return self.next()




