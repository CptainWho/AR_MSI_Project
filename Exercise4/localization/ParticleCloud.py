# -*- coding: utf-8 -*-
""" Module ParticleCloud
"""

__project__ = 'Exercise 4'
__module__  = 'ParticleCloud'
__author__  = 'Philipp Lohrer'
__date__    = '21.06.2015'

__version__ = '0.1'

# Standard library imports
from math import pi
import random as rnd
# Local imports
from Exercise4.util import Calculations as Calc


class ParticleCloud():
    """ class description:

    """

    def __init__(self, world_ref, particles=None):
        if particles is not None:
            self.particles = particles
        else:
            self.particles = []

        self.world_ref = world_ref

    def __iter__(self):
        return iter(self.particles)

    def __len__(self):
        return len(self.particles)

    def __contains__(self, particle):
        return True if particle in self.particles else False

    def append(self, particle):
        self.particles.append(particle)
        # Draw Particle in world
        p_number = particle.get_number()
        self.world_ref.draw_particle(particle, number=p_number)

    def remove(self, particles):
        for particle in particles:
            self.particles.remove(particle)
            # Undraw particle in world
            self.world_ref.undraw_particle(particle)

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


class Particle():
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

    def __call__(self, x, y, theta, color='black', number=None):
        """ Change position of this particle
        :param x:       x-coord
        :param y:       y-coord
        :param theta:   angle [-pi ... pi]
        :param color    (string) color, default=black
        :param number:  (int) number, default=None
        :return:    -
        """

        self.world_ref.undraw_particle(self)
        self.x, self.y, self.theta = x, y, theta
        self.world_ref.draw_particle(self, color, number)

    def get_pos(self):
        """
        :return: particle position [x, y]
        """
        return [self.x, self.y]

    def get_theta(self):
        """
        :return: particle orientation
        """

        return self.theta

    def get_number(self):
        """
        :return: (int) particle number
        """

        return self.number

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
            self.particle_weight = self.particle_weight * (landmark_distances[i] - est_dist_to_landmark) * \
                                   Calc.add_angles(landmark_angles[i], -rel_angle_to_landmark)

            if debug:
                print '\test dist to landmark %d: %0.2f' % (i, est_dist_to_landmark),
                print '--> diff = %0.2f' % (landmark_distances[i] - est_dist_to_landmark)
                print '\test angle to landmark %d: %0.2f' % (i, rel_angle_to_landmark / pi * 180.0),
                print '--> diff = %0.2f' % (Calc.add_angles(landmark_angles[i], -rel_angle_to_landmark) / pi * 180)
                print '\t--> resulting weight: %0.2f' % self.particle_weight

        return self.particle_weight
