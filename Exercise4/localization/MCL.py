# -*- coding: utf-8 -*-
""" Module MCL
"""

__project__ = 'Exercise 4'
__module__  = 'MCL'
__author__  = 'Philipp Lohrer'
__date__    = '21.06.2015'

__version__ = '0.1'

# Standard library imports
import numpy as np
# Local imports



def MCL(particle_cloud, movement, landmark_positions, sensor_data):

    # 0. Set up empty particle_cloud
    particles_resampled = []

    # 1. Move particles
    particle_cloud.move_particles(movement)

    # 2. Calculate weight of each particle and save it as list
    particle_cloud.weight_particles(landmark_positions=landmark_positions, sensor_data=sensor_data)
    weights = particle_cloud.get_weight_particles()

    # 3. Resampling
    for i in xrange(len(particle_cloud)):
        # 3.1 Pick one particle randomly via weighted choice
        particle = np.random.choice(particle_cloud, p=weights)
        # 3.2 Append particle to resampled particle_cloud
        particles_resampled.append(particle)

    # 4. Update particle_cloud
    particle_cloud.update(particles_resampled)

