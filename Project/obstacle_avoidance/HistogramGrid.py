# -*- coding: utf-8 -*-
""" Module Description:
Histogram Grid for obstacle avoidance.
"""

__project__ = 'Project'
__module__ = 'HistogramGrid'
__author__ = 'Philipp Lohrer'
__email__ = 'plohrer@htwg-konstanz.de'
__date__ = '16.07.2015'

# Changelog:
# 13.07.2015 (Ecki):    ?
# 15.07.2015 (Phil):    implemented path 'edge' and 'middle'
# 16.07.2015 (Phil):    added mode 'simple' for simple obstacle avoidance without past values
#                       implemented parallel plotting while in simple-mode

__version__ = '1.1'

# Imports
#################################################################
# Standard library imports
import numpy as np
from math import *
from numbers import Number
import matplotlib.pyplot as plt
# Local imports
from Project.util import Calculations as Calc
#################################################################


class HistogramGrid:
    """ Class description:
    Creates an empty histogram grid which can be filled with values for obstacle avoidance
    For this purpose the minimums of the polar histogram can be retrieved and a route to
    avoid an detected obstacle can be calculated.
    Furthermore the histogram and the histogram grid can be dynamically displayed via matplotlib.
    """

    def __init__(self, width=5, height=5, cell_size=0.1, hist_resolution=10.0, hist_threshold=2.0, plot_grid=False):
        """ Initialize grid
        :param width: int
        :param height: int
        :param cell_size: default 0.1
        :param hist_resolution: angle for each histogram sector, default: 10°
        :param hist_threshold: threshold for minimum-search, default: 50.0 occupancy
        :return: -
        """

        self.x_size = int(width/cell_size)
        self.y_size = int(height/cell_size)
        # If x/y_size is even, add 1
        if self.x_size % 2 == 0:
            self.x_size += 1
        if self.y_size % 2 == 0:
            self.y_size += 1
        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int)
        self.width = width
        self.height = height
        self.cell_size = float(cell_size)

        self.histogram = None
        self.hist_resolution = hist_resolution
        self.hist_threshold = hist_threshold

        if plot_grid:
            # Pyplot: enable interactive (= non-blocking) mode
            plt.ion()
            # Pyplot: create 2 subplots for histogram grid and polar histogram
            fig, (self.ax1, self.ax2) = plt.subplots(nrows=2, ncols=1)
            # Create plt-obj of matshow for faster data updating
            self.plt_grid = None
            # Initialize histogram with empty data
            self.plt_hist = None

        # values for closed loop
        self.kp_v = 2.0  # 1.0  # 0.7
        self.kp_omega = 2.5  # 2.1  # ^= 120°/s^2
        #self.kp_v = 0.7
        #self.kp_omega = 0.7

        self.min_valley_angle_old = 0

    def _init_grid(self, axis):
        """ Initialize HistogramGrid
        :param axis: axis for plotting
        :return: pyplot matshow obj
        """

        plt_grid = axis.matshow(self.grid)

        return plt_grid

    def _init_hist(self, axis):
        """ Initialize histogram
        :param axis: axis for plotting
        :return: pyplot bar obj
        """
        # get sector angles and occupancy
        sector_angles, sector_occupancy = self.histogram
        # Extract sector angles as strings
        text_sector_angles = [np.array_str(angle, precision=2) for angle in sector_angles[0]]
        # Width of the bars
        width = self.hist_resolution / 2.0
        # Set up bars
        plt_hist = axis.bar(sector_angles[0], sector_occupancy[0], width=width)
        # add text for labels, title and axis ticks
        axis.set_xlabel('sector angles')
        axis.set_ylabel('occupancy value')
        axis.set_title('Histogram')
        axis.set_xticks(sector_angles[0])
        axis.set_xticklabels(text_sector_angles,  rotation='vertical')

        return plt_hist

    def avoid_obstacle(self, robot_loc, target_point, sensor_data, max_sense_dist, mode='simple',  debug=False):
        """ Calculates nearest way in regard of given target point around detected obstacles
        :param robot_loc:       Robot_Location reference
        :param target_point:    [x,y]
        :param sensor_data:     [angles, distances]
        :param mode:            'simple', 'middle' or 'edge' choose path in min_valley, default=simple
        :return:                speed, angular velocity ([v,omega])
        """

        v_max = robot_loc.get_max_robot_speed()
        omega_max = robot_loc.get_max_robot_omega()
        valley_edge_offset = 20.0 / 180.0 * pi  # Offset to hold to an edge of a valley
        closest_angle = None

        # Get direction to target_point [-pi...pi]
        target_angle = Calc.get_angle_from_point_to_point(robot_loc.get_robot_point(), target_point)

        # Create polar histogram and transform sector_angles from range [0..2pi] to [-pi...pi]
        sector_angles, sector_distances = sensor_data
        # sector_angles = sector_angles / 180.0 * pi
        sector_angles[sector_angles > pi] -= 2 * pi

        # Scale distances
        sector_distances[np.isnan(sector_distances)] = max_sense_dist
        sector_occupancy = max_sense_dist - sector_distances

        # Check if polar histogram is empty. If so, no obstacle is around -> use target_angle directly
        if not np.any(sector_occupancy):
            # Set omega proportional to diff(target_angle, closest_angle)
            omega = self.kp_omega * Calc.diff(robot_loc.get_robot_angle(), target_angle)

            # Set speed v to v_max
            v = v_max
        else:
            # Polar histogram contains at least 1 occupancy value -> search closest angle

            # 1. Search for occupancy values below threshold and return indexes
            min_indexes = np.where(sector_occupancy < self.hist_threshold)[0]

            # 2. Group all found min_indexes to valleys (check for neighborhood indexes)
            min_valleys = []
            valley_temp = np.array([min_indexes[0]])
            for i in xrange(1, np.size(min_indexes)):
                if min_indexes[i-1] + 1 == min_indexes[i]:
                    # Direct neighbor found -> add index to valley_temp
                    valley_temp = np.hstack((valley_temp, min_indexes[i]))
                else:
                    # Next index is not direct neighbor -> add valley_temp to min_valleys and start a new valley_temp
                    if np.size(valley_temp) > 1:
                        min_valleys.append(valley_temp)
                    valley_temp = min_indexes[i]
            if np.size(valley_temp) > 1:
                min_valleys.append(valley_temp)
            # 2.1 Check if min_valleys contains only one min_valley with all sectors
            if np.size(min_valleys) == np.size(sector_angles):
                pass
            # 2.2 Check if first (0°) and last sector (360° - hist_resolution) are neighbors.
            #     If so concat the first and the last min_valley
            elif min_valleys[0][0] == 0 and min_valleys[-1][-1] == np.size(sector_angles) - 1:
                min_valleys = [np.hstack((min_valleys[-1], min_valleys[0]))] + min_valleys[1:-1]
            # 2.3 Check if last sector was a single min_index (not treated as min_valley and discarded).
            #     If so and if 0° in first min_valley, append last sector to this min_valley
            elif min_valleys[0][0] == 0 and min_indexes[-1] == np.size(sector_angles) - 1:
                min_valleys = [np.hstack((min_indexes[-1], min_valleys[0]))] + min_valleys[1:]
            # 2.3 Check if first sector was a single min_index (not treated as min_valley and discarded).
            #     If so and if last sector in last min_valley, append first sector to this min_valley
            elif min_indexes[0] == 0 and min_valleys[-1][-1] == np.size(sector_angles) - 1:
                min_valleys = [np.hstack((min_valleys[-1], min_indexes[0]))] + min_valleys[0:-1]

            # Choose either the simple, middle or an edge of the best min_valley
            if mode == 'simple' or mode == 'middle':
                # 3. Check if target_angle is in one of the min_valleys, if so set target_angle directly as closest_angle
                for min_valley in min_valleys:
                    min_valley_angles = sector_angles[min_valley]
                    try:
                        if Calc.angle_in_range(min_valley_angles[0], min_valley_angles[-1], target_angle, offset=valley_edge_offset):
                            closest_angle = target_angle
                            closest_min_valley = min_valley
                            break
                    except IndexError:
                        continue
                # 4. If target_angle is not in one of the min_valleys -> search min_valley_edge closest to target_angle
                if closest_angle is None:
                    # Calculate the middle of each min_valley and min_valley_weight according to valley size (quadratic)
                    min_valley_angles = np.empty(0, dtype=np.float)
                    min_valley_weights = np.empty(0, dtype=np.float)
                    for min_valley in min_valleys:
                        min_valley_angle = Calc.get_average_angle(sector_angles[min_valley])
                        if min_valley_angle is None:
                            # Use old angle
                            min_valley_angle = self.min_valley_angle_old
                        self.min_valley_angle_old = min_valley_angle
                        min_valley_angles = np.hstack((min_valley_angles, min_valley_angle))
                        min_valley_weights = np.hstack((min_valley_weights, np.size(min_valley) ** 2.0))

                    # Search angle closest to target_angle
                    closest_angle, angle_diffs = Calc.search_closest_angle(target_angle, min_valley_angles)

                    closest_min_valley = min_valleys[np.argwhere(min_valley_angles == closest_angle)]

            # 5. Set omega proportional to diff(target_angle, closest_angle)
            omega = self.kp_omega * Calc.diff(robot_loc.get_robot_angle(), closest_angle)
            if abs(omega) > omega_max:
                omega = np.sign(omega) * omega_max

            # 6. Set speed v anti-proportional to occupancy value of chosen valley and omega
            # v = self.kp_v * (1 - np.sum(sector_occupancy[closest_min_valley]) /
            #         (np.size(closest_min_valley) * self.hist_threshold)) * (1 - abs(omega) / omega_max)
            v = self.kp_v * (1 - abs(omega) / omega_max)

        return [v, omega]

    def backup_avoid_obstacle(self, robot_loc, target_point, mode='simple',  debug=False):
        """ Calculates nearest way in regard of given target point around detected obstacles
        :param robot_loc:       Robot_Location reference
        :param target_point:    [x,y]
        :param mode:            'simple', 'middle' or 'edge' choose path in min_valley, default=simple
        :return:                speed, angular velocity ([v,omega])
        """

        v_max = robot_loc.get_max_robot_speed()
        omega_max = robot_loc.get_max_robot_omega()
        valley_edge_offset = 10.0 / 180.0 * pi  # Offset to hold to an edge of a valley
        closest_angle = None

        # Get direction to target_point [-pi...pi]
        target_angle = Calc.get_angle_from_point_to_point(robot_loc.get_robot_point(), target_point)

        # Create polar histogram and transform sector_angles from range [0..2pi] to [-pi...pi]
        sector_angles, sector_occupancy = self.create_histogram(debug=False)
        sector_angles = sector_angles.flatten() / 180.0 * pi
        sector_angles[sector_angles > pi] -= 2 * pi
        sector_occupancy = sector_occupancy.flatten()
        # Check if polar histogram is empty. If so, no obstacle is around -> use target_angle directly
        if not np.any(sector_occupancy):
            # Set omega proportional to diff(target_angle, closest_angle)
            omega = self.kp_omega * Calc.diff(robot_loc.get_robot_angle(), target_angle)

            # Set speed v to v_max
            v = v_max
        else:
            # Polar histogram contains at least 1 occupancy value -> search closest angle

            # 1. Search for occupancy values below threshold and return indexes
            min_indexes = np.where(sector_occupancy < self.hist_threshold)[0]

            # 2. Group all found min_indexes to valleys (check for neighborhood indexes)
            min_valleys = []
            valley_temp = np.array([min_indexes[0]])
            for i in xrange(1, np.size(min_indexes)):
                if min_indexes[i-1] + 1 == min_indexes[i]:
                    # Direct neighbor found -> add index to valley_temp
                    valley_temp = np.hstack((valley_temp, min_indexes[i]))
                else:
                    # Next index is not direct neighbor -> add valley_temp to min_valleys and start a new valley_temp
                    if np.size(valley_temp) > 1:
                        min_valleys.append(valley_temp)
                    valley_temp = min_indexes[i]
            if np.size(valley_temp) > 1:
                min_valleys.append(valley_temp)
            # 2.1 Check if min_valleys contains only one min_valley with all sectors
            if np.size(min_valleys) == np.size(sector_angles):
                pass
            # 2.2 Check if first (0°) and last sector (360° - hist_resolution) are neighbors.
            #     If so concat the first and the last min_valley
            elif min_valleys[0][0] == 0 and min_valleys[-1][-1] == np.size(sector_angles) - 1:
                min_valleys = [np.hstack((min_valleys[-1], min_valleys[0]))] + min_valleys[1:-1]
            # 2.3 Check if last sector was a single min_index (not treated as min_valley and discarded).
            #     If so and if 0° in first min_valley, append last sector to this min_valley
            elif min_valleys[0][0] == 0 and min_indexes[-1] == np.size(sector_angles) - 1:
                min_valleys = [np.hstack((min_indexes[-1], min_valleys[0]))] + min_valleys[1:]
            # 2.3 Check if first sector was a single min_index (not treated as min_valley and discarded).
            #     If so and if last sector in last min_valley, append first sector to this min_valley
            elif min_indexes[0] == 0 and min_valleys[-1][-1] == np.size(sector_angles) - 1:
                min_valleys = [np.hstack((min_valleys[-1], min_indexes[0]))] + min_valleys[0:-1]

            # Choose either the simple, middle or an edge of the best min_valley
            if mode == 'simple' or mode == 'middle':
                # 3. Check if target_angle is in one of the min_valleys, if so set target_angle directly as closest_angle
                for min_valley in min_valleys:
                    min_valley_angles = sector_angles[min_valley]
                    try:
                        if Calc.angle_in_range(min_valley_angles[0], min_valley_angles[-1], target_angle, offset=valley_edge_offset):
                            closest_angle = target_angle
                            closest_min_valley = min_valley
                            break
                    except IndexError:
                        continue
                # 4. If target_angle is not in one of the min_valleys -> search min_valley_edge closest to target_angle
                if closest_angle is None:
                    # Calculate the middle of each min_valley and min_valley_weight according to valley size (quadratic)
                    min_valley_angles = np.empty(0, dtype=np.float)
                    min_valley_weights = np.empty(0, dtype=np.float)
                    for min_valley in min_valleys:
                        min_valley_angle = Calc.get_average_angle(sector_angles[min_valley])
                        if min_valley_angle is None:
                            # Use old angle
                            min_valley_angle = self.min_valley_angle_old
                        self.min_valley_angle_old = min_valley_angle
                        min_valley_angles = np.hstack((min_valley_angles, min_valley_angle))
                        min_valley_weights = np.hstack((min_valley_weights, np.size(min_valley) ** 2.0))

                    # Search angle closest to target_angle
                    closest_angle, angle_diffs = Calc.search_closest_angle(target_angle, min_valley_angles)

                    closest_min_valley = min_valleys[np.argwhere(min_valley_angles == closest_angle)]
            if mode == 'edge':
                # 3. Check if target_angle is in one of the min_valleys, if so set target_angle directly as closest_angle
                for min_valley in min_valleys:
                    min_valley_angles = sector_angles[min_valley]
                    try:
                        if Calc.angle_in_range(min_valley_angles[0], min_valley_angles[-1], target_angle, offset=valley_edge_offset):
                            closest_angle = target_angle
                            closest_min_valley = min_valley
                            break
                    except IndexError:
                        continue

                # 4. If target_angle is not in one of the min_valleys -> search min_valley_edge closest to target_angle
                if closest_angle is None:
                    # 4.1 Get left and right min_valley_edges and apply valley_edge_offset
                    min_valley_edges_right = np.asarray([sector_angles[mv[-1]] - valley_edge_offset for mv in min_valleys])
                    min_valley_edges_left = np.asarray([sector_angles[mv[0]] + valley_edge_offset for mv in min_valleys])
                    min_valley_edges = np.vstack((min_valley_edges_left, min_valley_edges_right)).transpose()
                    # 4.2 Check if min_valley is big enough to apply offset
                    min_valley_diffs = Calc.diff_custom(min_valley_edges_left, min_valley_edges_right, counterclock=True)
                    min_valley_big_enough = np.asarray([False if mvd < 0 else True for mvd in min_valley_diffs])
                    # 4.3 Discard all min_valleys which are too small for offset
                    try:
                        min_valley_angles = min_valley_edges[min_valley_big_enough].flatten()
                    except IndexError as e:
                        print e
                        print min_valley_edges
                        print min_valley_big_enough
                    # 4.4 Search min_valley_edge closest to target angle
                    closest_angle, angle_diffs = Calc.search_closest_angle(target_angle, min_valley_angles)
                    # 4.5 Get corresponding min_valley
                    for i, mve in enumerate(min_valley_edges):
                        if closest_angle in mve:
                            closest_min_valley = min_valleys[i]

            # 5. Set omega proportional to diff(target_angle, closest_angle)
            omega = self.kp_omega * Calc.diff(robot_loc.get_robot_angle(), closest_angle)
            if abs(omega) > omega_max:
                omega = np.sign(omega) * omega_max
            print '####'
            print (target_angle * 180.0 / pi)
            print (closest_angle * 180.0 / pi)
            print (robot_loc.get_robot_angle() * 180.0 / pi)
            print (Calc.diff(robot_loc.get_robot_angle(), closest_angle) * 180.0 / pi)
            print omega

            # 6. Set speed v anti-proportional to occupancy value of chosen valley and omega
            v = self.kp_v * (1 - np.sum(sector_occupancy[closest_min_valley]) /
                       (np.size(closest_min_valley) * self.hist_threshold)) * (1 - abs(omega) / omega_max)

        if debug:
            print 'DEBUG: avoid_obstacle()'
            print '\tminimum valleys mean angles'
            print min_valley_angles
            print '\ttarget angle: %0.2f' % (target_angle * 180.0 / pi)
            print '\tclosest angle: %0.2f' % (closest_angle * 180.0 / pi)

        return [v, omega]

    def move_grid(self, dx, dy, debug=False):
        """ Shift grid values according to robot motion dx, dy
        :param dx: relative robot movements in x direction
        :param dy: relative robot movements in y direction
        :return: x_residual, y_residual
        """

        x_shift = int(round(dx / self.cell_size))
        y_shift = int(round(dy / self.cell_size))

        # Calculate remaining (=residual) dx, dy
        x_residual = (x_shift - dx / self.cell_size) * self.cell_size
        y_residual = (y_shift - dy / self.cell_size) * self.cell_size

        # Shift array values via padding
        # np.pad(array_like,((before_axis1,after_axis1),(before_axis2,after_axis2)), ...
        # ... mode='constant')[column_start : column_stop, row_start : row_stop]

        if x_shift > 0:
            self.grid = np.pad(self.grid, ((0, 0), (0, x_shift)), mode='constant')[:, x_shift:]
        elif x_shift < 0:
            x_shift = abs(x_shift)
            self.grid = np.pad(self.grid, ((0, 0), (x_shift, 0)), mode='constant')[:, :-x_shift]

        if y_shift > 0:
            self.grid = np.pad(self.grid, ((y_shift, 0), (0, 0)), mode='constant')[:-y_shift, :]
        elif y_shift < 0:
            y_shift = abs(y_shift)
            self.grid = np.pad(self.grid, ((0, y_shift), (0, 0)), mode='constant')[y_shift:, :]

        if debug:
            print 'DEBUG: move_grid()'
            print '\tdx = %0.2f, dy = %0.2f' % (dx, dy)
            print '\t-> x_shift = %i, y_shift = %i' % (x_shift, y_shift)
            print '\t-> x_residual = %0.2f, y_residual = %0.2f' % (x_residual, y_residual)
            print self.grid

        return x_residual, y_residual

    def set_value(self, r, theta, value=1, debug=False):
        """ Set grid value at r, theta
        :param r: radius
        :param theta: angle
        :param value: default 1
        :return: value successfully set in histogram: True/False
        """

        # Convert polar coordinates to cartesian coordinates
        x, y = Calc.polar_2_cartesian(r, theta)

        # Coord Transformation: Place origin in middle of coord-system and reverse y-axis
        xi = int(x/self.cell_size + self.x_size / 2.0)
        yi = int(-y/self.cell_size + self.y_size / 2.0)

        # Check if coordinates exceed grid boundaries
        if xi < 0 or xi >= self.x_size:
            # DEBUG
            if debug:
                print 'DEBUG: set_value()'
                print 'Warning set_value()'
                print '\tx_size mismatch!'
            return False
        if yi < 0 or yi >= self.y_size:
            # DEBUG
            if debug:
                print 'DEBUG: set_value()'
                print 'Warning set_value()'
                print '\ty_size mismatch!'
            return False
        self.grid[yi, xi] += value

        # DEBUG
        if debug:
            print 'DEBUG: set_value()'
            print '\tGiven polar coordinates'
            print '\tr = %0.2f, theta = %0.2f' % (r, theta)
            print '\tResulting cartesian coordinates:'
            print '\tx = %0.2f, y = %0.2f' % (x, y)
            print '\tGrid coordinates:'
            print '\txi = %i, yi = %i' % (xi, yi)
            # print '\tResulting grid:'
            # print self.grid

        return True

    def reset_histogram(self):
        """ Reset the entire histogram and clear all values
        :return: -
        """

        self.grid = np.zeros((self.x_size, self.y_size), dtype=np.int)

    def create_histogram(self, debug=False, **kwargs):
        """ Creates histogram and returns the angles with corresponding occupancy values
        :param resolution: angle for each sector, default: 10°
        :param debug: enable/disable debug-printing
        :return: numpy array([[sector_angles],[sector_occupancy]])
        """

        if 'resolution' in kwargs:
            if isinstance(kwargs['resolution'], Number):
                resolution = kwargs['resolution']
            else:
                resolution = self.hist_resolution
        else:
            resolution = self.hist_resolution

        # Constants
        weight_const_a = 1.0
        weight_const_b = 1.0 / 5.0  # 1 / max. sense value

        # Create arrays for sector_angles ([angles]) and sector_occupancy ([empty])
        sectors = int(360/float(resolution) + 1)  # include 360° angle for easier looping
        sector_angles = np.array([x * resolution for x in xrange(sectors)])
        sector_occupancy = np.zeros(sectors)

        # Get occupied cells from histogram grid and save their indexes ( array([[y1,y2,...,yn],[x1,x2,..,xn]]) )
        occupied_cell_indexes = np.asarray(np.nonzero(self.grid))

        # Get occupancy_values of occupied cells
        occupancy_values = self.grid[occupied_cell_indexes[0], occupied_cell_indexes[1]]

        # Reverse Coord Transformation: Shift origin back to left upper corner of coord-system and reverse y-axis
        occupied_cell_indexes_x = (occupied_cell_indexes[1] - (self.x_size - 1) / 2.0) * self.cell_size
        occupied_cell_indexes_y = (-occupied_cell_indexes[0] + (self.y_size - 1) / 2.0) * self.cell_size

        # Transpose cell_indexes to get (x,y) tuples ( array([[y1,x1],[y2,x2],...[yn,xn]]) )
        # occupied_cell_indexes_t = np.transpose(occupied_cell_indexes)

        # Convert cell indexes to polar coordinates, calculate weight_m and add it to the corresponding sector
        # distances = array([d1,d2,...d_n])
        distances = np.sqrt(occupied_cell_indexes_y**2 + occupied_cell_indexes_x**2)
        # angles = array([alpha1,alpha2,...,alpha_n])
        angles = np.arctan2(occupied_cell_indexes_y, occupied_cell_indexes_x) / pi * 180.0
        # If angle < 0 add 360° to get range 0°...360°
        angles[angles < 0] += 360.0
        # Calculate weight for each occupied cell
        weights = occupancy_values**2 * (weight_const_a - weight_const_b * distances)

        # Add weights to corresponding sector n according to angle alpha
        for i in xrange(len(sector_angles) - 1):
            # Find indexes of all weight-angles which fit into current sector_angle
            indexes = np.argwhere([sector_angles[i] <= angle < sector_angles[i+1] for angle in angles])
            # Add sum of all indexed weights to current sector
            sector_occupancy[i] = np.sum(weights[indexes])

        # Discard last sector and its occupancy_value
        sector_angles = sector_angles[:-1]
        sector_occupancy = sector_occupancy[:-1]

        # Make histogram smoother
        # l = 5
        # occupancy_sum = 0
        # len_sector_occupancy = np.size(sector_occupancy)
        # for k in xrange(np.size(sector_occupancy) - 1):
        #     for q in xrange(0, l+1):
        #         occupancy_sum += float(q + 1) * sector_occupancy[k-l+q]
        #     for q in xrange(l, 0, -1):
        #         index = k+l-q+1
        #         if index >= len_sector_occupancy:
        #             index -= len_sector_occupancy
        #         occupancy_sum += float(q) * sector_occupancy[index]
        #     sector_occupancy[k] = occupancy_sum / (2 * l + 1)

        # Generate histogram, discard 360° sector and its occupancy value (last element)
        histogram = np.array([[sector_angles], [sector_occupancy]])

        # DEBUG
        if debug:
            print 'DEBUG: create_histogram()'
            print '\tsector_angles:\n\t', sector_angles
            print'\tsector_occupancy:\n\t', sector_occupancy
            print'\toccupied_cell_indexes:\n\t', occupied_cell_indexes
            print'\toccupancy_values:\n\t', occupancy_values
            print'\toccupied_cell_indexes transformed:\n\t', occupied_cell_indexes
            print '\tdistances:\n\t', distances
            print '\tangles:\n\t', angles
            print '\toccupancy values:\n\t', occupancy_values
            print '\tweights:\n\t', weights
            print '\tsector_occupancy updated:\n\t', sector_occupancy
            # print '\thistogram transposed:\n\t', histogram.transpose()

        return histogram

    def get_value(self, x, y):
        """ Get grid value at the coordinate (x,y)
        :param x: coord x
        :param y: coord y
        :return: grid value at (x,y)
        """

        if x < 0 or x > self.width:
            return
        if y < 0 or y > self.width:
            return
        xi = int(x/self.cell_size)
        yi = int(y/self.cell_size)

        return self.grid[yi, xi]

    def draw_grid(self, debug=False):
        """ Draw current grid
        :return: -
        """

        if debug:
            print 'DEBUG draw_grid()'
            print self.grid

        if self.plt_grid is None:
            self.plt_grid = self._init_grid(self.ax1)
            cbar = plt.colorbar(mappable=self.plt_grid)
            cbar.set_label('Occupancy', fontsize=16)
        else:
            self.plt_grid.set_data(self.grid)
            self.plt_grid.autoscale()
        plt.show()

    def draw_hist(self):
        """ Draw given histogram
        :return: -
        """
        # get sector angles and occupancy
        self.histogram = self.create_histogram(resolution=self.hist_resolution)
        sector_angles, sector_occupancy = self.histogram
        if self.plt_hist is None:
            self.plt_hist = self._init_hist(self.ax2)
        else:
            [bar.set_height(sector_occupancy[0][i]) for i, bar in enumerate(self.plt_hist)]
        self.ax2.set_ylim([0, np.max(sector_occupancy[0])])
        plt.show()



