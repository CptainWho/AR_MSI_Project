# -*- coding: utf-8 -*-
""" Module Description:
Implements statemachine functionality
"""

__author1__ = 'Ecki'
__author2__ = 'Philipp Lohrer'
__date__ = '30.05.2015'
__version__ = '1.0'

# Standard library imports
# Local imports


class StateMachine:
    """ Class description:
    Implements statemachine functionality,

    """
    def __init__(self, transitions):
        """ Initialization
        :param transitions:
        :return:
        """
        self.transitions = transitions
        self.init_state = 'CornerReached'
        self.current_state = self.init_state

    def next_state(self, debug=False):
        """ Determine next state according to given transition and current state
        :return: next state
        """

        if self.current_state == 'NoObstacle':
            # First check if any obstacle is in sight
            if self.transitions.obstacleInSight():
                self.current_state = 'Obstacle'
            if self.transitions.nextPointReached():
                if self.transitions.endPointReached():
                    self.current_state = 'TargetReached'
                else:
                    self.current_state = 'CornerReached'

        elif self.current_state == 'Obstacle':
            # First check if obstacle is still in sight
            if not self.transitions.obstacleInSight():
                self.current_state = 'NoObstacle'
            elif self.transitions.nextPointReached():
                if self.transitions.endPointReached():
                    self.current_state = 'TargetReached'
                else:
                    self.current_state = 'CornerReached'

        elif self.current_state == 'CornerReached':
            if self.transitions.aimingToNextPoint() and not self.transitions.obstacleInSight():
                self.current_state = 'NoObstacle'
            else:
                self.current_state = 'Obstacle'

        # DEBUG
        if debug:
            print 'Next state: %s' % self.current_state

        return self.current_state