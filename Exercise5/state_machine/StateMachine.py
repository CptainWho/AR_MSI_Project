# -*- coding: utf-8 -*-

__author__ = 'Ecki'

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
        self.init_state = 'NoObstacle'
        self.current_state = self.init_state
        self.old_state = self.current_state

    def next_state(self, debug=False):
        """ Determine next state according to given transition and current state
        :param next_point
        :return: next state
        """

        if self.current_state == 'NoObstacle':
            # First check if any obstacle is in sight
            if self.transitions.next_room_reached():
                self.current_state = 'RoomReached'
            elif self.transitions.obstacle_in_sight():
                self.current_state = 'Obstacle'

        elif self.current_state == 'Obstacle':
            # First check if obstacle is still in sight
            if not self.transitions.obstacle_in_sight():
                self.current_state = 'NoObstacle'
            elif self.transitions.next_room_reached():
                self.current_state = 'RoomReached'

        elif self.current_state == 'RoomReached':
            if not self.transitions.all_rooms_visited():
                self.current_state = 'NoObstacle'
            else:
                self.current_state = 'Finished'

        elif self.current_state == 'Finished':
            pass

        # DEBUG
        if debug:
            print 'Next state: %s' % self.current_state

        if self.current_state is not self.old_state:
            print self.current_state

        self.old_state = self.current_state

        return self.current_state