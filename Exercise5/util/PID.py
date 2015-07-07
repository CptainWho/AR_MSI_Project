# -*- coding: utf-8 -*-
__author__ = 'Ecki'


class PID:
    """
    Closed-loop PID controller
    """

    def __init__(self, k_p, k_i, k_d, dt):
        """
        :param k_p: proportional parameter
        :param k_i: integral parameter
        :param k_d: derivative parameter
        :param dt: time-step
        :return: -
        """
        # closed-loop parameters
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        # time-step
        self.dt = dt
        # internal storage
        self.error_old = 0
        self.int_error_dt = 0

    def control(self, error):
        """ Return next controller output according to given error
        :param error: current error
        :return: controller output (float)
        """
        # build derivative
        d_error_dt = (error - self.error_old) / self.dt
        # build integral
        self.int_error_dt += (error + self.error_old) * self.dt / 2
        # apply closed-loop
        output = - self.k_p * error - self.k_d * d_error_dt - self.k_i * self.int_error_dt
        # refresh old value
        self.error_old = error

        return output

    def clear_storage(self):
        """ Clear internal storage for integral and derivative
        :return: -
        """
        self.error_old = 0
        self.int_error_dt = 0