# -*- coding: utf-8 -*-
__author__ = 'Ecki'


# class for a closed-loop PID controller
class PID:

    def __init__(self, k_p, k_i, k_d, dt):
        # closed-loop parameters
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        # timestep
        self.dt = dt
        # internal storage
        self.error_old = 0
        self.int_error_dt = 0


    # apply closed-loop
    def control(self, error):
        # build derivative
        derror_dt = (error - self.error_old) / self.dt
        # build integral
        self.int_error_dt += (error + self.error_old) * self.dt / 2
        # apply closed-loop
        output = - self.k_p * error - self.k_d * derror_dt - self.k_i * self.int_error_dt
        # refresh old value
        self.error_old = error

        return output


    # clear internal storage for integral and derivate
    def clearStorage(self):
        self.error_old = 0
        self.int_error_dt = 0