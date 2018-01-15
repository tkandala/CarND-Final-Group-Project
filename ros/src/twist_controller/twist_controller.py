from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

KP_VEL = 0.33
KI_VEL = 0.00000001
KD_VEL = 0.005

#brake = (vehicle_mass + fuel_capacity * GAS_DENSITY) * acceleration * wheel_radius
#Now taking the parameter values for the simulator (dbw_sim.launch), I get, with the max acceleration allowed:
#brake = (1080.0 + 0.0 * 2.858) * (-5) * 0.335 = -1809 Nm
#
#About yaw angle and x, y, z x_values
#http://www.formula1-dictionary.net/motions_of_f1_car.html

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, yaw_controller):

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.yaw_controller = yaw_controller
        self.pid_vel = PID(KP_VEL, KI_VEL, KD_VEL, decel_limit, accel_limit)

    def control(self, target_linear_v, target_angular_v, current_linear_v, dbw_status, sample_time):
        if dbw_status == False:
            self.pid_vel.reset()
            return 0., 0., 0.

        error_linear_v = target_linear_v - current_linear_v
        value = self.pid_vel.step(error_linear_v, sample_time)

        brake = 0.0
        throttle = 0.0
        if value > 0.0:
            throttle = value
        elif value < -self.brake_deadband:
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * abs(value) * self.wheel_radius

        steer = self.yaw_controller.get_steering(target_linear_v, target_angular_v, current_linear_v)
        return throttle, brake, steer
