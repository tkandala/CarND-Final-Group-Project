from pid import PID
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_THROTTLE = 1.0

KP_VEL = 0.8
KI_VEL = 0.0
KD_VEL = 0.05

#brake = (vehicle_mass + fuel_capacity * GAS_DENSITY) * acceleration * wheel_radius
#Now taking the parameter values for the simulator (dbw_sim.launch), I get, with the max acceleration allowed:
#brake = (1080.0 + 0.0 * 2.858) * (-5) * 0.335 = -1809 Nm
#Does it imply that I shall send the simulator a brake value between 0 and 1809?

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
        # TODO: Change the arg, kwarg list to suit your needs
        if dbw_status.data == False:
            self.pid_vel.reset()
            return 0., 0., 0.

        error_linear_v = target_linear_v.x - current_linear_v.x
        value = self.pid_vel.step(error_linear_v, sample_time)

        rospy.loginfo('error_linear_v:%s current_linear_v.x:%s value:%s', error_linear_v, current_linear_v.x, value)
        # Return throttle, brake, steer
        brake = 0.0
        throttle = 0.0
        if value > 0.0:
            throttle = value * MAX_THROTTLE
        elif value < -self.brake_deadband:
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * abs(value) * self.wheel_radius

        steer = self.yaw_controller.get_steering(target_linear_v.x, target_angular_v.z, current_linear_v.x)
        return throttle, brake, steer
