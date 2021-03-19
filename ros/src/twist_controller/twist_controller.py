import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, accel_limit, brake_deadband, wheel_radius, steer_ratio, max_lat_accel, max_steer_angle, wheel_base):
        kp = 0.3
        ki = 0.1
        kd = 0.0
        min_throttle = 0.0
        max_throttle = 0.5
        self.throttle_control = PID(kp, ki, kd, min_throttle, max_throttle)

        tau = 0.5
        ts = 0.02
        self.velocity_low_pass_filter = LowPassFilter(tau, ts)

        self.yaw_control = YawController(wheel_base, steer_ratio, 0, max_lat_accel, max_steer_angle)
        self.last_velocity = 0
        self.last_time = rospy.get_time()
        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius

    def control(self, current_velocity, dbw_enabled, linear_velocity, angular_velocity):
        if not dbw_enabled: #if manual is enabled, disable controller
            self.throttle_control.reset()
            return 0, 0, 0 # Return throttle, brake, steer

        brake = 0
        current_velocity = self.velocity_low_pass_filter.filt(current_velocity)
        steer = self.yaw_control.get_steering(linear_velocity, angular_velocity, current_velocity)

        self.last_velocity = current_velocity
        velocity_error = linear_velocity - current_velocity

        current_time = rospy.get_time()
        elapsed_time = current_time - self.last_time
        self.last_time = current_time

        currrent_throttle = self.throttle_control.step(velocity_error, elapsed_time)
        
        if linear_velocity == 0 and current_velocity < 0.1: #at a stop
            currrent_throttle = 0
            brake = 700
        elif currrent_throttle < 0.1 and velocity_error < 0:
            currrent_throttle = 0
            deceleration = max(velocity_error, self.decel_limit)
            brake = abs(deceleration) * self.vehicle_mass * self.wheel_radius
        
        # Return throttle, brake, steer
        return currrent_throttle, brake, steer
