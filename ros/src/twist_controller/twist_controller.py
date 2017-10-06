from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
LINEAR_P_TERM = 0.6
LINEAR_I_TERM = 0.05
LINEAR_D_TERM = 0.1 


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.linear_velocity_controller = PID(LINEAR_P_TERM, LINEAR_I_TERM, LINEAR_D_TERM, self.decel_limit, self.accel_limit)
        self.steering_controller = YawController(self.wheel_base, self.steer_ratio, 1, self.max_lat_accel, self.max_steer_angle)

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        # arguments: 
        # desired target velocity, desired angular velocity, current linear velocity, time_elapsed, dbw_enable_status

        target_linear_v = args[0]
        target_steer = args[1]
        curr_linear_v = args[2]
        time_elapsed = args[3]
        dbw_enabled = args[4]

        vel_cte = target_linear_v - curr_linear_v 

        velocity = self.linear_velocity_controller.step(vel_cte, time_elapsed)
        steer = self.steering_controller.get_steering(target_linear_v, target_steer, curr_linear_v)
        
        throttle = 0.0
        brake = 0.0

        if velocity > 0:
            brake = 0.0
            throttle = velocity
        else:
            brake = -1.0 * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * velocity

        return throttle, brake, steer
