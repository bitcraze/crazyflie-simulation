class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.setpoint = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def update(self, setpoint, feedback_value, dt):
        error = setpoint - feedback_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

class PIDController3D:
    def __init__(self, gainsX, gainsY, gainsZ):
        self.x = PIDController(gainsX[0], gainsX[1], gainsX[2])
        self.y = PIDController(gainsY[0], gainsY[1], gainsY[2])
        self.z = PIDController(gainsZ[0], gainsZ[1], gainsZ[2])

    def update(self, actual_value, desired_value, dt):
        return [self.x.update(actual_value[0], desired_value[0], dt),
                self.y.update(actual_value[1], desired_value[1], dt),
                self.z.update(actual_value[2], desired_value[2], dt)]

class PIDControllerFullState:

    def __init__(self, gains_pos, gains_vel, gains_att, gains_att_rate):
        self.pos = PIDController3D(gains_pos[0], gains_pos[1], gains_pos[2])
        self.vel = PIDController3D(gains_vel[0], gains_vel[1], gains_vel[2])
        self.att = PIDController3D(gains_att[0], gains_att[1], gains_att[2])
        self.rate = PIDController3D(gains_att_rate[0], gains_att_rate[1], gains_att_rate[2])


    def update(self, actual_value, desired_value, pos_mode, att_mode, desired_thrust, dt):

        # pos_mode = 0: position control
        # pos_mode = 1: velocity control
        # pos_mode = 2: attitude control

        # att_mode = 0: attitude control
        # att_mode = 1: attitude rate control

        # First run position controller
        action_pos = self.pos.update(actual_value[0], desired_value[0], dt)

        # setup setpoints for velocity controller
        setpoint_vel = [0, 0, 0]
        for i in range(3):
            if pos_mode[i] == 0:
                setpoint_vel[i] = action_pos[i]
            elif pos_mode[i] == 1:
                setpoint_vel[i] = desired_value[1][i]

        # Run velocity controller
        action_vel = self.vel.update(actual_value[1], setpoint_vel, dt)

        # setup setpoints for attitude controller
        setpoint_att = [0, 0, 0]
        for i in range(2):
            if pos_mode[i] == 0 or pos_mode[i] == 1:
                setpoint_att[i] = action_vel[i]
            elif pos_mode[i] == 2:
                setpoint_att[i] = desired_value[2][i]

        setpoint_att[2] = desired_value[2][2]

        if pos_mode[2]== 0 or pos_mode[2]==1:
            action_thrust = action_vel[2]
        elif pos_mode[2] == 2:
            action_thrust = desired_thrust

        action_att = self.att.update(actual_value[2], setpoint_att, dt)

        # setup setpoints for attitude rate controller
        setpoint_rate = [0, 0, 0]
        for i in range(3):
            if att_mode[i] == 0:
                setpoint_rate[i] = action_att[i]
            elif att_mode[i] == 1:
                setpoint_rate[i] = desired_value[3][i]

        # Run attitude rate controller
        action_rate = self.rate.update(actual_value[3], setpoint_rate, dt)

        return action_rate[0], action_rate[1], action_rate[2], action_thrust



