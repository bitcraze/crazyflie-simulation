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
                self.y.update(actual_value[0], desired_value[1], dt),
                self.z.update(actual_value[0], desired_value[2], dt)]

class PIDControllerFullState:

    def __init__(self, gainsPos, gainsVel, gainsAtt):
        self.pos = PIDController3D(gainsPos[0], gainsPos[1], gainsPos[2])
        self.vel = PIDController3D(gainsVel[0], gainsVel[1], gainsVel[2])
        self.att = PIDController3D(gainsAtt[0], gainsAtt[1], gainsAtt[2])


    def update(self, actual_value, desired_value, pos_mode, desired_thrust, dt):

        # First run position controller
        action_pos = self.pos.update(actual_value, desired_value, dt)

        # setup setpoints for velocity controller
        setpoint_vel = [0, 0, 0]
        for i in range(3):
            if pos_mode[i] == 0:
               setpoint_vel[i] = action_pos[i]
            elif pos_mode[i] == 1:
                setpoint_vel[i] = desired_value[i]

        # Run velocity controller
        action_vel = self.vel.update(actual_value, desired_value, dt)

        # setup setpoints for attitude controller
        setpoint_att = [0, 0, 0]
        for i in range(2):
            if pos_mode[i] == 0 or pos_mode[i]==1:
                setpoint_att[i] = action_vel[i]
            elif pos_mode[i] == 2:
                setpoint_att[i] = desired_value[i]

        if pos_mode[2]== 0 or pos_mode[2]==1:
            action_thrust = action_vel[2]
        elif pos_mode[2] == 2:
            action_thrust = desired_thrust

        action_att = [0, 0, 0]



        # Run attitude controller
        action_att = self.att.update(actual_value, desired_value, dt)



