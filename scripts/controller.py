class PID:
    
    def __init__(self, dt, _max, _min, Kp, Kd, Ki):
        self.dt = dt
        self.max_u = _max
        self.min_u = _min
        self._Kp = Kp
        self._Kd = Kd
        self._Ki = Ki
        self._pre_error = 0
        self._integral = 0
        self._loopCount = 0
        self._antiWindUpCycle = 100

    def calculate(self, setpoint,  pv):
        # Calculate error
        error = setpoint - pv

        # Proportional term
        Pout = self._Kp * error

        # Integral term
        self._integral += error * self.dt
        Iout = self._Ki * self._integral

        # Derivative term
        derivative = (error - self._pre_error) / self.dt
        Dout = self._Kd * derivative

        #  Calculate total u
        u = Pout + Iout + Dout

        # Restrict to max/min
        if( u > self.max_u ):
            u = self.max_u
        elif( u < self.min_u ):
            u = self.min_u

        # Save error to previous error
        self._pre_error = error

        # anti wind up for the integral error
        self._loopCount += 1
        if self._loopCount >= self._antiWindUpCycle:
            self._integral = 0
            self._loopCount = 0




        return u
