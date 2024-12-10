
class PID:
    """
    PID controller.

    Attributes:
        kp (float):                 Proportional gain
        ki (float):                 Integrational gain
        kd (float):                 Derivational gain
        time_interval (float):      Time between cal. control variable 
                                    (corrective action) 
        desired_val (float):        Goal value of system 
        prev_error (float)
        integral_error (float)
        bias
    """

    def __init__(self, time_interval: float, goal: float, kp: float = 0.0, ki: 
                 float = 0.0, kd: float = 0.0, bias: float = 0.0): 
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.time_interval = time_interval
        self.integral_error = 0.0
        self.desired_val = goal
        self.prev_error = 0.0
        self.bias = bias 

    def compute_adjustment(self, current_val: float) -> float:
        error = self.desired_val - current_val 
        self.integral_error += self.time_interval * error
        derivation_error =  (error - self.prev_error) / self.time_interval 
        self.prev_error = error

        return (self.kp * error) + (self.ki * self.integral_error) + \
                (self.kd * derivation_error) + self.bias

    def getError(self):
        return self.prev_error