import numpy as np
import control as ct

class LQRController:
    def __init__(self, saturation):
        self.saturation = saturation
        self.K = None  # control state feedback gain
        self.S = None  # Riccati solution
        self.E = None  # closed-loop eigenvalues
        self.printed = 0

    def solveOCP(self, A, B, Q, R, debug=False):
        self.K, self.S, self.E = ct.lqr(A, B, Q, R)
        if debug:
            print("K:", self.K)
            print("S:", self.S)
            print("E:", self.E)

    def control(self, state, setpoint, debug=False):
        error = setpoint - state
        output = np.dot(self.K, error)
        output = output[0]  # extract scalar if K returns a one-element array
        output = np.clip(output, -self.saturation, self.saturation)
        # if debug:
        if debug and self.printed < 100:
            # pass
            print("Control output:", output, "state", state)
            self.printed += 1
        return output
