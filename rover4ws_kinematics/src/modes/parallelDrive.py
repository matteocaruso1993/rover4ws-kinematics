from .base import BaseKinematics
import numpy as np


class LateralDrive(BaseKinematics):
    def __init__(self, config_path=None, config_override=None):
        mode = "parallel"
        super().__init__(mode=mode, config_path=config_path, config_override=config_override)

    def _computeICR(self, in_speed_high_level):
        return (None, None, None) #To- DO

    def _applyWheelsCommands(self, in_speed_high_level):
        self._current_steer[:] = np.pi/2
        self._current_wheel_speed[:] = in_speed_high_level[1]/self.config["wheel_radius"]

    def _applyWheelsHomingPosition(self):
        self._current_steer[:] = np.pi/2
        self._current_wheel_speed[:] = 0

    def _preprocessHighLevelSpeeds(self, high_level_speed):
        high_level_speed[2] = 0
        high_level_speed[0] = 0
        self._v_wheels[:,1] = high_level_speed[1]
        
        return high_level_speed


if __name__ == '__main__':
    b = LateralDrive()
    #b.computeInverseKinematics([0,0,0])
    #b._validateWheelsState()
    b.kinematicsStep([0,-1,0])
    b.show(draw_wheels_arrows=True, draw_computed_wheel_lin_speed=True)
