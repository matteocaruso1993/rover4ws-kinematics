from .base import BaseKinematics
from ..utils.icr_handler import IcrHandler
import numpy as np


class ParallelAckermann(BaseKinematics):
    def __init__(self, config_path=None):
        mode = "outer_ack"
        self._icr_handler = IcrHandler(mode=mode)
        self._icr_handler.initialize()
        super().__init__(mode, config_path=config_path)

    def computeInverseKinematics(self, in_speed_high_level):
        return super().computeInverseKinematics(in_speed_high_level)

        self._constrainICR()
    def _preprocessHighLevelSpeeds(self, high_level_speed):
        high_level_speed[2] = 0
        if not np.any(high_level_speed):
            #Homing requested
            self._homing_requested = True
        else:
            self._homing_requested = False

        if high_level_speed[1] >= 0:
            high_level_speed[2] = 1e-9
        else:
            high_level_speed[2] = -1e-9

        self._last_commanded_speed = high_level_speed
        return high_level_speed
    
    def _constrainIcr(self, x, y):
        #Here we apply the symmetric-ackermann constraint
        self._last_computed_icr = np.array([x, y,0])
        self._icr_handler.validateIcr([x,y])
        x_icr,y_icr = self._icr_handler.getProjectedIcr()
        return (x_icr, y_icr)


    def show(self, plot=True, show_frame=False, draw_wheels_arrows=False, draw_computed_wheel_lin_speed=False):
        return super().show(plot=plot, show_frame=show_frame, draw_wheels_arrows=draw_wheels_arrows, draw_computed_wheel_lin_speed=draw_computed_wheel_lin_speed)

if __name__ == '__main__':
    car = ParallelAckermann()
    #car.computeInverseKinematics([0.0,0,0.0])
    car.kinematicsStep([0,-0.5,-1/1e9])
    print(car._current_wheel_speed)
    print(car._current_steer)
    car.show(True, draw_wheels_arrows=True, draw_computed_wheel_lin_speed=True)
