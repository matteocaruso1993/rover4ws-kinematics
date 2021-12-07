from .base import BaseKinematics
from ..utils.icr_handler import IcrHandler
import numpy as np


class CarLike(BaseKinematics):
    def __init__(self, config_path=None):
        mode="car_like"
        self._icr_handler = IcrHandler(mode=mode)
        self._icr_handler.initialize()
        super().__init__(mode=mode, config_path=config_path)
        

    def _constrainIcr(self, x, y):
        #Here we apply the Car-Like constraint
        self._last_computed_icr = np.array([x, y,0])
        self._icr_handler.validateIcr([x,y])
        x_icr,y_icr = self._icr_handler.getProjectedIcr()
        return (x_icr, y_icr)
    
    def show(self, plot=True, show_frame=False, draw_wheels_arrows=False, draw_computed_wheel_lin_speed=False):
        return super().show(plot=plot, show_frame=show_frame, draw_wheels_arrows=draw_wheels_arrows, draw_computed_wheel_lin_speed=draw_computed_wheel_lin_speed)


if __name__ == '__main__':
    car = CarLike()
    #car.computeInverseKinematics([0.0,0,0.0])
    car.kinematicsStep([0,0,-10])
    print(car._current_wheel_speed)
    car.show(True)

