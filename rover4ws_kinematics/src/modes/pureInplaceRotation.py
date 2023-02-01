from .base import BaseKinematics
import numpy as np



class InplaceRotation(BaseKinematics):
    def __init__(self, config_path=None, config_override=None):
        mode = 'in_place'
        super().__init__(mode=mode, config_path=config_path, config_override=config_override)

    def _applyWheelsHomingPosition(self):
        self._v_wheels[:,:] = 0
        self._current_wheel_speed[:] = 0
        #self._current_steer = np.array(4*[np.pi/4]) #To do

    def _preprocessHighLevelSpeeds(self, high_level_speed):
        high_level_speed[0] = 0
        high_level_speed[1] = 0
        if not np.any(high_level_speed):
            #Homing requested
            self._homing_requested = True
        else:
            self._homing_requested = False
        self._last_commanded_speed = high_level_speed
        return high_level_speed


    def _computeICR(self, in_speed_high_level):
        Rx,Ry,R = (0,0,0)
        Rx, Ry = self._constrainIcr(Rx,Ry)
        return (Rx,Ry,R)



    def show(self, plot=True, show_frame=False, draw_wheels_arrows=False, draw_computed_wheel_lin_speed=False):
        return super().show(plot=plot, show_frame=show_frame, draw_wheels_arrows=draw_wheels_arrows, draw_computed_wheel_lin_speed=draw_computed_wheel_lin_speed)


if __name__ == '__main__':
    car = InplaceRotation()
    #car.computeInverseKinematics([0.0,0,0.0])
    car.kinematicsStep([0,0,1])
    print(car._current_wheel_speed)
    car.show(True)
