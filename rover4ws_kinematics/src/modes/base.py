import numpy as np
import os
import yaml
from yaml.loader import SafeLoader
import matplotlib.pyplot as plt
import matplotlib.patches as Patch
import matplotlib as mpl
from copy import deepcopy

#Abstact base class for kinematics



class BaseKinematics:
    def __init__(self, mode, config_path = None):
        self.mode = mode
        self.config_path =  config_path
        self.config = None
        self._wheels_mapping = {
            0:"fl_wheel",
            1:"fr_wheel",
            2:"bl_wheel",
            3:"br_wheel"
        }

        self.reset()

        self.loadConfig()
        self.computeInverseKinematics(self._last_commanded_speed)
 
    def loadConfig(self,print_output=False):
        if self.config_path is None:
            par_dir = os.path.dirname(__file__)
            config_filename = os.path.abspath(os.path.join(par_dir,'..','..','config','config.yaml'))
            if print_output:
                print(config_filename)

            try:
                with open(config_filename,'r') as f:
                    data = yaml.load(f, Loader=SafeLoader)
                    self.config = data
                    self.config_path = config_filename

            except:
                pass

    def computeForwardKinematics(self, in_speed_wheels, in_steer):
        self.last_commanded_wheels_speed = in_speed_wheels
        pass

    def computeInverseKinematics(self, in_speed_high_level):
        in_speed_high_level = self._preprocessHighLevelSpeeds(in_speed_high_level)

        x_ICR, y_ICR, R = self._computeICR(in_speed_high_level)

        self._last_valid_icr = np.array([x_ICR, y_ICR,0])

        if self._homing_requested:
            self._applyWheelsHomingPosition()
        else:
            self._applyWheelsCommands(in_speed_high_level)
                  
    def _preprocessHighLevelSpeeds(self,high_level_speed):
        if not np.any(high_level_speed):
            #Homing requested
            self._homing_requested = True
        else:
            self._homing_requested = False

        if high_level_speed[2] == 0:
            high_level_speed[2] = 1e-9
        self._last_commanded_speed = high_level_speed
        return high_level_speed

    def _clipWheelsSpeed(self):
        max_computed_speed = np.max(np.abs(self._current_wheel_speed))
        if not max_computed_speed == 0:
            max_speed_admissible = self.config['max_wheel_speed']
            if max_computed_speed > max_speed_admissible:
                scale_factor = max_speed_admissible/max_computed_speed
                self._current_wheel_speed*=scale_factor

    def _applyWheelsHomingPosition(self):
        self._v_wheels[:,:] = 0
        self._current_wheel_speed[:] = 0
        self._current_steer[:] = 0

    def _applyWheelsCommands(self,in_speed_high_level):
        pos_wheels = np.array([[self.config['len_x_leg'], +self.config['len_y_leg'], 0],
                                    [self.config['len_x_leg'], -self.config['len_y_leg'], 0],
                                    [-self.config['len_x_leg'], +self.config['len_y_leg'], 0],
                                    [-self.config['len_x_leg'], -self.config['len_y_leg'], 0]])
        pos_ICR = self._last_valid_icr


        for i in range(len(self._v_wheels)):
            self._v_wheels[i,:] = np.cross(np.array([[0,0,in_speed_high_level[2]]]), pos_wheels[i,:]-pos_ICR)
            self._current_wheel_speed[i] = np.linalg.norm(self._v_wheels[i,:])/self.config['wheel_radius']
            #if self._v_wheels[i,0]== 0 and self._v_wheels[i,1] == 0:
            #    self._current_steer[i] = self._last_steer[i]
            #else:
            self._current_steer[i] = np.arctan(self._v_wheels[i,1]/self._v_wheels[i,0])

    def _computeICR(self, in_speed_high_level):

        Rx = -in_speed_high_level[1]/in_speed_high_level[2]
        Ry = in_speed_high_level[0]/in_speed_high_level[2]

        Rx, Ry = self._constrainIcr(Rx,Ry)
        R = np.linalg.norm([Rx,Ry])

        return (Rx,Ry,R)

    def _constrainIcr(self,x,y):
        #No constraint
        self._last_computed_icr = np.array([x, y,0])
        return (x,y)

    def show(self, plot=True,show_frame = False, draw_wheels_arrows=False,draw_computed_wheel_lin_speed=False):
        chassis_scale = 1
        chassis = Patch.Rectangle((-chassis_scale*self.config['len_y_leg'],-chassis_scale*self.config['len_x_leg']), chassis_scale*2*self.config['len_y_leg'],chassis_scale*2*self.config['len_x_leg'],color='k', alpha=0.3)
        wheel_fl = Patch.Rectangle((-self.config['len_y_leg']-self.config['wheel_width']/2,self.config['len_x_leg']-self.config['wheel_radius']), self.config['wheel_width'],2*self.config['wheel_radius'],color='r', alpha=1)
        wheel_fr = Patch.Rectangle((self.config['len_y_leg']-self.config['wheel_width']/2,self.config['len_x_leg']-self.config['wheel_radius']), self.config['wheel_width'],2*self.config['wheel_radius'],color='r', alpha=1)
        wheel_bl = Patch.Rectangle((-self.config['len_y_leg']-self.config['wheel_width']/2,-self.config['len_x_leg']-self.config['wheel_radius']), self.config['wheel_width'],2*self.config['wheel_radius'],color='r', alpha=1)
        wheel_br = Patch.Rectangle((self.config['len_y_leg']-self.config['wheel_width']/2,-self.config['len_x_leg']-self.config['wheel_radius']), self.config['wheel_width'],2*self.config['wheel_radius'],color='r', alpha=1)
        
        wheels = [wheel_fl,wheel_fr,wheel_bl,wheel_br]
        wheels_steer = np.degrees(self._current_steer)

        if plot:
            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1)
            ax.grid(True)
            for n, wheel in enumerate(wheels):
                wheel_centre = (wheel.xy[0] + wheel.get_width()/2, wheel.xy[1] + wheel.get_height()/2)
                t2 = mpl.transforms.Affine2D().rotate_deg_around(*wheel_centre,wheels_steer[n]) + ax.transData
                wheel.set_transform(t2)
                max_arr_len = 0.2
                #scale = self._current_wheel_speed[n]*0.2/max_arr_len
                scale =1
                ax.add_patch(wheel)
                ax.plot(wheel_centre[0],wheel_centre[1])
                if draw_wheels_arrows:
                    arr_len = 1*np.sign(self._current_wheel_speed[n])
                    dy = np.sin(np.radians(wheels_steer[n]))
                    dx = np.cos(np.radians(wheels_steer[n]))
                    ax.arrow(wheel_centre[0],wheel_centre[1], arr_len*-dy, arr_len*dx, color='r')
                if draw_computed_wheel_lin_speed:
                    arr_len = 1
                    angle = np.arctan2(self._v_wheels[n,1],self._v_wheels[n,0])

                    dy = np.sin(angle)
                    dx = np.cos(angle)
                    ax.arrow(wheel_centre[0],wheel_centre[1], arr_len*-dy, arr_len*dx,color='b',linestyle='-.')
            ax.add_patch(chassis)

            #Axis
            if show_frame:
                ax.annotate('x',(0,0),xytext=(0.0, 1.0),xycoords='data',textcoords='data',arrowprops=dict(arrowstyle= '<-',
                                color='black',
                                lw=1,
                                ls='-'))
                ax.annotate('y',(0,0),xytext=(-1.0, 0.0),xycoords='data',textcoords='data',arrowprops=dict(arrowstyle= '<-',
                                color='black',
                                lw=1,
                                ls='-'))
            try:
                ax.plot(-self._last_computed_icr[1], self._last_computed_icr[0],'or')
                ax.plot(-self._last_valid_icr[1], self._last_valid_icr[0],'ob')
                ax.plot([-self._last_computed_icr[1], -self._last_valid_icr[1]], [self._last_computed_icr[0], self._last_valid_icr[0]],'-.',color='k')
            except:
                pass
            ax.set_xlim([-2,2])
            ax.set_ylim([-2,2])
            plt.show()
        
        return (chassis, wheels)

    def _postProcessWheelsCommands(self):
        self._validateWheelsState()
        self._clipWheelsSpeed()

    def _validateWheelsState(self):
        if self._homing_requested:
            pass
        else:
            norm_vs = np.linalg.norm(self._v_wheels, axis=1)
            vel_versors = (self._v_wheels.T/norm_vs).T
            for i in range(len(self._current_wheel_speed)):
                key = self._wheels_mapping[i]
                limits = self.config["wheels_limits"][key]

                if self._current_steer[i] < limits[0]:
                    self._current_steer[i] += np.pi
                if self._current_steer[i] > limits[1]:
                    self._current_steer[i] -= np.pi
                
                if self._current_steer[i] >= limits[0] and self._current_steer[i]<=limits[1]:
                    print("valid steer angle")
                else:
                    print("Invalid steer angle")

                wheel_angle_versors = np.array([np.cos(self._current_steer[i]), np.sin(self._current_steer[i])])

                res = np.dot(vel_versors[i,:2], wheel_angle_versors)
                if np.isclose(res,1):
                    pass
                elif np.isclose(res,-1):
                    self._current_wheel_speed[i]*=-1
                else:
                    print('There is some error')

    def reset(self):
        self._current_steer = np.zeros((4,), dtype=float)
        self._last_steer = np.zeros((4,), dtype=float)

        self._current_wheel_speed = np.zeros((4,),dtype=float)
        self._last_wheel_speed = np.zeros((4,), dtype=float)


        self._v_wheels = np.zeros((4,3), dtype=float)

        self._last_commanded_speed = np.array([0.0,0,0])
        self._last_computed_icr = None
        self._last_valid_icr = None
        self._homing_requested = True

    def kinematicsStep(self, in_high_level_speeds, update_speeds=True, get_output=False):
        self.computeInverseKinematics(in_high_level_speeds)
        self._postProcessWheelsCommands()

        if update_speeds:
            self.computeForwardKinematics(self._current_wheel_speed, self._current_steer)

        if get_output:
            return (self._current_wheel_speed,self._current_steer)
        else:
            return None
    def updateHighLevelSpeeds(self,high_level_speeds):
        pass

if __name__ == '__main__':
    import time
    b = BaseKinematics('pippo')
    #b.computeInverseKinematics([0,0,0])
    #b._validateWheelsState()
    t_now = time.time()
    b.kinematicsStep([0,0,-1])
    print(t_now - time.time())
    b.show(draw_wheels_arrows=True, draw_computed_wheel_lin_speed=True)
    #b._computeICR([1,1,1])
    #b.computeInverseKinematics([1,1,1])
