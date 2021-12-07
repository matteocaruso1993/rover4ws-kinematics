from numpy.core.defchararray import array
from numpy.core.shape_base import vstack
from yaml.loader import SafeLoader
import yaml
import os
import numpy as np
from skspatial.objects import Line
from skspatial.objects import Point
from skspatial.plotting import plot_2d
import matplotlib.pyplot as plt
import matplotlib.patches as Patch
import matplotlib as mpl
import shapely.geometry as ShGeom


class IcrHandler:
    def __init__(self, config_path=None, mode=None) -> None:
        self.config_path = None
        
        self.config = None
        if mode is None:
            self._mode = "outer_ack"
        else:
            self._mode = mode

        self._valid_modes = ["outer_ack","inner_ack","car_like","symm_ack", "full_ack"]
        self.constraints = dict()


        self.data = dict()
        self.data["wheel_fl"] = dict()
        self.data["wheel_bl"] = dict()
        self.data["wheel_fr"] = dict()
        self.data["wheel_br"] = dict()
        self.data["intersections"] = dict()

        self.loadConfig(config_path=config_path)

    def setMode(self, mode):
        if mode not in self._valid_modes:
            return
        else:
            self._mode = mode

    def getMode(self):
        return self._mode

    def _computeOuterIcrIntersections(self) -> None:
        if self.config is None:
            pass
        else:
            #Equation rect wheel left front
            alpha_crit_fl = self.config['angle_crit']
            m2_fl = -1/np.tan(alpha_crit_fl)

            #Equation rect wheel left back
            alpha_crit_bl = -self.config['angle_crit']
            m2_bl = -1/np.tan(alpha_crit_bl)

            A = np.array([
                [m2_fl, -1],
                [m2_bl, -1]
            ])

            B = np.array([
                [m2_fl*self.config["len_x_leg"]-self.config["len_y_leg"]],
                [m2_bl*(-self.config["len_x_leg"])-self.config["len_y_leg"]]
            ])

            x = np.matmul(np.linalg.inv(A),B)

            self.data["wheel_fl"]["direction"] = [-np.sin(alpha_crit_fl), np.cos(alpha_crit_fl)]
            self.data["wheel_fl"]["position"] = [self.config["len_x_leg"],self.config["len_y_leg"]]
            self.data["wheel_fl"]["m"] = m2_fl
            self.data["wheel_fl"]["q"] = self.config["len_y_leg"]
            self.data["wheel_fl"]["x0"] = self.config["len_x_leg"]

            self.data["wheel_bl"]["direction"] = [np.sin(alpha_crit_fl), np.cos(alpha_crit_fl)]
            self.data["wheel_bl"]["position"] = [-self.config["len_x_leg"],self.config["len_y_leg"]]
            self.data["wheel_bl"]["m"] = m2_bl
            self.data["wheel_bl"]["q"] = self.config["len_y_leg"]
            self.data["wheel_bl"]["x0"] = -self.config["len_x_leg"]

            self.data["wheel_fr"]["direction"] = [-np.sin(alpha_crit_fl),-np.cos(alpha_crit_fl)]
            self.data["wheel_fr"]["position"] = [self.config["len_x_leg"],-self.config["len_y_leg"]]
            self.data["wheel_fr"]["m"] = m2_bl
            self.data["wheel_fr"]["q"] = -self.config["len_y_leg"]
            self.data["wheel_fr"]["x0"] = self.config["len_x_leg"]

            self.data["wheel_br"]["direction"] = [np.sin(alpha_crit_fl),-np.cos(alpha_crit_fl)]
            self.data["wheel_br"]["position"] = [-self.config["len_x_leg"],-self.config["len_y_leg"]]
            self.data["wheel_br"]["m"] = m2_fl
            self.data["wheel_br"]["q"] = -self.config["len_y_leg"]
            self.data["wheel_br"]["x0"] = -self.config["len_x_leg"]

            self.data["intersections"]["top"] = x
            self.data["intersections"]["bottom"] = [x[0],-x[1]]

    def _computeInnerIcrIntersections(self) -> None:
        #Top intersection
        self.constraints["inner_ackermann"] = dict()
        self.constraints["inner_ackermann"]['intersections'] = dict()
        self.constraints["inner_ackermann"]['constraints'] = dict()

        #Equation rect wheel left front
        alpha_crit_fl = -self.config['angle_max']
        m2_fl = -1/np.tan(alpha_crit_fl)

        #Equation rect wheel left back
        alpha_crit_bl = +self.config['angle_max']
        m2_bl = -1/np.tan(alpha_crit_bl)

        A = np.array([
            [m2_fl, -1],
            [m2_bl, -1]
        ])

        B = np.array([
            [m2_fl*self.config["len_x_leg"]-self.config["len_y_leg"]],
            [m2_bl*(-self.config["len_x_leg"])-self.config["len_y_leg"]]
        ])

        x = np.squeeze(np.matmul(np.linalg.inv(A),B).T) #Top point
        x1 = [x[0],-x[1]] #Bottom Point


        #Right intersection
        alpha_crit_fl = self.config['angle_crit']
        m2_fl = -1/np.tan(alpha_crit_fl)

        #Equation rect wheel left back
        alpha_crit_fr = -self.config['angle_crit']
        m2_bl = -1/np.tan(alpha_crit_fr)

        A = np.array([
            [m2_fl, -1],
            [m2_bl, -1]
        ])

        B = np.array([
            [m2_fl*self.config["len_x_leg"]-self.config["len_y_leg"]],
            [m2_bl*(self.config["len_x_leg"])+self.config["len_y_leg"]]
        ])
        x2 = np.squeeze(np.matmul(np.linalg.inv(A),B).T) #Right point
        x3 = [-x2[0],x2[1]] #Left point

        self.constraints["inner_ackermann"]['intersections']["top"] = x
        self.constraints["inner_ackermann"]['intersections']["right"] = x2
        self.constraints["inner_ackermann"]['intersections']["bottom"] = x1
        self.constraints["inner_ackermann"]['intersections']["left"] = x3

        coords = []
        
        for key in self.constraints["inner_ackermann"]['intersections'].keys():
            coords.append(tuple(self.constraints["inner_ackermann"]['intersections'][key]))
        
        self.constraints["inner_ackermann"]['constraints'] = ShGeom.Polygon(coords)
        
        #plt.plot(*self.constraints["inner_ackermann"]['constraints'].exterior.xy)
        #plt.show()


    def _computeCarLikeConstraints(self) -> None:
        if self.config is None:
            pass
        else:
            #Equation rect wheel left front
            alpha_crit_fl = self.config['angle_crit']

            self.constraints["car_like"] = dict()
            self.constraints["car_like"]['intersections'] = dict()
            self.constraints["car_like"]['constraints'] = dict()

            x_intercept = self.config["len_y_leg"] + 2*self.config["len_x_leg"]*np.tan(np.pi/2 - alpha_crit_fl)
            self.constraints["car_like"]['intersections']['top'] = [-self.config["len_x_leg"],x_intercept]
            self.constraints["car_like"]['intersections']['bottom'] = [-self.config["len_x_leg"], -x_intercept]

            self.constraints["car_like"]["constraints"]["top"] = Line(point=self.constraints["car_like"]['intersections']['top'],\
                direction=[0,1])

            self.constraints["car_like"]["constraints"]["bottom"] = Line(point=self.constraints["car_like"]['intersections']['bottom'],\
                direction=[0,-1])


    def _computeSymmetricAckermannConstraints(self) -> None:
        if self.config is None:
            pass
        else:
            self.constraints["symm_ackermann"] = dict()
            self.constraints["symm_ackermann"]['intersections'] = dict()
            self.constraints["symm_ackermann"]['constraints'] = dict()


            self.constraints["symm_ackermann"]['intersections']['top'] = np.squeeze(self.data["intersections"]["top"])
            self.constraints["symm_ackermann"]['intersections']['bottom'] = np.squeeze(self.data["intersections"]["bottom"])

            self.constraints["symm_ackermann"]["constraints"]["top"] = Line(point=self.constraints["symm_ackermann"]['intersections']['top'],\
                direction=[0,1])

            self.constraints["symm_ackermann"]["constraints"]["bottom"] = Line(point=self.constraints["symm_ackermann"]['intersections']['bottom'],\
                direction=[0,-1])
    
    def initialize(self):
        self._computeOuterIcrIntersections()
        self._computeInnerIcrIntersections()
        self._computeCarLikeConstraints()
        self._computeSymmetricAckermannConstraints()

    def validateIcr(self,icr):
        self._last_commanded_icr = icr
        valid, new_pt, d = self._isIcrValid(icr)
        self._last_projected_icr = new_pt
        self._last_valid = valid

        if not valid:
            print("Warning: Commanded ICR doesn't match constraints")

    def getProjectedIcr(self):
        return self._last_projected_icr

    def _isIcrValid(self,icr):
        #Check if belongs to top region or bottom region
        valid = False
        p = Point(icr)
        p_sh = ShGeom.Point(tuple(icr))

        modes = None
        if self._mode == "full_ack":
            modes = ['outer_ack','inner_ack']
        else:
            modes = [self._mode]

        points_elected = []
        distances_elected = []

        for mode in modes:

            if mode == "outer_ack":
                if icr[1] >=0:
                    if icr[1] >= self.data["wheel_fl"]["m"]*(icr[0] - self.data["wheel_fl"]["x0"]) + self.data["wheel_fl"]["q"]\
                        and icr[1] >= self.data["wheel_bl"]["m"]*(icr[0] - self.data["wheel_bl"]["x0"]) + self.data["wheel_bl"]["q"]:
                        valid = True
                        dist = 0
                        points_elected.append(p)
                        distances_elected.append(dist)
                    else:
                        valid = False
                    
                        l = np.linalg.norm(icr)
                        line1 = Line(point=self.data["wheel_fl"]["position"], direction=self.data["wheel_fl"]["direction"])
                        line2 = Line(point=self.data["wheel_bl"]["position"], direction=self.data["wheel_bl"]["direction"])
                        lines = [line1,line2]

                        valid_pt = None
                        dist = float("inf")

                        """
                        for line in lines:
                            proj_pt = line.project_point(icr)
                            d_tmp = p.distance_point(proj_pt)
                            if d_tmp < dist:
                                valid_pt = proj_pt
                                dist = d_tmp
                        """
                        if icr[1] < self.data["wheel_fl"]["m"]*(icr[0] - self.data["wheel_fl"]["x0"]) + self.data["wheel_fl"]["q"]\
                            and icr[1] < self.data["wheel_bl"]["m"]*(icr[0] - self.data["wheel_bl"]["x0"]) + self.data["wheel_bl"]["q"]:
                            valid_pt = Point(np.squeeze(self.data['intersections']['top'].T))
                            dist = p.distance_point(valid_pt)

                        if icr[0] >= (icr[1]- self.data["wheel_fl"]["q"])/self.data["wheel_fl"]["m"] + self.data["wheel_fl"]["x0"]:
                            #Project on line 2
                            valid_pt = line2.project_point(icr)
                            dist = p.distance_point(valid_pt)
                        
                        if icr[0] <= (icr[1]- self.data["wheel_bl"]["q"])/self.data["wheel_bl"]["m"] + self.data["wheel_bl"]["x0"]:
                            #Project on line 1
                            valid_pt = line1.project_point(icr)
                            dist = p.distance_point(valid_pt)

                        points_elected.append(valid_pt)
                        distances_elected.append(dist)

                if icr[1] <0:
                    if icr[1] <= self.data["wheel_fr"]["m"]*(icr[0] - self.data["wheel_fr"]["x0"]) + self.data["wheel_fr"]["q"]\
                        and icr[1] <= self.data["wheel_br"]["m"]*(icr[0] - self.data["wheel_br"]["x0"]) + self.data["wheel_br"]["q"]:
                        valid = True
                        dist = 0
                        points_elected.append(p)
                        distances_elected.append(dist)
                    else:
                        valid = False
                    
                        #l = np.linalg.norm(icr)
                        line1 = Line(point=self.data["wheel_fr"]["position"], direction=self.data["wheel_fr"]["direction"])
                        line2 = Line(point=self.data["wheel_br"]["position"], direction=self.data["wheel_br"]["direction"])
                        lines = [line1,line2]

                        valid_pt = None
                        dist = float("inf")
                        """
                        for line in lines:
                            proj_pt = line.project_point(icr)
                            d_tmp = p.distance_point(proj_pt)
                            if d_tmp < dist:
                                valid_pt = proj_pt
                                dist = d_tmp
                        """
                        if icr[1] > self.data["wheel_fr"]["m"]*(icr[0] - self.data["wheel_fr"]["x0"]) + self.data["wheel_fr"]["q"]\
                            and icr[1] > self.data["wheel_br"]["m"]*(icr[0] - self.data["wheel_br"]["x0"]) + self.data["wheel_br"]["q"]:
                            valid_pt = Point(np.squeeze(self.data['intersections']['bottom']))
                            dist = p.distance_point(valid_pt)

                        if icr[0] >= (icr[1]- self.data["wheel_fr"]["q"])/self.data["wheel_fr"]["m"] + self.data["wheel_fr"]["x0"]:
                            #Project on line 2
                            valid_pt = line2.project_point(icr)
                            dist = p.distance_point(valid_pt)
                        
                        if icr[0] <= (icr[1]- self.data["wheel_br"]["q"])/self.data["wheel_br"]["m"] + self.data["wheel_br"]["x0"]:
                            #Project on line 1
                            valid_pt = line1.project_point(icr)
                            dist = p.distance_point(valid_pt)
                        
                        points_elected.append(valid_pt)
                        distances_elected.append(dist)

            elif mode == 'inner_ack':
                #Check if point is inside polygon
                poly = self.constraints["inner_ackermann"]['constraints']
                valid = poly.contains(p_sh)
                if valid:
                    dist = 0
                    points_elected.append(p)
                    distances_elected.append(dist)
                else:
                    pol_ext = ShGeom.LinearRing(poly.exterior.coords)
                    d = pol_ext.project(p_sh)
                    p = pol_ext.interpolate(d)
                    closest_point_coords = list(p.coords)[0]
                    points_elected.append(Point(closest_point_coords))
                    distances_elected.append(d)

            elif mode == 'car_like':
                if icr[1]>=0:
                    #We are on the top
                    line = self.constraints["car_like"]['constraints']["top"]
                else:
                    line = self.constraints["car_like"]['constraints']["bottom"]

                proj_pt = line.project_point(icr)
                if icr[1] >= 0 and proj_pt[1] < self.constraints["car_like"]['intersections']["top"][1]:
                    proj_pt = Point(self.constraints["car_like"]['intersections']["top"])
                elif icr[1] <0 and proj_pt[1] > self.constraints["car_like"]['intersections']["bottom"][1]:
                    proj_pt = Point(self.constraints["car_like"]['intersections']["bottom"])
                if np.array_equal(proj_pt,p):
                    valid = True
                    d = 0
                else:
                    d = p.distance_point(proj_pt)



                points_elected.append(proj_pt)
                distances_elected.append(d)

            elif mode == 'symm_ack':
                if icr[1]>=0:
                    #We are on the top
                    line = self.constraints["symm_ackermann"]['constraints']["top"]
                else:
                    line = self.constraints["symm_ackermann"]['constraints']["bottom"]

                proj_pt = line.project_point(icr)
                if icr[1] >= 0 and proj_pt[1] < self.constraints["symm_ackermann"]['intersections']["top"][1]:
                    proj_pt = Point(self.constraints["symm_ackermann"]['intersections']["top"])
                elif icr[1] <0 and proj_pt[1] > self.constraints["symm_ackermann"]['intersections']["bottom"][1]:
                    proj_pt = Point(self.constraints["symm_ackermann"]['intersections']["bottom"])
                
                if np.array_equal(proj_pt,p):
                    valid = True
                    d = 0
                else:
                    d = p.distance_point(proj_pt)

                points_elected.append(proj_pt)
                distances_elected.append(d)

        idx = np.argmin(distances_elected)
        return valid, points_elected[idx], distances_elected[idx]            

    def loadConfig(self,config_path):
        if config_path is None:
            par_dir = os.path.dirname(__file__)
            config_filename = os.path.abspath(os.path.join(par_dir,'..','..','config','config.yaml'))
            print(config_filename)

            try:
                with open(config_filename,'r') as f:
                    data = yaml.load(f, Loader=SafeLoader)
                    self.config = data
                    self.config_path = config_filename

            except:
                pass


    def show(self, input_angles,plot = True, plot_axis=False):
        chassis_scale = 1
        chassis = Patch.Rectangle((-chassis_scale*self.config['len_x_leg'],-chassis_scale*self.config['len_y_leg']), chassis_scale*2*self.config['len_x_leg'],chassis_scale*2*self.config['len_y_leg'],color='k', alpha=0.3)
        wheel_fl = Patch.Rectangle((-self.config['len_x_leg']-self.config['wheel_radius']/2,self.config['len_y_leg']-self.config['wheel_width']), self.config['wheel_radius'],2*self.config['wheel_width'],color='r', alpha=1)
        wheel_fr = Patch.Rectangle((self.config['len_x_leg']-self.config['wheel_radius']/2,self.config['len_y_leg']-self.config['wheel_width']), self.config['wheel_radius'],2*self.config['wheel_width'],color='r', alpha=1)
        wheel_bl = Patch.Rectangle((-self.config['len_x_leg']-self.config['wheel_radius']/2,-self.config['len_y_leg']-self.config['wheel_width']), self.config['wheel_radius'],2*self.config['wheel_width'],color='r', alpha=1)
        wheel_br = Patch.Rectangle((self.config['len_x_leg']-self.config['wheel_radius']/2,-self.config['len_y_leg']-self.config['wheel_width']), self.config['wheel_radius'],2*self.config['wheel_width'],color='r', alpha=1)
        
        wheels = [wheel_fl,wheel_fr,wheel_bl,wheel_br]
        wheels_steer = np.degrees(input_angles)

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
                #ax.arrow(wheel_centre[0],wheel_centre[1], scale*np.sin(-np.radians(wheels_steer[n])), scale*np.cos(np.radians(wheels_steer[n])))

            
            ax.add_patch(chassis)

            #Axis
            if plot_axis:
                ax.annotate('x',(0,0),xytext=(1.0, 0.0),xycoords='data',textcoords='data',arrowprops=dict(arrowstyle= '<-',
                                color='black',
                                lw=1,
                                ls='-'))
                ax.annotate('y',(0,0),xytext=(0.0, 10.0),xycoords='data',textcoords='data',arrowprops=dict(arrowstyle= '<-',
                                color='black',
                                lw=1,
                                ls='-'))

            #ax.scatter(self.data["intersections"]["top"][0],self.data["intersections"]["top"][1])
            #ax.scatter(self.data["intersections"]["bottom"][0],self.data["intersections"]["bottom"][1])

            data_fill_top = np.array([self.data["intersections"]["top"][0],self.data["intersections"]["top"][1]]).T
            data_fill_bottom = np.array(self.data["intersections"]["bottom"]).T

            l_len = 10
            for key in self.data.keys():
                if "wheel_" in key:
                    pt = self.data[key]["position"]
                    dir = self.data[key]["direction"]
                    pt_to_plot = np.vstack((pt,pt+10*np.array(dir)))
                    if key == "wheel_fl" or key =="wheel_bl":
                        data_fill_top = np.vstack((data_fill_top,pt+l_len*np.array(dir)))
                    else:
                        data_fill_bottom = np.vstack((data_fill_bottom,pt+l_len*np.array(dir)))
                    ax.plot(pt_to_plot[:,0],pt_to_plot[:,1],"-k")
                
            ax.fill(data_fill_top[:,0],data_fill_top[:,1], alpha=0.5,color='g')
            ax.fill(data_fill_bottom[:,0],data_fill_bottom[:,1], alpha=0.5,color='g')



            #ax.plot(self._last_computed_icr[0], self._last_computed_icr[1],'or')
            ax.set_xlim([-3,3])
            ax.set_ylim([-3,3])
            ax.axis("equal")
            

            ax.scatter(self._last_commanded_icr[0],self._last_commanded_icr[1],label='commanded ICR',color='r')
            ax.scatter(self._last_projected_icr[0], self._last_projected_icr[1],label='projected ICR', color='b')
            if not self._last_valid:
                data = np.array([self._last_commanded_icr,self._last_projected_icr])
                ax.plot(data[:,0],data[:,1],'-.')

            for mode in self.constraints.keys():
                if mode == "inner_ackermann":
                    x,y = self.constraints[mode]["constraints"].exterior.xy
                    ax.fill(x,y,alpha=0.5,color='g')
                    ax.plot(x,y,color='k')
                else:
                    for key in self.constraints[mode]["constraints"]:
                        line = self.constraints[mode]["constraints"][key]
                        tmp = np.empty((0,2),dtype=float)
                        tmp = np.vstack((tmp, line.point))
                        tmp = np.vstack((tmp, np.array(line.point) + l_len*np.array(line.direction)))
                        ax.plot(tmp[:,0],tmp[:,1],'-.',label = '%s %s constraint'%(mode,key),color='k')

            ax.legend()
            plt.show()

    def _updateOriginalAxis(self,ax):
        pass

    
if __name__ == '__main__':
    icr = IcrHandler()
    icr.initialize()
    #icr.setMode("outer_ack")
    icr.setMode("full_ack")
    icr.validateIcr([0,0])
    icr.show([0,0,0,0])