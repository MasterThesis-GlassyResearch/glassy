#!/usr/bin/env python


from glassy_msgs.msg import PathInfo



import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches




class DubinsGenerator():
    def __init__(self, r_min):
        """
        Class
        """

        self.r_min = r_min
        

        self.c1_r=None
        self.c1_l=None
        self.c2_r=None
        self.c2_l=None

        self.p1 = None
        self.p2 = None
        self.yaw1 = None
        self.yaw2 = None

        self.RSR = 0
        self.LSL = 1
        self.RSL = 2
        self.LSR = 3
        self.RLR = 4
        self.LRL = 5

        self.p_fin= np.array([0,0])
        self.p_init = np.array([0,0])
        self.heading_fin = 0
        self.heading_init = 0

        self.full_path_type = []
        self.full_path_info = []

        self.total_length = 0



    def wrapTo2Pi(self, ang):
        ang = ang%(2*np.pi)
        if ang<0:
            ang = ang+2*np.pi
        return ang

    def wrapToPi(self, ang):
        ang = ang%(np.pi)
        return ang
    

    def DubinsInterpolator(self, waypoints):
        self.total_length = 0
        num_waypoints = len(waypoints)
        self.p_init = waypoints[0][:2]
        self.heading_init = waypoints[0][2]

        self.p_fin = waypoints[-1][:2]
        self.heading_fin = waypoints[-1][2]
        self.full_path_info.clear()
        self.full_path_type.clear()
        for i in range(0, num_waypoints-1):
            self.waypointsDef(waypoints[i], waypoints[i+1])
            self.total_length = self.total_length+self.DubinPathFinder()  

    def DubinPathFinder(self):
        self.findCenterCircles()
        checkCCC= False
        if(np.linalg.norm(self.p1-self.p2)<4*self.r_min):
            checkCCC =True

        if(checkCCC):
            lengths_array = np.zeros(6)

            lengths_array[self.RLR] = self.RLR_dist_calc()
            lengths_array[self.LRL] = self.LRL_dist_calc()
        else:
            lengths_array = np.zeros(4)
        

        lengths_array[self.RSR] = self.RSR_dist_calc()
        lengths_array[self.LSL] = self.LSL_dist_calc()
        lengths_array[self.RSL] = self.RSL_dist_calc()
        lengths_array[self.LSR] = self.LSR_dist_calc()

        best_length = np.nanargmin(lengths_array)

        match best_length:
            case self.LSL:
                self.LSL_path()
            case self.RSL:
                self.RSL_path()
            case self.LSR:
                self.LSR_path()
            case self.RLR:
                self.RLR_path()
                
            case self.LRL:
                self.LRL_path()
            case _:
                #default is RSR
                self.RSR_path()
        return lengths_array[best_length]
    
    
    def findCenterCircles(self):
        self.c1_r = self.p1 + self.r_min*np.array([np.cos(self.yaw1-np.pi/2), np.sin(self.yaw1-np.pi/2)])
        self.c1_l = self.p1 + self.r_min*np.array([np.cos(self.yaw1+np.pi/2), np.sin(self.yaw1+np.pi/2)])
        self.c2_r = self.p2 + self.r_min*np.array([np.cos(self.yaw2-np.pi/2), np.sin(self.yaw2-np.pi/2)])
        self.c2_l = self.p2 + self.r_min*np.array([np.cos(self.yaw2+np.pi/2), np.sin(self.yaw2+np.pi/2)])

        
    def plotCenterCircles(self):

        ax = plt.gca()
        ax.set_aspect('equal')
        plt.grid(True)
        plt.arrow(self.p1[0], self.p1[1], 0.2*np.cos(self.yaw1), 0.2*np.sin(self.yaw1), head_width=0.2)
        plt.arrow(self.p2[0], self.p2[1], 0.2*np.cos(self.yaw2), 0.2*np.sin(self.yaw2), head_width= 0.2)
        circles = []
        circles.append(plt.Circle(self.c1_r, self.r_min, color='b', fill=False))
        circles.append(plt.Circle(self.c1_l, self.r_min, color='b', fill=False))
        circles.append(plt.Circle(self.c2_r, self.r_min, color='b', fill=False))
        circles.append(plt.Circle(self.c2_l, self.r_min, color='b', fill=False))
        for circ in circles:
            ax.add_patch(circ)
        plt.show()

    def waypointsDef(self, wp1, wp2):
        self.p1 = wp1[:2]
        self.p2 = wp2[:2]
        self.yaw1 = self.wrapTo2Pi(wp1[2])
        self.yaw2 = self.wrapTo2Pi(wp2[2])


    #-----------------------------------
    #------- PATH Cost calculation
    #-----------------------------------

    def RSR_dist_calc(self):
        line_dist = np.linalg.norm(self.c2_r-self.c1_r)
        line_ang = self.wrapTo2Pi(np.arctan2(self.c2_r[1]-self.c1_r[1], self.c2_r[0]-self.c1_r[0]))
        angular_dist = self.r_min*( self.wrapTo2Pi(self.yaw1-line_ang)+self.wrapTo2Pi(line_ang-self.yaw2))
        return line_dist+angular_dist
    
    def LSL_dist_calc(self):
        line_dist = np.linalg.norm(self.c2_l-self.c1_l)
        line_ang = self.wrapTo2Pi(np.arctan2(self.c2_l[1]-self.c1_l[1], self.c2_l[0]-self.c1_l[0]))
        angular_dist = self.r_min*( self.wrapTo2Pi(line_ang-self.yaw1)+self.wrapTo2Pi(self.yaw2-line_ang))
        return line_dist+angular_dist

    def RSL_dist_calc(self):
        center_circle_dist = np.linalg.norm(self.c2_l-self.c1_r)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_l[1]-self.c1_r[1], self.c2_l[0]-self.c1_r[0]))
        dist_squared = center_circle_dist**2 - 4*self.r_min**2
        ratio_adj_ov_hyp = 2*self.r_min/center_circle_dist
        if(dist_squared<0 or ratio_adj_ov_hyp<-1 or ratio_adj_ov_hyp>1):
            return np.nan
        line_dist = np.sqrt(dist_squared)
        alpha = self.wrapTo2Pi(np.arccos(ratio_adj_ov_hyp))
        escape_angle = self.wrapTo2Pi(center_line_angle- np.pi/2 + alpha)
        angle_dist = self.r_min*(self.wrapTo2Pi( self.yaw1-escape_angle) + self.wrapTo2Pi(self.yaw2 - escape_angle))
        return line_dist + angle_dist



    def LSR_dist_calc(self):
        center_circle_dist = np.linalg.norm(self.c2_r-self.c1_l)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_r[1]-self.c1_l[1], self.c2_r[0]-self.c1_l[0]))

        dist_squared = center_circle_dist**2 - 4*self.r_min**2
        ratio_adj_ov_hyp = 2*self.r_min/center_circle_dist
        if(dist_squared<0 or ratio_adj_ov_hyp<-1 or ratio_adj_ov_hyp>1):
            return np.nan
        line_dist = np.sqrt(dist_squared)
        alpha = self.wrapTo2Pi(np.arccos(ratio_adj_ov_hyp))

        escape_angle = self.wrapTo2Pi(center_line_angle+ np.pi/2 - alpha)
        angle_dist = self.r_min*(self.wrapTo2Pi(escape_angle - self.yaw1) + self.wrapTo2Pi( escape_angle- self.yaw2))
        return line_dist + angle_dist

    def LRL_dist_calc(self):
        center_circle_dist = np.linalg.norm(self.c2_l-self.c1_l)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_l[1]-self.c1_l[1], self.c2_l[0]-self.c1_l[0]))
        try:
            alpha = self.wrapTo2Pi(np.arcsin(center_circle_dist/(2*2*self.r_min)))
        except:
            return np.nan
        beta = np.pi/2-alpha
        angle_c1 = self.wrapTo2Pi(self.wrapTo2Pi(center_line_angle +beta+np.pi/2) -self.yaw1)
        angle_c2 = 2*np.pi- 2*alpha
        angle_c3 = self.wrapTo2Pi(self.yaw2-self.wrapTo2Pi(center_line_angle -beta -np.pi/2))
        return self.r_min*(angle_c1+angle_c2+angle_c3)

    def RLR_dist_calc(self):
        center_circle_dist = np.linalg.norm(self.c2_r-self.c1_r)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_r[1]-self.c1_r[1], self.c2_r[0]-self.c1_r[0]))
        try:
            alpha = self.wrapTo2Pi(np.arcsin(center_circle_dist/(2*2*self.r_min)))
        except:
            return np.nan
        beta = np.pi/2-alpha
        angle_c1 = self.wrapTo2Pi(self.yaw1-self.wrapTo2Pi(center_line_angle -beta-np.pi/2))
        angle_c2 = 2*np.pi- 2*alpha
        angle_c3 = self.wrapTo2Pi(self.wrapTo2Pi(center_line_angle +beta +np.pi/2)-self.yaw2)
        return self.r_min*(angle_c1+angle_c2+angle_c3)
    
    
    #-----------------------------------
    #------- PATH creation and append
    #-----------------------------------

    def RSR_path(self):
        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.p1)
        self.full_path_info.append(self.c1_r)
        line_ang = self.wrapTo2Pi(np.arctan2(self.c2_r[1]-self.c1_r[1], self.c2_r[0]-self.c1_r[0]))

        self.full_path_info.append(np.array([-self.wrapTo2Pi(self.yaw1-line_ang)]))

        self.full_path_type.append(PathInfo.STRAIGHT)
        self.full_path_info.append(self.c1_r + self.r_min* np.array([np.cos(line_ang+np.pi/2), np.sin(line_ang+np.pi/2)]))
        self.full_path_info.append(self.c2_r + self.r_min* np.array([np.cos(line_ang+np.pi/2), np.sin(line_ang+np.pi/2)]))

        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.c2_r + self.r_min* np.array([np.cos(line_ang+np.pi/2), np.sin(line_ang+np.pi/2)]))
        self.full_path_info.append(self.c2_r)
        self.full_path_info.append(np.array([-self.wrapTo2Pi(line_ang-self.yaw2)]))


    def LSL_path(self):
        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.p1)
        self.full_path_info.append(self.c1_l)
        line_ang = self.wrapTo2Pi(np.arctan2(self.c2_l[1]-self.c1_l[1], self.c2_l[0]-self.c1_l[0]))

        self.full_path_info.append(np.array([self.wrapTo2Pi(line_ang-self.yaw1)]))

        self.full_path_type.append(PathInfo.STRAIGHT)
        self.full_path_info.append(self.c1_l + self.r_min* np.array([np.cos(line_ang-np.pi/2), np.sin(line_ang-np.pi/2)]))
        self.full_path_info.append(self.c2_l + self.r_min* np.array([np.cos(line_ang-np.pi/2), np.sin(line_ang-np.pi/2)]))

        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.c2_l + self.r_min* np.array([np.cos(line_ang-np.pi/2), np.sin(line_ang-np.pi/2)]))
        self.full_path_info.append(self.c2_l)
        self.full_path_info.append(np.array([self.wrapTo2Pi(self.yaw2-line_ang)]))


    def LSR_path(self):

        center_circle_dist = np.linalg.norm(self.c2_r-self.c1_l)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_r[1]-self.c1_l[1], self.c2_r[0]-self.c1_l[0]))
        # if()
        alpha = self.wrapTo2Pi(np.arccos(2*self.r_min/center_circle_dist))
        escape_angle = self.wrapTo2Pi(center_line_angle+ np.pi/2 - alpha)

        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.p1)
        self.full_path_info.append(self.c1_l)

        self.full_path_info.append(np.array([self.wrapTo2Pi(escape_angle-self.yaw1)]))

        self.full_path_type.append(PathInfo.STRAIGHT)
        self.full_path_info.append(self.c1_l + self.r_min* np.array([np.cos(escape_angle-np.pi/2), np.sin(escape_angle-np.pi/2)]))
        self.full_path_info.append(self.c2_r + self.r_min* np.array([np.cos(escape_angle+np.pi/2), np.sin(escape_angle+np.pi/2)]))

        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.c2_r + self.r_min* np.array([np.cos(escape_angle+np.pi/2), np.sin(escape_angle+np.pi/2)]))
        self.full_path_info.append(self.c2_r)
        self.full_path_info.append(np.array([-self.wrapTo2Pi(escape_angle-self.yaw2)]))

    def RSL_path(self):

        center_circle_dist = np.linalg.norm(self.c2_l-self.c1_r)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_l[1]-self.c1_r[1], self.c2_l[0]-self.c1_r[0]))
        alpha = np.arccos(2*self.r_min/center_circle_dist)
        escape_angle = self.wrapTo2Pi(center_line_angle- np.pi/2 + alpha)

        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.p1)
        self.full_path_info.append(self.c1_r)

        self.full_path_info.append(np.array([-self.wrapTo2Pi(self.yaw1-escape_angle)]))

        self.full_path_type.append(PathInfo.STRAIGHT)
        self.full_path_info.append(self.c1_r + self.r_min* np.array([np.cos(escape_angle+np.pi/2), np.sin(escape_angle+np.pi/2)]))
        self.full_path_info.append(self.c2_l + self.r_min* np.array([np.cos(escape_angle-np.pi/2), np.sin(escape_angle-np.pi/2)]))

        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.c2_l + self.r_min* np.array([np.cos(escape_angle-np.pi/2), np.sin(escape_angle-np.pi/2)]))
        self.full_path_info.append(self.c2_l)
        self.full_path_info.append(np.array([self.wrapTo2Pi(self.yaw2-escape_angle)]))



    def RLR_path(self):

        center_circle_dist = np.linalg.norm(self.c2_r-self.c1_r)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_r[1]-self.c1_r[1], self.c2_r[0]-self.c1_r[0]))
        # alpha = np.arccos(2*self.r_min/center_circle_dist)

        alpha = self.wrapTo2Pi(np.arcsin(center_circle_dist/(2*2*self.r_min)))
        beta = self.wrapTo2Pi(np.pi/2-alpha)
        escape_angle = self.wrapTo2Pi(center_line_angle-beta-np.pi/2)

        #first arc
        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.p1)
        self.full_path_info.append(self.c1_r)
        self.full_path_info.append(np.array([-self.wrapTo2Pi(self.yaw1-escape_angle)]))

        # second arc 
        self.full_path_type.append(PathInfo.ARC)
        center3 = self.c1_r+2*self.r_min*np.array([np.cos(center_line_angle-beta), np.sin(center_line_angle-beta)])
        tang1 = self.c1_r+self.r_min*np.array([np.cos(center_line_angle-beta), np.sin(center_line_angle-beta)])
        self.full_path_info.append(tang1)
        self.full_path_info.append(center3)
        self.full_path_info.append(np.array([2*np.pi- 2*alpha]))

        #third arc
        self.full_path_type.append(PathInfo.ARC)
        entrance_angle = center_line_angle-beta+2*np.pi-2*alpha
        self.full_path_info.append(self.c2_r + self.r_min* np.array([np.cos(entrance_angle), np.sin(entrance_angle)]))
        self.full_path_info.append(self.c2_r)
        self.full_path_info.append(np.array([-self.wrapTo2Pi(self.wrapTo2Pi(-np.pi/2+entrance_angle)-self.yaw2)]))


    def LRL_path(self):

        center_circle_dist = np.linalg.norm(self.c2_l-self.c1_l)
        center_line_angle = self.wrapTo2Pi(np.arctan2(self.c2_l[1]-self.c1_l[1], self.c2_l[0]-self.c1_l[0]))
        # alpha = np.arccos(2*self.r_min/center_circle_dist)

        alpha = self.wrapTo2Pi(np.arcsin(center_circle_dist/(2*2*self.r_min)))
        beta = self.wrapTo2Pi(np.pi/2-alpha)
        escape_angle = self.wrapTo2Pi(center_line_angle+beta+np.pi/2)

        #first arc
        self.full_path_type.append(PathInfo.ARC)
        self.full_path_info.append(self.p1)
        self.full_path_info.append(self.c1_l)
        self.full_path_info.append(np.array([self.wrapTo2Pi(escape_angle-self.yaw1)]))

        # second arc 
        self.full_path_type.append(PathInfo.ARC)
        center3 = self.c1_l+2*self.r_min*np.array([np.cos(center_line_angle+beta), np.sin(center_line_angle+beta)])
        tang1 = self.c1_l+self.r_min*np.array([np.cos(center_line_angle+beta), np.sin(center_line_angle+beta)])
        self.full_path_info.append(tang1)
        self.full_path_info.append(center3)
        self.full_path_info.append(np.array([-(2*np.pi- 2*alpha)]))

        #third arc
        self.full_path_type.append(PathInfo.ARC)
        entrance_angle = center_line_angle+beta-2*np.pi+2*alpha
        self.full_path_info.append(self.c2_l + self.r_min* np.array([np.cos(entrance_angle), np.sin(entrance_angle)]))
        self.full_path_info.append(self.c2_l)
        self.full_path_info.append(np.array([self.wrapTo2Pi(self.yaw2-self.wrapTo2Pi(entrance_angle+np.pi/2))]))
    

    def full_path_plot(self):
        ax = plt.gca()
        ax.set_aspect('equal')
        plt.title("Dubins Path, r: {:.2f}m, len: {:.2f}m".format(self.r_min, self.total_length))
        plt.xlabel("x(m)")
        plt.ylabel("y(m)")
        plt.grid(True)
        plt.arrow(self.p_init[0], self.p_init[1], 0.02*np.cos(self.heading_init), 0.02*np.sin(self.heading_init), head_width=0.2).set_facecolor('g')
        plt.arrow(self.p_fin[0], self.p_fin[1], 0.02*np.cos(self.heading_fin), 0.02*np.sin(self.heading_fin), head_width= 0.2).set_facecolor('y')
        
        # three steps for a circle 
        # two steps for a line
        i = 0
        col=None
        for segment in self.full_path_type:
            if segment==PathInfo.ARC:
                theta1 = np.rad2deg(np.arctan2(self.full_path_info[i][1]- self.full_path_info[i+1][1], self.full_path_info[i][0]- self.full_path_info[i+1][0]))

                if(self.full_path_info[i+2][0]>0):
                    theta2 = theta1+ np.rad2deg(self.full_path_info[i+2][0])
                    col = 'm'
                else:
                    temp = theta1
                    theta1 = theta1+ np.rad2deg(self.full_path_info[i+2][0])
                    theta2 = temp
                    col = 'r'

                arc = mpatches.Arc(self.full_path_info[i+1], self.r_min*2, self.r_min*2, angle=0, theta1=theta1, theta2=theta2)
                arc.set_edgecolor(col)
                ax.add_patch(arc)
                i = i+3
            else:
                plt.plot([self.full_path_info[i][0], self.full_path_info[i+1][0]], [self.full_path_info[i][1], self.full_path_info[i+1][1]], 'b')
                i = i+2
                pass
            

def main(args=None):

    test = DubinsGenerator(1)
    waypoints = [np.array([0,0,0]), np.array([10,0, 0]), np.array([10,-13, np.pi]), np.array([-4,0, -np.pi/2]), np.array([0,0, 0])]
    test.DubinsInterpolator(waypoints)
    test.full_path_plot()



    plt.show()

if __name__ == '__main__':
    main()