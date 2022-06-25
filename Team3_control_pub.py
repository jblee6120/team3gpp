# -*- coding:utf-8 -*-
#! /usr/bin/env python3

from math import atan2, cos, degrees, radians, sin

import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray


class CPP:
    def __init__(self):
        rospy.init_node("P")
        self.P = rospy.Publisher("controller", Float32MultiArray, queue_size=1)
        self.first_imu = float(input("first imu? : "))
        self.tj=[] 
        self.heading_imu = self.first_imu
        self.heading_pp = 0
        self.P_veh = [0,0]
        self.speed = 0.6*480
        self.pre_veh = [0,0]
        self.xpath = []
        self.ypath = []
        # from planning                                                                                                                                                                                                                                                                                                                                                                                                                                            
        self.get_traj = False
        self.is_slow = 0 
        self.is_stop = False
        self.sub = rospy.Subscriber('/path', Float32MultiArray, self.PathCallback)
        self.sub2 = rospy.Subscriber('/xyheading', Float32MultiArray, self.XYHeadingCallback)
        # self.sub3 = rospy.Subscriber('/cont_vision', Float32MultiArray, self.VisionCallback)

    def trans_head_imu_to_pp(self, imu):
        if self.first_imu>0:
            self.heading_pp = imu - self.first_imu
            if self.heading_pp<-180:
                self.heading_pp = self.heading_pp + 360
        
        if self.first_imu<0:
            self.heading_pp = imu - self.first_imu
            if self.heading_pp>180:
                self.heading_pp = self.heading_pp - 360

    def PathCallback(self, msg):
        for i in range (int(len(msg.data)/2)):
            self.tj.append([float(msg.data[2*i]), float(msg.data[2*i+1])])
        self.get_traj = True
    
    def XYHeadingCallback(self, msg): # [x, y, heading, vision]
        self.P_veh[0] = float(msg.data[0])
        self.P_veh[1] = float(msg.data[1])
        self.xpath.append(self.P_veh[0])
        self.ypath.append(self.P_veh[1])
        self.heading_imu = float(msg.data[2])
        if(msg.data[3]==1):
            self.is_slow=1
        


    def divide_traj(self):
        # self.P_veh = self.tj[]
        self.P_veh=[self.tj[0][0], self.tj[0][1]]
        self.div_tj=[]
        self.div_unit= 0.5 
        for i in range(len(self.tj)-1):
            for j in range(round(self.get_distance(self.tj[i+1],self.tj[i])/self.div_unit)):
                self.div_tj.append([self.tj[i][0]+(self.tj[i+1][0]-self.tj[i][0])*j*self.div_unit/ \
                    self.get_distance(self.tj[i+1],self.tj[i]),self.tj[i][1]+(self.tj[i+1][1] \
                        -self.tj[i][1])*j*self.div_unit/self.get_distance(self.tj[i+1],self.tj[i])]) 
        self.div_tj.append(self.tj[-1]) 
        
        for i in range(len(self.div_tj)):
            plt.plot(self.div_tj[i][0], self.div_tj[i][1], 'bo')
        for i in range(len(self.tj)):
            plt.plot(self.tj[i][0], self.tj[i][1], 'ro')
        print(self.tj)
        print('\n\n',self.div_tj)
        plt.show()

    def get_distance(self, A, B): 
        distance=((A[0]-B[0])**2+(A[1]-B[1])**2)**(1/2)
        return distance

    def pure_pursuit(self):
        self.trans_head_imu_to_pp(self.heading_imu)
        print('pp heading:', self.heading_pp)
        index=0
        min_dis=10 
        for i in range(len(self.div_tj)): 
            distance=self.get_distance(self.div_tj[i], self.P_veh)
            if distance<=min_dis:  
                min_dis=distance
                self.pre_veh=[self.div_tj[i][0], self.div_tj[i][1]] 
                index=i

        lookahead=3
        index_goal=index+lookahead
        if index_goal>len(self.div_tj)-1:
            index_goal=len(self.div_tj)-1



        Ld=(self.get_distance(self.div_tj[index_goal], self.P_veh))
        L=0.58 
        
        dif_x=self.div_tj[index_goal][0]-self.P_veh[0]
        dif_y=self.div_tj[index_goal][1]-self.P_veh[1]
        beta=atan2(dif_x, dif_y)
       
        alpha=radians(self.heading_pp)-beta 
       

        e=sin(alpha)*Ld 

        self.steering=degrees(atan2(2*L*e,(Ld)**2)) 
        if self.steering>=30:
            self.steering=30
        elif self.steering<=-30:
            self.steering=-30

        print('index_goal:',index_goal)
        print('Ld:',Ld)
        print('beta:',beta)
        print('alpha:',alpha)
        print('e:',e)
        print('steering:', self.steering)

    def run(self):
        inin = Float32MultiArray()        
        if self.is_slow==1:
            while(self.speed!=0):
                self.speed -= 50 
                if self.speed < 0:
                    self.speed = 0
                    self.is_stop = True 
                self.pure_pursuit()
                inin.data=[]
                inin.data.append(self.speed)
                inin.data.append(1585+self.steering*9
                )
                inin.data.append(0)
                inin.data.append(0)
                self.P.publish(inin)
        
        self.pure_pursuit()
        inin.data=[]
        inin.data.append(self.speed)
        inin.data.append(1585+self.steering*10)
        inin.data.append(0)
        inin.data.append(0)
        
        self.P.publish(inin)
        if self.get_distance(self.P_veh, self.div_tj[len(self.div_tj)-1])<=0.6:
            self.speed=0
            self.is_stop = True
           
        

if __name__ == "__main__":
    
    PP1 = CPP()
    rate = rospy.Rate(10)
    while PP1.get_traj == False:
        continue

    PP1.divide_traj()
    while not rospy.is_shutdown():
        PP1.run()
        if PP1.is_stop == True:
            break
        rate.sleep()

    # for i in range(len(PP1.div_tj)):
    #     plt.plot(PP1.div_tj[i][0], PP1.div_tj[i][1], 'ro')

    # # plt.plot(PP1.div_tj)
    # for i in range(len(PP1.tj)):
    #     plt.plot(PP1.tj[i][0], PP1.tj[i][1], 'bo')        
    # for i in range(len(PP1.xpath)):
    #     plt.plot(PP1.xpath[i], PP1.ypath[i], 'go')

    print("Path is")
    
    
    
    # PP2 = CPP()
    # rate = rospy.Rate(10)
    # while PP2.get_traj == False:
    #     continue

    # PP2.divide_traj()
    # while not rospy.is_shutdown():
    #     PP2.run()
    #     if PP2.is_stop == True:
    #         break
    #     rate.sleep()