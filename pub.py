# -*- coding:utf-8 -*-
#! /usr/bin/env python3

from math import atan2, cos, degrees, radians, sin

import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray


class CPP:
    def __init__(self):
        rospy.init_node("P")
        self.P = rospy.Publisher("/controller", Float32MultiArray, queue_size=1)
        self.P2 = rospy.Publisher("/divided_trajectory", Float32MultiArray, queue_size=1)
        self.first_imu = -32.61 -38.31
        self.tj=[] #로봇이 가야할 경로의 좌표, 처음 좌표는 항상 로봇의 현위치
        self.heading_imu = self.first_imu 
        self.heading_pp = 0
        self.P_veh = [0,0]
        self.speed = 480
        self.pre_veh = [0,0]
        self.xpath = []
        self.ypath = []

        self.divided_path = Float32MultiArray()
        # from planning                                                                                                                                                                                                                                                                                                                                                                                                                                            
        self.get_traj = False
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
            self.is_stop=True


        


    def divide_traj(self):
        # self.P_veh = self.tj[]
        self.P_veh=[self.tj[0][0], self.tj[0][1]]
        self.div_tj=[]
        self.div_unit= 0.3 # 한 구간을 나누기 위한 수 (unit : m)
        for i in range(len(self.tj)-1):
            for j in range(round(self.get_distance(self.tj[i+1],self.tj[i])/self.div_unit)):
                self.div_tj.append([self.tj[i][0]+(self.tj[i+1][0]-self.tj[i][0])*j*self.div_unit/ \
                    self.get_distance(self.tj[i+1],self.tj[i]),self.tj[i][1]+(self.tj[i+1][1] \
                        -self.tj[i][1])*j*self.div_unit/self.get_distance(self.tj[i+1],self.tj[i])]) # 기존의 y좌표에서 간격을 더하면서 경로 생성
        self.div_tj.append(self.tj[-1]) #목적지 좌표 추가

        for i in range (int(len(self.div_tj))):
            self.divided_path.data.append(self.div_tj[i][0])
            self.divided_path.data.append(self.div_tj[i][1])


        for i in range(len(self.div_tj)):
            plt.plot(self.div_tj[i][0], self.div_tj[i][1], 'bo')
        for i in range(len(self.tj)):
            plt.plot(self.tj[i][0], self.tj[i][1], 'ro')
        print(self.tj)
        print('\n\n',self.div_tj)
        plt.show()

    def get_distance(self, A, B): #점과 점 사이의 거리를 구하는 함수
        distance=((A[0]-B[0])**2+(A[1]-B[1])**2)**(1/2)
        return distance

    def pure_pursuit(self):
        self.trans_head_imu_to_pp(self.heading_imu)
        print('pp heading:', self.heading_pp)
        index=0
        min_dis=10 # 차량과 경로 인덱스의 거리 최소값 임의로 설정
        for i in range(len(self.div_tj)): # 모든 경로상에 인덱스에 대해..
            distance=self.get_distance(self.div_tj[i], self.P_veh)
            if distance<=min_dis:  
                min_dis=distance
                self.pre_veh=[self.div_tj[i][0], self.div_tj[i][1]] # 최소인 경우 해당 인덱스는 현재 차량의 위치가 된다.
                index=i

        lookahead=5
        index_goal=index+lookahead
        if index_goal>len(self.div_tj)-1:
            index_goal=len(self.div_tj)-1



        Ld=(self.get_distance(self.div_tj[index_goal], self.P_veh))
        L=0.58 # 바퀴간 거리 0.58
        
        dif_x=self.div_tj[index_goal][0]-self.P_veh[0]
        dif_y=self.div_tj[index_goal][1]-self.P_veh[1]
        beta=atan2(dif_x, dif_y)
       
        alpha=radians(self.heading_pp)-beta # alpha값(각도) 구하기
       

        e=sin(alpha)*Ld # e가 양수(right) / e가 음수(left)

        self.steering=degrees(atan2(2*L*e,(Ld)**2)) # pp에서의 steering 값
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
        if self.is_stop==True:
            self.speed = 0
            self.pure_pursuit()
            inin.data=[]
            inin.data.append(self.speed)
            inin.data.append(1585-self.steering*9)
            inin.data.append(0)
            inin.data.append(0)
            self.P.publish(inin)
        else:
            self.pure_pursuit()
            inin.data=[]
            inin.data.append(self.speed)
            inin.data.append(1585-self.steering*9)
            inin.data.append(0)
            inin.data.append(0)
        
        self.P.publish(inin)
        if self.get_distance(self.P_veh, self.div_tj[len(self.div_tj)-1])<=0.6:
            self.speed=0
            self.is_stop = True
            self.P2.publish(self.divided_path)
        

if __name__ == "__main__":
    
    PP1 = CPP()
    rate = rospy.Rate(100)
    while PP1.get_traj == False:
        continue

    PP1.divide_traj()
    while not rospy.is_shutdown():
        PP1.run()
        if PP1.is_stop == True:
            PP1.run()
            break
        rate.sleep()
     
    
    PP2 = CPP()
    rate = rospy.Rate(10)
    while PP2.get_traj == False:
        continue

    PP2.divide_traj()
    while not rospy.is_shutdown():
        PP2.run()
        if PP2.is_stop == True:
            PP2.run()
            break
        rate.sleep()

    PP3 = CPP()
    rate = rospy.Rate(10)
    while PP3.get_traj == False:
        continue

    PP3.divide_traj()
    while not rospy.is_shutdown():
        PP3.run()
        if PP3.is_stop == True:
            PP3.run()
            break
        rate.sleep()
