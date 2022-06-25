from math import atan2, cos, sin, radians, degrees
from time import sleep
import matplotlib.pyplot as plt


class vehicle:
    def __init__(self):
        # 로봇
        self.P_veh = [-1,-1] # 로봇의 좌표 [x, y]
        self.trajectory=[self.P_veh,[-3,2],[-6,2],[-9,1]] #로봇이 가야할 경로의 좌표, 처음 좌표는 항상 로봇의 현위치
        self.overcurve=0
        self.total_time=0
        self.heading=0 # 북쪽이 0 
        self.go_left=0 

    def divide_traj(self):
        self.div_traj=[]
        self.div_unit=1/2 # 한 구간을 나누기 위한 수
        for i in range(len(self.trajectory)-1):
            for j in range(int(self.get_distance(self.trajectory[i+1],self.trajectory[i])/self.div_unit)):
                self.div_traj.append([self.trajectory[i][0]+(self.trajectory[i+1][0]-self.trajectory[i][0])*j*self.div_unit/self.get_distance(self.trajectory[i+1],self.trajectory[i]),self.trajectory[i][1]+(self.trajectory[i+1][1]-self.trajectory[i][1])*j*self.div_unit/self.get_distance(self.trajectory[i+1],self.trajectory[i])]) # 기존의 y좌표에서 간격을 더하면서 경로 생성
        for i in range(len(self.div_traj)):
            plt.plot(self.div_traj[i][0], self.div_traj[i][1], 'ro') # 경로를 나눈 것은 빨간색으로 표시
        self.div_traj.append(self.trajectory[-1]) #목적지 좌표 추가
        #print(self.div_traj)

    def get_distance(self, A, B): #점과 점 사이의 거리를 구하는 함수
        distance=((A[0]-B[0])**2+(A[1]-B[1])**2)**(1/2)
        return distance

    def pure_pursuit(self):
        index=0
        min_dis=10 # 차량과 경로 인덱스의 거리 최소값 임의로 설정
        for i in range(len(self.div_traj)): # 모든 경로상에 인덱스에 대해..
            distance=self.get_distance(self.div_traj[i], self.P_veh)
            if distance<=min_dis:
                min_dis=distance
                self.pre_veh=[self.div_traj[i][0], self.div_traj[i][1]] # 최소인 경우 해당 인덱스는 현재 차량의 위치가 된다.
                index=i
                #print([self.div_traj[index][0], self.div_traj[index][1]])

        lookahead=2
        index_goal=index+lookahead
        if index_goal>len(self.div_traj)-1:
            index_goal=len(self.div_traj)-1

        Ld=self.get_distance(self.div_traj[index_goal], self.pre_veh)
        L=0.58 # 바퀴간 거리
        
        beta=atan2(self.div_traj[index_goal][1]-self.P_veh[1],self.div_traj[index_goal][0]-self.P_veh[0]) # 차량->목적지 heading 값(각도아님)
        alpha=radians(self.heading)-beta # alpha값(각도) 구하기

        e=sin(alpha)*Ld # e가 양수(좌) / e가 음수(우)

        self.steering=degrees(atan2(2*L*e,(Ld)**2))
        if self.steering>=30:
            self.steering=30
            self.overcurve=self.overcurve+1
        elif self.steering<=-30:
            self.steering=-30
            self.overcurve=self.overcurve+1
        self.move_angle=self.heading-self.steering
        self.heading=self.move_angle

        """"
        print('index:',index)
        print('index_goal:',index_goal)
        print('steering:',self.steering)
        print('move_angle:',self.move_angle)
        print('go_left:',self.go_left)

        print('경로길이:', len(self.div_traj))
        """


    def move(self):
        self.V_veh=[1*sin(radians(self.move_angle)), 1*cos(radians(self.move_angle))]
        time_interval=0.2 # 몇초 간격으로 차량을 움직일 것인가.
        self.P_veh=[self.P_veh[0]+time_interval*self.V_veh[0], self.P_veh[1]+time_interval*self.V_veh[1]]
        plt.plot(self.P_veh[0], self.P_veh[1],'bo')
        #print('차량이 이동했습니다. 현재 차량의 위치는',self.P_veh,'입니다.')
        self.total_time=self.total_time+time_interval
        #print('주행 시간은',self.total_time,'입니다.')
        


    def run(self):
        self.divide_traj()
        plt.grid()
        for i in range(len(self.trajectory)):
            plt.plot(self.trajectory[i][0], self.trajectory[i][1], 'ko')
        while(True):
            self.pure_pursuit()
            self.move()
            if self.get_distance(self.P_veh, self.div_traj[len(self.div_traj)-1])<=0.1 or self.total_time >= 100:
                #print('목적지에 도착하였습니다.')
                plt.xlabel('meter')
                plt.ylabel('meter')
                plt.show()
                #print('급커브했따', self.overcurve)
                break
        

A=vehicle()
A.run()