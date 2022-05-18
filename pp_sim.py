from math import abs, atan2, cos, sin
import numpy as np
import matplotlib.pyplot as plt


class vehicle:
    def __init__(self):
        # 로봇
        self.P_veh = [0, 0] # 로봇의 좌표 [x, y]
        plt.plot(self.P_veh[0], self.P_veh[1], color='violet') # 차량은 보라색으로 점을 찍는다.
        self.trajectory=[[0,0],[0,3],[3,3],[3,6],[6,6]]
        for i in range(len(self.trajectory)):
            plt.plot(self.trajectory[i][0], self.trajectory[i][1], color='black') # 경로눈 검음색으로 점을 찍는다.
        

    def divide_traj(self):
        self.div_traj=[]
        self.div_unit=1/6 # 한 구간을 나누기 위한 수
        for i in range(len(self.trajectory)-1):
            if(self.trajectory[i][0]==self.trajectory[i+1][0]): # x좌표가 서로 같은 경우
                for j in range(6):
                    self.div_traj[j]=[self.trajectory[i][0],self.trajectory[i][1]+j*self.div_unit] # 기존의 y좌표에서 간격을 더하면서 경로 생성
            else: # y좌표가 서로 같은 경우
                for j in range(6):
                    self.div_traj[j]=[self.trajectory[i][0]+j*self.div_unit,self.trajectory[i][1]] # 기존의 y좌표에서 간격을 더하면서 경로 생성
        for i in range(len(self.div_traj)):
            plt.plot(self.div_traj[i][0], self.div_traj[i][1], color='red') # 경로를 나눈 것은 빨간색으로 표시


    def get_distance(self, A, B):
        distance=((A[0]-B[0])**2+(A[1]-B[1])**2)**(1/2)
        return distance

    def pure_pursuit(self):
        min_dis=3 # 차량과 경로 인덱스의 거리 최소값 임의로 설정
        for i in range(len(self.traj_div)-1): # 모든 경로상에 인덱스에 대해..
            distance=self.get_distacne(self.traj_div[i], P_veh)
            if distance<min_dis:
                min_dis=distance
                P_veh=[self.traj_div[i][0], self.traj_div[i][1]] # 최소인 경우 해당 인덱스는 현재 차량의 위치가 된다.
                index=i
        lookahead=4
        Ld=self.get_distance(self.P_veh, self.traj_div[index+lookahead])
        L=0.47 # 바퀴간 거리
        e=abs(self.P_veh[0]-self.traj_div[index+lookahead][0])
        self.steering=atan2(2*L*e/(Ld)**2)

    def move(self):
        self.V_veh=[0.1*cos(self.steering), 0.1*sin(self.steering)]
        time_interval=0.1 # 몇초 간격으로 차량을 움직일 것인가.
        self.P_veh=[self.P_veh[0]+time_interval*self.V_Veh[0], self.P_veh[1]+time_interval*self.V_Veh[1]]
        plt.plot(self.P_veh[0], self.P_veh[1], color='violet')


    def run(self):
        self.divide_traj()
        while(True):
            self.pure_pursuit()
            self.move()
            if self.get_distance(self.P_veh, self.traj_div[len(self.traj_div)-1])<=0.1:
                print('목적지에 도착하였습니다.')
                break
        

        
'''   def move_rob(self):
        # <계산한 속도만큼 이동. (0.1초 동안)>
        new_P_rob_x = self.P_rob[0] + self.V_rob_avg[0]*self.time_interval # 등가속도 직선 운동이라고 가정 하였으므로, 평균속도를 이용한다
        new_P_rob_y = self.P_rob[1] + self.V_rob_avg[1]*self.time_interval
        new_P_rob = [new_P_rob_x, new_P_rob_y]

        # <이동한 거리 누적..>
        move_distance = self.get_size(self.get_difference(new_Pㄴ_rob, self.P_rob)) 
        self.total_distance += move_distance

        # <이동 시간 누적..>
        self.total_time = self.total_time + self.time_interval
        self.P_rob = new_P_rob
        
    

    def print_result(self):
        print("로봇이 목적지에 도착하였습니다.")
        print("걸린 시간은 {0}초 입니다." .format(self.total_time))
        print("총 이동거리는 {0: .5f}m 입니다." .format(self.total_distance))
        print("커브 횟수는 {0}입니다." .format(self.count_curve))
        print("급커브 횟수는 {0}회 입니다." .format(self.count_sharp_curve))
        print("충돌 횟수는 {0}회 입니다." .format(self.count_crash))
        plt.show()
        exit(0)

    def check_finish(self):
        distance_to_goal = self.get_size(self.get_difference(self.goal, self.P_rob))
        print("목적지 까지의 거리 :", distance_to_goal)
        if(distance_to_goal <= 0.3): # 목적지에 도달했을 경우
            self.print_result()    
        if(self.P_rob[0] > self.goal[0] and self.P_rob[1] > self.goal[1]): # 프로그램상 목적지를 지나쳤을 경우
            V_rob_avg_size = self.get_size(self.V_rob_avg)
            wasted_time = distance_to_goal/V_rob_avg_size
            self.total_time = self.total_time - wasted_time
            self.total_distance = self.total_distance - distance_to_goal
            self.print_result()

    def play(self):
        self.set_obs()
        while(True):
            plt.plot(self.P_rob[0], self.P_rob[1], 'bo')
            plt.plot(self.P_obs[0], self.P_obs[1], 'ro')
            self.get_F_att()
            distance_to_obs = self.get_size(self.get_difference(self.P_rob, self.P_obs))
            print("차량과 장애물 사이의 거리 :", distance_to_obs)
            if(distance_to_obs <= 0.8):
                self.count_crash = self.count_crash + 1
            if(distance_to_obs <= self.R_o): # 탐지거리 안에 들어올 경우 
                print("장애물이 차량의 탐지범위 {0}m 안으로 들어왔습니다." .format(self.R_o))
                self.get_F_rep()
            else:
                print("장애물이 탐지범위 {0}m 밖에 있습니다." .format(self.R_o))
                self.F_rep=[0, 0]
            self.get_speed()
            self.move_rob()
            self.move_obs()
            plt.xlim(0,20)
            plt.ylim(0,20)
            plt.pause(0.3)
            self.check_finish()
            

        
data1 = data()
data1.play() '''

A=vehicle()
print(A.div_traj) 
