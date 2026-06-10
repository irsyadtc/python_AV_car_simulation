import pygame
import math
import numpy as np

class Road:
    def __init__(self, name, lane_direction_1, lane_direction_2, waypoint):
        self.name = name
        self.laneDirection1 = lane_direction_1
        self.laneDirection2 = lane_direction_2
        self.waypoint = waypoint

        self.roadWidth = 35

        self.waypoint_direction_1 = self.generate_lanes_waypoint_dir1()
        self.waypoint_direction_2 = self.generate_lanes_waypoint_dir2()
        print(f"waypoint_direction_1: {self.waypoint_direction_1}")

    def generate_lanes_waypoint_dir1(self):
        # print("generate lane1")
        if(self.laneDirection1 >= 1):
            lane = list()
            #calc first point perpendicular vector
            rvx = self.waypoint[1][0] - self.waypoint[0][0]
            rvy = self.waypoint[1][1]- self.waypoint[1][1]
            road_v = [[rvx],[rvy]]  #first road vector
            road_uv = [[road_v[0][0]/np.linalg.norm(road_v)],[road_v[1][0]/np.linalg.norm(road_v)]] #unit vector
            #perpendicular vector
            perp = np.dot([[0,1],[-1,0]],road_uv) #rotate vector ccw
            road_perpendicular_uv = [perp[0][0],perp[1][0]]
            road_perpendicular_v = np.multiply(road_perpendicular_uv,self.roadWidth)
            # print(f"rp_v: {road_perpendicular_v}")


            for l in range(self.laneDirection1):
                index_ = 0
                l_w = list()
                for w in self.waypoint:
                    # print(f"index_: {index_}")
                    if index_ == 0:
                        x_start = w[0] + (0.5*road_perpendicular_v[0])
                        y_start = w[1] + (0.5*road_perpendicular_v[1])
                        # print(f"x_start: {x_start} y_start: {y_start}")
                        l_w.append([x_start, y_start])
                    else:
                        #set next lane waypoint
                        #get road vector
                        xv_ = w[0] - self.waypoint[index_-1][0]
                        yv_ = w[1] - self.waypoint[index_-1][1]
                        rv_ = [xv_,yv_]
                        # print(f"rv: {rv_}")
                        new_x = l_w[-1][0] + rv_[0]
                        new_y = l_w[-1][1] + rv_[1]
                        l_w.append([new_x,new_y])
                        # print(f"l_w[{index_}]: {l_w[index_]}")
                        
                    index_ = index_+1
                lane.append(l_w)
            # print(f"lane: {lane}")
            return lane
        else:
            return None
        
    def generate_lanes_waypoint_dir2(self):
        # print("generate lane2")
        if(self.laneDirection2 >= 1):
            lane = list()
            #calc first point perpendicular vector
            rvx = self.waypoint[1][0] - self.waypoint[0][0]
            rvy = self.waypoint[1][1]- self.waypoint[1][1]
            road_v = [[rvx],[rvy]]  #first road vector
            road_uv = [[road_v[0][0]/np.linalg.norm(road_v)],[road_v[1][0]/np.linalg.norm(road_v)]] #unit vector
            #perpendicular vector
            perp = np.dot([[0,-1],[1,0]],road_uv) #rotate vector cw
            road_perpendicular_uv = [perp[0][0],perp[1][0]]
            road_perpendicular_v = np.multiply(road_perpendicular_uv,self.roadWidth)

            for l in range(self.laneDirection2):
                index_ = 0
                l_w = list()
                for w in self.waypoint:
                    if index_ == 0:
                        x_start = w[0] + (0.5*road_perpendicular_v[0])
                        y_start = w[1] + (0.5*road_perpendicular_v[1])
                        l_w.append([x_start, y_start])
                    else:
                        #set next lane waypoint
                        #get road vector
                        xv_ = w[0] - self.waypoint[index_-1][0]
                        yv_ = w[1] - self.waypoint[index_-1][1]
                        rv_ = [xv_,yv_]
                        new_x = l_w[-1][0] + rv_[0]
                        new_y = l_w[-1][1] + rv_[1]
                        l_w.append([new_x,new_y])
                        
                    index_ = index_+1
                lane.append(l_w)
            # print(f"lane: {lane}")
            return lane
        else:
            return None
            

    def draw(self,win):
        if(self.laneDirection1 + self.laneDirection2 == 1):
            line = pygame.draw.lines(win,[199,198,168],False,self.waypoint,self.roadWidth)
        elif(self.laneDirection1 + self.laneDirection2 == 0):
            pass
        else:
            if(self.laneDirection1 >= 1):
                for i in self.waypoint_direction_1:
                    line = pygame.draw.lines(win,[199,198,168],False,i,self.roadWidth)
            if(self.laneDirection2 >= 1):
                for j in self.waypoint_direction_2:
                    line = pygame.draw.lines(win,[199,198,168],False,j,self.roadWidth)
        
        # dash line
        dash_length = 10
        for k in self.laneDirection1:
            #dash line

    def dash_line(self):
        pass