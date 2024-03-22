
import heapq
import sys
import random

import numpy as np
from a_star_wait import simulate, decode_robot_moves_lists, run_orders_list, mark
import time


N = 200
robot_num = 10
berth_num = 10
boat_num = 10

class Robot:
    def __init__(self, startX=0, startY=0, goods=0, status=0, mbx=0, mby=0):
        self.x = startX
        self.y = startY
        self.goods = goods
        self.status = status
        self.mbx = mbx
        self.mby = mby

robot = [Robot() for _ in range(robot_num)]

class Berth:
    def __init__(self, x=0, y=0, transport_time=0, loading_speed=0):
        self.x = x
        self.y = y
        self.transport_time = transport_time
        self.loading_speed = loading_speed

berth = [Berth() for _ in range(berth_num)]

class Boat:
    def __init__(self, num=0, pos=0, status=0):
        self.num = num
        self.pos = pos
        self.status = status

boat = [Boat() for _ in range(boat_num)]

money = 0
boat_capacity = 0
id = 0
ch = []
gds = [[0 for _ in range(N)] for _ in range(N)]
mymap = np.zeros((N, N))

def Init():
    global boat_capacity
    for i in range(N):  # 获取200x200地图
        line = input()
        ch.append(line.strip())

    for i in range(berth_num):
        line = input()
        berth_list = [int(c) for c in line.split(sep=" ")]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    
    for i in range(N):
        for j in range(N):
            mymap[i, j] = mark[ch[i][j]]
    boat_capacity = int(input())
    okk = input()
    print("OK")
    sys.stdout.flush()

good_pos_list = []
robot_pos_list = []
def Input():
    id, money = map(int, input().split(" "))
    num = int(input())
    for i in range(num):
        x, y, val = map(int, input().split())
        # gds[x][y] = val
        good_pos_list.append((x, y))
    for i in range(robot_num):
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())
        robot_pos_list.append((robot[i].x, robot[i].y))
    for i in range(5):
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()
    return id

if __name__ == "__main__":
    Init()
    start = time.time()
    for frame in range(1):
        id = Input()

        initial_positions, robot_moves_map = simulate(mymap, robot_pos_list, good_pos_list)
        robots_orders_list = decode_robot_moves_lists(robot_moves_map)
        run_orders_list(robots_orders_list, Input)

        # print("OK")
        # sys.stdout.flush()
