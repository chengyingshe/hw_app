
import sys
import random
import numpy as np

n = 200
robot_num = 10
berth_num = 10
boat_num = 5

LAND, SEA, OBS, ROB, BERT = 0, 1, 2, 3, 4
mark = {
    '.': LAND,
    '*': SEA,
    '#': OBS,
    'A': ROB,
    'B': BERT
}

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
mymap = np.zeros((n, n))
goods = np.zeros((n, n))

def Init():
    global boat_capacity
    for i in range(n):  # 获取200x200地图
        line = input()
        chars = line.split()
        for j in range(n):
            mymap[i, j] = mark[chars[j]]
        # ch.append(chars)
    for i in range(berth_num):  # 获取泊位数据
        line = input()
        berth_list = [int(c) for c in line.split()]
        id = berth_list[0]
        berth[id].x = berth_list[1]
        berth[id].y = berth_list[2]
        berth[id].transport_time = berth_list[3]
        berth[id].loading_speed = berth_list[4]
    boat_capacity = int(input())  # 获取船的容积
    okk = input()
    print("OK")
    sys.stdout.flush()
    
def Input():
    id, money = map(int, input().split())  # 帧序号，当前金币数
    num = int(input())  # 新增货物的数量
    for i in range(num):  # num行新增货物的数量
        x, y, val = map(int, input().split())  # 货物坐标，金额
        goods[x][y] = val
    for i in range(robot_num):  # 10行机器人的信息
        robot[i].goods, robot[i].x, robot[i].y, robot[i].status = map(int, input().split())
    for i in range(5):  # 5行船的信息
        boat[i].status, boat[i].pos = map(int, input().split())
    okk = input()  # 'OK'
    return id

def get_nearest_robot_id_from_good(gd_x, gd_y, exp=[]):
    # 获取距离货物最近的机器人id（去除不能使用的机器人）
    d2 = []
    for r in robot:
        i = robot.index(r)
        if i not in exp:
            d2.append([i, (r.x - gd_x)**2 + (r.y - gd_y)**2])
    return min(d2, key=lambda x: x[1])[0]

def get_all_good_pos_from_robot(rob, x_range=20, y_range=20):
    # 获取机器人一定范围内所有货物坐标
    x_min = rob.x - int(x_range / 2)
    x_max = rob.x + int(x_range / 2)
    y_min = rob.y - int(y_range / 2)
    y_max = rob.y + int(y_range / 2)
    x_min = x_min if x_min >= 0 else 0
    x_max = x_max if x_max < n else n
    y_min = y_min if y_min >= 0 else 0
    y_max = y_max if y_max < n else n
    gd_pos = []
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            if mymap[x, y] == LAND and goods[x, y] > 0:
                gd_pos.append((x, y))
    return gd_pos

def get_nearest_berth_id(cur_x, cur_y):
    # 获取距离当前位置最近的港口
    dist = [(cur_x - b.x)**2 + (cur_y - b.y)**2 for b in berth]
    return np.argmin(dist)


if __name__ == "__main__":
    Init()
    for frame in range(1, 15001):
        id = Input()

        """
        这部分进行每一帧的信息处理然后给出机器人和船的运动决策
        """
        gd_pos = get_all_good_pos_from_robot(robot[0])
        
        


        
        print("OK")
        sys.stdout.flush()