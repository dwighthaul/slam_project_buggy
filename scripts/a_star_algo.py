#!/usr/bin/python

from heapq import heappush, heappop
import math
import time

from random import *


defaultValue = "."
wallValue = "#"

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
    


class RandomPoint(Point):
    def __init__(self, max_x, max_y):
        super(RandomPoint, self).__init__(randint(0, max_x), randint(0, max_y))



class node:
    xPos = 0
    yPos = 0
    distance = 0
    priority = 0
    def __init__(self, xPos, yPos, distance, priority):
        self.xPos = xPos
        self.yPos = yPos
        self.distance = distance
        self.priority = priority
    def __lt__(self, other): # comparison method for priority queue
        return self.priority < other.priority
    def updatePriority(self, xDest, yDest):
        self.priority = self.distance + self.estimate(xDest, yDest) * 10 # A*
    # give higher priority to going straight instead of diagonally
    def nextMove(self, dirs, d): # d: direction to move
        if dirs == 8 and d % 2 != 0:
            self.distance += 14
        else:
            self.distance += 10
    def estimate(self, xDest, yDest):
        xd = xDest - self.xPos
        yd = yDest - self.yPos
        d = math.sqrt(xd * xd + yd * yd)

        return(d)


class PathCreator(object):

    def __init__(self, i_map, i_start, i_goal, i_height, i_width):
        super(PathCreator, self).__init__()
        self.map = i_map
        self.start = i_start
        self.goal = i_goal
        self.height =  i_height
        self.width = i_width 


    def pathFind(self, the_map, n, m, dirs, dx, dy):
        closed_nodes_map = []
        open_nodes_map = []
        dir_map = []
        row = [0] * n
        for i in range(m):
            closed_nodes_map.append(list(row))
            open_nodes_map.append(list(row))
            dir_map.append(list(row))

        pq = [[], []]
        pqi = 0

        n0 = node(self.start.x, self.start.y, 0, 0)
        n0.updatePriority(self.goal.x, self.goal.y)
        heappush(pq[pqi], n0)
        open_nodes_map[self.start.y][self.start.x] = n0.priority

        # Implementation of the algorith
        while len(pq[pqi]) > 0:

            n1 = pq[pqi][0]
            n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)

            # start with the start position (postion of the robot)
            x = n0.xPos
            y = n0.yPos
            heappop(pq[pqi])
            open_nodes_map[y][x] = 0
            closed_nodes_map[y][x] = wallValue

            if x == self.goal.x and y == self.goal.y:
                path = ''
                while not (x == self.start.x and y == self.start.y):
                    j = dir_map[y][x]
                    c = str((j + dirs / 2) % dirs)
                    path = c + path
                    x += dx[j]
                    y += dy[j]
                return path

            for i in range(dirs):
                xdx = x + dx[i]
                ydy = y + dy[i]
                if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > self.width- 1
                        or the_map[ydy][xdx] == wallValue or closed_nodes_map[ydy][xdx] == wallValue):
                    # generate a child node
                    m0 = node(xdx, ydy, n0.distance, n0.priority)
                    m0.nextMove(dirs, i)
                    m0.updatePriority(self.goal.x, self.goal.y)
                    # if it is not in the open list then add into that
                    if open_nodes_map[ydy][xdx] == 0:
                        open_nodes_map[ydy][xdx] = m0.priority
                        heappush(pq[pqi], m0)
                        # mark its parent node direction
                        dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                    elif open_nodes_map[ydy][xdx] > m0.priority:
                        # update the priority
                        open_nodes_map[ydy][xdx] = m0.priority
                        # update the parent direction
                        dir_map[ydy][xdx] = (i + dirs / 2) % dirs
                        # replace the node
                        # by emptying one pq to the other one
                        # except the node to be replaced will be ignored
                        # and the new node will be pushed in instead
                        while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                            heappush(pq[1 - pqi], pq[pqi][0])
                            heappop(pq[pqi])
                        heappop(pq[pqi]) # remove the target node
                        # empty the larger size priority queue to the smaller one
                        if len(pq[pqi]) > len(pq[1 - pqi]):
                            pqi = 1 - pqi
                        while len(pq[pqi]) > 0:
                            heappush(pq[1-pqi], pq[pqi][0])
                            heappop(pq[pqi])       
                        pqi = 1 - pqi
                        heappush(pq[pqi], m0) # add the better node instead
        return '' # if no route found




    def fillWalls(self, wallList):

        for i in range(len(wallList)):

            x = wallList[i] / self.height
            y = wallList[i] % self.width

            self.map[y][x] = wallValue




    def main(self):

        # print 'Map size (X,Y): ', self.width, self.height
        # print 'Start: ', self.start.x, self.start.y
        # print 'Finish: ', self.goal.x, self.goal.y 
        t = time.time()

        dirs = 8 # number of possible directions to move on the map
        if dirs == 4:
            dx = [1, 0, -1, 0]
            dy = [0, 1, 0, -1]
        elif dirs == 8:
            dx = [1, 1, 0, -1, -1, -1, 0, 1]
            dy = [0, 1, 1, 1, 0, -1, -1, -1]


        # print 'Start to genarate the route'

        route = self.pathFind(self.map, len(self.map), len(self.map[0]), dirs, dx, dy)

        # print 'Time to generate the route (seconds): ', time.time() - t
        # print 'Route:'
        # print route


        listPoint = [self.start]

        if len(route) > 0:
            x = self.start.x
            y = self.start.y
            self.map[y][x] = "S"
            for i in range(len(route)):
                j = int(route[i])
                x += dx[j]
                y += dy[j]
                listPoint.append(Point(y, x))
                self.map[y][x] = "+"
            self.map[y][x] = "G"


        for p in listPoint:
            pass
            # print(p.x),
            # print(p.y)
            

        return listPoint

    def drawMap(self):
        print 'Map:'
        for x in range(self.height):
            raw = ""
            for y in range(self.width):
                raw = raw + self.map[y][x]
            print(raw)



if __name__ == '__main__':

    height = 94
    width = 95

    the_map = []


    row = [defaultValue] * height 
    for i in range(width):
        the_map.append(list(row))


    start = RandomPoint(height, width)
    goal = RandomPoint(height, width)

    wallList = [249, 250, 343, 344, 345, 346, 437, 438, 439, 440, 441, 442, 443, 532, 533, 534, 535, 536, 537, 538, 539, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 720, 721, 722, 723, 724, 725, 726, 727, 728, 729, 730, 731, 815, 816, 817, 818, 819, 820, 821, 822, 823, 824, 825, 826, 827, 911, 912, 913, 914, 917, 918, 919, 920, 921, 922, 923, 1007, 1008, 1013, 1014, 1015, 1016, 1017, 1018, 1019, 1109, 1110, 1111, 1112, 1113, 1114, 1115, 1205, 1206, 1207, 1208, 1209, 1210, 1211, 1301, 1302, 1303, 1304, 1305, 1306, 1398, 1399, 1400, 1494, 1547, 1641, 1642, 1643, 1735, 1736, 1737, 1738, 1739, 1740, 1777, 1830, 1831, 1832, 1833, 1834, 1835, 1836, 1871, 1872, 1873, 1925, 1926, 1927, 1928, 1929, 1930, 1931, 1932, 1947, 1948, 1958, 1959, 1965, 1966, 1967, 1968, 1969, 2020, 2021, 2022, 2023, 2024, 2025, 2026, 2041, 2042, 2043, 2044, 2052, 2053, 2054, 2055, 2060, 2061, 2062, 2063, 2114, 2115, 2116, 2117, 2118, 2119, 2120, 2121, 2135, 2136, 2137, 2138, 2139, 2140, 2146, 2147, 2148, 2149, 2150, 2151, 2154, 2155, 2156, 2157, 2209, 2210, 2211, 2212, 2213, 2214, 2215, 2216, 2229, 2230, 2231, 2232, 2233, 2234, 2235, 2242, 2243, 2244, 2245, 2246, 2247, 2248, 2249, 2250, 2251, 2252, 2253, 2305, 2306, 2307, 2308, 2309, 2310, 2311, 2312, 2324, 2325, 2326, 2327, 2328, 2329, 2338, 2339, 2340, 2341, 2342, 2343, 2344, 2345, 2346, 2347, 2348, 2349, 2401, 2402, 2404, 2405, 2406, 2420, 2421, 2422, 2423, 2434, 2435, 2436, 2439, 2440, 2441, 2442, 2443, 2444, 2445, 2500, 2516, 2517, 2530, 2535, 2536, 2537, 2538, 2539, 2540, 2541, 2542, 2545, 2618, 2619, 2620, 2631, 2632, 2633, 2634, 2635, 2636, 2637, 2638, 2639, 2640, 2641, 2712, 2713, 2714, 2715, 2716, 2727, 2728, 2729, 2730, 2731, 2732, 2733, 2734, 2735, 2736, 2737, 2806, 2807, 2808, 2809, 2810, 2811, 2812, 2823, 2824, 2825, 2826, 2827, 2828, 2829, 2830, 2831, 2832, 2833, 2834, 2835, 2900, 2901, 2902, 2903, 2904, 2905, 2906, 2907, 2920, 2921, 2922, 2923, 2924, 2925, 2926, 2927, 2928, 2929, 2930, 2931, 2994, 2995, 2996, 2997, 2998, 2999, 3000, 3001, 3017, 3020, 3021, 3022, 3023, 3024, 3025, 3026, 3027, 3089, 3090, 3091, 3092, 3093, 3094, 3095, 3096, 3116, 3117, 3118, 3119, 3120, 3121, 3185, 3186, 3187, 3188, 3189, 3190, 3211, 3212, 3214, 3215, 3219, 3220, 3281, 3282, 3283, 3284, 3305, 3306, 3307, 3313, 3314, 3315, 3316, 3378, 3399, 3400, 3401, 3402, 3403, 3407, 3408, 3409, 3410, 3411, 3412, 3493, 3494, 3495, 3496, 3497, 3498, 3501, 3502, 3503, 3504, 3505, 3506, 3507, 3587, 3588, 3589, 3590, 3591, 3592, 3596, 3597, 3598, 3599, 3600, 3601, 3681, 3682, 3683, 3684, 3685, 3686, 3692, 3693, 3694, 3695, 3776, 3777, 3778, 3779, 3780, 3781, 3782, 3788, 3789, 3872, 3873, 3874, 3875, 3876, 3877, 3878, 3968, 3969, 3970, 3971, 3972, 3973, 3974, 4064, 4065, 4066, 4067, 4068, 4069, 4158, 4159, 4160, 4161, 4162, 4163, 4252, 4253, 4254, 4255, 4256, 4257, 4258, 4346, 4347, 4348, 4349, 4350, 4351, 4352, 4353, 4354, 4440, 4441, 4442, 4443, 4444, 4445, 4446, 4447, 4448, 4449, 4534, 4535, 4536, 4537, 4538, 4539, 4540, 4541, 4542, 4543, 4628, 4629, 4630, 4631, 4632, 4633, 4634, 4635, 4636, 4637, 4705, 4723, 4724, 4725, 4726, 4727, 4729, 4730, 4767, 4768, 4799, 4800, 4801, 4819, 4820, 4821, 4861, 4862, 4863, 4864, 4893, 4894, 4895, 4896, 4897, 4915, 4955, 4956, 4957, 4958, 4959, 4960, 4987, 4988, 4989, 4990, 4991, 4992, 5042, 5050, 5051, 5052, 5053, 5054, 5055, 5056, 5070, 5083, 5084, 5085, 5086, 5136, 5137, 5138, 5146, 5147, 5148, 5149, 5150, 5151, 5152, 5164, 5165, 5166, 5179, 5180, 5183, 5184, 5185, 5186, 5187, 5230, 5231, 5232, 5233, 5234, 5242, 5243, 5244, 5245, 5246, 5247, 5248, 5258, 5259, 5260, 5261, 5262, 5269, 5277, 5278, 5279, 5280, 5281, 5282, 5283, 5326, 5327, 5328, 5338, 5339, 5340, 5341, 5342, 5343, 5344, 5353, 5354, 5355, 5356, 5363, 5364, 5365, 5371, 5372, 5373, 5374, 5375, 5376, 5377, 5378, 5379, 5422, 5434, 5435, 5436, 5437, 5438, 5439, 5440, 5447, 5448, 5449, 5450, 5451, 5457, 5458, 5459, 5460, 5461, 5467, 5468, 5469, 5470, 5471, 5472, 5473, 5530, 5531, 5532, 5533, 5534, 5535, 5536, 5537, 5543, 5544, 5545, 5553, 5554, 5555, 5559, 5563, 5564, 5565, 5566, 5567, 5571, 5572, 5626, 5627, 5628, 5629, 5630, 5631, 5632, 5633, 5639, 5642, 5643, 5644, 5649, 5653, 5654, 5655, 5658, 5659, 5660, 5661, 5662, 5663, 5664, 5665, 5666, 5667, 5668, 5722, 5723, 5724, 5725, 5726, 5727, 5728, 5729, 5730, 5736, 5737, 5738, 5739, 5740, 5747, 5748, 5749, 5750, 5751, 5754, 5755, 5756, 5757, 5758, 5759, 5760, 5761, 5762, 5763, 5764, 5818, 5819, 5820, 5821, 5822, 5823, 5824, 5825, 5826, 5830, 5831, 5832, 5833, 5834, 5835, 5836, 5842, 5843, 5844, 5845, 5846, 5850, 5851, 5852, 5853, 5854, 5855, 5856, 5857, 5858, 5914, 5915, 5916, 5917, 5918, 5919, 5920, 5921, 5922, 5926, 5927, 5928, 5929, 5930, 5931, 5938, 5939, 5940, 5944, 5945, 5946, 5947, 5948, 5949, 5950, 5951, 5952, 6010, 6011, 6012, 6013, 6014, 6015, 6016, 6017, 6018, 6022, 6023, 6024, 6025, 6026, 6034, 6038, 6039, 6040, 6041, 6042, 6043, 6044, 6045, 6046, 6106, 6107, 6108, 6109, 6110, 6111, 6112, 6113, 6114, 6118, 6119, 6120, 6133, 6134, 6135, 6136, 6137, 6138, 6139, 6140, 6142, 6143, 6203, 6204, 6205, 6206, 6207, 6208, 6209, 6214, 6228, 6229, 6230, 6231, 6232, 6236, 6237, 6238, 6239, 6299, 6300, 6301, 6302, 6303, 6304, 6324, 6325, 6326, 6330, 6331, 6332, 6333, 6334, 6335, 6396, 6397, 6398, 6403, 6415, 6416, 6420, 6426, 6427, 6428, 6429, 6430, 6431, 6492, 6493, 6494, 6496, 6497, 6498, 6499, 6509, 6510, 6511, 6512, 6522, 6523, 6524, 6525, 6526, 6527, 6587, 6588, 6589, 6590, 6591, 6592, 6593, 6594, 6595, 6603, 6604, 6605, 6606, 6607, 6608, 6618, 6619, 6620, 6621, 6622, 6623, 6677, 6681, 6682, 6683, 6684, 6685, 6686, 6687, 6688, 6689, 6690, 6697, 6698, 6699, 6700, 6701, 6702, 6703, 6714, 6715, 6716, 6717, 6718, 6719, 6771, 6772, 6773, 6777, 6778, 6779, 6780, 6781, 6782, 6783, 6784, 6791, 6792, 6793, 6794, 6795, 6796, 6797, 6810, 6811, 6812, 6813, 6814, 6815, 6865, 6866, 6867, 6868, 6869, 6872, 6873, 6874, 6875, 6876, 6877, 6878, 6879, 6885, 6886, 6887, 6888, 6889, 6890, 6891, 6906, 6907, 6908, 6909, 6910, 6911, 6961, 6962, 6963, 6968, 6969, 6970, 6971, 6972, 6973, 6979, 6980, 6981, 6982, 6983, 6984, 6985, 7002, 7003, 7004, 7005, 7006, 7057, 7064, 7065, 7066, 7067, 7073, 7074, 7075, 7076, 7077, 7078, 7079, 7098, 7099, 7100, 7161, 7167, 7168, 7169, 7170, 7171, 7172, 7173, 7194, 7262, 7263, 7264, 7265, 7266, 7267, 7358, 7359, 7360, 7361, 7454, 7455, 8057, 8058, 8064, 8151, 8152, 8153, 8154, 8157, 8158, 8159, 8160, 8245, 8246, 8247, 8248, 8249, 8250, 8251, 8252, 8253, 8254, 8255, 8256, 8339, 8340, 8341, 8342, 8343, 8344, 8345, 8346, 8347, 8348, 8349, 8350, 8435, 8436, 8437, 8438, 8439, 8440, 8441, 8442, 8443, 8444, 8529, 8530, 8531, 8534, 8535, 8536, 8537, 8538, 8623, 8624, 8625, 8626, 8627, 8630, 8719, 8720, 8721, 8815]


    createPath = PathCreator(the_map, start, goal)

    createPath.fillWalls(wallList)

    createPath.main()

    createPath.drawMap()



