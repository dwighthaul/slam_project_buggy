#!/usr/bin/env python
import rospy
import sys

from std_msgs.msg import String, Int16
from sensor_msgs.msg import LaserScan


from slam_project_buggy.msg import LaserMsgDistance



default_value_distance = -1
list_directions = ["front", "left", "back", "right"]


class GetMeanDirection():

    data = LaserMsgDistance()

    start = 0
    stop = 10
    rmyRosPy = []
    side_lidar = ""
    mean_value = 0


    def __init__(self, side ="front"):
        node = 'get_data_side' 
        rospy.init_node(node, anonymous=True)

        self.side_lidar = side

        if side== "front":
            self.start = 0
            self.stop = 10

        elif side == "left":
            self.start = 90
            self.stop = 100

        elif side == "back":
            self.start = 180
            self.stop = 190

        elif side == "right":
            self.start = 270
            self.stop = 280

        rospy.loginfo("Side set to the %s", side)


        self.mean_value = default_value_distance




    def modifyList(self, l):
        for x in range(len(l) -1, -1, -1):

            if l[x] == float('Inf') :
                del l[x]

        if len(l) == 0:
            l = [default_value_distance]

        return l

    def getMean(self, l):
        return sum(l)/len(l)


    def callback_sub_scan(self, data):
        self.mean_value= self.getMean(self.modifyList(list(data.ranges[self.start:self.stop])))

        self.data.side = self.side_lidar
        self.data.distance = self.mean_value


    def main(self):
        rospy.Subscriber("scan", LaserScan, self.callback_sub_scan)
        rospy.spin()


if __name__ == '__main__':

    side = "front"

    if len(sys.argv) > 1 and sys.argv[1] in list_directions:
        side= sys.argv[1]

    try:
        g = GetMeanDirection(side)
        g.main()

    except rospy.ROSInterruptException:
        pass
