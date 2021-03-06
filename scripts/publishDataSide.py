#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String, Int16
from sensor_msgs.msg import LaserScan

from slam_project_buggy.msg import LaserMsgDistance


side = "front"
default_value_distance = -1
list_directions = ["front", "left", "back", "right"]


class GetMeanDirection():


    data = LaserMsgDistance()

    start = 0
    stop = 10
    rmyRosPy = []
    side_lidar = ""
    mean_value = 0


    def __init__(self):


        print(side)

        rospy.init_node('main', anonymous=True)

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


    def callback(self, data):
        self.mean_value= self.getMean(self.modifyList(list(data.ranges[self.start:self.stop])))

        self.data.side = self.side_lidar
        self.data.distance = self.mean_value


        # rospy.loginfo("Side: %s -- Mean: %s", self.side_lidar, self.mean_value)


        
    def main(self):
        topic = 'publ_mean_value_'+ self.side_lidar

        pub = rospy.Publisher(topic, LaserMsgDistance, queue_size=10)

        rospy.Subscriber("scan", LaserScan, self.callback)


        while not rospy.is_shutdown():
            pub.publish(self.data)
            rospy.sleep(0.5)


if __name__ == '__main__':


    g = GetMeanDirection()

    # if len(sys.argv) > 1 and sys.argv[1] in list_directions:
    #     side= sys.argv[1]

    # try:
    #     g = GetMeanDirection(side)
    #     g.main()

    # except rospy.ROSInterruptException:
    #     pass
