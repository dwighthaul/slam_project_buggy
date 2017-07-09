#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String, Int16
from sensor_msgs.msg import LaserScan

from slam_project_buggy.msg import LaserMsgDistance
from slam_project_buggy.msg import AllSidesDistance



default_value_distance = -1
list_sides = ["front", "left", "back", "right"]


class GetMeanDirection():


    data = AllSidesDistance()

    start = [0, 90, 180, 270]
    stop = [10, 100, 190, 280]
    mean_value = [default_value_distance] * 4


    def __init__(self, side ="front"):
        rospy.init_node('node_all_distances', anonymous=True)



    def modifyLists(self, l):

        for x in range(len(l) -1, -1, -1):

            if l[x] == float('Inf') :
                del l[x]
        if len(l) == 0:
            l = [default_value_distance]


        return l




    def getMean(self, l):
        return sum(l)/len(l)


    def callback(self, data_scan):

        databuffer = AllSidesDistance()

        for side in range(len(list_sides)):

            the_list = self.modifyLists(list(data_scan.ranges[self.start[side]:self.stop[side]]))
            mean_value= self.getMean(the_list)


            # Create msg 
            msgDistance = LaserMsgDistance()
            msgDistance.side = list_sides[side]
            msgDistance.distance = mean_value
            databuffer.listDistances.append(msgDistance)

        self.data = databuffer

        
    def main(self):

        pub = rospy.Publisher('allMeanValues', AllSidesDistance, queue_size=10)

        rospy.Subscriber("scan", LaserScan, self.callback)


        while not rospy.is_shutdown():
            pub.publish(self.data)
            rospy.sleep(0.5)


if __name__ == '__main__':

    side = "front"

    if len(sys.argv) > 1 and sys.argv[1] in list_sides:
        side= sys.argv[1]

    try:
        g = GetMeanDirection(side)
        g.main()

    except rospy.ROSInterruptException:
        pass
