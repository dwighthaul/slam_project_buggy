#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('slam_project_buggy')
import rospy
import tf



# rosPoint as because of the Point from the vectors modules
from geometry_msgs.msg import  Point as rosPoint
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion, PolygonStamped
import turtlesim.srv


from actionlib import SimpleActionClient

from std_msgs.msg import String, Int16, Header
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from slam_project_buggy.msg import ListPoints

import math
from vectors import Point, Vector

# For the action
from slam_project_buggy.msg import setDirectionRobotAction, setDirectionRobotGoal
from navigToPoint import navigToPoint

class Object(object):
    pass



class ptpSendOrder():

    doesPointSelected = False
    
    pointSelected = rosPoint()

    pointSelected.x = 0.0
    pointSelected.y = 0.0
    pointSelected.z = 0.0

    pointRobot = rosPoint()
    vectorRobot = Quaternion()

    vectorRobotToPointselected = Vector3()


    def __init__(self):

        rospy.init_node('navig_to_point', anonymous=True)
        rospy.Subscriber("list_point_path", ListPoints, self.callback_getlist_point, queue_size=1)



        self.navigToPoint = navigToPoint()



    def callback_getlist_point(self, fullListPoint):
        print("Points Received")
        pointReach = False

        print(len(fullListPoint.data))

        last = fullListPoint.data[len(fullListPoint.data)-1]

        fullListPoint.data = fullListPoint.data[0:][::5]

        fullListPoint.data.append(last)

        for p in range(1,len(fullListPoint.data)):
            pointReach = False

            data = Object()

            data.point = fullListPoint.data[p]
            
            print(data.point)

            self.navigToPoint.callback_save_point_clicked(data)
            if self.navigToPoint.main():
                return


        rospy.loginfo("Target Reach")



    def main(self):

        listPoint = Object()

        # listPoint.data = [ rosPoint(3.30000004917, 2.70000004023,  0), rosPoint(3.35000004992, 2.65000003949,  0), rosPoint(3.40000005066, 2.60000003874,  0), rosPoint(3.45000005141, 2.550000038,  0), rosPoint(3.50000005215, 2.50000003725,  0), rosPoint(3.50000005215, 2.45000003651,  0), rosPoint(3.5500000529, 2.40000003576,  0), rosPoint(3.5500000529, 2.35000003502,  0), rosPoint(3.5500000529, 2.30000003427,  0), rosPoint(3.5500000529, 2.25000003353,  0), rosPoint(3.5500000529, 2.20000003278,  0), rosPoint(3.5500000529, 2.15000003204,  0), rosPoint(3.5500000529, 2.10000003129,  0), rosPoint(3.5500000529, 2.05000003055,  0), rosPoint(3.5500000529, 2.0000000298,  0), rosPoint(3.5500000529, 1.95000002906,  0), rosPoint(3.60000005364, 1.90000002831,  0), rosPoint(3.60000005364, 1.85000002757,  0), rosPoint(3.65000005439, 1.80000002682,  0), rosPoint(3.65000005439, 1.75000002608,  0), rosPoint(3.65000005439, 1.70000002533,  0), rosPoint(3.70000005513, 1.65000002459,  0), rosPoint(3.70000005513, 1.60000002384,  0), rosPoint(3.75000005588, 1.5500000231,  0), rosPoint(3.75000005588, 1.50000002235,  0), rosPoint(3.80000005662, 1.45000002161,  0), rosPoint(3.80000005662, 1.40000002086,  0)]



        while not rospy.is_shutdown():
            rospy.sleep(1)


if __name__ == '__main__':


    try:

        g = ptpSendOrder()
        g.main()

    except rospy.ROSInterruptException:
        pass
