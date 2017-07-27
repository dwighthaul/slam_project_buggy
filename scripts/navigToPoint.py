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

from slam_project_buggy.msg import LaserMsgDistance
from slam_project_buggy.msg import AllSidesDistance

import math
from vectors import Point, Vector

# For the action
from slam_project_buggy.msg import setDirectionRobotAction, setDirectionRobotGoal





class navigToPoint():

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

        rospy.Subscriber("clicked_point", PointStamped, self.callback_save_point_clicked)
        rospy.Subscriber("slam_out_pose", PoseStamped, self.callback_save_point_robot)

        self.pubPolygon = rospy.Publisher('angleRepresentation', PolygonStamped, queue_size=10)


        self.MarkerArrowRobot = rospy.Publisher('directionArrowRobot', Marker, queue_size=10)
        self.ArrowToTarget = rospy.Publisher('directionToTarget', Marker, queue_size=10)

        # Action to send 
        self.client = SimpleActionClient('speed_from_action', setDirectionRobotAction)


        rospy.loginfo("Looking for a sever")
        # self.client.wait_for_server()
        rospy.loginfo("Server found")

        self.main()


    def callback_save_point_clicked(self, data):
        print(data)
        self.doesPointSelected = True
        self.pointSelected = data.point

    def callback_save_point_robot(self, data):
        self.pointRobot = data.pose.position
        self.vectorRobot = data.pose.orientation



    def createMarker(self, r, g, b, base_frame, typeMarker):
        marker = Marker()
        marker.header.frame_id = base_frame
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0

        marker.type = typeMarker
        marker.action = marker.ADD
            
        t = rospy.Duration()
        marker.lifetime = t

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        return marker


    def publishMarkers(self):

        self.MarkerArrowRobot.publish(self.marker)
        self.ArrowToTarget.publish(self.marketTarget)
        self.pubPolygon.publish(self.polygonAngleSamped)

        self.polygonAngleSamped.header.stamp = rospy.Time.now()

        self.polygonAngleSamped.polygon.points = []
        self.polygonAngleSamped.polygon.points.append(self.pointRobot)
        self.polygonAngleSamped.polygon.points.append(self.pointSelected)

        dist = math.sqrt((self.pointSelected.x - self.pointRobot.x) ** 2 + (self.pointSelected.y - self.pointRobot.y) ** 2)
        vec_pointSelected = Point(self.pointSelected.x, self.pointSelected.y, 0)
        vec_pointRobot = Point(self.pointRobot.x, self.pointRobot.y, 0)

        angleRobotRad = 2 * math.asin(self.vectorRobot.z)
        angleRobotDeg = math.degrees(angleRobotRad)


        vec_robot = Vector(math.cos(angleRobotRad), math.sin(angleRobotRad), 0)


        farPointRobot = Point(dist*vec_robot.x +vec_pointRobot.x, dist*vec_robot.y + vec_pointRobot.y, 0)
        self.polygonAngleSamped.polygon.points.append(rosPoint(farPointRobot.x, farPointRobot.y, 0))

        vec_between_Robot = Vector.from_points(vec_pointRobot, farPointRobot)
        vec_between_Target = Vector.from_points(vec_pointRobot, vec_pointSelected)

        angle_between = 0
        dot = vec_between_Robot.x*-vec_between_Target.y + vec_between_Robot.y*vec_between_Target.x

        try :
            angle_between = vec_between_Robot.angle(vec_between_Target)


        except ZeroDivisionError:
            print("You can't divide by zero")

        # self.marketTarget.pose.orientation.z = math.sin(angle_between * math.pi / 180)

        # Positive : right 
        return [dist, angle_between, (dot>0)]



    def updateDirection(self, angle, dist, isright):

        if angle > 10:
            print("rotate " + ("right" if isright else "left"))
            # self.rotate(angle, isright)

        elif dist > 0.5:
            print("front")
            # self.goforward()
        else:
            print("stop")
            # self.stop()


    def goforward(self):

        action = setDirectionRobotGoal(direction = "forward", angle=0, side="", publishToArduino=True, backToPrevious=False)

        self.client.send_goal(action)
        self.client.wait_for_result()

        retour = self.client.get_result()




    def rotate(self, angle, isright):
        a_side = "right" if isright else "left"

        action = setDirectionRobotGoal(direction = "rotate", angle=0, side=a_side, publishToArduino=True, backToPrevious=False)

        self.client.send_goal(action)
        self.client.wait_for_result()

        retour = self.client.get_result()
        # print('rotate')

    def stop(self):

        action = setDirectionRobotGoal(direction = "stop", angle=0, side="", publishToArduino=True, backToPrevious=False)

        self.client.send_goal(action)
        self.client.wait_for_result()

        retour = self.client.get_result()




    def main(self):

        self.marker = self.createMarker(0.25, 0.18, 0.45, "/laser", Marker.ARROW)
        self.marketTarget = self.createMarker(0.49, 0.1, 0.04, "/map", Marker.ARROW)

        angle_between = 0


        self.polygonAngleSamped = PolygonStamped()
        self.polygonAngleSamped.header.frame_id = "/map";


        while not rospy.is_shutdown():

            if self.doesPointSelected:

                [dist, angle, isright] = self.publishMarkers()

                # print(angle)

                self.updateDirection(angle, dist, isright)


            # self.pub.publish(dist)
            rospy.sleep(0.1)


if __name__ == '__main__':


    try:

        g = navigToPoint()


    except rospy.ROSInterruptException:
        pass
