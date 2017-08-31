#!/usr/bin/env python

import rospy
import roslib
import rospy
import tf


# rosPoint as because of the Point from the vectors modules
from nav_msgs.msg import  OccupancyGrid
from geometry_msgs.msg import  Point as rosPoint
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Vector3, Quaternion, PolygonStamped

from std_msgs.msg import ColorRGBA

import math
from vectors import Point, Vector
from visualization_msgs.msg import Marker

# For the action
from slam_project_buggy.msg import setDirectionRobotAction, setDirectionRobotGoal, ListPoints

from a_star_algo import *




class pointToPointPath():

    doesPointSelected = False
    
    pointSelected = rosPoint()

    pointSelected.x = 0.0
    pointSelected.y = 0.0
    pointSelected.z = 0.0

    pointRobot = rosPoint()
    vectorRobot = Quaternion()

    vectorRobotToPointselected = Vector3()

    threshold = 30

    def __init__(self):
        rospy.init_node('navig_to_end_point', anonymous=True)

        rospy.Subscriber("final_target", PointStamped, self.cb_get_final_point)
        rospy.Subscriber("costmap/costmap/costmap", OccupancyGrid, self.cb_get_map)
        rospy.Subscriber("slam_out_pose", PoseStamped, self.cb_get_robot_position)

        self.mapIsSet = False
        self.finalPointIsSet = False
        self.robot_position = False

        self.goal_position = rospy.Publisher('goal_position_marker', Marker, queue_size=1)
        self.start_point = rospy.Publisher('start_point_marker', Marker, queue_size=1)
        self.path_points_marker = rospy.Publisher('path_points_marker', Marker, queue_size=1)

        self.path_points = rospy.Publisher('list_point_path', ListPoints, queue_size=1)

        self.main()

    def cb_get_robot_position(self, data):
        self.robot_position = data
        self.robot_positionIsSet = True


    def cb_get_map(self, data):
        self.map = data
        print("Map is set")
        self.mapIsSet = True



    def cb_get_final_point(self, data):


        if self.mapIsSet and self.robot_positionIsSet:
            costmap = self.map
            ListPointStamped = self.createMarker(0.012, 0.012, 0.039, "/map", Marker.LINE_STRIP)
            ListPointStamped.points = []
            print(ListPointStamped.points)
            self.path_points_marker.publish(ListPointStamped)


            rospy.loginfo("Target point selected")
            resolution = costmap.info.resolution
            self.pointSelected = data.point

            GoalPointStamped = self.createMarker(0.886, 0.886, 0.984, "/map", Marker.SPHERE)
            gx = data.point.x
            gy = data.point.y

            intgx = int(gx)
            intgy = int(gy)

            g_point = rosPoint(gx, gy, 0)

            width = costmap.info.width
            height = costmap.info.height

            deltaWidth = width/2
            deltaHeight = height/2

            GoalPointStamped.pose.position = g_point
            self.goal_position.publish(GoalPointStamped)


            self.robot_position.pose.position

            convert = resolution
            convert = 50 / 2.5

            start = Point(int(deltaWidth + convert * self.robot_position.pose.position.y), int(deltaHeight + convert * self.robot_position.pose.position.x))
            goal = Point(int(deltaWidth + (gy * convert)), int(deltaHeight + (gx * convert)))

            listPoint = self.createPath(costmap, start, goal)

            self.path_points.publish(listPoint)

        else:
            rospy.loginfo("The map has not been publish yet")


    def createPath(self, costmap, start, goal):
        rospy.loginfo("Creating Path")

        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution

        deltaWidth = width/2
        deltaHeight = height/2

        default_value_grid = "."
        wallValue = "#"

        grid = []
        row = [defaultValue] * height 
        for i in range(width):
            grid.append(list(row))




        # create list id wall
        wallList = []
        for t in xrange(len(costmap.data)):
            if costmap.data[t] > self.threshold:
                wallList.append(t)

        # print(wallList)
        # transform list to grid




        t1 = time.time()
        createPath = PathCreator(grid, start, goal, height, width)
        rospy.loginfo("Time initiate the PathCreator (seconds): " + str(time.time()-t1) )

        t2 = time.time()
        createPath.fillWalls(wallList)
        rospy.loginfo("Time to fill the walls (seconds): " + str(time.time()-t2) )

        t3 = time.time()
        listPoint = createPath.main()
        rospy.loginfo("Time to generate the route (seconds): " + str(time.time()-t3) )


        StartPointStamped = self.createMarker(0.039, 0.031, 0.129, "/map", Marker.SPHERE)
        sx = resolution * (listPoint[0].y - deltaWidth)
        sy = resolution * (listPoint[0].x - deltaHeight)
        s_point = rosPoint(sx, sy, 0)
        StartPointStamped.pose.position = s_point


        ListPointStamped = self.createMarker(0.055, 0.455, 0.608, "/map", Marker.LINE_STRIP)

        ListPointRos = []
        ListPointsColor = []

        # print("[")

        for p in range(1, len(listPoint)):
            x = resolution * (listPoint[p].x - deltaWidth)
            y = resolution * (listPoint[p].y - deltaHeight)
            point = rosPoint(x, y, 0)
            ListPointRos.append(point)
            # print("Point("+str(x) + ", " + str(y) + ",  0),")

            percentage = (p*100/len(listPoint))

            r = 0.039 * (1-percentage/100) + (percentage * 0.886 /100)
            g = 0.031 * (1-percentage/100) + (percentage * 0.886 /100)
            b = 0.129 * (1-percentage/100) + (percentage * 0.984 /100)

            ListPointsColor.append(ColorRGBA(r, g, b, 1))

        ListPointStamped.points = ListPointRos


        self.start_point.publish(StartPointStamped)
        self.path_points_marker.publish(ListPointStamped)


        rospy.loginfo("Creating Path: Done")
        return ListPointRos



    def createMarker(self, r, g, b, base_frame, typeMarker):
        marker = Marker()
        marker.header.frame_id = base_frame
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
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



    def createSubMap(self, grid):
        subgrid = grid[482:576]

        sub_width = len(subgrid)
        sub_height = len(subgrid[0])

        print("sub_width  :" + str(sub_width))
        print("sub_height :" + str(sub_height))

        findFirst = False
        indiceLast = 0
        minimal = 1024
        maximal = 0


        for sw in range(sub_width):
            for sh in range(sub_height):
                if subgrid[sw][sh] != default_value_grid:
                    indiceLast = sh

                    if not findFirst:
                        findFirst = True
                        if sh < minimal:
                            minimal = sh

            if indiceLast> maximal:
                maximal = indiceLast

            findFirst = False

        print("Max " + str(maximal))
        print("Min " + str(minimal))

        sub_subgrid = subgrid

        for sw in range(sub_width):
            sub_subgrid[sw]= sub_subgrid[sw][450:545]


        print(len(sub_subgrid))
        print(len(sub_subgrid[0]))

        indice = 0

        indiceList = []

        for ssw in xrange(len(sub_subgrid)):
            row = ""
            for ssh in xrange(len(sub_subgrid[ssw])):
                if sub_subgrid[ssw][ssh] != default_value_grid:
                    indiceList.append(indice)
                row = row + sub_subgrid[ssw][ssh]
                indice = indice +1
            print(row),


            print("")

        


        self.getFirstAndLastChar(data)



    def getFirstAndLastChar(self, data):
        width = data.info.width
        height = data.info.height

        findFirst = False
        indiceLast = 0

        i = 0        


        for w in range(width):
            for h in range(height):
                if data.data[i] != 0:

                    indiceLast = i
                    if not findFirst:
                        findFirst = True
                        print("First:" + str(i))
                i=i+1

        print(indiceLast)









    def main(self):



        while not rospy.is_shutdown():

            rospy.sleep(1)


if __name__ == '__main__':


    try:

        g = pointToPointPath()


    except rospy.ROSInterruptException:
        pass


