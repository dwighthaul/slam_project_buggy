#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int16

from slam_project_buggy.msg import AllSidesDistance
from sensor_msgs.msg import LaserScan

from slam_project_buggy.msg import setDirectionRobotAction, setDirectionRobotGoal


import slam_project_buggy
from actionlib import SimpleActionClient



limit_distance = 0.3
list_sides = ["front", "left", "back", "right"]

class clientSetDirectionRobot():


    direction = ""

    ## Count the number of time that the listener 
    buffer_loop =0
    buffer_loop_max = 1
    turnSide= ""
    ready_to_send = True

    def __init__(self):
        rospy.init_node('send_direction_client', anonymous=True)

        # queue_size=1 to avoid stacking of the calls
        rospy.Subscriber("allMeanValues", AllSidesDistance, self.setDirection_cb, queue_size=1)
        rospy.loginfo('Init Client')
        self.client = SimpleActionClient('speed_from_action', setDirectionRobotAction)

        self.client.wait_for_server()



    def sendDirection(self, action):

        self.client.send_goal(action)

        self.client.wait_for_result()

        retour = self.client.get_result()

        return retour





    # direction, angle, side
    # direction: rotate, forward, backward, stop : str
    # angle: -90 to 90 : int8
    # side: left or right : str

    # see setDirectionRobot in the action folder
    def setDirection_cb(self, data):

        if self.ready_to_send:

            try:

                front = data.listDistances[0].distance
                left  = data.listDistances[1].distance
                back  = data.listDistances[2].distance
                right = data.listDistances[3].distance
                rospy.loginfo('\nF: %.2f \nR: %.2f \nL: %.2f \nB: %.2f', front, right, left, back)


                action_set = False

                angle_r= 0
                side_r=""
                backToPrevious_r = True
                direction_r = ""

                if front < limit_distance and front != -1 :

                    if back < limit_distance and back != -1 :
                        action_set = True
                        direction = "stop"


                    elif right > limit_distance and left > limit_distance:
                        action_set = True
                        direction_r = "backward"



                    elif right < limit_distance and right != -1 :
                        action_set = True
                        direction_r = "rotate"
                        angle_r = 90
                        side_r ="left"

                    elif left < limit_distance and left != -1 :
                        action_set = True
                        direction_r = "rotate"
                        angle_r = 90
                        side_r ="right"


                elif back < limit_distance and back != -1 :
                    action_set = True
                    direction_r = "forward"





                if action_set:
                    self.ready_to_send = False
                    action = setDirectionRobotGoal(direction = direction_r, angle=angle_r, side=side_r, backToPrevious=backToPrevious_r)
                    rospy.loginfo('Call the server, send action: %s %s', action.direction, action.side)


                    self.sendDirection(action)


                    self.ready_to_send = True







            except IndexError:
                pass



if __name__ == '__main__':
    try:
        d = clientSetDirectionRobot()
        rospy.spin()


    except rospy.ROSInterruptException:
        print("Error")