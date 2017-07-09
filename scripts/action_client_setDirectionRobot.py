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
        self.client = SimpleActionClient('set_direction', setDirectionRobotAction)

        # Waits until the action server has started up and started
        # listening for goals.
        # rospy.loginfo("Connect to server")
        self.client.wait_for_server()



    def sendDirection(self, action):


        # Creates a goal to send to the action server.

        # # Sends the goal to the action server.
        # rospy.loginfo("Send the given goal")
        self.client.send_goal(action)

        # # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        # # Prints out the result of executing the action
        # rospy.loginfo("Get the results")
        retour = self.client.get_result()  # A FibonacciResult

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



                if front < limit_distance and front != -1 :

                    if back < limit_distance and back != -1 :
                        action_set = True
                        action = setDirectionRobotGoal(direction = "stop", angle= 0, side="", backToPrevious=True)



                    elif right > limit_distance and left > limit_distance:
                        action_set = True
                        action = setDirectionRobotGoal(direction = "backward", angle= 0, side="", backToPrevious=True)


                    elif right < limit_distance and right != -1 :
                        action_set = True
                        action = setDirectionRobotGoal(direction = "rotate", angle= 90, side="left", backToPrevious=True)

                    elif left < limit_distance and left != -1 :
                        action_set = True
                        action = setDirectionRobotGoal(direction = "rotate", angle= 90, side="right", backToPrevious=True)


                elif back < limit_distance and back != -1 :
                    action_set = True
                    action = setDirectionRobotGoal(direction = "forward", angle= 0, side="", backToPrevious=True)





                if action_set:
                    self.ready_to_send = False
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