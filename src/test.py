#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Entrance: (3.6, -8.3)
# Atrium Center: (-3.6, -12.5)
# Atrium Lower Left: (-5.0, -15.8)
# End Hallway: (-3.25, -2.6)
# Devon 115 near door: (-4.2, -7.8)
# Devon 115 left: (-8.8, -9.3)


def movebase_client(coord):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = coord[0]
    goal.target_pose.pose.position.y = coord[1]
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    tour_highlights = [(3.6, -8.3), (-3.6, -12.5), (-5.0, -15.8), (-3.25, -2.6), (-4.2, -7.8), (-8.8, -9.3)]

    for coord in tour_highlights:
        try:
            rospy.init_node('movebase_client_py')
            result = movebase_client(coord)
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    rospy.loginfo("Done with tour!")