#!/usr/bin/env python
import rospy
import actionlib
import helpers
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
from tour import TourMap


def movebase_client(curr_coord, next_coord):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    angle_to_goal = math.atan2(next_coord[1] - curr_coord[1], next_coord[0] - curr_coord[0])
    q_angle = quaternion_from_euler(0, 0, angle_to_goal, axes='sxyz')
    q = Quaternion(*q_angle)
    goal.target_pose.pose = Pose(Point(next_coord[0], next_coord[1], 0), q)

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    
def go_to_start(tour_map, curr_coord):
    start = helpers.find_start(curr_coord, tm.highlights, tm.zones)
    rospy.loginfo('Starting tour at node {}.'.format(start.id))

    try:
        success = movebase_client(curr_coord, start.coord)
        if success:
            tm.remove_highlight(start.id)
            new_coord = start.coord
            rospy.loginfo('Successfully arrived at node {}.'.format(start.id))
            print(start.blurb)
    except rospy.ROSInterruptException:
        rospy.loginfo('Something went wrong trying to go to node {}. ):'.format(start.id))
    
    return start

def continue_tour(tour_map, curr_highlight):
    next_highlight = helpers.find_closest_highlight(tour_map, curr_highlight)
    rospy.loginfo('Attempting to go from node {} to node {}.'.format(curr_highlight.id, next_highlight.id))
    curr_coord = curr_highlight.coord

    gates = tm.get_edge(curr_highlight.id, next_highlight.id).gates
    if gates is not None:
        rospy.loginfo('Gate(s) detected going from node {} to node {}.'.format(curr_highlight.id, next_highlight.id))
        for gate in gates:
            try:
                rospy.loginfo('Attempting to go to gate at coordinates {}.'.format(gate.coord))

                success = movebase_client(curr_coord, gate.coord)
                if success:
                    rospy.loginfo('Successfully arrived at gate at coordinates {}.'.format(gate.coord))
                    curr_coord = gate.coord

                    ans = raw_input('Can someone please open the door so that I am able to continue the tour? (y/n) ')
                    if ans == 'n' or ans == 'N':
                        tour_map.prune(gate.id)
                        tour_map.remove_highlight(next_highlight.id)
                        return curr_highlight
                    elif ans == 'y' or ans =='Y':
                        print('Thank you! :)')
                        rospy.loginfo('Gate at {} passed, continuing to node {}.'.format(gate.coord, next_highlight.id))
                    else:
                        raise Exception('Invalid answer! Must be "y" or "n".')
            except rospy.ROSInterruptException:
                rospy.loginfo('Something went wrong trying to go to the gate at {}. ):'.format(gate.coord))

    try:
        rospy.init_node('movebase_client_py')
        success = movebase_client(curr_coord, next_highlight.coord)
        if success:
            tm.remove_highlight(next_highlight.id)
            curr_coord = next_highlight.coord
            rospy.loginfo('Successfully arrived at node {}.'.format(next_highlight.id))
            print(next_highlight.blurb)
    except rospy.ROSInterruptException:
        rospy.loginfo('Something went wrong trying to go to node {}. ):'.format(next_highlight.id))

    return next_highlight

if __name__ == '__main__':
    tm = TourMap()
    rospy.init_node('movebase_client_py')
    curr_coord = helpers.gazebo_to_rviz_coords((1, 1))
    curr_highlight = go_to_start(tm, curr_coord)

    while len(tm.highlights) > 0:
        curr_highlight = continue_tour(tm, curr_highlight)

    rospy.loginfo("Done with tour!")