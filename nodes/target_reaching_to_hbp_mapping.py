#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, String
from visuomotor_manager import ArmManager
import actionlib
from fzi_manipulation_msgs.msg import RawTrajectory, PlayTrajectoryAction, PlayTrajectoryGoal
from fzi_std_msgs.msg import Float64Array
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class TargetReachingToHBPMapping:
    def __init__(self):
        self.has_joint_state = False
        self.has_joint_cmd = False
        self.pose_reference_frame = rospy.get_param('~pose_reference_frame', 'table_corner_link')
        self.arm_manager = ArmManager(self.pose_reference_frame)
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        robot = 'hbp'
        self.arm_joint_cmds = {}
        self.last_positions_to_send = []
        self.arm_1_joint_cmd_sub = rospy.Subscriber('/' + robot + '/arm_1_joint/cmd_pos', Float64, self.cmd_callback, callback_args="arm_1_joint", queue_size=1)
        self.arm_2_joint_cmd_sub = rospy.Subscriber('/' + robot + '/arm_2_joint/cmd_pos', Float64, self.cmd_callback, callback_args="arm_2_joint", queue_size=1)
        self.arm_3_joint_cmd_sub = rospy.Subscriber('/' + robot + '/arm_3_joint/cmd_pos', Float64, self.cmd_callback, callback_args="arm_3_joint", queue_size=1)
        self.data_pub = rospy.Publisher('/data_pub', String, queue_size=1)
        arm_trajectory_controller_param = "/arm/" + "arm_eff_traj_controller"
        self.arm_traj_client = actionlib.SimpleActionClient(arm_trajectory_controller_param + "/follow_joint_trajectory", FollowJointTrajectoryAction)

    def cmd_callback(self, cmd, joint_name):
        self.has_joint_cmd = True
        self.arm_joint_cmds[joint_name] = cmd.data
        to_pub = "cmds: ["
        if "arm_1_joint" in self.arm_joint_cmds:
            to_pub += str(self.arm_joint_cmds["arm_1_joint"])
        to_pub += " ,"
        if "arm_2_joint" in self.arm_joint_cmds:
            to_pub += str(self.arm_joint_cmds["arm_2_joint"])
        to_pub += " ,"
        if "arm_3_joint" in self.arm_joint_cmds:
            to_pub += str(self.arm_joint_cmds["arm_3_joint"])
        to_pub += "]"
        self.data_pub.publish(to_pub)

    def joint_states_callback(self, joint_state):
        self.last_joint_state = list(joint_state.position[0:len(self.arm_manager.joint_names)])
        if not self.has_joint_state:
            self.has_joint_state = True

    def send_arm_trajectory2(self, duration=0.5):
        rospy.loginfo("send_arm_trajectory2 method")
        if not self.has_joint_state or not self.has_joint_cmd:
            return
        positions_to_send = self.last_joint_state
        if "arm_1_joint" in self.arm_joint_cmds:
            positions_to_send[0] = self.arm_joint_cmds["arm_1_joint"]
        if "arm_2_joint" in self.arm_joint_cmds:
            positions_to_send[1] = self.arm_joint_cmds["arm_2_joint"]
        if "arm_3_joint" in self.arm_joint_cmds:
            positions_to_send[2] = self.arm_joint_cmds["arm_3_joint"]
        rospy.loginfo("current pos: {}".format(positions_to_send))
        rospy.loginfo("last pos: {}".format(self.last_positions_to_send))
        if self.last_positions_to_send:
            rospy.loginfo("diff: {}".format(list(map(lambda x,y:abs(x-y), self.last_positions_to_send, positions_to_send))))
            pos_diff = list(map(lambda x,y:abs(x-y), self.last_positions_to_send, positions_to_send))
            if all(diff <= 0.009 for diff in pos_diff):
                return
        self.last_positions_to_send = positions_to_send
        self.arm_joint_cmds = {}
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory.joint_names = self.arm_manager.joint_names
        waypoint = JointTrajectoryPoint()
        for i in range(len(self.arm_manager.joint_names)):
            waypoint.positions = positions_to_send
        waypoint.time_from_start = rospy.Duration.from_sec(duration)
        arm_goal.trajectory.points.append(waypoint)
        self.arm_traj_client.send_goal(arm_goal)

    def send_arm_trajectory(self, duration=0.5):
        rospy.loginfo("send_arm_trajectory method")
        if not self.has_joint_state or not self.has_joint_cmd:
            return
        self.arm_manager.new_traj(trajectory_type=RawTrajectory.TT_JOINT_BASED)
        path = RawTrajectory()
        path.joint_names = self.arm_manager.joint_names
        path.waypoint_types.append(RawTrajectory.TT_JOINT_BASED)
        path.waypoint_names.append('wp')
        positions_to_send = self.last_joint_state
        if "arm_1_joint" in self.arm_joint_cmds:
            positions_to_send[0] = self.arm_joint_cmds["arm_1_joint"]
        if "arm_2_joint" in self.arm_joint_cmds:
            positions_to_send[1] = self.arm_joint_cmds["arm_2_joint"]
        if "arm_3_joint" in self.arm_joint_cmds:
            positions_to_send[2] = self.arm_joint_cmds["arm_3_joint"]
        rospy.loginfo("current pos: {}".format(positions_to_send))
        rospy.loginfo("last pos: {}".format(self.last_positions_to_send))
        if self.last_positions_to_send:
            rospy.loginfo("diff: {}".format(list(map(lambda x,y:abs(x-y), self.last_positions_to_send, positions_to_send))))
            pos_diff = list(map(lambda x,y:abs(x-y), self.last_positions_to_send, positions_to_send))
            if all(diff <= 0.009 for diff in pos_diff):
                return
        self.last_positions_to_send = positions_to_send
        self.arm_joint_cmds = {}
        path.positions.append(Float64Array(data=positions_to_send))
        path.name = 'programmed'
        path.reference_frame = self.arm_manager.robot_base_link
        path.type = RawTrajectory.TT_JOINT_BASED
        rospy.loginfo("cancel all")
        self.arm_manager.arm_client.cancel_all_goals()
        self.arm_manager.append_point(0, path)
        rospy.loginfo("send_arm_trajectory to client")
        self.arm_manager.arm_client.send_goal(PlayTrajectoryGoal())
        #self.arm_manager.arm_client.wait_for_result()
        #rospy.sleep(1.0)

        #to_pub = String()
        #s = ''
        #for i in range(len(joint_names)):
            #s += '{}: {} - ({})\n'.format(i, joint_names[i], positions[i])
        #to_pub.data = s
        #self.data_pub.publish(to_pub)

def main(argv=None):
    rospy.init_node("TargetReachingToHBPMapping")
    hbp_mapping = TargetReachingToHBPMapping()
    rospy.loginfo("TargetReachingToHBPMapping initialized")
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        #hbp_mapping.arm_manager.arm_client.cancel_all_goals()
        #rospy.loginfo("cancel all")
        hbp_mapping.send_arm_trajectory2()
        rate.sleep()

if __name__ == "__main__":
    main()
