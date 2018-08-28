import rospy
import rospkg
import sys
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
from tf import TransformListener
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import moveit_commander
import moveit_msgs.msg

class KinovaUtilities(object):

    def __init__(self):
        super(KinovaUtilities, self).__init__()
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('Kinova_Utilities', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:

        self.arm_group_name = "arm"
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        print "============ Printing arm joint values"
        #print self.arm_group.get_current_joint_values()

        self.gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)
        print "============ Printing gripper joint values"
        #print self.gripper_group.get_current_joint_values()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        current_pose = self.arm_group.get_current_pose()
        print "============ Current Pose"
        print current_pose
        print ""

        aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
        rospy.loginfo("Got: " + str(aruco_pose))

        # self.listener = TransformListener()
        # gripper_pos = geometry_msgs.msg.PoseStamped()
        # gripper_pos.header.frame_id = "j2n6s300_end_effector"
        # gripper_pos.pose.orientation.w = 1.0
        # start_pos_manual = self.listener.transformPose("world", gripper_pos)
        #
        # rospy.loginfo("Pose via manual TF is: ")
        # rospy.loginfo(start_pos_manual)



        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = -0.0823710076136
        pose_goal.orientation.x = -0.643457482628
        pose_goal.orientation.y = -0.20399941731
        pose_goal.orientation.z = 0.733186008385

        # pose_goal.orientation.w = 1

        # pose_goal.position.x = aruco_pose.pose.position.x
        # pose_goal.position.y = aruco_pose.pose.position.y
        # pose_goal.position.z = aruco_pose.pose.position.z

        # pose_goal.position.x = current_pose.pose.position.x
        # pose_goal.position.y = current_pose.pose.position.y
        # pose_goal.position.z = current_pose.pose.position.z

        pose_goal.position.x = aruco_pose.pose.position.x - 0.07
        pose_goal.position.y = aruco_pose.pose.position.y
        pose_goal.position.z = aruco_pose.pose.position.z - 0.03
        #
        #
        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        # self.move_gripper()
    ## END_SUB_TUTORIAL

    def move_gripper(self):
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = 0.71
        joint_goal[1] = 0.71
        joint_goal[2] = 0.71

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #self.gripper_group.go(joint_goal, wait=True)
        # self.gripper_group.set_named_target("Open")

        self.gripper_group.go(joint_goal, wait=True)

    def open_gripper(self):
        # We can get the joint values from the group and adjust some of the values:
        # joint_goal = self.gripper_group.get_current_joint_values()
        # joint_goal[0] = 0.2
        # joint_goal[1] = 0.2
        # joint_goal[2] = 0.2

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.set_named_target("Open")
        plan = self.gripper_group.plan()
        self.gripper_group.execute(plan)
        rospy.sleep(1)

    def close_gripper(self):
        #We can get the joint values from the group and adjust some of the values:
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = 1.2
        joint_goal[1] = 1.2
        joint_goal[2] = 1.2

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()

        # self.gripper_group.set_named_target("Close")
        # plan = self.gripper_group.plan()
        # self.gripper_group.execute(plan)
        # rospy.sleep(1)

    def strip_leading_slash(self, s):
        return s[1:] if s.startswith("/") else s

    def spawn_object(self):
        model_path = "/home/rafaelpossas/.gazebo/models/"

        table_pose = Pose(position=Point(x=0.6, y=0.5, z=0.67))
        table_reference_frame = "world"

        with open(model_path + "coke_can/model.sdf", "r") as table_file:
            table_xml = table_file.read().replace('\n', '')

        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("cafe_table", table_xml, "/", table_pose, table_reference_frame)

            
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
        return false

    def delete_object(self):
        return false


if __name__ == '__main__':
    utilities = KinovaUtilities()
