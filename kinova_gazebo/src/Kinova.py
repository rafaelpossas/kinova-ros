import rospy
import rospkg
import sys
from geometry_msgs.msg import PoseStamped, Pose
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
        print self.arm_group.get_current_joint_values()

        self.gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)
        print "============ Printing gripper joint values"
        print self.gripper_group.get_current_joint_values()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
        rospy.loginfo("Got: " + str(aruco_pose))
    ## END_SUB_TUTORIAL

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


        # self.gripper_group.set_named_target("Close")
        # plan = self.gripper_group.plan()
        # self.gripper_group.execute(plan)
        # rospy.sleep(1)

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
    utilities.open_gripper()
