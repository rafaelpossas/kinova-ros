import rospy
import rospkg
import sys
from geometry_msgs.msg import PoseStamped, Pose
import geometry_msgs.msg
from tf import TransformListener
from tf2_ros import TransformException
import random
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    Pose,
    Point,
    Quaternion,
)
import moveit_commander
import moveit_msgs.msg

class KinovaUtilities(object):

    JOINT_1_MIN_MAX = (-3.14, 3.14)
    JOINT_2_MIN_MAX = (0.82, 5.46)
    JOINT_3_MIN_MAX = (0.33, 5.95)
    JOINT_4_MIN_MAX = (-3.14, 3.14)
    JOINT_5_MIN_MAX = (-3.14, 3.14)
    JOINT_6_MIN_MAX = (-3.14, 3.14)

    CUBE_GZ_NAME = "cube"

    X_CUBE_MIN = 0.76
    X_CUBE_MAX = 1.03

    Y_CUBE_MIN = 0.55
    Y_CUBE_MAX = 1.05

    def __init__(self):
        super(KinovaUtilities, self).__init__()
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('Kinova_Utilities', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        self.robot = moveit_commander.RobotCommander()

        group_names = self.robot.get_group_names()
        print "============ Robot Groups:", self.robot.get_group_names()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        self.scene = moveit_commander.PlanningSceneInterface()
        self.box_name = "cube"

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
        print self.robot.get_current_state()
        print ""

        current_pose = self.arm_group.get_current_pose()
        print "============ Current Pose"
        print current_pose
        print ""

        # aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
        # rospy.loginfo("Got: " + str(aruco_pose))
        #self.return_home_position()

        self.tf = TransformListener()

        rate = rospy.Rate(10.0)




        #print trans

        # rand_x = random.uniform(self.X_CUBE_MIN, self.X_CUBE_MAX)
        # rand_y = random.uniform(self.Y_CUBE_MIN, self.Y_CUBE_MAX)
        #
        # self.delete_gazebo_object(self.CUBE_GZ_NAME)
        # self.spawn_gazebo_object(x=rand_x, y=rand_y, model_name=self.CUBE_GZ_NAME)
        #
        # try:
        #     self.tf.waitForTransform("/world", "/tag_0", rospy.Time(), rospy.Duration(4))
        #     now = rospy.Time(0)
        #     (trans, rot) = self.tf.lookupTransform("/world", "/tag_0", now)
        #     self.add_rviz_object(self.CUBE_GZ_NAME, trans, rot)
        # except TransformException, e:
        #     rospy.logerr("Could not find the TAG: {0}".format(e))


        #self.add_object("cube", trans,rot)

        # tag_pos = geometry_msgs.msg.PoseStamped()
        # tag_pos.header.frame_id = "tag_0"
        # tag_pos.pose.orientation.w = 1.0
        # start_pos_manual = self.listener.transformPose("world", tag_pos)
        # #
        # rospy.loginfo("Pose via manual TF is: ")
        # rospy.loginfo(start_pos_manual)
        #
        # self.return_home_position()
        # self.pre_grasp_face_4(trans)

        # self.return_home_position()
        # self.pre_grasp_face_2(trans)
        # # #
        # self.return_home_position()
        # self.pre_grasp_face_1(trans)

        # self.return_home_position()
        try:
            self.joint_planning()
        except moveit_commander.MoveItCommanderException, e:
            print(e)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out

        return False


    def add_rviz_object(self, name, pos, rot):
        timeout = 4

        self.scene.remove_world_object(name)
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()

        # p.pose.position.x = 0.917182
        # p.pose.position.y = 0.858523
        # p.pose.position.z = 0.755576

        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1] - 0.03
        p.pose.position.z = pos[2] - 0.02

        p.pose.orientation.x = rot[0]
        p.pose.orientation.y = rot[1]
        p.pose.orientation.z = rot[2]
        p.pose.orientation.w = rot[3]

        self.scene.add_box(name, p, (0.05,0.05,0.05))

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    ## END_SUB_TUTORIAL
    def return_home_position(self):
        self.arm_group.set_named_target("Home")
        plan = self.arm_group.plan()
        self.arm_group.execute(plan)

        rospy.sleep(1)

    def joint_planning(self, joint_deltas=[]):
        goal = self.arm_group.get_current_joint_values()

        # joint_goal[0] = 4.8046
        # joint_goal[1] = 2.9248
        # joint_goal[2] = 1.0020
        # joint_goal[3] = 4.2031
        # joint_goal[4] = 1.4458
        # joint_goal[5] = 1.3233

        self.arm_group.go(goal, wait=True)
        self.arm_group.stop()

    def open_gripper(self):
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = 0.2
        joint_goal[1] = 0.2
        joint_goal[2] = 0.2

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #self.gripper_group.go(joint_goal, wait=True)
        # self.gripper_group.set_named_target("Open")

        self.gripper_group.go(joint_goal, wait=True)

    # def open_gripper(self):
    #     # We can get the joint values from the group and adjust some of the values:
    #     # joint_goal = self.gripper_group.get_current_joint_values()
    #     # joint_goal[0] = 0.2
    #     # joint_goal[1] = 0.2
    #     # joint_goal[2] = 0.2
    #
    #     # The go command can be called with joint values, poses, or without any
    #     # parameters if you have already set the pose or joint target for the group
    #     #self.gripper_group.go(joint_goal, wait=True)
    #     self.gripper_group.set_named_target("Open")
    #     plan = self.gripper_group.plan()
    #     self.gripper_group.execute(plan)
    #     rospy.sleep(1)
    def pre_grasp_face_2(self, trans):
        self.close_gripper()

        self.tf.waitForTransform("/world", "/tag_0", rospy.Time(0), rospy.Duration(4))

        cube_point = PointStamped()
        cube_point.header.frame_id = "tag_0"
        cube_point.header.stamp = rospy.Time(0)
        cube_point.point.x = -0.01
        cube_point.point.y = -0.1
        cube_point.point.z = 0.0

        trans = self.tf.transformPoint("world", cube_point)

        pose_goal = geometry_msgs.msg.Pose()

        # Pushing from right pose
        pose_goal.orientation.x = -0.0316320883026
        pose_goal.orientation.y = 0.999261878539
        pose_goal.orientation.z = 0.0204806930057
        pose_goal.orientation.w = 0.00745991303797

        pose_goal.position.x = trans.point.x
        pose_goal.position.y = trans.point.y
        pose_goal.position.z = trans.point.z

        self.arm_group.set_pose_target(pose_goal)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def pre_grasp_face_4(self, trans):
        self.close_gripper()

        self.tf.waitForTransform("/world", "/tag_0", rospy.Time(0), rospy.Duration(4))

        cube_point = PointStamped()
        cube_point.header.frame_id = "tag_0"
        cube_point.header.stamp = rospy.Time(0)
        cube_point.point.x = -0.02
        cube_point.point.y = 0.07
        cube_point.point.z = 0.0

        trans = self.tf.transformPoint("world", cube_point)

        pose_goal = geometry_msgs.msg.Pose()

        # Pushing from right pose
        pose_goal.orientation.x = 0.999713383089
        pose_goal.orientation.y = 0.0175424601738
        pose_goal.orientation.z = -0.00329246532167
        pose_goal.orientation.w = 0.0159553576136

        pose_goal.position.x = trans.point.x
        pose_goal.position.y = trans.point.y
        pose_goal.position.z = trans.point.z

        self.arm_group.set_pose_target(pose_goal)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def pre_grasp_face_1(self, trans):
        self.close_gripper()

        self.tf.waitForTransform("/world", "/tag_0", rospy.Time(0), rospy.Duration(4))

        cube_point = PointStamped()
        cube_point.header.frame_id = "tag_0"
        cube_point.header.stamp = rospy.Time(0)
        cube_point.point.x = 0.07
        cube_point.point.y = -0.01
        cube_point.point.z = 0.0

        trans = self.tf.transformPoint("world", cube_point)

        pose_goal = geometry_msgs.msg.Pose()

        # Pushing from back pose
        pose_goal.orientation.x = 0.74959557154
        pose_goal.orientation.y = -0.659549814281
        pose_goal.orientation.z = -0.0335475179298
        pose_goal.orientation.w = 0.0444419357171

        pose_goal.position.x = trans.point.x
        pose_goal.position.y = trans.point.y
        pose_goal.position.z = trans.point.z

        self.arm_group.set_pose_target(pose_goal)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()


    def close_gripper(self):
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = 1.2
        joint_goal[1] = 1.2
        joint_goal[2] = 1.2

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        #self.gripper_group.go(joint_goal, wait=True)
        # self.gripper_group.set_named_target("Open")

        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()
        self.gripper_group.clear_pose_targets()

    def strip_leading_slash(self, s):
        return s[1:] if s.startswith("/") else s



    # POSE: Y_MAX = 1.05 Y_MIN=0.55
    #       X_MAX = 0.76 X_MIN=1.03
    #       Z_MIN_MAX = 0.75
    def spawn_gazebo_object(self,x, y, model_path="/home/rafaelpossas/.gazebo/models/",
                     model_file_path="aruco_cube/aruco_cube.sdf",
                     model_name="cube"):

        model_pose = Pose(position=Point(x=x, y=y, z=0.75))
        model_reference_name = "world"

        with open(model_path + model_file_path, "r") as model_file:
            model_xml = model_file.read().replace('\n', '')

        rospy.wait_for_service('/gazebo/spawn_sdf_model')

        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf(model_name, model_xml, "/", model_pose, model_reference_name)

            
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
            return False
        return True

    def delete_gazebo_object(self,name="cube"):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            delete_sdf = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_sdf(name)
        except rospy.ServiceException, e:
            rospy.logerr("Delete SDF service call failed: {0}".format(e))
            return False

        return True


if __name__ == '__main__':
    utilities = KinovaUtilities()
