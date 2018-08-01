#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
//void openGripper(move_group_interface::MoveGroup &gripper, move_group_interface::MoveGroup &arm) {
//
//    std::map<std::string, double> gripper_joints;
//    gripper_joints["j2s7s300_joint_finger_1"] = 0.2;
//    gripper_joints["j2s7s300_joint_finger_2"] = 0.2;
//    gripper_joints["j2s7s300_joint_finger_3"] = 0.2;
//
//
//    gripper.setJointValueTarget(gripper_joints);
//    arm.setJointValueTarget("j2s7s300_joint_7", 3);

//}
namespace kinova {

    class JacoControl {

    private:
        ros::NodeHandle nh_;
        std::string robot_type_;
        bool robot_connected_;

        robot_model::RobotModelPtr robot_model_;
        planning_scene::PlanningScenePtr planning_scene_;
        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

        moveit::planning_interface::MoveGroupInterface *group_;
        moveit::planning_interface::MoveGroupInterface *gripper_group_;

        actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> *finger_client_;

        moveit::planning_interface::PlanningSceneInterface current_scene;
    public:
        ~JacoControl()
        {
            // shut down pub and subs
            //sub_joint_.shutdown();
            //sub_pose_.shutdown();

            // release memory
            delete group_;
            delete gripper_group_;
            delete finger_client_;
        }
        JacoControl(ros::NodeHandle &nh) {
            nh_ = nh;
            ros::NodeHandle pn("~");

            nh_.param<std::string>("/robot_type", robot_type_, "j2n6s300");
            nh_.param<bool>("/robot_connected", robot_connected_, true);

            // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
//            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//            robot_model_ = robot_model_loader.getModel();
//
//            // construct a `PlanningScene` that maintains the state of the world (including the robot).
//            planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
//            planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
//
//
            group_ = new moveit::planning_interface::MoveGroupInterface("arm");
            gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

            group_->setEndEffectorLink(robot_type_ + "_end_effector");

            finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
                    ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);

            while (robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the finger action server to come up");
            }
            clear_workscene();

            ros::WallDuration(1.0).sleep();

            current_scene.addCollisionObjects(create_workscene());

            ros::WallDuration(1.0).sleep();

            group_->clearPathConstraints();
            group_->setNamedTarget("Home");

            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode ret = group_->plan(my_plan);
            group_->execute(my_plan);

            ros::WallDuration(1.0).sleep();
            gripper_group_->setNamedTarget("Close");
            gripper_group_->move();




        }
        void clear_workscene() {

            std::vector<moveit_msgs::CollisionObject> collision_objects;

            moveit_msgs::CollisionObject table;
            table.id = "table";
            table.operation = table.REMOVE;

            moveit_msgs::CollisionObject pole;
            pole.id = "pole";
            pole.operation = pole.REMOVE;

            moveit_msgs::CollisionObject part;
            part.id = "part";
            part.operation = part.REMOVE;

            collision_objects.push_back(table);
            collision_objects.push_back(pole);
            collision_objects.push_back(part);
            current_scene.addCollisionObjects(collision_objects);

        }
        std::vector<moveit_msgs::CollisionObject> create_workscene() {

            std::vector<moveit_msgs::CollisionObject> collision_objects;

            moveit_msgs::CollisionObject table;
            table.id = "table";


            shape_msgs::SolidPrimitive table_primitive;
            table_primitive.type = table_primitive.BOX;
            table_primitive.dimensions.resize(3);
            table_primitive.dimensions[0] = 0.5;
            table_primitive.dimensions[1] = 1.5;
            table_primitive.dimensions[2] = 0.35;

            geometry_msgs::Pose table_pose;
            table_pose.position.x = 0.4;
            table_pose.position.y = 0;
            table_pose.position.z = 0.175;
            table_pose.orientation.w = 1.0;

            table.primitives.push_back(table_primitive);
            table.primitive_poses.push_back(table_pose);
            table.operation = table.ADD;

            collision_objects.push_back(table);

            moveit_msgs::CollisionObject pole;
            pole.id = "pole";

            shape_msgs::SolidPrimitive pole_primitive;
            pole_primitive.type = pole_primitive.BOX;
            pole_primitive.dimensions.resize(3);
            pole_primitive.dimensions[0] = 0.3;
            pole_primitive.dimensions[1] = 0.1;
            pole_primitive.dimensions[2] = 1.0;

            geometry_msgs::Pose pole_pose;
            pole_pose.position.x = 0.5;
            pole_pose.position.y = -0.35;
            pole_pose.position.z = 0.85;
            pole_pose.orientation.w = 1.0;

            pole.primitives.push_back(pole_primitive);
            pole.primitive_poses.push_back(pole_pose);
            pole.operation = pole.ADD;

            collision_objects.push_back(pole);

            moveit_msgs::CollisionObject part;
            part.id = "part";

            shape_msgs::SolidPrimitive part_primitive;
            part_primitive.type = part_primitive.BOX;
            part_primitive.dimensions.resize(3);
            part_primitive.dimensions[0] = 0.1;
            part_primitive.dimensions[1] = 0.05;
            part_primitive.dimensions[2] = 0.3;

            geometry_msgs::Pose part_pose;
            part_pose.position.x = 0.3;
            part_pose.position.y = -0.5;
            part_pose.position.z = 0.5;
            part_pose.orientation.w = 1.0;

            part.primitives.push_back(part_primitive);
            part.primitive_poses.push_back(part_pose);
            part.operation = part.ADD;

            collision_objects.push_back(part);

            return collision_objects;
        }

        void pick(moveit::planning_interface::MoveGroupInterface &group) {
            std::vector<moveit_msgs::Grasp> grasps;

            geometry_msgs::PoseStamped p;
            p.header.frame_id = "root";
            p.pose.position.x = 0.28;
            p.pose.position.y = -0.50795596691818223;
            p.pose.position.z = 0.47650131991195382;
            p.pose.orientation.x = -0.68737298105709721;
            p.pose.orientation.y = 0.21350456946033522;
            p.pose.orientation.z = -0.69310141100728517;
            p.pose.orientation.w = 0.039301625818107352;
            moveit_msgs::Grasp g;
            g.grasp_pose = p;

            g.pre_grasp_approach.direction.vector.x = 1.0;
            g.pre_grasp_approach.direction.header.frame_id = "j2s7s300_link_7";
            g.pre_grasp_approach.min_distance = 0.2;
            g.pre_grasp_approach.desired_distance = 0.4;

            g.post_grasp_retreat.direction.header.frame_id = "root";
            g.post_grasp_retreat.direction.vector.z = 1.0;
            g.post_grasp_retreat.min_distance = 0.1;
            g.post_grasp_retreat.desired_distance = 0.25;

            //openGripper(g.pre_grasp_posture);

            //closeGripper(g.grasp_posture);

            grasps.push_back(g);
            //group.setSupportSurfaceName("table");
            group.pick("part", grasps);
        }
    };
//    void openGripper(trajectory_msgs::JointTrajectory& posture){
//        posture.joint_names.resize(3);
//        posture.joint_names[0] = "j2s7s300_joint_finger_1";
//        posture.joint_names[1] = "j2s7s300_joint_finger_2";
//        posture.joint_names[2] = "j2s7s300_joint_finger_3";
//
//        posture.points.resize(1);
//        posture.points[0].positions.resize(3);
//        posture.points[0].positions[0] = 0.2;
//        posture.points[0].positions[1] = 0.2;
//        posture.points[0].positions[2] = 0.2;
//    }
//    void closeGripper(trajectory_msgs::JointTrajectory& posture){
//        posture.joint_names.resize(3);
//        posture.joint_names[0] = "j2s7s300_joint_finger_1";
//        posture.joint_names[1] = "j2s7s300_joint_finger_2";
//        posture.joint_names[2] = "j2s7s300_joint_finger_3";
//
//        posture.points.resize(1);
//        posture.points[0].positions.resize(3);
//        posture.points[0].positions[0] = 1.2;
//        posture.points[0].positions[1] = 1.2;
//        posture.points[0].positions[2] = 1.2;
//    }
//    void cartesian_planning(moveit::planning_interface::MoveGroupInterface &group)
//    {
//        std::vector<geometry_msgs::Pose> waypoints;
//        geometry_msgs::PoseStamped start_pose = group.getCurrentPose();
//        waypoints.push_back(start_pose.pose);
//
//        geometry_msgs::PoseStamped end_pose;
//        end_pose.header.frame_id = "root";
//        end_pose.pose.position.x = 0.19239438219466182;
//        end_pose.pose.position.y = -0.50795596691818223;
//        end_pose.pose.position.z = 0.47650131991195382;
//        end_pose.pose.orientation.x = -0.68737298105709721;
//        end_pose.pose.orientation.y = 0.21350456946033522;
//        end_pose.pose.orientation.z = -0.69310141100728517;
//        end_pose.pose.orientation.w = 0.039301625818107352;
//        waypoints.push_back(end_pose.pose);
//
//        moveit_msgs::RobotTrajectory trajectory;
//        const double jump_threshold = 0.0;
//        const double eef_step = 0.01;
//        double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
//        ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
//        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//        my_plan.trajectory_ = trajectory;
//        group.execute(my_plan);
//
//
//    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::JacoControl pick_place(node);

    return 0;
}
