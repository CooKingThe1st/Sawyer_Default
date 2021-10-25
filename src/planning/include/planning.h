#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <string>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


namespace my_planning
{
    class MyPlanningClass
    {
        public:
            MyPlanningClass(): move_group(PLANNING_GROUP)
            {
                move_group.allowReplanning(true);
                move_group.setNumPlanningAttempts(10);
                move_group.setPlanningTime(10);

                // visual_tools.reset(new rviz_visual_tools::RvizVisualTools("world","/rviz_visual_markers"));
                // visual_tools.deleteAllMarkers();
                // // Remote control is an introspection tool that allows users to step through a high level script
                // // via buttons and keyboard shortcuts in RViz
                // visual_tools.loadRemoteControl();

                // text_pose.translation().z() = 1.75;
                // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
                // visual_tools.trigger();

                // ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
                // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
            }
//define function
            void goToPoseGoal(geometry_msgs::Pose &pose);
            void goToJointState(std::vector<double> &diff_joint);
            void goToJointZero();
            void cartesianPath(std::vector<double> waypoints);
            void safeToPoseGoal(geometry_msgs::Pose pose);
            void resetValues();
            void addObjects();
            void makeWall(std::string blk_name, double *pose);
            void removeObjects();

            std::size_t pick(geometry_msgs::Pose cube_pose, double hover_distance);
            void place(geometry_msgs::Pose tray_pose, double hover_distance);

            void sorting(std::size_t color_code);
            void test(std::size_t id_test);

        private:
        // declare
            const std::string PLANNING_GROUP = "right_arm";

            moveit::planning_interface::MoveGroupInterface move_group;
            moveit::planning_interface::PlanningSceneInterface virtual_world;
            const robot_state::JointModelGroup* joint_model_group;
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;

             // Visualization
            // ^^^^^^^^^^^^^
            //
            // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
            // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
            // moveit_visual_tools::MoveItVisualTools visual_tools;
            //     // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
            // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();

    };
}
