#include <planning.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

//define target pose
geometry_msgs::Pose target_pose_random;

//define target joint
std::vector<double> target_diff_joint_left{-1.0, -1.0, -1.0, -1.0};
std::vector<double> target_diff_joint_right{-1.0, 1.0, 1.0, 1.0};
std::vector<double> target_diff_joint0_left{-0.5};
std::vector<double> target_diff_joint0_right{+0.5};
// define target cartesian path, y  const, pair z,x
std::vector<double> target_path_line{-0.1, 0, 0.2, 0, -0.1, 0};
std::vector<double> target_path_tri{0.1, 0, -0.2, -0.1, 0, 0.2};

// should be careful here, coz of bug: https://answers.ros.org/question/350643/node-crashes-with-ros-does-not-seem-to-be-running/
my_planning::MyPlanningClass *plan_obj;
// actually this can't be done though, callback is forbidden
// https://groups.google.com/g/moveit-users/c/s9b7IJuKRKY
// https://answers.ros.org/question/231025/moveit-planning-not-continue-after-successful-planning/
// but function can be implemented though   
// https://answers.ros.org/question/334689/how-to-use-move-group-interface-in-a-sub-function/
// hereby there are two choices for us

// for getting the zero pose, consider open rviz visual
// https://answers.ros.org/question/279245/unable-to-move-arm-with-moveit/

void chatbotCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("From topic chat bot command: [%s]", msg->data.c_str());
  plan_obj->resetValues();

    if (msg -> data == "pose_random")
            plan_obj->goToPoseGoal(target_pose_random);
        else
    if (msg -> data == "pose_zero")
            plan_obj->goToJointZero();
        else
    if (msg -> data == "joint_left")
            plan_obj->goToJointState(target_diff_joint0_left);
        else
    if (msg -> data == "joint_right")
            plan_obj->goToJointState(target_diff_joint0_right);
        else
    if (msg -> data == "path_line")
            plan_obj->cartesianPath(target_path_line);
        else
    if (msg -> data == "path_tri")
            plan_obj->cartesianPath(target_path_tri);
        else
    if (msg -> data == "auto_sort")
            plan_obj->sorting(7);
        else
    if (msg -> data == "sort_red")
            plan_obj->sorting(0);
        else
    if (msg -> data == "sort_blue")
            plan_obj->sorting(1);
        else
    if (msg -> data == "sort_green")
            plan_obj->sorting(2);
        else
    if (msg -> data == "test_red")
            plan_obj->test(0);
        else
    if (msg -> data == "test_blue")
            plan_obj->test(1);
        else
    if (msg -> data == "test_green")
            plan_obj->test(2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_rviz");
// note that async is a must
// https://github.com/ros-planning/moveit/issues/1187
// and when using launch file, need to remap /joint_state to /robot/joint-state
// https://answers.ros.org/question/324123/cant-get-robot-arms-current-pose-with-a-error-failed-to-fetch-current-robot-state/
// also, the reason for using 3 thread is each one for
// moveit, this callback and robot

    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(3);
    spinner.start();

        //define const
    plan_obj = new my_planning::MyPlanningClass;
    plan_obj->goToJointZero();
    ROS_INFO("had ran to pose zero");
    plan_obj->addObjects();

    target_pose_random.orientation.w = 1.0;
    target_pose_random.orientation.x = 0.0;
    target_pose_random.orientation.y = 0.0;
    target_pose_random.orientation.z = 0.0;
    target_pose_random.position.x = -0.4;
    target_pose_random.position.y = 0.6;
    target_pose_random.position.z = 0.2;

    ros::Subscriber sub = node_handle.subscribe("gcp_chatbot", 1000, chatbotCallback);
    ros::waitForShutdown();
    // ros::spin();
}