#include <planning.h>
#include <stdlib.h>

// http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html
// code: https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
//define function
namespace my_planning
{
    // moves its arm to the pose goal, 
    // inverse kinematic, given end effector pose, output plan for joint
        void MyPlanningClass::goToPoseGoal(geometry_msgs::Pose &pose)
        {

            move_group.setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

            ROS_INFO("start planning");

            bool success = (move_group.plan(pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if(!success) // get error
            {
                ROS_INFO("unfortunately");
                // throw std::runtime_error("No plan found for pose");
            }
            else
                move_group.move(); //execute
        }

    // moves its arm to the joint goal, which diff from current joint in radians
    // forward kinematic, given joint value, output plan 
        void MyPlanningClass::goToJointState(std::vector<double> &diff_joint)
        {
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            std::vector<double> joint_goal_positions;
            joint_model_group = current_state -> getJointModelGroup(PLANNING_GROUP);
            current_state -> copyJointGroupPositions(joint_model_group, joint_goal_positions);
            //joint_positions = move_group.getCurrentJointValues();

            if (diff_joint.size() > joint_goal_positions.size())
            {
                ROS_INFO("error input diff joint");
                throw std::runtime_error("invalid diff joint");
            }
            // modified to a new joint state
            for (std::size_t i = 0; i < std::min(diff_joint.size(), joint_goal_positions.size()); i++)
                joint_goal_positions[i] += diff_joint[i];

            move_group.setJointValueTarget(joint_goal_positions);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

            if(!success)
            {
                // throw std::runtime_error("No plan found for diff joint");
                ROS_INFO("unforntately joint failed");
            }
            move_group.move(); //blocking
        }

        void MyPlanningClass::goToJointZero()
        {
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
            std::vector<double> joint_goal_positions;
            joint_model_group = current_state -> getJointModelGroup(PLANNING_GROUP);
            current_state -> copyJointGroupPositions(joint_model_group, joint_goal_positions);

            
            // modified to a new joint state
            for (std::size_t i = 0; i < joint_goal_positions.size(); i++)
                joint_goal_positions[i] = 0;
            joint_goal_positions[1] = 0;

            move_group.setJointValueTarget(joint_goal_positions);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

            if(!success)
            {
                // throw std::runtime_error("No plan found for diff joint");
                ROS_INFO("unforntately joint failed");
            }
            move_group.move(); //blocking
        }

        // move according to waypath, multiple inverse kinematics
        void MyPlanningClass::cartesianPath(std::vector<double> waypoints)
        {
            ROS_INFO("path_plan");
            geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
            ROS_INFO("current x [%f]", current_pose.position.x);
            ROS_INFO("current y [%f]", current_pose.position.y);
            ROS_INFO("current z [%f]", current_pose.position.z);

            std::vector<geometry_msgs::Pose> path;
            path.push_back(current_pose);
            for (std::size_t i = 0; i < waypoints.size()/2; i++)
            {
                current_pose.position.z += waypoints[i * 2];
                current_pose.position.x += waypoints[i * 2 + 1];
                path.push_back(current_pose);
                ROS_INFO("after_add");
                ROS_INFO("current x [%f]", current_pose.position.x);
                ROS_INFO("current y [%f]", current_pose.position.y);
                ROS_INFO("current z [%f]", current_pose.position.z);
            }

            move_group.setMaxVelocityScalingFactor(0.1);
            // move_group.setMaxVelocityScalingFactor(1);

            // We want the Cartesian path to be interpolated at a resolution of 1 cm
            // which is why we will specify 0.01 as the max step in Cartesian
            // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
            // Warning - disabling the jump threshold while operating real hardware can cause
            // large unpredictable motions of redundant joints and could be a safety issue
            moveit_msgs::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            // should try to change this to view the effect
            const double eef_step = 0.01;
            double fraction = move_group.computeCartesianPath(path, eef_step, jump_threshold, trajectory);
            ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
            my_plan.trajectory_= trajectory;
            move_group.execute(my_plan);
        }

       void MyPlanningClass::safeToPoseGoal(geometry_msgs::Pose pose)
        {
            ROS_INFO("safe_planning");
            geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
            std::size_t step = 4;
            std::size_t fail_counted = 0;

            geometry_msgs::Pose ik_delta;
            ik_delta.position.x = (current_pose.position.x - pose.position.x) / step;
            ik_delta.position.y = (current_pose.position.y - pose.position.y) / step;
            ik_delta.position.z = (current_pose.position.z - pose.position.z) / step;
            ik_delta.orientation.x = (current_pose.orientation.x - pose.orientation.x) / step;
            ik_delta.orientation.y = (current_pose.orientation.y - pose.orientation.y) / step;
            ik_delta.orientation.z = (current_pose.orientation.z - pose.orientation.z) / step;
            ik_delta.orientation.w = (current_pose.orientation.w - pose.orientation.w) / step;
            moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

            for (std::size_t d = 0; d <= step; d++)
            {
                geometry_msgs::Pose ik_step;
                ik_step.position.x = current_pose.position.x  - d*ik_delta.position.x;
                ik_step.position.y = current_pose.position.y  - d*ik_delta.position.y;
                ik_step.position.z = current_pose.position.z  - d*ik_delta.position.z;
                ik_step.orientation.x = current_pose.orientation.x  - d*ik_delta.orientation.x;
                ik_step.orientation.y = current_pose.orientation.y  - d*ik_delta.orientation.y;
                ik_step.orientation.z = current_pose.orientation.z  - d*ik_delta.orientation.z;
                ik_step.orientation.w = current_pose.orientation.w  - d*ik_delta.orientation.w;

                move_group.setPoseTarget(ik_step);

                bool success = (move_group.plan(pose_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if(!success) // get error
                {
                    // ROS_INFO("unfortunately");
                    // throw std::runtime_error("No plan found for pose");
                    fail_counted = fail_counted + 1;
                    continue;
                }
                else
                    move_group.move(); //execute
            }
            ROS_INFO("failed couted %zu ", fail_counted);
        }

        void MyPlanningClass::resetValues()
        {
            //set the start state and operational speed
            move_group.setStartStateToCurrentState();
            move_group.setMaxVelocityScalingFactor(1);
            move_group.clearPoseTargets();
        }

         void MyPlanningClass::makeWall(std::string blk_name, double *pose)
         {
             moveit_msgs::CollisionObject wall;
            //set the relative frame
            wall.header.frame_id = move_group.getPlanningFrame();
            wall.id = blk_name;

            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.913;
            primitive.dimensions[1] = 0.913;
            primitive.dimensions[2] = 0.04+0.74+ 0.04;

            geometry_msgs::Pose wall_pose;
            wall_pose.orientation.w = 1.0;
            wall_pose.position.x = pose[0];
            wall_pose.position.y = pose[1];
            wall_pose.position.z = pose[2];

            wall.primitives.push_back(primitive);
            wall.primitive_poses.push_back(wall_pose);
            wall.operation = wall.ADD;

            std::vector<moveit_msgs::CollisionObject> collisionObjects;
            collisionObjects.push_back(wall);
            ros::Duration(2).sleep();

            virtual_world.addCollisionObjects(collisionObjects);
            ROS_INFO_STREAM("Added: " << blk_name);
         }

        void MyPlanningClass::addObjects()
        {
            double wall_pose1[3] = {0.75, 0, -0.58};
            makeWall("wall_1", wall_pose1);

            double wall_pose2[3] = {-0.2, 0.75, -0.58};
            makeWall("wall_2", wall_pose2);
        }

        void MyPlanningClass::removeObjects()
        {
            std::vector<std::string> object_ids;
            object_ids.push_back("wall_1");
            object_ids.push_back("wall_2");
            virtual_world.removeCollisionObjects(object_ids);
        }

        std::vector<geometry_msgs::Pose> gen_harded_coded_pick_pose()
        {
            geometry_msgs::Pose base_pose;
            base_pose.orientation.w = 0.00253311793936;
            base_pose.orientation.x = -0.00142460053167;
            base_pose.orientation.y = 0.999994209902;
            base_pose.orientation.z = -0.00177030764765;
            base_pose.position.z = 0.02; //hover

            std::vector<geometry_msgs::Pose> harded_coded;
            base_pose.position.x = 0.55; base_pose.position.y = 0.085; harded_coded.push_back(base_pose);
                                         base_pose.position.y = -0.198; harded_coded.push_back(base_pose);
                                         base_pose.position.y = 0.383; harded_coded.push_back(base_pose);

            // base_pose.position.x = 0.75; base_pose.position.y = -0.198; harded_coded.push_back(base_pose);
            //                              base_pose.position.y = 0.085; harded_coded.push_back(base_pose);
            //                              base_pose.position.y = 0.385; harded_coded.push_back(base_pose);
            return harded_coded;
        }

        std::vector<geometry_msgs::Pose> gen_harded_coded_place_pose()
        {
            geometry_msgs::Pose base_pose;
            base_pose.position.z = 0.1; //hover
            base_pose.position.y = 0.6;

            std::vector<geometry_msgs::Pose> harded_coded;

            base_pose.position.x = 0.1;  harded_coded.push_back(base_pose);
            base_pose.position.x = -0.2;  harded_coded.push_back(base_pose);
            base_pose.position.x = -0.5;  harded_coded.push_back(base_pose);
            

            return harded_coded;
        }
//   frame_id: ''
// name: [head_pan, right_gripper_l_finger_joint, right_gripper_r_finger_joint, right_j0, right_j1,
//   right_j2, right_j3, right_j4, right_j5, right_j6]
// position: [0.0011985463948063213, 0.009583517800550398, -0.01995391298784495, -1.4639367189533852, 0.1968848344842069, -1.5583568069487352, -1.89210185689244, 1.3867031039464033, 1.6556511090735704, 2.1597185238769327]
// velocity: [0.005714812089491596, 0.019494461226233963, -0.01154080297342318, 0.004126803210262874, 0.007537111093901002, -0.01903640490731954, 0.003526020768566552, 0.0005360531173291411, -0.0003105960079088019, 0.007151913053910941]
// effort: [0.05597212164874707, -20.0, 20.0, 0.003159156009517796, -24.993487295508505, 12.306412088761478, -2.4903082177596287, 3.5948921810249406, 0.12632046416770534, 0.05259992664946117]

// name: [head_pan, right_gripper_l_finger_joint, right_gripper_r_finger_joint, right_j0, right_j1,
//   right_j2, right_j3, right_j4, right_j5, right_j6]
// position: [0.0011474344651896118, 0.011114145936337982, -0.018281479761374193, 0.17961809816585905, 0.6042424544925522, 0.2966549894182622, -2.022214670661107, -1.0005366641746818, 2.8579980204553586, 0.7801426087143684]
// velocity: [-0.00017642789753573307, -0.05200068976925795, 0.08468345142962282, 0.10716797115557673, 0.045805758652950716, -0.0024731097455207574, 0.012530032874757582, 0.01985363644189024, 0.012675656082891416, 0.023992587365391185]
// effort: [0.01183266101694877, -20.0, 20.0, 3.9874802386604222, -28.69979628598572, -10.397607031982886, -4.939787098972877, 1.3728630495738139, 0.4585859051250798, -0.049855181603395055]

        std::size_t MyPlanningClass::pick(geometry_msgs::Pose cube_pose, double hover_distance)
        {
            // open_the_gripper
            // should change to find package in newer version
            system("python /home/ros/ros_ws/src/planning/scripts/open_gripper.py");
            geometry_msgs::Pose hover_pose = cube_pose; 
            hover_pose.position.z += hover_distance;
            goToPoseGoal(hover_pose);
            // capture the image
            system("rosservice call /image_saver/save");
            ros::Duration(0.5).sleep();
            std::size_t return_color = system("python3.7 /home/ros/ros_ws/src/planning/scripts/get_dominant_color.py");
            ROS_INFO_STREAM("Detected: " << return_color);
            if (return_color == 1024) return 7; // none cube

            // approach the cube
            cube_pose.position.z -= hover_distance;
            move_group.setMaxVelocityScalingFactor(0.7);
            goToPoseGoal(cube_pose);
            move_group.setMaxVelocityScalingFactor(1);
            // safeToPoseGoal(cube_pose);
            system("python /home/ros/ros_ws/src/planning/scripts/close_gripper.py");
            
            goToPoseGoal(hover_pose);
            
            // return color of cube
            if (return_color == 0) return 0; // red
            if (return_color == 256) return 1; // blue
            if (return_color == 512) return 2; // green
        }

        void MyPlanningClass::place(geometry_msgs::Pose tray_pose, double hover_distance)
        {
            std::vector<double> rotate_left{+1.5};
            goToJointState(rotate_left);

            geometry_msgs::Pose hover_pose = tray_pose;
            hover_pose.position.z += hover_distance;
            geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
            hover_pose.orientation = current_pose.orientation;
            goToPoseGoal(hover_pose);
            // drop the cube
            system("python /home/ros/ros_ws/src/planning/scripts/open_gripper.py");
            std::vector<double> rotate_right{-1.5};
            goToJointState(rotate_right);
        }


        void MyPlanningClass::sorting(std::size_t color_code)
        {
            double hover_distance = 0.15;
            std::vector<geometry_msgs::Pose> hard_coded_pick_pose = gen_harded_coded_pick_pose();
            std::vector<geometry_msgs::Pose> hard_coded_place_pose = gen_harded_coded_place_pose();

            for (std::size_t d = 0; d < hard_coded_pick_pose.size(); d++)
            {
                std::size_t color_cube = pick(hard_coded_pick_pose[d], hover_distance);
                if (color_cube == 7) continue; //none cube
                if (color_code == 7 or color_code == color_cube) 
                    place(hard_coded_place_pose[color_cube], hover_distance - 0.07);
            }
        }

        void MyPlanningClass::test(std::size_t id_test)
        {
            double hover_distance = 0.15;
            std::vector<geometry_msgs::Pose> hard_coded_pick_pose = gen_harded_coded_pick_pose();
            std::vector<geometry_msgs::Pose> hard_coded_place_pose = gen_harded_coded_place_pose();

            std::size_t color_cube = pick(hard_coded_pick_pose[id_test], hover_distance);
            place(hard_coded_place_pose[id_test], hover_distance);
        }
}
