#include "../include/action_handler.hpp"

#include <iostream>


namespace Chessaton {

    namespace rvt = rviz_visual_tools;
    rclcpp::Logger logger = rclcpp::get_logger("ActionHandler");

    ActionHandler::ActionHandler(rclcpp::Node::SharedPtr newNode) {
        node = newNode;
        move_group_interface_arm = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP_ARM);
        move_group_interface_hand = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP_HAND);
        move_group_interface_arm_hand = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP_ARM_HAND);
    }

    void ActionHandler::visualize_path(moveit::planning_interface::MoveGroupInterface::Plan plan, geometry_msgs::msg::Point pos) {
        moveit_visual_tools::MoveItVisualTools visual_tools(node, "chessaton_link0", "move_group_visual",
                                                            move_group_interface_arm->getRobotModel());

        visual_tools.deleteAllMarkers();

        const moveit::core::JointModelGroup* joint_model_group = move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().x() = pos.x+0.01;
        text_pose.translation().y() = pos.y+0.01;
        text_pose.translation().z() = pos.z+0.02;

        RCLCPP_INFO(logger, "goal pos: %f %f %f", pos.x, pos.y, pos.z);

        // Visualizing plans
        visual_tools.publishSphere(pos, rvt::ORANGE);
        // visual_tools.publishAxisLabeled(pos, "pos1");
        visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::MEDIUM);
        visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group, rvt::PURPLE);
        visual_tools.trigger();
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(1.0);
    }

    void ActionHandler::visualize_path(moveit::planning_interface::MoveGroupInterface::Plan plan, geometry_msgs::msg::Pose pose) {
        moveit_visual_tools::MoveItVisualTools visual_tools(node, "chessaton_link0", "move_group_visual",
                                                            move_group_interface_arm->getRobotModel());

        visual_tools.deleteAllMarkers();

        const moveit::core::JointModelGroup* joint_model_group = move_group_interface_arm->getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().x() = pose.position.x+0.01;
        text_pose.translation().y() = pose.position.y+0.01;
        text_pose.translation().z() = pose.position.z+0.02;

        RCLCPP_INFO(logger, "goal pos: %f %f %f orientation: %f %f %f %f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

        // Visualizing plans
        visual_tools.publishSphere(pose, rvt::ORANGE);
        visual_tools.publishAxisLabeled(pose, "pose1");
        visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::MEDIUM);
        visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group, rvt::PURPLE);
        visual_tools.trigger();
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(1.0);
    }

    bool ActionHandler::home() {
         // Plan the move
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_interface_arm_hand->setPlanningTime(60);
        move_group_interface_arm_hand->setJointValueTarget(move_group_interface_arm_hand->getNamedTargetValues("home"));
        auto const success = static_cast<bool>(move_group_interface_arm_hand->plan(my_plan));

        // Execute the plan
        if(success) {
            move_group_interface_arm_hand->execute(my_plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        return success;       
    }

    bool ActionHandler::move_to(geometry_msgs::msg::Point pos) {
        // Basic Information
        RCLCPP_INFO(logger, "Arm Planning frame: %s", move_group_interface_arm->getPlanningFrame().c_str());
        RCLCPP_INFO(logger, "End effector link: %s", move_group_interface_arm->getEndEffectorLink().c_str());
        geometry_msgs::msg::PoseStamped current_pose = move_group_interface_arm->getCurrentPose(); 
        RCLCPP_INFO(logger, "Initial pos: %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

        // Plan the move
        move_group_interface_arm->setPlanningTime(60);
        move_group_interface_arm->setPositionTarget(pos.x, pos.y, pos.z);
        move_group_interface_arm->setGoalTolerance(0.002);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto const success = static_cast<bool>(move_group_interface_arm->plan(my_plan));

        ActionHandler::visualize_path(my_plan, pos);

        // Execute the plan
        if(success) {
            move_group_interface_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        return success;
    }

    bool ActionHandler::move_to(geometry_msgs::msg::Pose pose) {
        // Basic Information
        RCLCPP_INFO(logger, "Arm Planning frame: %s", move_group_interface_arm->getPlanningFrame().c_str());
        RCLCPP_INFO(logger, "End effector link: %s", move_group_interface_arm->getEndEffectorLink().c_str());
        geometry_msgs::msg::PoseStamped current_pose = move_group_interface_arm->getCurrentPose(); 
        RCLCPP_INFO(logger, "Initial pos: %f %f %f orientation: %f %f %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);

        // Plan the move
        move_group_interface_arm->setPlanningTime(60);
        move_group_interface_arm->setPoseTarget(pose);
        move_group_interface_arm->setGoalTolerance(0.002);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        auto const success = static_cast<bool>(move_group_interface_arm->plan(my_plan));

        ActionHandler::visualize_path(my_plan, pose);

        // Execute the plan
        if(success) {
            move_group_interface_arm->execute(my_plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        return success;
    }

    bool ActionHandler::move_gripper(double goal) {
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface_hand->getCurrentState()->getJointModelGroup(PLANNING_GROUP_HAND);
        std::vector<double> joint_group_positions;
        moveit::core::RobotStatePtr current_state = move_group_interface_hand->getCurrentState(10);

        // Plan the move
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_interface_hand->setPlanningTime(60);
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        // Opposite directions
        joint_group_positions[0] = goal;
        joint_group_positions[1] = -goal;
        move_group_interface_hand->setJointValueTarget(joint_group_positions);

        auto const success = static_cast<bool>(move_group_interface_hand->plan(my_plan));

        // Execute the plan
        if(success) {
            move_group_interface_hand->execute(my_plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        return success;
    }

    bool ActionHandler::move_gripper(std::string state) {
        // Plan the move
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_interface_hand->setPlanningTime(60);
        move_group_interface_hand->setJointValueTarget(move_group_interface_hand->getNamedTargetValues(state));
        auto const success = static_cast<bool>(move_group_interface_hand->plan(my_plan));

        // Execute the plan
        if(success) {
            move_group_interface_hand->execute(my_plan);
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }

        return success;
    }

    bool ActionHandler::add_collision_obj(shape_msgs::msg::SolidPrimitive primitive, geometry_msgs::msg::Pose object_pose, std::string object_id) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Collision object
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface_arm->getPlanningFrame();
        collision_object.id = object_id;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(object_pose);
        collision_object.operation = collision_object.ADD;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);

        // Add object
        auto const success = planning_scene_interface.applyCollisionObjects(collision_objects);

        if (!success)
            RCLCPP_ERROR(logger, "Adding collision object failed!");

        return success;
    }

    void ActionHandler::remove_obj(std::string object_id) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::CollisionObject collision_object;
        std::vector<std::string> object_ids;
        moveit_msgs::msg::AttachedCollisionObject aco;

        aco.object.id = object_id;
        aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(aco);

        collision_object.id = object_id;
        collision_object.operation = collision_object.REMOVE; 
        object_ids.push_back(collision_object.id);
        planning_scene_interface.removeCollisionObjects(object_ids); 
    }

    void ActionHandler::allow_collision(std::string object_id, bool allow) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(node, "robot_description"));

        planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
        collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
        acm.setEntry(object_id, "left_finger", allow);
        acm.setEntry(object_id, "right_finger", allow);
        acm.setEntry(object_id, "chessaton_gripper", allow);
        // moveit_msgs::msg::PlanningScene diff_scene;
        // ls->getPlanningSceneDiffMsg(diff_scene);
    
        // planning_scene_interface.applyPlanningScene(diff_scene); 
    }

    void ActionHandler::attach_obj(std::string object_id) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::AttachedCollisionObject aco;

        aco.object.id = object_id;
        aco.link_name = "right_finger";
        aco.touch_links.push_back("left_finger");
        aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        planning_scene_interface.applyAttachedCollisionObject(aco);
    }

    void ActionHandler::detach_obj(std::string object_id) {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::AttachedCollisionObject aco;
    
        aco.object.id = object_id;
        aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        planning_scene_interface.applyAttachedCollisionObject(aco);
    }
} // namespace chessaton