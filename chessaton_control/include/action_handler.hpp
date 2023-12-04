// #include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// #include <moveit_msgs/msg/AttachedCollisionObject.hpp>
#include <moveit_msgs/msg/collision_object.hpp>


namespace Chessaton {

struct Position {
    double x, y, z;
    Position(double a, double b, double c) {this->x = a; this->y = b; this->z = c;}
};

class ActionHandler {
    private:
        rclcpp::Node::SharedPtr node;
        moveit::planning_interface::MoveGroupInterface* move_group_interface_arm;
        moveit::planning_interface::MoveGroupInterface* move_group_interface_hand;
        moveit::planning_interface::MoveGroupInterface* move_group_interface_arm_hand;
    public:
        std::string PLANNING_GROUP_ARM = "chessaton_arm";
        std::string PLANNING_GROUP_HAND = "chessaton_hand";
        std::string PLANNING_GROUP_ARM_HAND = "chessaton_arm_hand";

        ActionHandler(rclcpp::Node::SharedPtr);
        void visualize_path(moveit::planning_interface::MoveGroupInterface::Plan,  geometry_msgs::msg::Point);
        void visualize_path(moveit::planning_interface::MoveGroupInterface::Plan,  geometry_msgs::msg::Pose);
        bool home();
        bool move_to(geometry_msgs::msg::Point);
        bool move_to(geometry_msgs::msg::Pose);
        bool move_gripper(double);
        bool move_gripper(std::string);
        bool add_collision_obj(shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose, std::string);
        void remove_obj(std::string);
        void allow_collision(std::string);
        void attach_obj(std::string);
};

extern rclcpp::Logger logger;  // ROS logger
}