// #include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>


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
    public:
        std::string PLANNING_GROUP_ARM = "chessaton_arm";
        std::string PLANNING_GROUP_HAND = "chessaton_hand";
        ActionHandler(rclcpp::Node::SharedPtr);
        bool move_to(Position);
};

}