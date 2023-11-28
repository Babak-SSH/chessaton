#include "../include/action_handler.hpp"

#include <iostream>


namespace Chessaton {
    ActionHandler::ActionHandler(rclcpp::Node::SharedPtr newNode) {
        node = newNode;
        move_group_interface_arm = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP_ARM);
        move_group_interface_hand = new moveit::planning_interface::MoveGroupInterface(node, PLANNING_GROUP_HAND);
    }

    bool ActionHandler::move_to(Position pose) {
        std::cout << "move_to" << std::endl;
        return true;
    }

} // namespace chessaton