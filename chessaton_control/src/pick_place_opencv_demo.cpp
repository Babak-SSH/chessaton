#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "../include/action_handler.hpp"

#include "chessaton_interfaces/srv/box_positions.hpp"


template <class ServiceT>
class ServiceNodeSync
{
    typedef typename ServiceT::Request RequestT;
    typedef typename ServiceT::Response ResponseT;
public:
    ServiceNodeSync(std::string name): node(std::make_shared<rclcpp::Node>(name))
    {    }

    void init(std::string service) {
        client = node->create_client<ServiceT>(service);
        std::cout << "created?" << std::endl;
        client->wait_for_service();
        std::cout << "yup created" << std::endl;
    }

    ResponseT sendRequest(const RequestT &req) {
        return sendRequest(std::make_shared<RequestT>(req));
    }

    ResponseT sendRequest(const std::shared_ptr<RequestT> &req_ptr) {
        auto future_result = client->async_send_request(req_ptr);
        if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request failed.");
        }
        auto result = *future_result.get();

        return result;
    }

protected:
    rclcpp::Node::SharedPtr node;
    typename rclcpp::Client<ServiceT>::SharedPtr client;
};

int main(int argc, char * argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "pick_place_opencv_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // we use sim_time for simulation in gazebo
    rclcpp::Parameter sim_time_param("use_sim_time", true);
    node->set_parameter(sim_time_param);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread([&executor]() { executor.spin(); }).detach();

    Chessaton::ActionHandler robot_mp(node);

    ServiceNodeSync<chessaton_interfaces::srv::BoxPositions> service_node_sync("position_client");
    service_node_sync.init("box_positions");

    auto request = std::make_shared<chessaton_interfaces::srv::BoxPositions::Request>();

    auto response = service_node_sync.sendRequest(request);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "box position x: %f", response.box_position.x);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "box position y: %f", response.box_position.y);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "box position z: %f", response.box_position.z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "target position x: %f", response.target_position.x);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "target position y: %f", response.target_position.y);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "target position z: %f", response.target_position.z);

    geometry_msgs::msg::Pose grasp_pose;
    grasp_pose.orientation.w = 1;
    grasp_pose.position.x = (double)((int)(response.box_position.x*100))/100;
    grasp_pose.position.y = (double)((int)(response.box_position.y*100))/100;
    grasp_pose.position.z = (double)((int)(response.box_position.z*100))/100 * 0.8;
    geometry_msgs::msg::Pose lift_pose = grasp_pose;
    lift_pose.position.z += 0.1;
    lift_pose.position.y += 0.05;

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.orientation.w = 1;
    goal_pose.position.x = ((double)((int)(response.target_position.x*100))/100)+0.01;
    goal_pose.position.y = ((double)((int)(response.target_position.y*100))/100)-0.05; // going back one quarter(0.2/4) from the middle of the box
    goal_pose.position.z = ((double)((int)(response.target_position.z*100))/100)+grasp_pose.position.z;// 0.20
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "position x: %f", goal_pose.position.x);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "position y: %f", goal_pose.position.y);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "position z: %f", goal_pose.position.z);
    geometry_msgs::msg::Pose approach_goal_pose = goal_pose;
    approach_goal_pose.position.z += 0.05;
    geometry_msgs::msg::Pose retreat_pose = goal_pose;
    retreat_pose.position.y -= 0.08;

    // open gripper
    robot_mp.move_gripper("hand_open");

    // move tcp to box
    robot_mp.move_to(grasp_pose);

    // grasp the box
    robot_mp.move_gripper(0.014);

    // lift the box
    robot_mp.move_to(lift_pose);

    // go to goal position
    robot_mp.move_to(approach_goal_pose);
    robot_mp.move_to(goal_pose);

    // open the gripper to release the box
    robot_mp.move_gripper("hand_open");
    robot_mp.move_to(retreat_pose);

    // got to home position
    robot_mp.home();

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}