#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include "chess_handler.hpp"
#include "position.hpp"
#include "chessaton_interfaces/srv/set_state.hpp"


int main(int argc, char * argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "chess_robot",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // we use sim_time for simulation in gazebo
    rclcpp::Parameter sim_time_param("use_sim_time", true);
    node->set_parameter(sim_time_param);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    Chessaton::ActionHandler robotMP(node);

    rclcpp::Client<chessaton_interfaces::srv::SetState>::SharedPtr client =
      node->create_client<chessaton_interfaces::srv::SetState>("set_state");

    auto request = std::make_shared<chessaton_interfaces::srv::SetState::Request>();
    request->active = true;

    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    Chessaton::ChessHandler robotCH(robotMP, START_FEN);
    bool game = true;

    while (game) {
        std::string best_move = robotCH.get_engine_move(node, 11);
        if (!best_move.find("end")) {
            game = false;
            best_move = best_move.substr(0, best_move.find(" end"));
        }
        robotCH.make_move(best_move);
    }

    request->active = false;
    result = client->async_send_request(request);

    // shutdown ROS
    rclcpp::shutdown();
    return 0;
}