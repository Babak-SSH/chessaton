#include "../include/action_handler.hpp"


int main(int argc, char * argv[]) {
   // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "move_group_demo",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    // we use sim_time for simulation in gazebo
    rclcpp::Parameter sim_time_param("use_sim_time", true);
    node->set_parameter(sim_time_param);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    Chessaton::ActionHandler robot_mp(node);

    // get the position to move to
    double x,y,z;

    std::cout << "enter position!" << std::endl;    
    std::cout << "x: ";
    std::cin >> x;
    std::cout << "y: ";
    std::cin >> y;
    std::cout << "z: ";
    std::cin >> z;

    Chessaton::Position position(x, y, z);

    robot_mp.move_to(position);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}