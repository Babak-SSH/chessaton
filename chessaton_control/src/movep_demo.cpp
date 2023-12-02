#include "../include/action_handler.hpp"


int main(int argc, char * argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "movep_demo",
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
    double px, py, pz, ox, oy, oz, ow;
    int goal_type;

    std::cout << "- Goal type\n     + Option N1: position\n     + Option N2: pose" << std::endl;
    std::cout << "  Please select: ";
    std::cin >> goal_type;

    if (goal_type == 1) {
        std::cout << "enter position!" << std::endl;    
        std::cout << "x: ";
        std::cin >> px;
        std::cout << "y: ";
        std::cin >> py;
        std::cout << "z: ";
        std::cin >> pz;

        geometry_msgs::msg::Point position;
        position.x = px;
        position.y = py;
        position.z = pz;

        robot_mp.move_to(position);
    }
    else if (goal_type == 2) {
        std::cout << "enter position!" << std::endl;    
        std::cout << "position x: ";
        std::cin >> px;
        std::cout << "position y: ";
        std::cin >> py;
        std::cout << "position z: ";
        std::cin >> pz;
        std::cout << "orientation x: ";
        std::cin >> ox;
        std::cout << "orientation y: ";
        std::cin >> oy;
        std::cout << "orientation z: ";
        std::cin >> oz;
        std::cout << "orientation w: ";
        std::cin >> ow;

        geometry_msgs::msg::Pose pose;
        pose.orientation.x = ox;
        pose.orientation.y = oy;
        pose.orientation.z = oz;
        pose.orientation.w = ow;
        pose.position.x = px;
        pose.position.y = py;
        pose.position.z = pz;

        robot_mp.move_to(pose);
    }
    else {
        std::cout << "  Please select a valid option!" << std::endl;
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}