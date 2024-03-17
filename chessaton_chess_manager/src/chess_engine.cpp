#include <iostream>
#include <filesystem>
#include <memory>
#include <string>
#include <unistd.h>

#include <boost/process.hpp>
#include <boost/process/io.hpp>
#include <boost/process/pipe.hpp>
#include <boost/algorithm/string/join.hpp>

#include <rclcpp/rclcpp.hpp>

#include "chessaton_interfaces/srv/get_best_move.hpp"
#include "chessaton_interfaces/srv/set_state.hpp"


using namespace boost::process;
namespace fs = std::filesystem;

std::shared_ptr<rclcpp::Node> node;
ipstream out_pipe;
opstream in_pipe;

bool set_state(const std::shared_ptr<chessaton_interfaces::srv::SetState::Request> request,
                    std::shared_ptr<chessaton_interfaces::srv::SetState::Response>   response) {
    if(request->active) {
        std::string line;
        in_pipe << "isready" << std::endl;
        std::getline(out_pipe, line);
        std::cout << line << std::endl;
    } else {
        in_pipe << "quit" << std::endl;
        node.reset();
        // shutdown ROS
        rclcpp::shutdown();
    }
}

bool get_best_move(const std::shared_ptr<chessaton_interfaces::srv::GetBestMove::Request> request,
                    std::shared_ptr<chessaton_interfaces::srv::GetBestMove::Response>   response) {
    std::string line;
    std::string key = "bestmove ";
    bool end = false;

    in_pipe << "position fen " + request->fen + " moves " +boost::algorithm::join(request->moves, " ") << std::endl;  
    in_pipe << "go depth " + std::to_string(request->depth) << std::endl;
    while (std::getline(out_pipe, line)) {

        std::cout << line << std::endl;
        if (!line.find("mate 1")) {
            end = true;
        }
        if (!line.find(key)) {
            if (end)
                response->best_move = line.substr(line.find(key)+key.length()) + " end";
            else
                response->best_move = line.substr(line.find(key)+key.length());
            return true;
        }
    }
    return false;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>(
       "get_best_move_server",
       rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    rclcpp::Service<chessaton_interfaces::srv::SetState>::SharedPtr node_service = 
        node->create_service<chessaton_interfaces::srv::SetState>("set_state", &set_state);

    rclcpp::Service<chessaton_interfaces::srv::GetBestMove>::SharedPtr move_service = 
        node->create_service<chessaton_interfaces::srv::GetBestMove>("get_best_move", &get_best_move);

    std::string filePath = __FILE__;
    std::size_t found = filePath.find_last_of("/");
    std::string dir = filePath.substr(0,found);

    child c(dir+"/engines/kojiro", std_out > out_pipe, std_in < in_pipe);

    rclcpp::spin(node);
}