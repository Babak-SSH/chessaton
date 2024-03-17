#ifndef CHESS_HANDLER_HPP
#define CHESS_HANDLER_HPP

#include <action_handler.hpp>
#include <array>

namespace Chessaton {

struct moveInfo {
    int source;
    int target;
    std::string promotion = "-";
    bool capture = false;
    bool enpassant = false;
    int en_sqr;
    bool castling = false;
};

class ChessHandler {
    private:
        const ActionHandler& robotMP;
        std::string startFen;
        std::vector<std::string> moves;

        geometry_msgs::msg::Pose grasp_pose;
        geometry_msgs::msg::Pose approach_grasp_pose;
        geometry_msgs::msg::Pose lift_pose;

        geometry_msgs::msg::Pose goal_pose;
        geometry_msgs::msg::Pose approach_goal_pose;
        geometry_msgs::msg::Pose retreat_pose;

        geometry_msgs::msg::Quaternion gripperQ;

        std::array<std::string, 64> board;
        int rSq;

        moveInfo parse_move(std::string);
        void update_board(moveInfo);

    protected:
    public:
        ChessHandler(const ActionHandler&, std::string);

        std::string get_engine_move(rclcpp::Node::SharedPtr, int);
        void check_contacts();
        void pick_piece(int, std::string);
        void put_piece(int, std::string);
        void remove_piece(int, std::string);
        void promote_piece(int, std::string, std::string, std::string);
        void make_move(std::string);

        // float graspZ = 0.045;
        float approachZ;
        float liftZ;
        float goalZ;
        float retreatZ;
        float graspWidth;
        float quaternionX;
        float quaternionY;
        float quaternionZ;
        float quaternionW;
};

}
#endif