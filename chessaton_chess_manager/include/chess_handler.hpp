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
        /**
         * @brief robot manipulator class to control the robot movement
         * 
         */
        const ActionHandler& robotMP;
        /**
         * @brief FEN representation of standard start position of chess.
         * 
         */
        std::string startFen;
        /**
         * @brief used to store the moves history (needed for UCI protocol)
         * 
         */
        std::vector<std::string> moves;

        /**
         * @brief position & orientation to grasp the pieces
         * 
         */
        geometry_msgs::msg::Pose grasp_pose;
        /**
         * @brief position & orientation to get closer to grasp pose
         * 
         */
        geometry_msgs::msg::Pose approach_grasp_pose;
        /**
         * @brief position & orientation to lift pieces from chessboard
         * 
         */
        geometry_msgs::msg::Pose lift_pose;
        /**
         * @brief position & orientation to above the center of the board to avoid collision
         * 
         */
        geometry_msgs::msg::Pose center_pose;

        /**
         * @brief position & orientation to get closer to goal
         * 
         */
        geometry_msgs::msg::Pose approach_goal_pose;
        /**
         * @brief position & orientation to put the piece in appointed square
         * 
         */
        geometry_msgs::msg::Pose goal_pose;
        /**
         * @brief position & orientation to retreat from goal square 
         * 
         */
        geometry_msgs::msg::Pose retreat_pose;

        // rotations of gripper
        /// TODO: convert it to a function.
        geometry_msgs::msg::Quaternion gripperQ90;
        geometry_msgs::msg::Quaternion gripperQ70;
        geometry_msgs::msg::Quaternion gripperQ60;
        geometry_msgs::msg::Quaternion gripperQ45;
        geometry_msgs::msg::Quaternion gripperQ0;

        /**
         * @brief chessboard representation with chars
         * 
         */
        std::array<std::string, 64> board;
        /**
         * @brief available dump square for remove pieces
         * 
         */
        int rSq;

        /**
         * @brief convert string output from engine to readable data
         * 
         * @return moveInfo 
         */
        moveInfo parse_move(std::string);
        /**
         * @brief updating board info 
         * 
         */
         /// TODO: should be replaced with chess vision!!! :(
        void update_board(moveInfo);

    protected:
    public:
        /**
         * @brief Construct a new Chess Handler object
         * 
         */
        ChessHandler(const ActionHandler&, std::string);
        /**
         * @brief Get the engine bestmove
         * 
         * @return std::string 
         */
        std::string get_engine_move(rclcpp::Node::SharedPtr, int);
        /**
         * @brief check contact between piece and gripper
         * 
         */
        /// TODO: should be implemented
        void check_contacts();
        /**
         * @brief pick piece from given square
         * 
         */
        void pick_piece(int, std::string);
        /**
         * @brief put piece in appointed square
         * 
         */
        void put_piece(int, std::string);
        /**
         * @brief remove piece from given square
         * 
         */
        void remove_piece(int, std::string);
        /**
         * @brief remove the piece from given square and promote it
         * 
         */
        /// TODO: should be implemented
        void promote_piece(int, std::string, std::string, std::string);
        /**
         * @brief do the given chess move
         * 
         */
        void make_move(std::string);
};

}
#endif