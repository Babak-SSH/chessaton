#ifndef CHESS_HANDLER_HPP
#define CHESS_HANDLER_HPP

#include <action_handler.hpp>

namespace Chessaton {

class ChessHandler {
    private:
        const ActionHandler& robotMP;
        std::string startFen;
        std::string currentFen;

        geometry_msgs::msg::Pose grasp_pose;
        geometry_msgs::msg::Pose approach_grasp_pose;
        geometry_msgs::msg::Pose lift_pose;

        geometry_msgs::msg::Pose goal_pose;
        geometry_msgs::msg::Pose approach_goal_pose;
        geometry_msgs::msg::Pose retreat_pose;

        geometry_msgs::msg::Quaternion gripperQ;
    protected:
    public:
        ChessHandler(const ActionHandler&);

        void update_board();
        void get_engine_move();
        void check_contacts();
        void pick_piece(int);
        void put_piece(int);
        void remove_piece(int);
        void promote_piece(int, int);

        float graspZ = 0.045;
        float approachZ = 0.06;
        float liftZ = 0.1;
        float goalZ = 0.045;
        float retreatZ = 0.1;
        float graspWidth = 0.003;
        float quaternionX = 0;
        float quaternionY = 0.707107;
        float quaternionZ = 0;
        float quaternionW = 0.707107;
};

}
#endif