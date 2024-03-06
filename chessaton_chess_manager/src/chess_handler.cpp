// #include <action_handler.hpp>
#include "chess_handler.hpp"
#include "position.hpp"

namespace Chessaton {

ChessHandler::ChessHandler(const ActionHandler& newRobotMP) : robotMP(newRobotMP) {

    graspZ = 0.045;
    approachZ = 0.06;
    liftZ = 0.1;
    goalZ = 0.045;
    retreatZ = 0.1;
    graspWidth = 0.003;
    quaternionX = 0;
    quaternionY = 0.707107;
    quaternionZ = 0;
    quaternionW = 0.707107;
}

void ChessHandler::update_board() {

}

void ChessHandler::get_engine_move() {

}

void ChessHandler::check_contacts() {

}

void ChessHandler::pick_piece(int sq) {
    gripperQ.x = quaternionX;
    gripperQ.y = quaternionY;
    gripperQ.z = quaternionZ;
    gripperQ.w = quaternionW;
    grasp_pose.orientation = gripperQ;
    std::tie(grasp_pose.position.x, grasp_pose.position.y) = SqToPos[sq];
    grasp_pose.position.z = graspZ;

    approach_grasp_pose = grasp_pose;
    approach_grasp_pose.position.z = approachZ;

    lift_pose  = grasp_pose;
    lift_pose.position.z = liftZ;

    // open gripper
    // robotMP.move_gripper("hand_open");
    robotMP.move_gripper(graspWidth+0.003);

    // approach to chess piece 
    robotMP.move_to(approach_grasp_pose);

    // move tcp to chess piece
    robotMP.move_to(grasp_pose);

    // grasp the chess piece
    robotMP.move_gripper(graspWidth);

    // lift the chess piece 
    robotMP.move_to(lift_pose);


}

void ChessHandler::put_piece(int sq) {
    gripperQ.x = quaternionX;
    gripperQ.y = quaternionY;
    gripperQ.z = quaternionZ;
    gripperQ.w = quaternionW;
    goal_pose.orientation = gripperQ;
    std::tie(goal_pose.position.x, goal_pose.position.y) = SqToPos[sq];
    goal_pose.position.z = goalZ;

    approach_goal_pose = goal_pose;
    approach_goal_pose.position.z = approachZ;

    retreat_pose = goal_pose;
    retreat_pose.position.z = retreatZ;

    // go to goal position  
    robotMP.move_to(approach_goal_pose);
    robotMP.move_to(goal_pose);
   
    // open the gripper to release the chess piece
    // robotMP.move_gripper("hand_open");
    robotMP.move_gripper(graspWidth+0.003);

    // retreat from goal pose
    robotMP.move_to(retreat_pose);
}

void ChessHandler::remove_piece(int sq) {

}

void ChessHandler::promote_piece(int sq, int type) {

}

}