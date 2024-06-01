#include "chess_handler.hpp"
#include "position.hpp"
#include "chessaton_interfaces/srv/get_best_move.hpp"
#include <memory>
#include <unistd.h>


namespace Chessaton {

ChessHandler::ChessHandler(const ActionHandler& newRobotMP, std::string fen) : robotMP(newRobotMP) {
    startFen = fen;

    gripperQ90.x = 0;
    gripperQ90.y = 0.707107;
    gripperQ90.z = 0;
    gripperQ90.w = 0.707107;

    gripperQ70.x = 0;
    gripperQ70.y = 0.573576436;
    gripperQ70.z = 0;
    gripperQ70.w = 0.819152044;
    
    gripperQ45.x = 0;
    gripperQ45.y = 0.382683432;
    gripperQ45.z = 0;
    gripperQ45.w = 0.923879533;

    gripperQ60.x = 0;
    gripperQ60.y = 0.5;
    gripperQ60.z = 0;
    gripperQ60.w = 0.866025404;

    gripperQ0.x = 0;
    gripperQ0.y = 0;
    gripperQ0.z = 0;
    gripperQ0.w = 1;

    board = {"rw", "nw", "bw", "qw", "kw", "bw", "nw", "rw",
             "pw", "pw", "pw", "pw", "pw", "pw", "pw", "pw",
             "-" , "-" , "-" , "-" , "-" , "-" , "-" , "-" , 
             "-" , "-" , "-" , "-" , "-" , "-" , "-" , "-" ,  
             "-" , "-" , "-" , "-" , "-" , "-" , "-" , "-" , 
             "-" , "-" , "-" , "-" , "-" , "-" , "-" , "-" ,  
             "pb", "pb", "pb", "pb", "pb", "pb", "pb", "pb",
             "rb", "nb", "bb", "qb", "kb", "bb", "nb", "rb",
             };
    rSq = r1;
}

std::string ChessHandler::get_engine_move(rclcpp::Node::SharedPtr node, int depth) {
    rclcpp::Client<chessaton_interfaces::srv::GetBestMove>::SharedPtr client =
      node->create_client<chessaton_interfaces::srv::GetBestMove>("get_best_move");

    auto request = std::make_shared<chessaton_interfaces::srv::GetBestMove::Request>();
    request->fen = startFen;
    request->moves = moves;
    request->depth = depth;

    // wait to recieve the bestmove from chess engine
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    std::string best_move = result.get()->best_move;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "best move: %s", best_move.c_str());
    return best_move;
}

moveInfo ChessHandler::parse_move(std::string move) {
    moveInfo mInfo;
    mInfo.source = StrToIdx[move.substr(0, 2)];
    mInfo.target = StrToIdx[move.substr(2, 2)];
    // check for capture
    if (board[mInfo.target] != "-") {
        mInfo.capture = true;
    }
    // check for promotion
    if (move.length() == 5) {
        mInfo.promotion = move.substr(4, 1);
    }
    // check for enpassant
    if (board[mInfo.source] == "pw" && board[mInfo.target] == "-"
        && mInfo.target - mInfo.source != 8 
        && mInfo.target - mInfo.source != 16) {
        if (mInfo.target - mInfo.source == 9 ) {
            mInfo.enpassant = true;
            mInfo.en_sqr = mInfo.source+1;
            mInfo.capture = true;
        }
        else if (mInfo.target - mInfo.source == 7 ) {
            mInfo.enpassant = true;
            mInfo.en_sqr = mInfo.source-1;
            mInfo.capture = true;
        }
    }
    if (board[mInfo.source] == "pb" && board[mInfo.target] == "-"
        && mInfo.source - mInfo.target != 8 
        && mInfo.source - mInfo.target != 16) {
        if (mInfo.source - mInfo.target == 9 ) {
            mInfo.enpassant = true;
            mInfo.en_sqr = mInfo.source-1;
            mInfo.capture = true;
        }
        else if (mInfo.source - mInfo.target == 7 ) {
            mInfo.enpassant = true;
            mInfo.en_sqr = mInfo.source+1;
            mInfo.capture = true;
        }
    }
    // check for castling
    if (board[mInfo.source] == "kw" && (move == "e1c1" || move == "e1g1")) {
        mInfo.castling = true; 
    }
    else if (board[mInfo.source] == "kb" && (move == "e8c8" || move == "e8g8")) {
        mInfo.castling = true;
    }

    return mInfo;
}

void ChessHandler::update_board(moveInfo mInfo) {
    // check castling
    if (mInfo.castling) {
        if (mInfo.source == e1) {
            if (mInfo.target == c1) {
                board[d1] = "rw";
                board[a1] = "-";
            } else if (mInfo.target == g1) {
                board[f1] = "rw";
                board[h1] = "-";
            }
        }
        else if (mInfo.source == e8) {
            if (mInfo.target == c8) {
                board[d8] = "rb";
                board[a8] = "-";
            } else if (mInfo.target == g8) {
                board[f8] = "rb";
                board[h8] = "-";
            }
        }
    } else {
        // check captures
        if (mInfo.capture) {
            // check enpassant
            if (mInfo.enpassant) {
                board[mInfo.en_sqr] = "-";
            } else {
                board[mInfo.target] = "-";
            }
        }
    }
    // move piece
    board[mInfo.target] = board[mInfo.source];
    board[mInfo.source] = "-";
    // check promotion
    if (mInfo.promotion != "-") {
        board[mInfo.target][0] = mInfo.promotion[0];
    }
}

void ChessHandler::check_contacts() {
    // not implemented
}

void ChessHandler::pick_piece(int sq, std::string pType) {
    /// we take a sharper rotation when we are in the first 4 row.
    if (sq > h4) {
        grasp_pose.orientation = gripperQ60;
        approach_grasp_pose.orientation = gripperQ60;
        lift_pose.orientation = gripperQ60;
    } else {
        grasp_pose.orientation = gripperQ70;
        approach_grasp_pose.orientation = gripperQ70;
        lift_pose.orientation = gripperQ70;
    }

    std::tie(grasp_pose.position.x, grasp_pose.position.y) = SqToPos[sq];
    grasp_pose.position.z = BoardZ + graspZ[pType];

    approach_grasp_pose = grasp_pose;
    approach_grasp_pose.position.z = BoardZ + approachZ;

    lift_pose  = grasp_pose;
    lift_pose.position.z = BoardZ + liftZ;

    center_pose.orientation = gripperQ0;
    center_pose.position.x = centerX;
    center_pose.position.z = centerZ;

    // open gripper
    robotMP.move_gripper(graspOpen);
    sleep(0.5);

    // approach to chess piece 
    robotMP.move_to(approach_grasp_pose);
    sleep(0.5);

    // move tcp to chess piece
    robotMP.move_to(grasp_pose);
    sleep(0.5);

    // grasp the chess piece
    robotMP.move_gripper(graspWidth[pType]);
    sleep(0.5);

    // lift the chess piece 
    robotMP.move_to(lift_pose);
    sleep(0.5);

    // move above the center of board
    robotMP.move_to(center_pose);
    sleep(0.5);
}

void ChessHandler::put_piece(int sq, std::string pType) {
    /// we take a sharper rotation when we are in the first 4 row.
    if (sq > h4) {
        goal_pose.orientation = gripperQ60;
        approach_goal_pose.orientation = gripperQ60;
        retreat_pose.orientation = gripperQ60;
    } else {
        goal_pose.orientation = gripperQ70;
        approach_goal_pose.orientation = gripperQ70;
        retreat_pose.orientation = gripperQ70;
    }

    std::tie(goal_pose.position.x, goal_pose.position.y) = SqToPos[sq];
    goal_pose.position.z = BoardZ + graspZ[pType];

    approach_goal_pose = goal_pose;
    /// last constant is a offset to not push into the chessboard
    /// if some physical inaccuracies happen.
    approach_goal_pose.position.z = BoardZ + approachZ + 0.002;

    retreat_pose = goal_pose;
    retreat_pose.position.z = BoardZ + retreatZ;

    center_pose.orientation = gripperQ0;
    center_pose.position.x = centerX;
    center_pose.position.z = centerZ;

    // go to goal position  
    robotMP.move_to(approach_goal_pose);
    sleep(0.5);
    robotMP.move_to(goal_pose);
    sleep(0.5);
   
    // open the gripper to release the chess piece
    robotMP.move_gripper(graspOpen);
    sleep(0.5);

    // retreat from goal pose
    robotMP.move_to(retreat_pose);
    sleep(0.5);

    // move above the center of board
    robotMP.move_to(center_pose);
    sleep(0.5);
}

void ChessHandler::remove_piece(int sq, std::string pType) {
    pick_piece(sq, pType);
    put_piece(rSq, pType);
    rSq++;
}

void ChessHandler::promote_piece(int sq, std::string pType, std::string side, std::string pro) {
    remove_piece(sq, pType);
    if (side == "w")
        pick_piece(r17, pro);
    else
        pick_piece(l17, pro);
    put_piece(sq, pro);
}

void ChessHandler::make_move(std::string move) {
    moveInfo mInfo = parse_move(move);
    std::string ps = board[mInfo.source].substr(0, 1);
    std::string pt = board[mInfo.target].substr(0, 1);
    std::string side = board[mInfo.source].substr(1, 1);

    // check castling
    if (mInfo.castling) {
        if (mInfo.source == e1) {
            if (mInfo.target == c1) {
                pick_piece(a1, "r");
                put_piece(d1, "r");
            } else if (mInfo.target == g1) {
                pick_piece(h1, "r");
                put_piece(f1, "r");
            }
        }
        else if (mInfo.source == e8) {
            if (mInfo.target == c8) {
                pick_piece(a8, "r");
                put_piece(d8, "r");
            } else if (mInfo.target == g8) {
                pick_piece(h8, "r");
                put_piece(f8, "r");
            }
        }
    } else {
        // check captures
        if (mInfo.capture) {
            // check enpassant
            if (mInfo.enpassant) {
                remove_piece(mInfo.en_sqr, pt);
            } else {
                remove_piece(mInfo.target, pt);
            }
        }
    }
    // move piece
    pick_piece(mInfo.source, ps);
    put_piece(mInfo.target, ps);
    // check promotion
    if (mInfo.promotion != "-") {
        // TODO: currently only one queen promotion is available.
        promote_piece(mInfo.source, ps, side, "q");
    }

    moves.push_back(move);
    update_board(mInfo);
}

}