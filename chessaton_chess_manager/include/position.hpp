#ifndef POSITION_HPP
#define POSITION_HPP

#include <map>
#include <string>


// chessboard and pieces

// center cord of chessboard
extern const float BoardX;
extern const float BoardY;
extern const float BoardZ;
extern const float Step; // square tile size

// x values
extern const float Row1;
extern const float Row2;
extern const float Row3;
extern const float Row4;
extern const float Row5;
extern const float Row6;
extern const float Row7;
extern const float Row8;

// y values
extern const float FileA;
extern const float FileB;
extern const float FileC;
extern const float FileD;
extern const float FileE;
extern const float FileF;
extern const float FileG;
extern const float FileH;

enum sq {
    a1, b1, c1, d1, e1, f1, g1, h1,
    a2, b2, c2, d2, e2, f2, g2, h2,
    a3, b3, c3, d3, e3, f3, g3, h3,
    a4, b4, c4, d4, e4, f4, g4, h4,
    a5, b5, c5, d5, e5, f5, g5, h5,
    a6, b6, c6, d6, e6, f6, g6, h6,
    a7, b7, c7, d7, e7, f7, g7, h7,
    a8, b8, c8, d8, e8, f8, g8, h8,
    // squares for removed pieces
    r1, r2, r3, r4, r5, r6, r7, r8,
    r9, r10,r11,r12,r13,r14,r15,r16,
    l1, l2, l3, l4, l5, l6, l7, l8,
    l9, l10,l11,l12,l13,l14,l15,l16,
    r17, l17,
};

extern std::map<int, std::pair<float, float>> SqToPos;
extern std::map<std::string, int> StrToIdx; 
extern std::map<std::string, float> graspZ;
extern std::map<std::string, float> graspWidth;

extern const std::string START_FEN;

// robot
extern const float approachZ;
extern const float liftZ;
extern const float centerZ;
extern const float centerX; 
extern const float retreatZ;
extern const float graspOpen; 

#endif