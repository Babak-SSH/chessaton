#include "position.hpp"


// center cord of chessboard
const float BoardX = 0.18;
const float BoardY = 0.0;
const float BoardZ = 0.135;
const float Step = 0.027; // square tile size

// x values
const float Row1 = BoardX + (-3.5*Step);
const float Row2 = BoardX + (-2.5*Step);
const float Row3 = BoardX + (-1.5*Step);
const float Row4 = BoardX + (-0.5*Step);
const float Row5 = BoardX + (0.5*Step);
const float Row6 = BoardX + (1.5*Step);
const float Row7 = BoardX + (2.5*Step);
const float Row8 = BoardX + (3.5*Step);

// y values
const float FileA = BoardY + (3.5*Step);
const float FileB = BoardY + (2.5*Step);
const float FileC = BoardY + (1.5*Step);
const float FileD = BoardY + (0.5*Step);
const float FileE = BoardY + (-0.5*Step);
const float FileF = BoardY + (-1.5*Step);
const float FileG = BoardY + (-2.5*Step);
const float FileH = BoardY + (-3.5*Step);
const float FileR1 = BoardY + (-4.5*Step);
const float FileR2 = BoardY + (-5.5*Step);
const float FileR3 = BoardY + (-6.5*Step);
const float FileL1 = BoardY + (4.5*Step);
const float FileL2 = BoardY + (5.5*Step);
const float FileL3 = BoardY + (6.5*Step);

std::map<int, std::pair<float, float>> SqToPos = {
                                                {a1, {Row1,FileA}}, {b1, {Row1,FileB}}, {c1, {Row1,FileC}}, {d1, {Row1,FileD}}, {e1, {Row1,FileE}}, {f1, {Row1,FileF}}, {g1, {Row1,FileG}}, {h1, {Row1,FileH}}, 
                                                {a2, {Row2,FileA}}, {b2, {Row2,FileB}}, {c2, {Row2,FileC}}, {d2, {Row2,FileD}}, {e2, {Row2,FileE}}, {f2, {Row2,FileF}}, {g2, {Row2,FileG}}, {h2, {Row2,FileH}}, 
                                                {a3, {Row3,FileA}}, {b3, {Row3,FileB}}, {c3, {Row3,FileC}}, {d3, {Row3,FileD}}, {e3, {Row3,FileE}}, {f3, {Row3,FileF}}, {g3, {Row3,FileG}}, {h3, {Row3,FileH}}, 
                                                {a4, {Row4,FileA}}, {b4, {Row4,FileB}}, {c4, {Row4,FileC}}, {d4, {Row4,FileD}}, {e4, {Row4,FileE}}, {f4, {Row4,FileF}}, {g4, {Row4,FileG}}, {h4, {Row4,FileH}}, 
                                                {a5, {Row5,FileA}}, {b5, {Row5,FileB}}, {c5, {Row5,FileC}}, {d5, {Row5,FileD}}, {e5, {Row5,FileE}}, {f5, {Row5,FileF}}, {g5, {Row5,FileG}}, {h5, {Row5,FileH}}, 
                                                {a6, {Row6,FileA}}, {b6, {Row6,FileB}}, {c6, {Row6,FileC}}, {d6, {Row6,FileD}}, {e6, {Row6,FileE}}, {f6, {Row6,FileF}}, {g6, {Row6,FileG}}, {h6, {Row6,FileH}}, 
                                                {a7, {Row7,FileA}}, {b7, {Row7,FileB}}, {c7, {Row7,FileC}}, {d7, {Row7,FileD}}, {e7, {Row7,FileE}}, {f7, {Row7,FileF}}, {g7, {Row7,FileG}}, {h7, {Row7,FileH}}, 
                                                {a8, {Row8,FileA}}, {b8, {Row8,FileB}}, {c8, {Row8,FileC}}, {d8, {Row8,FileD}}, {e8, {Row8,FileE}}, {f8, {Row8,FileF}}, {g8, {Row8,FileG}}, {h8, {Row8,FileH}}, 
                                                {r1, {Row1, FileR1}}, {r2, {Row2, FileR1}}, {r3, {Row3, FileR1}}, {r4, {Row4, FileR1}}, {r5, {Row5, FileR1}}, {r6, {Row6, FileR1}}, {r7, {Row7, FileR1}}, {r8, {Row8, FileR1}},
                                                {r9, {Row1, FileR2}}, {r10, {Row2, FileR2}}, {r11, {Row3, FileR2}}, {r12, {Row4, FileR2}}, {r13, {Row5, FileR2}}, {r14, {Row6, FileR2}}, {r15, {Row7, FileR2}}, {r16, {Row8, FileR2}}, 
                                                {l1, {Row1, FileL1}}, {l2, {Row2, FileL1}}, {l3, {Row3, FileL1}}, {l4, {Row4, FileL1}}, {l5, {Row5, FileL1}}, {l6, {Row6, FileL1}}, {l7, {Row7, FileL1}}, {l8, {Row8, FileL1}},
                                                {l9, {Row1, FileL2}}, {l10, {Row2, FileL2}}, {l11, {Row3, FileL2}}, {l12, {Row4, FileL2}}, {l13, {Row5, FileL2}}, {l14, {Row6, FileL2}}, {l15, {Row7, FileL2}}, {l16, {Row8, FileL2}},
                                                {r17, {Row1, FileR3}}, {l17, {Row1, FileL3}},
                                                };

std::map<std::string, int> StrToIdx = {
                                    {"a1", a1}, {"b1", b1}, {"c1", c1}, {"d1", d1}, {"e1", e1}, {"f1", f1}, {"g1", g1}, {"h1", h1}, 
                                    {"a2", a2}, {"b2", b2}, {"c2", c2}, {"d2", d2}, {"e2", e2}, {"f2", f2}, {"g2", g2}, {"h2", h2}, 
                                    {"a3", a3}, {"b3", b3}, {"c3", c3}, {"d3", d3}, {"e3", e3}, {"f3", f3}, {"g3", g3}, {"h3", h3}, 
                                    {"a4", a4}, {"b4", b4}, {"c4", c4}, {"d4", d4}, {"e4", e4}, {"f4", f4}, {"g4", g4}, {"h4", h4}, 
                                    {"a5", a5}, {"b5", b5}, {"c5", c5}, {"d5", d5}, {"e5", e5}, {"f5", f5}, {"g5", g5}, {"h5", h5}, 
                                    {"a6", a6}, {"b6", b6}, {"c6", c6}, {"d6", d6}, {"e6", e6}, {"f6", f6}, {"g6", g6}, {"h6", h6}, 
                                    {"a7", a7}, {"b7", b7}, {"c7", c7}, {"d7", d7}, {"e7", e7}, {"f7", f7}, {"g7", g7}, {"h7", h7}, 
                                    {"a8", a8}, {"b8", b8}, {"c8", c8}, {"d8", d8}, {"e8", e8}, {"f8", f8}, {"g8", g8}, {"h8", h8} 
                                    };

std::map<std::string, float> graspZ = {
                                    {"k", 0.04},
                                    {"q", 0.04},
                                    {"b", 0.03},
                                    {"n", 0.03},
                                    {"r", 0.03},
                                    {"p", 0.03}
};

std::map<std::string, float> graspWidth = {
                                    {"k", 0.003},
                                    {"q", 0.003},
                                    {"b", 0.003},
                                    {"n", 0.003},
                                    {"r", 0.003},
                                    {"p", 0.003}
};

const std::string START_FEN = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"; 

// robot 
const float approachZ = 0.08;
const float liftZ = 0.08;
const float centerZ = 0.25;
const float centerX = 0.18;
const float retreatZ = 0.08;
const float graspOpen = 0.008;
