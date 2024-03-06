#include "position.hpp"


// center cord of chessboard
const float BoardX = 0.25;
const float BoardY = 0.0;
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


std::map<int, std::pair<float, float>> SqToPos = {
                                                {a1, {Row1,FileA}}, {b1, {Row1,FileB}}, {c1, {Row1,FileC}}, {d1, {Row1,FileD}}, {e1, {Row1,FileE}}, {f1, {Row1,FileF}}, {g1, {Row1,FileG}}, {h1, {Row1,FileH}}, 
                                                {a2, {Row2,FileA}}, {b2, {Row2,FileB}}, {c2, {Row2,FileC}}, {d2, {Row2,FileD}}, {e2, {Row2,FileE}}, {f2, {Row2,FileF}}, {g2, {Row2,FileG}}, {h2, {Row2,FileH}}, 
                                                {a3, {Row3,FileA}}, {b3, {Row3,FileB}}, {c3, {Row3,FileC}}, {d3, {Row3,FileD}}, {e3, {Row3,FileE}}, {f3, {Row3,FileF}}, {g3, {Row3,FileG}}, {h3, {Row3,FileH}}, 
                                                {a4, {Row4,FileA}}, {b4, {Row4,FileB}}, {c4, {Row4,FileC}}, {d4, {Row4,FileD}}, {e4, {Row4,FileE}}, {f4, {Row4,FileF}}, {g4, {Row4,FileG}}, {h4, {Row4,FileH}}, 
                                                {a5, {Row5,FileA}}, {b5, {Row5,FileB}}, {c5, {Row5,FileC}}, {d5, {Row5,FileD}}, {e5, {Row5,FileE}}, {f5, {Row5,FileF}}, {g5, {Row5,FileG}}, {h5, {Row5,FileH}}, 
                                                {a6, {Row6,FileA}}, {b6, {Row6,FileB}}, {c6, {Row6,FileC}}, {d6, {Row6,FileD}}, {e6, {Row6,FileE}}, {f6, {Row6,FileF}}, {g6, {Row6,FileG}}, {h6, {Row6,FileH}}, 
                                                {a7, {Row7,FileA}}, {b7, {Row7,FileB}}, {c7, {Row7,FileC}}, {d7, {Row7,FileD}}, {e7, {Row7,FileE}}, {f7, {Row7,FileF}}, {g7, {Row7,FileG}}, {h7, {Row7,FileH}}, 
                                                {a8, {Row8,FileA}}, {b8, {Row8,FileB}}, {c8, {Row8,FileC}}, {d8, {Row8,FileD}}, {e8, {Row8,FileE}}, {f8, {Row8,FileF}}, {g8, {Row8,FileG}}, {h8, {Row8,FileH}} 
                                                };