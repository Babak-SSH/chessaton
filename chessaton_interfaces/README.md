# chessaton_interfaces

chessatons message, service and action interface types.

```bash
.
├── action                          # [dir] action types
├── msg                             # [dir] msg types
├──srv                              # [dir] service types
    ├── BoxPositions.srv            # box and target positions
    ├── GetBestMove.srv             # best move from chess engine
    └── SetState.srv                # setting state of ros node
├── package.xml                     # Ros2 package metadata
├── README.md
└──  CMakeLists.txt                 # colcon-enabled CMake recipe
```