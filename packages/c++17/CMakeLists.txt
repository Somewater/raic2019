cmake_minimum_required(VERSION 3.5)
project(MyStrategy)

# for debug
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 \
#     -fno-optimize-sibling-calls -fno-strict-aliasing \
#     -Wall -Wtype-limits -Wno-unknown-pragmas")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 \
#    -fno-optimize-sibling-calls -fno-strict-aliasing \
#    -lm -s -O2 -Wall -Wtype-limits -Wno-unknown-pragmas")

# original
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -static \
     -fno-optimize-sibling-calls -fno-strict-aliasing -D_LINUX \
     -lm -s -O2 -Wall -Wtype-limits -Wno-unknown-pragmas")

file(GLOB strategy_SRC "*.cpp" "model/*.cpp" "csimplesocket/*.cpp")

add_executable(MyStrategy ${strategy_SRC})
