cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++14")

set(SOURCES main.cpp model.cpp tgaimage.cpp texture.cpp geometry.cpp)
set(HEADERS model.h tgaimage.h texture.h geometry.h)

add_executable (render ${SOURCES} ${HEADERS})
add_custom_target(
    Resources ALL
    ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/obj ${PROJECT_BINARY_DIR}/obj
    SOURCES ${Resources}
)
