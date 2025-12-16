cmake_minimum_required(VERSION 3.16)

project(woorobot_sdk VERSION 1.0
        DESCRIPTION "우로봇 C++ SDK"
        LANGUAGES CXX
)

# C++17 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

file(GLOB_RECURSE SDK_SOURCES
        woorobot_sdk_cpp/*.cpp
)

set(SDK_PUBLIC_INCLUDE_DIR include)
set(SDK_PRIVATE_SRC_DIR woorobot_sdk_cpp)

set(EXAMPLE_MAIN example/cpp/main.cpp)
set(EXAMPLE_CONFIG example/cpp/config.cpp)

list(REMOVE_ITEM SDK_SOURCES ${EXAMPLE_MAIN})
list(REMOVE_ITEM SDK_SOURCES ${EXAMPLE_CONFIG})

add_library(woorobot_sdk SHARED ${SDK_SOURCES})
target_include_directories(woorobot_sdk
        PUBLIC ${SDK_PUBLIC_INCLUDE_DIR}
        PRIVATE ${SDK_PRIVATE_SRC_DIR}
)

add_executable(woorobot_example ${EXAMPLE_MAIN})
add_executable(woorobot_config ${EXAMPLE_CONFIG})
target_link_libraries(woorobot_example PRIVATE woorobot_sdk)
target_link_libraries(woorobot_config PRIVATE woorobot_sdk)

# python 바인딩 모듈
# include(FetchContent)
# FetchContent_Declare(
#         pybind11
#         GIT_REPOSITORY https://github.com/pybind/pybind11.git
#         GIT_TAG v3.0.0
# )
# FetchContent_MakeAvailable(pybind11)
# set(PYBIND11_FINDPYTHON ON)
# pybind11_add_module(woorobot_python woorobot_sdk_python/bindings.cpp)
# set_target_properties(woorobot_python PROPERTIES OUTPUT_NAME "_core")

# target_include_directories(woorobot_python
#     PRIVATE ${SDK_PUBLIC_INCLUDE_DIR}
#     PRIVATE ${SDK_PRIVATE_SRC_DIR}
# )

# install(TARGETS woorobot_python
#     LIBRARY DESTINATION woorobot_sdk_python
#     RUNTIME DESTINATION woorobot_sdk_python
# )
# install(TARGETS woorobot_sdk LIBRARY DESTINATION woorobot_sdk_python)
# set_target_properties(woorobot_python PROPERTIES INSTALL_RPATH "\$ORIGIN")

# target_link_libraries(woorobot_python PRIVATE woorobot_sdk)