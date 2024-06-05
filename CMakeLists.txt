# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# if(CONFIG_IDF_TARGET_ESP32P4) # is not working... :(
# set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/system/console/advanced/components
#                          esp32_p4_function_ev_board)
# endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(webrtc_example)
