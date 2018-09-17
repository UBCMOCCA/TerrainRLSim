cmake_minimum_required(VERSION 2.6)
project(cmaes)

add_definitions(
  -O3
  -pedantic
  -Wall
  -Wextra
  -DNDEBUG
)

include_directories(cma-es)

# doxygen, source:
# http://permalink.gmane.org/gmane.comp.programming.tools.cmake.user/4930
find_package(Doxygen)
  if(DOXYGEN_FOUND)
  set(DOXYGEN_INPUT doxygen.cfg)
  add_custom_target(apidoc
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif(DOXYGEN_FOUND)

add_executable(evo1 example1.cpp)
add_executable(evo2 example2.cpp)
