cmake_minimum_required(VERSION 3.5)
project(amswarm)

add_compile_options(-std=c++17)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-Ofast -fopenmp -g ${CMAKE_CXX_FLAGS} -lpthread -Wall -Wextra")
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

link_libraries(stdc++fs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)
find_package(eigen-quadprog)
find_package(ACADO REQUIRED)

catkin_package(
)
add_subdirectory(submodules/qpOASES)

include_directories(
include
${catkin_INCLUDE_DIRS}
submodules/qpOASES/include
)

# AM 
add_library(lib_am_swarm src/algorithm/optim_am_swarm.cpp src/simulator/sim_am_swarm.cpp)
add_executable(swarm_am_nav src/main_am_swarm.cpp)
add_dependencies(swarm_am_nav ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_am_nav lib_am_swarm eigen-quadprog yaml-cpp ${catkin_LIBRARIES})


# SCP
add_library(lib_scp_swarm src/algorithm/optim_scp_swarm.cpp src/simulator/sim_scp_swarm.cpp)
add_executable(swarm_scp_nav src/main_scp_swarm.cpp)
add_dependencies(swarm_scp_nav ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_scp_nav lib_scp_swarm eigen-quadprog yaml-cpp qpOASES ${catkin_LIBRARIES})


# # GEN ACADO
# add_executable(generate_acado src/main_acado_gen.cpp)
# target_include_directories(generate_acado PUBLIC ${ACADO_INCLUDE_DIRS})
# target_link_libraries(generate_acado yaml-cpp ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES})
# set_target_properties(generate_acado PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} )

# ACADO 2
add_library(acado_solver_2
  submodules/qpoases/SRC/Bounds.cpp
  submodules/qpoases/SRC/Constraints.cpp
  submodules/qpoases/SRC/CyclingManager.cpp
  submodules/qpoases/SRC/Indexlist.cpp
  submodules/qpoases/SRC/MessageHandling.cpp
  submodules/qpoases/SRC/QProblem.cpp
  submodules/qpoases/SRC/QProblemB.cpp
  submodules/qpoases/SRC/SubjectTo.cpp
  submodules/qpoases/SRC/Utils.cpp
  submodules/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  submodules/codegen_2/acado_qpoases_interface.cpp
  submodules/codegen_2/acado_integrator.c
  submodules/codegen_2/acado_solver.c
  submodules/codegen_2/acado_auxiliary_functions.c)


target_include_directories(acado_solver_2 PUBLIC
  submodules/codegen_2/
  submodules/qpoases
  submodules/qpoases/INCLUDE
  submodules/qpoases/SRC)

add_executable(swarm_acado_nav_2 src/main_acado_nav.cpp)
target_link_libraries(swarm_acado_nav_2 ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES} yaml-cpp acado_solver_2)

# ACADO 4
add_library(acado_solver_4
  submodules/qpoases/SRC/Bounds.cpp
  submodules/qpoases/SRC/Constraints.cpp
  submodules/qpoases/SRC/CyclingManager.cpp
  submodules/qpoases/SRC/Indexlist.cpp
  submodules/qpoases/SRC/MessageHandling.cpp
  submodules/qpoases/SRC/QProblem.cpp
  submodules/qpoases/SRC/QProblemB.cpp
  submodules/qpoases/SRC/SubjectTo.cpp
  submodules/qpoases/SRC/Utils.cpp
  submodules/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  submodules/codegen_4/acado_qpoases_interface.cpp
  submodules/codegen_4/acado_integrator.c
  submodules/codegen_4/acado_solver.c
  submodules/codegen_4/acado_auxiliary_functions.c)


target_include_directories(acado_solver_4 PUBLIC
  submodules/codegen_4/
  submodules/qpoases
  submodules/qpoases/INCLUDE
  submodules/qpoases/SRC)

add_executable(swarm_acado_nav_4 src/main_acado_nav.cpp)
target_link_libraries(swarm_acado_nav_4 ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES} yaml-cpp acado_solver_4)

# ACADO 6
add_library(acado_solver_6
  submodules/qpoases/SRC/Bounds.cpp
  submodules/qpoases/SRC/Constraints.cpp
  submodules/qpoases/SRC/CyclingManager.cpp
  submodules/qpoases/SRC/Indexlist.cpp
  submodules/qpoases/SRC/MessageHandling.cpp
  submodules/qpoases/SRC/QProblem.cpp
  submodules/qpoases/SRC/QProblemB.cpp
  submodules/qpoases/SRC/SubjectTo.cpp
  submodules/qpoases/SRC/Utils.cpp
  submodules/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  submodules/codegen_6/acado_qpoases_interface.cpp
  submodules/codegen_6/acado_integrator.c
  submodules/codegen_6/acado_solver.c
  submodules/codegen_6/acado_auxiliary_functions.c)


target_include_directories(acado_solver_6 PUBLIC
  submodules/codegen_6/
  submodules/qpoases
  submodules/qpoases/INCLUDE
  submodules/qpoases/SRC)

add_executable(swarm_acado_nav_6 src/main_acado_nav.cpp)
target_link_libraries(swarm_acado_nav_6 ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES} yaml-cpp acado_solver_6)

# ACADO 8
add_library(acado_solver_8
  submodules/qpoases/SRC/Bounds.cpp
  submodules/qpoases/SRC/Constraints.cpp
  submodules/qpoases/SRC/CyclingManager.cpp
  submodules/qpoases/SRC/Indexlist.cpp
  submodules/qpoases/SRC/MessageHandling.cpp
  submodules/qpoases/SRC/QProblem.cpp
  submodules/qpoases/SRC/QProblemB.cpp
  submodules/qpoases/SRC/SubjectTo.cpp
  submodules/qpoases/SRC/Utils.cpp
  submodules/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  submodules/codegen_8/acado_qpoases_interface.cpp
  submodules/codegen_8/acado_integrator.c
  submodules/codegen_8/acado_solver.c
  submodules/codegen_8/acado_auxiliary_functions.c)


target_include_directories(acado_solver_8 PUBLIC
  submodules/codegen_8/
  submodules/qpoases
  submodules/qpoases/INCLUDE
  submodules/qpoases/SRC)

add_executable(swarm_acado_nav_8 src/main_acado_nav.cpp)
target_link_libraries(swarm_acado_nav_8 ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES} yaml-cpp acado_solver_8)