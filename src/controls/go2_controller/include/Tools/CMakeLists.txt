add_subdirectory(OsqpEigen)

add_library(Tools

    SingleRigidBody.cpp
    Kinematics.cpp
    Trajectories.cpp
    TaskSpacePDControl.cpp
    Centroidal_Dynamics.cpp
    Pinocchio_Interface.cpp
    Gait_Generator.cpp
    Trajectory_Generator.cpp
    Model_Predictive.cpp
)

target_link_libraries(Tools osqpeigen)