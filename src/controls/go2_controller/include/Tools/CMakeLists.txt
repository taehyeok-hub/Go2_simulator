add_subdirectory(OsqpEigen)

add_library(Tools

    SingleRigidBody.cpp
    Kinematics.cpp
    Trajectories.cpp
    TaskSpacePDControl.cpp

)

target_link_libraries(Tools osqpeigen)