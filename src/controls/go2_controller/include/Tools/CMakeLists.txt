add_subdirectory(OsqpEigen)

add_library(Tools

    SingleRigidBody.cpp
    Kinematics.cpp
    Trajectories.cpp
    TaskSpacePDControl.cpp
    Centroidal_Dynamics.cpp
    # Centt.cpp
    Pinocchio_Interface.cpp
    
)

target_link_libraries(Tools osqpeigen)