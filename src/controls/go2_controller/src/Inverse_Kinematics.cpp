#include "go2_controller.h"

// FK 결과
// FL : x = 0.205272 y = 0.130531 z = -0.414215
// FR : x = 0.205272 y = -0.130531 z = -0.414215
// RL : x = -0.180228 y = 0.130531 z = -0.414215
// RR : x = -0.180228 y = -0.130531 z = -0.414215


// ee에 EE_pose 넣겠고
// theta1 = 0 이후 사용 X
Eigen::Vector3d Calculate_V(Eigen::Vector3d& ee, Eigen::Vector3d& bb) // double angle 생략함
{
    // 내가 짰던 코드
    // Eigen::Matrix3d Rotate_Mz << cos(rad(angle)) , -sin(rad(angle)), 0, sin(rad(angle)), cos(rad(angle)), 0, 0, 0, 1;
    // Eigen::Matrix3d Rotate_Mx << 1, 0, 0, 0, cos(rad(angle)), -sin(rad(angle)), 0, sin(rad(angle)), cos(rad(angle));
    // Eigen::Vector3d Result_V = transpose(Rotate_Mz * Rotate_Mx) * (ee - bb);

    // return Result_V;

    Eigen::Matrix3d Rz, Rx, R;

    Rz << 0, -1, 0,
          1,  0, 0,
          0,  0, 1;

    Rx << 1, 0 , 0,
          0, 0, -1,
          0, 1,  0;
    
    R = Rz * Rx;

    return R.transpose() * (ee -bb);
}

// Eigen::Vector3d Calculate_Atan2(Eigen::Vector3d& p_hip, double a1, double a2, double a3)
// {
//     // 내가 짰던 것
//     // Eigen::Vector3d joint;
//     // double theta1, double theta2, double theta3;
//     // double D = (sqrt(nn(2)^2 + nn(1)^2) - a2^2 - a3^2) / (2 * a2 * a3);
    
//     // theta1 = atan2(nn(1), nn(0));
//     // theta3 = atan2(sqrt(1 - D^2), D);
//     // theta2 = atan2(nn(2), nn(1)) - atan2(a2 * sin(theta3), a2 + a3*cos(theta3));

//     // joint << theta1, theta2, theta3;
   
//     // return joint;
//     // ----------------------------------------------------------------------------------


//     Eigen::Vector3d joint_angles;
//     double theta1, theta2, theta3;
    
//     double x_h = p_hip(0);  
//     double y_h = p_hip(1);
//     double z_h = p_hip(2);
    
//     //Geometrical하게 IK푸는 경우 (Articulated Manipulator)
//     theta1 = atan2(y_h, x_h);
    
//     double l_xy = sqrt(x_h * x_h +y_h * y_h);
//     double D = (l_xy * l_xy + z_h * z_h - a2 * a2 - a3 * a3) / (2 * a2 * a3);

//     double D = (y_h* y_h + z_h * z_h - a2 * a2 - a3 * a3) / (2 * a2 * a3);
//     theta3 = atan2(sqrt(1-D*D), D);
//     // # theta1 = 0 으로 인한 변경점 존재
//     theta2 = atan2(z_h, y_h) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));

//     joint_angles << theta1, theta2, theta3;
//     return joint_angles;
// }

// theta1 = 0 가정 후 진행한 Atan2
Eigen::Vector3d Calculate_Atan2(Eigen::Vector3d& p_thigh, double a2, double a3)
{
    Eigen::Vector3d joint_angles; // joint angle 모아두는 곳을 만들어놓고
    double theta2, theta3; // 
    double x = p_thigh.x();
    double z = p_thigh.z();

    double L_ik_sq = p_thigh.squaredNorm(); // 거리
    double D = (L_ik_sq - a2*a2 - a3*a3) / (2 * a2 * a3);
    
    theta3 = atan2(sqrt(1 - D * D), D);
    theta2 = atan2(x, -z) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));

    joint_angles << 0, theta2, theta3;

    return joint_angles;
}


void go2_controller::geometrical_IK()
{
    // Eigen::Vector3d FL_base_hip(0.19275, 0.145, 0); // 값을 바로 선언할 때는 이렇게 한답니다 (매개변수 있는 생성자를 사용하는 것과 같습니다). 
    // Eigen::Vector3d FR_base_hip(0.19275, -0.145, 0);
    // Eigen::Vector3d RL_base_hip(-0.19275, 0.145, 0);
    // Eigen::Vector3d RR_base_hip(-0.19275, -0.145, 0);

    //선언 하고 할당을 따로 할 때에는
    // Eigen::Vector3d FL_base_hip;
    // FL_base_hip << 0.19275, 0.145, 0; 이렇게 합니다.

    // const double l1 = 0.085, l2 = 0.215, l3 = 0.215; // 변하지 않는 수니깐 상수취급하는 keyword 추가

    // Eigen::Vector3d h_xyz;

    // // FL
    // h_xyz = Calculate_V(EE_Pose_FL, FL_base_hip);
    // //all_joint_angles << Calculate_Atan2(h_xyz, l2, l3); -> 할당할 때 이렇게 하면 안되고, matrix.block<>()처럼 이렇게 해야됨.
    // all_joint_angles.segment<3>(0) = Calculate_Atan2(h_xyz, l1, l2, l3);

    // // FR
    // h_xyz = Calculate_V(EE_Pose_FR, FR_base_hip);
    // all_joint_angles.segment<3>(3) = Calculate_Atan2(h_xyz, l1, l2, l3);

    // // RL
    // h_xyz = Calculate_V(EE_Pose_RL, RL_base_hip);
    // all_joint_angles.segment<3>(6) = Calculate_Atan2(h_xyz, l1, l2, l3);

    // // RR
    // h_xyz = Calculate_V(EE_Pose_RR, RR_base_hip);
    // all_joint_angles.segment<3>(9) = Calculate_Atan2(h_xyz, l1, l2, l3);


    // std::cout << all_joint_angles.transpose() << std::endl;


    // theta1 = 0 가정 후 진행.
    Eigen::Vector3d FL_thigh_origin(0.19275, 0.145, -0.085);
    Eigen::Vector3d FR_thigh_origin(0.19275, -0.145, -0.085);
    Eigen::Vector3d RL_thigh_origin(-0.19275, 0.145, -0.085);
    Eigen::Vector3d RR_thigh_origin(-0.19275, -0.145, -0.085);

    const double l2 = 0.215, l3 = 0.215; // Thigh, Calf 링크 길이

    // FK
    Eigen::Vector3d V_ik_fl = EE_Pose_FL - FL_thigh_origin;
    all_joint_angles.segment<3>(0) = Calculate_Atan2(V_ik_fl, l2, l3);

    // FK
    Eigen::Vector3d V_ik_fr = EE_Pose_FR - FR_thigh_origin;
    all_joint_angles.segment<3>(3) = Calculate_Atan2(V_ik_fr, l2, l3);
    
    // FK
    Eigen::Vector3d V_ik_rl = EE_Pose_RL - RL_thigh_origin;
    all_joint_angles.segment<3>(6) = Calculate_Atan2(V_ik_rl, l2, l3);

    // FK
    Eigen::Vector3d V_ik_rr = EE_Pose_RR - RR_thigh_origin;
    all_joint_angles.segment<3>(9) = Calculate_Atan2(V_ik_rr, l2, l3);
    
    std::cout << all_joint_angles.transpose() << std::endl;
}
