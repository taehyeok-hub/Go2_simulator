#include "go2_controller.h"

Eigen::Matrix4d A_Matrix(double theta, double d, double a, double alpha)
{
    Eigen::Matrix4d A;
    A << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
         sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0,           sin(alpha),               cos(alpha),              d,
         0,           0,                        0,                       1;
    
    return A;
}




void go2_controller::Create_Jacobian(const Eigen::VectorXd& q)
{
    const double base_to_hip_x = 0.1934;
    const double base_to_hip_y = 0.0465;
    const double hip_to_thigh = 0.0955;
    const double thigh_to_calf = 0.213;
    const double calf_to_foot = 0.213;

    Eigen::Matrix4d A_ROTATE = A_Matrix(M_PI/2.0, 0, 0, M_PI/2.0);

    // Base 에서 Hip으로 가는 코드를 만드는 중
    Eigen::Matrix4d A_base_hip_FL = Eigen::Matrix4d::Identity();
    A_base_hip_FL(0,3) = base_to_hip_x; A_base_hip_FL(1,3) = base_to_hip_y; // FL
    Eigen::Matrix4d A_hip_FL = A_base_hip_FL * A_ROTATE;

    Eigen::Matrix4d A_base_hip_FR = Eigen::Matrix4d::Identity();
    A_base_hip_FR(0,3) = base_to_hip_x; A_base_hip_FR(1,3) = -base_to_hip_y; // FR
    Eigen::Matrix4d A_hip_FR = A_base_hip_FR * A_ROTATE;

    Eigen::Matrix4d A_base_hip_RL = Eigen::Matrix4d::Identity();
    A_base_hip_RL(0,3) = -base_to_hip_x; A_base_hip_RL(1,3) = base_to_hip_y; // RL
    Eigen::Matrix4d A_hip_RL = A_base_hip_RL * A_ROTATE;
     
    Eigen::Matrix4d A_base_hip_RR = Eigen::Matrix4d::Identity();
    A_base_hip_RR(0,3) = -base_to_hip_x; A_base_hip_RR(1,3) = -base_to_hip_y; // RR
    Eigen::Matrix4d A_hip_RR = A_base_hip_RR * A_ROTATE;


    // FL 구하기
    Eigen::Matrix4d A_thigh_FL = A_hip_FL * A_Matrix(q(0)+M_PI/2.0, 0, -hip_to_thigh, M_PI/2.0);
    Eigen::Matrix4d A_calf_FL = A_thigh_FL * A_Matrix(q(1), 0, -thigh_to_calf, 0);
    Eigen::Matrix4d A_ee_FL = A_calf_FL * A_Matrix(q(2), 0, -calf_to_foot, 0);

    // FR 구하기
    Eigen::Matrix4d A_thigh_FR = A_hip_FR * A_Matrix(q(3)+M_PI/2.0, 0, hip_to_thigh, M_PI/2.0);
    Eigen::Matrix4d A_calf_FR = A_thigh_FR * A_Matrix(q(4), 0, thigh_to_calf, 0);
    Eigen::Matrix4d A_ee_FR = A_calf_FR * A_Matrix(q(5), 0, calf_to_foot, 0);

    // RL 구하기
    Eigen::Matrix4d A_thigh_RL = A_hip_RL * A_Matrix(q(6)+M_PI/2.0, 0, -hip_to_thigh, M_PI/2.0);
    Eigen::Matrix4d A_calf_RL = A_thigh_RL * A_Matrix(q(7), 0, -thigh_to_calf, 0);
    Eigen::Matrix4d A_ee_RL = A_calf_RL * A_Matrix(q(8), 0, -calf_to_foot, 0);

    // RR 구하기
    Eigen::Matrix4d A_thigh_RR = A_hip_RR * A_Matrix(q(9)+M_PI/2.0, 0, hip_to_thigh, M_PI/2.0);
    Eigen::Matrix4d A_calf_RR = A_thigh_RR * A_Matrix(q(10), 0, thigh_to_calf, 0);
    Eigen::Matrix4d A_ee_RR = A_calf_RR * A_Matrix(q(11), 0, calf_to_foot, 0);
    
    // z_FL 추출하기
    Eigen::Vector3d z_hip_FL = A_hip_FL.block<3,1>(0,2);
    Eigen::Vector3d z_thigh_FL = A_thigh_FL.block<3,1>(0,2);
    Eigen::Vector3d z_calf_FL = A_calf_FL.block<3,1>(0,2);
    // o_FL 추출하기
    Eigen::Vector3d o_hip_FL = A_hip_FL.block<3,1>(0,3);
    Eigen::Vector3d o_thigh_FL = A_thigh_FL.block<3,1>(0,3);
    Eigen::Vector3d o_calf_FL = A_calf_FL.block<3,1>(0,3);
    Eigen::Vector3d o_ee_FL = A_ee_FL.block<3,1>(0,3);

    // z_FR 추출하기
    Eigen::Vector3d z_hip_FR = A_hip_FR.block<3,1>(0,2);
    Eigen::Vector3d z_thigh_FR = A_thigh_FR.block<3,1>(0,2);
    Eigen::Vector3d z_calf_FR = A_calf_FR.block<3,1>(0,2);
    // o_FR 추출하기
    Eigen::Vector3d o_hip_FR = A_hip_FR.block<3,1>(0,3);
    Eigen::Vector3d o_thigh_FR = A_thigh_FR.block<3,1>(0,3);
    Eigen::Vector3d o_calf_FR = A_calf_FR.block<3,1>(0,3);
    Eigen::Vector3d o_ee_FR = A_ee_FR.block<3,1>(0,3);

    // z_RL 추출하기
    Eigen::Vector3d z_hip_RL = A_hip_RL.block<3,1>(0,2);
    Eigen::Vector3d z_thigh_RL = A_thigh_RL.block<3,1>(0,2);
    Eigen::Vector3d z_calf_RL = A_calf_RL.block<3,1>(0,2);
    // o_RL 추출하기
    Eigen::Vector3d o_hip_RL = A_hip_RL.block<3,1>(0,3);
    Eigen::Vector3d o_thigh_RL = A_thigh_RL.block<3,1>(0,3);
    Eigen::Vector3d o_calf_RL = A_calf_RL.block<3,1>(0,3);
    Eigen::Vector3d o_ee_RL = A_ee_RL.block<3,1>(0,3);

    // z_RR 추출하기
    Eigen::Vector3d z_hip_RR = A_hip_RR.block<3,1>(0,2);
    Eigen::Vector3d z_thigh_RR = A_thigh_RR.block<3,1>(0,2);
    Eigen::Vector3d z_calf_RR = A_calf_RR.block<3,1>(0,2);
    // o_RR 추출하기
    Eigen::Vector3d o_hip_RR = A_hip_RR.block<3,1>(0,3);
    Eigen::Vector3d o_thigh_RR = A_thigh_RR.block<3,1>(0,3);
    Eigen::Vector3d o_calf_RR = A_calf_RR.block<3,1>(0,3);
    Eigen::Vector3d o_ee_RR = A_ee_RR.block<3,1>(0,3);

    

    // J_FL 자코비안 구하기
    J_FL.block<3,1>(0,0) = z_hip_FL.cross(o_ee_FL - o_hip_FL);
    J_FL.block<3,1>(0,1) = z_thigh_FL.cross(o_ee_FL - o_thigh_FL);
    J_FL.block<3,1>(0,2) = z_calf_FL.cross(o_ee_FL - o_calf_FL);
    J_FL.block<3,1>(3,0) = z_hip_FL;
    J_FL.block<3,1>(3,1) = z_thigh_FL;
    J_FL.block<3,1>(3,2) = z_calf_FL;


     // J_FR 자코비안 구하기
    J_FR.block<3,1>(0,0) = z_hip_FR.cross(o_ee_FR - o_hip_FR);
    J_FR.block<3,1>(0,1) = z_thigh_FR.cross(o_ee_FR - o_thigh_FR);
    J_FR.block<3,1>(0,2) = z_calf_FR.cross(o_ee_FR - o_calf_FR);
    J_FR.block<3,1>(3,0) = z_hip_FR;
    J_FR.block<3,1>(3,1) = z_thigh_FR;
    J_FR.block<3,1>(3,2) = z_calf_FR;
    
     // J_RL 자코비안 구하기
    J_RL.block<3,1>(0,0) = z_hip_RL.cross(o_ee_RL - o_hip_RL);
    J_RL.block<3,1>(0,1) = z_thigh_RL.cross(o_ee_RL - o_thigh_RL);
    J_RL.block<3,1>(0,2) = z_calf_RL.cross(o_ee_RL - o_calf_RL);
    J_RL.block<3,1>(3,0) = z_hip_RL;
    J_RL.block<3,1>(3,1) = z_thigh_RL;
    J_RL.block<3,1>(3,2) = z_calf_RL;

     // J_RR 자코비안 구하기
    J_RR.block<3,1>(0,0) = z_hip_RR.cross(o_ee_RR - o_hip_RR);
    J_RR.block<3,1>(0,1) = z_thigh_RR.cross(o_ee_RR - o_thigh_RR);
    J_RR.block<3,1>(0,2) = z_calf_RR.cross(o_ee_RR - o_calf_RR);
    J_RR.block<3,1>(3,0) = z_hip_RR;
    J_RR.block<3,1>(3,1) = z_thigh_RR;
    J_RR.block<3,1>(3,2) = z_calf_RR;
    

    // 6x12의 통합 자코비안 만들기
    J.block<6,3>(0,0) = J_FL;
    J.block<6,3>(0,3) = J_FR;
    J.block<6,3>(0,6) = J_RL;
    J.block<6,3>(0,9) = J_RR;

    //std::cout << J << std::endl;
}