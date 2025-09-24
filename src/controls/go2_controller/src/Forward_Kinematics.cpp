#include "go2_controller.h"

// // 참조에 의한 전달, Pass-by-Reference : 함수 안에서 axis를 변경하면 그 값이 반영된다.
// // (단순 회전 행렬) 
// Eigen::Matrix4d CreateRmatrix(double angle, const Eigen::Vector3d& axis)
// {
//     Eigen::Matrix4d Rotation_Matrix = Eigen::Matrix4d::Identity(); // Matrix4d 클래스의 Rotation_Matrix 객체 생성
//     Eigen::AngleAxisd angleAxis(angle, axis.normalized()); // AngleAxisd 클래스의 angleAxis 객체 생성
//     Rotation_Matrix.block<3, 3>(0, 0) = angleAxis.toRotationMatrix();
//     return Rotation_Matrix;
// }

// // x, y, z만큼 이동하는 4x4 변환 행렬을 생성합니다. (병진 이동 행렬)
// Eigen::Matrix4d CreateTmatrix(double x, double y, double z)
// {
//     Eigen::Matrix4d Translation_Matrix = Eigen::Matrix4d::Identity();
//     Translation_Matrix(0, 3) = x;
//     Translation_Matrix(1, 3) = y;
//     Translation_Matrix(2, 3) = z;
//     return Translation_Matrix;
// }


// void go2_controller::Forward_Kinematics(Eigen::VectorXd q, Eigen::VectorXd dq)
// {
//     // 변수
//     const double base_to_hip_x = 0.1934; // 0.19275
//     const double base_to_hip_y = 0.0465; // 0.145
//     const double hip_to_thigh_y = 0.0955; // 0.085
//     const double thigh_to_calf_z = -0.213; // 0.215
//     const double calf_to_foot_z = -0.213; // 0.215

//     // 회전 축
//     const Eigen::Vector3d hip_axis = {1, 0, 0};
//     const Eigen::Vector3d thigh_axis = {0, 1, 0};
//     const Eigen::Vector3d calf_axis = {0, 1, 0};
    
//     Eigen::Matrix4d T_base_FL = Eigen::Matrix4d::Identity();
//     Eigen::Matrix4d T_base_FR = Eigen::Matrix4d::Identity();
//     Eigen::Matrix4d T_base_RL = Eigen::Matrix4d::Identity();
//     Eigen::Matrix4d T_base_RR = Eigen::Matrix4d::Identity();

//     Eigen::Matrix4d T_base_hip = Eigen::Matrix4d::Zero();
//     Eigen::Matrix4d T_hip_thigh = Eigen::Matrix4d::Zero();
//     Eigen::Matrix4d T_thigh_calf = Eigen::Matrix4d::Zero();
//     Eigen::Matrix4d T_calf_foot = Eigen::Matrix4d::Zero();

//     // FL joint
//     T_base_hip = CreateTmatrix(base_to_hip_x, base_to_hip_y, 0) * CreateRmatrix(q(0), hip_axis);
//     T_hip_thigh = CreateTmatrix(0, hip_to_thigh_y, 0) * CreateRmatrix(q(1), thigh_axis);
//     T_thigh_calf = CreateTmatrix(0, 0, thigh_to_calf_z) * CreateRmatrix(q(2), calf_axis);
//     T_calf_foot = CreateTmatrix(0, 0, calf_to_foot_z); // 고정관절임. (EE)

//     T_base_FL = T_base_hip * T_hip_thigh * T_thigh_calf * T_calf_foot;

//     // FR joint
//     T_base_hip = CreateTmatrix(base_to_hip_x, -base_to_hip_y, 0) * CreateRmatrix(q(3), hip_axis);
//     T_hip_thigh = CreateTmatrix(0, -hip_to_thigh_y, 0) * CreateRmatrix(q(4), thigh_axis); // 대칭
//     T_thigh_calf = CreateTmatrix(0, 0, thigh_to_calf_z) * CreateRmatrix(q(5), calf_axis);
//     T_calf_foot = CreateTmatrix(0, 0, calf_to_foot_z); // 고정관절임. (EE)

//     T_base_FR = T_base_hip * T_hip_thigh * T_thigh_calf * T_calf_foot;

//     // RL joint
//     T_base_hip = CreateTmatrix(-base_to_hip_x, base_to_hip_y, 0) * CreateRmatrix(q(6), hip_axis);
//     T_hip_thigh = CreateTmatrix(0, hip_to_thigh_y, 0) * CreateRmatrix(q(7), thigh_axis);
//     T_thigh_calf = CreateTmatrix(0, 0, thigh_to_calf_z) * CreateRmatrix(q(8), calf_axis);
//     T_calf_foot = CreateTmatrix(0, 0, calf_to_foot_z); // 고정관절임. (EE)

//     T_base_RL = T_base_hip * T_hip_thigh * T_thigh_calf * T_calf_foot;

//     // RR joint
//     T_base_hip = CreateTmatrix(-base_to_hip_x, -base_to_hip_y, 0) * CreateRmatrix(q(9), hip_axis);
//     T_hip_thigh = CreateTmatrix(0, -hip_to_thigh_y, 0) * CreateRmatrix(q(10), thigh_axis); // 대칭
//     T_thigh_calf = CreateTmatrix(0, 0, thigh_to_calf_z) * CreateRmatrix(q(11), calf_axis);
//     T_calf_foot = CreateTmatrix(0, 0, calf_to_foot_z); // 고정관절임. (EE)

//     T_base_RR = T_base_hip * T_hip_thigh * T_thigh_calf * T_calf_foot;

//     EE_Pose_FL = T_base_FL.block<3, 1>(0,3);
//     EE_Pose_FR = T_base_FR.block<3, 1>(0,3);
//     EE_Pose_RL = T_base_RL.block<3, 1>(0,3);
//     EE_Pose_RR = T_base_RR.block<3, 1>(0,3);

//     std::cout << "FL : x = " << EE_Pose_FL(0) << " y = " << EE_Pose_FL(1) << " z = " << EE_Pose_FL(2) << std::endl;
//     std::cout << "FR : x = " << EE_Pose_FR(0) << " y = " << EE_Pose_FR(1) << " z = " << EE_Pose_FR(2) << std::endl;
//     std::cout << "RL : x = " << EE_Pose_RL(0) << " y = " << EE_Pose_RL(1) << " z = " << EE_Pose_RL(2) << std::endl;
//     std::cout << "RR : x = " << EE_Pose_RR(0) << " y = " << EE_Pose_RR(1) << " z = " << EE_Pose_RR(2) << std::endl;
// }











const double pi = 3.14159265358979;
const double body_length = 0.19275;
const double body_width = 0.145;
const double l1 = 0.085;
const double l2 = 0.215;
const double l3 = 0.215; 

// 함수 원형 선언
Eigen::Matrix4d A_Matrix(double a, double alpha, double d, double theta);
// A_Matrix라는 함수가 무엇인지 모르는 상태에서 Forward_Kinematics 함수 안에서 사용하려고 했기 때문에 발생합니다.

void go2_controller::Forward_Kinematics(Eigen::VectorXd q, Eigen::VectorXd dq)
{
    // Base 에서 Hip으로 가는 코드를 만드는 중
    Eigen::Matrix4d A1_FL = Eigen::Matrix4d::Identity();
    A1_FL(0,3) = body_length; A1_FL(1,3) = body_width; // FL
    Eigen::Matrix4d A1_FR = Eigen::Matrix4d::Identity();
    A1_FR(0,3) = body_length; A1_FR(1,3) = -body_width; // FR
    Eigen::Matrix4d A1_RL = Eigen::Matrix4d::Identity();
    A1_RL(0,3) = -body_length; A1_RL(1,3) = body_width; // RL
    Eigen::Matrix4d A1_RR = Eigen::Matrix4d::Identity();
    A1_RR(0,3) = -body_length; A1_RR(1,3) = -body_width; // RR

    Eigen::Matrix4d AM = A_Matrix(0, pi/2.0, 0, pi/2.0);

    // 선언 및 초기화 시키기
    Eigen::Matrix4d A2 = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d A3 = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d A4 = Eigen::Matrix4d::Zero();
    
    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T3 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T4 = Eigen::Matrix4d::Identity();
    

    
    // FL
    A2 = A_Matrix(-0.085, pi/2.0, 0, pi/2.0+q(0));
    A3 = A_Matrix(-0.215, 0, 0, q(1));
    A4 = A_Matrix(-0.215, 0, 0, q(2));

    T1 = A1_FL * AM * A2 * A3 * A4;

    // FR
    A2 = A_Matrix(0.085, -pi/2.0, 0, -pi/2.0+q(3));
    A3 = A_Matrix(0.215, 0, 0, q(4));
    A4 = A_Matrix(0.215, 0, 0, q(5));

    T2 = A1_FR * AM * A2 * A3 * A4;

    // RL
    A2 = A_Matrix(-0.085, pi/2.0, 0, pi/2.0+q(6));
    A3 = A_Matrix(-0.215, 0, 0, q(7));
    A4 = A_Matrix(-0.215, 0, 0, q(8));

    T3 = A1_RL * AM * A2 * A3 * A4;

    // RR
    A2 = A_Matrix(0.085, -pi/2.0, 0, -pi/2.0+q(9));
    A3 = A_Matrix(0.215, 0, 0, q(10));
    A4 = A_Matrix(0.215, 0, 0, q(11));

    T4 = A1_RR * AM * A2 * A3 * A4;

    EE_Pose_FL = T1.block<3,1>(0, 3);
    EE_Pose_FR = T2.block<3,1>(0, 3);
    EE_Pose_RL = T3.block<3,1>(0, 3);
    EE_Pose_RR = T4.block<3,1>(0, 3);

    std::cout << "FL : x = " << EE_Pose_FL(0) << " y = " << EE_Pose_FL(1) << " z = " << EE_Pose_FL(2) << std::endl;
    std::cout << "FR : x = " << EE_Pose_FR(0) << " y = " << EE_Pose_FR(1) << " z = " << EE_Pose_FR(2) << std::endl;
    std::cout << "RL : x = " << EE_Pose_RL(0) << " y = " << EE_Pose_RL(1) << " z = " << EE_Pose_RL(2) << std::endl;
    std::cout << "RR : x = " << EE_Pose_RR(0) << " y = " << EE_Pose_RR(1) << " z = " << EE_Pose_RR(2) << std::endl;
}

Eigen::Matrix4d A_Matrix(double a, double alpha, double d, double theta)
{
    Eigen::Matrix4d A;
    A << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
         sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0,           sin(alpha),               cos(alpha),              d,
         0,           0,                        0,                       1;
    
    return A;
}
