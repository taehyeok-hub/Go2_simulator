#include "go2_controller.h"

void go2_controller::TaskSpacePDControl(double Kp_X, double Kp_Y, double Kp_Z, double Kd_X, double Kd_Y, double Kd_Z)
{
    // 초기화
    Kp_Task.diagonal() << Kp_X, Kp_Y, Kp_Z;
    Kd_Task.diagonal() << Kd_X, Kd_Y, Kd_Z;

    // EE_Pose_(FL,FR,RL,RR) 은 이미 구했으니까,
    // EE_Vel_(FL,FR,RL,RR) 설정
    // EE_Vel(ee의 속도) = J(자코비안) * dq(각 관절의 속도)
    // Eigen::Vector3d EE_Vel_FL = jacobian.block<3,3>(0,0) * dq_.segment<3>(0);
    // Eigen::Vector3d EE_Vel_FR = jacobian.block<3,3>(0,3) * dq_.segment<3>(3);
    // Eigen::Vector3d EE_Vel_RL = jacobian.block<3,3>(0,6) * dq_.segment<3>(6);
    // Eigen::Vector3d EE_Vel_RR = jacobian.block<3,3>(0,9) * dq_.segment<3>(9);
    
    // 각 EE에서의(Task Space) Force 벡터 구하기
    Eigen::Vector3d Force_FL = Kp_Task * (EE_Pose_desired[FL] - ee_pose[FL]) + Kd_Task * (EE_Vel_desired[FL] - ee_vel[FL]);
    Eigen::Vector3d Force_FR = Kp_Task * (EE_Pose_desired[FR] - ee_pose[FR]) + Kd_Task * (EE_Vel_desired[FR] - ee_vel[FR]);
    Eigen::Vector3d Force_RL = Kp_Task * (EE_Pose_desired[RL] - ee_pose[RL]) + Kd_Task * (EE_Vel_desired[RL] - ee_vel[RL]);
    Eigen::Vector3d Force_RR = Kp_Task * (EE_Pose_desired[RR] - ee_pose[RR]) + Kd_Task * (EE_Vel_desired[RR] - ee_vel[RR]);

    // J^T를 곱해서 Joint Space로 바꾸기
    // tau = J_v^T * F
    // 참고: 자코비안의 상위 3개 행(선속도 부분)만 사용합니다.
    Eigen::Vector3d Torque_FL = jacobian.block<3,3>(0,0).transpose() * Force_FL;
    Eigen::Vector3d Torque_FR = jacobian.block<3,3>(0,3).transpose() * Force_FR;
    Eigen::Vector3d Torque_RL = jacobian.block<3,3>(0,6).transpose() * Force_RL;
    Eigen::Vector3d Torque_RR = jacobian.block<3,3>(0,9).transpose() * Force_RR;


    // torque_에다가 합치기.
    torque_.segment<3>(0) = Torque_FL;
    torque_.segment<3>(3) = Torque_FR;
    torque_.segment<3>(6) = Torque_RL;
    torque_.segment<3>(9) = Torque_RR;
}

void go2_controller::TaskPD_LegControl(int leg)
{   
    Kp_Task.diagonal() << 500.0, 500.0, 500.0;
    Kd_Task.diagonal() << 30.0, 30.0, 30.0;

    Eigen::Vector3d Force_ = Kp_Task * (EE_Pose_desired[leg] - ee_pose[leg]) + Kd_Task * (EE_Vel_desired[leg] - ee_vel[leg]);
    Eigen::Vector3d Torque_ = jacobian.block<3,3>(0,0).transpose() * Force_;
    Torque[leg] = Torque_;
}