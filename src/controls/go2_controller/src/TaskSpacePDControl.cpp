#include "go2_controller.h"

void go2_controller::TaskSpacePDControl(double Kp, double Kd)
{
    // 초기화
    Kp_Task.diagonal() << Kp, Kp, Kp;
    Kd_Task.diagonal() << Kd, Kd, Kd;
   
    // EE_Pose_(FL,FR,RL,RR) 은 이미 구했으니까,
    // EE_Vel_(FL,FR,RL,RR) 설정
    // EE_Vel(ee의 속도) = J(자코비안) * dq(각 관절의 속도)
    Eigen::Vector3d EE_Vel_FL = J_FL.block<3,3>(0,0) * dq_.segment<3>(0);
    Eigen::Vector3d EE_Vel_FR = J_FR.block<3,3>(0,0) * dq_.segment<3>(3);
    Eigen::Vector3d EE_Vel_RL = J_RL.block<3,3>(0,0) * dq_.segment<3>(6);
    Eigen::Vector3d EE_Vel_RR = J_RR.block<3,3>(0,0) * dq_.segment<3>(9);
    
    // 각 EE에서의(Task Space) Force 벡터 구하기
    Eigen::Vector3d Force_FL = Kp_Task * (EE_Pose_FL_desired - EE_Pose_FL) + Kd_Task * (EE_Vel_FL_desired - EE_Vel_FL);
    Eigen::Vector3d Force_FR = Kp_Task * (EE_Pose_FR_desired - EE_Pose_FR) + Kd_Task * (EE_Vel_FR_desired - EE_Vel_FR);
    Eigen::Vector3d Force_RL = Kp_Task * (EE_Pose_RL_desired - EE_Pose_RL) + Kd_Task * (EE_Vel_RL_desired - EE_Vel_RL);
    Eigen::Vector3d Force_RR = Kp_Task * (EE_Pose_RR_desired - EE_Pose_RR) + Kd_Task * (EE_Vel_RR_desired - EE_Vel_RR);

    // J^T를 곱해서 Joint Space로 바꾸기
    // tau = J_v^T * F
    // 참고: 자코비안의 상위 3개 행(선속도 부분)만 사용합니다.
    Eigen::Vector3d Torque_FL = J_FL.block<3,3>(0,0).transpose() * Force_FL;
    Eigen::Vector3d Torque_FR = J_FR.block<3,3>(0,0).transpose() * Force_FR;
    Eigen::Vector3d Torque_RL = J_RL.block<3,3>(0,0).transpose() * Force_RL;
    Eigen::Vector3d Torque_RR = J_RR.block<3,3>(0,0).transpose() * Force_RR;


    // torque_에다가 합치기.
    torque_.segment<3>(0) = Torque_FL;
    torque_.segment<3>(3) = Torque_FR;
    torque_.segment<3>(6) = Torque_RL;
    torque_.segment<3>(9) = Torque_RR;
}