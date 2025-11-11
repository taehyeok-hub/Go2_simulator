#include "SingleRigidBody.h"

SingleRigidBody::SingleRigidBody()
{
    M = 6.921;
    g_world << 0, 0, -9.81;
    I_body << 0.02448, 0.00012166, 0.0014849,
              0.00012166, 0.098077, -3.12E-05,
              0.0014849, -3.12E-05, 0.107;
    
    p_com_w.setZero();
    
    // A-matrix 구현할 떼 필요함. 
    R_world_to_body = R;
    
}

Eigen::Matirx3d SingleRigidBody::MakeCross2Skew(const Eigen::Vector3d &p)
{
    Eigen::Matrix3d P_skew;
    P_skew << 0, -p(3), p(2),
              p(3), 0, -p(1),
             -p(2), p(1), 0;
    return P_skew;
}

void SingleRigidBody::SetFootPosition(const Eigen::Vector3d& p_fl, const Eigen::Vector3d& p_fr,
                                      const Eigen::Vector3d& p_rl, const Eigen::Vector3d& p_rr
                                      const Eigen::Vector3d& p_com)
{
    // parameter로 들어올 변수 -> EE_Pose_FL, EE_Pose_FR, EE_Pose_RL, EE_Pose_RR --> FK 결과
    // p_com --> IMU 센서로 들여옴.

    p_leg_w[0] = p_fl;
    p_leg_w[1] = p_fr;
    p_leg_w[2] = p_rl;
    p_leg_w[3] = p_rl;
    p_com_w = p_com;

    Update_A_Matrix()
}

void SingleRigidBody::Update_A_Matrix()
{
    Eigen::Matirx3d I_3x3 = Eigen::Matrix3d::Identity();
    std::array<Eigen::Matrix3d, 4> P_skew_leg; // r_leg를

    for (int i=0; i<4; i++) 
    {
        // r_leg_w = p_leg_w[i] - p_com_w  -----> r_leg_b = R * r_leg_w
        r_leg_b[i] = R_world_to_body * (p_leg_w[i] - p_com_w);
        P_skew_leg[i] = MakeCross2Skew(r_leg_b[i];)

        A_Matrix_b.block<3,3>(0,3*i) = I_3x3;
        A_Matrix_b.block<3,3>(3,3*i) = P_skew_leg[i];
    }
    
}

void SingleRigidBody::ComputeDesiredWrench(const Eigen::Vector3d& x_ddot_ref, const Eigen::Vector3d& w_dot_ref) // Wrench 는 힘과 토크를 하나로 묶어 부르는 용어임.
{
    b.segment<3>(0) = M * g_world + M * x_ddot_ref;    
    b.segment<3>(3) = I_body * w_dot_ref;
}

void SingleRigidBody::Solve_SRBM_Force(const Eigen::VectorXd& input_vec)
{
    // F = A_mat^T*(A_mat*A_mat^T)^(-1)*b_vec 이거를 하는겁니다. (유사역행렬)
    // 이렇게 하면 A_mat*A_mat^T가 6x6이 되어서 계산이 가능해집니다. 

    b_Vector_b = input_vec;
    F = A_Matrix_b.transpose() * (A_Matrix_b * A_Matrix_b.transpose()).inverse() * b_Vector_b;

    f_FL = F.segmemt<3>(0);
    f_FR = F.segmemt<3>(3);
    f_RL = F.segmemt<3>(6);
    f_RR = F.segmemt<3>(9);
}


/*
먼저
어차피 객체로 접근할 수 있으니까, 
객체로 ,
*/