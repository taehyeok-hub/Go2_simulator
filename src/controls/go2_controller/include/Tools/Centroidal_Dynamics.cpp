#include "Centroidal_Dynamics.hpp"

/*
    오답 노트
    1. COM_Pose 랑, COM_RPY, COM_Quat는 그냥 global로 받아도 된다. -> 움직이지 않는 것이 목적인 코드이기 때문에
    2.
*/

Centroidal_Dynamics::Centroidal_Dynamics()
{
    mass = 15.0;
    mu = 0.3;
    gravity << 0, 0, -9.81;
    I_body << 0.02448, 0.00012166, 0.0014849, 0.00012166, 0.098077, -3.12E-05, 0.0014849, -3.12E-05, 0.107;

    Kp_Pos.setIdentity();
    Kp_Pos.diagonal() << 30000.0, 35000.0, 20000.0; //  10000.0, 10000.0, 10000.0 // 30000.0, 35000.0, 20000.0

    Kd_Pos.setIdentity();
    Kd_Pos.diagonal() << 2000.0, 2300.0, 1200.0; // 600.0, 600.0, 600.0 // 2000.0, 2300.0, 1200.0

    Kp_Ori.setIdentity();
    Kp_Ori.diagonal() << 6000.0, 6000.0, 6000.0; //  5000.0, 5000.0, 5000.0 // 6000.0, 6000.0, 6000.0

    Kd_Ori.setIdentity();
    Kd_Ori.diagonal() << 225.0, 225.0, 225.0; //  200.0, 200.0, 200.0 // 225.0, 225.0, 225.0

    Hessian.resize(num_of_variables, num_of_variables);
    Gradient.resize(num_of_variables);
    LinearMatrix.resize(num_of_constraints, num_of_variables);
    LowerBound.resize(num_of_constraints);
    UpperBound.resize(num_of_constraints);
    QP_Solution.resize(num_of_variables);

    Hessian.setZero();
    Gradient.setZero();
    LinearMatrix.setZero();
    LowerBound.setZero();
    UpperBound.setZero();
    QP_Solution.setZero();

    Q = Eigen::MatrixXd::Identity(6, 6); // 아주 정밀한 Gain Tuning을 진행할 때 활용
    Q(0, 0) = 0.1;
    Q(1, 1) = 0.1;
    Q(2, 2) = 10.0; // X축, Y축, Z축 (선형)가중치

    Q(3, 3) = 100.0;
    Q(4, 4) = 30.0;
    Q(5, 5) = 30.0; // Roll, Pitch, Yaw 방향 가중치

    Set_LinearMatrix();
}

Centroidal_Dynamics::~Centroidal_Dynamics() {}

void Centroidal_Dynamics::Set_RobotState(Eigen::VectorXd COM_Pose_, Eigen::VectorXd COM_Vel_, Eigen::VectorXd COM_Quat_, Eigen::VectorXd COM_RPY_, Eigen::VectorXd COM_RPY_D_)
{
    COM_Pose = COM_Pose_; // VectorXd ==> Vector3d 로 만들어둔다. 
    COM_Vel = COM_Vel_;
    COM_Quat = COM_Quat_;
    COM_RPY = COM_RPY_;
    COM_RPY_D = COM_RPY_D_;

    Set_QuatRotationMatrix(COM_Quat);

    Err_Pos(X) = COM_Pose(X);
    Err_Pos(Y) = COM_Pose(Y);
    Err_Pos(Z) = COM_Pose(Z);
}

void Centroidal_Dynamics::Set_FootPosition(Eigen::Vector3d Pino_FL_, Eigen::Vector3d Pino_FR_, Eigen::Vector3d Pino_RL_, Eigen::Vector3d Pino_RR_)
{
    EE_Pose_FL_Body = Pino_FL_;
    EE_Pose_FR_Body = Pino_FR_;
    EE_Pose_RL_Body = Pino_RL_;
    EE_Pose_RR_Body = Pino_RR_;
}

void Centroidal_Dynamics::Set_FKFootPosition(std::array<Eigen::Vector3d, 4> EE_Pose)
{
    EE_Pose_FL_Body = EE_Pose[FL];
    EE_Pose_FR_Body = EE_Pose[FR];
    EE_Pose_RL_Body = EE_Pose[RL];
    EE_Pose_RR_Body = EE_Pose[RR];
}

void Centroidal_Dynamics::Set_RefGait(Eigen::VectorXd Gait_Ref_[]) // 성민이형 거
{
    for (int leg = 0; leg < NUM_LEG; ++leg)
    {
        Gait_Ref[leg] = Gait_Ref_[leg](0);
    }
}

void Centroidal_Dynamics::Set_GaitPhase(Eigen::Vector4d Target_State_)
{
    Leg_Gait[FL] = Target_State_(0);
    Leg_Gait[FR] = Target_State_(1);
    Leg_Gait[RL] = Target_State_(2);
    Leg_Gait[RR] = Target_State_(3);
    
    // std::cout << "=== CENT_LEG_GAIT === \n FL : " << Leg_Gait[0] << " | FR : " << Leg_Gait[1] << " | RL : " << Leg_Gait[2] << " | RR : " << Leg_Gait[3] << std::endl;  
}

void Centroidal_Dynamics::Compute_A_Matrix()
{
    for (int i = 0; i < 4; i++)
    {
        A_Matrix.block<3, 3>(0, 3 * i) = Eigen::Matrix3d::Identity();;
    }

    A_Matrix.block<3, 3>(3, 0) = Set_VecCross2Skew(EE_Pose_FL_Body);
    A_Matrix.block<3, 3>(3, 3) = Set_VecCross2Skew(EE_Pose_FR_Body);
    A_Matrix.block<3, 3>(3, 6) = Set_VecCross2Skew(EE_Pose_RL_Body);
    A_Matrix.block<3, 3>(3, 9) = Set_VecCross2Skew(EE_Pose_RR_Body);

    // std::cout << "===== A_Matrix ===== \n" << A_Matrix << std::endl;
}

void Centroidal_Dynamics::Set_Reference(Eigen::VectorXd COM_Ref_) 
{
    /* COM_Ref_ : Reference COM 위치 -> 속도 -> 각도(RPY) -> 각속도(RPY_dot)*/
    Ref_Pos << COM_Ref_(0), COM_Ref_(1), COM_Ref_(2); 
    Ref_Vel << COM_Ref_(3), COM_Ref_(4), COM_Ref_(5);    // 병진 이동할 때 활용할 것으로 예상
    Ref_RPY_D << 0.0, 0.0, 0.0;                          // 이거는 회전 이동할 때 활용할 것으로 예상되지만, roll, pitch는 활용할 의미가 없기 때문에 yaw만 사용함.

    Eigen::Vector3d e_rpy = Eigen::Vector3d::Zero();
    e_rpy(0) = Wrap2PI(COM_Ref_(6) - COM_RPY(0));
    e_rpy(1) = Wrap2PI(COM_Ref_(7) - COM_RPY(1));
    e_rpy(2) = Wrap2PI(COM_Ref_(8) - COM_RPY(2));

    Err_R = e_rpy;
    // Err_R = ErrOri_so3(R_wb, I);         

    Lin_Acc_ref = (Kp_Pos * (Ref_Pos - Err_Pos) + Kd_Pos * (Ref_Vel - COM_Vel)) / mass;         // 전진, 후진 보행시 ??
    Ang_Acc_ref = (Kp_Ori * Err_R - Kd_Ori * COM_RPY_D);
}

void Centroidal_Dynamics::Compute_B_Vector()
{
    Eigen::Vector3d gravity_body = R_wb * gravity;

    B_Vector.segment<3>(0) = mass * gravity_body + mass * Lin_Acc_ref;
    B_Vector.segment<3>(3) = I_body * Ang_Acc_ref;

    // std::cout << "===== B_Vector ===== \n" << B_Vector << std::endl;
}

void Centroidal_Dynamics::Set_CostFunction()
{
    /* sparse 행렬 활용법 : Dense한 행렬을 만들어서 여기서 계산한 다음에 sparse로 흩뿌린다. */
    Eigen::MatrixXd dense_Hessian = A_Matrix.transpose() * Q * A_Matrix;
    Hessian = dense_Hessian.sparseView();
    Gradient = (-1) * A_Matrix.transpose() * Q * B_Vector;
}

void Centroidal_Dynamics::Set_LinearMatrix()
{
    LinearMatrix.setZero();

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        int rowBase = 5 * leg;
        int colBase = 3 * leg;

        LinearMatrix.insert(0 + rowBase, 0 + colBase) = 1;
        LinearMatrix.insert(1 + rowBase, 0 + colBase) = 1;
        LinearMatrix.insert(2 + rowBase, 1 + colBase) = 1;
        LinearMatrix.insert(3 + rowBase, 1 + colBase) = 1;
        LinearMatrix.insert(4 + rowBase, 2 + colBase) = 1;

        LinearMatrix.insert(0 + rowBase, 2 + colBase) = mu;
        LinearMatrix.insert(1 + rowBase, 2 + colBase) = -mu;
        LinearMatrix.insert(2 + rowBase, 2 + colBase) = mu;
        LinearMatrix.insert(3 + rowBase, 2 + colBase) = -mu;
    }
}

void Centroidal_Dynamics::Set_Constraint(double fz_min_, double fz_max_)
{
    const double inf = OsqpEigen::INFTY;
    
    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        double contact = Gait_Ref[leg]; // Leg_Gait[leg] , Gait_Ref[leg]

        double fz_min = contact * fz_min_;
        double fz_max = contact * fz_max_;

        // if (Gait_Ref[leg] == SWING)
        // {
        //     gt = 0.0;
        // }
        // else if (Gait_Ref[leg] == STANCE)
        // {
        //     gt = 1.0;
        // }

        LowerBound.segment<5>(5 * leg) << 0, -inf, 0, -inf, fz_min; // fz_min = 2
        UpperBound.segment<5>(5 * leg) << inf, 0, inf, 0, fz_max; // fz_max = 200
    }

    // std::cout << "=== LowerBound === \n" << LowerBound << std::endl;
    // std::cout << "=== UpperBound === \n" << UpperBound << std::endl;
}


void Centroidal_Dynamics::Solve_QP()
{
    Set_CostFunction();

    if (!solver.isInitialized())
    {
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(num_of_variables);
        solver.data()->setNumberOfConstraints(num_of_constraints);

        solver.data()->setHessianMatrix(Hessian);
        solver.data()->setGradient(Gradient);
        solver.data()->setLinearConstraintsMatrix(LinearMatrix);
        solver.data()->setLowerBound(LowerBound);
        solver.data()->setUpperBound(UpperBound);
        solver.initSolver();
    }
    else
    {
        solver.updateHessianMatrix(Hessian);
        solver.updateGradient(Gradient);
        solver.updateBounds(LowerBound, UpperBound);
    }

    solver.solveProblem();

    QP_Solution = solver.getSolution();

    for (int i = 0; i < num_of_variables; i++)
    {
        F_Vector(i) = QP_Solution(i);
    }

    // std::cout << "===== Force_Vector ===== \n" << F_Vector << std::endl;
}