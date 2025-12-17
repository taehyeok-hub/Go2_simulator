#include "Centroidal_Dynamics.hpp"

Centroidal_Dynamics::Centroidal_Dynamics()
{
    mass = 15.0;
    mu = 0.7;
    gravity << 0, 0, -9.81;
    I_body << 0.02448, 0.00012166, 0.0014849, 0.00012166, 0.098077, -3.12E-05, 0.0014849, -3.12E-05, 0.107;

    Kp_Pos.setIdentity();
    Kp_Pos.diagonal() << 100.0, 100.0, 100.0;

    Kd_Pos.setIdentity();
    Kd_Pos.diagonal() << 1.0, 1.0, 1.0;

    Kp_Ori.setIdentity();
    Kp_Ori.diagonal() << 100.0, 100.0, 100.0;

    Kd_Ori.setIdentity();
    Kd_Ori.diagonal() << 1.0, 1.0, 1.0;

    Hessian.resize(num_of_variables, num_of_variables);
    Gradient.resize(num_of_variables);
    LinearMatrix.resize(num_of_constraints, num_of_variables);
    LowerBound.resize(num_of_constraints);
    UpperBound.resize(num_of_constraints);
}

Centroidal_Dynamics::~Centroidal_Dynamics() {}

void Centroidal_Dynamics::Set_RobotState(Eigen::VectorXd COM_Pose_, Eigen::VectorXd COM_Vel_, Eigen::VectorXd COM_Quat_, Eigen::VectorXd COM_rpy_, Eigen::VectorXd COM_rpy_dot_)
{
    COM_Pose = COM_Pose_;
    COM_Vel = COM_Vel_;
    COM_Quat = COM_Quat_;
    COM_rpy = COM_rpy_;
    COM_rpy_dot = COM_rpy_dot_;

    Set_QuatRotationMatrix(COM_Quat);
}

void Centroidal_Dynamics::Set_FootPosition(std::array<Eigen::Vector3d, 4> EE_Pose)
{
   EE_Pose_FL_Body = EE_Pose[FL];
   EE_Pose_FR_Body = EE_Pose[FR];
   EE_Pose_RL_Body = EE_Pose[RL];
   EE_Pose_RR_Body = EE_Pose[RR];
}

void Centroidal_Dynamics::Compute_A_Matrix()
{
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 4; i++)
    {
        A_Matrix.block<3,3>(0, 3 * i) = I;
    }
    
    A_Matrix.block<3,3>(3,0) = Set_VecCross2Skew(EE_Pose_FL_Body);
    A_Matrix.block<3,3>(3,3) = Set_VecCross2Skew(EE_Pose_FR_Body);
    A_Matrix.block<3,3>(3,6) = Set_VecCross2Skew(EE_Pose_RL_Body);
    A_Matrix.block<3,3>(3,9) = Set_VecCross2Skew(EE_Pose_RR_Body);

    std::cout << "===== A_Matrix ===== \n" << A_Matrix << std::endl; 
}

void Centroidal_Dynamics::Compute_B_Vector(Eigen::Vector3d des_Pos_, Eigen::Matrix3d des_Ori_)
{
    Eigen::Vector3d gravity_body = R_wb * gravity;
    
    des_Pos = des_Pos_; 
    des_Ori = des_Ori_;
    Err_Pos = des_Pos - COM_Pose;
    Err_Ori = ErrOri_CrossProduct(R_bw, des_Ori);

    Eigen::Vector3d COM_Acc_ref = (Kp_Pos * R_wb * (des_Pos - COM_Pose) - Kd_Pos * R_wb * COM_Vel) / mass;
    Eigen::Vector3d COM_RPYdot_ref = (Kp_Ori * (ErrOri_CrossProduct(R_bw, des_Ori)) - Kd_Ori * COM_rpy_dot); 

    B_Vector.segment<3>(0) = -mass * gravity_body + mass * COM_Acc_ref;
    B_Vector.segment<3>(3) = I_body * COM_RPYdot_ref;

    std::cout << "===== B_Matrix ===== \n" << B_Vector << std::endl; 
}

void Centroidal_Dynamics::Set_CostFunction()
{
    /* sparse 행렬 활용법 : Dense한 행렬을 만들어서 여기서 계산한 다음에 sparse로 흩뿌린다. */ 
    Eigen::MatrixXd dense_Hessian = A_Matrix.transpose() * A_Matrix;
    Hessian = dense_Hessian.sparseView(); 
    Gradient = (-1) * A_Matrix.transpose() * B_Vector;
}

void Centroidal_Dynamics::Set_LinearMatrix()
{
    LinearMatrix.setZero();

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        LinearMatrix.insert(0 + 5 * leg, 0 + 3 * leg) = 1;
        LinearMatrix.insert(1 + 5 * leg, 0 + 3 * leg) = 1;
        LinearMatrix.insert(2 + 5 * leg, 1 + 3 * leg) = 1;
        LinearMatrix.insert(3 + 5 * leg, 1 + 3 * leg) = 1;
        LinearMatrix.insert(4 + 5 * leg, 2 + 3 * leg) = 1;

        LinearMatrix.insert(0 + 5 * leg, 2 + 3 * leg) = -mu;
        LinearMatrix.insert(1 + 5 * leg, 2 + 3 * leg) = mu;
        LinearMatrix.insert(2 + 5 * leg, 2 + 3 * leg) = -mu;
        LinearMatrix.insert(3 + 5 * leg, 2 + 3 * leg) = mu;
    }
}

void Centroidal_Dynamics::Set_Constraint(double fz_max)
{
    const double inf = OsqpEigen::INFTY;
    double fz_min = 1e-6;

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        LowerBound.segment<5>(5 * leg) << -inf, 0, -inf, 0, fz_min;
        UpperBound.segment<5>(5 * leg) << 0, inf, 0, inf, fz_max;
    }
}

void Centroidal_Dynamics::Solve_QP()
{
    Set_CostFunction();

    if (!solver.isInitialized())
    {
        solver.settings()->setVerbosity(true);
        solver.settings()->setWarmStart(true);
        solver.data()->setNumberOfVariables(NUM_LEG * NUM_AXIS);
        solver.data()->setNumberOfConstraints(num_of_constraints * NUM_LEG);

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

}