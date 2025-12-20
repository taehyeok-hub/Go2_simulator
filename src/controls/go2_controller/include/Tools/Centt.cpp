#include "Centt.hpp"

/* 
    답답한 점
    : 뭐가 문제있는건지 모르겠다.
    안되면 Global부터 진행해서 순차적으로 Body로 진행해보라고 햇는데 사실상 의미 없다. 
    Global로 바꾸려고 해도 어차피 마지막에는 Body로 바꿔야하고, I_body가 실시간으로 계속 변하니까 부담스러움. 
    FK 결과가 Body임.
    QP는 문제가 없는거같은데 그러면 A, B Matrix중에 문제가 있다는건데 그게 내 눈에는 보이질 않음.
    
    Des_Pos를 어디에다가 엮는지
    
    오답노트
    1. QP에 가중치 행렬 Q 집어넣기 

*/

Centt::Centt()
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
    QP_Solution.resize(num_of_variables);

    Hessian.setZero();
    Gradient.setZero();
    LinearMatrix.setZero();
    LowerBound.setZero();
    UpperBound.setZero();
    QP_Solution.setZero();

    Set_LinearMatrix();
    Set_Constraint();
}

Centt::~Centroidal_Dynamics() {}

void Centt::Set_RobotState(Eigen::VectorXd COM_Pose_, Eigen::VectorXd COM_Vel_, Eigen::VectorXd COM_Quat_, Eigen::VectorXd COM_rpy_, Eigen::VectorXd COM_rpy_dot_)
{
    COM_Pose = COM_Pose_;
    COM_Vel = COM_Vel_;
    COM_Quat = COM_Quat_;
    COM_rpy = COM_rpy_;
    COM_rpy_dot = COM_rpy_dot_;

    Set_QuatRotationMatrix(COM_Quat);
}

void Centt::Set_FootPosition(std::array<Eigen::Vector3d, 4> EE_Pose)
{
   EE_Pose_FL_Body = EE_Pose[FL];
   EE_Pose_FR_Body = EE_Pose[FR];
   EE_Pose_RL_Body = EE_Pose[RL];
   EE_Pose_RR_Body = EE_Pose[RR];
}

void Centt::Compute_A_Matrix()
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

    // std::cout << "===== A_Matrix ===== \n" << A_Matrix << std::endl; 
}

void Centt::Compute_B_Vector(Eigen::Vector3d des_Pos_, Eigen::Matrix3d des_Ori_)
{
    Eigen::Vector3d gravity_body = R_wb * gravity;
    
    des_Pos = des_Pos_; 
    des_Ori = des_Ori_;
    Err_Pos = des_Pos - COM_Pose;
    Err_Ori(0) = WrapToPi(0.0 - COM_rpy(0));
    Err_Ori(1) = WrapToPi(0.0 - COM_rpy(1));
    Err_Ori(2) = WrapToPi(0.0 - COM_rpy(2));

    std::cout << "des_pos : " << des_Pos << std::endl;
    std::cout << "COM_Pose : " << COM_Pose << std::endl;

    Eigen::Vector3d COM_Acc_ref = (Kp_Pos * (des_Pos - COM_Pose) - Kd_Pos * R_wb * COM_Vel) / mass;
    Eigen::Vector3d COM_RPYdot_ref = (Kp_Ori * Err_Ori - Kd_Ori * R_wb * COM_rpy_dot); 

    B_Vector.segment<3>(0) = -mass * gravity_body + mass * COM_Acc_ref;
    B_Vector.segment<3>(3) = I_body * COM_RPYdot_ref;


    // std::cout << "===== Error_Pose ===== \n" << Err_Pos << std::endl; 
    // std::cout << "===== Error_Orientation ===== \n" << Err_Ori << std::endl; 
    // std::cout << "===== B_Matrix ===== \n" << B_Vector << std::endl; 
}

void Centt::Set_CostFunction()
{
    /* sparse 행렬 활용법 : Dense한 행렬을 만들어서 여기서 계산한 다음에 sparse로 흩뿌린다. */ 
    Eigen::MatrixXd dense_Hessian = A_Matrix.transpose() * A_Matrix;
    Hessian = dense_Hessian.sparseView(); 
    Gradient = (-1) * A_Matrix.transpose() * B_Vector;
}

void Centt::Set_LinearMatrix()
{
    LinearMatrix.setZero();

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        LinearMatrix.insert(0 + 5 * leg, 0 + 3 * leg) = 1; 
        LinearMatrix.insert(1 + 5 * leg, 0 + 3 * leg) = 1;
        LinearMatrix.insert(2 + 5 * leg, 1 + 3 * leg) = 1;
        LinearMatrix.insert(3 + 5 * leg, 1 + 3 * leg) = 1;
        LinearMatrix.insert(4 + 5 * leg, 2 + 3 * leg) = 1;

        LinearMatrix.insert(0 + 5 * leg, 2 + 3 * leg) = mu;
        LinearMatrix.insert(1 + 5 * leg, 2 + 3 * leg) = -mu;
        LinearMatrix.insert(2 + 5 * leg, 2 + 3 * leg) = mu;
        LinearMatrix.insert(3 + 5 * leg, 2 + 3 * leg) = -mu;
    }
}

void Centt::Set_Constraint(double fz_max)
{
    const double inf = OsqpEigen::INFTY;
    double fz_min = 0;

    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        LowerBound.segment<5>(5 * leg) << 0, -inf, 0, -inf, fz_min;
        UpperBound.segment<5>(5 * leg) << inf, 0, inf, 0, fz_max;
    }
}

void Centt::Solve_QP()
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