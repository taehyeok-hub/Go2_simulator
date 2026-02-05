#include "Model_Predictive.hpp"

ModelPredictive::ModelPredictive()
{
    mass = 15.0;
    mu = 0.3;
    gravity << 0, 0, -9.81;
    I_body << 0.02448, 0.00012166, 0.0014849, 0.00012166, 0.098077, -3.12E-05, 0.0014849, -3.12E-05, 0.107;

    // Initialize parameters
    prediction_horizon = 20; // 예측 지평선 설정
    control_horizon = 5;     // 제어 지평선 설정
    dt = 0.025;              // 샘플링 시간 설정 -> 이거 잘못됐는데

    Force.setZero(NUM_DOF);
    Body_Ref.setZero(NUM_DOF);
    Local_gravity.setZero(NUM_AXIS);
    A_Cost.setZero(NUM_DOF * 2 * prediction_horizon, NUM_DOF * 2 * prediction_horizon);
    B_Cost.setZero(NUM_DOF * 2 * prediction_horizon, 1);
    Q_Cost.setIdentity(NUM_DOF * 2 * prediction_horizon, NUM_DOF * 2 * prediction_horizon);
    Q.setIdentity(NUM_DOF, NUM_DOF);
    R.setIdentity(NUM_DOF, NUM_DOF);
    MPC_State.setZero(NUM_DOF * 2 * prediction_horizon, 1);

    num_of_variables = 2 * NUM_DOF * prediction_horizon;   // 480
    num_of_constraints = 7 * NUM_DOF * prediction_horizon; // 1680

    Hessian.resize(num_of_variables, num_of_variables); // 480 x 480
    Gradient.resize(num_of_variables); // 480 x 1
    LinearMatrix.resize(num_of_constraints, num_of_variables); // 1680 x 480
    LowerBound.resize(num_of_constraints); // 1680 x 1
    UpperBound.resize(num_of_constraints); // 1680 x 1
    QP_Solution.resize(num_of_variables); // 480 x 1

    Hessian.setZero();
    Gradient.setZero();
    LinearMatrix.setZero();
    LowerBound.setZero();
    UpperBound.setZero();
    QP_Solution.setZero();

    Q.diagonal() << 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0; // 로봇 상태에 대한 가중치
    R.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01; // GRF에 대한 가중치

    for (int i = 0; i < 4; i++)
    {
        if (i < 3)
        {
            A_eq[i].setZero(NUM_DOF * prediction_horizon, NUM_DOF * 2 * prediction_horizon); // 0 ~ 2 : 240 x 480 
            L_eq[i].setZero(NUM_DOF * prediction_horizon, 1); // 0 ~ 2 : 240 x 1 
            U_eq[i].setZero(NUM_DOF * prediction_horizon, 1); // 0 ~ 2 : 240 x 1 
        }
        else
        {
            A_eq[i].setZero(4 * NUM_DOF * prediction_horizon, NUM_DOF * 2 * prediction_horizon); // 3 : 960 x 480
            L_eq[i].setZero(4 * NUM_DOF * prediction_horizon, 1); // 3 : 960 x 1
            U_eq[i].setZero(4 * NUM_DOF * prediction_horizon, 1); // 3 : 960 x 1
        }

    }

    for (int i = 0; i < prediction_horizon; i++)
    {
        Q_Cost.block<NUM_DOF,NUM_DOF>(24 * i, 24 * i) = Q;
        Q_Cost.block<NUM_DOF,NUM_DOF>(24 * i + 12, 24 * i + 12) = R;
    }

    for (int i = 0; i < NUM_LEG; i++)
    {
        Gait_Phase[i].setZero(prediction_horizon); 
    }
}

ModelPredictive::~ModelPredictive() {}

void ModelPredictive::SetBodyState(Eigen::VectorXd Body_Pos_, Eigen::VectorXd Body_Vel_, Eigen::VectorXd Body_Quat_, Eigen::VectorXd Body_RPY_, Eigen::VectorXd Body_RPY_D_)
{
    Body_Pos = Body_Pos_;
    Body_Vel = Body_Vel_;
    Body_Quat = Body_Quat_;
    Body_RPY = Body_RPY_;
    Body_RPY_D = Body_RPY_D_;

    SetQuatRotationMatrix(Body_Quat);
    Local_gravity = R_wb * gravity;
}

void ModelPredictive::SetFootState(Eigen::Vector3d Foot_FL_, Eigen::Vector3d Foot_FR_, Eigen::Vector3d Foot_RL_, Eigen::Vector3d Foot_RR_)
{
    // Pinocchio
    Foot_FL = Foot_FL_;
    Foot_FR = Foot_FR_;
    Foot_RL = Foot_RL_;
    Foot_RR = Foot_RR_;
}

void ModelPredictive::ComputeA()
{
    Eigen::Matrix3d Ix3  = Eigen::Matrix3d::Identity();

    A_Matrix.setZero();
    A_Matrix.block<3,3>(0,6) = Ix3;
    A_Matrix.block<3,3>(3,9) = Ix3;

    A_Matrix = dt * A_Matrix + Eigen::Matrix<double, 12, 12>::Identity();

    // std::cout << "===== A_Matrix ===== \n" << A_Matrix << std::endl;
}

void ModelPredictive::ComputeB()
{
    Eigen::Matrix3d Zero = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Ix3_M = Eigen::Matrix3d::Identity() / mass;

    Eigen::Matrix3d r_skew[NUM_LEG];
    r_skew[FL] = I_body.inverse() * SetVecCross2Skew(Foot_FL);
    r_skew[FR] = I_body.inverse() * SetVecCross2Skew(Foot_FR);
    r_skew[RL] = I_body.inverse() * SetVecCross2Skew(Foot_RL);
    r_skew[RR] = I_body.inverse() * SetVecCross2Skew(Foot_RR);

    B_Matrix.setZero();
    for (int leg = 0; leg < NUM_LEG; leg++)
    {
        B_Matrix.block<3, 3>(6, 3 * leg) << r_skew[leg];
        B_Matrix.block<3, 3>(9, 3 * leg) << Ix3_M;
    }

    B_Matrix = dt * B_Matrix;

    // std::cout << "===== B_Matrix ===== \n" << B_Matrix << std::endl;
}

void ModelPredictive::ComputeX()
{
    X_Matrix.block<3, 1>(0, 0) = Body_RPY;   // local
    X_Matrix.block<3, 1>(3, 0) = Body_Pos;   // local
    X_Matrix.block<3, 1>(6, 0) = Body_RPY_D; // local
    X_Matrix.block<3, 1>(9, 0) = Body_Vel;   // local

    // std::cout << "===== X_Matrix ===== \n" << X_Matrix << std::endl;   
}

void ModelPredictive::ComputeC()
{
    Eigen::Vector3d Zero = Eigen::Vector3d::Zero();
    C_Matrix.block<3, 1>(0, 0) = Zero;
    C_Matrix.block<3, 1>(3, 0) = Zero;
    C_Matrix.block<3, 1>(6, 0) = Zero;
    C_Matrix.block<3, 1>(9, 0) = Local_gravity;

    C_Matrix = dt * C_Matrix;

    // std::cout << "===== C_Matrix ===== \n" << C_Matrix << std::endl;   
}

void ModelPredictive::UpdateXMatrix() // test용 함수
{
    X_Next = A_Matrix * X_Matrix + B_Matrix * Force + C_Matrix;
    X_Matrix = X_Next;
}

void ModelPredictive::ComputeModel()
{
    ComputeA();
    ComputeB();
    ComputeC();
    ComputeX();
}

void ModelPredictive::SetRefGait(Eigen::VectorXd Gait_Phase_[])
{
    // 각 다리마다 Prediction Horizon 만큼 상태를 저장해서 돌아감. -> 다리 4개 x 예측 지평선
    for (int i = 0; i < prediction_horizon; i++) 
    {
        for (int leg = 0; leg < NUM_LEG; leg++)
        {
            Gait_Phase[leg](i) = Gait_Phase_[leg](i);
        }
    }

    // for (int i = 0; i < NUM_LEG; i++)
    // {
    //     std::cout << "===== Gait_Phase[" << i << "] ===== \n" << Gait_Phase[i] << std::endl;
    // }
}

void ModelPredictive::SetBodyReference(Eigen::VectorXd Body_Ref_)
{
    // RPY_Command, Pos_Command, ANG_Command, Vel_Command
    Body_Ref << Body_Ref_(6), Body_Ref_(7), Body_Ref_(8), Body_Ref_(0), Body_Ref_(1), Body_Ref_(2),
                Body_Ref_(9), Body_Ref_(10), Body_Ref_(11), Body_Ref_(3), Body_Ref_(4), Body_Ref_(5);

    std::cout << "===== Body_Ref ===== \n" << Body_Ref << std::endl;

    A_Cost.setIdentity();
    B_Cost.setZero();

    for (int i = 0; i < prediction_horizon; i++)
    {
        // x part
        B_Cost.block<12, 1>(24 * i, 0) = Body_Ref;

        // u part (reference 없음)
        B_Cost.block<12, 1>(24 * i + 12, 0).setZero();
    }

    // std::cout << "===== B_Cost ===== \n" << B_Cost << std::endl;   
}

void ModelPredictive::SetCostFunction()
{
    /* sparse 행렬 활용법 : Dense한 행렬을 만들어서 여기서 계산한 다음에 sparse로 흩뿌린다. */
    Eigen::MatrixXd dense_Hessian = A_Cost.transpose() * Q_Cost * A_Cost;
    Hessian = dense_Hessian.sparseView();
    Gradient = 2.0 * (-1) * A_Cost.transpose() * Q_Cost * B_Cost;
}

void ModelPredictive::SetEqConstraint1() // A_matrix, B_matrix, C_matrix 엄밀히 말하면 다르지만, 같다고 가정함.
{
    // Model Dynamcis Constraints
    Eigen::Matrix<double, 12, 12> Ix12 = Eigen::Matrix<double, 12, 12>::Identity();
    A_eq[0].setZero();

    for (int i = 0; i < prediction_horizon; i++)
    {
        A_eq[0].block<12, 12>(i * 12, 24 * i) = Ix12;           // x_{k+i+1} 앞의 I
        A_eq[0].block<12, 12>(i * 12, 24 * i + 12) = -B_Matrix; // u_{k+i} 앞의 -B

        // 다음 상태의 물리 법칙을 위해 현재 상태의 -A를 다음 행에 배치
        if (i < prediction_horizon - 1)
        {
            A_eq[0].block<12, 12>((i + 1) * 12, 24 * i) = -A_Matrix; // -A * x_{k+i+1}
        }
    }

    for (int i = 0; i < prediction_horizon; i++)
    {
        if (i == 0)
        {
            L_eq[0].block<12, 1>(i * 12, 0) = A_Matrix * X_Matrix + C_Matrix; // C
            U_eq[0].block<12, 1>(i * 12, 0) = A_Matrix * X_Matrix + C_Matrix; // 등식 제약조건
        }
        else
        {
            L_eq[0].block<12, 1>(i * 12, 0) = C_Matrix; // C
            U_eq[0].block<12, 1>(i * 12, 0) = C_Matrix; // 등식 제약조건
        }
    }
}

void ModelPredictive::SetEqConstraint2() 
{
    // SwingLeg_Constraints
    // 각 단계마다 4개의 다리에 제약 -> 12 x N
    Eigen::Matrix3d Ix3 = Eigen::Matrix3d::Identity();

    for (int i = 0; i < prediction_horizon; i++)
    {
        Eigen::Matrix<double, 12, 12> Select1 = Eigen::Matrix<double, 12, 12>::Zero();

        for (int leg = 0; leg < NUM_LEG; leg++)
        {   
            if (Gait_Phase[leg](i) == SWING) // prediction_horizon 까지만 가지고 있음 된다.
            {
                Select1.block<3,3>(3 * leg, 3 * leg) = Ix3;
            }
        }

        A_eq[1].block<12, 12>(12 * i, 24 * i + 12) = Select1;
    } 

    L_eq[1].setZero();
    U_eq[1].setZero();
}

void ModelPredictive::SetIneqConstraint1() 
{
    // Fz Constraints
    for (int i = 0; i < prediction_horizon; i++)
    {
        Eigen::Matrix<double, 12, 12> Select2 = Eigen::Matrix<double, 12, 12>::Zero();

        for (int leg = 0; leg < NUM_LEG; leg++)
        {   
            Select2(3 * leg, 3 * leg) = 0.0;
            Select2(3 * leg + 1, 3 * leg + 1) = 0.0;
            Select2(3 * leg + 2, 3 * leg + 2) = 1.0;
        }

        A_eq[2].block<12, 12>(12 * i, 24 * i + 12) = Select2;
    }

    for (int i = 0; i < prediction_horizon; i++)
    {
        // Fz_min : 최소 지지력 
        L_eq[2].block<12,1>(12 * i, 0) << -inf, -inf, 0.0, -inf, -inf, 0.0, -inf, -inf, 0.0, -inf, -inf, 0.0;

        // Fz_max : 최대 지지력 
        U_eq[2].block<12,1>(12 * i, 0) << inf, inf, 200.0, inf, inf, 200.0, inf, inf, 200.0, inf, inf, 200.0;
    }
}

void ModelPredictive::SetIneqConstraint2()
{
    // Friction Cone Constraints
    // Fx - mu * Fz <= 0 , Fx - mu * Fz >= 0
    
    // static int count = 0;
    // if (count == 0)
    // {
    //     A_eq[3].resize(NUM_DOF * 4 * prediction_horizon, NUM_DOF * 2 * prediction_horizon);
    //     L_eq[3].resize(NUM_DOF * 4 * prediction_horizon, 1);
    //     U_eq[3].resize(NUM_DOF * 4 * prediction_horizon, 1);

    //     A_eq[3].setZero();
    //     L_eq[3].setZero();
    //     U_eq[3].setZero();

    //     count++;
    // }

    
    for (int i = 0; i < prediction_horizon; i++)
    {
        Eigen::Matrix<double, 12, 12> Select3 = Eigen::Matrix<double, 12, 12>::Zero();

        for (int leg = 0; leg < NUM_LEG; leg++)
        {   
            Select3(3 * leg, 3 * leg) = 1.0;
            Select3(3 * leg, 3 * leg + 1) = 0.0;
            Select3(3 * leg, 3 * leg + 2) = -mu;
        }

        A_eq[3].block<12, 12>(12 * i, 24 * i + 12) = Select3;

        L_eq[3].block<12,1>(12 * i , 0) << -inf, 0.0, -inf, -inf, 0.0, -inf, -inf, 0.0, -inf, -inf, 0.0, -inf;
        U_eq[3].block<12,1>(12 * i , 0) << 0.0, 0.0, inf, 0.0, 0.0, inf, 0.0, 0.0, inf, 0.0, 0.0, inf;
    }

    
    for (int i = 0; i < prediction_horizon; i++)
    {
        Eigen::Matrix<double, 12, 12> Select4 = Eigen::Matrix<double, 12, 12>::Zero();

        for (int leg = 0; leg < NUM_LEG; leg++)
        {   
            Select4(3 * leg, 3 * leg) = 1.0;
            Select4(3 * leg + 1, 3 * leg + 1) = 0.0;
            Select4(3 * leg + 2, 3 * leg + 2) = mu;
        }

        A_eq[3].block<12, 12>(NUM_DOF * prediction_horizon + 12 * i, 24 * i + 12) = Select4;

        L_eq[3].block<12,1>(NUM_DOF * prediction_horizon  + 12 * i , 0) << 0.0, 0.0, -inf, 0.0, 0.0, -inf, 0.0, 0.0, -inf, 0.0, 0.0, -inf;
        U_eq[3].block<12,1>(NUM_DOF * prediction_horizon  + 12 * i , 0) << inf, 0.0, inf, inf, 0.0, inf, inf, 0.0, inf, inf, 0.0, inf;
    }

    for (int i = 0; i < prediction_horizon; i++)
    {
        Eigen::Matrix<double, 12, 12> Select5 = Eigen::Matrix<double, 12, 12>::Zero();

        for (int leg = 0; leg < NUM_LEG; leg++)
        {   
            Select5(3 * leg, 3 * leg) = 0.0;
            Select5(3 * leg + 1, 3 * leg + 1) = 1.0;
            Select5(3 * leg + 2, 3 * leg + 2) = -mu;
        }

        A_eq[3].block<12, 12>(2 * NUM_DOF * prediction_horizon + 12 * i, 24 * i + 12) = Select5;

        L_eq[3].block<12,1>(2 * NUM_DOF * prediction_horizon  + 12 * i , 0) << 0.0, -inf, -inf, 0.0, -inf, -inf, 0.0, -inf, -inf, 0.0, -inf, -inf;
        U_eq[3].block<12,1>(2 * NUM_DOF * prediction_horizon  + 12 * i , 0) << 0.0, 0.0, inf, 0.0, 0.0, inf, 0.0, 0.0, inf, 0.0, 0.0, inf;
    }

    for (int i = 0; i < prediction_horizon; i++)
    {
        Eigen::Matrix<double, 12, 12> Select6 = Eigen::Matrix<double, 12, 12>::Zero();

        for (int leg = 0; leg < NUM_LEG; leg++)
        {   
            Select6(3 * leg, 3 * leg) = 0.0;
            Select6(3 * leg + 1, 3 * leg + 1) = 1.0;
            Select6(3 * leg + 2, 3 * leg + 2) = mu;
        }

        A_eq[3].block<12, 12>(3 * NUM_DOF * prediction_horizon + 12 * i, 24 * i + 12) = Select6;

        L_eq[3].block<12,1>(3 * NUM_DOF * prediction_horizon  + 12 * i , 0) << 0.0, 0.0, -inf, 0.0, 0.0, -inf, 0.0, 0.0, -inf, 0.0, 0.0, -inf;
        U_eq[3].block<12,1>(3 * NUM_DOF * prediction_horizon  + 12 * i , 0) << 0.0, inf, inf, 0.0, inf, inf, 0.0, inf, inf, 0.0, inf, inf;
    }

}

void ModelPredictive::SetConstraints()
{
    SetEqConstraint1();
    SetEqConstraint2();
    SetIneqConstraint1();
    SetIneqConstraint2();

    Eigen::MatrixXd A_eq_dense = Eigen::MatrixXd::Zero(num_of_constraints, num_of_variables);
   

    // Combine all constraints into LinearMatrix, LowerBound, UpperBound
    for (int i = 0; i < 4; i++)
    {
        if (i < 3)
        {
            A_eq_dense.block<240, 480>(NUM_DOF * i * prediction_horizon, 0) = A_eq[i];
            LowerBound.segment<240>(NUM_DOF * i * prediction_horizon) = L_eq[i];
            UpperBound.segment<240>(NUM_DOF * i * prediction_horizon) = U_eq[i];
        }
        else
        {
            A_eq_dense.block<960, 480>(NUM_DOF * i * prediction_horizon, 0) = A_eq[i];
            LowerBound.segment<960>(NUM_DOF * i * prediction_horizon) = L_eq[i];
            UpperBound.segment<960>(NUM_DOF * i * prediction_horizon) = U_eq[i];
        }
    } 

    LinearMatrix = A_eq_dense.sparseView();
}

void ModelPredictive::SolveQP()
{
    if (!solver.isInitialized())
    {
        solver.settings()->setVerbosity(true);
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
        MPC_State(i) = QP_Solution(i);
    }

    std::cout << "===== Force_Now ===== \n" << MPC_State.block<12,1>(12,0) << std::endl;
} 

