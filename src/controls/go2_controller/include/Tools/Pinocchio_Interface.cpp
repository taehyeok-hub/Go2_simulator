#include "Pinocchio_Interface.hpp"

// --------------------------
// 내부 유틸: 감쇠 의사역행렬 (wide matrix: 3xN)
// J_pinv = J^T (J J^T + λ^2 I)^{-1}
// --------------------------
static inline Eigen::Matrix<double, NUM_JOINT, PinocchioInterface::kTaskDim>
DampedPseudoInverse3xN(const Eigen::Matrix<double, PinocchioInterface::kTaskDim, NUM_JOINT>& J,
                       double lambda = 1e-6)
{
    Eigen::Matrix<double, PinocchioInterface::kTaskDim, PinocchioInterface::kTaskDim> JJt =
        J * J.transpose();
    JJt.diagonal().array() += lambda * lambda;

    Eigen::Matrix<double, PinocchioInterface::kTaskDim, PinocchioInterface::kTaskDim> JJt_inv =
        JJt.ldlt().solve(Eigen::Matrix<double, PinocchioInterface::kTaskDim, PinocchioInterface::kTaskDim>::Identity());

    return J.transpose() * JJt_inv; // (NUM_JOINT x 3)
}

void PinocchioInterface::Initialize()
{
    std::cout << "Pinocchio Init" << std::endl;

    _q.setZero(n);
    _dq.setZero(n);
    _ddq.setZero(n);

    _pos.setZero();
    _J.setZero();
    _dJ.setZero();

    target_torque_all_.setZero(n);
    target_torque_.setZero(n);

    r_G_.setZero(n);

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        q_[i].setZero();
        dq_[i].setZero();
        ddq_[i].setZero();

        pos_[i].setZero();
        vel_[i].setZero();
        acc_[i].setZero();

        J_[i].setZero();
        dJ_[i].setZero();

        M_[i].setZero();
        C_[i].setZero();

        G_[i].setZero();
        B_[i].setZero();
        T_[i].setZero();

        pos_d_[i].setZero();
        vel_d_[i].setZero();
        acc_d_[i].setZero();

        err_[i].setZero();
        err_dot_[i].setZero();

        target_torque_leg_[i].setZero();
        rpy[i].setZero();
    }

    Kp_.setZero();
    Kd_.setZero();
}

void PinocchioInterface::SetRobotBodyParameter(Eigen::Vector3d body_pos,
                                               Eigen::Vector3d body_vel,
                                               Eigen::Matrix3d body_rot)
{
    body_pos_ = body_pos;
    body_vel_ = body_vel;
    rot_matrix = body_rot;
}

void PinocchioInterface::SetRobotParameter(Eigen::VectorXd q, Eigen::VectorXd dq)
{
    _q = q;
    _dq = dq;

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        // 하드코딩(4) 제거: NUM_JOINT 사용
        q_[i]  = _q.block<NUM_JOINT, 1>(i * NUM_JOINT, 0);
        dq_[i] = _dq.block<NUM_JOINT, 1>(i * NUM_JOINT, 0);
    }

    pinocchio::forwardKinematics(_model, _data, _q);
    pinocchio::computeJointJacobians(_model, _data, _q);
    pinocchio::computeJointJacobiansTimeVariation(_model, _data, _q, _dq);

    // frame placement 최신화(중요)
    pinocchio::updateFramePlacements(_model, _data);
}

void PinocchioInterface::SetKinematics(int legtype)
{
    const int leg = legtype;

    frame_id = _model.getFrameId(foot_name_[leg]);

    // frame placement 최신화(안전)
    pinocchio::updateFramePlacement(_model, _data, frame_id);

    const auto& oMf = _data.oMf[frame_id];

    // rpy는 남겨두되, task space에는 포함하지 않음(3D만 사용)
    rpy[leg] = pinocchio::rpy::matrixToRpy(oMf.rotation());

    // task space: (x,y,z)만
    _pos = oMf.translation();
    pos_[leg] = _pos;

    SetJacobian(leg);

    // task space 속도: v = J dq
    vel_[leg] = J_[leg] * dq_[leg];

    // 가속도는 ddq를 안 쓰는 구조라면 생략 가능
    // acc_[leg] = dJ_[leg] * dq_[leg] + J_[leg] * ddq_[leg];
}

void PinocchioInterface::SetJacobian(int legtype)
{
    const int leg = legtype;

    // pinocchio는 기본적으로 6 x nv jacobian을 준다.
    Eigen::MatrixXd J6(6, _model.nv);
    Eigen::MatrixXd dJ6(6, _model.nv);

    J6.setZero();
    dJ6.setZero();

    pinocchio::getFrameJacobian(_model, _data, frame_id,
                               pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J6);

    pinocchio::getFrameJacobianTimeVariation(_model, _data, frame_id,
                                             pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ6);

    // leg에 해당하는 관절 구간만 추출 + 상단 3행(선속도)만 사용
    const int col0 = leg * NUM_JOINT;

    J_[leg]  = J6.block(0, col0, kTaskDim, NUM_JOINT);
    dJ_[leg] = dJ6.block(0, col0, kTaskDim, NUM_JOINT);

    _J  = J_[leg];
    _dJ = dJ_[leg];
}

void PinocchioInterface::SetDynamics(
    const Eigen::Matrix<double, kTaskDim, NUM_LEG>& pos_d,
    const Eigen::Matrix<double, kTaskDim, NUM_LEG>& vel_d,
    const Eigen::Matrix<double, kTaskDim, NUM_LEG>& acc_d,
    const TaskMatrix& Kp,
    const TaskMatrix& Kd)
{
    Kp_ = Kp;
    Kd_ = Kd;

    // 동역학 항 계산
    pinocchio::crba(_model, _data, _q);
    pinocchio::nonLinearEffects(_model, _data, _q, _dq);
    pinocchio::computeGeneralizedGravity(_model, _data, _q);

    r_M_ = _data.M;
    r_T_ = _data.tau;
    r_B_ = _data.nle;
    r_G_ = _data.g;

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        const int idx = static_cast<int>(i) * NUM_JOINT;

        // ⚠️ 원본 코드의 i*NUM_LEG 오타 수정: i*NUM_JOINT가 맞음
        M_[i] = r_M_.block<NUM_JOINT, NUM_JOINT>(idx, idx);
        M_[i].triangularView<Eigen::StrictlyLower>() =
            M_[i].transpose().triangularView<Eigen::StrictlyLower>();

        B_[i] = r_B_.block<NUM_JOINT, 1>(idx, 0);
        T_[i] = r_T_.block<NUM_JOINT, 1>(idx, 0);
        G_[i] = r_G_.block<NUM_JOINT, 1>(idx, 0);

        // 목표 task (x,y,z)
        pos_d_[i] = pos_d.col(i);
        vel_d_[i] = vel_d.col(i);
        acc_d_[i] = acc_d.col(i);

        // 오차
        err_[i]     = pos_d_[i] - pos_[i];
        err_dot_[i] = vel_d_[i] - vel_[i];
    }
}

// (기존 MatrixXd/Matrix4d 방식 호출을 유지하고 싶으면 아래 오버로드를 헤더에도 선언해서 사용)
void PinocchioInterface::SetDynamics(Eigen::MatrixXd pos_d,
                                     Eigen::MatrixXd vel_d,
                                     Eigen::MatrixXd acc_d,
                                     Eigen::Matrix4d Kp,
                                     Eigen::Matrix4d Kd)
{
    // 3x3만 사용 (상위 3x3)
    TaskMatrix Kp3 = Kp.block<kTaskDim, kTaskDim>(0, 0);
    TaskMatrix Kd3 = Kd.block<kTaskDim, kTaskDim>(0, 0);

    Eigen::Matrix<double, kTaskDim, NUM_LEG> pd, vd, ad;
    pd.setZero(); vd.setZero(); ad.setZero();

    // 원본은 row(i)에 task를 넣는 구조였는데, 여기서는 col로 통일
    // pos_d: (NUM_LEG x 3) 또는 (NUM_LEG x 4)로 들어온다고 가정하고 앞 3개만 사용
    for (int i = 0; i < NUM_LEG; i++)
    {
        pd.col(i) = pos_d.row(i).head<kTaskDim>().transpose();
        vd.col(i) = vel_d.row(i).head<kTaskDim>().transpose();
        ad.col(i) = acc_d.row(i).head<kTaskDim>().transpose();
    }

    SetDynamics(pd, vd, ad, Kp3, Kd3);
}

void PinocchioInterface::SetGravity()
{
    pinocchio::crba(_model, _data, _q);
    pinocchio::nonLinearEffects(_model, _data, _q, _dq);
    pinocchio::computeGeneralizedGravity(_model, _data, _q);

    r_M_ = _data.M;
    r_B_ = _data.nle;
    r_G_ = _data.g;

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        const int idx = static_cast<int>(i) * NUM_JOINT;

        // ⚠️ 원본 코드의 i*NUM_LEG 오타 수정
        M_[i] = r_M_.block<NUM_JOINT, NUM_JOINT>(idx, idx);
        M_[i].triangularView<Eigen::StrictlyLower>() =
            M_[i].transpose().triangularView<Eigen::StrictlyLower>();

        B_[i] = r_B_.block<NUM_JOINT, 1>(idx, 0);
        G_[i] = r_G_.block<NUM_JOINT, 1>(idx, 0);
    }
}

Eigen::VectorXd PinocchioInterface::GetDynamics()
{
    const double torque_limit_ = 1000.0;

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        // qdd_des = J_pinv * (xdd_des + Kp*e + Kd*edot - dJ*dq)
        auto Jpinv = DampedPseudoInverse3xN(J_[i], 1e-6);

        JointVector qdd_des =
            Jpinv * (acc_d_[i] + Kp_ * err_[i] + Kd_ * err_dot_[i] - dJ_[i] * dq_[i]);

        JointVector tau = M_[i] * qdd_des + B_[i];

        // 토크 제한(선택)
        for (int j = 0; j < NUM_JOINT; j++)
        {
            if (tau(j) > torque_limit_) tau(j) = torque_limit_;
            if (tau(j) < -torque_limit_) tau(j) = -torque_limit_;
        }

        target_torque_leg_[i] = tau;
        target_torque_all_.segment(i * NUM_JOINT, NUM_JOINT) = tau;
    }

    return target_torque_all_;
}

void PinocchioInterface::SetTaskspacePD(const TaskMatrix& Kp,
                                        const TaskMatrix& Kd,
                                        const TaskVector pos_d[NUM_LEG],
                                        const TaskVector vel_d[NUM_LEG],
                                        const TaskVector acc_d[NUM_LEG])
{
    Kp_ = Kp;
    Kd_ = Kd;

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        pos_d_[i] = pos_d[i];
        vel_d_[i] = vel_d[i];
        acc_d_[i] = acc_d[i];

        err_[i]     = pos_d_[i] - pos_[i];
        err_dot_[i] = vel_d_[i] - vel_[i];
    }
}

// (기존 시그니처 유지용 래퍼: 헤더에도 같이 선언해서 쓰기)
void PinocchioInterface::SetTaskspacePD(Eigen::MatrixXd Kp,
                                        Eigen::MatrixXd Kd,
                                        Eigen::VectorXd pos_d[],
                                        Eigen::VectorXd vel_d[],
                                        Eigen::VectorXd acc_d[])
{
    TaskMatrix Kp3 = Kp.block<kTaskDim, kTaskDim>(0, 0);
    TaskMatrix Kd3 = Kd.block<kTaskDim, kTaskDim>(0, 0);

    TaskVector pd[NUM_LEG], vd[NUM_LEG], ad[NUM_LEG];

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        pd[i] = pos_d[i].head<kTaskDim>();
        vd[i] = vel_d[i].head<kTaskDim>();
        ad[i] = acc_d[i].head<kTaskDim>();
    }

    SetTaskspacePD(Kp3, Kd3, pd, vd, ad);
}

void PinocchioInterface::ComputeCTM()
{
    Eigen::Matrix<double, NUM_JOINT, 1> tmp_torque[NUM_LEG];
    const double torque_limit_ = 500;

    for (size_t i = 0; i < NUM_LEG; i++)
    {
        auto Jinv = DampedPseudoInverse3xN(J_[i], lambda); // (NUM_JOINT x 3)

        tmp_torque[i] = M_[i] * Jinv * (acc_d_[i] - dJ_[i] * dq_[i]) + B_[i] + J_[i].transpose() * (Kp_ * err_[i] + Kd_ * err_dot_[i]); // Task-space PD

        // // CTC + Task-Space PD
        // tmp_torque[i] = M_[i] * J_[i].inverse() * (acc_d_[i] - dJ_[i] * dq_[i]) + NLE_[i] + J_[i].transpose() * (Kp_ * err_[i] + Kd_ * err_dot_[i]);

        // Task-Space PD
        // tmp_torque[i] =  J_[i].transpose() * (Kp_ * err_[i] + Kd_ * err_dot_[i]);

        target_torque_.block(i * NUM_JOINT, 0, NUM_JOINT, 1) = tmp_torque[i];
    }
}