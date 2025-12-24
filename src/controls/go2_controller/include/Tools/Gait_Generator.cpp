#include "Gait_Generator.hpp"

Gait_Generator::Gait_Generator()
{
    // Gait.State1 << 1.0, 1.0, 1.0, 1.0;
    // Gait.State2 << 1.0, 0.0, 0.0, 1.0;
    // Gait.State3 << 0.0, 1.0, 1.0, 0.0;

    Trot_Pattern.setZero(4, 4);
    Trot_Pattern << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;
    State1 << 1.0, 1.0, 1.0, 1.0;
    State2 << 1.0, 0.0, 0.0, 1.0;
    State3 << 1.0, 1.0, 1.0, 1.0;
    State4 << 0.0, 1.0, 1.0, 0.0;
}

Gait_Generator::~Gait_Generator() {}

void Gait_Generator::Trot_Gait_Generator(double switch_time)
{
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    // 보행 주기 -> 이초마다 교차
    int period = static_cast<int>(switch_time * 500);

    // switch (Gait_Switch) {
    // case 0:
    //     Target_State = State1;
    //     break;
    // case 1:
    //     Target_State = State2;
    //     break;
    // case 2:
    //     Target_State = State3;
    //     break;
    // case 3:
    //     Target_State = State4;
    //     break;
    // }

    // if (Switch_Time < period)
    // {
    //     Switch_Time++;
    // }
    // else if (Switch_Time == period)
    // {
    //     Switch_Time = 0;
    //     Gait_Switch = (Gait_Switch + 1) % 4;
    // }

    // 2. 공통 카운터 로직 (어떤 상태든 시간은 똑같이 흐름)
    Switch_Time++;
    if (Switch_Time >= period)
    {
        // 0 -> 1 -> 2 -> 3 ->0 순환
        Gait_Switch = (Gait_Switch + 1) % 4;  
        Switch_Time = 0;
    }


    // 3. 현재 인덱스에 맞는 행을 바로 할당 (if문 4개가 한 줄로!)
    Target_State = I.row(Gait_Switch);
}