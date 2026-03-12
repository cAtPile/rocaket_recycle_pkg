# rocaket_recycle_pkg

## recycle_info.msg

## stepper_cmd.msg
#stepper_cmd.msg
int32 stepper_a;
int32 stepper_b;
int32 stepper_c;
int32 stepper_d;

const int moveSteps_x1 = stepper_a;   
const int moveSteps_x2 =stepper_b;
const int moveSteps_y1 = stepper_c;
const int moveSteps_y2 = stepper_d;

## todo
//满足双向通信
1. serial_test_master.cpp//测试arduino与主机的通信,主机
2. serial_test_arduino.cpp//arduino从机
3. serial_ctrl.cpp//主机控制电机
4. serial_bridge.cpp//ros参与下的主机控制
5. mission_core.cpp//视觉，电机，数据预处理
