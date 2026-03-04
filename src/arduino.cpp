/*  ESP32-S3 + A4988 + 4899编码器  4轴PID位置闭环
 *  库依赖：
 *  - AccelStepper   (库管理器安装)
 *  - ESP32Encoder   (库管理器安装)
 *  上传后串口 115200 查看实时位置
 ****************************************************/
#include <AccelStepper.h>
#include <ESP32Encoder.h>

// ================= 1. 硬件引脚定义 =================
const int enablePin = 8;     // 低电平使能

const int x1stepPin = 2,  x1dirPin = 5;
const int x2stepPin = 3,  x2dirPin = 6;
const int y1stepPin = 4,  y1dirPin = 7;
const int y2stepPin = 12, y2dirPin = 13;

// 编码器 AB 相（必须带中断的引脚）
const int X1_A = 15, X1_B = 14;
const int X2_A = 22, X2_B = 23;
const int Y1_A = 36, Y1_B = 37;
const int Y2_A = 41, Y2_B = 42;

// ================= 2. PID 结构体与函数声明 =================
typedef struct {
    float aim;
    float err;
    float errNext;
    float errLast;
    float Kp, Ki, Kd;
} PIDType;

void initPID(PIDType *p);
void calculateIncrement(PIDType *p, float realVal, float *pInc);

// ================= 3. 全局对象 =================
ESP32Encoder encX1, encX2, encY1, encY2;
AccelStepper stepperX1(1, x1stepPin, x1dirPin);
AccelStepper stepperX2(1, x2stepPin, x2dirPin);
AccelStepper stepperY1(1, y1stepPin, y1dirPin);
AccelStepper stepperY2(1, y2stepPin, y2dirPin);

int x,y;//视觉输入坐标(待改正)
 
const int moveSteps_x1 = x;   
const int moveSteps_x2 = x+80; 
const int moveSteps_y1 = y; 
const int moveSteps_y2 = y+80; 
// ================= 4. 初始化 =================
void setup() {
    Serial.begin(115200);

    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, LOW);   // 使能

    // 电机参数
    stepperX1.setMaxSpeed(300);  stepperX1.setAcceleration(20);
    stepperX2.setMaxSpeed(300);  stepperX2.setAcceleration(20);
    stepperY1.setMaxSpeed(300);  stepperY1.setAcceleration(20);
    stepperY2.setMaxSpeed(300);  stepperY2.setAcceleration(20);

    // 编码器
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encX1.attachSingleEdge(X1_A, X1_B);
    encX2.attachSingleEdge(X2_A, X2_B);
    encY1.attachSingleEdge(Y1_A, Y1_B);
    encY2.attachSingleEdge(Y2_A, Y2_B);
    encX1.setCount(0); encX2.setCount(0);
    encY1.setCount(0); encY2.setCount(0);

    stepperX1.setCurrentPosition(0);
    stepperX2.setCurrentPosition(0);
    stepperY1.setCurrentPosition(0);
    stepperY2.setCurrentPosition(0);

    startPIDTimer();   // 启动 1 kHz 闭环
}

// ================= 5. 主循环 =================
void loop() {
    // 简单往复运动（目标由 PID 实时修正）
    if (stepperX1.currentPosition() == 0)        stepperX1.moveTo(moveSteps_x1);
    else if (stepperX1.currentPosition() == moveSteps_x1) stepperX1.moveTo(0);

    if (stepperX2.currentPosition() == 0)        stepperX2.moveTo(moveSteps_x2);
    else if (stepperX2.currentPosition() == moveSteps_x2) stepperX2.moveTo(0);

    if (stepperY1.currentPosition() == 0)        stepperY1.moveTo(moveSteps_y1);
    else if (stepperY1.currentPosition() == moveSteps_y1) stepperY1.moveTo(0);

    if (stepperY2.currentPosition() == 0)        stepperY2.moveTo(moveSteps_y2);
    else if (stepperY2.currentPosition() == moveSteps_y2) stepperY2.moveTo(0);

    stepperX1.run();
    stepperX2.run();
    stepperY1.run();
    stepperY2.run();

    // 每 200 ms 打印一次真实位置
    static uint32_t t = 0;
    if (millis() - t > 200) {
        t = millis();
        Serial.printf("X1:%ld  X2:%ld  Y1:%ld  Y2:%ld\n",
                      encX1.getCount(), encX2.getCount(),
                      encY1.getCount(), encY2.getCount());
    }
}

// ================= 6. PID 实现 =================
void initPID(PIDType *p) {
    p->aim = p->err = p->errNext = p->errLast = 0;
    p->Kp = 0.6f;  p->Ki = 0.015f;  p->Kd = 0.20f;
}

void calculateIncrement(PIDType *p, float realVal, float *pInc) {
    p->err = p->aim - realVal;
    *pInc = p->Kp * (p->err - p->errNext)
          + p->Ki * p->err
          + p->Kd * (p->err - 2 * p->errNext + p->errLast);
    p->errLast = p->errNext;
    p->errNext = p->err;
}

// ================= 7. 1 kHz 硬件定时器闭环 =================
PIDType pidX1, pidX2, pidY1, pidY2;

void closeLoopUpdate() {
    static uint32_t lastUs = 0;
    if (micros() - lastUs < 1000) return;
    lastUs = micros();

    int32_t realX1 = encX1.getCount();
    int32_t realX2 = encX2.getCount();
    int32_t realY1 = encY1.getCount();
    int32_t realY2 = encY2.getCount();

    float incX1, incX2, incY1, incY2;
    calculateIncrement(&pidX1, realX1, &incX1);
    calculateIncrement(&pidX2, realX2, &incX2);
    calculateIncrement(&pidY1, realY1, &incY1);
    calculateIncrement(&pidY2, realY2, &incY2);

    // 把增量叠加到 AccelStepper 目标
    stepperX1.moveTo(stepperX1.targetPosition() + (int32_t)incX1);
    stepperX2.moveTo(stepperX2.targetPosition() + (int32_t)incX2);
    stepperY1.moveTo(stepperY1.targetPosition() + (int32_t)incY1);
    stepperY2.moveTo(stepperY2.targetPosition() + (int32_t)incY2);
}

// 1. 全局定时器句柄 ---------------------------------
hw_timer_t *timer = NULL;  

// 2. 中断服务函数 -----------------------------------
void IRAM_ATTR onTimer() {
    BaseType_t xHP = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)NULL, &xHP);  // 强制类型转换
    if (xHP) portYIELD_FROM_ISR();
}

// 3. 启动定时器函数 ---------------------------------
void startPIDTimer() {
    initPID(&pidX1); initPID(&pidX2);
    initPID(&pidY1); initPID(&pidY2);

    timer = timerBegin(1000000);        // 1 MHz
    timerAttachInterrupt(timer, &onTimer);   // 原来是 pidTimer，改为 timer
    timerAlarm(timer, 1000, true, 0);        // 1 kHz

    xTaskCreatePinnedToCore(
        [](void *) {
            for (;;) {
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                closeLoopUpdate();
            }
        }, "pidTask", 4096, NULL, 3, NULL, 1);
}