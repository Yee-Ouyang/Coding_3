#ifndef minikame_h   // 如果没有定义 minikame_h
#define minikame_h   // 定义 minikame_h

#include <Servo.h>   // 包含 Servo 库，用于控制舵机
#include "octosnake.h"  // 包含 octosnake 库

// 定义一个 MiniKame 类
class MiniKame {

    public:  // 公共成员函数
        void init();  // 初始化函数
        void imitate(int s0,int s1,int s2,int s3,int s4,int s5,int s6,int s7,int s8);
        void run(float steps, float period);  // 前进函数
        void walk(float steps, float period);  // 行走函数
        void runback(float steps, float period);  // 后退函数
        void turnL(float steps, float period);  // 向左转函数
        void turnR(float steps, float period);  // 向右转函数
        void moonwalkL(float steps, float period);  // 向左的月球行走函数
        void dance(float steps, float period);  // 跳舞函数
        void upDown(float steps, float period);  // 上下移动函数
        void pushUp(float steps, float period);  // 做俯卧撑函数
        void hello();  // 打招呼函数
        void walkback(float steps, float period);  // 后退行走函数
        void lookDown();  // 向下看函数
        void lookUp();  // 向上看函数
        void lookUpRight();  // 向右上看函数
        void lookUpLeft();  // 向左上看函数
        void lookDownRight();  // 向右下看函数
        void lookDownLeft();  // 向左下看函数
        void lookLeft();  // 向左看函数
        void lookRight();  // 向右看函数
        void shakeNo();  // 摇头（表示不）函数
        void calibrate();  // 校准函数
        void afraid();  // 害怕动作函数
        void confused();  // 困惑动作函数
        void bye();  // 再见函数
        void shakeYes(float steps, float period);  // 点头（表示是）函数
        void headbang(float steps, float period);  // 用头撞击动作函数
        void home();  // 返回起始位置函数
        void center();  // 中心位置函数
        void rest();  // 休息函数
        void pie();  // 圆周率运动/自定义动作函数
        void zero();  // 归零所有舵机函数
        void frontBack(float steps, float period);  // 前后摆动函数

        void setServo(int id, float target);  // 设置舵机目标角度函数
        void reverseServo(int id);  // 反转舵机方向函数
        float getServo(int id);  // 获取舵机当前角度函数
        void moveServos(int time, float target[9]);  // 同时移动多个舵机到目标位置函数

    private:  // 私有成员变量和函数
        Oscillator oscillator[9];  // 振荡器数组，用于控制舵机的动作
        Servo servo[9];  // 舵机数组
        int board_pins[9];  // 舵机连接到控制板上的引脚数组
        int trim[9];  // 舵机微调值数组
        bool reverse[9];  // 舵机是否反向数组
        unsigned long _init_time;  // 动作开始时间
        unsigned long _final_time;  // 动作结束时间
        unsigned long _partial_time;  // 动作部分时间
        float _increment[9];  // 舵机每一步的增量数组
        float _servo_position[9];  // 舵机当前位置数组

        //int angToUsec(float value);  // 角度转换为微秒函数（未使用，被注释掉了）
        void execute(float steps, float period[9], int amplitude[9], int offset[9], int phase[9]);  // 执行动作函数
};

#endif // 结束 ifndef minikame_h
