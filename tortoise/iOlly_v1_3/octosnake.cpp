#include <Arduino.h> // 包含 Arduino 核心库
#include <Servo.h> // 包含 Servo 库，用于控制舵机
#include "octosnake.h" // 包含 octosnake 库
#include <math.h> // 包含数学库以获取正弦和弧度转换功能

// 定义 PI，因为某些 Arduino 环境中可能未预定义 PI
#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

// Oscillator 类的构造函数
Oscillator::Oscillator() {
    _period = 2000; // 设置周期为 2000 毫秒
    _amplitude = 50; // 设置振幅为 50
    _phase = 0; // 设置相位为 0
    _offset = 0; // 设置偏移量为 0
    _stop = false; // 设置停止标志为 false，表示不停止
    _ref_time = millis(); // 设置参考时间为当前时间
    _delta_time = 0; // 设置时间差为 0
    _trim = 0; // 设置微调为 0
}

// 更新函数，根据时间变化计算并返回当前的位置
float Oscillator::refresh() {
    _delta_time = (millis() - _ref_time) % _period; // 计算从参考时间开始的时间差，并对周期取模
    return (float)_amplitude * sin(time_to_radians(_delta_time) + degrees_to_radians(_phase)) + _offset + _trim;
}

// 重置参考时间为当前时间
void Oscillator::reset() {
    _ref_time = millis();
}

// 设置周期
void Oscillator::setPeriod(int period) {
    _period = period;
}

// 设置振幅
void Oscillator::setAmplitude(int amplitude) {
    _amplitude = amplitude;
}

// 设置相位
void Oscillator::setPhase(int phase) {
    _phase = phase;
}

// 设置偏移量
void Oscillator::setOffset(int offset) {
    _offset = offset;
}

// 设置微调量
void Oscillator::setTrim(int trim) {
    _trim = trim;
}

// 设置参考时间
void Oscillator::setTime(unsigned long ref) {
    _ref_time = ref;
}

// 获取参考时间
unsigned long Oscillator::getTime() {
    return _ref_time;
}

// 时间转换为弧度的函数
float Oscillator::time_to_radians(double time) {
    return time * 2 * PI / _period; // 将时间转换为周期内的弧度值
}

// 角度转换为弧度的函数
float Oscillator::degrees_to_radians(float degrees) {
    return degrees * PI / 180; // 将角度转换为弧度值
}

// 角度转换为时间的函数
float Oscillator::degrees_to_time(float degrees) {
    return degrees * _period / 360; // 将角度转换为周期内的时间值
}
