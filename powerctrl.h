//
// Created by 3545 on 25-9-23.
//

#ifndef POWERCTRL_H
#define POWERCTRL_H


#include <numeric>
#include <cstdint>
#include <cmath>
#include <vector>

// 反馈或发送的电流除这个等于真实电流，其实这个应该是819.2的(反馈和发送都是819.2)但是由于我用1000.0拟合的模型所以这里是1000.0
#define M_RealCurrent_Conversion 1000.0

//自主测出的3508第二代参数,在输入转速为电机反馈转速(rpm),输入电流为反馈电流数据/1000.0的情况下
#define M_3508_K0 0.65213
#define M_3508_K1 (-0.15659)
#define M_3508_K2 0.00041660
#define M_3508_K3 0.00235415
#define M_3508_K4 0.20022
#define M_3508_K5 1.08e-7

//6020参数
#define M_6020_K0 0.7507578
#define M_6020_K1 (-0.0759636)
#define M_6020_K2 (-0.00153397)
#define M_6020_K3 0.01225624
#define M_6020_K4 0.19101805
#define M_6020_K5 0.0000066450

//视为error较小时的总error参照值
//error小于大于这个值才会根据error分配功率，否则均分功率
//此参数取决于你的pid参数，参考取自所有电机稳定状态下的最大error值之和，你的pid调的越好这个值越小
#define M_Too_Small_AllErrors 500.0

//小陀螺功率补偿(也可作为 为了稳定不超功率而衰减的 保险补偿)
//关于是否开启此，见.c中std::vector<double> power_allocation_by_error函数内的说明
// #define SmallGyro_Power_Compensation
//补偿的比率，这个数越大 在小陀螺时功率限制的越稳定，但同时非小陀螺时功率利用率也越低
//一般在0.05到0.10之间选择
#define M_SmallGyro_Power_Compensation_Alpha 0.05

//为每个电机预留小部分功率的临界总功率值
//总功率超过这个值就会为每个电机预留功率
//正常情况下 为了稳定不超功率 将这个值设置为整场比赛可能会出现的最低功率来使用这个功能
//当这个值过大的时候（150以上,超过比赛最大分配功率）则视为关闭这个功能
#define M_Motor_ReservedPower_Border 50.0
//每个电机预留功率
//据我测试这个值一般为8就已经能简单抑制由于快速变换速度导致的超功率问题了（虽然可能对于低功率时8还是有一点多）
//这个值越大，在非频繁变换速度的情况下损失的功率越多 同时在频繁变换速度的情况下越稳定
#define M_PerMotor_ReservedPower 8.0

enum E_Motor_PowerModel_Type{M3508_powermodel,GM6020_powermodel};
enum E_CalMotorPower_Negative_Status_Type{E_disabled_negative,E_enable_negative};

double get_real_current(double current);
double cal_motor_power_by_model(E_Motor_PowerModel_Type motor_type ,double current, double speed,E_CalMotorPower_Negative_Status_Type Negative_Status = E_disabled_negative);
double calculate_attenuation(E_Motor_PowerModel_Type motor_type, double desired_current, double current_speed, double power_limit);
std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, double total_power_limit);

double rotate_speed_allocation(int16_t vx, int16_t vy, int16_t rotate, double alpha);

void applyLowPassFilter(double& value, double new_value, double alpha );
class MovingAverageFilter {
public:
    explicit MovingAverageFilter(size_t size);
    double update(double new_value);

private:
    std::vector<double> buffer;
    size_t size;
    size_t index;
    size_t count;
    double sum;
};


#endif //POWERCTRL_H
