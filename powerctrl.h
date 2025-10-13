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

//自主测出的第二代参数,在输入转速为电机反馈转速(rpm),输入电流为反馈电流数据/1000.0的情况下
#define M_3508_K0 0.65213
#define M_3508_K1 (-0.15659)
#define M_3508_K2 0.00041660
#define M_3508_K3 0.00235415
#define M_3508_K4 0.20022
#define M_3508_K5 1.08e-7

//视为error较小时的总error参照值
//此参数取决于你的pid参数，参考取自所有电机静态下的最大error值之和
#define M_Too_Small_AllErrors 1000.0

double get_real_current(double current);
double cal_m3508_power_by_model(double current,double speed);

double calculate_attenuation(double desired_current, double current_speed, double power_limit);
std::vector<double> power_allocation_by_speed(std::vector<float>& motor_speeds_vector, double total_power_limit);
std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, double total_power_limit);
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
