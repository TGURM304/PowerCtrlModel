//
// Created by 3545 on 25-9-23.
//

#ifndef POWERCTRL_H
#define POWERCTRL_H


#include <numeric>
#include <cstdint>
#include <cmath>
#include <vector>

#define M_RealCurrent_Conversion 1000.0

#define M_3508_K0 0.65213
#define M_3508_K1 (-0.15659)
#define M_3508_K2 0.00041660
#define M_3508_K3 0.00235415
#define M_3508_K4 0.20022
#define M_3508_K5 1.08e-7

#define M_6020_K0 0.7507578
#define M_6020_K1 (-0.0759636)
#define M_6020_K2 (-0.00153397)
#define M_6020_K3 0.01225624
#define M_6020_K4 0.19101805
#define M_6020_K5 0.0000066450

#define M_Too_Small_AllErrors 500.0

#define M_SmallGyro_Power_Compensation_Alpha 0.05

#define M_Motor_ReservedPower_Border 50.0
#define M_PerMotor_ReservedPower 8.0

enum E_Motor_PowerModel_Type{M3508_powermodel,GM6020_powermodel};
enum E_CalMotorPower_Negative_Status_Type{E_disabled_negative,E_enable_negative};

double get_real_current(double current);
double cal_motor_power_by_model(E_Motor_PowerModel_Type motor_type ,double current, double speed,E_CalMotorPower_Negative_Status_Type Negative_Status = E_disabled_negative);
double calculate_attenuation(E_Motor_PowerModel_Type motor_type, double desired_current, double current_speed, double power_limit);
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
