//
// Created by 3545 on 25-9-23.
//

#include "powerctrl.h"


//获取真实(其实并不真实，是过单位换算的)电流（此处根据3508电机定义获取）
//自2025.10.12之后的版本后，计算功率的模型函数和计算衰减系数的函数均在内部调用此函数
//所以你可以在调用那两个函数时直接填入电机反馈的电流值
//该函数仅用于优化模型时采样电流来显示的作用
double get_real_current(double current) {

    const double real_current = current/M_RealCurrent_Conversion;//此处实际反馈电流不知道什么单位，按毫安估算所以除1000.0!!!!!不过貌似其实应该除819.2,但是我已经拟合完了
    return real_current;                                    //同样的，你在调用计算衰减系数函数时也要调用此函数或把想要发送的电流除1000.0！！！！！！！！！！！！！！
                                                            //也就是在.h文件里宏定义的M_RealCurrent_Conversion这个
}

// 功率模型函数的实现
//在调用这个函数时填入电机电流（反馈电流和发送电流均可，单位一致）和电机反馈速度，接收得到的预测功率值
double cal_motor_power_by_model(E_Motor_PowerModel_Type motor_type ,double current, double speed) {

    //近似认为电机正反转所有参数高度对称，所以加上绝对值
    current = std::abs(get_real_current(current));
    speed = std::abs(speed);

    switch (motor_type) {
    case M3508_powermodel:
        return M_3508_K0 +
               M_3508_K1 * current +
               M_3508_K2 * speed +
               M_3508_K3 * current * speed +
               M_3508_K4 * current * current +
               M_3508_K5 * speed * speed;
    case GM6020_powermodel:
        return M_6020_K0 +
               M_6020_K1 * current +
               M_6020_K2 * speed +
               M_6020_K3 * current * speed +
               M_6020_K4 * current * current +
               M_6020_K5 * speed * speed;
    default:
        return 0.0;
    }

}

//基于所需速度的等比功率分配
//这一版理论不如根据error值分配的版本，详细见下一个函数
std::vector<double> power_allocation_by_speed(std::vector<float>& motor_speeds_vector, const double total_power_limit) {

    //查输入是否有效
    if (motor_speeds_vector.size() != 4) {
        return {0.0, 0.0, 0.0, 0.0} ;
    }

    //对速度取绝对值
    for (float& speed : motor_speeds_vector) {
        speed = std::abs(speed);
    }

    // 如果总功率上限为0，那么所有电机的功率上限都为0
    if (total_power_limit <= 1e-9) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    // 1. 计算总速度
    const double total_speed = std::accumulate(motor_speeds_vector.begin(), motor_speeds_vector.end(), 0.0);

    // 2. 处理总速度为0的特殊情况
    // 如果所有电机都不转，一个公平的策略是平均分配功率上限，或者都为0。
    // 这里选择平均分配，因为即使不转，可能也需要基础功耗。
    if (total_speed <= 1e-9) {
        double equal_share = total_power_limit / motor_speeds_vector.size();
        return {equal_share, equal_share, equal_share, equal_share};
    }

    // 3. 按比例分配功率
    std::vector<double> motor_power_limits_vector(4);
    for (int i = 0; i < 4; ++i) {
        const double ratio = motor_speeds_vector[i] / total_speed;
        motor_power_limits_vector[i] = ratio * total_power_limit;
    }

    return motor_power_limits_vector;
}

//基于电机pid的error值来分配功率
//且在总error值较低时采取均分功率的策略
//你需要先创建一个4个float大小的vector，在里面按顺序填上你的电机速度，然后把这个填入函数形参，再把总功率上限填入函数形参
//之后创建一个4个double大小的vector，用于接收函数返回的分配好的四个电机功率，顺序和你填入的相同
//example:
//std::vector<double> motor_error_vector = {你pid的error1, error2, error3, error4};
//double max_power_limit = 150.0 //你的设定功率上限
//std::vector<double> motor_limit_power_vector = power_allocation_by_error(motor_error_vector, max_power_limit);//用来接收,顺序和你填入电机error的一致
std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, const double total_power_limit) {

    //查输入是否有效
    if (motor_errors_vector.size() != 4) {
        return {0.0, 0.0, 0.0, 0.0} ;
    }

    //对error取绝对值
    for (double& error : motor_errors_vector) {
        error = std::abs(error);
    }

    // 如果总功率上限为0，那么所有电机的功率上限都为0
    if (total_power_limit <= 1e-9) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    // 1. 计算总error
    const double total_error = std::accumulate(motor_errors_vector.begin(), motor_errors_vector.end(), 0.0);

    // 2. 处理总error过小的特殊情况
    // 此时引起总error过小的可能是因为底盘静止或者在斜坡上等原因
    // 这里选择平均分配总功率给每个电机
    if (total_error <= M_Too_Small_AllErrors) {
        double equal_share = total_power_limit / motor_errors_vector.size();
        return {equal_share, equal_share, equal_share, equal_share};
    }

    // 3. 按比例分配功率
    std::vector<double> motor_power_limits_vector(4);
    for (int i = 0; i < 4; ++i) {
        const double ratio = motor_errors_vector[i] / total_error;
        motor_power_limits_vector[i] = ratio * total_power_limit;
    }

    return motor_power_limits_vector;
}

//计算衰减系数，输入 想要发送的电流，这一时刻的速度，这个电机的最大分配功率，返回一个计算后的衰减系数
//用这个衰减系数乘输出电流后更新给电机就会使电机消耗的功率限制在你设定的功率下（以模型计算出来的功率为参照物）
double calculate_attenuation(E_Motor_PowerModel_Type motor_type, double desired_current, double current_speed, const double power_limit) {
    //在调用这个函数时要把想要发送的电流（直接发给电机的数）除1000.0才会算出准确值！！！！！！！！！！！！！！
    //近似认为电机正反转所有参数高度对称，所以加上绝对值
    double real_desired_current = std::abs(get_real_current(desired_current));
    double real_current_speed = std::abs(current_speed);

    //未超上限不衰减
    if (const double predicted_power = cal_motor_power_by_model(motor_type, desired_current, current_speed);
       predicted_power <= power_limit) {
        return 1.0;
    }

    //超上限了，则带入当前速度w，最大功率限制Pmax 于预测模型，解关于 I(衰减后) 的方程得到 I(衰减后)
    //再根据I衰减后 =衰减系数k * I原本想要发送值  解出衰减系数k，取值于0.0-1.0间

    const double a = M_3508_K4 * real_desired_current * real_desired_current;
    const double b = (M_3508_K1 + M_3508_K3 * real_current_speed) * real_desired_current;
    const double c = M_3508_K0 + M_3508_K2 * real_current_speed + M_3508_K5 * real_current_speed * real_current_speed - power_limit;
    const double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return 0.0;
    }

    const double k = (-b + std::sqrt(discriminant)) / (2 * a);

    if (k < 0.0) return 0.0;
    if (k > 1.0) return 1.0;
    return k;

}

//应用一阶低通滤波求一个平滑的功率曲线，系数alpha越大越平滑
void applyLowPassFilter(double& value, const double new_value, const double alpha) {
    value = alpha * new_value + (1.0 - alpha) * value;
}

//平均数滑动滤波，输出一个经过多少次平均后得到的值，size越大越平滑
MovingAverageFilter::MovingAverageFilter(const size_t size)
    : size(size), index(0), count(0), sum(0.0) {
    buffer.resize(size, 0.0);
}

double MovingAverageFilter::update(const double new_value) {
    sum -= buffer[index];
    buffer[index] = new_value;
    sum += new_value;
    index = (index + 1) % size;
    if (count < size) count++;
    return sum / count;
}




