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
//如果你想测出即将发送的功率来进行功率控制，请填入发送电流
//反之，如果你只是想简单预测一下电机组消耗的功率，请填入反馈电流
//注意：在负载很小且高速度时（比如底盘架起来让电机高速转）使用此模型可能会不准（表现为多预测5%-10%左右）推测这一点和模型采样未涉及有关
double cal_motor_power_by_model(E_Motor_PowerModel_Type motor_type ,double current, double speed,E_CalMotorPower_Negative_Status_Type Negative_Status) {

    //额外算一下速度和电流异号情况
    double product = current*speed;
    double power_sign = 1;
    //取消下面注释则启用正负功率计算
    //但是正负功率的计算仅适用于对电机反馈的电流计算，而不能对给电机发送的电流算！！！
    //所以一般没什么用
    //使用：根据调用函数第四个参数来确定开启或关闭
    if (Negative_Status == E_enable_negative) {
            if(product < 0 ) {
            power_sign = -1;
        }
    }

    //近似认为电机正反转所有参数高度对称，所以加上绝对值
    current = std::abs(get_real_current(current));
    speed = std::abs(speed);

    switch (motor_type) {
    case M3508_powermodel:
        return (M_3508_K0 +
               M_3508_K1 * current +
               M_3508_K2 * speed +
               M_3508_K3 * current * speed +
               M_3508_K4 * current * current +
               M_3508_K5 * speed * speed)*power_sign;
    case GM6020_powermodel:
        return (M_6020_K0 +
               M_6020_K1 * current +
               M_6020_K2 * speed +
               M_6020_K3 * current * speed +
               M_6020_K4 * current * current +
               M_6020_K5 * speed * speed)*power_sign;
    default:
        return 0.0;
    }

}

//基于电机pid的error值来分配功率
//且在总error值较低时采取均分功率的策略
//你需要先创建一个4个float大小的vector，在里面按顺序填上你的电机速度，然后把这个填入函数形参，再把总功率上限填入函数形参
//之后创建一个4个double大小的vector，用于接收函数返回的分配好的四个电机功率，顺序和你填入的相同
std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, double total_power_limit) {

    //经测试 在根据error分配总功率的情况下 当小陀螺或速度剧烈变化时 给轮向电机发送的电流值剧烈变化，衰减系数无解，只能发送0电流
    //而即使发送0电流也会继续有功率产生（具体原因见calculate_attenuation函数的说明）
    //增加数值约功率上限的5%-10%占比,貌似总功率越大增加的数值占比越小
    //所以最好读取 裁判系统缓冲功率 或 超级电容功率 来进行功率闭环
    //如果没有以上操作可以选择牺牲一点功率上限换取稳定不超限（即下面define内的内容）
    //此选项通过在powerctrl.h中取消注释define来启用，同时也在那里更改SmallGyro_Power_Compensation_Alpha的值
    #ifdef SmallGyro_Power_Compensation
    total_power_limit *= (1-SmallGyro_Power_Compensation_Alpha);
    #endif

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
    const double total_error = motor_errors_vector[0]+motor_errors_vector[1]+motor_errors_vector[2]+motor_errors_vector[3];

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

    if(power_limit < 0) {
        return 0.0;
    }

    //在调用这个函数时要把想要发送的电流（直接发给电机的数）除1000.0才会算出准确值！！！！！！！！！！！！！！
    //近似认为电机正反转所有参数高度对称，所以加上绝对值
    double real_desired_current = std::abs(get_real_current(desired_current));
    double real_current_speed = std::abs(current_speed);

    // 未超上限不衰减（不包含负功率的情况，因为近似认定只要给他发送电流均为耗电电机,所以不使能正负）
    if (const double predicted_power = cal_motor_power_by_model(motor_type, desired_current, current_speed);
       predicted_power <= power_limit) {
        return 1.0;
       }

    double motor_k0,motor_k1,motor_k2,motor_k3,motor_k4,motor_k5;
    if(motor_type == M3508_powermodel) {
        motor_k0 = M_3508_K0;
        motor_k1 = M_3508_K1;
        motor_k2 = M_3508_K2;
        motor_k3 = M_3508_K3;
        motor_k4 = M_3508_K4;
        motor_k5 = M_3508_K5;
    }else if (motor_type == GM6020_powermodel) {
        motor_k0 = M_6020_K0;
        motor_k1 = M_6020_K1;
        motor_k2 = M_6020_K2;
        motor_k3 = M_6020_K3;
        motor_k4 = M_6020_K4;
        motor_k5 = M_6020_K5;
    }else {
        return 0.0;
    }

    //超上限了，则带入当前速度w，最大功率限制Pmax 于预测模型，解关于 I(衰减后) 的方程得到 I(衰减后)
    //再根据I衰减后 =衰减系数k * I原本想要发送值  解出衰减系数k，取值于0.0-1.0间
    const double a = motor_k4 * real_desired_current * real_desired_current;
    const double b = (motor_k1 + motor_k3 * real_current_speed) * real_desired_current;
    const double c = motor_k0 + motor_k2 * real_current_speed + motor_k5 * real_current_speed * real_current_speed - power_limit;
    const double discriminant = b * b - 4 * a * c;

    //a或b为0（电流为0）的情况
    if (std::abs(a) < 1e-9) {
        if (std::abs(b) < 1e-9) {
            // 常数项方程：c <= 0 时无需衰减
            return (c <= 1e-9) ? 1.0 : 0.0;
        } else {
            // 一次方程：k = -c / b
            double k = -c / b;
            if (k < 0.0) return 0.0;
            if (k > 1.0) return 1.0;
            return k;
        }
    }

    //判别式小于0，说明方程无解
    //如果方程无解，那么说明电流无论给什么值也不能在限定功率下正常工作（就是由于限定功率太小，导致电流给什么值都会超功率）
    //所以当根据error分配功率时，在轮组速度急剧变化（尤其是小陀螺平移的时候）某个电机被分配到了很小的功率 所以即使这个情况返回了0.0的衰减系数 使发送电流为0
    //那么还是会超功率的（电机空转时发送电流为0但由于有速度存在，还是会消耗功率）
    //有3种解决方法：1，在功率底盘总功率分配时少分配一点作为保险（比如比赛限定功率75w，那么只分配70w，这样会安全一点，但也有可能瞬间超）
    //            2.根据超级电容或者底盘缓冲功率做一个闭环：如果检测到这两个功率短时间内大量消耗，那么就减小一点总限定功率
    //       todo:3.当检测到这种情况时 使出现问题的电机不采用error分配，强行把发送电流为0时的功率分配进去（我很建议这种方案，但是还没有写，准备要去写这个）
    //       todo:4.和3类似，分配功率时 给每个轮子预留最小消耗功率 （从而防止分配时无视这部分功率而分配给别的电机导致最终超出这部分功率）
    if (discriminant < 0) {
        return 0.0; ;
    }

    //两个相等的根
    if (std::abs(discriminant) < 1e-9) {
        double k = -1.0 * b / (2 * a);
        if (k < 0.0) return 0.0;
        if (k > 1.0) return 1.0;
        return k;
    }

    //其余情况只剩两个不等根，优先取1,0内最大根
    //同样存在：当使用error分配时，在k1k2均不存在于0-1内的时候会超功率
    //具体原因和解决办法见上面判别式为0的注释
    double k1 = (-b - std::sqrt(discriminant)) / (2 * a);
    double k2 = (-b + std::sqrt(discriminant)) / (2 * a);
    if ( (k1 > 0.0 and k1 < 1.0 ) and (k2 > 0.0 and k2 < 1.0) ) {
        return std::max(k1, k2);
    }else if ( (k1 > 0.0 and k1 < 1.0) and(k2 > 1.0 or k2 < 0.0) ) {
        return k1;
    }else if ( (k1 < 0.0 or k1 > 1.0 ) and(k2 > 0.0 and k2 < 1.0) ) {
        return k2;
    }else {
        return 0.0;
    }

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
