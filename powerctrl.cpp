//
// Created by 3545 on 25-9-23.
//

#include "powerctrl.h"


double get_real_current(double current) {
    const double real_current = current/M_RealCurrent_Conversion;
    return real_current;
}

double cal_motor_power_by_model(E_Motor_PowerModel_Type motor_type ,double current, double speed,E_CalMotorPower_Negative_Status_Type Negative_Status) {

    double product = current*speed;
    double power_sign = 1;

    if (Negative_Status == E_enable_negative) {
            if(product < 0 ) {
            power_sign = -1;
        }
    }

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

std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, double total_power_limit) {

    #ifdef M_SmallGyro_Power_Compensation_Alpha
    total_power_limit *= (1-M_SmallGyro_Power_Compensation_Alpha);
    #endif

    if (motor_errors_vector.size() != 4) {
        return {0.0, 0.0, 0.0, 0.0} ;
    }

    for (double& error : motor_errors_vector) {
        error = std::abs(error);
    }

    if (total_power_limit <= 1e-9) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    const double total_error = motor_errors_vector[0]+motor_errors_vector[1]+motor_errors_vector[2]+motor_errors_vector[3];

    if (total_error <= M_Too_Small_AllErrors) {
        double equal_share = total_power_limit / motor_errors_vector.size();
        return {equal_share, equal_share, equal_share, equal_share};
    }

    std::vector<double> motor_power_limits_vector(4);
    if(total_power_limit < M_Motor_ReservedPower_Border) {
        for (int i = 0; i < 4; ++i) {
            const double ratio = motor_errors_vector[i] / total_error;
            motor_power_limits_vector[i] = ratio * total_power_limit;
        }
    }else {
        for (int j = 0; j < 4; ++j) {
            const double ratio = motor_errors_vector[j] / (total_error - 4*M_PerMotor_ReservedPower);
            motor_power_limits_vector[j] = ratio * (total_power_limit-4 * M_PerMotor_ReservedPower) + M_PerMotor_ReservedPower;
        }
    }

    return motor_power_limits_vector;
}

double calculate_attenuation(E_Motor_PowerModel_Type motor_type, double desired_current, double current_speed, const double power_limit) {

    if(power_limit < 0) {
        return 0.0;
    }

    double real_desired_current = std::abs(get_real_current(desired_current));
    double real_current_speed = std::abs(current_speed);

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

    const double a = motor_k4 * real_desired_current * real_desired_current;
    const double b = (motor_k1 + motor_k3 * real_current_speed) * real_desired_current;
    const double c = motor_k0 + motor_k2 * real_current_speed + motor_k5 * real_current_speed * real_current_speed - power_limit;
    const double discriminant = b * b - 4 * a * c;

    if (std::abs(a) < 1e-9) {
        if (std::abs(b) < 1e-9) {
            return (c <= 1e-9) ? 1.0 : 0.0;
        } else {
            double k = -c / b;
            if (k < 0.0) return 0.0;
            if (k > 1.0) return 1.0;
            return k;
        }
    }

    if (discriminant < 0) {
        return 0.0; ;
    }

    if (std::abs(discriminant) < 1e-9) {
        double k = -1.0 * b / (2 * a);
        if (k < 0.0) return 0.0;
        if (k > 1.0) return 1.0;
        return k;
    }

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

void applyLowPassFilter(double& value, const double new_value, const double alpha) {
    value = alpha * new_value + (1.0 - alpha) * value;
}

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
