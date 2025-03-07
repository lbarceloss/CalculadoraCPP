#pragma once
#include "SmartData.h"  // Supondo que SmartData ser� definido separadamente

struct Data {
    double power;
    double desvio;
    double power_range;
    SmartData smart_data;

    // Construtor
    Data(double power, double desvio, double power_range, const SmartData& smart_data)
        : power(power), desvio(desvio), power_range(power_range), smart_data(smart_data) {
    }
};
