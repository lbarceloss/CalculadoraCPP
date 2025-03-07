#pragma once
#include <memory>
#include "PowerOptions.h"

class Power {
public:
    int Pwr;
    std::unique_ptr<PowerOptions> Options;

    // Construtor
    Power(int power, std::unique_ptr<PowerOptions> opt)
        : Pwr(power), Options(std::move(opt)) {
    }
};
