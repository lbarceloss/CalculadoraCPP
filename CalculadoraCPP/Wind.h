#pragma once
#include <cmath>
#include "Vector3D.h"

class Wind {
public:
    double wind = 0.0;
    double degree = 0.0;

    // Calcula o vetor de vento baseado no ângulo e intensidade do vento
    Vector3D GetWind() const {
        return Vector3D(
            wind * std::sin(degree * M_PI / 180.0) * -1.0,
            0.0,
            wind * std::cos(degree * M_PI / 180.0)
        );
    }
};
