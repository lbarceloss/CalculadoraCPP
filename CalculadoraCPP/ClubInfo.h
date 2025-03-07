#pragma once

// Enum para os tipos de taco
enum class ClubType {
    WOOD,
    IRON,
    SW,
    PW,
    PT
};

// Enum para os estados de distância (supondo que era definido em DistanceState)
enum class DistanceState {
    LESS_10,
    LESS_15,
    LESS_28,
    LESS_58,
    BIGGER_OR_EQUAL_58
};

class ClubInfo {
public:
    ClubType type;
    DistanceState type_distance = DistanceState::BIGGER_OR_EQUAL_58;
    double rotation_spin;
    double rotation_curve;
    double power_factor;
    double degree;
    double power_base;

    // Construtor
    ClubInfo(ClubType type, double rotation_spin, double rotation_curve, double power_factor, double degree, double power_base)
        : type(type), rotation_spin(rotation_spin), rotation_curve(rotation_curve),
        power_factor(power_factor), degree(degree), power_base(power_base) {
    }
};
