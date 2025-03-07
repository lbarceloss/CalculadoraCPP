#pragma once

// Enum para estados de distância
enum class DistanceState {
    LESS_10,
    LESS_15,
    LESS_28,
    LESS_58,
    BIGGER_OR_EQUAL_58
};

// Classe estática para cálculo do estado de distância
struct TypeDistance {
    static DistanceState CalculateTypeDistance(double distance) {
        if (distance >= 58.0) {
            return DistanceState::BIGGER_OR_EQUAL_58;
        }
        else if (distance < 10.0) {
            return DistanceState::LESS_10;
        }
        else if (distance < 15.0) {
            return DistanceState::LESS_15;
        }
        else if (distance < 28.0) {
            return DistanceState::LESS_28;
        }
        else {
            return DistanceState::LESS_58;
        }
    }
};
