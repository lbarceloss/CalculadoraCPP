#pragma once
#include <string>
#include <unordered_map>

// Enum para tipos de tiro
enum class ShotTypeEnum {
    DUNK,
    TOMAHAWK,
    SPIKE,
    COBRA
};

// Enum para os diferentes níveis de power shot
enum class PowerShotFactoryEnum {
    NO_POWER_SHOT = 0,
    ONE_POWER_SHOT = 1,
    TWO_POWER_SHOT = 2,
    ITEM_15_POWER_SHOT = 3
};

// Classe estática equivalente a PowerShotFactory
struct PowerShotFactory {
    // Método estático que retorna o valor correspondente ao tipo de power shot
    static int GetPowerShotFactory(int powershot) {
        switch (powershot) {
        case static_cast<int>(PowerShotFactoryEnum::ONE_POWER_SHOT):
            return 10;
        case static_cast<int>(PowerShotFactoryEnum::TWO_POWER_SHOT):
            return 20;
        case static_cast<int>(PowerShotFactoryEnum::ITEM_15_POWER_SHOT):
            return 15;
        default:
            return 0;
        }
    }

    // Retorna o valor inteiro correspondente a um tipo de tiro
    static int GetIntValueFromShotType(ShotTypeEnum type) {
        return static_cast<int>(type);
    }

    // Usa um mapa para buscar rapidamente valores correspondentes a strings
    static int GetIntValueFromShotTypePassingAString(const std::string& type) {
        static const std::unordered_map<std::string, int> shotTypeMap = {
            {"DUNK", 0},
            {"TOMAHAWK", 1},
            {"SPIKE", 2},
            {"COBRA", 3}
        };

        auto it = shotTypeMap.find(type);
        return (it != shotTypeMap.end()) ? it->second : 0;
    }
};
