#pragma once

#include <cmath>  // Para M_PI
#include "ClubInfo.h"
#include "Options.h"
#include "PowerShotFactory.h"
#include "Constraints.h"
#include "DistanceState.h"

// Enum para os tipos de taco (supondo que será definido depois)
enum class ClubType { WOOD, IRON, SW, PW, PT };

// Enum para os estados de distância (supondo que será definido depois)
enum class DistanceState { LESS_10, LESS_15, LESS_28, LESS_58, BIGGER_OR_EQUAL_58 };

class Club {
public:
    ClubType type = ClubType::WOOD;
    DistanceState distance_state = DistanceState::BIGGER_OR_EQUAL_58;

    double rotation_spin = 0.55;
    double rotation_curve = 1.61;
    double power_factor = 236;
    double degree = 10;
    double power_base = 230;

    // Método para inicializar com os valores de ClubInfo
    void Init(const ClubInfo& club_info) {
        type = club_info.type;
        rotation_spin = club_info.rotation_spin;
        rotation_curve = club_info.rotation_curve;
        power_factor = club_info.power_factor;
        degree = club_info.degree;
        power_base = club_info.power_base;
    }

    // Converte o ângulo de graus para radianos
    double GetDregRad() const {
        return degree * M_PI / 180.0;
    }

    // Calcula a potência do taco com base em diferentes fatores
    double GetPower(const Options& extraPower, int pwrSlot, int ps, double spin) {
        double pwrjard = 0.0;

        switch (type) {
        case ClubType::WOOD:
            pwrjard = extraPower.Power.Options.Total(ps) + PowerShotFactory::GetPowerShotFactory(ps) + ((pwrSlot - 15) * 2);
            pwrjard *= 1.5;
            pwrjard /= power_base;
            pwrjard += 1;
            pwrjard *= power_factor;
            break;

        case ClubType::IRON:
            pwrjard = ((PowerShotFactory::GetPowerShotFactory(ps) / power_base + 1.0) * power_factor) +
                (extraPower.Power.Options.Total(ps) * power_factor * 1.3) / power_base;
            break;

        case ClubType::SW:
        case ClubType::PW: {
            auto getPowerByDegree = [](double degree, int _spin) {
                return 0.5 + ((0.5 * (degree + (_spin * Constraints::_00D19B98))) / (56.0 / 180.0 * M_PI));
                };

            switch (distance_state) {
            case DistanceState::LESS_10:
            case DistanceState::LESS_15:
            case DistanceState::LESS_28:
                pwrjard = (getPowerByDegree(GetDregRad(), static_cast<int>(spin)) * (52.0 + (ps > 0 ? 28.0 : 0))) +
                    (extraPower.Power.Options.Total(ps) * power_factor) / power_base;
                break;
            case DistanceState::LESS_58:
                pwrjard = (getPowerByDegree(GetDregRad(), static_cast<int>(spin)) * (80.0 + (ps > 0 ? 18.0 : 0))) +
                    (extraPower.Power.Options.Total(ps) * power_factor) / power_base;
                break;
            case DistanceState::BIGGER_OR_EQUAL_58:
                pwrjard = ((PowerShotFactory::GetPowerShotFactory(ps) / power_base + 1.0) * power_factor) +
                    (extraPower.Power.Options.Total(ps) * power_factor) / power_base;
                break;
            }
            break;
        }
        case ClubType::PT:
            pwrjard = power_factor;
            break;
        }

        return pwrjard;
    }

    // Método GetPower2 convertido para C++
    double GetPower2(const Options& extraPower, int pwrSlot, int ps) {
        double pwrjard = (extraPower.Power.Options.Auxpart + extraPower.Power.Options.Mascot + extraPower.Power.Options.Card) / 2 + (pwrSlot - 15);

        if (ps >= 0)
            pwrjard += (extraPower.Power.Options.PsCard / 2);

        pwrjard /= 170;

        return pwrjard + 1.5;
    }

    // Calcula o alcance do taco dependendo do tipo e distância
    double GetRange(const Options& extraPower, double pwrSlot, int ps) {
        double pwr_range = power_base + extraPower.Power.Options.Total(ps) + PowerShotFactory::GetPowerShotFactory(ps);

        if (type == ClubType::WOOD)
            pwr_range += (pwrSlot - 15) * 2;

        if (type == ClubType::PW) {
            switch (distance_state) {
            case DistanceState::LESS_10:
            case DistanceState::LESS_15:
            case DistanceState::LESS_28:
                pwr_range = 30.0 + (ps != 0 ? 30.0 : 0.0) + extraPower.Power.Options.Total(ps);
                break;
            case DistanceState::LESS_58:
                pwr_range = 60.0 + (ps != 0 ? 20.0 : 0.0) + extraPower.Power.Options.Total(ps);
                break;
            case DistanceState::BIGGER_OR_EQUAL_58:
                pwr_range = power_base + extraPower.Power.Options.Total(ps) + PowerShotFactory::GetPowerShotFactory(ps);
                break;
            }
        }

        return pwr_range;
    }
};
