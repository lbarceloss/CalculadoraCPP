#pragma once
#include <cmath>
#include <unordered_map>
#include <string>
#include "Ball.h"
#include "Club.h"
#include "Wind.h"
#include "Options.h"
#include "PowerShotFactory.h"
#include "Constraints.h"
#include "Vector3D.h"

class QuadTree {
public:
    Ball ball;
    Club club;
    Wind wind;

    int gravity_factor = 1;
    double gravity = 34.295295715332; // Gravidade em Yards (escala Pangya)

    Vector3D _21D8_vect = Vector3D(0.0, 0.0, 0.0);
    Vector3D ball_position_init = Vector3D(0.0, 0.0, 0.0);

    double power_range_shot = 0.0;
    int shot = PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::DUNK);
    double power_factor_shot = 0.0;
    double percentShot_sqrt = 0.0;
    double spike_init = -1;
    double spike_med = -1;
    double power_factor = 0.0;
    double cobra_init = -1;

    QuadTree() : club(), ball(), wind() {}

    double GetGravity() const {
        return gravity * gravity_factor;
    }

    void InitShot(Ball& ball, Club& club, Wind& wind, Options& options) {
        this->ball = ball;
        this->club = club;
        this->wind = wind;

        shot = options.Shot;
        spike_init = -1;
        spike_med = -1;
        cobra_init = -1;

        this->ball.position = options.Position;
        ball_position_init = options.Position;

        club.distance_state = TypeDistance::CalculateTypeDistance(options.Distance);

        ball.max_height = ball.position.y;
        ball.count = 0;
        ball.num_max_height = -1;

        double pwr = club.GetPower(options, options.PowerObj->Pwr, options.Ps, options.Spin);
        power_range_shot = club.GetRange(options, options.PowerObj->Pwr, options.Ps);
        power_factor = pwr;

        pwr *= std::sqrt(options.PercentShot);

        if (options.Shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::TOMAHAWK) ||
            options.Shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE)) {
            pwr *= 1.3;
        }
        else {
            pwr *= 1.0;
        }

        pwr *= std::sqrt(options.Ground * 0.01);
        power_factor_shot = pwr;
        percentShot_sqrt = std::sqrt(options.PercentShot);

        ball.curve = options.Curve;
        ball.spin = options.Spin;

        std::unordered_map<std::string, double> value1 = GetValuesDegree(options.MiraRad + (0 - (ball.curve * Constraints::_00D17908)), 1);
        std::unordered_map<std::string, double> value2 = GetValuesDegree(club.distance_state == DistanceState::BIGGER_OR_EQUAL_58 ? club.GetDregRad() : club.GetDregRad() + (ball.spin * Constraints::_00D19B98), 0);

        ball.curve -= GetSlope(options.MiraRad - options.SlopeMiraRad, static_cast<double>(std::rand()) / RAND_MAX);
        pwr *= ((std::abs(ball.curve) * 0.1) + 1);

        Vector3D vectA(value2["neg_sin"], value2["neg_rad"], value2["cos2"]);
        vectA.MultiplyScalar(pwr);

        Vector3D v1(value1["cos"], value1["rad"], value1["sin"]);
        Vector3D v2(value1["_C"], value1["_10"], value1["_14"]);
        Vector3D v3(value1["neg_sin"], value1["neg_rad"], value1["cos2"]);
        Vector3D v4(value1["_24"], value1["_28"], value1["_2C"]);

        ball.velocity.x = v2.x * vectA.y + vectA.x * v1.x + v3.x * vectA.z + v4.x;
        ball.velocity.y = v1.y * vectA.x + v2.y * vectA.y + v3.y * vectA.z + v4.y;
        ball.velocity.z = v1.z * vectA.x + v2.z * vectA.y + v3.z * vectA.z + v4.z;

        ball.rotation_curve = ball.curve * options.PercentShot;
        ball.rotation_spin = club.distance_state == DistanceState::BIGGER_OR_EQUAL_58
            ? (club.GetPower2(options, options.PowerObj->Pwr, options.Ps) * options.PercentShot) * options.PercentShot
            : 0.0;

        ball.ball_48 = ball.ball_44;
    }

    std::unordered_map<std::string, double> GetValuesDegree(double degree, int option = 0) {
        std::unordered_map<std::string, double> obj;

        if (option == 0) {
            obj["cos"] = 1.0;
            obj["rad"] = 0.0;
            obj["sin"] = 0.0;
            obj["_C"] = 0.0;
            obj["_10"] = std::cos(degree);
            obj["_14"] = std::sin(degree) * -1;
            obj["neg_sin"] = 0.0;
            obj["neg_rad"] = std::sin(degree);
            obj["cos2"] = obj["_10"];
            obj["_24"] = 0.0;
            obj["_28"] = 0.0;
            obj["_2C"] = 0.0;
        }
        else if (option == 1) {
            obj["cos"] = std::cos(degree);
            obj["rad"] = 0.0;
            obj["sin"] = std::sin(degree);
            obj["_C"] = 0.0;
            obj["_10"] = 1.0;
            obj["_14"] = 0.0;
            obj["neg_sin"] = obj["sin"] * -1;
            obj["neg_rad"] = 0.0;
            obj["cos2"] = obj["cos"];
            obj["_24"] = 0.0;
            obj["_28"] = 0.0;
            obj["_2C"] = 0.0;
        }

        return obj;
    }

    double GetSlope(double aim, double lineBall) {
        Vector3D ball_slope_cross_const_vect = ball.slope.Clone().Normalize();

        std::unordered_map<std::string, Vector3D> slope_matrix;
        slope_matrix["v1"] = ball_slope_cross_const_vect.Clone().Normalize();
        slope_matrix["v2"] = ball.slope.Clone();
        slope_matrix["v3"] = ball_slope_cross_const_vect.Clone().Cross(ball.slope).Normalize();
        slope_matrix["v4"] = Vector3D(0.0, 0.0, 0.0);

        std::unordered_map<std::string, double> value1 = GetValuesDegree(aim * -1, 1);
        std::unordered_map<std::string, double> value2 = GetValuesDegree(lineBall * -2.0, 1);

        return slope_matrix["v2"].x * Constraints::_00D66CA0;
    }
};
