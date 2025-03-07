#pragma once
#include <cmath>
#include <unordered_map>
#include <string>
#include <any>
#include "Vector3D.h"
#include "Ball.h"
#include "Club.h"
#include "Wind.h"
#include "Options.h"
#include "QuadTree.h"
#include "Constraints.h"

struct Utils {
    static double DiffYZ(const Vector3D& v1, const Vector3D& v2) {
        return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.z - v2.z, 2));
    }

    static std::unordered_map<std::string, std::any> FindPower(std::any power_player, const ClubInfo& club_info, int shot, int power_shot, double distance, double height, const Wind& wind, double angle, double ground, double spin, double curve, std::any slope, double aim, double percent) {
        double heigth_colision = height * 1.094 * 3.2;
        double distance_scale = distance * 3.2;

        Ball vball;
        Club vclub = Constraints::club;

        if (!club_info.type) {
            vclub.Init(club_info);
        }

        vclub.distance_state = TypeDistance::CalculateTypeDistance(distance_scale);

        double slope_aim_rad = 0.0;

        if (slope.type() == typeid(Vector3D)) {
            Vector3D _slope = std::any_cast<Vector3D>(slope);
            vball.slope = _slope;
            slope_aim_rad = _slope.y;
            vball.slope.y = 1.0;
        }
        else {
            vball.slope = Vector3D(std::any_cast<double>(slope) * Constraints::slope_break_to_curve_slope * -1, 1.0, 0.0);
        }

        std::unordered_map<std::string, std::any> found;
        found["power"] = -1;
        found["desvio"] = 0.0;

        Constraints::wind = wind;
        Constraints::wind.degree = angle;

        Options options = SetUpOptions(power_player, distance, percent, ground, aim, slope_aim_rad, spin, curve, shot, power_shot);

        double power_range = vclub.GetRange(options, options.PowerObj->Pwr, options.Ps);
        double margin = 0.05;

        QuadTree qt;

        double limit_checking = 1000;
        int count = 0;
        bool is_find = false;
        double ret;
        int side = 0;
        double feed = 0.00006;

        do {
            if (options.PercentShot > 1.3)
                options.PercentShot = 1.3;
            else if (options.PercentShot < 0.0)
                options.PercentShot = 0.1;

            qt.InitShot(vball, vclub, wind, options);
            ret = FindColisionHeight(vball, margin, distance_scale, qt, heigth_colision);

            if (ret == 0.0)
                is_find = true;
            else {
                if (options.PercentShot == 1.3 && ret > 0)
                    break;
                if (options.PercentShot == 0.1 && ret < 0)
                    break;
                if (side == 0)
                    side = (ret < 0 ? -1 : 1);
                else if ((ret < 0 && side == 1) || (ret > 0 && side == -1))
                    feed *= 0.5;

                options.PercentShot += ret * feed;
            }

        } while (!is_find && (count++) < limit_checking);

        if (is_find) {
            double _desvio = (vball.position.x + (options.Position.x + (std::tan(options.MiraRad) * distance_scale))) * Constraints::DESVIO_SCALE_PANGYA_TO_YARD;

            Data data = Data(
                options.PercentShot,
                _desvio,
                power_range,
                SmartData(
                    _desvio,
                    heigth_colision,
                    vclub,
                    options
                )
            );

            found["power"] = data.power;
            found["desvio"] = data.desvio;
            found["power_range"] = data.power_range;
            found["smart_data"] = data.smart_data;
        }

        return found;
    }

private:
    static double FindColisionHeight(Ball& vball, double margin, double distance_scale, QuadTree& quad_t, double colision_height) {
        int counter = 0;
        Ball copy_ball;

        do {
            copy_ball.Copy(vball);
            quad_t.BallProcess(Constraints::_00D083A0);
        } while ((vball.position.y > colision_height || vball.num_max_height == -1) && (counter++) < 3000);

        double last_step = std::abs((colision_height - copy_ball.position.y) / (vball.position.y - copy_ball.position.y));

        vball.Copy(copy_ball);
        quad_t.BallProcess(Constraints::_00D083A0, Constraints::_00D083A0 * last_step);

        if (std::abs(distance_scale - vball.position.z) <= margin)
            return 0.0;

        return distance_scale - vball.position.z;
    }

    static double DoubleUnderThirty(double number) {
        return number / 30.0;
    }

    static Options SetUpOptions(std::any power_player, double distance, double percent, double ground, double aim, double slope_aim_rad, double spin, double curve, int shot, int power_shot) {
        bool is_integer = false;

        if (power_player.type() == typeid(int)) {
            is_integer = true;
        }

        if (is_integer) {
            return Options(
                distance,
                percent,
                ground,
                aim,
                slope_aim_rad,
                DoubleUnderThirty(spin),
                DoubleUnderThirty(curve),
                Vector3D(0.0, 0.0, 0.0),
                shot,
                power_shot,
                std::make_unique<Power>(std::any_cast<int>(power_player), std::make_unique<PowerOptions>(
                    0, 4, 4, 0, 0, 8
                ))
            );
        }
        else {
            return std::any_cast<Options>(power_player);
        }
    }
};
