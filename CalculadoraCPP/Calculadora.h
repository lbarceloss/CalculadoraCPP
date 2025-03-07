#pragma once
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <cstdlib>   // for rand (if needed) or use <random>
#include <algorithm> // for std::max, etc.


namespace Calculadora {

    // Forward declarations (if needed)
    // ---------------------------------
    class Vector3D;
    class Ball;
    class Club;
    class Wind;
    class QuadTree;
    class Options;
    class Power;
    class PowerOptions;
    class SmartData;
    class ClubInfo;

    // ---------------------------------------------------------
    // Enum Definitions
    // ---------------------------------------------------------
    enum class ClubType {
        WOOD,
        IRON,
        SW,
        PW,
        PT
    };

    enum class DistanceState {
        LESS_10,
        LESS_15,
        LESS_28,
        LESS_58,
        BIGGER_OR_EQUAL_58
    };

    enum class ShotTypeEnum {
        DUNK,
        TOMAHAWK,
        SPIKE,
        COBRA
    };

    // In C# code, these were just class constants. We can make them static or constexpr in C++.
    // We'll replicate them all under a Constraints struct or namespace.
    // ---------------------------------------------------------
    struct Constraints {
        static constexpr double DESVIO_SCALE_PANGYA_TO_YARD = 0.3125 / 1.5;
        static constexpr double _00D3D008 = 0.00001;
        static constexpr double _00D046A8 = -1.0;
        static constexpr double _00D00190 = 0.75;
        static constexpr double _00D083A0 = 0.02;
        static constexpr double _00D66CF8 = 3.0;
        static constexpr double _00D3D028 = 0.00008;
        static constexpr double _00D1A888 = 12.566371;
        static constexpr double _00D3D210 = 25.132742;
        static constexpr double _00CFF040 = 0.1;
        static constexpr double _00D66CA0 = 0.5;
        static constexpr double _00D16928 = 0.002;
        static constexpr double _00D17908 = 0.349065847694874;
        static constexpr double _00D19B98 = 0.0698131695389748;
        static constexpr double _00D16758 = 0.01;
        static constexpr double slope_break_to_curve_slope = 0.00875;

        static constexpr double YARDS_TO_PB = 0.2167;
        static constexpr double YARDS_TO_PBA = 0.8668;
        static constexpr double YARDS_TO_PBA_PLUS = 1.032;
    };

    // ---------------------------------------------------------
    // Vector3D
    // ---------------------------------------------------------
    class Vector3D {
    public:
        double x, y, z;

        Vector3D(double xVal = 0.0, double yVal = 0.0, double zVal = 0.0)
            : x(xVal), y(yVal), z(zVal) {
        }

        Vector3D(const Vector3D& other) : x(other.x), y(other.y), z(other.z) {}

        Vector3D& Normalize() {
            double len = Length();
            if (len != 0.0) {
                x /= len;
                y /= len;
                z /= len;
            }
            return *this;
        }

        double Length() const {
            return std::sqrt(x * x + y * y + z * z);
        }

        Vector3D& MultiplyScalar(double value) {
            x *= value;
            y *= value;
            z *= value;
            return *this;
        }

        Vector3D& Add(const Vector3D& v) {
            x += v.x;
            y += v.y;
            z += v.z;
            return *this;
        }

        Vector3D& Sub(const Vector3D& v) {
            x -= v.x;
            y -= v.y;
            z -= v.z;
            return *this;
        }

        Vector3D& DivideScalar(double value) {
            if (value != 0.0) {
                x /= value;
                y /= value;
                z /= value;
            }
            else {
                x = y = z = 0.0;
            }
            return *this;
        }

        Vector3D& Cross(const Vector3D& v) {
            double tx = x, ty = y, tz = z;
            x = ty * v.z - tz * v.y;
            y = tz * v.x - tx * v.z;
            z = tx * v.y - ty * v.x;
            return *this;
        }

        Vector3D Clone() const {
            return Vector3D(x, y, z);
        }
    };

    // ---------------------------------------------------------
    // Power Shot definitions
    // ---------------------------------------------------------
    class PowerShotFactory {
    public:
        static const int NO_POWER_SHOT = 0;
        static const int ONE_POWER_SHOT = 1;
        static const int TWO_POWER_SHOT = 2;
        static const int ITEM_15_POWER_SHOT = 3;

        static int GetPowerShotFactory(int powershot) {
            int power_shot_factory = 0;
            switch (powershot) {
            case ONE_POWER_SHOT: power_shot_factory = 10; break;
            case TWO_POWER_SHOT: power_shot_factory = 20; break;
            case ITEM_15_POWER_SHOT: power_shot_factory = 15; break;
            default: power_shot_factory = 0; break;
            }
            return power_shot_factory;
        }

        static int GetIntValueFromShotType(ShotTypeEnum type) {
            switch (type) {
            case ShotTypeEnum::DUNK:      return 0;
            case ShotTypeEnum::TOMAHAWK:  return 1;
            case ShotTypeEnum::SPIKE:     return 2;
            case ShotTypeEnum::COBRA:     return 3;
            }
            return 0;
        }
    };

    // ---------------------------------------------------------
    // PowerOptions
    // ---------------------------------------------------------
    class PowerOptions {
    public:
        int Auxpart;
        int Mascot;
        int Card;
        int PsAuxpart;
        int PsMascot;
        int PsCard;

        PowerOptions(int auxpart = 0, int mascot = 0, int card = 0, int psAuxpart = 0, int psMascot = 0, int psCard = 0)
            : Auxpart(auxpart), Mascot(mascot), Card(card),
            PsAuxpart(psAuxpart), PsMascot(psMascot), PsCard(psCard)
        {
        }

        int Total(int option) {
            int pwr = Auxpart + Mascot + Card;
            if (option == 1 || option == 2 || option == 3) {
                pwr += (PsAuxpart + PsMascot + PsCard);
            }
            return pwr;
        }
    };

    // ---------------------------------------------------------
    // Power
    // ---------------------------------------------------------
    class Power {
    public:
        int Pwr;
        PowerOptions Options;

        Power(int power, const PowerOptions& opt)
            : Pwr(power), Options(opt) {
        }
    };

    // ---------------------------------------------------------
    // Ball
    // ---------------------------------------------------------
    class Ball {
    public:
        Vector3D position;
        Vector3D slope;
        int state_process;
        double max_height;
        int num_max_height;
        int count;
        Vector3D velocity;
        double ball_28;
        double ball_2C;
        double ball_30;
        double curve;
        double spin;
        double rotation_curve;
        double rotation_spin;
        int ball_44;
        int ball_48;
        int ball_70;
        int ball_90;
        int ball_BC;
        double mass;
        double diametro;

        Ball()
            : position(0.0, 0.0, 0.0),
            slope(0.0, 1.0, 0.0),
            state_process(0),
            max_height(0.0),
            num_max_height(-1),
            count(0),
            velocity(0.0, 0.0, 0.0),
            ball_28(0.0),
            ball_2C(0.0),
            ball_30(0.0),
            curve(0.0),
            spin(0.0),
            rotation_curve(0.0),
            rotation_spin(0.0),
            ball_44(0),
            ball_48(0),
            ball_70(-1),
            ball_90(0),
            ball_BC(0),
            mass(0.045926999),
            diametro(0.14698039)
        {
        }

        void Copy(const Ball& other) {
            position = other.position.Clone();
            slope = other.slope.Clone();
            velocity = other.velocity.Clone();
            state_process = other.state_process;
            max_height = other.max_height;
            spin = other.spin;
            curve = other.curve;
            count = other.count;
            num_max_height = other.num_max_height;
            ball_28 = other.ball_28;
            ball_2C = other.ball_2C;
            ball_30 = other.ball_30;
            ball_44 = other.ball_44;
            ball_48 = other.ball_48;
            ball_70 = other.ball_70;
            ball_90 = other.ball_90;
            ball_BC = other.ball_BC;
            rotation_curve = other.rotation_curve;
            rotation_spin = other.rotation_spin;
        }
    };

    // ---------------------------------------------------------
    // Wind
    // ---------------------------------------------------------
    class Wind {
    public:
        double wind;
        double degree;

        Wind() : wind(0.0), degree(0.0) {}

        Vector3D GetWind() {
            double rad = degree * M_PI / 180.0;
            double sinD = std::sin(rad);
            double cosD = std::cos(rad);

            // * -1 on the sin part as in code:
            // wind * Math.Sin(degree * Math.PI / 180) * -1
            return Vector3D(wind * sinD * -1, 0.0, wind * cosD);
        }
    };

    // ---------------------------------------------------------
    // ClubInfo
    // ---------------------------------------------------------
    class ClubInfo {
    public:
        ClubType type;
        DistanceState type_distance;
        double rotation_spin;
        double rotation_curve;
        double power_factor;
        double degree;
        double power_base;

        ClubInfo(
            ClubType t,
            double rotSpin,
            double rotCurve,
            double pwrFactor,
            double deg,
            double pwrBase
        ) : type(t),
            type_distance(DistanceState::BIGGER_OR_EQUAL_58),
            rotation_spin(rotSpin),
            rotation_curve(rotCurve),
            power_factor(pwrFactor),
            degree(deg),
            power_base(pwrBase)
        {
        }
    };

    // ---------------------------------------------------------
    // Club
    // ---------------------------------------------------------
    class Club {
    public:
        ClubType type;
        DistanceState distance_state;
        double rotation_spin;
        double rotation_curve;
        double power_factor;
        double degree;
        double power_base;

        Club()
            : type(ClubType::WOOD),
            distance_state(DistanceState::BIGGER_OR_EQUAL_58),
            rotation_spin(0.55),
            rotation_curve(1.61),
            power_factor(236),
            degree(10),
            power_base(230)
        {
        }

        void Init(const ClubInfo& club_info) {
            this->type = club_info.type;
            this->rotation_spin = club_info.rotation_spin;
            this->rotation_curve = club_info.rotation_curve;
            this->power_factor = club_info.power_factor;
            this->degree = club_info.degree;
            this->power_base = club_info.power_base;
        }

        double GetDregRad() {
            return this->degree * M_PI / 180.0;
        }

        double GetPower(
            const class Options& extraPower,
            int pwrSlot,
            int ps,
            double spin
        );

        double GetPower2(const class Options& extraPower, int pwrSlot, int ps);

        double GetRange(const class Options& extraPower, double pwrSlot, int ps);
    };

    // ---------------------------------------------------------
    // Options
    // ---------------------------------------------------------
    class Options {
    public:
        double Distance;
        double PercentShot;
        double Ground;
        double MiraRad;
        double SlopeMiraRad;
        double Spin;
        double Curve;
        Vector3D Position;
        int Shot;
        int Ps;
        Power PowerObj;

        Options(
            double distance,
            double percentShot,
            double ground,
            double miraRad,
            double slopeMiraRad,
            double spin,
            double curve,
            const Vector3D& position,
            int shot,
            int ps,
            const Power& power
        ) : Distance(distance),
            PercentShot(percentShot),
            Ground(ground),
            MiraRad(miraRad),
            SlopeMiraRad(slopeMiraRad),
            Spin(spin),
            Curve(curve),
            Position(position),
            Shot(shot),
            Ps(ps),
            PowerObj(power)
        {
        }
    };

    // ---------------------------------------------------------
    // Club method implementations (need Options defined)
    // ---------------------------------------------------------
    double Club::GetPower(const Options& extraPower, int pwrSlot, int ps, double spin) {
        double pwrjard = 0.0;

        switch (this->type) {
        case ClubType::WOOD:
        {
            pwrjard = extraPower.PowerObj.Options.Total(ps)
                + PowerShotFactory::GetPowerShotFactory(ps)
                + ((pwrSlot - 15) * 2);
            pwrjard *= 1.5;
            pwrjard /= this->power_base;
            pwrjard += 1.0;
            pwrjard *= this->power_factor;
            break;
        }
        case ClubType::IRON:
        {
            pwrjard = ((PowerShotFactory::GetPowerShotFactory(ps) / this->power_base + 1.0) * this->power_factor)
                + (extraPower.PowerObj.Options.Total(ps) * this->power_factor * 1.3) / this->power_base;
            break;
        }
        case ClubType::SW:
        case ClubType::PW:
        {
            // In the C# code, SW is treated the same as PW
            auto getPowerByDegree = [&](double deg, int _spin) {
                return 0.5 + ((0.5 * (deg + (_spin * Constraints::_00D19B98))) / ((56.0 / 180.0) * M_PI));
                };

            switch (this->distance_state) {
            case DistanceState::LESS_10:
            case DistanceState::LESS_15:
            case DistanceState::LESS_28:
                pwrjard = (getPowerByDegree(this->GetDregRad(), (int)spin)
                    * (52.0 + (ps > 0 ? 28.0 : 0.0)))
                    + (extraPower.PowerObj.Options.Total(ps) * this->power_factor) / this->power_base;
                break;
            case DistanceState::LESS_58:
                pwrjard = (getPowerByDegree(this->GetDregRad(), (int)spin)
                    * (80.0 + (ps > 0 ? 18.0 : 0.0)))
                    + (extraPower.PowerObj.Options.Total(ps) * this->power_factor) / this->power_base;
                break;
            case DistanceState::BIGGER_OR_EQUAL_58:
                pwrjard = ((PowerShotFactory::GetPowerShotFactory(ps) / this->power_base + 1.0) * this->power_factor)
                    + (extraPower.PowerObj.Options.Total(ps) * this->power_factor) / this->power_base;
                break;
            }
            break;
        }
        case ClubType::PT:
        {
            pwrjard = this->power_factor;
            break;
        }
        }
        return pwrjard;
    }

    double Club::GetPower2(const Options& extraPower, int pwrSlot, int ps) {
        double pwrjard = ((extraPower.PowerObj.Options.Auxpart
            + extraPower.PowerObj.Options.Mascot
            + extraPower.PowerObj.Options.Card) / 2.0)
            + (pwrSlot - 15);

        if (ps >= 0)
            pwrjard += (extraPower.PowerObj.Options.PsCard / 2.0);

        pwrjard /= 170.0;

        return pwrjard + 1.5;
    }

    double Club::GetRange(const Options& extraPower, double pwrSlot, int ps) {
        double pwr_range = this->power_base
            + extraPower.PowerObj.Options.Total(ps)
            + PowerShotFactory::GetPowerShotFactory(ps);

        if (this->type == ClubType::WOOD) {
            pwr_range += (pwrSlot - 15) * 2.0;
        }

        if (this->type == ClubType::PW) {
            switch (this->distance_state) {
            case DistanceState::LESS_10:
            case DistanceState::LESS_15:
            case DistanceState::LESS_28:
                pwr_range = 30.0 + (ps != 0 ? 30.0 : 0.0) + extraPower.PowerObj.Options.Total(ps);
                break;
            case DistanceState::LESS_58:
                pwr_range = 60.0 + (ps != 0 ? 20.0 : 0.0) + extraPower.PowerObj.Options.Total(ps);
                break;
            case DistanceState::BIGGER_OR_EQUAL_58:
                pwr_range = this->power_base
                    + extraPower.PowerObj.Options.Total(ps)
                    + PowerShotFactory::GetPowerShotFactory(ps);
                break;
            }
        }
        return pwr_range;
    }

    // ---------------------------------------------------------
    // Additional utility
    // ---------------------------------------------------------
    static DistanceState CalculateTypeDistance(double distance) {
        // replicate logic: if (distance >= 58) -> B, else ...
        if (distance >= 58.0)
            return DistanceState::BIGGER_OR_EQUAL_58;
        else if (distance < 10.0)
            return DistanceState::LESS_10;
        else if (distance < 15.0)
            return DistanceState::LESS_15;
        else if (distance < 28.0)
            return DistanceState::LESS_28;
        else if (distance < 58.0)
            return DistanceState::LESS_58;
        // default
        return DistanceState::LESS_10;
    }

    // ---------------------------------------------------------
    // SmartData
    // ---------------------------------------------------------
    class SmartData {
    public:
        double desvio;
        double height;
        Club club;
        Options options;

        SmartData(double d, double h, const Club& c, const Options& opt)
            : desvio(d), height(h), club(c), options(opt) {
        }
    };

    // ---------------------------------------------------------
    // Data container
    // ---------------------------------------------------------
    class Data {
    public:
        double power;
        double desvio;
        double power_range;
        SmartData smart_data;

        Data(double p, double d, double pr, const SmartData& sd)
            : power(p), desvio(d), power_range(pr), smart_data(sd) {
        }
    };

    // ---------------------------------------------------------
    // QuadTree
    // (Implements shot physics step by step)
    // ---------------------------------------------------------
    class QuadTree {
    public:
        Ball ball;
        Club club;
        Wind wind;
        int gravity_factor;
        double gravity;
        Vector3D _21D8_vect;
        Vector3D ball_position_init;
        double power_range_shot;
        int shot;
        double power_factor_shot;
        double percentShot_sqrt;
        double spike_init;
        double spike_med;
        double power_factor;
        double cobra_init;

        QuadTree()
            : gravity_factor(1),
            gravity(34.295295715332), // from code
            _21D8_vect(0.0, 0.0, 0.0),
            ball_position_init(0.0, 0.0, 0.0),
            power_range_shot(0.0),
            shot(0),
            power_factor_shot(0.0),
            percentShot_sqrt(0.0),
            spike_init(-1.0),
            spike_med(-1.0),
            power_factor(0.0),
            cobra_init(-1.0)
        {
        }

        double GetGravity() {
            return this->gravity * this->gravity_factor;
        }

        void InitShot(Ball& ballRef, Club& clubRef, Wind& windRef, const Options& options);
        std::map<std::string, double> GetValuesDegree(double degree, int option = 0);
        double GetSlope(double aim, double lineBall);
        std::map<std::string, Vector3D> ValuesDegreesToMatrix(const std::map<std::string, double>& dictionary);
        std::map<std::string, Vector3D> ApplyMatrix(const std::map<std::string, Vector3D>& m1, const std::map<std::string, Vector3D>& m2);
        void BallProcess(double steptime, double finalVal = -1.0);
        void BounceProcess(double steptime, double finalVal);
        Vector3D ApplyForce();
    };

    // Implementation
    // ---------------------------------------------------------
    void QuadTree::InitShot(Ball& ballRef, Club& clubRef, Wind& windRef, const Options& options) {
        this->ball = ballRef;
        this->club = clubRef;
        this->wind = windRef;

        this->shot = options.Shot;
        this->spike_init = -1.0;
        this->spike_med = -1.0;
        this->cobra_init = -1.0;

        this->ball.position = options.Position.Clone();
        this->ball_position_init = options.Position.Clone();
        this->club.distance_state = CalculateTypeDistance(options.Distance);

        this->ball.max_height = this->ball.position.y;
        this->ball.count = 0;
        this->ball.num_max_height = -1;

        double pwr = this->club.GetPower(options, options.PowerObj.Pwr, options.Ps, options.Spin);
        this->power_range_shot = this->club.GetRange(options, options.PowerObj.Pwr, options.Ps);
        this->power_factor = pwr;

        pwr *= std::sqrt(options.PercentShot);

        if (options.Shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::TOMAHAWK)
            || options.Shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE)) {
            pwr *= 1.3;
        }
        else {
            pwr *= 1.0;
        }

        pwr *= std::sqrt(options.Ground * 0.01);

        this->power_factor_shot = pwr;
        this->percentShot_sqrt = std::sqrt(options.PercentShot);

        this->ball.curve = options.Curve;
        this->ball.spin = options.Spin;

        auto value1 = this->GetValuesDegree(options.MiraRad + (0 - (this->ball.curve * Constraints::_00D17908)), 1);
        auto value2 = this->GetValuesDegree(
            (this->club.distance_state == DistanceState::BIGGER_OR_EQUAL_58)
            ? this->club.GetDregRad()
            : this->club.GetDregRad() + (this->ball.spin * Constraints::_00D19B98),
            0);

        // emulate random
        double rnd = (double)std::rand() / (double)RAND_MAX;
        this->ball.curve -= (this->GetSlope(options.MiraRad - options.SlopeMiraRad, rnd));

        pwr *= ((std::fabs(this->ball.curve) * 0.1) + 1.0);

        Vector3D vectA(value2["neg_sin"], value2["neg_rad"], value2["cos2"]);
        vectA.MultiplyScalar(pwr);

        Vector3D v1(value1["cos"], value1["rad"], value1["sin"]);
        Vector3D v2(value1["_C"], value1["_10"], value1["_14"]);
        Vector3D v3(value1["neg_sin"], value1["neg_rad"], value1["cos2"]);
        Vector3D v4(value1["_24"], value1["_28"], value1["_2C"]);

        // ball.velocity.x
        this->ball.velocity.x = (v2.x * vectA.y) + (vectA.x * v1.x) + (v3.x * vectA.z) + v4.x;
        // ball.velocity.y
        this->ball.velocity.y = (v1.y * vectA.x) + (v2.y * vectA.y) + (v3.y * vectA.z) + v4.y;
        // ball.velocity.z
        this->ball.velocity.z = (v1.z * vectA.x) + (v2.z * vectA.y) + (v3.z * vectA.z) + v4.z;

        this->ball.rotation_curve = this->ball.curve * options.PercentShot;
        this->ball.rotation_spin = (this->club.distance_state == DistanceState::BIGGER_OR_EQUAL_58)
            ? ((this->club.GetPower2(options, options.PowerObj.Pwr, options.Ps) * options.PercentShot) * options.PercentShot)
            : 0.0;

        this->ball.ball_48 = this->ball.ball_44;
    }

    std::map<std::string, double> QuadTree::GetValuesDegree(double degree, int option) {
        std::map<std::string, double> obj;
        if (option == 0) {
            obj["cos"] = 1.0;
            obj["rad"] = 0.0;
            obj["sin"] = 0.0;
            obj["_C"] = 0.0;
            obj["_10"] = std::cos(degree);
            obj["_14"] = std::sin(degree) * -1.0;
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
            obj["neg_sin"] = obj["sin"] * -1.0;
            obj["neg_rad"] = 0.0;
            obj["cos2"] = obj["cos"];
            obj["_24"] = 0.0;
            obj["_28"] = 0.0;
            obj["_2C"] = 0.0;
        }
        return obj;
    }

    double QuadTree::GetSlope(double aim, double lineBall) {
        Vector3D ball_slope_cross_const_vect = this->ball.slope.Clone().Normalize();

        std::map<std::string, Vector3D> slope_matrix;
        slope_matrix["v1"] = ball_slope_cross_const_vect.Clone().Normalize();
        slope_matrix["v2"] = this->ball.slope.Clone();
        slope_matrix["v3"] = ball_slope_cross_const_vect.Clone().Cross(this->ball.slope).Normalize();
        slope_matrix["v4"] = Vector3D(0.0, 0.0, 0.0);

        auto value1 = this->GetValuesDegree(aim * -1, 1);
        auto value2 = this->GetValuesDegree(lineBall * -2.0, 1);

        auto m1 = this->ApplyMatrix(this->ValuesDegreesToMatrix(value2), slope_matrix);
        auto m2 = this->ApplyMatrix(m1, this->ValuesDegreesToMatrix(value1));

        return m2["v2"].x * Constraints::_00D66CA0;
    }

    std::map<std::string, Vector3D> QuadTree::ValuesDegreesToMatrix(const std::map<std::string, double>& dictionary) {
        std::map<std::string, Vector3D> obj;
        obj["v1"] = Vector3D(dictionary.at("cos"), dictionary.at("rad"), dictionary.at("sin"));
        obj["v2"] = Vector3D(dictionary.at("_C"), dictionary.at("_10"), dictionary.at("_14"));
        obj["v3"] = Vector3D(dictionary.at("neg_sin"), dictionary.at("neg_rad"), dictionary.at("cos2"));
        obj["v4"] = Vector3D(dictionary.at("_24"), dictionary.at("_28"), dictionary.at("_2C"));
        return obj;
    }

    std::map<std::string, Vector3D> QuadTree::ApplyMatrix(const std::map<std::string, Vector3D>& m1, const std::map<std::string, Vector3D>& m2) {
        std::map<std::string, Vector3D> obj;
        // v1
        obj["v1"] = Vector3D(
            m1.at("v1").x * m2.at("v1").x + m1.at("v1").y * m2.at("v2").x + m1.at("v1").z * m2.at("v3").x,
            m1.at("v1").x * m2.at("v1").y + m1.at("v1").y * m2.at("v2").y + m1.at("v1").z * m2.at("v3").y,
            m1.at("v1").x * m2.at("v1").z + m1.at("v1").y * m2.at("v2").z + m1.at("v1").z * m2.at("v3").z
        );
        // v2
        obj["v2"] = Vector3D(
            m1.at("v2").x * m2.at("v1").x + m1.at("v2").y * m2.at("v2").x + m1.at("v2").z * m2.at("v3").x,
            m1.at("v2").x * m2.at("v1").y + m1.at("v2").y * m2.at("v2").y + m1.at("v2").z * m2.at("v3").y,
            m1.at("v2").x * m2.at("v1").z + m1.at("v2").y * m2.at("v2").z + m1.at("v2").z * m2.at("v3").z
        );
        // v3
        obj["v3"] = Vector3D(
            m1.at("v3").x * m2.at("v1").x + m1.at("v3").y * m2.at("v2").x + m1.at("v3").z * m2.at("v3").x,
            m1.at("v3").x * m2.at("v1").y + m1.at("v3").y * m2.at("v2").y + m1.at("v3").z * m2.at("v3").y,
            m1.at("v3").x * m2.at("v1").z + m1.at("v3").y * m2.at("v2").z + m1.at("v3").z * m2.at("v3").z
        );
        // v4
        obj["v4"] = Vector3D(
            m1.at("v4").x * m2.at("v1").x + m1.at("v4").y * m2.at("v2").x + m1.at("v4").z * m2.at("v3").x,
            m1.at("v4").x * m2.at("v1").y + m1.at("v4").y * m2.at("v2").y + m1.at("v4").z * m2.at("v3").y,
            m1.at("v4").x * m2.at("v1").z + m1.at("v4").y * m2.at("v2").z + m1.at("v4").z * m2.at("v3").z
        );
        return obj;
    }

    void QuadTree::BallProcess(double steptime, double finalVal) {
        this->BounceProcess(steptime, finalVal);

        // COBRA
        if (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::COBRA) && this->cobra_init < 0) {
            if (this->percentShot_sqrt < std::sqrt(0.8)) {
                this->percentShot_sqrt = std::sqrt(0.8);
            }
            if (this->ball.count == 0) {
                this->ball.velocity.y = 0.0;
                this->ball.velocity.Normalize().MultiplyScalar(this->power_factor_shot);
            }
            double diff = this->ball.position.Clone().Sub(this->ball_position_init).Length();
            double cobra_init_up = ((this->power_range_shot * this->percentShot_sqrt) - 100.0) * 3.2;

            if (diff >= cobra_init_up) {
                double power_multiply = 0.0;

                if (this->club.type == ClubType::WOOD) {
                    if (this->club.power_base == 230.0) power_multiply = 74.0;
                    else if (this->club.power_base == 210.0) power_multiply = 76.0;
                    else if (this->club.power_base == 190.0) power_multiply = 80.0;
                }
                this->cobra_init = this->ball.count;
                this->ball.velocity.Normalize().MultiplyScalar(power_multiply).MultiplyScalar(this->percentShot_sqrt);
                this->ball.rotation_spin = 2.5;
            }
        }
        else {
            // If not COBRA or COBRA already triggered
            if (this->spike_init < 0 && this->cobra_init < 0 && this->club.distance_state == DistanceState::BIGGER_OR_EQUAL_58) {
                this->ball.rotation_spin -= (Constraints::_00D66CA0 - (this->ball.spin * Constraints::_00CFF040)) * Constraints::_00D083A0;
            }
            else if (
                (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE) && this->spike_init >= 0) ||
                (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::COBRA) && this->cobra_init >= 0)
                ) {
                this->ball.rotation_spin -= Constraints::_00D083A0;
            }

            if (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE) && this->ball.count == 0) {
                this->ball.velocity.y = 0.0;
                this->ball.velocity.Normalize().MultiplyScalar(this->power_factor_shot);

                this->ball.velocity.Normalize().MultiplyScalar(72.5).MultiplyScalar(this->percentShot_sqrt * 2.0);
                this->ball.rotation_spin = 3.1;
                this->spike_init = this->ball.count;
            }

            if (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE)
                && this->ball.num_max_height >= 0
                && (this->ball.num_max_height + 0x3C) < this->ball.count
                && this->spike_med < 0
                ) {
                this->spike_med = this->ball.count;
                if (this->club.type == ClubType::WOOD) {
                    double new_power = 0.0;
                    if (this->club.power_base == 230.0) {
                        new_power = 344.0;
                        if ((this->power_factor * this->percentShot_sqrt) < 344.0)
                            new_power -= (this->power_factor * this->percentShot_sqrt);
                        else
                            new_power = 0.0;
                        new_power = new_power / 112.0 * 21.5;
                        new_power = -8.0 - new_power;
                        this->ball.velocity.y = new_power;
                    }
                    else if (this->club.power_base == 210.0) {
                        new_power = 306.0;
                        if ((this->power_factor * this->percentShot_sqrt) < 306.0)
                            new_power -= (this->power_factor * this->percentShot_sqrt);
                        else
                            new_power = 0.0;
                        new_power = new_power / 105.0 * 20.5;
                        new_power = -10.3 - new_power;
                        this->ball.velocity.y = new_power;
                    }
                    else if (this->club.power_base == 190.0) {
                        new_power = 273.0;
                        if ((this->power_factor * this->percentShot_sqrt) < 273.0)
                            new_power -= (this->power_factor * this->percentShot_sqrt);
                        else
                            new_power = 0.0;
                        new_power = new_power / 100.0 * 20.2;
                        new_power = -10.8 - new_power;
                        this->ball.velocity.y = new_power;
                    }
                }
                this->ball.velocity.Normalize().MultiplyScalar(this->percentShot_sqrt * 7.0);
                this->ball.rotation_spin = this->ball.spin;
            }
        }

        if (this->ball.velocity.y < 0 && this->ball.num_max_height < 0) {
            this->ball.max_height = this->ball.position.y;
            this->ball.num_max_height = this->ball.count;
        }

        this->ball.count++;
    }

    void QuadTree::BounceProcess(double steptime, double finalVal) {
        if (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE)
            && this->ball.num_max_height >= 0
            && (this->ball.num_max_height + 0x3C) > this->ball.count) {
            return;
        }

        Vector3D accellVect = this->ApplyForce();
        Vector3D otherVect = accellVect.Clone();
        otherVect.DivideScalar(this->ball.mass).MultiplyScalar(steptime);

        this->ball.velocity.Add(otherVect);

        if (this->ball.num_max_height == -1) {
            Vector3D tmpVect = this->_21D8_vect.Clone().DivideScalar(this->ball.mass).MultiplyScalar(steptime);
            this->ball.velocity.Add(tmpVect);
        }

        this->ball.ball_2C += (this->ball.rotation_curve * Constraints::_00D1A888 * steptime);
        this->ball.ball_30 += (this->ball.rotation_spin * Constraints::_00D3D210 * steptime);

        double step = (finalVal >= 0.0 ? finalVal : steptime);
        this->ball.position.Add(this->ball.velocity.Clone().MultiplyScalar(step));
    }

    Vector3D QuadTree::ApplyForce() {
        Vector3D retVect(0.0, 0.0, 0.0);

        if (this->ball.rotation_curve != 0.0) {
            Vector3D vectorb(this->ball.velocity.z * Constraints::_00D046A8, 0.0, this->ball.velocity.x);
            vectorb.Normalize();
            if (this->cobra_init < 0 || this->spike_init < 0) {
                vectorb.MultiplyScalar(Constraints::_00D00190 * this->ball.rotation_curve * this->club.rotation_curve);
            }
            retVect.Add(vectorb);
        }

        if (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE) && this->spike_init < 0) {
            return Vector3D(0.0, 0.0, 0.0);
        }
        else if (this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::COBRA) && this->cobra_init < 0) {
            return retVect;
        }

        Vector3D windVect = this->wind.GetWind();
        windVect.MultiplyScalar((this->shot == PowerShotFactory::GetIntValueFromShotType(ShotTypeEnum::SPIKE))
            ? Constraints::_00D16758
            : Constraints::_00D083A0);
        retVect.Add(windVect);

        retVect.y = retVect.y - (this->GetGravity() * this->ball.mass);

        if (this->ball.rotation_spin != 0.0) {
            retVect.y = retVect.y + (this->club.rotation_spin * Constraints::_00D66CF8 * this->ball.rotation_spin);
        }

        Vector3D velVect = this->ball.velocity.Clone();
        velVect.MultiplyScalar(velVect.Length() * Constraints::_00D3D028);
        retVect.Sub(velVect);

        return retVect;
    }

    // ---------------------------------------------------------
    // Utils
    // ---------------------------------------------------------
    namespace Utils {
        double DiffYZ(const Vector3D& v1, const Vector3D& v2) {
            return std::sqrt(std::pow(v1.x - v2.x, 2) + std::pow(v1.z - v2.z, 2));
        }

        // Example FindPower function from the code. Because it references so many classes,
        // we replicate everything in C++ form. This is fairly large but demonstrates usage.
        // -------------------------------------------------------------------------
        static double DoubleUnderThirty(double number) {
            return number / 30.0;
        }

        static Options SetUpOptions(
            int power_player_or_isSomething,
            double distance, double percent, double ground,
            double aim, double slope_aim_rad, double spin, double curve,
            int shot, int power_shot
        ) {
            // For demonstration, we'll always treat the input as an integer power
            // to match the original code's "is_integer" check.
            PowerOptions pOpts(0, 4, 4, 0, 0, 8);
            Power pwrObj(power_player_or_isSomething, pOpts);
            Options opt(
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
                pwrObj
            );
            return opt;
        }

        static double FindColisionHeight(Ball& vball, double margin, double distance_scale,
            QuadTree& quad_t, double colision_height) {
            int counter = 0;
            Ball copy_ball;
            do {
                copy_ball.Copy(vball);
                quad_t.BallProcess(Constraints::_00D083A0);
            } while ((vball.position.y > colision_height || vball.num_max_height == -1) && (counter++) < 3000);

            double last_step = std::fabs((colision_height - copy_ball.position.y) / (vball.position.y - copy_ball.position.y));

            vball.Copy(copy_ball);
            quad_t.BallProcess(Constraints::_00D083A0, Constraints::_00D083A0 * last_step);

            if (std::fabs(distance_scale - vball.position.z) <= margin) {
                return 0.0;
            }
            return distance_scale - vball.position.z;
        }

        static std::map<std::string, double> FindPower(
            int power_player,
            const ClubInfo* club_info,
            int shot,
            int power_shot,
            double distance,
            double height,
            Wind wind,
            double angle,
            double ground,
            double spin,
            double curve,
            const Vector3D& slope, // or double slope
            double aim,
            double percent
        ) {
            double heigth_colision = height * 1.094 * 3.2;
            double distance_scale = distance * 3.2;

            Ball vball;
            Club vclub;

            if (club_info != nullptr) {
                vclub.Init(*club_info);
            }

            vclub.distance_state = CalculateTypeDistance(distance_scale);

            double slope_aim_rad = slope.y; // example usage (in actual code you'd adapt)

            // We store the result here:
            std::map<std::string, double> found;
            found["power"] = -1.0;
            found["desvio"] = 0.0;

            Options options = SetUpOptions(power_player, distance_scale, percent,
                ground, aim, slope_aim_rad,
                spin, curve, shot, power_shot);

            double power_range = vclub.GetRange(options, options.PowerObj.Pwr, options.Ps);
            double margin = 0.05;
            QuadTree qt;

            double limit_checking = 1000;
            int count = 0;
            bool is_find = false;
            double ret = 0.0;
            int side = 0;
            double feed = 0.00006;

            do {
                if (options.PercentShot > 1.3)
                    options.PercentShot = 1.3;
                else if (options.PercentShot < 0.0)
                    options.PercentShot = 0.1;

                qt.InitShot(vball, vclub, wind, options);
                ret = FindColisionHeight(vball, margin, distance_scale, qt, heigth_colision);

                if (ret == 0.0) {
                    is_find = true;
                }
                else {
                    if (options.PercentShot == 1.3 && ret > 0) break;
                    if (options.PercentShot == 0.1 && ret < 0) break;

                    if (side == 0) {
                        side = (ret < 0.0 ? -1 : 1);
                    }
                    else if ((ret < 0.0 && side == 1) || (ret > 0.0 && side == -1)) {
                        feed *= 0.5;
                    }
                    options.PercentShot += ret * feed;
                }

            } while (!is_find && (count++) < limit_checking);

            if (is_find) {
                double _desvio = (vball.position.x + (options.Position.x + (std::tan(options.MiraRad) * distance_scale))) *
                    Constraints::DESVIO_SCALE_PANGYA_TO_YARD;
                double final_power = options.PercentShot;

                // We build Data:
                SmartData sd(_desvio, heigth_colision, vclub, options);
                Data data(final_power, _desvio, power_range, sd);

                found["power"] = data.power;
                found["desvio"] = data.desvio;
                found["power_range"] = data.power_range;
                // etc. (if needed, store more)
            }

            return found;
        }

    } // namespace Utils

} // namespace Calculadora
