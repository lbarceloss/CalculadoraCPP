#pragma once
#include <memory>
#include "Vector3D.h"

class PowerOptions {
public:
    int Auxpart;
    int Mascot;
    int Card;
    int PsAuxpart;
    int PsMascot;
    int PsCard;

    // Construtor
    PowerOptions(int auxpart = 0, int mascot = 0, int card = 0, int psAuxpart = 0, int psMascot = 0, int psCard = 0)
        : Auxpart(auxpart), Mascot(mascot), Card(card),
        PsAuxpart(psAuxpart), PsMascot(psMascot), PsCard(psCard) {
    }

    // Retorna o total da potência
    int Total(int option) const {
        int pwr = Auxpart + Mascot + Card;
        if (option == 1 || option == 2 || option == 3) {
            pwr += PsAuxpart + PsMascot + PsCard;
        }
        return pwr;
    }
};

class Power {
public:
    int Pwr;
    std::unique_ptr<PowerOptions> Options;

    // Construtor
    Power(int power, std::unique_ptr<PowerOptions> opt)
        : Pwr(power), Options(std::move(opt)) {
    }
};

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
    std::unique_ptr<Power> PowerObj;

    // Construtor
    Options(double distance, double percentShot = 1.0, double ground = 0.0, double miraRad = 0.0, double slopeMiraRad = 0.0,
        double spin = 0.0, double curve = 0.0, const Vector3D& position = Vector3D(0.0, 0.0, 0.0), int shot = 0, int ps = 0,
        std::unique_ptr<Power> power = nullptr)
        : Distance(distance), PercentShot(percentShot), Ground(ground), MiraRad(miraRad), SlopeMiraRad(slopeMiraRad),
        Spin(spin / 30), Curve(curve / 30), Position(position), Shot(shot), Ps(ps),
        PowerObj(std::move(power ? std::move(power) : std::make_unique<Power>(31, std::make_unique<PowerOptions>()))) {
    }
};
