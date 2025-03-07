#pragma once
#include "Vector3D.h"
#include "Club.h"
#include "Wind.h"
#include "Ball.h"

class Constraints {
public:
    // Constantes matemáticas
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

    // Variáveis estáticas
    static Vector3D _00E42544_vect_slope;
    static Club club;
    static Wind wind;
    static Ball ball;

    static constexpr double YARDS_TO_PB = 0.2167;
    static constexpr double YARDS_TO_PBA = 0.8668;
    static constexpr double YARDS_TO_PBA_PLUS = 1.032;
};
