#pragma once
#include "Vector3D.h"

class Ball {
public:
    Vector3D position{ 0.0, 0.0, 0.0 };
    Vector3D slope{ 0.0, 1.0, 0.0 };
    Vector3D velocity{ 0.0, 0.0, 0.0 };

    int state_process = 0;
    double max_height = 0.0;
    int num_max_height = -1;
    int count = 0;

    double ball_28 = 0.0;
    double ball_2C = 0.0;
    double ball_30 = 0.0;

    double curve = 0.0;
    double spin = 0.0;

    double rotation_curve = 0.0;
    double rotation_spin = 0.0;

    int ball_44 = 0;
    int ball_48 = 0;
    int ball_70 = -1;
    int ball_90 = 0;
    int ball_BC = 0;

    double mass = 0.045926999;
    double diametro = 0.14698039;

    // Construtor padrão
    Ball() = default;

    // Método para copiar os dados de outro objeto Ball
    void Copy(const Ball& other) {
        position = other.position;
        slope = other.slope;
        velocity = other.velocity;
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
    }
};
