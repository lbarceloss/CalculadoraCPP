#pragma once
#include "Club.h"
#include "Options.h"

struct SmartData {
    double desvio;
    double height;
    Club club;
    Options options;

    // Construtor
    SmartData(double desvio, double height, const Club& club, const Options& options)
        : desvio(desvio), height(height), club(club), options(options) {
    }
};
