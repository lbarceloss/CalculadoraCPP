#pragma once
#include <array>
#include <string>

struct ShotType {
    static constexpr std::array<const char*, 4> shot_type_enum = {
        "DUNK",
        "TOMAHAWK",
        "SPIKE",
        "COBRA"
    };
};
