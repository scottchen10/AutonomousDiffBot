#pragma once
#include <string>

struct Wheel {
    std::string name = "";
    double velocity = 0;
    double position = 0;
    double cmd_vel = 0;
    int counts_per_rev;

    Wheel(const std::string name, int counts_per_rev) {
        this->name = name;
        this->counts_per_rev = counts_per_rev;
    }
};