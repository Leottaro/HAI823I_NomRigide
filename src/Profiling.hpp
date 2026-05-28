#pragma once

#include <chrono>

struct ProfileFrame {
    double collision_ms = 0.;
    double point_triangle_collision_ms = 0.;
    double edge_triangle_collision_ms = 0.;
    double triangle_point_collision_ms = 0.;
    double self_point_triangle_collision_ms = 0.;
    double constraints_ms = 0.;
    double rendering_ms = 0.;
};

inline ProfileFrame g_profile_frame;

inline double profileNowMs() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double, std::milli>(clock::now().time_since_epoch()).count();
}

inline void resetProfileFrame() {
    g_profile_frame = {};
}
