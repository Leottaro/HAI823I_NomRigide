#pragma once

#include <chrono>

struct ProfileFrame {
    double frame_ms = 0.;
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

inline ProfileFrame& operator+=(ProfileFrame& lhs, const ProfileFrame& rhs) {
    lhs.frame_ms += rhs.frame_ms;
    lhs.collision_ms += rhs.collision_ms;
    lhs.point_triangle_collision_ms += rhs.point_triangle_collision_ms;
    lhs.edge_triangle_collision_ms += rhs.edge_triangle_collision_ms;
    lhs.triangle_point_collision_ms += rhs.triangle_point_collision_ms;
    lhs.self_point_triangle_collision_ms += rhs.self_point_triangle_collision_ms;
    lhs.constraints_ms += rhs.constraints_ms;
    lhs.rendering_ms += rhs.rendering_ms;
    return lhs;
}

inline ProfileFrame operator/(const ProfileFrame& frame, double divisor) {
    if (divisor <= 0.) {
        return {};
    }
    return {
        frame.frame_ms / divisor,
        frame.collision_ms / divisor,
        frame.point_triangle_collision_ms / divisor,
        frame.edge_triangle_collision_ms / divisor,
        frame.triangle_point_collision_ms / divisor,
        frame.self_point_triangle_collision_ms / divisor,
        frame.constraints_ms / divisor,
        frame.rendering_ms / divisor,
    };
}

class ScopedTimer {
  public:
    explicit ScopedTimer(double& target_ms)
        : m_target_ms(target_ms), m_start_ms(profileNowMs()) {}

    ~ScopedTimer() {
        m_target_ms += profileNowMs() - m_start_ms;
    }

  private:
    double& m_target_ms;
    double m_start_ms;
};
