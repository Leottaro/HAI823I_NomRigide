#include "DynamicObject.hpp"

void DynamicObject::addConstraint(
    uint _cardinality,
    const constraint_function& _function,
    const gradient_function& _gradient,
    const std::vector<uint>& _indices,
    double _stiffness,
    const ConstraintType& _type) {
    M++;
    m_cardinalities.push_back(_cardinality);
    m_functions.push_back(_function);
    m_gradients.push_back(_gradient);
    m_indices.push_back(_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(_type);
    m_debug_types.push_back(CUSTOM_CONSTRAINT);
}

void DynamicObject::addDistanceConstraint(uint _p0, uint _p1, double _stiffness, double _targeted_distance) {
    M++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);
    m_debug_types.push_back(DISTANCE_CONSTRAINT);

    m_functions.push_back([_targeted_distance](const std::vector<glm::dvec3>& _p) {
        return glm::distance(_p[0], _p[1]) - _targeted_distance;
    });
    m_gradients.push_back([](const std::vector<glm::dvec3>& _p) {
        glm::dvec3 n = glm::normalize(_p[0] - _p[1]);
        return std::vector<glm::dvec3>{n, -n};
    });
}
void DynamicObject::addDistanceConstraint(uint _p0, uint _p1, double _stiffness) {
    addDistanceConstraint(_p0, _p1, _stiffness, glm::distance(m_positions[_p0], m_positions[_p1]));
}

void DynamicObject::addBendingConstraint(uint _p0, uint _p1, uint _p2, uint _p3, double _stiffness, double _targeted_angle) {
    M++;
    m_cardinalities.push_back(4);
    m_indices.push_back({_p0, _p1, _p2, _p3});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);
    m_debug_types.push_back(BENDING_CONSTRAINT);

    m_functions.push_back([_targeted_angle](const std::vector<glm::dvec3>& _p) {
        glm::dvec3 e = glm::normalize(_p[1] - _p[0]); // arête commune

        glm::dvec3 n1 = glm::normalize(glm::cross(_p[2] - _p[0], _p[2] - _p[1]));
        glm::dvec3 n2 = glm::normalize(glm::cross(_p[3] - _p[1], _p[3] - _p[0]));
        double cosTheta = glm::dot(n1, n2);
        double theta = acos(cosTheta);
        double sign = glm::dot(glm::cross(n1, n2), e);
        if (sign < 0)
            theta = -theta;
        return theta - _targeted_angle;
    });
    m_gradients.push_back([](const std::vector<glm::dvec3>& _p) {
        // bridson model
        // p0 and p1 : common edge
        glm::dvec3 e = _p[1] - _p[0];
        double elen = glm::length(e);

        glm::dvec3 n1 = glm::cross(_p[2] - _p[0], _p[2] - _p[1]);
        glm::dvec3 n2 = glm::cross(_p[3] - _p[1], _p[3] - _p[0]);
        double n1sq = glm::length2(n1);
        double n2sq = glm::length2(n2);

        // gp2 = u1, gp3 = u2, gp0 = u3, gp1 = u4
        glm::dvec3 gp2 = elen * (n1 / n1sq);
        glm::dvec3 gp3 = elen * (n2 / n2sq);
        glm::dvec3 gp0 = glm::dot(_p[2] - _p[1], e) / elen * (n1 / n1sq) + glm::dot(_p[3] - _p[1], e) / elen * (n2 / n2sq);
        glm::dvec3 gp1 = -glm::dot(_p[2] - _p[0], e) / elen * (n1 / n1sq) - glm::dot(_p[3] - _p[0], e) / elen * (n2 / n2sq);

        // glm::dvec3 sum = gp0 + gp1 + gp2 + gp3;
        // if (glm::length(sum) > 1.e-4) {
        //     std::cout << "ERREUR : sum != 0" << std::endl;
        // }

        return std::vector<glm::dvec3>{-gp0, -gp1, -gp2, -gp3};
    });
}
void DynamicObject::addBendingConstraint(uint _p0, uint _p1, uint _p2, uint _p3, double _stiffness) {
    glm::dvec3 e = glm::normalize(m_positions[_p1] - m_positions[_p0]);
    glm::dvec3 n1 = glm::normalize(glm::cross(m_positions[_p2] - m_positions[_p0], m_positions[_p2] - m_positions[_p1]));
    glm::dvec3 n2 = glm::normalize(glm::cross(m_positions[_p3] - m_positions[_p1], m_positions[_p3] - m_positions[_p0]));
    double cosTheta = glm::clamp(glm::dot(n1, n2), -1., 1.);
    double theta = acos(cosTheta);
    double sign = glm::dot(glm::cross(n1, n2), e);
    if (sign < 0)
        theta = -theta;
    addBendingConstraint(_p0, _p1, _p2, _p3, _stiffness, theta);
}

void DynamicObject::addVolumeConstraint(std::vector<glm::uvec3> _indices, double _stiffness, double _pressure, double _targeted_volume) {
    M++;
    m_cardinalities.push_back(getPositions().size());
    std::vector<uint> all_indices(N);
    std::iota(all_indices.begin(), all_indices.end(), 0);
    m_indices.push_back(all_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);
    m_debug_types.push_back(VOLUME_CONSTRAINT);

    m_functions.push_back([_targeted_volume, _pressure, _indices](const std::vector<glm::dvec3>& _p) {
        double V = 0;
        for (size_t i = 0; i < _indices.size(); i++) {
            const glm::dvec3 p1 = _p[_indices[i][0]];
            const glm::dvec3 p2 = _p[_indices[i][1]];
            const glm::dvec3 p3 = _p[_indices[i][2]];
            V += glm::dot(glm::cross(p1, p2), p3) / 6.;
        }
        return V - _pressure * _targeted_volume;
    });
    m_gradients.push_back([_indices](const std::vector<glm::dvec3>& _p) {
        std::vector<glm::dvec3> grads(_p.size(), glm::dvec3(0.0));
        for (size_t i = 0; i < _indices.size(); i++) {
            uint i1 = _indices[i][0];
            uint i2 = _indices[i][1];
            uint i3 = _indices[i][2];

            glm::dvec3 p1 = _p[i1];
            glm::dvec3 p2 = _p[i2];
            glm::dvec3 p3 = _p[i3];

            grads[i1] += glm::cross(p2, p3) / 6.;
            grads[i2] += glm::cross(p3, p1) / 6.;
            grads[i3] += glm::cross(p1, p2) / 6.;
        }
        return grads;
    });
}

void DynamicObject::addVolumeConstraint(std::vector<glm::uvec3> _indices, double _stiffness, double _pressure) {
    double V = 0;
    for (size_t i = 0; i < _indices.size(); i++) {
        const glm::dvec3 p1 = m_positions[_indices[i][0]];
        const glm::dvec3 p2 = m_positions[_indices[i][1]];
        const glm::dvec3 p3 = m_positions[_indices[i][2]];
        V += glm::dot(glm::cross(p1, p2), p3) / 6.;
    }
    addVolumeConstraint(_indices, _stiffness, _pressure, V);
}

void DynamicObject::addCollisionConstraint(uint _p0, glm::dvec3 _intersection, glm::dvec3 _normal) {
    Mcoll++;
    m_cardinalities.push_back(1);
    m_indices.push_back({_p0});
    m_stiffnesses.push_back(1.);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(VERTEX_COLLISION_CONSTRAINT);

    m_functions.push_back([_intersection, _normal](const std::vector<glm::dvec3>& _p) {
        return glm::dot(_p[0] - _intersection, _normal);
    });
    m_gradients.push_back([_normal](const std::vector<glm::dvec3>& _p) {
        return std::vector<glm::dvec3>{_normal};
    });
}
void DynamicObject::addEdgeCollisionConstraint(uint _p0, uint _p1,
                                               double _t1, glm::dvec3 _point1, glm::dvec3 _normal1,
                                               double _t2, glm::dvec3 _point2, glm::dvec3 _normal2) {
    // Create first contact constraint
    Mcoll++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(1.0);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(EDGE_COLLISION_CONSTRAINT);

    double thickness = m_surface_thickness;
    m_functions.push_back([thickness, _t1, _point1, _normal1](const std::vector<glm::dvec3>& _p) {
        glm::dvec3 edge_pt = (1.0 - _t1) * _p[0] + _t1 * _p[1];
        double dist = glm::dot(edge_pt - _point1, _normal1);
        return dist - thickness;
    });

    m_gradients.push_back([_t1, _normal1](const std::vector<glm::dvec3>& _p) {
        return std::vector<glm::dvec3>{
            (1.0 - _t1) * _normal1,
            _t1 * _normal1};
    });

    // Create second contact constraint
    Mcoll++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(1.0);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(EDGE_COLLISION_CONSTRAINT);

    m_functions.push_back([thickness, _t2, _point2, _normal2](const std::vector<glm::dvec3>& _p) {
        glm::dvec3 edge_pt = (1.0 - _t2) * _p[0] + _t2 * _p[1];
        double dist = glm::dot(edge_pt - _point2, _normal2);
        return dist - thickness;
    });

    m_gradients.push_back([_t2, _normal2](const std::vector<glm::dvec3>& _p) {
        return std::vector<glm::dvec3>{
            (1.0 - _t2) * _normal2,
            _t2 * _normal2};
    });
}

void DynamicObject::addStaticPointDynamicTriangleConstraint(uint _p0, uint _p1, uint _p2, glm::dvec3 _static_point, glm::dvec3 _barycentrics, glm::dvec3 _normal) {
    Mcoll++;
    m_cardinalities.push_back(3);
    m_indices.push_back({_p0, _p1, _p2});
    m_stiffnesses.push_back(1.);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(TRAINGLE_COLLISION_CONSTRAINT);

    double thickness = m_surface_thickness;
    m_functions.push_back([thickness, _static_point, _barycentrics, _normal](const std::vector<glm::dvec3>& _p) {
        glm::dvec3 surface_pt = _barycentrics[0] * _p[0] + _barycentrics[1] * _p[1] + _barycentrics[2] * _p[2];
        return glm::dot(surface_pt - _static_point, _normal) - thickness;
    });

    m_gradients.push_back([_barycentrics, _normal](const std::vector<glm::dvec3>& _p) {
        return std::vector<glm::dvec3>{
            _barycentrics[0] * _normal,
            _barycentrics[1] * _normal,
            _barycentrics[2] * _normal};
    });
}

void DynamicObject::addSelfCollisionConstraint(uint _q, uint _p0, uint _p1, uint _p2) {
    Mcoll++;
    m_cardinalities.push_back(4);
    m_indices.push_back({_q, _p0, _p1, _p2});
    m_stiffnesses.push_back(1.);
    m_types.push_back(INEQUALITY_CONSTRAINT);
    m_debug_types.push_back(SELF_COLLISION_CONSTRAINT);

    double thickness = m_surface_thickness;
    m_functions.push_back([thickness](const std::vector<glm::dvec3>& _p) {
        const glm::dvec3& q = _p[0];
        const glm::dvec3& p1 = _p[1];
        const glm::dvec3& p2 = _p[2];
        const glm::dvec3& p3 = _p[3];

        const glm::dvec3 e2 = p2 - p1;
        const glm::dvec3 e3 = p3 - p1;
        glm::dvec3 n = glm::cross(e2, e3);
        double n_len = glm::length(n);

        if (n_len < 1e-8)
            return 0.0;

        return glm::dot(q - p1, n) / n_len - thickness;
    });

    m_gradients.push_back([](const std::vector<glm::dvec3>& _p) {
        const glm::dvec3& q = _p[0];
        const glm::dvec3& p1 = _p[1];
        const glm::dvec3& p2 = _p[2];
        const glm::dvec3& p3 = _p[3];

        const glm::dvec3 e2 = p2 - p1;
        const glm::dvec3 e3 = p3 - p1;
        const glm::dvec3 x = q - p1;

        glm::dvec3 n = glm::cross(e2, e3);
        double n_len = glm::length(n);

        if (n_len < 1e-8)
            return std::vector<glm::dvec3>{glm::dvec3(0), glm::dvec3(0), glm::dvec3(0), glm::dvec3(0)};

        glm::dvec3 n_hat = n / n_len;

        // C = dot(x, n) / |n| - h
        double signed_dist = glm::dot(x, n_hat);
        glm::dvec3 x_proj = x - signed_dist * n_hat; // projection of x onto triangle plane

        // dC/dq = n_hat
        glm::dvec3 grad_q = n_hat;

        // dC/de2 = cross(e3, x_proj) / |n|
        // Since e2 = p2 - p1, dC/dp2 = dC/de2
        glm::dvec3 grad_p2 = glm::cross(e3, x_proj) / n_len;

        // dC/de3 = cross(x_proj, e2) / |n|
        // Since e3 = p3 - p1, dC/dp3 = dC/de3
        glm::dvec3 grad_p3 = glm::cross(x_proj, e2) / n_len;

        // dC/dp1 = -dC/dq - dC/de2 - dC/de3 (translation invariance)
        glm::dvec3 grad_p1 = -grad_q - grad_p2 - grad_p3;

        return std::vector<glm::dvec3>{grad_q, grad_p1, grad_p2, grad_p3};
    });
}