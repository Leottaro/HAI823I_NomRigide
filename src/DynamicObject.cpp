#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/matrix.hpp>

#include "DynamicObject.hpp"
#include <iostream>
#include <unordered_map>

bool computeBarycentrics(const glm::dvec3 &v0, const glm::dvec3 &v1, const glm::dvec3 &v2, const glm::dvec3 &normal, const glm::dvec3 &p, glm::dvec3 &barycentrics) {
    double total_area_sq = glm::length2(normal);
    if (total_area_sq < 1e-16)
        return false;

    // Signed barycentric coordinates
    barycentrics.x = glm::dot(glm::cross(v1 - p, v2 - p), normal) / total_area_sq;
    barycentrics.y = glm::dot(glm::cross(v2 - p, v0 - p), normal) / total_area_sq;
    barycentrics.z = glm::dot(glm::cross(v0 - p, v1 - p), normal) / total_area_sq;

    if (barycentrics.x < -1e-5 || 1. + 1e-5 < barycentrics.x ||
        barycentrics.y < -1e-5 || 1. + 1e-5 < barycentrics.y ||
        barycentrics.z < -1e-5 || 1. + 1e-5 < barycentrics.z) {
        return false;
    }

    return true;
}

bool rayTriangleIntersection(const glm::dvec3 &origin, const glm::dvec3 &direction,
                             const glm::dvec3 &v0, const glm::dvec3 &v1, const glm::dvec3 &v2, const glm::dvec3 &normal,
                             double &t, glm::dvec3 &intersection, glm::dvec3 &barycentrics) {
    // Check if ray is parallel
    double dot = glm::dot(direction, normal);
    if (std::abs(dot) <= 1.e-8) {
        return false;
    }

    // determine intersection
    t = -(glm::dot(normal, origin - v0)) / dot;
    intersection = origin + t * direction;

    // barycentric coordinates
    return computeBarycentrics(v0, v1, v2, normal, intersection, barycentrics);
}

/*
READ "3.5. Damping" of ./articles/Position_Based_Dynamics.pdf
(1) xcm = (∑i xi*mi )/( ∑i mi )
(2) vcm = (∑i vi*mi )/( ∑i mi )
(3) L = ∑i ri × (mi*vi)
(4) I = ∑i r̃i r̃Ti mi
(5) ω = I−1L
(6) forall vertices i
(7) ∆vi = vcm + ω × ri − vi
(8) vi ← vi + kdamping ∆vi
(9) endfor
*/
void DynamicObject::dampVelocities(double k_damping) {
    bool is_fixed = false;
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i]) {
            is_fixed = true;
            break;
        }
    }

    if (is_fixed) {
        for (uint i = 0; i < N; i++) {
            if (!m_fixed[i]) {
                m_velocities[i] *= (1.0f - m_ambient_friction_coefficient);
            }
        }
        return;
    }

    double total_mass = 0.;
    glm::dvec3 xcm = glm::dvec3(0.); // (1) : global linear velocity
    glm::dvec3 vcm = glm::dvec3(0.); // (2)
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i])
            continue;
        xcm += m_positions[i] * m_masses[i];  // (1)
        vcm += m_velocities[i] * m_masses[i]; // (2)
        total_mass += m_masses[i];
    }

    if (total_mass == 0.) {
        return; // Nothing to damp
    }

    xcm /= total_mass; // (1)
    vcm /= total_mass; // (2)

    glm::dvec3 L = glm::dvec3(0.); // (3)
    glm::dmat3 I = glm::dmat3(0.); // (4)
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i])
            continue;
        glm::dvec3 ri = m_positions[i] - xcm;
        // (3)
        L += glm::cross(ri, m_masses[i] * m_velocities[i]);
        // (4)
        // glm::dmat3 r_tilde_i = glm::dmat3(0, -ri.z, ri.y, ri.z, 0, -ri.x, -ri.y, ri.x, 0);
        // I += r_tilde_i * glm::transpose(r_tilde_i) * m_masses[i];
        I += m_masses[i] * glm::dmat3(ri.y * ri.y + ri.z * ri.z, -ri.x * ri.y, -ri.x * ri.z, -ri.x * ri.y, ri.x * ri.x + ri.z * ri.z, -ri.y * ri.z, -ri.x * ri.z, -ri.y * ri.z, ri.x * ri.x + ri.y * ri.y);
    }

    // check invertibility
    if (std::abs(glm::determinant(I)) < 1e-8) {
        return;
    }
    glm::dvec3 omega = glm::inverse(I) * L; // (5): angular velocity

    // (6)-(9)
    for (uint i = 0; i < N; i++) {
        if (m_fixed[i])
            continue;
        glm::dvec3 ri = m_positions[i] - xcm;
        glm::dvec3 dvi = vcm + glm::cross(omega, ri) - m_velocities[i]; // (7)
        m_velocities[i] += k_damping * dvi;                             // (8)
    }
}

/*
READ "3.1. Algorithm Overview" of ./articles/Position_Based_Dynamics.pdf
 (1)  forall vertices i
 (2)      initialize xi = x0i , vi = v0i , wi = 1/mi
 (3)  endfor
 (4)  loop
 (5)      forall vertices i do vi ← vi + ∆twifext (xi)
 (6)      dampVelocities(v1 ,..., vN )
 (7)      forall vertices i do pi ← xi + ∆tvi
 (8)      forall vertices i do generateCollisionConstraints(xi → pi)
 (9)      loop solverIterations times
(10)          projectConstraints(C1 ,..., CM+Mcoll , p1,..., pN )
(11)      endloop
(12)      forall vertices i
(13)          v i ← (pi − xi )/∆t
(14)          x i ← pi
(15)      endfor
(16)      velocityUpdate(v1 ,..., vN )
(17)  endloop
*/

bool DynamicObject::update(double _delta_time, uint _solver_iterations, const std::vector<StaticBody> &static_bodies) {
    std::vector<glm::dvec3> new_positions(N); // p_i

    // (5) external forces (gravity, etc...) (for now, just gravity)
    for (uint pj = 0; pj < N; pj++)
        m_velocities[pj] = m_fixed[pj] ? m_velocities[pj] : m_velocities[pj] + _delta_time * glm::dvec3(0., -9.807, 0.);

    // (6)
    dampVelocities(1.);

    // (7)
    for (uint pj = 0; pj < N; pj++)
        new_positions[pj] = m_fixed[pj] ? m_positions[pj] : m_positions[pj] + _delta_time * m_velocities[pj];

    // (8)
    std::unordered_map<size_t, glm::dvec3> colliding_vertices;
    for (uint pj = 0; pj < N; pj++) {
        glm::dvec3 origin = m_positions[pj];
        glm::dvec3 direction = new_positions[pj] - m_positions[pj];

        for (const StaticBody &static_body : static_bodies) {
            const std::vector<glm::vec3> &mesh_positions = static_body.m_mesh->vertexPositions();
            const std::vector<glm::vec3> &mesh_normals = static_body.m_mesh->vertexNormals();
            const std::vector<glm::uvec3> &mesh_triangles = static_body.m_mesh->triangleIndices();
            glm::mat4 transformation = static_body.m_transformation->computeTransformationMatrix();

            double min_t = DBL_MAX, max_t = -DBL_MAX, t;
            glm::dvec3 closest_intersection, closest_normal, furthest_intersection, furthest_normal, intersection, barycentrics;

            double min_dist = DBL_MAX, dist;
            glm::dvec3 closest_surface, closest_surface_normal, surface;

            for (const glm::uvec3 &triangle : mesh_triangles) {
                glm::dvec3 v0 = applyTransformation(mesh_positions[triangle[0]], 1.f, transformation);
                glm::dvec3 v1 = applyTransformation(mesh_positions[triangle[1]], 1.f, transformation);
                glm::dvec3 v2 = applyTransformation(mesh_positions[triangle[2]], 1.f, transformation);

                glm::dvec3 n0 = applyTransformation(mesh_normals[triangle[0]], 0.f, transformation);
                glm::dvec3 n1 = applyTransformation(mesh_normals[triangle[1]], 0.f, transformation);
                glm::dvec3 n2 = applyTransformation(mesh_normals[triangle[2]], 0.f, transformation);
                glm::dvec3 triangle_normal = glm::cross(v1 - v0, v2 - v0);

                // ray intersections
                if (rayTriangleIntersection(origin, direction, v0, v1, v2, triangle_normal, t, intersection, barycentrics)) {
                    if (t < min_t) {
                        min_t = t;
                        closest_intersection = intersection;
                        closest_normal = barycentrics[0] * n0 + barycentrics[1] * n1 + barycentrics[2] * n2;
                    }
                    if (t > max_t) {
                        max_t = t;
                        furthest_intersection = intersection;
                        furthest_normal = barycentrics[0] * n0 + barycentrics[1] * n1 + barycentrics[2] * n2;
                    }
                }

                // closest surface
                glm::dvec3 project_on_plane = v0 + glm::cross(glm::normalize(triangle_normal), glm::cross(new_positions[pj] - v0, glm::normalize(triangle_normal)));
                computeBarycentrics(v0, v1, v2, triangle_normal, project_on_plane, barycentrics);

                // https://www.desmos.com/calculator/eeqkstj2ck
                const auto fallback_in_triangle = [project_on_plane](const glm::dvec3 &p1, const glm::dvec3 &p2) {
                    glm::dvec3 direction = p2 - p1;
                    double n_squared = std::pow(glm::distance(p2, p1), 2);
                    double dot = glm::dot(direction, project_on_plane - p1);
                    double dot_over_one = dot / n_squared;
                    double r = std::clamp(dot_over_one, 0., 1.);
                    return p1 + direction * r;
                };

                surface = barycentrics[0] < 0.   ? fallback_in_triangle(v1, v2)
                          : barycentrics[1] < 0. ? fallback_in_triangle(v2, v0)
                          : barycentrics[2] < 0. ? fallback_in_triangle(v0, v1)
                                                 : project_on_plane;

                dist = glm::distance(new_positions[pj], surface);

                if (dist < min_dist) {
                    min_dist = dist;
                    closest_surface = surface;
                    closest_surface_normal = n0 * barycentrics[0] + n1 * barycentrics[1] + n2 * barycentrics[2];
                }
            }

            if (0. <= min_t && min_t <= 1.) {
                // WILL ENTER THE OBJECT
                // std::cout << "POINT WILL ENTER: 0 <= " << min_t << " <= 1" << std::endl;
                addCollisionConstraint(pj, closest_intersection, glm::normalize(closest_normal));
                colliding_vertices.insert({pj, glm::normalize(closest_normal)});
            } else if (min_t < 0. && max_t > 1.) {
                // COMPLETLY INSIDE THE OBJECT
                // std::cout << "POINT INSIDE: " << min_t << " < 0 && " << max_t << " > 1" << std::endl;
                addCollisionConstraint(pj, closest_surface, glm::normalize(closest_surface_normal));
                colliding_vertices.insert({pj, glm::normalize(closest_surface_normal)});
            }
        }
    }

    for (const glm::uvec2 &edge : m_lines) {
        uint e0 = edge[0];
        uint e1 = edge[1];

        glm::dvec3 origin = new_positions[e0];
        glm::dvec3 direction = new_positions[e1] - new_positions[e0];

        for (const StaticBody &static_body : static_bodies) {
            const std::vector<glm::vec3> &mesh_positions = static_body.m_mesh->vertexPositions();
            const std::vector<glm::vec3> &mesh_normals = static_body.m_mesh->vertexNormals();
            const std::vector<glm::uvec3> &mesh_triangles = static_body.m_mesh->triangleIndices();
            glm::mat4 transformation = static_body.m_transformation->computeTransformationMatrix();

            double min_t = DBL_MAX, max_t = -DBL_MAX, t;
            glm::dvec3 closest_intersection, closest_normal, furthest_intersection, furthest_normal, intersection, barycentrics;

            for (const glm::uvec3 &triangle : mesh_triangles) {
                glm::dvec3 v0 = applyTransformation(mesh_positions[triangle[0]], 1.f, transformation);
                glm::dvec3 v1 = applyTransformation(mesh_positions[triangle[1]], 1.f, transformation);
                glm::dvec3 v2 = applyTransformation(mesh_positions[triangle[2]], 1.f, transformation);

                glm::dvec3 n0 = applyTransformation(mesh_normals[triangle[0]], 0.f, transformation);
                glm::dvec3 n1 = applyTransformation(mesh_normals[triangle[1]], 0.f, transformation);
                glm::dvec3 n2 = applyTransformation(mesh_normals[triangle[2]], 0.f, transformation);
                glm::dvec3 face_normal = glm::cross(v1 - v0, v2 - v0);

                if (!rayTriangleIntersection(origin, direction, v0, v1, v2, face_normal, t, intersection, barycentrics)) {
                    continue;
                }

                if (t < min_t) {
                    min_t = t;
                    closest_intersection = intersection;
                    closest_normal = barycentrics[0] * n0 + barycentrics[1] * n1 + barycentrics[2] * n2;
                }
                if (t > max_t) {
                    max_t = t;
                    furthest_intersection = intersection;
                    furthest_normal = barycentrics[0] * n0 + barycentrics[1] * n1 + barycentrics[2] * n2;
                }
            }

            if (min_t < 0. || min_t > 1. || max_t < 0. || max_t > 1.)
                continue;

            closest_normal = glm::normalize(closest_normal);
            furthest_normal = glm::normalize(furthest_normal);
            addEdgeCollisionConstraint(e0, e1, min_t, closest_intersection, closest_normal, max_t, furthest_intersection, furthest_normal);
            colliding_vertices.insert({e0, 0.5 * (1.0 - min_t) * closest_normal});
            colliding_vertices.insert({e0, 0.5 * (1.0 - max_t) * furthest_normal});
            colliding_vertices.insert({e1, 0.5 * min_t * closest_normal});
            colliding_vertices.insert({e1, 0.5 * max_t * furthest_normal});
            // accum(e0, closest_normal);
            // accum(e1, closest_normal);
        }
    }
    for (const StaticBody &static_body : static_bodies) {
        const std::vector<glm::vec3> &static_positions = static_body.m_mesh->vertexPositions();
        glm::mat4 transformation = static_body.m_transformation->computeTransformationMatrix();

        for (size_t i = 0; i < static_positions.size(); i++) {
            glm::dvec3 static_point = applyTransformation(static_positions[i], 1.f, transformation);

            for (const glm::uvec3 &triangle : m_triangles) {
                uint p0 = triangle[0];
                uint p1 = triangle[1];
                uint p2 = triangle[2];

                glm::dvec3 v0 = new_positions[p0];
                glm::dvec3 v1 = new_positions[p1];
                glm::dvec3 v2 = new_positions[p2];

                glm::dvec3 unnormalized_normal = glm::cross(v1 - v0, v2 - v0);
                if (glm::length(unnormalized_normal) < 1e-8)
                    continue;
                glm::dvec3 normal = glm::normalize(unnormalized_normal);

                // signed distance from static point to the new triangle plane
                double current_dist = glm::dot(static_point - v0, normal);
                glm::dvec3 project_on_plane = static_point - normal * current_dist;

                glm::dvec3 barycentrics;
                bool is_inside = computeBarycentrics(v0, v1, v2, unnormalized_normal, project_on_plane, barycentrics);

                // allow a margin for fast-moving edge crossings
                if (!is_inside) {
                    // too far from the triangle
                    if (barycentrics.x < -0.1 || barycentrics.y < -0.1 || barycentrics.z < -0.1)
                        continue;
                    // clamp for constraint evaluation
                    barycentrics.x = std::max(0.0, std::min(1.0, barycentrics.x));
                    barycentrics.y = std::max(0.0, std::min(1.0, barycentrics.y));
                    barycentrics.z = std::max(0.0, std::min(1.0, barycentrics.z));
                    double s = barycentrics.x + barycentrics.y + barycentrics.z;
                    barycentrics /= s;
                }

                // signed distance from static point to the old triangle plane
                glm::dvec3 old_v0 = m_positions[p0];
                glm::dvec3 old_v1 = m_positions[p1];
                glm::dvec3 old_v2 = m_positions[p2];
                glm::dvec3 old_unnorm_normal = glm::cross(old_v1 - old_v0, old_v2 - old_v0);
                double old_dist = 0.0;
                if (glm::length(old_unnorm_normal) > 1e-8) {
                    glm::dvec3 old_normal = glm::normalize(old_unnorm_normal);
                    old_dist = glm::dot(static_point - old_v0, old_normal);
                } else {
                    old_dist = current_dist;
                }

                // decide push direction
                double side_sign = (old_dist * current_dist < 0.0) ? old_dist : current_dist;
                glm::dvec3 push_normal = (side_sign > 0.0) ? -normal : normal;

                double thickness = 0.02;
                // proximity
                bool proximity = std::abs(current_dist) < thickness * 1.5;
                // simpple continue collision detection
                bool cross_frame = (old_dist * current_dist < 0.0) && std::abs(old_dist) > 1e-4;

                if (proximity || cross_frame) {
                    addStaticPointDynamicTriangleConstraint(p0, p1, p2, static_point, barycentrics, push_normal);
                    // Accumulate collision normals per vertex (average if multiple collisions)
                    colliding_vertices.insert({p0, push_normal / 3.});
                    colliding_vertices.insert({p1, push_normal / 3.});
                    colliding_vertices.insert({p2, push_normal / 3.});
                }
            }
        }
    }

    // (9)-(11)
    for (uint i = 0; i < _solver_iterations; i++) {
        for (uint ci = 0; ci < M + Mcoll; ci++) {
            // gather function input (and total weight)
            std::vector<glm::dvec3> affected_points(m_cardinalities[ci]);
            double total_weigths = 0.;
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                uint pj = m_indices[ci][i];
                affected_points[i] = new_positions[pj];
                total_weigths += m_weights[pj];
            }
            if (total_weigths == 0.) {
                continue;
            }

            if (total_weigths != total_weigths) {
                std::cerr << "Nan total weights..." << std::endl;
                return false;
            }

            double function_value = m_functions[ci](affected_points);
            if (m_types[ci] == INEQUALITY_CONSTRAINT && function_value >= 0.) {
                // The constraint is already satisfied so we don't project it
                continue;
            }

            // Determine S
            std::vector<glm::dvec3> gradients = m_gradients[ci](affected_points);
            double denominator = 0.;
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                denominator += glm::length2(gradients[i]);
            }
            if (denominator != denominator || denominator == 0.) {
                std::cerr << "invalid denominator=" << denominator << " for constraint " << ci << " of cardinality " << m_cardinalities[ci] << ". You may need to lower the deltaTime!" << std::endl;
                return false;
            }
            double s = function_value / denominator;

            // add the deltas
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                uint pj = m_indices[ci][i];
                glm::dvec3 delta_pj = -s * (double(m_cardinalities[ci]) * m_weights[pj] / total_weigths) * gradients[i];
                double k_prime = 1. - std::pow(1. - m_stiffnesses[ci], 1. / _solver_iterations);
                new_positions[pj] += k_prime * delta_pj;
            }
        }
    }

    // (12)-(15)
    for (uint pj = 0; pj < N; pj++) {
        m_velocities[pj] = (new_positions[pj] - m_positions[pj]) / _delta_time; // (13)
        m_positions[pj] = new_positions[pj];                                    // (14)
    }

    // (16)
    for (auto [pj, collision_normal] : colliding_vertices) {
        collision_normal = glm::normalize(collision_normal);

        // Decompose velocity into normal and tangential components
        double v_dot_n = glm::dot(m_velocities[pj], collision_normal);
        glm::dvec3 v_normal = v_dot_n * collision_normal;
        glm::dvec3 v_tangent = m_velocities[pj] - v_normal;

        // Apply restitution (reflection in the direction of collision normal)
        // Negative sign because we reflect away from surface
        v_normal *= -m_restitution_coefficient;

        // Apply friction (dampen velocity perpendicular to collision normal)
        v_tangent *= (1. - m_friction_coefficient);

        // Reconstruct velocity
        m_velocities[pj] = v_normal + v_tangent;
    }

    // cancel collision constraitns
    Mcoll = 0;
    m_cardinalities.resize(M);
    m_indices.resize(M);
    m_stiffnesses.resize(M);
    m_types.resize(M);
    m_functions.resize(M);
    m_gradients.resize(M);

    return true;
}

void DynamicObject::addVertex(const glm::dvec3 &_position, const glm::dvec3 &_velocity, double _mass, bool _fixed) {
    N++;
    N_fixed += _fixed ? 1 : 0;
    m_positions.push_back(_position);
    m_velocities.push_back(_velocity);
    m_masses.push_back(_mass);
    m_weights.push_back(_fixed ? 0. : 1. / _mass);
    m_fixed.push_back(_fixed);
}

void DynamicObject::addConstraint(
    uint _cardinality,
    const constraint_function &_function,
    const gradient_function &_gradient,
    const std::vector<uint> &_indices,
    double _stiffness,
    const ConstraintType &_type) {
    M++;
    m_cardinalities.push_back(_cardinality);
    m_functions.push_back(_function);
    m_gradients.push_back(_gradient);
    m_indices.push_back(_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(_type);
}

void DynamicObject::addDistanceConstraint(uint _p0, uint _p1, double _stiffness, double _targeted_distance) {
    M++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);

    m_functions.push_back([_targeted_distance](const std::vector<glm::dvec3> &_p) {
        return glm::distance(_p[0], _p[1]) - _targeted_distance;
    });
    m_gradients.push_back([](const std::vector<glm::dvec3> &_p) {
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

    m_functions.push_back([_targeted_angle](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 e = glm::normalize(_p[1] - _p[0]); // arête commune

        glm::dvec3 n1 = glm::normalize(glm::cross(_p[2] - _p[0], _p[2] - _p[1]));
        glm::dvec3 n2 = glm::normalize(glm::cross(_p[3] - _p[1], _p[3] - _p[0]));
        double cosTheta = glm::clamp(glm::dot(n1, n2), -1., 1.);
        double theta = acos(cosTheta);
        double sign = glm::dot(glm::cross(n1, n2), e);
        if (sign < 0)
            theta = -theta;
        return theta - _targeted_angle;
    });
    m_gradients.push_back([](const std::vector<glm::dvec3> &_p) {
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

void DynamicObject::addCollisionConstraint(uint _p0, glm::dvec3 _intersection, glm::dvec3 _normal) {
    Mcoll++;
    m_cardinalities.push_back(1);
    m_indices.push_back({_p0});
    m_stiffnesses.push_back(1.);
    m_types.push_back(INEQUALITY_CONSTRAINT);

    m_functions.push_back([_intersection, _normal](const std::vector<glm::dvec3> &_p) {
        return glm::dot(_p[0] - _intersection, _normal);
    });
    m_gradients.push_back([_normal](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{_normal};
    });
}

void DynamicObject::addEdgeCollisionConstraint(uint _p0, uint _p1,
                                               double _t1, glm::dvec3 _point1, glm::dvec3 _normal1,
                                               double _t2, glm::dvec3 _point2, glm::dvec3 _normal2) {
    double edge_radius = 0.05;

    // Create first contact constraint
    Mcoll++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(1.0);
    m_types.push_back(INEQUALITY_CONSTRAINT);

    m_functions.push_back([_t1, _point1, _normal1, edge_radius](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 edge_pt = (1.0 - _t1) * _p[0] + _t1 * _p[1];
        double dist = glm::dot(edge_pt - _point1, _normal1);
        return dist - edge_radius;
    });

    m_gradients.push_back([_t1, _normal1](const std::vector<glm::dvec3> &_p) {
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

    m_functions.push_back([_t2, _point2, _normal2, edge_radius](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 edge_pt = (1.0 - _t2) * _p[0] + _t2 * _p[1];
        double dist = glm::dot(edge_pt - _point2, _normal2);
        return dist - edge_radius;
    });

    m_gradients.push_back([_t2, _normal2](const std::vector<glm::dvec3> &_p) {
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

    double thickness = 0.01;

    m_functions.push_back([_static_point, _barycentrics, _normal, thickness](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 surface_pt = _barycentrics[0] * _p[0] + _barycentrics[1] * _p[1] + _barycentrics[2] * _p[2];
        return glm::dot(surface_pt - _static_point, _normal) - thickness;
    });

    m_gradients.push_back([_barycentrics, _normal](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{
            _barycentrics[0] * _normal,
            _barycentrics[1] * _normal,
            _barycentrics[2] * _normal};
    });
}

void DynamicObject::addVolumeConstraint(std::vector<glm::uvec3> _indices, double _stiffness, double _pressure, double _targeted_volume) {
    M++;
    m_cardinalities.push_back(getPositions().size());
    std::vector<uint> all_indices(N);
    std::iota(all_indices.begin(), all_indices.end(), 0);
    m_indices.push_back(all_indices);
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(EQUALITY_CONSTRAINT);

    m_functions.push_back([_targeted_volume, _pressure, _indices](const std::vector<glm::dvec3> &_p) {
        double V = 0;
        for (size_t i = 0; i < _indices.size(); i++) {
            const glm::dvec3 p1 = _p[_indices[i][0]];
            const glm::dvec3 p2 = _p[_indices[i][1]];
            const glm::dvec3 p3 = _p[_indices[i][2]];
            V += glm::dot(glm::cross(p1, p2), p3) / 6.;
        }
        return V - _pressure * _targeted_volume;
    });
    m_gradients.push_back([_indices](const std::vector<glm::dvec3> &_p) {
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

// Object interaction

void DynamicObject::findNearestPointToLine(const glm::dvec3 &_position, const glm::dvec3 &_direction, uint &point, double &distance, glm::dvec3 &projection) const {
    point = 0;
    distance = DBL_MAX;

    for (uint pj = 0; pj < N; pj++) {
        glm::dvec3 proj = projectPointOnLine(m_positions[pj], _position, _direction);
        double dist = glm::distance(m_positions[pj], proj);

        if (dist < distance) {
            point = pj;
            distance = dist;
            projection = proj;
        }
    }

    distance = distance;
}

bool DynamicObject::updateInteractions(GLFWwindow *_window, const glm::dvec3 &_camera_pos, const glm::dvec3 &_cursor_worldpos) {
    // Hovering
    glm::dvec3 cursor_direction = glm::normalize(_cursor_worldpos - _camera_pos);

    uint point;
    double distance;
    glm::dvec3 projection;
    findNearestPointToLine(_camera_pos, cursor_direction, point, distance, projection);
    hovered_point = distance < 0.3 ? point : UINT32_MAX;

    // Grabbing
    if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE || glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE) {
        if (grabbed_point != UINT32_MAX) {
            // std::cout << "finished grabbing point " << grabbed_point << "..." << std::endl;
            setVertexFixed(grabbed_point, grabbed_fixed);
            grabbed_point = UINT32_MAX;
            return true;
        }
        return false;
    }

    if (grabbed_point != UINT32_MAX) {
        // std::cout << "grabbing point " << grabbed_point << "..." << std::endl;
        m_positions[grabbed_point] = projectPointOnLine(m_positions[grabbed_point], _camera_pos, cursor_direction);
        return true;
    }

    if (distance < 0.3) {
        // std::cout << "grabbing point " << point << " with distance " << distance << std::endl;
        grabbed_point = point;
        m_positions[grabbed_point] = projection;
        grabbed_fixed = m_fixed[grabbed_point];
        setVertexFixed(grabbed_point, true);
        return true;
    }

    return false;
}

// OpenGL uinterface

void DynamicObject::initRendering() {
    // usual display
    glGenVertexArrays(1, &m_VAO);
    glBindVertexArray(m_VAO);

    glGenBuffers(1, &m_positions_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &m_lines_EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);

    glGenBuffers(1, &m_triangles_EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);

    // fixed particles
    glGenVertexArrays(1, &m_fixed_VAO);
    glBindVertexArray(m_fixed_VAO);

    glGenBuffers(1, &m_fixed_positions_VBO);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, m_fixed_positions_VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glVertexAttribDivisor(0, 1); // Advance once per instance

    updateRenderedPositions();
    updateRenderedConstraints();
}

void DynamicObject::updateRenderedPositions() {
    std::vector<glm::vec3> positions_float(N);

    std::vector<glm::vec3> fixed_particles_positions_data;
    std::vector<float> fixed_particles_sizes_data;
    fixed_particles_positions_data.reserve(N_fixed);
    fixed_particles_sizes_data.reserve(N_fixed);
    for (uint pj = 0; pj < N; pj++) {
        glm::vec3 float_pos = m_positions[pj];
        positions_float[pj] = float_pos;
        if (m_fixed[pj]) {
            fixed_particles_positions_data.push_back(float_pos);
        }
    }

    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, positions_float.size() * sizeof(glm::vec3), positions_float.data(), GL_DYNAMIC_DRAW);

    glBindVertexArray(m_fixed_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_fixed_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, fixed_particles_positions_data.size() * sizeof(glm::vec3), fixed_particles_positions_data.data(), GL_DYNAMIC_DRAW);
}

void DynamicObject::updateRenderedConstraints() {
    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_lines.size() * sizeof(glm::uvec2), m_lines.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_triangles.size() * sizeof(glm::uvec3), m_triangles.data(), GL_STATIC_DRAW);
}

void DynamicObject::render(DynamicRenderType _type) const {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    switch (_type) {
    case PointRender:
        glDrawArrays(GL_POINTS, 0, m_positions.size());
        break;
    case LineRender:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
        glDrawElements(GL_LINES, m_lines.size() * 2, GL_UNSIGNED_INT, 0);
        break;
    case TriangleRender:
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);
        glDrawElements(GL_TRIANGLES, m_triangles.size() * 3, GL_UNSIGNED_INT, 0);
        break;
    case Auto:
        if (!m_triangles.empty()) {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);
            glDrawElements(GL_TRIANGLES, m_triangles.size() * 3, GL_UNSIGNED_INT, 0);
        } else if (!m_lines.empty()) {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
            glDrawElements(GL_LINES, m_lines.size() * 2, GL_UNSIGNED_INT, 0);
        } else {
            glDrawArrays(GL_POINTS, 0, m_positions.size());
        }
        break;
    }
}

void DynamicObject::renderFixedVerices() const {
    glBindVertexArray(m_fixed_VAO);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(N_fixed));
}

void DynamicObject::renderHoveredVertex() const {
    if (hovered_point == UINT32_MAX)
        return;

    glm::vec3 hovered_position = m_positions[hovered_point];

    GLuint temp_VAO, temp_VBO;

    glGenVertexArrays(1, &temp_VAO);
    glBindVertexArray(temp_VAO);

    // buffer Creation
    glGenBuffers(1, &temp_VBO);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, temp_VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glVertexAttribDivisor(0, 1); // Advance once per instance

    // fill the buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3), glm::value_ptr(hovered_position), GL_DYNAMIC_DRAW);

    // render the buffer
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(1));
}

void DynamicObject::clear() {
    N = N_fixed = 0;
    m_positions.clear();
    m_velocities.clear();
    m_masses.clear();
    m_weights.clear();
    m_fixed.clear();

    M = Mcoll = 0;
    m_cardinalities.clear();
    m_functions.clear();
    m_gradients.clear();
    m_indices.clear();
    m_stiffnesses.clear();
    m_types.clear();
    m_lines.clear();
    m_triangles.clear();

    if (m_VAO) {
        glDeleteVertexArrays(1, &m_VAO);
        m_VAO = 0;
    }
    if (m_positions_VBO) {
        glDeleteBuffers(1, &m_positions_VBO);
        m_positions_VBO = 0;
    }
    if (m_lines_EBO) {
        glDeleteBuffers(1, &m_lines_EBO);
        m_lines_EBO = 0;
    }
    if (m_triangles_EBO) {
        glDeleteBuffers(1, &m_triangles_EBO);
        m_triangles_EBO = 0;
    }
}
