#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/matrix.hpp>

#include "DynamicObject.hpp"
#include <iostream>
#include <unordered_map>

double length2(const glm::dvec3 &vec) {
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

bool computeBarycentrics(const glm::dvec3 &v0, const glm::dvec3 &v1, const glm::dvec3 &v2, const glm::dvec3 &normal, const glm::dvec3 &p, glm::dvec3 &barycentrics) {
    double total_area = glm::length(normal); // this is actually the 2 times the area but it doesn't matter for the barycentric coordinates
    barycentrics.x = glm::length(glm::cross(v1 - p, v2 - p)) / total_area - 1.e-8;
    barycentrics.y = glm::length(glm::cross(p - v0, v2 - v0)) / total_area - 1.e-8;
    barycentrics.z = glm::length(glm::cross(v1 - v0, p - v0)) / total_area - 1.e-8;
    if (barycentrics.x < 0. || 1. < barycentrics.x ||
        barycentrics.y < 0. || 1. < barycentrics.y ||
        barycentrics.z < 0. || 1. < barycentrics.z ||
        barycentrics.x + barycentrics.y + barycentrics.z < 0. || 1. < barycentrics.x + barycentrics.y + barycentrics.z) {
        return false;
    }

    return true;
}

double rayTriangleIntersection(const glm::dvec3 &origin, const glm::dvec3 &direction,
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
        float air_friction = 0.001f;
        for (uint i = 0; i < N; i++) {
            if (!m_fixed[i]) {
                m_velocities[i] *= (1.0f - air_friction);
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

// TODO: imgui solver settings
#define SOLVER_ITERATIONS 100

bool DynamicObject::update(double _delta_time, const std::vector<StaticBody> &static_bodies, const std::vector<DynamicObject*>& dynamic_bodies) {
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
    double collision_radius = 0.1;
    std::unordered_map<size_t, glm::dvec3> colliding_vertices;
    for (uint pj = 0; pj < N; pj++) {
        glm::dvec3 origin = m_positions[pj];
        glm::dvec3 direction = new_positions[pj] - m_positions[pj];

        for (DynamicObject* other : dynamic_bodies) {

            if (other == this || m_fixed[pj]) continue;

            for (const glm::uvec2& edge : other->m_lines) {

                glm::dvec3 a0 = other->m_positions[edge[0]];
                glm::dvec3 a1 = other->m_positions[edge[1]];

                glm::dvec3 d2 = a1 - a0;

                glm::dvec3 r = origin - a0;

                double a_dot = glm::dot(direction, direction); // ditance^2 entre poisition et new_position
                double e_dot = glm::dot(d2, d2); // distance^2 entre les 2 points de l'arête
                double f = glm::dot(d2, r);

                double s, t; // origin + s * direction et a0 + t * d2 points les plus proches sur les deux segments

                if (a_dot <= 1e-8 && e_dot <= 1e-8) continue; // si les deux segments sont trop petits

                if (a_dot <= 1e-8) { // si le mouvement du point est trop petit
                    s = 0.0;
                    t = glm::clamp(f / e_dot, 0.0, 1.0);
                } else {
                    double c = glm::dot(direction, r);

                    if (e_dot <= 1e-8) { // si l'arête est trop petite
                        t = 0.0;
                        s = glm::clamp(-c / a_dot, 0.0, 1.0);
                    } else {
                        double b_dot = glm::dot(direction, d2);
                        double denom = a_dot * e_dot - b_dot * b_dot;

                        if (denom != 0.0) // si les segments ne sont pas parallèles
                            s = glm::clamp((b_dot * f - c * e_dot) / denom, 0.0, 1.0);
                        else
                            s = 0.0;

                        t = (b_dot * s + f) / e_dot;

                        if (t < 0.0) { // si t est en dehors de l'arête, on le clamp sur 0 ou 1 et on recalcule s
                            t = 0.0;
                            s = glm::clamp(-c / a_dot, 0.0, 1.0);
                        } else if (t > 1.0) { // si t est en dehors de l'arête, on le clamp sur 0 ou 1 et on recalcule s
                            t = 1.0;
                            s = glm::clamp((b_dot - c) / a_dot, 0.0, 1.0);
                        }
                    }
                }

                glm::dvec3 closest_p = origin + s * direction; // plus proche sur le mouvement du vertex
                glm::dvec3 closest_e = a0 + t * d2; // plus proche sur l'arête

                glm::dvec3 delta = closest_p - closest_e;
                double dist = glm::length(delta);

                if (dist < collision_radius && dist > 1e-8) {

                    //glm::dvec3 normal = glm::normalize(delta);
                    glm::vec3 normal = glm::dvec3(0., 1., 0.);
                    glm::dvec3 contact = closest_e;

                    // if(normal.y < 0.) {
                    //     std::cout << normal.x << " " << normal.y << " " << normal.z << std::endl;
                    // }

                    addCollisionConstraint(pj, contact, normal, 1.0);
                    colliding_vertices.insert({pj, normal});
                }
            }
        }

        for (const StaticBody &static_body : static_bodies) {
            const std::vector<glm::vec3> &mesh_positions = static_body.m_mesh->vertexPositions();
            const std::vector<glm::vec3> &mesh_normals = static_body.m_mesh->vertexNormals();
            const std::vector<glm::uvec3> &mesh_triangles = static_body.m_mesh->triangleIndices();
            glm::mat4 transformation = static_body.m_transformation->computeTransformationMatrix();

            double min_t = DBL_MAX, max_t = -DBL_MAX, t;
            glm::dvec3 closest_intersection, closest_normal, intersection, barycentrics;

            double min_dist = DBL_MAX, dist;
            glm::dvec3 closest_surface, closest_surface_normal, surface;

            for (const glm::uvec3 &triangle : mesh_triangles) {
                glm::dvec3 v0 = applyTransformation(mesh_positions[triangle[0]], 1.f, transformation);
                glm::dvec3 v1 = applyTransformation(mesh_positions[triangle[1]], 1.f, transformation);
                glm::dvec3 v2 = applyTransformation(mesh_positions[triangle[2]], 1.f, transformation);

                glm::dvec3 n0 = applyTransformation(mesh_normals[triangle[0]], 0.f, transformation);
                glm::dvec3 n1 = applyTransformation(mesh_normals[triangle[1]], 0.f, transformation);
                glm::dvec3 n2 = applyTransformation(mesh_normals[triangle[2]], 0.f, transformation);
                glm::dvec3 normal = glm::cross(v1 - v0, v2 - v0);

                // ray intersections
                if (!rayTriangleIntersection(origin, direction, v0, v1, v2, normal, t, intersection, barycentrics)) {
                    continue;
                }
                if (t < min_t) {
                    min_t = t;
                    closest_intersection = intersection;
                    closest_normal = barycentrics[0] * n0 + barycentrics[1] * n1 + barycentrics[2] * n2;
                }

                // closest surface
                glm::dvec3 project_on_plane = glm::cross(normal, glm::cross(new_positions[pj] - v0, normal));
                computeBarycentrics(v0, v1, v2, normal, project_on_plane, barycentrics);

                // https://www.desmos.com/calculator/eeqkstj2ck
                const auto fallback_in_triangle = [project_on_plane](const glm::dvec3 &p1, const glm::dvec3 &p2) {
                    glm::dvec3 direction = p2 - p1;
                    double n_squared = std::pow(glm::distance(p2, p1), 2);
                    double dot = glm::dot(direction, project_on_plane - p1);
                    double dot_over_one = dot / n_squared;
                    double r = std::max(0., std::min(1., dot_over_one));
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
                addCollisionConstraint(pj, closest_intersection, glm::normalize(closest_normal), 1.);
                colliding_vertices.insert({pj, glm::normalize(closest_normal)});
            } else if (min_t < 0. && max_t > 1.) {
                // COMPLETLY INSIDE THE OBJECT
                addCollisionConstraint(pj, closest_surface, glm::normalize(closest_surface_normal), 1.);
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

            double min_t = DBL_MAX, t;
            glm::dvec3 closest_intersection, closest_normal, intersection, barycentrics;
            bool hit = false;

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

                if (t >= 0.0 && t <= 1.0 && t < min_t) {
                    min_t = t;
                    closest_intersection = intersection;
                    closest_normal = glm::normalize(barycentrics[0] * n0 + barycentrics[1] * n1 + barycentrics[2] * n2);
                    hit = true;
                }
            }

            if (hit) {
                addEdgeCollisionConstraint(e0, e1, min_t, closest_intersection, closest_normal, 1.0);
            }
        }

        // double m_collision_radius = 0.05f;
        // for (DynamicObject* other : dynamic_bodies) {

        //     if (other == this) continue;

        //     for (uint i = 0; i < N; i++) {
        //         if (m_fixed[i]) continue;

        //         glm::dvec3 p0 = m_positions[i];     // position ancienne
        //         glm::dvec3 p1 = new_positions[i];   // position nouvelle

        //         for (const glm::uvec2& edge : other->m_lines) {

        //             glm::dvec3 a = other->m_positions[edge[0]];
        //             glm::dvec3 b = other->m_positions[edge[1]];

        //             // -------- SEGMENT vs SEGMENT distance --------
        //             glm::dvec3 d1 = p1 - p0;
        //             glm::dvec3 d2 = b - a;
        //             glm::dvec3 r = p0 - a;

        //             double a_dot = glm::dot(d1, d1);
        //             double e_dot = glm::dot(d2, d2);
        //             double f = glm::dot(d2, r);

        //             double s, t;

        //             if (a_dot <= 1e-8 && e_dot <= 1e-8) continue;

        //             if (a_dot <= 1e-8) {
        //                 s = 0.0;
        //                 t = glm::clamp(f / e_dot, 0.0, 1.0);
        //             } else {
        //                 double c = glm::dot(d1, r);
        //                 if (e_dot <= 1e-8) {
        //                     t = 0.0;
        //                     s = glm::clamp(-c / a_dot, 0.0, 1.0);
        //                 } else {
        //                     double b_dot = glm::dot(d1, d2);
        //                     double denom = a_dot * e_dot - b_dot * b_dot;

        //                     if (denom != 0.0)
        //                         s = glm::clamp((b_dot * f - c * e_dot) / denom, 0.0, 1.0);
        //                     else
        //                         s = 0.0;

        //                     t = (b_dot * s + f) / e_dot;

        //                     if (t < 0.0) {
        //                         t = 0.0;
        //                         s = glm::clamp(-c / a_dot, 0.0, 1.0);
        //                     } else if (t > 1.0) {
        //                         t = 1.0;
        //                         s = glm::clamp((b_dot - c) / a_dot, 0.0, 1.0);
        //                     }
        //                 }
        //             }

        //             glm::dvec3 closest_p = p0 + s * d1;
        //             glm::dvec3 closest_e = a + t * d2;

        //             glm::dvec3 delta = closest_p - closest_e;
        //             double dist = glm::length(delta);

        //             if (dist < m_collision_radius && dist > 1e-8) {

        //                 glm::dvec3 normal = glm::normalize(delta);
        //                 glm::dvec3 contact = closest_e;

        //                 addCollisionConstraint(i, contact, normal, 1.0);
        //                 colliding_vertices.insert({i, normal});

        //                 if(normal.y > 0.) {
        //                     std::cout << normal.x << " " << normal.y << " " << normal.z << std::endl;
        //                 }
        //             }
        //         }
        //     }
        // }
    }



    // (9)-(11)
    for (uint i = 0; i < SOLVER_ITERATIONS; i++) {
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
                denominator += length2(gradients[i]);
            }
            if(denominator == 0.) {
                continue;
            }
            if (denominator != denominator) {
                std::cerr << "invalid denominator : " << denominator << std::endl;
                return false;
            }
        
            double s = function_value / denominator;

            // add the deltas
            for (uint i = 0; i < m_cardinalities[ci]; i++) {
                uint pj = m_indices[ci][i];
                glm::dvec3 delta_pj = -s * (double(m_cardinalities[ci]) * m_weights[pj] / total_weigths) * gradients[i];
                double k_prime = 1. - std::pow(1. - m_stiffnesses[ci], 1. / SOLVER_ITERATIONS);
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
    m_positions.push_back(_position);
    m_velocities.push_back(_velocity);
    m_masses.push_back(_mass);
    m_weights.push_back(_fixed ? 0. : 1. / _mass);
    m_fixed.push_back(_fixed);
}

void DynamicObject::setVertexFixed(uint _pj, bool _fixed) {
    m_fixed[_pj] = _fixed;
    m_weights[_pj] = _fixed ? 0. : 1. / m_masses[_pj];
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
        double n1sq = length2(n1);
        double n2sq = length2(n2);

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

void DynamicObject::addCollisionConstraint(uint _p0, glm::dvec3 _intersection, glm::dvec3 _normal, double _stiffness) {
    Mcoll++;
    m_cardinalities.push_back(1);
    m_indices.push_back({_p0});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(INEQUALITY_CONSTRAINT);

    m_functions.push_back([_intersection, _normal](const std::vector<glm::dvec3> &_p) {
        return glm::dot(_p[0] - _intersection, _normal);
    });
    m_gradients.push_back([_normal](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{_normal};
    });
}

void DynamicObject::addEdgeCollisionConstraint(uint _p0, uint _p1, double _alpha, glm::dvec3 _surface_point, glm::dvec3 _normal, double _stiffness) {
    Mcoll++;
    m_cardinalities.push_back(2);
    m_indices.push_back({_p0, _p1});
    m_stiffnesses.push_back(_stiffness);
    m_types.push_back(INEQUALITY_CONSTRAINT);

    double edge_radius = 0.05;

    m_functions.push_back([_alpha, _surface_point, _normal, edge_radius](const std::vector<glm::dvec3> &_p) {
        glm::dvec3 edge_pt = (1.0 - _alpha) * _p[0] + _alpha * _p[1];
        return glm::dot(edge_pt - _surface_point, _normal) - edge_radius;
    });

    m_gradients.push_back([_alpha, _normal](const std::vector<glm::dvec3> &_p) {
        return std::vector<glm::dvec3>{
            (1.0 - _alpha) * _normal,
            _alpha * _normal};
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
        glm::dvec3 proj = projectPointOnLine(_position, _direction, m_positions[pj]);
        double dist = glm::distance(m_positions[pj], proj);

        if (dist < distance) {
            point = pj;
            distance = dist;
            projection = proj;
        }
    }

    distance = sqrt(distance);
}

bool DynamicObject::updateInteractions(GLFWwindow *_window, const glm::dvec3 &_camera_pos, const glm::dvec3 &_cursor_worldpos) {
    if (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE || glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_RELEASE) {
        if (grabbed_point != UINT32_MAX) {
            std::cout << "finished grabbing point " << grabbed_point << "..." << std::endl;
            setVertexFixed(grabbed_point, grabbed_fixed);
            grabbed_point = UINT32_MAX;
            return true;
        }
        return false;
    }

    glm::dvec3 cursor_direction = glm::normalize(_cursor_worldpos - _camera_pos);
    if (grabbed_point != UINT32_MAX) {
        std::cout << "grabbing point " << grabbed_point << "..." << std::endl;
        m_positions[grabbed_point] = projectPointOnLine(_camera_pos, cursor_direction, m_positions[grabbed_point]);
        return true;
    }

    uint point;
    double distance;
    glm::dvec3 projection;
    findNearestPointToLine(_camera_pos, cursor_direction, point, distance, projection);
    if (distance < 0.3) {
        std::cout << "grabbing point " << point << " with distance " << distance << std::endl;
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
    glGenVertexArrays(1, &m_VAO);
    glBindVertexArray(m_VAO);

    glGenBuffers(1, &m_positions_VBO);
    updateRenderedPositions();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &m_lines_EBO);
    updateRenderedConstraints();

    glBindVertexArray(0);
}

void DynamicObject::updateRenderedPositions() {
    std::vector<glm::vec3> positions_float(m_positions.begin(), m_positions.end());
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, positions_float.size() * sizeof(glm::vec3), positions_float.data(), GL_DYNAMIC_DRAW);
}

void DynamicObject::updateRenderedConstraints() {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_lines_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_lines.size() * sizeof(glm::uvec2), m_lines.data(), GL_STATIC_DRAW);
}

void DynamicObject::render() const {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    if (m_lines.empty()) {
        glDrawArrays(GL_POINTS, 0, m_positions.size());
    } else {
        glDrawElements(GL_LINES, m_lines.size() * 2, GL_UNSIGNED_INT, 0);
    }
}

void DynamicObject::clear() {
    N = 0;
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
}
