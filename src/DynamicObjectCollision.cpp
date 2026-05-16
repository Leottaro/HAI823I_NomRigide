#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/matrix.hpp>

#include "DynamicObject.hpp"
#include "AABB.hpp"
#include <iostream>

bool rayTriangleIntersection(const glm::dvec3& origin, const glm::dvec3& direction,
                             const glm::dvec3& v0, const glm::dvec3& v1, const glm::dvec3& v2, const glm::dvec3& normal,
                             double& t, glm::dvec3& intersection, glm::dvec3& barycentrics) {
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

template <size_t p1, size_t p2, size_t p3, size_t n>
size_t hash(size_t x, size_t y, size_t z) {
    return ((x * p1) ^ (y * p2) ^ (z * p3)) % n;
}

inline void accumulateCollisionsResponse(uint _pj, const glm::dvec3& _normal, std::unordered_map<uint, glm::dvec3>& _collisions_responses) {
    if (_collisions_responses.find(_pj) == _collisions_responses.end()) {
        _collisions_responses.insert({_pj, _normal});
    } else {
        _collisions_responses[_pj] += _normal;
    }
}
void DynamicObject::detectPointTriangleCollision(const std::vector<glm::dvec3>& new_positions, const std::vector<StaticBody>& static_bodies, std::unordered_map<uint, glm::dvec3>& _collisions_responses) {
    for (uint pj = 0; pj < N; pj++) {
        glm::dvec3 origin = m_positions[pj];
        glm::dvec3 direction = new_positions[pj] - m_positions[pj];

        for (const StaticBody& static_body : static_bodies) {
            const std::vector<glm::vec3>& mesh_positions = static_body.m_mesh->vertexPositions();
            const std::vector<glm::vec3>& mesh_normals = static_body.m_mesh->vertexNormals();
            const std::vector<glm::uvec3>& mesh_triangles = static_body.m_mesh->triangleIndices();
            glm::mat4 transformation = static_body.m_transformation->computeTransformationMatrix();

            double min_t = DBL_MAX, max_t = -DBL_MAX, t;
            glm::dvec3 closest_intersection, closest_normal, furthest_intersection, furthest_normal, intersection, barycentrics;

            double min_dist = DBL_MAX, dist;
            glm::dvec3 closest_surface, closest_surface_normal, surface;

            for (const glm::uvec3& triangle : mesh_triangles) {
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
                dist = closestPointInTriangle(m_positions[pj], v0, v1, v2, glm::normalize(triangle_normal), surface, barycentrics);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_surface = surface;
                    closest_surface_normal = n0 * barycentrics[0] + n1 * barycentrics[1] + n2 * barycentrics[2];
                }
            }

            if (0. <= min_t && min_t <= 1.) {
                // WILL ENTER THE OBJECT
                // std::cout << "POINT WILL ENTER: 0 <= " << min_t << " <= 1" << std::endl;
                closest_normal = glm::normalize(closest_normal);
                addCollisionConstraint(pj, closest_intersection, closest_normal);
                accumulateCollisionsResponse(pj, closest_normal, _collisions_responses);
            } else if (min_t < 0. && max_t > 1.) {
                // COMPLETLY INSIDE THE OBJECT
                // std::cout << "POINT INSIDE: " << min_t << " < 0 && " << max_t << " > 1" << std::endl;
                closest_surface_normal = glm::normalize(closest_surface_normal);
                addCollisionConstraint(pj, closest_surface, closest_surface_normal);
                accumulateCollisionsResponse(pj, closest_surface_normal, _collisions_responses);
            }
        }
    }
}
void DynamicObject::detectEdgeEdgeCollision(const std::vector<glm::dvec3>& new_positions, const std::vector<StaticBody>& static_bodies, std::unordered_map<uint, glm::dvec3>& _collisions_responses) {
    for (const glm::uvec2& edge : m_lines) {
        uint e0 = edge[0];
        uint e1 = edge[1];

        glm::dvec3 origin = new_positions[e0];
        glm::dvec3 direction = new_positions[e1] - new_positions[e0];

        for (const StaticBody& static_body : static_bodies) {
            const std::vector<glm::vec3>& mesh_positions = static_body.m_mesh->vertexPositions();
            const std::vector<glm::vec3>& mesh_normals = static_body.m_mesh->vertexNormals();
            const std::vector<glm::uvec3>& mesh_triangles = static_body.m_mesh->triangleIndices();
            glm::mat4 transformation = static_body.m_transformation->computeTransformationMatrix();

            double min_t = DBL_MAX, max_t = -DBL_MAX, t;
            glm::dvec3 closest_intersection, closest_normal, furthest_intersection, furthest_normal, intersection, barycentrics;

            for (const glm::uvec3& triangle : mesh_triangles) {
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
            accumulateCollisionsResponse(e0, 0.5 * glm::normalize((1.0 - min_t) * closest_normal + (1.0 - max_t) * furthest_normal), _collisions_responses);
            accumulateCollisionsResponse(e1, 0.5 * glm::normalize(min_t * closest_normal + max_t * furthest_normal), _collisions_responses);
        }
    }
}
void DynamicObject::detectTrianglePointCollision(const std::vector<glm::dvec3>& new_positions, const std::vector<StaticBody>& static_bodies, std::unordered_map<uint, glm::dvec3>& _collisions_responses) {
    for (const StaticBody& static_body : static_bodies) {
        const std::vector<glm::vec3>& static_positions = static_body.m_mesh->vertexPositions();
        glm::mat4 transformation = static_body.m_transformation->computeTransformationMatrix();

        for (size_t i = 0; i < static_positions.size(); i++) {
            glm::dvec3 static_point = applyTransformation(static_positions[i], 1.f, transformation);

            for (const glm::uvec3& triangle : m_triangles) {
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

                // proximity
                bool proximity = std::abs(current_dist) < m_surface_thickness * 1.5;
                // simpple continue collision detection
                bool cross_frame = (old_dist * current_dist < 0.0) && std::abs(old_dist) > 1e-4;

                if (proximity || cross_frame) {
                    addStaticPointDynamicTriangleConstraint(p0, p1, p2, static_point, barycentrics, push_normal);
                    // Accumulate collision normals per vertex (average if multiple collisions)
                    push_normal /= 3.;
                    accumulateCollisionsResponse(p0, push_normal, _collisions_responses);
                    accumulateCollisionsResponse(p1, push_normal, _collisions_responses);
                    accumulateCollisionsResponse(p2, push_normal, _collisions_responses);
                }
            }
        }
    }
}

void DynamicObject::detectSelfPointTriangleCollision(const std::vector<glm::dvec3>& new_positions, std::unordered_map<uint, glm::dvec3>& _collisions_responses) {
    for (uint q = 0; q < N; q++) {
        const glm::dvec3& origin = m_positions[q];
        const glm::dvec3 direction = new_positions[q] - m_positions[q];

        for (uint ti = 0; ti < m_triangles.size(); ti++) {
            uint p1 = m_triangles[ti][0];
            uint p2 = m_triangles[ti][1];
            uint p3 = m_triangles[ti][2];

            // Skip immediate neighbors
            if (q == p1 || q == p2 || q == p3)
                continue;

            // Broad phase: AABB
            AABB<double> aabb;
            for (uint i = 0; i < 3; i++) {
                aabb.addPosition(m_positions[m_triangles[ti][i]]);
                aabb.addPosition(new_positions[m_triangles[ti][i]]);
            }
            aabb.expand(m_surface_thickness);
            if (!aabb.intersect(origin, direction))
                continue;

            // Compute initial plane normal at t=0
            glm::dvec3 n_init = glm::cross(m_positions[p2] - m_positions[p1], m_positions[p3] - m_positions[p1]);
            double n_init_len = glm::length(n_init);

            double prev_dist = std::numeric_limits<double>::max();
            double prev_signed_dist = 0.0;

            // Accurately initialize prev_signed_dist BEFORE the loop to avoid the 0.0 trap
            if (n_init_len > 1e-12) {
                n_init /= n_init_len;
                // Using (plane_point - point) to maintain your original sign polarity:
                // Negative = In Front, Positive = Behind
                prev_signed_dist = glm::dot(m_positions[p1] - m_positions[q], n_init);
            }

            glm::dvec3 surface, barycentrics;

            // 15 steps is plenty for numerical CCD. 10,000 will destroy your framerate.
            const uint CCD_STEPS = 100;

            for (uint i = 0; i <= CCD_STEPS; ++i) {
                double t = double(i) / double(CCD_STEPS);
                glm::dvec3 qt = glm::lerp(m_positions[q], new_positions[q], t);
                glm::dvec3 p1t = glm::lerp(m_positions[p1], new_positions[p1], t);
                glm::dvec3 p2t = glm::lerp(m_positions[p2], new_positions[p2], t);
                glm::dvec3 p3t = glm::lerp(m_positions[p3], new_positions[p3], t);
                glm::dvec3 normal = glm::cross(p2t - p1t, p3t - p1t);
                double normal_len = glm::length(normal);
                if (normal_len < 1e-12)
                    continue;
                glm::dvec3 n = normal / normal_len;

                double dist = closestPointInTriangle(qt, p1t, p2t, p3t, n, surface, barycentrics);
                double signed_dist = glm::dot(p1t - qt, n); // negative <=> in front

                bool crossing = (prev_signed_dist > 1e-8 && signed_dist < -1e-8) || (prev_signed_dist < -1e-8 && signed_dist > 1e-8); // Only triggers if it definitively passed through the plane, ignoring float noise
                bool valid_crossing = crossing && (dist < m_surface_thickness * 1.5);                                                 // Ensure the crossing actually happened inside the bounds of the triangle edges
                bool swept_proximity = (prev_dist > m_surface_thickness && dist <= m_surface_thickness);                              // Only triggers if the point actively ENTERED the thickness shell
                if (swept_proximity) {
                    bool from_behind = signed_dist > 0.0;
                    addSelfCollisionConstraint(q, p1, from_behind ? p3 : p2, from_behind ? p2 : p3);
                    std::cout << "SELF COLLISION" << std::endl;
                    break;
                }

                prev_dist = dist;
                prev_signed_dist = signed_dist;
            }
        }
    }
}