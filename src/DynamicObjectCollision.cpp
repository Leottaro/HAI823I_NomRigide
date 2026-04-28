#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/matrix.hpp>

#include "DynamicObject.hpp"
#include "AABB.hpp"

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

template <size_t p1, size_t p2, size_t p3, size_t n>
size_t hash(size_t x, size_t y, size_t z) {
    return ((x * p1) ^ (y * p2) ^ (z * p3)) % n;
}

inline void accumulateCollisionsResponse(uint _pj, const glm::dvec3 &_normal, std::unordered_map<uint, glm::dvec3> &_collisions_responses) {
    if (_collisions_responses.find(_pj) == _collisions_responses.end()) {
        _collisions_responses.insert({_pj, _normal});
    } else {
        _collisions_responses[_pj] += _normal;
    }
}
void DynamicObject::detectPointTriangleCollision(const std::vector<glm::dvec3> &new_positions, const std::vector<StaticBody> &static_bodies, std::unordered_map<uint, glm::dvec3> &_collisions_responses) {
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
void DynamicObject::detectEdgeEdgeCollision(const std::vector<glm::dvec3> &new_positions, const std::vector<StaticBody> &static_bodies, std::unordered_map<uint, glm::dvec3> &_collisions_responses) {
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
            accumulateCollisionsResponse(e0, 0.5 * glm::normalize((1.0 - min_t) * closest_normal + (1.0 - max_t) * furthest_normal), _collisions_responses);
            accumulateCollisionsResponse(e1, 0.5 * glm::normalize(min_t * closest_normal + max_t * furthest_normal), _collisions_responses);
        }
    }
}
void DynamicObject::detectTrianglePointCollision(const std::vector<glm::dvec3> &new_positions, const std::vector<StaticBody> &static_bodies, std::unordered_map<uint, glm::dvec3> &_collisions_responses) {
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
void DynamicObject::detectSelfPointTriangleCollision(const std::vector<glm::dvec3> &new_positions, std::unordered_map<uint, glm::dvec3> &_collisions_responses) {
    for (uint q = 0; q < N; q++) {
        for (uint ti = 0; ti < m_triangles.size(); ti++) {
            uint p1 = m_triangles[ti][0];
            uint p2 = m_triangles[ti][1];
            uint p3 = m_triangles[ti][2];
            if (q == p1 || q == p2 || q == p3)
                continue;

            // Borad phase: AABB
            AABB<double> aabb;
            for (uint i = 0; i < 3; i++) {
                aabb.addPosition(m_positions[m_triangles[ti][i]]);
                aabb.addPosition(new_positions[m_triangles[ti][i]]);
            }
            aabb.expand(m_surface_thickness); // Expand AABB by m_surface_thickness so nearby-but-not-crossing cases are caught
            const glm::dvec3 &origin = m_positions[q];
            const glm::dvec3 direction = new_positions[q] - m_positions[q];
            if (!aabb.intersect(origin, direction))
                continue;

            glm::dvec3 barycentrics;
            for (uint i = 1; i <= 10000; i++) { // TODO: better idea ?
                double t_prev = double(i - 1) / 10000.0;
                double t_curr = double(i) / 10000.0;

                glm::dvec3 qt_prev = glm::lerp(m_positions[q], new_positions[q], t_prev);
                glm::dvec3 p1t_prev = glm::lerp(m_positions[p1], new_positions[p1], t_prev);
                glm::dvec3 p2t_prev = glm::lerp(m_positions[p2], new_positions[p2], t_prev);
                glm::dvec3 p3t_prev = glm::lerp(m_positions[p3], new_positions[p3], t_prev);

                glm::dvec3 qt = glm::lerp(m_positions[q], new_positions[q], t_curr);
                glm::dvec3 p1t = glm::lerp(m_positions[p1], new_positions[p1], t_curr);
                glm::dvec3 p2t = glm::lerp(m_positions[p2], new_positions[p2], t_curr);
                glm::dvec3 p3t = glm::lerp(m_positions[p3], new_positions[p3], t_curr);

                glm::dvec3 n_prev = glm::cross(p2t_prev - p1t_prev, p3t_prev - p1t_prev);
                glm::dvec3 n_curr = glm::cross(p2t - p1t, p3t - p1t);

                double d_prev = glm::dot(qt_prev - p1t_prev, n_prev);
                double d_curr = glm::dot(qt - p1t, n_curr);

                double n_len_prev = glm::length(n_prev);
                double n_len_curr = glm::length(n_curr);
                if (n_len_prev < 1e-8 || n_len_curr < 1e-8)
                    continue;

                // Signed distances (in world units, not scaled by |n|)
                double sd_prev = d_prev / n_len_prev;
                double sd_curr = d_curr / n_len_curr;

                // Trigger when either:
                //   (a) the vertex crosses the triangle plane (sign change), OR
                //   (b) the vertex enters the thickness shell (|sd| < thickness)
                bool crossing = (sd_prev * sd_curr < 0.0);
                bool proximity = (std::abs(sd_curr) < m_surface_thickness);

                if (crossing || proximity) {
                    if (computeBarycentrics(p1t, p2t, p3t, n_curr, qt, barycentrics)) {
                        bool from_behind = sd_prev > 0.0;
                        // std::cout << "self collision: " << q << " with ("
                        //           << p1 << ", "
                        //           << (from_behind ? p2 : p3) << ", "
                        //           << (from_behind ? p3 : p2) << ")" << std::endl;
                        addSelfCollisionConstraint(q, p1,
                                                   from_behind ? p2 : p3,
                                                   from_behind ? p3 : p2);
                        break;
                    }
                }
            }
        }
    }
}