#include <poly34.h>

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

void DynamicObject::detectSelfPointTriangleCollision(const std::vector<glm::dvec3>& new_positions, const std::vector<DynamicObject*>& dynamic_objects, std::vector<std::unordered_map<uint, glm::dvec3>>& _collisions_responses) {
    uint self_i = 0;
    for (uint object_i = 0; object_i < dynamic_objects.size(); object_i++) {
        if (dynamic_objects[object_i] == this) {
            self_i = object_i;
            break;
        }
    }

    for (uint q = 0; q < N; q++) {
        const glm::dvec3 dq = new_positions[q] - m_positions[q];

        for (uint object_i = 0; object_i < dynamic_objects.size(); object_i++) {
            DynamicObject* obj = dynamic_objects[object_i];
            for (uint ti = 0; ti < obj->m_triangles.size(); ti++) {
                uint p1 = obj->m_triangles[ti][0];
                uint p2 = obj->m_triangles[ti][1];
                uint p3 = obj->m_triangles[ti][2];

                // Skip immediate neighbors
                if (q == p1 || q == p2 || q == p3)
                    continue;

                // Broad phase: AABB
                AABB<double> aabb;
                for (uint i = 0; i < 3; i++) {
                    aabb.addPosition(m_positions[obj->m_triangles[ti][i]]);
                    aabb.addPosition(new_positions[obj->m_triangles[ti][i]]);
                }
                aabb.expand(obj->m_surface_thickness);
                if (!aabb.intersect(m_positions[q], dq))
                    continue;

                // Pre-calculate t=0 normal vector for side determination
                glm::dvec3 N0 = glm::cross(m_positions[p2] - m_positions[p1], m_positions[p3] - m_positions[p1]);
                // Determine side from the initial state (t=0) to prevent the 0.0 sign bug
                double signed_dist_init = glm::dot(m_positions[q] - m_positions[p1], N0);
                bool from_behind = signed_dist_init > 0.0;

                // ==========================================
                // THICKNESS FIX 1: Proximity check at t = 1
                // ==========================================
                // Catches slow-moving objects entering the shell without crossing the mid-plane
                glm::dvec3 normal_end = glm::cross(new_positions[p2] - new_positions[p1], new_positions[p3] - new_positions[p1]);
                double normal_end_len = glm::length(normal_end);
                if (normal_end_len > 1e-12) {
                    glm::dvec3 n_end = normal_end / normal_end_len;
                    glm::dvec3 surface_end, bary_end;
                    double dist_end = closestPointInTriangle(new_positions[q], new_positions[p1], new_positions[p2], new_positions[p3], n_end, surface_end, bary_end);

                    if (dist_end <= obj->m_surface_thickness) {
                        addSelfCollisionConstraint(q, p1, from_behind ? p2 : p3, from_behind ? p3 : p2);
                        continue; // Handled, skip the cubic solver for this pair
                    }
                }

                // ==========================================
                // Continuous Component: Cubic Solver (Tunneling)
                // ==========================================
                glm::dvec3 dp1 = new_positions[p1] - m_positions[p1];
                glm::dvec3 dp2 = new_positions[p2] - m_positions[p2];
                glm::dvec3 dp3 = new_positions[p3] - m_positions[p3];

                // A(t) = A0 + t*A1 (Edge 1)
                glm::dvec3 A0 = m_positions[p2] - m_positions[p1];
                glm::dvec3 A1 = dp2 - dp1;
                // B(t) = B0 + t*B1 (Edge 2)
                glm::dvec3 B0 = m_positions[p3] - m_positions[p1];
                glm::dvec3 B1 = dp3 - dp1;
                // C(t) = C0 + t*C1 (Point to Triangle)
                glm::dvec3 C0 = m_positions[q] - m_positions[p1];
                glm::dvec3 C1 = dq - dp1;

                // Expand the cross product (A(t) x B(t))
                glm::dvec3 N1 = glm::cross(A0, B1) + glm::cross(A1, B0);
                glm::dvec3 N2 = glm::cross(A1, B1);

                // Assemble Cubic Coefficients from C(t) . N(t) = 0
                double a = glm::dot(C1, N2);
                double b = glm::dot(C0, N2) + glm::dot(C1, N1);
                double c = glm::dot(C0, N1) + glm::dot(C1, N0);
                double d = glm::dot(C0, N0);

                // Solve for time 't'
                double roots[3];
                int num_roots = SolveP3(roots, b / a, c / a, d / a);

                // Test valid roots to see if they occurred inside/near the triangle
                for (int i = 0; i < num_roots; ++i) {
                    double t = roots[i];
                    if (t < 0. || t > 1.)
                        continue;

                    glm::dvec3 qt = m_positions[q] + t * dq;
                    glm::dvec3 p1t = m_positions[p1] + t * dp1;
                    glm::dvec3 p2t = m_positions[p2] + t * dp2;
                    glm::dvec3 p3t = m_positions[p3] + t * dp3;

                    glm::dvec3 normal = glm::cross(p2t - p1t, p3t - p1t);
                    double normal_len = glm::length(normal);
                    if (normal_len < 1e-12)
                        continue;

                    glm::dvec3 n = normal / normal_len;
                    glm::dvec3 surface, barycentrics;
                    double dist = closestPointInTriangle(qt, p1t, p2t, p3t, n, surface, barycentrics);

                    // THICKNESS FIX 2: If it crosses the plane within the thickness boundary of the edges
                    if (dist <= obj->m_surface_thickness) {
                        // Corrected: Uses the robust 'from_behind' computed at t=0
                        addSelfCollisionConstraint(q, p1, from_behind ? p2 : p3, from_behind ? p3 : p2);

                        accumulateCollisionsResponse(q, normal, _collisions_responses[self_i]);
                        // normal /= 3;
                        accumulateCollisionsResponse(p1, -normal, _collisions_responses[object_i]);
                        accumulateCollisionsResponse(p2, -normal, _collisions_responses[object_i]);
                        accumulateCollisionsResponse(p3, -normal, _collisions_responses[object_i]);
                        break;
                    }
                }
            }
        }
    }
}