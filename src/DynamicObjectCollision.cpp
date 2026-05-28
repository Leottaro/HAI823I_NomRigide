#include <poly34.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/matrix.hpp>

#include "DynamicObject.hpp"
#include "AABB.hpp"
#include "Profiling.hpp"
#include <iostream>

#ifdef USE_OPENMP
#include <omp.h>
#endif

namespace {
struct SelfCollisionCandidate {
    uint q;
    uint p0;
    uint p1;
    uint p2;
};
} // namespace

void DynamicObject::detectPointTriangleCollision(const std::vector<glm::dvec3>& full_frame_positions, const std::vector<StaticBody>& static_bodies, std::map<uint, glm::dvec3>& _collisions_responses, uint start, uint end) {
    ScopedTimer timer(g_profile_frame.point_triangle_collision_ms);

    for (const StaticBody& static_body : static_bodies) {
        const std::vector<glm::vec3>& mesh_positions = static_body.m_mesh->vertexPositions();
        const std::vector<glm::vec3>& mesh_normals = static_body.m_mesh->vertexNormals();
        const std::vector<glm::uvec3>& mesh_triangles = static_body.m_mesh->triangleIndices();
        glm::dmat4 transformation = static_body.m_transformation->computeTransformationMatrix();
        glm::dmat4 inverse_transformation = glm::inverse(transformation);

        for (uint pj = start; pj < end; pj++) {
            glm::dvec3 origin = applyTransformation(m_positions[pj], 1., inverse_transformation);
            glm::dvec3 arrival = applyTransformation(full_frame_positions[pj] - m_positions[pj], 0., inverse_transformation);
            glm::dvec3 direction = arrival - origin;

            // Broad phase
            AABB<float> bouding_movement;
            bouding_movement.addPosition(origin);
            bouding_movement.addPosition(arrival);
            bouding_movement.expand(m_surface_thickness);
            if (!bouding_movement.intersectAABB(static_body.m_mesh->aabb()))
                continue;

            double min_t = DBL_MAX, max_t = -DBL_MAX, t{0.};
            glm::dvec3 closest_intersection{0.}, closest_normal{0.}, furthest_intersection{0.}, furthest_normal{0.}, intersection{0.}, barycentrics{0.};

            double min_dist = DBL_MAX, dist = DBL_MAX;
            glm::dvec3 closest_surface{0.}, closest_surface_normal{0.}, surface{0.};

            for (const glm::uvec3& triangle : mesh_triangles) {
                glm::dvec3 v0 = glm::dvec3(mesh_positions[triangle[0]]);
                glm::dvec3 v1 = glm::dvec3(mesh_positions[triangle[1]]);
                glm::dvec3 v2 = glm::dvec3(mesh_positions[triangle[2]]);
                glm::dvec3 n0 = glm::dvec3(mesh_normals[triangle[0]]);
                glm::dvec3 n1 = glm::dvec3(mesh_normals[triangle[1]]);
                glm::dvec3 n2 = glm::dvec3(mesh_normals[triangle[2]]);
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
                dist = closestPointInTriangle(origin, v0, v1, v2, glm::normalize(triangle_normal), surface, barycentrics);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_surface = surface;
                    closest_surface_normal = n0 * barycentrics[0] + n1 * barycentrics[1] + n2 * barycentrics[2];
                }
            }

            // Add a margin for predicted collision
            double movement_length = glm::length(direction);
            double t_margin = (movement_length > 1e-8) ? (m_surface_thickness / movement_length) : 0.0;
            bool is_ray_hit = (min_t >= -t_margin && min_t <= 1.0 + t_margin);
            bool is_close_proximity = (dist < m_surface_thickness);

            if (is_ray_hit || is_close_proximity) {
                // WILL ENTER THE OBJECT
                glm::dvec3 final_normal;
                glm::dvec3 final_intersection;

                if (is_ray_hit) {
                    final_normal = closest_normal;
                    final_intersection = closest_intersection;
                } else {
                    final_normal = closest_surface_normal;
                    final_intersection = closest_surface;
                }

                if (glm::length2(final_normal) > 1e-12) {
                    // std::cout << "POINT WILL ENTER: 0 <= " << min_t << " <= 1" << std::endl;
                    final_intersection = applyTransformation(final_intersection, 1., transformation);
                    final_normal = applyTransformation(glm::normalize(final_normal), 0., transformation);
                    addCollisionConstraint(pj, final_intersection, final_normal);
                }
            } else if (min_t < 0. && max_t > 1.) {
                // COMPLETLY INSIDE THE OBJECT
                // std::cout << "POINT INSIDE: " << min_t << " < 0 && " << max_t << " > 1" << std::endl;
                closest_surface = applyTransformation(closest_surface, 1., transformation);
                closest_surface_normal = applyTransformation(glm::normalize(closest_surface_normal), 0., transformation);
                addCollisionConstraint(pj, closest_surface, closest_surface_normal);
                // accumulate_collision(pj, closest_surface_normal);
            }
        }
    }
}
void DynamicObject::detectEdgeTriangleCollision(const std::vector<glm::dvec3>& full_frame_positions, const std::vector<StaticBody>& static_bodies, std::map<uint, glm::dvec3>& _collisions_responses, uint start, uint end) {
    ScopedTimer timer(g_profile_frame.edge_triangle_collision_ms);

    for (const StaticBody& static_body : static_bodies) {
        const std::vector<glm::vec3>& mesh_positions = static_body.m_mesh->vertexPositions();
        const std::vector<glm::vec3>& mesh_normals = static_body.m_mesh->vertexNormals();
        const std::vector<glm::uvec3>& mesh_triangles = static_body.m_mesh->triangleIndices();
        glm::dmat4 transformation = static_body.m_transformation->computeTransformationMatrix();
        glm::dmat4 inverse_transformation = glm::inverse(transformation);

        for (uint edge_i = start; edge_i < end; edge_i++) {
            uint e0 = m_lines[edge_i][0];
            uint e1 = m_lines[edge_i][1];

            glm::dvec3 origin = applyTransformation(glm::dvec3(full_frame_positions[e0]), 1., inverse_transformation);
            glm::dvec3 arrival = applyTransformation(glm::dvec3(full_frame_positions[e1]), 1., inverse_transformation);
            glm::dvec3 direction = arrival - origin;

            // Broad phase
            AABB<float> bouding_movement;
            bouding_movement.addPosition(origin);
            bouding_movement.addPosition(arrival);
            bouding_movement.addPosition(applyTransformation(glm::dvec3(m_positions[e0]), 1., inverse_transformation));
            bouding_movement.addPosition(applyTransformation(glm::dvec3(m_positions[e1]), 1., inverse_transformation));
            bouding_movement.expand(m_surface_thickness);
            if (!bouding_movement.intersectAABB(static_body.m_mesh->aabb()))
                continue;

            double min_t = DBL_MAX, max_t = -DBL_MAX, t;
            glm::dvec3 closest_intersection, closest_normal, furthest_intersection, furthest_normal, intersection, barycentrics;

            for (const glm::uvec3& triangle : mesh_triangles) {
                glm::dvec3 v0 = mesh_positions[triangle[0]];
                glm::dvec3 v1 = mesh_positions[triangle[1]];
                glm::dvec3 v2 = mesh_positions[triangle[2]];
                glm::dvec3 n0 = mesh_normals[triangle[0]];
                glm::dvec3 n1 = mesh_normals[triangle[1]];
                glm::dvec3 n2 = mesh_normals[triangle[2]];
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
            closest_intersection = applyTransformation(closest_intersection, 1., transformation);
            furthest_intersection = applyTransformation(furthest_intersection, 1., transformation);
            closest_normal = applyTransformation(closest_normal, 0., transformation);
            furthest_normal = applyTransformation(furthest_normal, 0., transformation);

            addEdgeCollisionConstraint(e0, e1, min_t, closest_intersection, closest_normal, max_t, furthest_intersection, furthest_normal);
            accumulateCollisionsResponse(e0, 0.5 * glm::normalize((1.0 - min_t) * closest_normal + (1.0 - max_t) * furthest_normal), _collisions_responses);
            accumulateCollisionsResponse(e1, 0.5 * glm::normalize(min_t * closest_normal + max_t * furthest_normal), _collisions_responses);
        }
    }
}
void DynamicObject::detectTrianglePointCollision(const std::vector<glm::dvec3>& full_frame_positions, const std::vector<StaticBody>& static_bodies, std::map<uint, glm::dvec3>& _collisions_responses, uint start, uint end) {
    ScopedTimer timer(g_profile_frame.triangle_point_collision_ms);

    for (const StaticBody& static_body : static_bodies) {
        const std::vector<glm::vec3>& static_positions = static_body.m_mesh->vertexPositions();
        glm::dmat4 transformation = static_body.m_transformation->computeTransformationMatrix();

        for (size_t i = 0; i < static_positions.size(); i++) {
            glm::dvec3 static_point = applyTransformation(glm::dvec3(static_positions[i]), 1., transformation);

            for (uint triangle_i = start; triangle_i < end; triangle_i++) {
                uint p0 = m_triangles[triangle_i][0];
                uint p1 = m_triangles[triangle_i][1];
                uint p2 = m_triangles[triangle_i][2];

                // Broad phase
                AABB<float> bouding_movement;
                bouding_movement.addPosition(full_frame_positions[p0]);
                bouding_movement.addPosition(full_frame_positions[p1]);
                bouding_movement.addPosition(full_frame_positions[p2]);
                bouding_movement.addPosition(m_positions[p0]);
                bouding_movement.addPosition(m_positions[p1]);
                bouding_movement.addPosition(m_positions[p2]);
                bouding_movement.expand(m_surface_thickness);
                if (!bouding_movement.isInside(static_point))
                    continue;

                glm::dvec3 v0 = full_frame_positions[p0];
                glm::dvec3 v1 = full_frame_positions[p1];
                glm::dvec3 v2 = full_frame_positions[p2];

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

void DynamicObject::detectSelfPointTriangleCollision(const PositionHasher<double>& hasher, const std::vector<glm::dvec3>& full_frame_positions, std::map<uint, glm::dvec3>& _collisions_responses, uint start, uint end) {
    ScopedTimer timer(g_profile_frame.self_point_triangle_collision_ms);

    std::vector<SelfCollisionCandidate> candidates;

#ifdef USE_OPENMP
#pragma omp parallel
#endif
    {
        std::vector<SelfCollisionCandidate> local_candidates;

#ifdef USE_OPENMP
#pragma omp for schedule(dynamic, 4)
#endif
        for (int ti_int = static_cast<int>(start); ti_int < static_cast<int>(end); ti_int++) {
            uint ti = static_cast<uint>(ti_int);
            uint p1 = m_triangles[ti][0];
            uint p2 = m_triangles[ti][1];
            uint p3 = m_triangles[ti][2];

            AABB<double> aabb;
            for (uint i = 0; i < 3; i++) {
                aabb.addPosition(m_positions[m_triangles[ti][i]]);
                aabb.addPosition(full_frame_positions[m_triangles[ti][i]]);
            }
            aabb.expand(m_surface_thickness);

            // Broad phase: Spatial Hasher
            hasher.forAllGridCells(aabb, [&](const glm::u64vec3& _key) {
                for (uint q : hasher.lookupKey(_key)) {
                    // Skip immediate neighbors
                    if (q == p1 || q == p2 || q == p3)
                        continue;

                    // Pre-calculate t=0 normal vector for side determination
                    glm::dvec3 N0 = glm::cross(m_positions[p2] - m_positions[p1], m_positions[p3] - m_positions[p1]);
                    // Determine side from the initial state (t=0) to prevent the 0.0 sign bug
                    double signed_dist_init = glm::dot(m_positions[q] - m_positions[p1], N0);
                    bool from_behind = signed_dist_init > 0.0;

                    // Catches slow-moving objects entering the shell without crossing the mid-plane
                    glm::dvec3 normal_end = glm::cross(full_frame_positions[p2] - full_frame_positions[p1], full_frame_positions[p3] - full_frame_positions[p1]);
                    double normal_end_len = glm::length(normal_end);
                    if (normal_end_len > 1e-12) {
                        glm::dvec3 n_end = normal_end / normal_end_len;
                        glm::dvec3 surface_end, bary_end;
                        double dist_end = closestPointInTriangle(full_frame_positions[q], full_frame_positions[p1], full_frame_positions[p2], full_frame_positions[p3], n_end, surface_end, bary_end);

                        if (dist_end <= m_surface_thickness) {
                            local_candidates.push_back({q, p1, from_behind ? p2 : p3, from_behind ? p3 : p2});
                            // std::cout << "SELF COLLISION (PROXIMITY)" << std::endl;
                            continue; // Handled, skip the cubic solver for this pair
                        }
                    }

                    // Continuous Component: Cubic Solver (Tunneling)
                    glm::dvec3 dq = full_frame_positions[q] - m_positions[q];
                    glm::dvec3 dp1 = full_frame_positions[p1] - m_positions[p1];
                    glm::dvec3 dp2 = full_frame_positions[p2] - m_positions[p2];
                    glm::dvec3 dp3 = full_frame_positions[p3] - m_positions[p3];

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

                        // If it crosses the plane within the thickness boundary of the edges
                        if (dist <= m_surface_thickness) {
                            // Corrected: Uses the robust 'from_behind' computed at t=0
                            local_candidates.push_back({q, p1, from_behind ? p2 : p3, from_behind ? p3 : p2});
                            // std::cout << "SELF COLLISION (TUNNELLING)" << std::endl;
                            break;
                        }
                    }
                }
            });
        }

#ifdef USE_OPENMP
#pragma omp critical
#endif
        {
            candidates.insert(candidates.end(), local_candidates.begin(), local_candidates.end());
        }
    }

    for (const SelfCollisionCandidate& candidate : candidates) {
        addSelfCollisionConstraint(candidate.q, candidate.p0, candidate.p1, candidate.p2);
    }
}
