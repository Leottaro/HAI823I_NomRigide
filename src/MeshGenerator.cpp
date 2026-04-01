#include "Mesh.hpp"

void Mesh::setSingleTriangle() {
    m_positions = {
        glm::vec3(0., 0., 0.),
        glm::vec3(1., 0., 0.),
        glm::vec3(0., 1., 0.),
    };
    m_triangles = {
        glm::uvec3(0, 1, 2),
    };
    recomputePerVertexNormals();
    recomputePerVertexTextureCoordinates();
}

void Mesh::setSimpleGrid(size_t _nx, size_t _nz) {
    m_positions.resize(_nx * _nz);
    m_normals.resize(_nx * _nz);
    m_uvs.resize(_nx * _nz);
    m_triangles.resize((_nx - 1) * (_nz - 1) * 2);

    glm::vec3 normal = glm::vec3(0., 1., 0.);
    for (size_t iz = 0; iz < _nz; iz++) {
        float z = float(iz) / (_nz - 1);
        for (size_t ix = 0; ix < _nx; ix++) {
            float x = float(ix) / (_nx - 1);
            glm::vec3 position = glm::vec3(x - 0.5f, 0.f, z - 0.5f);
            glm::vec2 uv = glm::vec2(x, z);

            size_t v0 = iz * _nx + ix;
            m_positions[v0] = position;
            m_normals[v0] = normal;
            m_uvs[v0] = uv;

            if (ix == (_nx - 1) || iz == (_nz - 1))
                continue;

            size_t v1 = iz * _nx + (ix + 1);
            size_t v2 = (iz + 1) * _nx + ix;
            size_t v3 = (iz + 1) * _nx + (ix + 1);
            glm::uvec3 triangle1 = glm::uvec3(v0, v2, v1);
            glm::uvec3 triangle2 = glm::uvec3(v1, v2, v3);

            size_t t1 = iz * (_nx - 1) + ix;
            m_triangles[2 * t1] = triangle1;
            m_triangles[2 * t1 + 1] = triangle2;
        }
    }
}

void Mesh::setSimpleTerrain(size_t _nx, size_t _nz, glm::vec2 y_range) {
    setSimpleGrid(_nx, _nz);
    for (size_t i = 0; i < _nx * _nz; i++) {
        float rng = float(rand()) / float(RAND_MAX);
        m_positions[i].y = y_range[0] + rng * (y_range[1] - y_range[0]);
    }
    recomputePerVertexNormals();
}

void Mesh::setCube(size_t _n) {
    size_t n_vertices = 6 * _n * _n;
    m_positions.resize(n_vertices);
    m_normals.resize(n_vertices);

    size_t n_triangles = 12 * (_n - 1) * (_n - 1);
    m_triangles.resize(n_triangles);

    for (size_t face_depth = 0; face_depth < 2; face_depth++) {
        for (size_t face_axis = 0; face_axis < 3; face_axis++) {
            for (size_t i = 0; i < _n; i++) {
                float i_pos = float(i) / (_n - 1);
                for (size_t j = 0; j < _n; j++) {
                    float j_pos = float(j) / (_n - 1);

                    size_t v0 = j + _n * (i + _n * (face_axis + 3 * face_depth));

                    m_positions[v0][face_axis] = face_depth - 0.5f;
                    m_positions[v0][(face_axis + 1) % 3] = (face_depth == 0 ? j_pos : i_pos) - 0.5f;
                    m_positions[v0][(face_axis + 2) % 3] = (face_depth == 0 ? i_pos : j_pos) - 0.5f;

                    m_normals[v0] = glm::vec3(0.);
                    m_normals[v0][face_axis] = face_depth == 0 ? -1. : 1.;

                    if (i == (_n - 1) || j == (_n - 1))
                        continue;

                    size_t v1 = v0 + 1;
                    size_t v2 = v0 + _n;
                    size_t v3 = v2 + 1;
                    glm::uvec3 triangle1 = glm::uvec3(v0, v2, v1);
                    glm::uvec3 triangle2 = glm::uvec3(v1, v2, v3);

                    size_t t0 = j + (_n - 1) * (i + (_n - 1) * (face_axis + 3 * face_depth));
                    m_triangles[2 * t0] = triangle1;
                    m_triangles[2 * t0 + 1] = triangle2;
                }
            }
        }
    }

    recomputePerVertexTextureCoordinates();
}

void Mesh::setCubeSphere(size_t _n) {
    setCube(_n);
    size_t n_vertices = 6 * _n * _n;
    for (size_t i = 0; i < n_vertices; i++) {
        m_positions[i] = glm::normalize(m_positions[i]);
        m_normals[i] = m_positions[i];
    }
}
