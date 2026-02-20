#define _USE_MATH_DEFINES

#include "Mesh.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>

using namespace std;

Mesh::~Mesh() {
    clear();
}

void Mesh::centerAndScaleToUnit() {
    glm::vec3 center(0.);
    for (unsigned int i = 0; i < m_positions.size(); i++)
        center += m_positions[i];
    center /= m_positions.size();

    float maxD = distance(m_positions[0], center);
    for (unsigned int i = 1; i < m_positions.size(); i++) {
        float m = distance(m_positions[i], center);
        if (m > maxD)
            maxD = m;
    }
    for (unsigned int i = 0; i < m_positions.size(); i++)
        m_positions[i] = (m_positions[i] - center) / maxD;
}

void Mesh::loadOFF(const std::string &filename) {
    ifstream in(filename.c_str());
    if (!in)
        return;
    string offString;
    unsigned int sizeV, sizeT, tmp;
    in >> offString >> sizeV >> sizeT >> tmp;
    // cout << "loading mesh at \"" << filename << "\"" << endl
    //      << "siseV: " << sizeV << endl
    //      << "sizeT: " << sizeT << endl;

    m_positions.resize(sizeV);
    for (unsigned int i = 0; i < sizeV; i++) {
        in >> m_positions[i][0] >> m_positions[i][1] >> m_positions[i][2];
        // cout << "position: " << m_vertices[i][0] << "," << m_vertices[i][1] << "," << m_vertices[i][2];

        while (in.peek() == ' ')
            in.get();

        if (!(in.peek() == '\n' || in.peek() == '\r' || in.eof())) {
            in >> m_normals[i][0] >> m_normals[i][1] >> m_normals[i][2];
            // cout << ", normal: " << m_normals[i][0] << "," << m_normals[i][1] << "," << m_normals[i][2];
        }
        // cout << endl;

        while (in.peek() == ' ')
            in.get();
    }

    int s;
    m_triangles.resize(sizeT);
    for (unsigned int i = 0; i < sizeT; i++) {
        in >> s;
        in >> m_triangles[i][0] >> m_triangles[i][1] >> m_triangles[i][2];
        // cout << "Triangle: " << m_triangles[i][0] << "," << m_triangles[i][1] << "," << m_triangles[i][2];
        if (!(in.peek() == '\n' || in.peek() == '\r' || in.eof())) {
            string restOfLine;
            getline(in, restOfLine);
            // cout << "and some things";
        }
        // cout << endl;
    }
    in.close();

    centerAndScaleToUnit();
    recomputePerVertexNormals();
    recomputePerVertexTextureCoordinates();
}

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
    m_triangles.resize(_nx * _nz * 2);

    glm::vec3 normal = glm::vec3(0., 1., 0.);
    for (size_t iz = 0; iz < _nz; iz++) {
        float z = float(iz) / (_nz - 1);
        for (size_t ix = 0; ix < _nx; ix++) {
            float x = float(ix) / (_nx - 1);
            glm::vec3 position = glm::vec3(x, 0., z);
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

            m_triangles[2 * v0] = triangle1;
            m_triangles[2 * v0 + 1] = triangle2;
        }
    }
}

void Mesh::setSimpleTerrain(size_t _nx, size_t _nz, glm::vec2 y_range) {
    setSimpleGrid(_nx, _nz);
    for (size_t i = 0; i < _nx * _nz; i++) {
        float rng = float(rand()) / RAND_MAX;
        m_positions[i].y = y_range[0] + rng * (y_range[1] - y_range[0]);
    }
    recomputePerVertexNormals();
}

void Mesh::setCube(size_t _n) {
    size_t n_vertices = 6 * _n * _n;
    m_positions.resize(n_vertices);
    m_normals.resize(n_vertices);

    size_t n_triangles = n_vertices * 2;
    m_triangles.resize(n_triangles);

    for (size_t face_depth = 0; face_depth < 2; face_depth++) {
        for (size_t face_axis = 0; face_axis < 3; face_axis++) {
            for (size_t i = 0; i < _n; i++) {
                float i_pos = float(i) / (_n - 1);
                for (size_t j = 0; j < _n; j++) {
                    float j_pos = float(j) / (_n - 1);

                    size_t v0 = j + _n * (i + _n * (face_axis + 3 * face_depth));

                    m_positions[v0][face_axis] = face_depth;
                    m_positions[v0][(face_axis + 1) % 3] = face_depth == 0 ? j_pos : i_pos;
                    m_positions[v0][(face_axis + 2) % 3] = face_depth == 0 ? i_pos : j_pos;

                    m_normals[v0] = glm::vec3(0.);
                    m_normals[v0][face_axis] = face_depth == 0 ? -1. : 1.;

                    if (i == (_n - 1) || j == (_n - 1))
                        continue;

                    size_t v1 = (j + 1) + _n * (i + _n * (face_axis + 3 * face_depth));
                    size_t v2 = j + _n * ((i + 1) + _n * (face_axis + 3 * face_depth));
                    size_t v3 = (j + 1) + _n * ((i + 1) + _n * (face_axis + 3 * face_depth));
                    glm::uvec3 triangle1 = glm::uvec3(v0, v2, v1);
                    glm::uvec3 triangle2 = glm::uvec3(v1, v2, v3);
                    m_triangles[2 * v0] = triangle1;
                    m_triangles[2 * v0 + 1] = triangle2;
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
        m_normals[i] = glm::normalize(m_positions[i] - glm::vec3(0.5));
        m_positions[i] = m_normals[i];
    }
}

void Mesh::computeBoundingSphere(glm::vec3 &center, float &radius) const {
    center = glm::vec3(0.0);
    for (const glm::vec3 &p : m_positions) {
        center += p;
    }
    center /= m_positions.size();

    radius = 0.f;
    for (const glm::vec3 &p : m_positions) {
        radius = std::max(radius, distance(center, p));
    }
}

void Mesh::recomputePerVertexNormals(bool angleBased) {
    m_normals.clear();
    m_normals.resize(m_positions.size(), glm::vec3(0.0, 0.0, 0.0));

    for (unsigned int tIt = 0; tIt < m_triangles.size(); ++tIt) {
        glm::uvec3 t = m_triangles[tIt];
        glm::vec3 n_t = glm::cross(m_positions[t[1]] - m_positions[t[0]], m_positions[t[2]] - m_positions[t[0]]);
        m_normals[t[0]] += n_t;
        m_normals[t[1]] += n_t;
        m_normals[t[2]] += n_t;
    }
    for (unsigned int nIt = 0; nIt < m_normals.size(); ++nIt) {
        glm::normalize(m_normals[nIt]);
    }
}

void Mesh::recomputePerVertexTextureCoordinates() {
    m_uvs.clear();
    m_uvs.resize(m_positions.size(), glm::vec2(0.0, 0.0));

    float xMin = FLT_MAX, xMax = FLT_MIN;
    float yMin = FLT_MAX, yMax = FLT_MIN;
    for (glm::vec3 &p : m_positions) {
        xMin = std::min(xMin, p[0]);
        xMax = std::max(xMax, p[0]);
        yMin = std::min(yMin, p[1]);
        yMax = std::max(yMax, p[1]);
    }
    for (unsigned int pIt = 0; pIt < m_uvs.size(); ++pIt) {
        m_uvs[pIt] = glm::vec2((m_positions[pIt][0] - xMin) / (xMax - xMin), (m_positions[pIt][1] - yMin) / (yMax - yMin));
    }
}

void Mesh::init() {
    glGenVertexArrays(1, &m_VAO);
    glBindVertexArray(m_VAO);

    glGenBuffers(1, &m_positions_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glBufferData(GL_ARRAY_BUFFER, m_positions.size() * sizeof(glm::vec3), m_positions.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, m_positions_VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &m_normals_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_normals_VBO);
    glBufferData(GL_ARRAY_BUFFER, m_normals.size() * sizeof(glm::vec3), m_normals.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, m_normals_VBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &m_uvs_VBO);
    glBindBuffer(GL_ARRAY_BUFFER, m_uvs_VBO);
    glBufferData(GL_ARRAY_BUFFER, m_uvs.size() * sizeof(glm::vec2), m_uvs.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, m_uvs_VBO);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);

    glGenBuffers(1, &m_triangles_EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_triangles_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_triangles.size() * sizeof(glm::uvec3), m_triangles.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glBindVertexArray(0);
}

void Mesh::render() {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    glDrawElements(GL_TRIANGLES, m_triangles.size() * 3, GL_UNSIGNED_INT, 0);
}

void Mesh::clear() {
    m_positions.clear();
    m_normals.clear();
    m_uvs.clear();
    m_triangles.clear();
    if (m_VAO) {
        glDeleteVertexArrays(1, &m_VAO);
        m_VAO = 0;
    }
    if (m_positions_VBO) {
        glDeleteBuffers(1, &m_positions_VBO);
        m_positions_VBO = 0;
    }
    if (m_normals_VBO) {
        glDeleteBuffers(1, &m_normals_VBO);
        m_normals_VBO = 0;
    }
    if (m_uvs_VBO) {
        glDeleteBuffers(1, &m_uvs_VBO);
        m_uvs_VBO = 0;
    }
    if (m_triangles_EBO) {
        glDeleteBuffers(1, &m_triangles_EBO);
        m_triangles_EBO = 0;
    }
}
