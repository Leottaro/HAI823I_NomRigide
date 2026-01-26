#define _USE_MATH_DEFINES

#include "Mesh.hpp"

#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace std;

Mesh::~Mesh() {
    clear();
}

void Mesh::loadOFF(const std::string &filename) {
    clear();

    ifstream in(filename.c_str());
    if (!in) {
        throw std::ios_base::failure("[Mesh Loader][loadOFF] Cannot open " + filename);
    }

    string offString;
    unsigned int sizeV, sizeT, tmp;
    in >> offString >> sizeV >> sizeT >> tmp;

    m_vertex_positions.resize(sizeV);
    m_triangle_indices.resize(sizeT);
    size_t tracker = (sizeV + sizeT) / 20;
    for (unsigned int i = 0; i < sizeV; i++) {
        if (i % tracker == 0) {
            in >> m_vertex_positions[i][0] >> m_vertex_positions[i][1] >> m_vertex_positions[i][2];
        }
    }

    int s;
    for (unsigned int i = 0; i < sizeT; i++) {
        if ((sizeV + i) % tracker == 0) {
            in >> s;
        }
        for (unsigned int j = 0; j < 3; j++) {
            in >> m_triangle_indices[i][j];
        }
    }
    in.close();

    recomputePerVertexNormals();
    recomputePerVertexTextureCoordinates();
}

void Mesh::computeBoundingSphere(glm::vec3 &center, float &radius) const {
    center = glm::vec3(0.0);
    for (const glm::vec3 &p : m_vertex_positions) {
        center += p;
    }
    center /= m_vertex_positions.size();

    radius = 0.f;
    for (const glm::vec3 &p : m_vertex_positions) {
        radius = std::max(radius, distance(center, p));
    }
}

void Mesh::recomputePerVertexNormals(bool angleBased) {
    m_vertex_normals.clear();
    m_vertex_normals.resize(m_vertex_positions.size(), glm::vec3(0.0, 0.0, 0.0));

    for (unsigned int tIt = 0; tIt < m_triangle_indices.size(); ++tIt) {
        glm::uvec3 t = m_triangle_indices[tIt];
        glm::vec3 n_t = glm::cross(m_vertex_positions[t[1]] - m_vertex_positions[t[0]], m_vertex_positions[t[2]] - m_vertex_positions[t[0]]);
        m_vertex_normals[t[0]] += n_t;
        m_vertex_normals[t[1]] += n_t;
        m_vertex_normals[t[2]] += n_t;
    }
    for (unsigned int nIt = 0; nIt < m_vertex_normals.size(); ++nIt) {
        glm::normalize(m_vertex_normals[nIt]);
    }
}

void Mesh::recomputePerVertexTextureCoordinates() {
    m_vertex_tex_coords.clear();
    m_vertex_tex_coords.resize(m_vertex_positions.size(), glm::vec2(0.0, 0.0));

    float xMin = FLT_MAX, xMax = FLT_MIN;
    float yMin = FLT_MAX, yMax = FLT_MIN;
    for (glm::vec3 &p : m_vertex_positions) {
        xMin = std::min(xMin, p[0]);
        xMax = std::max(xMax, p[0]);
        yMin = std::min(yMin, p[1]);
        yMax = std::max(yMax, p[1]);
    }
    for (unsigned int pIt = 0; pIt < m_vertex_tex_coords.size(); ++pIt) {
        m_vertex_tex_coords[pIt] = glm::vec2((m_vertex_positions[pIt][0] - xMin) / (xMax - xMin), (m_vertex_positions[pIt][1] - yMin) / (yMax - yMin));
    }
}

void Mesh::init() {
    glCreateBuffers(1, &m_posVBO);                                                  // Generate a GPU buffer to store the positions of the vertices
    size_t vertexBufferSize = sizeof(glm::vec3) * m_vertex_positions.size();        // Gather the size of the buffer from the CPU-side vector
    glNamedBufferStorage(m_posVBO, vertexBufferSize, NULL, GL_DYNAMIC_STORAGE_BIT); // Create a data store on the GPU
    glNamedBufferSubData(m_posVBO, 0, vertexBufferSize, m_vertex_positions.data()); // Fill the data store from a CPU array

    glCreateBuffers(1, &m_normalVBO); // Same for normal
    glNamedBufferStorage(m_normalVBO, vertexBufferSize, NULL, GL_DYNAMIC_STORAGE_BIT);
    glNamedBufferSubData(m_normalVBO, 0, vertexBufferSize, m_vertex_normals.data());

    glCreateBuffers(1, &m_texCoordVBO); // Same for texture coordinates
    size_t texCoordBufferSize = sizeof(glm::vec2) * m_vertex_tex_coords.size();
    glNamedBufferStorage(m_texCoordVBO, texCoordBufferSize, NULL, GL_DYNAMIC_STORAGE_BIT);
    glNamedBufferSubData(m_texCoordVBO, 0, texCoordBufferSize, m_vertex_tex_coords.data());

    glCreateBuffers(1, &m_IBO); // Same for the index buffer, that stores the list of indices of the triangles forming the mesh
    size_t indexBufferSize = sizeof(glm::uvec3) * m_triangle_indices.size();
    glNamedBufferStorage(m_IBO, indexBufferSize, NULL, GL_DYNAMIC_STORAGE_BIT);
    glNamedBufferSubData(m_IBO, 0, indexBufferSize, m_triangle_indices.data());

    glCreateVertexArrays(1, &m_VAO); // Create a single handle that joins together attributes (vertex positions, normals) and connectivity (triangles indices)
    glBindVertexArray(m_VAO);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, m_posVBO);

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, m_normalVBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);

    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, m_texCoordVBO);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), 0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_IBO);
    glBindVertexArray(0); // Desactive the VAO just created. Will be activated at rendering time.
}

void Mesh::render() {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_triangle_indices.size() * 3), GL_UNSIGNED_INT, 0);
}

void Mesh::clear() {
    m_vertex_positions.clear();
    m_vertex_normals.clear();
    m_vertex_tex_coords.clear();
    m_triangle_indices.clear();
    if (m_VAO) {
        glDeleteVertexArrays(1, &m_VAO);
        m_VAO = 0;
    }
    if (m_posVBO) {
        glDeleteBuffers(1, &m_posVBO);
        m_posVBO = 0;
    }
    if (m_normalVBO) {
        glDeleteBuffers(1, &m_normalVBO);
        m_normalVBO = 0;
    }
    if (m_texCoordVBO) {
        glDeleteBuffers(1, &m_texCoordVBO);
        m_texCoordVBO = 0;
    }
    if (m_IBO) {
        glDeleteBuffers(1, &m_IBO);
        m_IBO = 0;
    }
}
