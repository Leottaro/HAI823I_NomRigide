#define _USE_MATH_DEFINES

#include "Mesh.hpp"

#include <filesystem>
#include <fstream>

using namespace std;

MeshType meshTypeFromInt(int _i) {
    switch (_i) {
    case 0:
        return LoadedMesh{};
    case 1:
        return SingleTriangleMesh{};
    case 2:
        return SimpleGridMesh{};
    case 3:
        return SimpleTerrainMesh{};
    case 4:
        return CubeMesh{};
    case 5:
        return CubeSphereMesh{};
    default:
        return SingleTriangleMesh{};
    }
}

int meshTypeToInt(const MeshType& _type) {
    return std::visit(
        [](const auto& mesh_spec) {
            using T = std::decay_t<decltype(mesh_spec)>;
            if constexpr (std::is_same_v<T, LoadedMesh>) {
                return 0;
            } else if constexpr (std::is_same_v<T, SingleTriangleMesh>) {
                return 1;
            } else if constexpr (std::is_same_v<T, SimpleGridMesh>) {
                return 2;
            } else if constexpr (std::is_same_v<T, SimpleTerrainMesh>) {
                return 3;
            } else if constexpr (std::is_same_v<T, CubeMesh>) {
                return 4;
            } else if constexpr (std::is_same_v<T, CubeSphereMesh>) {
                return 5;
            }
        },
        _type);
}

std::string meshTypeToString(const MeshType& _type) {
    return std::visit(
        [](const auto& mesh_spec) {
            using T = std::decay_t<decltype(mesh_spec)>;
            if constexpr (std::is_same_v<T, LoadedMesh>) {
                return std::filesystem::path(mesh_spec.path).stem().string();
            } else if constexpr (std::is_same_v<T, SingleTriangleMesh>) {
                return std::string("SingleTriangle");
            } else if constexpr (std::is_same_v<T, SimpleGridMesh>) {
                return std::string("SimpleGrid");
            } else if constexpr (std::is_same_v<T, SimpleTerrainMesh>) {
                return std::string("SimpleTerrain");
            } else if constexpr (std::is_same_v<T, CubeMesh>) {
                return std::string("Cube");
            } else if constexpr (std::is_same_v<T, CubeSphereMesh>) {
                return std::string("CubeSphere");
            }
        },
        _type);
}

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

void Mesh::loadOFF(const std::string& filename) {
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
        m_normals[nIt] = glm::normalize(m_normals[nIt]);
    }
}

void Mesh::recomputePerVertexTextureCoordinates() {
    m_uvs.clear();
    m_uvs.resize(m_positions.size(), glm::vec2(0.0, 0.0));

    float xMin = FLT_MAX, xMax = FLT_MIN;
    float yMin = FLT_MAX, yMax = FLT_MIN;
    for (glm::vec3& p : m_positions) {
        xMin = std::min(xMin, p[0]);
        xMax = std::max(xMax, p[0]);
        yMin = std::min(yMin, p[1]);
        yMax = std::max(yMax, p[1]);
    }
    for (unsigned int pIt = 0; pIt < m_uvs.size(); ++pIt) {
        m_uvs[pIt] = glm::vec2((m_positions[pIt][0] - xMin) / (xMax - xMin), (m_positions[pIt][1] - yMin) / (yMax - yMin));
    }
}

void Mesh::recomputeStructs() {
    m_aabb = AABB<float>();
    double hash_grid_size = 0.;
    std::vector<KdTriangle> kd_triangles;
    kd_triangles.reserve(m_triangles.size());
    for (size_t i = 0; i < m_triangles.size(); i++) {
        kd_triangles.push_back(KdTriangle(m_positions[m_triangles[i][0]], m_positions[m_triangles[i][1]], m_positions[m_triangles[i][2]], i));
        m_aabb.addPosition(m_positions[m_triangles[i][0]]);
        m_aabb.addPosition(m_positions[m_triangles[i][1]]);
        m_aabb.addPosition(m_positions[m_triangles[i][2]]);
        hash_grid_size += glm::distance(m_positions[m_triangles[i][0]], m_positions[m_triangles[i][1]]) +
                          glm::distance(m_positions[m_triangles[i][0]], m_positions[m_triangles[i][2]]) +
                          glm::distance(m_positions[m_triangles[i][1]], m_positions[m_triangles[i][2]]);
    }
    glm::vec3 m_aabb_size = m_aabb.max - m_aabb.min;
    uint8_t max_axis = m_aabb_size.x > m_aabb_size.y && m_aabb_size.x > m_aabb_size.z ? 0
                       : m_aabb_size.y > m_aabb_size.z                                ? 1
                                                                                      : 2;
    m_tree = KdTree(kd_triangles, m_aabb, 32, max_axis);

    m_positions_hasher = PositionHasher<float>(m_positions.size(), hash_grid_size);
    for (const glm::uvec3& triangle : m_triangles) {
        AABB<float> aabb;
        aabb.addPosition(m_positions[triangle[0]]);
        aabb.addPosition(m_positions[triangle[1]]);
        aabb.addPosition(m_positions[triangle[2]]);
        m_positions_hasher.insertRange(aabb, triangle[0]);
        m_positions_hasher.insertRange(aabb, triangle[1]);
        m_positions_hasher.insertRange(aabb, triangle[2]);
    }
    constructed_structs = true;
}

bool Mesh::rayIntersection(const glm::vec3& _origin, const glm::vec3& _direction, float& t, size_t& triangle_index, glm::vec3& intersection, glm::vec3& barycentrics) const {
    if (constructed_structs)
        return m_tree.intersect(_origin, _direction, t, triangle_index, intersection, barycentrics);

    t = FLT_MAX;
    float tmp_t;
    glm::vec3 tmp_inter, tmp_bary;
    for (size_t i = 0; i < m_triangles.size(); i++) {
        const glm::vec3& v0 = m_positions[m_triangles[i][0]];
        const glm::vec3& v1 = m_positions[m_triangles[i][1]];
        const glm::vec3& v2 = m_positions[m_triangles[i][2]];
        const glm::vec3& triangle_normal = glm::cross(v1 - v0, v2 - v0);

        // ray intersections
        if (!rayTriangleIntersection(_origin, _direction, v0, v1, v2, triangle_normal, tmp_t, tmp_inter, tmp_bary) || tmp_t >= t)
            continue;

        t = tmp_t;
        intersection = tmp_inter;
        barycentrics = tmp_bary;
        triangle_index = i;
    }
    return t == FLT_MAX;
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

void Mesh::render() const {
    glBindVertexArray(m_VAO); // Activate the VAO storing geometry data
    glDrawElements(GL_TRIANGLES, m_triangles.size() * 3, GL_UNSIGNED_INT, 0);
}

void Mesh::clear() {
    constructed_structs = false;
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
