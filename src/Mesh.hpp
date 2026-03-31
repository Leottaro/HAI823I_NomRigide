#pragma once

// GLEW
#include <GL/glew.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// GLFW
#include <GLFW/glfw3.h>

// EIGEN
#include <Eigen/Dense>

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// USUAL INCLUDES
#include "Transformation.hpp"
#include <vector>
#include <string>
#include <variant>

struct LoadedMesh {
    std::string path{""};
};
struct SimpleGridMesh {
    size_t nx{2};
    size_t nz{2};
};
struct SimpleTerrainMesh {
    size_t nx{2};
    size_t nz{2};
    glm::vec2 y_range{0., 0.1};
};
struct CubeMesh {
    size_t n{2};
};
struct CubeSphereMesh {
    size_t n{3};
};
struct SingleTriangleMesh {};

#define NB_MESH_TYPES 6
#define ALL_MESH_TYPES "LoadedMesh\0SingleTriangleMesh\0SimpleGridMesh\0SimpleTerrainMesh\0CubeMesh\0CubeSphereMesh\0"
using MeshType = std::variant<
    LoadedMesh,
    SingleTriangleMesh,
    SimpleGridMesh,
    SimpleTerrainMesh,
    CubeMesh,
    CubeSphereMesh>;
MeshType meshTypeFromInt(int _i);
int meshTypeToInt(const MeshType &_type);
std::string meshTypeToString(const MeshType &_type);

class Mesh {
    std::vector<glm::vec3> m_positions;
    std::vector<glm::vec3> m_normals;
    std::vector<glm::vec2> m_uvs;
    std::vector<glm::uvec3> m_triangles;

    GLuint m_VAO;
    GLuint m_positions_VBO;
    GLuint m_normals_VBO;
    GLuint m_uvs_VBO;
    GLuint m_triangles_EBO;

public:
    virtual ~Mesh();

    // INITIALIZERS
    Mesh() {}
    Mesh(const MeshType &_type) {
        std::visit(
            [this](const auto &mesh_spec) {
                using T = std::decay_t<decltype(mesh_spec)>;
                if constexpr (std::is_same_v<T, LoadedMesh>) {
                    loadOFF(mesh_spec.path);
                } else if constexpr (std::is_same_v<T, SingleTriangleMesh>) {
                    setSingleTriangle();
                } else if constexpr (std::is_same_v<T, SimpleGridMesh>) {
                    setSimpleGrid(mesh_spec.nx, mesh_spec.nz);
                } else if constexpr (std::is_same_v<T, SimpleTerrainMesh>) {
                    setSimpleTerrain(mesh_spec.nx, mesh_spec.nz, mesh_spec.y_range);
                } else if constexpr (std::is_same_v<T, CubeMesh>) {
                    setCube(mesh_spec.n);
                } else if constexpr (std::is_same_v<T, CubeSphereMesh>) {
                    setCubeSphere(mesh_spec.n);
                }
            },
            _type);
    }
    void loadOFF(const std::string &filename);
    void setSingleTriangle();
    void setSimpleGrid(size_t _nx, size_t _nz);                                           // Create a grid where x and z varies in [0;1]
    void setSimpleTerrain(size_t _nx, size_t _nz, glm::vec2 y_range = glm::vec2(0., 1.)); // Create a terrain where x and z varies in [0;1] and y varies in y_range
    void setCube(size_t _n);                                                              // Create a cube where x, y and z varies in [0;1]
    void setCubeSphere(size_t _n);                                                        // Create a CubeSphere of center (0,0,0) and radius 1

    // GETTERS
    inline const std::vector<glm::vec3> &vertexPositions() const { return m_positions; }
    inline std::vector<glm::vec3> &vertexPositions() { return m_positions; }
    inline const std::vector<glm::vec3> &vertexNormals() const { return m_normals; }
    inline std::vector<glm::vec3> &vertexNormals() { return m_normals; }
    inline const std::vector<glm::vec2> &vertexTexCoords() const { return m_uvs; }
    inline std::vector<glm::vec2> &vertexTexCoords() { return m_uvs; }
    inline const std::vector<glm::uvec3> &triangleIndices() const { return m_triangles; }
    inline std::vector<glm::uvec3> &triangleIndices() { return m_triangles; }

    void centerAndScaleToUnit();
    void computeBoundingSphere(glm::vec3 &center, float &radius) const;

    void recomputePerVertexNormals(bool angleBased = false);
    void recomputePerVertexTextureCoordinates();

    // OpenGL interface
    void init();
    void render() const;
    void clear();
};
