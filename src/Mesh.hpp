#pragma once

// GLEW
#include <GL/glew.h>

// GLM
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// USUAL INCLUDES
#include <memory>
#include <vector>

class Mesh {
protected:
    std::vector<glm::vec3> m_positions;

private:
    std::vector<glm::vec3> m_normals;
    std::vector<glm::vec2> m_uvs;
    std::vector<glm::uvec3> m_triangles;

    GLuint m_VAO;
    GLuint m_positions_VBO;
    GLuint m_normals_VBO;
    GLuint m_uvs_VBO;
    GLuint m_triangles_EBO;

    void centerAndScaleToUnit();

public:
    virtual ~Mesh();

    // INITIALIZERS
    Mesh() {}
    Mesh(const std::string &filename) { loadOFF(filename); }
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

    /// Compute the parameters of a sphere which bounds the mesh
    void computeBoundingSphere(glm::vec3 &center, float &radius) const;

    void recomputePerVertexNormals(bool angleBased = false);
    void recomputePerVertexTextureCoordinates();

    void init();
    void render();
    void clear();
};