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
private:
    std::vector<glm::vec3> m_vertices = std::vector<glm::vec3>();
    std::vector<glm::vec3> m_normals = std::vector<glm::vec3>();
    std::vector<glm::vec2> m_uvs = std::vector<glm::vec2>();
    std::vector<glm::uvec3> m_triangles = std::vector<glm::uvec3>();

    GLuint m_VAO = 0;
    GLuint m_vertices_VBO = 0;
    GLuint m_normals_VBO = 0;
    GLuint m_uvs_VBO = 0;
    GLuint m_triangles_EBO = 0;

    glm::vec3 m_translation = glm::vec3(0.);
    glm::vec3 m_rotation = glm::vec3(0.);
    glm::vec3 m_scale = glm::vec3(1.);

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
    inline const std::vector<glm::vec3> &vertexPositions() const { return m_vertices; }
    inline std::vector<glm::vec3> &vertexPositions() { return m_vertices; }
    inline const std::vector<glm::vec3> &vertexNormals() const { return m_normals; }
    inline std::vector<glm::vec3> &vertexNormals() { return m_normals; }
    inline const std::vector<glm::vec2> &vertexTexCoords() const { return m_uvs; }
    inline std::vector<glm::vec2> &vertexTexCoords() { return m_uvs; }
    inline const std::vector<glm::uvec3> &triangleIndices() const { return m_triangles; }
    inline std::vector<glm::uvec3> &triangleIndices() { return m_triangles; }

    // TRANSFORMATIONS
    inline const glm::vec3 getTranslation() const { return m_translation; }
    inline void setTranslation(const glm::vec3 &t) { m_translation = t; }
    inline const glm::vec3 getRotation() const { return m_rotation; }
    inline void setRotation(const glm::vec3 &r) { m_rotation = r; }
    inline glm::vec3 getScale() const { return m_scale; }
    inline void setScale(glm::vec3 s) { m_scale = s; }
    inline void setScale(float s) { m_scale = glm::vec3(s); }
    inline void setScaleX(float sx) { m_scale.x = sx; }
    inline void setScaleY(float sy) { m_scale.y = sy; }
    inline void setScaleZ(float sz) { m_scale.z = sz; }
    inline void setScaleXY(float s) { m_scale.x = m_scale.y = s; }
    inline void setScaleXZ(float s) { m_scale.x = m_scale.z = s; }
    inline void setScaleYZ(float s) { m_scale.y = m_scale.z = s; }
    inline glm::mat4 computeTransformationMatrix() const {
        glm::mat4 identity(1.0);
        glm::mat4 scale_matrix = glm::scale(identity, m_scale);
        glm::mat4 rotation_scale_matrix = glm::rotate(scale_matrix, m_rotation[0], glm::vec3(1.0, 0.0, 0.0));
        rotation_scale_matrix = glm::rotate(rotation_scale_matrix, m_rotation[1], glm::vec3(0.0, 1.0, 0.0));
        rotation_scale_matrix = glm::rotate(rotation_scale_matrix, m_rotation[2], glm::vec3(0.0, 0.0, 1.0));
        glm::mat4 transformation_matrix = glm::translate(rotation_scale_matrix, m_translation);
        return transformation_matrix;
    }

    /// Compute the parameters of a sphere which bounds the mesh
    void computeBoundingSphere(glm::vec3 &center, float &radius) const;

    void recomputePerVertexNormals(bool angleBased = false);
    void recomputePerVertexTextureCoordinates();

    void init();
    void render();
    void clear();
};