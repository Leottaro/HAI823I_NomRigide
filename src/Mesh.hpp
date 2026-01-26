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
    std::vector<glm::vec3> m_vertex_positions = std::vector<glm::vec3>();
    std::vector<glm::vec3> m_vertex_normals = std::vector<glm::vec3>();
    std::vector<glm::vec2> m_vertex_tex_coords = std::vector<glm::vec2>();
    std::vector<glm::uvec3> m_triangle_indices = std::vector<glm::uvec3>();

    glm::vec3 m_translation = glm::vec3(0.);
    glm::vec3 m_rotation = glm::vec3(0.);
    float m_scale = 0.;

    GLuint m_VAO = 0;
    GLuint m_posVBO = 0;
    GLuint m_normalVBO = 0;
    GLuint m_texCoordVBO = 0;
    GLuint m_IBO = 0;

public:
    virtual ~Mesh();

    // INITIALIZERS
    Mesh() {}
    Mesh(const std::string &filename) { loadOFF(filename); }
    void loadOFF(const std::string &filename);

    // GETTERS
    inline const std::vector<glm::vec3> &vertexPositions() const { return m_vertex_positions; }
    inline std::vector<glm::vec3> &vertexPositions() { return m_vertex_positions; }
    inline const std::vector<glm::vec3> &vertexNormals() const { return m_vertex_normals; }
    inline std::vector<glm::vec3> &vertexNormals() { return m_vertex_normals; }
    inline const std::vector<glm::vec2> &vertexTexCoords() const { return m_vertex_tex_coords; }
    inline std::vector<glm::vec2> &vertexTexCoords() { return m_vertex_tex_coords; }
    inline const std::vector<glm::uvec3> &triangleIndices() const { return m_triangle_indices; }
    inline std::vector<glm::uvec3> &triangleIndices() { return m_triangle_indices; }

    // TRANSFORMATIONS
    inline const glm::vec3 getTranslation() const { return m_translation; }
    inline void setTranslation(const glm::vec3 &t) { m_translation = t; }
    inline const glm::vec3 getRotation() const { return m_rotation; }
    inline void setRotation(const glm::vec3 &r) { m_rotation = r; }
    inline float getScale() const { return m_scale; }
    inline void setScale(float s) { m_scale = s; }
    inline glm::mat4 computeTransformationMatrix() const {
        glm::mat4 identity(1.0);
        glm::mat4 scale_matrix = glm::scale(identity, glm::vec3(m_scale));
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