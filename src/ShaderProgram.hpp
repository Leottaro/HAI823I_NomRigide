#pragma once

#include <GL/glew.h>
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <memory>
#include <string>

class ShaderProgram {
private:
    GLuint m_id = 0;

    std::string file2String(const std::string &filename);            // Loads the content of an ASCII file in a standard C++ string
    void loadShader(GLenum type, const std::string &shaderFilename); // Loads and compile a shader, before attaching it to a program

public:
    ShaderProgram();
    ShaderProgram(const std::string &vertexShaderFilename, const std::string &fragmentShaderFilename);
    virtual ~ShaderProgram();

    /// Generate a minimal shader program, made of one vertex shader and one fragment shader
    static std::shared_ptr<ShaderProgram> genBasicShaderProgram(const std::string &vertexShaderFilename, const std::string &fragmentShaderFilename);

    inline GLuint id() { return m_id; }

    inline void link() { glLinkProgram(m_id); }
    inline void use() { glUseProgram(m_id); }
    inline static void stop() { glUseProgram(0); }

    // UNIFORMS
    inline GLuint getLocation(const std::string &name) {
        return glGetUniformLocation(m_id, name.c_str());
    }
    inline void set(const std::string &name, int value) {
        glUniform1i(getLocation(name.c_str()), value);
    }
    inline void set(const std::string &name, GLuint value) {
        glUniform1i(getLocation(name.c_str()), value);
    }
    inline void set(const std::string &name, float value) {
        glUniform1f(getLocation(name.c_str()), value);
    }
    inline void set(const std::string &name, const glm::vec2 &value) {
        glUniform2fv(getLocation(name.c_str()), 1, glm::value_ptr(value));
    }
    inline void set(const std::string &name, const glm::vec3 &value) {
        glUniform3fv(getLocation(name.c_str()), 1, glm::value_ptr(value));
    }
    inline void set(const std::string &name, const glm::vec4 &value) {
        glUniform4fv(getLocation(name.c_str()), 1, glm::value_ptr(value));
    }
    inline void set(const std::string &name, const glm::mat4 &value) {
        glUniformMatrix4fv(getLocation(name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
    }
};