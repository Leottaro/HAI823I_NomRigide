#include "ShaderProgram.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

#include <exception>
#include <ios>

using namespace std;

ShaderProgram::ShaderProgram() : m_id(glCreateProgram()) {}

ShaderProgram::ShaderProgram(const string &vertexShaderFilename, const string &fragmentShaderFilename) : m_id(glCreateProgram()) {
    loadShader(GL_VERTEX_SHADER, vertexShaderFilename);
    loadShader(GL_FRAGMENT_SHADER, fragmentShaderFilename);
    link();
    use();
}

ShaderProgram::~ShaderProgram() {
    glDeleteProgram(m_id);
}

string ShaderProgram::file2String(const string &filename) {
    ifstream input(filename.c_str());
    if (!input)
        throw ios_base::failure("[Shader Program][file2String] Error: cannot open " + filename);
    stringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void ShaderProgram::loadShader(GLenum type, const string &shaderFilename) {
    GLuint shader = glCreateShader(type);                    // Create the shader, e.g., a vertex shader to be applied to every single vertex of a mesh
    string shaderSourceString = file2String(shaderFilename); // Loads the shader source from a file to a C++ string
    if (shaderSourceString.empty()) {
        cerr << "No content in shader " << shaderFilename << endl;
        glDeleteShader(shader);
        return;
    }
    const GLchar *shaderSource = (const GLchar *)shaderSourceString.c_str(); // Interface the C++ string through a C pointer
    glShaderSource(shader, 1, &shaderSource, NULL);                          // Load the vertex shader source code
    glCompileShader(shader);                                                 // THe GPU driver compile the shader
    GLint compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLsizei len;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        GLchar *log = new GLchar[len + 1];
        glGetShaderInfoLog(shader, len, &len, log);
        cerr << "Compilation error in shader " << shaderFilename << " : " << endl
             << log << endl;
        delete[] log;
        glDeleteShader(shader);
        return;
    }
    glAttachShader(m_id, shader); // Set the vertex shader as the one ot be used with the program/pipeline
    glDeleteShader(shader);
}