// GLEW
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

// EIGEN
#include <Eigen/Dense>

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>

// USUAL INCLUDES
#include <iostream>
#include "ShaderProgram.hpp"
#include "Camera.hpp"
using namespace std;

// Settings // TODO: SINGLETON
const GLuint WIDTH = 800, HEIGHT = 600;
GLFWwindow *window;

// TODO : SCENE
const vec3 target_pos(0.);

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void initWindow() {
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SCALE_FRAMEBUFFER, GL_FALSE); // https://discourse.glfw.org/t/resizing-window-results-in-wrong-aspect-ratio/1268

    window = glfwCreateWindow(WIDTH, HEIGHT, "ImGui OpenGL3 example", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
}

void windowSetup() {
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // Ensure we can capture the escape key being pressed below
    // glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide the mouse and enable unlimited mouvement
    glfwPollEvents();                     // Set the mouse at the center of the screen
    glClearColor(0.1f, 0.1f, 0.3f, 0.0f); // Dark blue background
    glEnable(GL_DEPTH_TEST);              // Enable depth test
    glDepthFunc(GL_LESS);                 // Accept fragment if it closer to the camera than the former one
    // glEnable(GL_CULL_FACE);               // Cull triangles which normal is not towards the camera
}

void globalInit() {
    // INITIALIZE GLFW
    if (!glfwInit())
        exit(EXIT_FAILURE);
    initWindow();

    // INITIALIZE GLEW
    GLenum err = glewInit();
    if (err != GLEW_OK && err != 4) {
        /* Problem: glewInit failed, something is seriously wrong. */
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // INITIALIZE IMGUI
    ImGui_ImplGlfwGL3_Init(window, true);

    windowSetup();
}

int main(void) {
    globalInit();

    GLuint VAO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);
    ShaderProgram shader = ShaderProgram("ressources/shaders/vertex_shader.glsl", "ressources/shaders/fragment_shader.glsl");
    glBindFragDataLocation(shader.id(), 0, "outColor");
    shader.link();

    // INIT OBJECTS
    GLfloat const vertices[] = {
        0.0f, 0.5f,
        0.5f, -0.5f,
        -0.5f, -0.5f};
    GLuint const elements[] = {
        0, 1, 2};

    // INIT CAMERA
    Camera camera(10., target_pos);

    // INIT OBJECTS
    GLuint VBO;
    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    GLuint EBO;
    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);

    GLint PositionAttribute = glGetAttribLocation(shader.id(), "position");
    glEnableVertexAttribArray(PositionAttribute);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glVertexAttribPointer(PositionAttribute, 2, GL_FLOAT, GL_FALSE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // timings
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;

    glfwSwapInterval(1); // VSync - avoid having 3000 fps
    do {
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input

        ImGui_ImplGlfwGL3_NewFrame(); // Imgui update

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        shader.use();                                       // Use program

        // OBJECTS UPDATE
        camera.update(deltaTime, window, target_pos);

        // Update uniforms
        shader.set("view", camera.getViewMatrix());
        shader.set("projection", camera.getProjectionMatrix());

        // Draw objects
        glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    } while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
             glfwWindowShouldClose(window) == 0);

    shader.~ShaderProgram();
    glDeleteBuffers(1, &EBO);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);

    glfwTerminate();

    return 0;
}