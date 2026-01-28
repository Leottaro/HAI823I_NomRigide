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
#include <imgui_impl_glfw_gl3.h>

// USUAL INCLUDES
#include <iostream>
#include "ShaderProgram.hpp"
#include "Camera.hpp"
#include "Mesh.hpp"
using namespace std;

// Settings // TODO: SINGLETON
GLuint window_width = 800, window_height = 600;
glm::vec2 cursor_pos = glm::vec2(0, 0);
glm::vec2 cursor_vel = glm::vec2(0, 0);
glm::vec2 scroll = glm::vec2(0, 0);
int polygon_mode = GL_FILL;
GLFWwindow *window;

// TODO : SCENE
const glm::vec3 target_pos(0.);

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

void initWindow();
void initOpenGL();
void globalInit();

int main(void) {
    globalInit();

    ShaderProgram shader = ShaderProgram("ressources/shaders/vertex_shader.glsl", "ressources/shaders/fragment_shader.glsl");
    shader.link();

    // INIT SCENE
    // init camera
    Camera camera(target_pos, 3., glm::vec2(-M_PI_4 * 0.5, 0.));
    // init meshes
    vector<Mesh> meshes(2);
    meshes[0].setSimpleTerrain(32, 32, glm::vec2(0., .1));
    meshes[0].setTranslation(glm::vec3(-0.5));
    meshes[0].setScaleXZ(3.);
    meshes[1].loadOFF("ressources/models/rhino2.off");

    for (Mesh &mesh : meshes) {
        mesh.init();
    }

    // TODO: init textures
    // TODO: setup lights

    // timings
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    glfwSwapInterval(1); // VSync - avoid having 3000 fps
    do {
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        ImGui_ImplGlfwGL3_NewFrame(); // Imgui update

        // OBJECTS UPDATE
        camera.update(deltaTime, window, glm::vec3(0.), cursor_vel, scroll);

        meshes[1].setTranslation(glm::vec3(0., 0., -1.));
        meshes[1].setRotation(glm::vec3(0., currentFrame * 2. * M_PI, 0.));

        // RENDER
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        shader.use();                                       // Use program

        // Update uniforms
        glm::mat4 projection = camera.getProjectionMatrix();
        glm::mat4 view = camera.getViewMatrix();
        shader.set("projection", projection);

        // Render Meshes
        for (Mesh &mesh : meshes) {
            glm::mat4 model = mesh.computeTransformationMatrix();
            glm::mat4 model_view = view * model;
            glm::mat4 normal_mat = glm::transpose(glm::inverse(model_view));
            shader.set("model_view", model_view);
            shader.set("normal_mat", normal_mat);
            mesh.render();
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    } while (glfwWindowShouldClose(window) == GLFW_FALSE);

    shader.~ShaderProgram();
    for (Mesh &mesh : meshes) {
        mesh.clear();
    }

    glfwTerminate();

    return 0;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    // cout << "window : " << width << ", " << width << endl;
    width = window_width;
    height = window_height;
    glViewport(0, 0, width, height);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    // cout << "key:" << key << " scancode:" << scancode << " action:" << action << " mods:" << mods << endl;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    } else if ((key == GLFW_KEY_W || key == GLFW_KEY_Z) && action == GLFW_PRESS) {
        if (polygon_mode == GL_FILL) {
            polygon_mode = GL_LINE;
        } else if (polygon_mode == GL_LINE) {
            polygon_mode = GL_POINT;
        } else if (polygon_mode == GL_POINT) {
            polygon_mode = GL_FILL;
        }
        glPolygonMode(GL_FRONT_AND_BACK, polygon_mode);
    }
}

void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos) {
    // cout << "cursor_pos: (" << xpos << ", " << ypos << ")" << endl;
    cursor_vel.x = xpos - cursor_pos.x;
    cursor_vel.y = ypos - cursor_pos.y;
    cursor_pos.x = xpos;
    cursor_pos.y = ypos;
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
    // cout << "scroll: (" << xoffset << ", " << yoffset << ")" << endl;
    scroll.x = xoffset;
    scroll.y = yoffset;
}

void initWindow() {
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SCALE_FRAMEBUFFER, GL_FALSE); // https://discourse.glfw.org/t/resizing-window-results-in-wrong-aspect-ratio/1268

    window = glfwCreateWindow(window_width, window_height, "ImGui OpenGL3 example", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetScrollCallback(window, scroll_callback);
}

void initOpenGL() {
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // Ensure we can capture the escape key being pressed below
    glClearColor(0.1f, 0.1f, 0.3f, 0.0f);                // Dark blue background
    glEnable(GL_DEPTH_TEST);                             // Enable depth test
    glDepthFunc(GL_LESS);                                // Accept fragment if it closer to the camera than the former one
    glEnable(GL_CULL_FACE);                              // Cull triangles which normal is not towards the camera
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
    ImGui_ImplGlfwGL3_Init(window, false);

    initOpenGL();
}