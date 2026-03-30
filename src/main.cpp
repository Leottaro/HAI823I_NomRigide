// GLEW
#include <GL/glew.h>

// GLM
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

// GLFW
#include <GLFW/glfw3.h>

// EIGEN
#include <Eigen/Dense>

// IMGUI
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// USUAL INCLUDES
#include <execinfo.h>
#include <stdio.h>

#include <iomanip>
#include <iostream>
#include <sstream>

#include "Camera.hpp"
#include "DynamicObject.hpp"
#include "Mesh.hpp"
#include "Scene.hpp"
#include "ShaderProgram.hpp"

using namespace std;

// TODO: SINGLETON
GLuint window_width = 1280, window_height = 720;
glm::vec2 cursor_pos = glm::vec2(0, 0);
glm::vec3 cursor_worldpos = glm::vec3(0, 0, 0);
glm::vec2 cursor_vel = glm::vec2(0, 0);
glm::vec2 scroll = glm::vec2(0, 0);
int polygon_mode = GL_FILL;
GLFWwindow *window;

Camera camera;
bool run_simulation = false;

void globalInit();

int main(void) {
    globalInit();

    ShaderProgram mesh_shader = ShaderProgram("ressources/shaders/mesh_vertex.glsl", "ressources/shaders/mesh_fragment.glsl");
    mesh_shader.link();
    ShaderProgram dynamic_shader = ShaderProgram("ressources/shaders/dynamic_vertex.glsl", "ressources/shaders/dynamic_fragment.glsl");
    dynamic_shader.link();

    // TODO: add/remove objects in imgui ?
    Scene scene = Scene();

    size_t size = 10;
    scene.addMesh(CubeMesh{2});
    // scene.addMesh(CubeMesh{size});
    scene.addMesh(LoadedMesh{"ressources/models/monkey.off"});

    Transformation *floor_transfo = scene.addStaticBody(StaticBodyDesc("sol", 0));
    floor_transfo->setTranslation(glm::vec3(0.f, -3.f, 0.f));
    floor_transfo->setScale(glm::vec3(10.f, 4.f, 50.f));
    floor_transfo->setEulerAngles(glm::vec3(M_PIf / 8.f, 0.f, 0.f));

    Transformation *dynamic_body_transformation = scene.addDynamicObject(DynamicObjectDesc("boule", 1, .75f, .75f, .75f, .75f));
    dynamic_body_transformation->setTranslation(glm::vec3(0.f, 3.f, 0.f));
    dynamic_body_transformation->setScale(glm::vec3(1.f));
    dynamic_body_transformation->setEulerAngles(glm::vec3(0.f));

    // dynamic_body.setVertexFixed(0, true);
    // dynamic_body.setVertexFixed(size - 1, true);
    // dynamic_body.setVertexFixed((size - 1) * size, true);
    // dynamic_body.setVertexFixed(size * size - 1, true);

    scene.resetObjects();

    // TODO: init textures
    // TODO: setup lights

    // timings
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    size_t frame_count = 0;
    glfwSwapInterval(1); // VSync - avoid having 3000 fps
    do {
        glfwSwapBuffers(window);
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        std::stringstream title;
        title << "ImGui OpenGL3 example";

        if (deltaTime == 0)
            title << " inf";
        else
            title << " FPS: " << std::fixed << std::setprecision(0) << (1.0 / deltaTime);

        if (!run_simulation)
            title << " (paused)";

        glfwSetWindowTitle(window, title.str().c_str());

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        frame_count++;

        // Imgui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // INTERFACES
        bool disable_mouse_actions = false;
        if (camera.updateInterface())
            disable_mouse_actions = true;
        if (scene.updateInterface())
            disable_mouse_actions = true;

        // OBJECTS UPDATE
        cursor_worldpos = applyTransformation(glm::vec3(2.f * (cursor_pos.x / window_width) - 1.f, 1.f - 2.f * (cursor_pos.y / window_height), camera.m_near_far.x), 1.f, glm::inverse(camera.getViewMatrix()) * glm::inverse(camera.getProjectionMatrix()));
        scene.updateInteractions(window, camera.m_position, cursor_worldpos);
        if (run_simulation) {
            if (!scene.updateSimulation(deltaTime)) {
                run_simulation = false;
            }
        }
        camera.update(window, deltaTime, cursor_vel, scroll, disable_mouse_actions);

        // RENDERING
        scene.render(dynamic_shader, mesh_shader, camera.getProjectionMatrix(), camera.getViewMatrix());

        // ImGui Render
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Reset some controls
        scroll = glm::vec2(0.);
        cursor_vel = glm::vec2(0.);
    } while (glfwWindowShouldClose(window) == GLFW_FALSE);

    scene.clear();
    dynamic_shader.~ShaderProgram();
    mesh_shader.~ShaderProgram();

    glfwTerminate();

    return 0;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    // cout << "framebuffer size: " << width << ", " << height << endl;
    window_width = width;
    window_height = height;
    glViewport(0, 0, width, height);
}

bool space_key_pressed = false;
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods) {
    // cout << "key:" << key << " scancode:" << scancode << " action:" << action << " mods:" << mods << endl;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    } else if (key == GLFW_KEY_Z && action == GLFW_PRESS) {
        if (polygon_mode == GL_FILL) {
            polygon_mode = GL_LINE;
        } else if (polygon_mode == GL_LINE) {
            polygon_mode = GL_POINT;
        } else if (polygon_mode == GL_POINT) {
            polygon_mode = GL_FILL;
        }
        glPolygonMode(GL_FRONT_AND_BACK, polygon_mode);
    } else if (key == GLFW_KEY_SPACE) {
        if (action == GLFW_PRESS) {
            if (!space_key_pressed) {
                space_key_pressed = true;
                run_simulation = !run_simulation;
            }
        } else if (space_key_pressed) {
            space_key_pressed = false;
        }
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    // cout << "mouse button:" << button << " action:" << action << " mods:" << mods << endl;
    if (button == GLFW_MOUSE_BUTTON_LEFT && mods == 0) {
        glfwSetInputMode(window, GLFW_CURSOR, action == GLFW_PRESS ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    }
}

void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos) {
    cursor_vel.x = xpos - cursor_pos.x;
    cursor_vel.y = ypos - cursor_pos.y;
    cursor_pos.x = xpos;
    cursor_pos.y = ypos;

    // cout << "cursor_pos: (" << cursor_pos.x << ", " << cursor_pos.y << ")\tcursor_vel: (" << cursor_vel.x << ", " << cursor_vel.y << ")" << endl;
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
    glfwWindowHint(GLFW_SCALE_FRAMEBUFFER, GL_FALSE); // https://discourse.glfw.org/t/resizing-window-results-in-wrong-aspect-ratio/1268s

    window = glfwCreateWindow(window_width, window_height, "ImGui OpenGL3 example", NULL, NULL);
    if (!window) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetScrollCallback(window, scroll_callback);

    if (glfwRawMouseMotionSupported())
        glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
}

void initOpenGL() {
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // Ensure we can capture the escape key being pressed below
    glClearColor(0.1f, 0.1f, 0.3f, 0.0f);                // Dark blue background
    glEnable(GL_DEPTH_TEST);                             // Enable depth test
    glDepthFunc(GL_LESS);                                // Accept fragment if it closer to the camera than the former one
    glEnable(GL_CULL_FACE);                              // Cull triangles which normal is not towards the camera
}

void globalInit() {
#if defined(__linux__)
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
    // INITIALIZE GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        exit(EXIT_FAILURE);
    }
    initWindow();

    // INITIALIZE GLEW
    GLenum err = glewInit();
    if (err != GLEW_OK && err != 4) {
        /* Problem: glewInit failed, something is seriously wrong. */
        fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
        exit(EXIT_FAILURE);
    }

    // INITIALIZE IMGUI
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // ImGuiIO &io = ImGui::GetIO();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // IF using Docking Branch
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true); // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
    ImGui_ImplOpenGL3_Init();

    initOpenGL();
}
