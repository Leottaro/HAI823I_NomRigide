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
#include <iostream>
#include "ShaderProgram.hpp"
#include "Camera.hpp"
#include "Mesh.hpp"
#include "DynamicObject.hpp"
using namespace std;

// TODO: SINGLETON
GLuint window_width = 800, window_height = 600;
glm::vec2 cursor_pos = glm::vec2(0, 0);
glm::vec2 cursor_vel = glm::vec2(0, 0);
glm::vec2 scroll = glm::vec2(0, 0);
int polygon_mode = GL_FILL;
GLFWwindow *window;

bool next_frame = true;

void globalInit();

int main(void) {
    globalInit();

    // ShaderProgram shader = ShaderProgram("ressources/shaders/vertex_shader.glsl", "ressources/shaders/fragment_shader.glsl");
    ShaderProgram shader = ShaderProgram("ressources/shaders/vertex_simple.glsl", "ressources/shaders/fragment_simple.glsl");
    shader.link();

    // TODO: SCENE
    // init meshes
    // vector<Mesh> meshes(0);
    // meshes[0].setCubeSphere(20);
    // meshes[0].setTranslation(glm::vec3(-0.5));
    // meshes[0].setScaleXZ(3.);
    // meshes[1].loadOFF("ressources/models/rhino2.off");
    // Transformation rhino_transfo;
    // init camera
    // glm::vec3 center;
    // float radius;
    // meshes[1].computeBoundingSphere(center, radius);
    Camera camera(glm::vec3(), 5., glm::vec2(-M_PI_4 * 0.5, 0.));

    DynamicObject triangle;
    triangle.addVertex(glm::vec3(0.), glm::vec3(0.), 1.f, true);
    triangle.addVertex(glm::vec3(-1., 1., -1.), glm::vec3(0.), 1.f, false);
    triangle.addVertex(glm::vec3(-1., 1., 1.), glm::vec3(0.), 1.f, false);
    triangle.addVertex(glm::vec3(1., 1., -1.), glm::vec3(0.), 1.f, false);
    triangle.addVertex(glm::vec3(1., 1., 1.), glm::vec3(0.), 1.f, false);

    // ROOT
    triangle.addDistanceConstraint(1.f, {0, 1}, 1.f, EQUALITY_CONSTRAINT);
    triangle.addDistanceConstraint(1.f, {0, 2}, 1.f, EQUALITY_CONSTRAINT);
    triangle.addDistanceConstraint(1.f, {0, 3}, 1.f, EQUALITY_CONSTRAINT);
    triangle.addDistanceConstraint(1.f, {0, 4}, 1.f, EQUALITY_CONSTRAINT);

    // SQUARE
    triangle.addDistanceConstraint(1.f, {1, 2}, 3.f, EQUALITY_CONSTRAINT);
    triangle.addDistanceConstraint(1.f, {2, 3}, 3.f, EQUALITY_CONSTRAINT);
    triangle.addDistanceConstraint(1.f, {3, 4}, 3.f, EQUALITY_CONSTRAINT);

    // for (Mesh &mesh : meshes) {
    //     mesh.init();
    // }

    // TODO: init textures
    // TODO: setup lights

    // timings
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    uint frame_count = 0;
    glfwSwapInterval(1); // VSync - avoid having 3000 fps
    do {
        frame_count++;

        glfwSwapBuffers(window);
        glfwPollEvents();

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Imgui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // OBJECTS UPDATE
        // rhino_transfo.setTranslation(glm::vec3(0., 0., -1.5));
        // rhino_transfo.setEulerAngles(glm::vec3(0., currentFrame * 2. * M_PI * 0.1, 0.));
        // rhino_transfo.updateRotation();
        // glm::vec4 cam_center = rhino_transfo.computeTransformationMatrix() * glm::vec4(center, 1.0);
        camera.update(window, deltaTime, glm::vec3(0.), cursor_vel, scroll);
        if (next_frame) {
            triangle.update(deltaTime);
            // next_frame = false;
        }

        // RENDER
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        shader.use();                                       // Use program

        // Update uniforms
        glm::mat4 projection = camera.getProjectionMatrix();
        glm::mat4 view = camera.getViewMatrix();
        shader.set("projection", projection);
        shader.set("view", view);

        // Render Meshes
        // for (int i = 0; i < meshes.size(); i++) {
        //     glm::mat4 model = glm::mat4(1.);
        //     glm::mat4 model_view = view * model;
        //     glm::mat4 normal_mat = glm::transpose(glm::inverse(model_view));
        //     shader.set("model_view", model_view);
        //     shader.set("normal_mat", normal_mat);
        //     meshes[i].render();
        // }
        triangle.init();
        triangle.render();

        // ImGui Render
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Reset some controls
        scroll = glm::vec2(0.);
        cursor_vel = glm::vec2(0.);
    } while (glfwWindowShouldClose(window) == GLFW_FALSE);

    shader.~ShaderProgram();
    // for (Mesh &mesh : meshes) {
    //     mesh.clear();
    // }
    triangle.clear();

    glfwTerminate();

    return 0;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    // cout << "framebuffer size: " << width << ", " << height << endl;
    window_width = width;
    window_height = height;
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
    } else if (key == GLFW_KEY_SPACE) {
        next_frame = true;
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods) {
    // cout << "mouse button:" << button << " action:" << action << " mods:" << mods << endl;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
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
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);

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
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // IF using Docking Branch
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true); // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
    ImGui_ImplOpenGL3_Init();

    initOpenGL();
}
