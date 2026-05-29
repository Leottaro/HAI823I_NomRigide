// GLEW
#include <GL/glew.h>

// GLM
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
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
#include "Profiling.hpp"
#include "Scene.hpp"
#include "ShaderProgram.hpp"

// OPENMP
#ifdef USE_OPENMP
#include <omp.h>
#endif

using namespace std;

// TODO: SINGLETON
GLuint window_width = 1280, window_height = 720;
int window_pox, window_poy;
bool fullscreen_window = false;

glm::vec2 cursor_pos = glm::vec2(0, 0);
glm::vec3 cursor_worldpos = glm::vec3(0, 0, 0);
glm::vec2 cursor_vel = glm::vec2(0, 0);
glm::vec2 scroll = glm::vec2(0, 0);
int polygon_mode = GL_FILL;
GLFWwindow* window;

Camera camera;
bool run_simulation = false;

void globalInit();
void renderProfilingInterface(float deltaTime, const ProfileFrame& profile_average);

void BunnyClothScene(Scene& scene) {
    scene.do_self_collision = false;
    scene.do_inter_dynamic_collision = false;
    scene.do_fixed_delta_time = true;
    scene.fixed_delta_time = 0.01;

    scene.addMesh(LoadedMesh{"ressources/models/bunny2.off"});
    scene.addMesh(SimpleGridMesh{40, 40});

    Transformation* lapin_transfo = scene.addStaticBody(StaticBodyDesc{"Lapin", 0});
    lapin_transfo->setTranslation(glm::vec3(0.f, 0.f, 0.f));
    lapin_transfo->setEulerAngles(glm::vec3(-glm::pi<float>() / 2.f, 0.f, glm::pi<float>()));
    lapin_transfo->setScale(glm::vec3(5.f, 5.f, 5.f));

    DynamicObjectDesc filet_desc("filet", 1, DynamicObjectDescPreset::Cloth);
    filet_desc.render_type = DynamicRenderType::LineRender;
    Transformation* filet_transfo = scene.addDynamicObject(filet_desc);
    filet_transfo->setTranslation(glm::vec3(0.f, 5.f, 0.f));
    filet_transfo->setScale(glm::vec3(10.f, 10.f, 10.f));
}

void BunnyClothBalloonScene(Scene& scene) {
    scene.do_gravity = false;
    scene.do_self_collision = false;
    scene.do_inter_dynamic_collision = false;
    scene.do_fixed_delta_time = true;
    scene.fixed_delta_time = 0.01;

    scene.addMesh(LoadedMesh{"ressources/models/bunny1.off"});

    DynamicObjectDesc lapin_desc("Lapin", 0, DynamicObjectDescPreset::ClothBalloon);
    lapin_desc.render_type = DynamicRenderType::TriangleRender;
    Transformation* lapin_transfo = scene.addDynamicObject(lapin_desc);
    lapin_transfo->setTranslation(glm::vec3(0.f, 0.f, 0.f));
    lapin_transfo->setScale(glm::vec3(5.f, 5.f, 5.f));
    lapin_transfo->setEulerAngles(glm::vec3(-glm::pi<float>() / 2.f, 0.f, glm::pi<float>()));
}

void netRockScene1(Scene& scene) {
    scene.do_self_collision = true;
    scene.do_inter_dynamic_collision = true;
    scene.do_fixed_delta_time = true;
    scene.fixed_delta_time = 0.01;

    uint grid_size = 40;
    scene.addMesh(SimpleGridMesh{grid_size, grid_size});
    scene.addMesh(CubeSphereMesh{3});

    DynamicObjectDesc filet_desc("filet", 0, DynamicObjectDescPreset::Cloth);
    filet_desc.render_type = DynamicRenderType::LineRender;
    filet_desc.fixed_vertices = {0, grid_size - 1, grid_size * (grid_size - 1), grid_size * grid_size - 1};
    filet_desc.distance_stiffness = 0.9f;
    Transformation* filet_transfo = scene.addDynamicObject(filet_desc);
    filet_transfo->setTranslation(glm::vec3(0.f, 0.f, 0.f));
    filet_transfo->setScale(glm::vec3(10.f, 10.f, 10.f));

    DynamicObjectDesc sphere_desc("sphere", 1, DynamicObjectDescPreset::RigidBody);
    sphere_desc.render_type = DynamicRenderType::TriangleRender;
    sphere_desc.render_color = glm::vec3(1.f, 0.f, 0.f);
    sphere_desc.vertex_mass = 100.f;
    Transformation* sphere_transformation = scene.addDynamicObject(sphere_desc);
    sphere_transformation->setScale(glm::vec3(1.f, 1.f, 1.f));
    sphere_transformation->setTranslation(glm::vec3(0.f, 2.f, 0.f));
}

void netRockScene2(Scene& scene) {
    scene.do_self_collision = true;
    scene.do_inter_dynamic_collision = true;
    scene.do_fixed_delta_time = true;
    scene.fixed_delta_time = 0.01;

    uint grid_size = 40;
    scene.addMesh(SimpleGridMesh{grid_size, grid_size});
    scene.addMesh(CubeSphereMesh{3});

    DynamicObjectDesc filet_desc("filet", 0, DynamicObjectDescPreset::Cloth);
    filet_desc.render_type = DynamicRenderType::LineRender;
    filet_desc.fixed_vertices = {0, grid_size - 1, grid_size * (grid_size - 1), grid_size * grid_size - 1};
    filet_desc.distance_stiffness = 0.5f;
    Transformation* filet_transfo = scene.addDynamicObject(filet_desc);
    filet_transfo->setTranslation(glm::vec3(0.f, 0.f, 0.f));
    filet_transfo->setScale(glm::vec3(10.f, 10.f, 10.f));

    DynamicObjectDesc sphere_desc("sphere", 1, DynamicObjectDescPreset::RigidBody);
    sphere_desc.render_type = DynamicRenderType::TriangleRender;
    sphere_desc.render_color = glm::vec3(1.f, 0.f, 0.f);
    sphere_desc.vertex_mass = 10.f;
    Transformation* sphere_transformation = scene.addDynamicObject(sphere_desc);
    sphere_transformation->setScale(glm::vec3(1.f, 1.f, 1.f));
    sphere_transformation->setTranslation(glm::vec3(0.f, 2.f, 0.f));
}

int main(void) {
    globalInit();

    ShaderProgram mesh_shader = ShaderProgram("ressources/shaders/mesh_vertex.glsl", "ressources/shaders/mesh_fragment.glsl");
    mesh_shader.link();
    ShaderProgram dynamic_shader = ShaderProgram("ressources/shaders/dynamic_vertex.glsl", "ressources/shaders/dynamic_fragment.glsl");
    dynamic_shader.link();

    Scene scene = Scene();
    // BunnyClothScene(scene);
    BunnyClothBalloonScene(scene);
    // netRockScene1(scene);
    // netRockScene2(scene);

    scene.resetObjects();

    // timings
    float deltaTime = 0.0f;
    float lastFrame = glfwGetTime();
    ProfileFrame profile_accumulator;
    ProfileFrame profile_average;
    size_t profile_sample_count = 0;
    double last_profile_update = lastFrame;
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
        resetProfileFrame();
        g_profile_frame.frame_ms = deltaTime * 1000.;

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
            int num_subSteps = scene.num_subSteps;
            float clamped_dt = std::min(deltaTime, 0.033f);
            float sub_dt = clamped_dt / (float)num_subSteps;

            for (int i = 0; i < num_subSteps; i++) {
                if (!scene.updateSimulation(sub_dt, clamped_dt, i == 0)) {
                    run_simulation = false;
                    break;
                }
            }

            // update scene for rendering after all small steps simulation
            scene.updateAllRendredPositions();
        }
        camera.update(window, deltaTime, cursor_vel, scroll, disable_mouse_actions);

        // RENDERING
        {
            ScopedTimer timer(g_profile_frame.rendering_ms);
            scene.render(dynamic_shader, mesh_shader, camera);
        }

        renderProfilingInterface(deltaTime, profile_average);

        // ImGui Render
        {
            ScopedTimer timer(g_profile_frame.rendering_ms);
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }

        profile_accumulator += g_profile_frame;
        profile_sample_count++;
        if (currentFrame - last_profile_update >= 0.5 && profile_sample_count > 0) {
            profile_average = profile_accumulator / static_cast<double>(profile_sample_count);
            profile_accumulator = {};
            profile_sample_count = 0;
            last_profile_update = currentFrame;
        }

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

void renderProfilingInterface(float deltaTime, const ProfileFrame& profile_average) {
    if (ImGui::Begin("Performance")) {
        double instant_frame_ms = deltaTime * 1000.;
        double instant_fps = deltaTime > 0.f ? 1. / deltaTime : 0.;
        double average_fps = profile_average.frame_ms > 0. ? 1000. / profile_average.frame_ms : instant_fps;

        ImGui::Text("FPS: %.1f", average_fps);
        ImGui::Text("Frame: %.3f ms", profile_average.frame_ms > 0. ? profile_average.frame_ms : instant_frame_ms);
        ImGui::Separator();

        ImGui::Text("Simulation");
        ImGui::Text("Collision detection: %.3f ms", profile_average.collision_ms);
        ImGui::Indent();
        ImGui::Text("Point -> Triangle: %.3f ms", profile_average.point_triangle_collision_ms);
        ImGui::Text("Edge -> Triangle: %.3f ms", profile_average.edge_triangle_collision_ms);
        ImGui::Text("Triangle -> Point: %.3f ms", profile_average.triangle_point_collision_ms);
        ImGui::Text("Self Point -> Triangle: %.3f ms", profile_average.self_point_triangle_collision_ms);
        ImGui::Unindent();
        ImGui::Text("Constraint projection: %.3f ms", profile_average.constraints_ms);
        ImGui::Separator();

        ImGui::Text("Collision counts");
        ImGui::Indent();
        ImGui::Text("Point -> Triangle: %.1f", profile_average.point_triangle_collision_count);
        ImGui::Text("Edge -> Triangle: %.1f", profile_average.edge_triangle_collision_count);
        ImGui::Text("Triangle -> Point: %.1f", profile_average.triangle_point_collision_count);
        ImGui::Text("Self Point -> Triangle: %.1f", profile_average.self_point_triangle_collision_count);
        ImGui::Unindent();
        ImGui::Separator();

        ImGui::Text("Rendering: %.3f ms", profile_average.rendering_ms);
    }
    ImGui::End();
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    // cout << "framebuffer size: " << width << ", " << height << endl;
    window_width = width;
    window_height = height;
    glViewport(0, 0, width, height);
}

bool space_key_pressed = false;
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    // cout << "key:" << key << " scancode:" << scancode << " action:" << action << " mods:" << mods << endl;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }

    if (key == GLFW_KEY_Z && action == GLFW_PRESS) {
        if (polygon_mode == GL_FILL) {
            polygon_mode = GL_LINE;
        } else if (polygon_mode == GL_LINE) {
            polygon_mode = GL_POINT;
        } else if (polygon_mode == GL_POINT) {
            polygon_mode = GL_FILL;
        }
        glPolygonMode(GL_FRONT_AND_BACK, polygon_mode);
    }

    if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        fullscreen_window = !fullscreen_window;
        if (fullscreen_window) {
            GLFWmonitor* monitor = glfwGetPrimaryMonitor();
            const GLFWvidmode* mode = glfwGetVideoMode(monitor);
            glfwGetWindowPos(window, &window_pox, &window_poy);
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, GLFW_DONT_CARE);
            window_width = mode->width;
            window_height = mode->height;
        } else {
            glfwSetWindowMonitor(window, nullptr, window_pox, window_poy, window_width, window_height, GLFW_DONT_CARE);
        }
    }

    if (key == GLFW_KEY_SPACE) {
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

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    // cout << "mouse button:" << button << " action:" << action << " mods:" << mods << endl;
    if (button == GLFW_MOUSE_BUTTON_LEFT && mods == 0) {
        glfwSetInputMode(window, GLFW_CURSOR, action == GLFW_PRESS ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
    }
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    cursor_vel.x = xpos - cursor_pos.x;
    cursor_vel.y = ypos - cursor_pos.y;
    cursor_pos.x = xpos;
    cursor_pos.y = ypos;

    // cout << "cursor_pos: (" << cursor_pos.x << ", " << cursor_pos.y << ")\tcursor_vel: (" << cursor_vel.x << ", " << cursor_vel.y << ")" << endl;
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
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

#ifdef USE_OPENMP
    std::cout << "OpenMP threads: " << omp_get_max_threads() << std::endl;
#else
    std::cout << "OpenMP disabled" << std::endl;
#endif
}
