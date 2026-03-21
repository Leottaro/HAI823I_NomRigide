#include "Scene.hpp"

#include <iostream>

void Scene::init() {
    static_bodies = pre_static_bodies;
    dynamic_objects.clear();
    for (const DynamicObjectDesc obj : pre_dynamic_objects) {
        dynamic_objects.push_back(DynamicObject::bodyFromMesh(StaticBody(obj.mesh, obj.transfo, obj.name), obj.distance_stiffness, obj.angle_stiffness));
    }
    std::cout << "Initialisation terminée avec " << static_bodies.size() << " objets statiques et " << dynamic_objects.size() << " objets dynamiques." << std::endl;
}

void Scene::updateProps() {
    if (ImGui::Begin("Objects")) {
        for (StaticBody& obj : pre_static_bodies) {
            ImGui::Text("%s", obj.m_name);
            glm::vec3 translation = obj.m_transformation->getTranslation();
            ImGui::DragFloat("Xpos", &translation.x, .01, -10., 10.);
            ImGui::DragFloat("Ypos", &translation.y, .01, -10., 10.);
            ImGui::DragFloat("Zpos", &translation.z, .01, -10., 10.);
            obj.m_transformation->setTranslation(translation);
        }
    }

    if (ImGui::Button("Reset Objects"))
        init();
    ImGui::End();
}