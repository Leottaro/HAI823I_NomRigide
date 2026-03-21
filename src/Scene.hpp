#include <vector>

#include "DynamicObject.hpp"

struct DynamicObjectDesc {
    Mesh* mesh;
    Transformation* transfo;
    char* name;
    float distance_stiffness;
    float angle_stiffness;

    DynamicObjectDesc(Mesh* mesh, Transformation* transfo, char* name, float distance_stiffness, float angle_stiffness)
        : mesh(mesh), transfo(transfo), name(name), distance_stiffness(distance_stiffness), angle_stiffness(angle_stiffness) {}
};

class Scene {
   private:
    std::vector<StaticBody> pre_static_bodies = {};
    std::vector<DynamicObjectDesc> pre_dynamic_objects = {};
    std::vector<StaticBody> static_bodies;
    std::vector<DynamicObject> dynamic_objects;
    bool should_reset = false;

   public:
    inline std::vector<StaticBody>& getStaticBodies() { return static_bodies; }
    inline std::vector<DynamicObject>& getDynamicObjects() { return dynamic_objects; }
    inline void addStaticBody(StaticBody& _sb) { pre_static_bodies.push_back(_sb); }
    inline void addDynamicObject(Mesh* _mesh, Transformation* _transfo, char* _name, float _distance_stiffness, float _angle_stiffness) {
        pre_dynamic_objects.push_back(DynamicObjectDesc(_mesh, _transfo, _name, _distance_stiffness, _angle_stiffness));
    }
    inline void addStaticBody(Mesh* _mesh, Transformation* _transfo, char* _name) { pre_static_bodies.push_back(StaticBody(_mesh, _transfo, _name)); }
    void init();
    void updateProps();
};