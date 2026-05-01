#pragma once

#include "Vector3f.h"
#include "light.hpp"
#include <cassert>
#include <vecmath.h>
#include <vector>

class Camera;
class Light;
class Material;
enum class MaterialType;
class Object3D;
class Group;
class Sphere;
class Plane;
class Triangle;
class Transform;
class Mesh;

#define MAX_PARSER_TOKEN_LENGTH 1024

class SceneParser {
public:

    SceneParser() = delete;
    SceneParser(const char *filename);

    ~SceneParser();

    Camera *getCamera() const {
        return camera;
    }

    Vector3f getBackgroundColor() const {
        return background_color;
    }

    int getNumLights() const {
        return lights.size();
    }

    Light *getLight(int i) const {
        assert(i >= 0 && i < int(lights.size()));
        return lights[i];
    }

    int getNumMaterials() const {
        return materials.size();
    }

    Material *getMaterial(int i) const {
        assert(i >= 0 && i < int(materials.size()));
        return materials[i];
    }

    Group *getGroup() const {
        return group;
    }
    
    Light::SampleResult sampleLight(const Vector3f &p) const;

private:

    void parseFile();
    void parsePerspectiveCamera();
    void parseBackground();
    void parseLights();
    Light *parsePointLight();
    Light *parseDirectionalLight();
    void parseMaterials();
    Material *parseMaterial(MaterialType defaultType);
    Object3D *parseObject(char token[MAX_PARSER_TOKEN_LENGTH]);
    Group *parseGroup();
    Sphere *parseSphere();
    Plane *parsePlane();
    Triangle *parseTriangle();
    Mesh *parseTriangleMesh();
    Transform *parseTransform();
    void generateAreaLights(Object3D *obj, const Matrix4f &parentMatrix = Matrix4f::identity());

    int getToken(char token[MAX_PARSER_TOKEN_LENGTH]);

    Vector3f readVector3f();

    float readFloat();
    int readInt();

    FILE *file;
    Camera *camera;
    Vector3f background_color;
    std::vector<Light*> lights;
    std::vector<Material*> materials;
    Material *current_material;
    Group *group;
    std::vector<Object3D*> aux_objects;
    std::vector<float> aux_sizes;
};
