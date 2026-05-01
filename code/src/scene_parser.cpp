#include <cstddef>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <algorithm>

#include "scene_parser.hpp"
#include "camera.hpp"
#include "light.hpp"
#include "material.hpp"
#include "object3d.hpp"
#include "group.hpp"
#include "mesh.hpp"
#include "sphere.hpp"
#include "plane.hpp"
#include "triangle.hpp"
#include "transform.hpp"
#include "random.hpp"

#define DegreesToRadians(x) ((M_PI * x) / 180.0f)

SceneParser::SceneParser(const char *filename) {

    // initialize some reasonable default values
    group = nullptr;
    camera = nullptr;
    background_color = Vector3f(0.5, 0.5, 0.5);
    current_material = nullptr;

    // parse the file
    assert(filename != nullptr);
    const char *ext = &filename[strlen(filename) - 4];

    if (strcmp(ext, ".txt") != 0) {
        printf("wrong file name extension\n");
        exit(0);
    }
    file = fopen(filename, "r");

    if (file == nullptr) {
        printf("cannot open scene file\n");
        exit(0);
    }
    parseFile();
    fclose(file);
    file = nullptr;

    generateAreaLights(group);
    for (size_t i = 1; i < aux_sizes.size(); ++i) {
        aux_sizes[i] += aux_sizes[i - 1];
    }

    if (lights.empty()) {
        printf("WARNING:    No lights specified\n");
    }
}

SceneParser::~SceneParser() {

    delete group;
    delete camera;

    for (auto material : materials) {
        delete material;
    }
    for (auto light : lights) {
        delete light;
    }
    for (auto obj : aux_objects) {
        delete obj;
    }
}

// ====================================================================
// ====================================================================

void SceneParser::parseFile() {
    //
    // at the top level, the scene can have a camera, 
    // background color and a group of objects
    // (we add lights and other things in future assignments)
    //
    char token[MAX_PARSER_TOKEN_LENGTH];
    while (getToken(token)) {
        if (!strcmp(token, "PerspectiveCamera")) {
            parsePerspectiveCamera();
        } else if (!strcmp(token, "Background")) {
            parseBackground();
        } else if (!strcmp(token, "Lights")) {
            parseLights();
        } else if (!strcmp(token, "Materials")) {
            parseMaterials();
        } else if (!strcmp(token, "Group")) {
            group = parseGroup();
        } else {
            printf("Unknown token in parseFile: '%s'\n", token);
            exit(0);
        }
    }
}

// ====================================================================
// ====================================================================

void SceneParser::parsePerspectiveCamera() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    // read in the camera parameters
    getToken(token);
    assert (!strcmp(token, "{"));
    getToken(token);
    assert (!strcmp(token, "center"));
    Vector3f center = readVector3f();
    getToken(token);
    assert (!strcmp(token, "direction"));
    Vector3f direction = readVector3f();
    getToken(token);
    assert (!strcmp(token, "up"));
    Vector3f up = readVector3f();
    getToken(token);
    assert (!strcmp(token, "angle"));
    float angle_degrees = readFloat();
    float angle_radians = DegreesToRadians(angle_degrees);
    getToken(token);
    assert (!strcmp(token, "width"));
    int width = readInt();
    getToken(token);
    assert (!strcmp(token, "height"));
    int height = readInt();
    getToken(token);
    assert (!strcmp(token, "}"));
    camera = new PerspectiveCamera(center, direction, up, width, height, angle_radians);
}

void SceneParser::parseBackground() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    // read in the background color
    getToken(token);
    assert (!strcmp(token, "{"));
    while (true) {
        getToken(token);
        if (!strcmp(token, "}")) {
            break;
        } else if (!strcmp(token, "color")) {
            background_color = readVector3f();
        } else {
            printf("Unknown token in parseBackground: '%s'\n", token);
            assert(0);
        }
    }
}

// ====================================================================
// ====================================================================

void SceneParser::parseLights() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));
    // read in the number of objects
    getToken(token);
    assert (!strcmp(token, "numLights"));
    int num_lights = readInt();
    lights.resize(num_lights);
    // read in the objects
    int count = 0;
    while (num_lights > count) {
        getToken(token);
        if (strcmp(token, "DirectionalLight") == 0) {
            lights[count] = parseDirectionalLight();
        } else if (strcmp(token, "PointLight") == 0) {
            lights[count] = parsePointLight();
        } else {
            printf("Unknown token in parseLight: '%s'\n", token);
            exit(0);
        }
        count++;
    }
    getToken(token);
    assert (!strcmp(token, "}"));
}

Light *SceneParser::parseDirectionalLight() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));
    getToken(token);
    assert (!strcmp(token, "direction"));
    Vector3f direction = readVector3f();
    getToken(token);
    assert (!strcmp(token, "color"));
    Vector3f color = readVector3f();
    getToken(token);
    assert (!strcmp(token, "}"));
    return new DirectionalLight(direction, color);
}

Light *SceneParser::parsePointLight() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));
    getToken(token);
    assert (!strcmp(token, "position"));
    Vector3f position = readVector3f();
    getToken(token);
    assert (!strcmp(token, "color"));
    Vector3f color = readVector3f();
    getToken(token);
    assert (!strcmp(token, "}"));
    return new PointLight(position, color);
}
// ====================================================================
// ====================================================================

void SceneParser::parseMaterials() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));
    // read in the number of objects
    getToken(token);
    assert (!strcmp(token, "numMaterials"));
    int num_materials = readInt();
    materials.resize(num_materials);
    // read in the objects
    int count = 0;
    while (num_materials > count) {
        getToken(token);
        if (!strcmp(token, "Material") ||
            !strcmp(token, "PhongMaterial")) {
            materials[count] = parseMaterial(MaterialType::DIFFUSE);
        } else if (!strcmp(token, "MirrorMaterial")) {
            materials[count] = parseMaterial(MaterialType::MIRROR);
        } else if (!strcmp(token, "GlassMaterial")) {
            materials[count] = parseMaterial(MaterialType::GLASS);
        } else {
            printf("Unknown token in parseMaterial: '%s'\n", token);
            exit(0);
        }
        count++;
    }
    getToken(token);
    assert (!strcmp(token, "}"));
}


static MaterialType parseMaterialTypeName(const char *typeName) {
    if (!strcmp(typeName, "diffuse") || !strcmp(typeName, "Diffuse") ||
        !strcmp(typeName, "phong") || !strcmp(typeName, "Phong")) {
        return MaterialType::DIFFUSE;
    }
    if (!strcmp(typeName, "mirror") || !strcmp(typeName, "Mirror")) {
        return MaterialType::MIRROR;
    }
    if (!strcmp(typeName, "emissive") || !strcmp(typeName, "Emissive")) {
        return MaterialType::EMISSIVE;
    }
    if (!strcmp(typeName, "glass") || !strcmp(typeName, "Glass")) {
        return MaterialType::GLASS;
    }
    printf("Unknown material type '%s'\n", typeName);
    exit(0);
}

Material *SceneParser::parseMaterial(MaterialType defaultType) {
    char token[MAX_PARSER_TOKEN_LENGTH];
    char filename[MAX_PARSER_TOKEN_LENGTH];
    filename[0] = 0;
    MaterialType type = defaultType;
    Vector3f diffuseColor = defaultType == MaterialType::MIRROR ? Vector3f::ZERO : Vector3f(1, 1, 1);
    Vector3f specularColor = defaultType == MaterialType::MIRROR ? Vector3f(1, 1, 1) : Vector3f::ZERO;
    Vector3f emission = Vector3f::ZERO;
    float shininess = 0;
    float ior = 1.5f;
    Vector3f transmissionColor = Vector3f(1, 1, 1);
    bool hasSpecularColor = false;
    bool hasMirrorColor = false;
    getToken(token);
    assert (!strcmp(token, "{"));
    while (true) {
        getToken(token);
        if (strcmp(token, "diffuseColor") == 0) {
            diffuseColor = readVector3f();
        } else if (strcmp(token, "specularColor") == 0) {
            specularColor = readVector3f();
            hasSpecularColor = true;
        } else if (strcmp(token, "mirrorColor") == 0 ||
                   strcmp(token, "reflectance") == 0 ||
                   strcmp(token, "reflectionColor") == 0) {
            specularColor = readVector3f();
            hasMirrorColor = true;
        } else if (strcmp(token, "ior") == 0 ||
            strcmp(token, "refractiveIndex") == 0) {
            ior = readFloat();
        } else if (strcmp(token, "transmissionColor") == 0 ||
                strcmp(token, "transmittance") == 0) {
            transmissionColor = readVector3f();
        } else if (strcmp(token, "shininess") == 0) {
            shininess = readFloat();
        } else if (strcmp(token, "type") == 0 ||
                   strcmp(token, "materialType") == 0) {
            getToken(token);
            type = parseMaterialTypeName(token);
        } else if (strcmp(token, "emission") == 0 ||
                   strcmp(token, "emissionColor") == 0) {
            emission = readVector3f();
        } else if (strcmp(token, "texture") == 0) {
            // Optional: read in texture and draw it.
            getToken(filename);
        } else {
            assert (!strcmp(token, "}"));
            break;
        }
    }
    if (type == MaterialType::MIRROR && !hasSpecularColor && !hasMirrorColor) {
        specularColor = diffuseColor.length() > 0 ? diffuseColor : Vector3f(1, 1, 1);
    }
    if (emission.length() > 0) {
        type = MaterialType::EMISSIVE;
    }
    auto *answer = new Material(diffuseColor, specularColor, emission, shininess, type, transmissionColor, ior);
    return answer;
}

// ====================================================================
// ====================================================================

Object3D *SceneParser::parseObject(char token[MAX_PARSER_TOKEN_LENGTH]) {
    Object3D *answer = nullptr;
    if (!strcmp(token, "Group")) {
        answer = (Object3D *) parseGroup();
    } else if (!strcmp(token, "Sphere")) {
        answer = (Object3D *) parseSphere();
    } else if (!strcmp(token, "Plane")) {
        answer = (Object3D *) parsePlane();
    } else if (!strcmp(token, "Triangle")) {
        answer = (Object3D *) parseTriangle();
    } else if (!strcmp(token, "TriangleMesh")) {
        answer = (Object3D *) parseTriangleMesh();
    } else if (!strcmp(token, "Transform")) {
        answer = (Object3D *) parseTransform();
    } else {
        printf("Unknown token in parseObject: '%s'\n", token);
        exit(0);
    }
    return answer;
}

// ====================================================================
// ====================================================================

Group *SceneParser::parseGroup() {
    //
    // each group starts with an integer that specifies
    // the number of objects in the group
    //
    // the material index sets the material of all objects which follow,
    // until the next material index (scoping for the materials is very
    // simple, and essentially ignores any tree hierarchy)
    //
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));

    // read in the number of objects
    getToken(token);
    assert (!strcmp(token, "numObjects"));
    int num_objects = readInt();

    auto *answer = new Group(num_objects);

    // read in the objects
    int count = 0;
    while (num_objects > count) {
        getToken(token);
        if (!strcmp(token, "MaterialIndex")) {
            // change the current material
            int index = readInt();
            assert (index >= 0 && index <= getNumMaterials());
            current_material = getMaterial(index);
        } else {
            Object3D *object = parseObject(token);
            assert (object != nullptr);
            answer->addObject(count, object);

            count++;
        }
    }
    getToken(token);
    assert (!strcmp(token, "}"));

    // return the group
    return answer;
}

// ====================================================================
// ====================================================================

Sphere *SceneParser::parseSphere() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));
    getToken(token);
    assert (!strcmp(token, "center"));
    Vector3f center = readVector3f();
    getToken(token);
    assert (!strcmp(token, "radius"));
    float radius = readFloat();
    getToken(token);
    assert (!strcmp(token, "}"));
    assert (current_material != nullptr);
    return new Sphere(center, radius, current_material);
}


Plane *SceneParser::parsePlane() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));
    getToken(token);
    assert (!strcmp(token, "normal"));
    Vector3f normal = readVector3f();
    getToken(token);
    assert (!strcmp(token, "offset"));
    float offset = readFloat();
    getToken(token);
    assert (!strcmp(token, "}"));
    assert (current_material != nullptr);
    return new Plane(normal, offset, current_material);
}


Triangle *SceneParser::parseTriangle() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    getToken(token);
    assert (!strcmp(token, "{"));
    getToken(token);
    assert (!strcmp(token, "vertex0"));
    Vector3f v0 = readVector3f();
    getToken(token);
    assert (!strcmp(token, "vertex1"));
    Vector3f v1 = readVector3f();
    getToken(token);
    assert (!strcmp(token, "vertex2"));
    Vector3f v2 = readVector3f();
    getToken(token);
    assert (!strcmp(token, "}"));
    assert (current_material != nullptr);
    return new Triangle(v0, v1, v2, current_material);
}

Mesh *SceneParser::parseTriangleMesh() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    char filename[MAX_PARSER_TOKEN_LENGTH];
    // get the filename
    getToken(token);
    assert (!strcmp(token, "{"));
    getToken(token);
    assert (!strcmp(token, "obj_file"));
    getToken(filename);
    getToken(token);
    assert (!strcmp(token, "}"));
    const char *ext = &filename[strlen(filename) - 4];
    assert(!strcmp(ext, ".obj"));
    std::cout << "loading mesh: " << filename << std::endl;
    Mesh *answer = new Mesh(filename, current_material);

    return answer;
}


Transform *SceneParser::parseTransform() {
    char token[MAX_PARSER_TOKEN_LENGTH];
    Matrix4f matrix = Matrix4f::identity();
    Object3D *object = nullptr;
    getToken(token);
    assert (!strcmp(token, "{"));
    // read in transformations: 
    // apply to the LEFT side of the current matrix (so the first
    // transform in the list is the last applied to the object)
    getToken(token);

    while (true) {
        if (!strcmp(token, "Scale")) {
            Vector3f s = readVector3f();
            matrix = matrix * Matrix4f::scaling(s[0], s[1], s[2]);
        } else if (!strcmp(token, "UniformScale")) {
            float s = readFloat();
            matrix = matrix * Matrix4f::uniformScaling(s);
        } else if (!strcmp(token, "Translate")) {
            matrix = matrix * Matrix4f::translation(readVector3f());
        } else if (!strcmp(token, "XRotate")) {
            matrix = matrix * Matrix4f::rotateX(DegreesToRadians(readFloat()));
        } else if (!strcmp(token, "YRotate")) {
            matrix = matrix * Matrix4f::rotateY(DegreesToRadians(readFloat()));
        } else if (!strcmp(token, "ZRotate")) {
            matrix = matrix * Matrix4f::rotateZ(DegreesToRadians(readFloat()));
        } else if (!strcmp(token, "Rotate")) {
            getToken(token);
            assert (!strcmp(token, "{"));
            Vector3f axis = readVector3f();
            float degrees = readFloat();
            float radians = DegreesToRadians(degrees);
            matrix = matrix * Matrix4f::rotation(axis, radians);
            getToken(token);
            assert (!strcmp(token, "}"));
        } else if (!strcmp(token, "Matrix4f")) {
            Matrix4f matrix2 = Matrix4f::identity();
            getToken(token);
            assert (!strcmp(token, "{"));
            for (int j = 0; j < 4; j++) {
                for (int i = 0; i < 4; i++) {
                    float v = readFloat();
                    matrix2(i, j) = v;
                }
            }
            getToken(token);
            assert (!strcmp(token, "}"));
            matrix = matrix2 * matrix;
        } else {
            // otherwise this must be an object,
            // and there are no more transformations
            object = parseObject(token);
            break;
        }
        getToken(token);
    }

    assert(object != nullptr);
    getToken(token);
    assert (!strcmp(token, "}"));
    return new Transform(matrix, object);
}

// ====================================================================
// ====================================================================

int SceneParser::getToken(char token[MAX_PARSER_TOKEN_LENGTH]) {
    // for simplicity, tokens must be separated by whitespace
    assert (file != nullptr);
    int success = fscanf(file, "%s ", token);
    if (success == EOF) {
        token[0] = '\0';
        return 0;
    }
    return 1;
}


Vector3f SceneParser::readVector3f() {
    float x, y, z;
    int count = fscanf(file, "%f %f %f", &x, &y, &z);
    if (count != 3) {
        printf("Error trying to read 3 floats to make a Vector3f\n");
        assert (0);
    }
    return Vector3f(x, y, z);
}


float SceneParser::readFloat() {
    float answer;
    int count = fscanf(file, "%f", &answer);
    if (count != 1) {
        printf("Error trying to read 1 float\n");
        assert (0);
    }
    return answer;
}


int SceneParser::readInt() {
    int answer;
    int count = fscanf(file, "%d", &answer);
    if (count != 1) {
        printf("Error trying to read 1 int\n");
        assert (0);
    }
    return answer;
}

void SceneParser::generateAreaLights(Object3D *obj, const Matrix4f &parentMatrix) {
    if (obj == nullptr) return;
    if (auto *group_obj = dynamic_cast<Group *>(obj)) {
        for (int i = 0; i < group_obj->getGroupSize(); i++) {
            generateAreaLights(group_obj->getGroupObject(i), parentMatrix);
        }
    } else if (auto *transform_obj = dynamic_cast<Transform *>(obj)) {
        generateAreaLights(transform_obj->getChild(), parentMatrix * transform_obj->getMatrix());
    } else if (auto *mesh_obj = dynamic_cast<Mesh *>(obj)) {
        for (int i = 0; i < (int) mesh_obj->t.size(); ++i) {
            Material *tri_mat = mesh_obj->getMaterial();
            if (!mesh_obj->t_matIndices.empty() && mesh_obj->t_matIndices[i] >= 0 &&
                mesh_obj->t_matIndices[i] < (int) mesh_obj->mtl_materials.size()) {
                tri_mat = mesh_obj->mtl_materials[mesh_obj->t_matIndices[i]];
            }
            if (tri_mat != nullptr && tri_mat->isEmissive()) {
                Vector3f v0 = transformPoint(parentMatrix, mesh_obj->v[mesh_obj->t[i][0]]);
                Vector3f v1 = transformPoint(parentMatrix, mesh_obj->v[mesh_obj->t[i][1]]);
                Vector3f v2 = transformPoint(parentMatrix, mesh_obj->v[mesh_obj->t[i][2]]);
                auto *tri = new Triangle(v0, v1, v2, tri_mat);
                aux_objects.push_back(tri);
                aux_sizes.push_back(tri->getArea());
                lights.push_back(new AreaLight(tri));
            }
        }
    } else {
        std::cout << "warning: non-triangle area light is not support yet" << std::endl;
        Material *material = obj->getMaterial();
        if (material != nullptr && material->isEmissive()) {
            auto *trans = new Transform(parentMatrix, obj);
            // aux_objects.push_back(trans);
            // aux_sizes.push_back(0);
            // lights.push_back(new AreaLight(trans));
        }
    }
}

Light::SampleResult SceneParser::sampleLight(const Vector3f &p) const {
    if (aux_objects.empty()) {
        return {};
    }
    float r = Random::get_float() * aux_sizes.back();
    int i = std::lower_bound(aux_sizes.begin(), aux_sizes.end(), r) - aux_sizes.begin();
    i += lights.size() - aux_objects.size();
    Light::SampleResult res = lights[i]->sample(p);
    res.pdf = 1.0 / aux_sizes.back();
    return res;
}
