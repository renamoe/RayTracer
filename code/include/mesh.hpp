#pragma once

#include <vector>
#include <array>
#include "object3d.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

struct BVHNode {
    AABB box;
    BVHNode *left;
    BVHNode *right;
    int start;
    int count;
    
    BVHNode() : box(), left(nullptr), right(nullptr), start(0), count(0) {}
    ~BVHNode() {
        if (left) delete left;
        if (right) delete right;
    }
};


class Mesh : public Object3D {

public:
    Mesh(const char *filename, Material *m);
    ~Mesh() override;

    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        const int &operator[](const int i) const { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
        int texcoord[3] = {-1, -1, -1};
        int normal[3] = {-1, -1, -1};
        int matIndex = -1;
    };
    bool intersect(const Ray &r, Hit &h, float tmin) override;
    bool occluded(const Ray &r, float tmin, float tmax) override;
    bool getBoundingBox(AABB &box) const override;
    Material *getTriangleMaterial(int triId) const;

    float getArea() const override;

private:
    struct TriangleData {
        Vector3f v0;
        Vector3f e1;
        Vector3f e2;
        Vector3f normal;
        Vector3f centroid;
        AABB box;
        Material *material = nullptr;
        Vector3f vertexNormal[3];
        Vector2f texCoord[3];
        bool hasNormals = false;
        bool hasTexCoords = false;
        float area = 0.0f;
    };

    void computeTriangleData();

    BVHNode* buildBVH(int start, int end);
    bool intersectTriangle(int triId, const Ray& ray,  Hit& hit , float tmin) const;
    bool occludedTriangle(int triId, const Ray& ray, float tmin, float tmax) const;

public:
    std::vector<Vector3f> verts;
    std::vector<Vector2f> texcoords;
    std::vector<TriangleIndex> tris;
    std::vector<Vector3f> normals;
    std::vector<Material*> mtl_materials;
private:
    std::vector<Vector3f> objNormals;
    std::vector<int> triIds;
    std::vector<TriangleData> triangleData;
    BVHNode* bvhRoot = nullptr;
};

