#include "mesh.hpp"
#include "triangle.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>

float minOnDim(const Vector3f &a, const Vector3f &b, const Vector3f &c, int dim) {
    return std::min(a[dim], std::min(b[dim], c[dim]));
}
float maxOnDim(const Vector3f &a, const Vector3f &b, const Vector3f &c, int dim) {
    return std::max(a[dim], std::max(b[dim], c[dim]));
}

AABB::AABB(const Vector3f &a, const Vector3f &b, const Vector3f &c) {
    min = Vector3f(minOnDim(a, b, c, 0), minOnDim(a, b, c, 1), minOnDim(a, b, c, 2));
    max = Vector3f(maxOnDim(a, b, c, 0), maxOnDim(a, b, c, 1), maxOnDim(a, b, c, 2));
}

void AABB::expand(const AABB &other) {
    min = Vector3f(std::min(min.x(), other.min.x()), std::min(min.y(), other.min.y()), std::min(min.z(), other.min.z()));
    max = Vector3f(std::max(max.x(), other.max.x()), std::max(max.y(), other.max.y()), std::max(max.z(), other.max.z()));
}

int AABB::longestAxis() const {
    Vector3f diag = max - min;
    if (diag.x() > diag.y() && diag.x() > diag.z()) {
        return 0;
    } else if (diag.y() > diag.z()) {
        return 1;
    } else {
        return 2;
    }
}

bool AABB::intersect(const Ray &ray, float tmin, float tmax) const {
    for (int dim = 0; dim < 3; ++dim) {
        float invD = 1.0f / ray.getDirection()[dim];
        float t0 = (min[dim] - ray.getOrigin()[dim]) * invD;
        float t1 = (max[dim] - ray.getOrigin()[dim]) * invD;
        if (invD < 0.0f) {
            std::swap(t0, t1);
        }
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax <= tmin) {
            return false;
        }
    }
    return true;
}

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {
    if (!bvhRoot) return false;

    bool result = false;
    std::vector<BVHNode*> stack;
    stack.push_back(bvhRoot);

    while (!stack.empty()) {
        BVHNode* node = stack.back();
        stack.pop_back();
        if (!node->box.intersect(r, tmin, h.getT())) {
            continue;
        }
        if (node->count > 0) {
            for (int i = 0; i < node->count; ++i) {
                int triId = triIds[node->start + i];
                result |= intersectTriangle(triId, r, h, tmin);
            }
        } else {
            if (node->left != nullptr) {
                stack.push_back(node->left);
            }
            if (node->right != nullptr) {
                stack.push_back(node->right);
            }
        }
    }
    return result;
}

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

Mesh::Mesh(const char *filename, Material *material) : Object3D(material) {

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = ""; // Auto-deduce from filename
    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(filename, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        return;
    }

    // Warnings like missing .mtl files are normal since we manage materials globally via the scene file.
    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Parse MTL materials
    for (const auto& m : materials) {
        mtl_materials.push_back(new Material(
            Vector3f(m.diffuse[0], m.diffuse[1], m.diffuse[2]),
            Vector3f(m.specular[0], m.specular[1], m.specular[2]),
            Vector3f(m.emission[0], m.emission[1], m.emission[2]),
            m.shininess
        ));
    }

    // Copy vertices
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        verts.push_back(Vector3f(attrib.vertices[i], attrib.vertices[i+1], attrib.vertices[i+2]));
    }

    // Copy triangles
    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            int fv = shapes[s].mesh.num_face_vertices[f];

            // tinyobjloader automatically triangulates polygons by default.
            if (fv == 3) {
                TriangleIndex trig;
                for (size_t v_idx = 0; v_idx < 3; v_idx++) {
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v_idx];
                    trig[v_idx] = idx.vertex_index;
                }
                trig.matIndex = shapes[s].mesh.material_ids[f];
                tris.push_back(trig);
            }
            index_offset += fv;
        }
    }

    computeNormal();
    computeBoxesAndCentroids();
    triIds.resize(tris.size());
    std::iota(triIds.begin(), triIds.end(), 0);
    if (!tris.empty()) {
        bvhRoot = buildBVH(0, (int)tris.size());
    }
    // std::cout << "BVH built for mesh " << filename << " with " << tris.size() << " triangles." << std::endl;
}

Mesh::~Mesh() {
    for (auto mat : mtl_materials) {
        delete mat;
    }
    if (bvhRoot) {
        delete bvhRoot;
    }
}

void Mesh::computeNormal() {
    normals.resize(tris.size());
    for (int triId = 0; triId < (int) tris.size(); ++triId) {
        TriangleIndex& triIndex = tris[triId];
        Vector3f a = verts[triIndex[1]] - verts[triIndex[0]];
        Vector3f b = verts[triIndex[2]] - verts[triIndex[0]];
        b = Vector3f::cross(a, b);
        normals[triId] = b / b.length();
    }
}

float Mesh::getArea() const {
    float area = 0;
    for (int triId = 0; triId < (int) tris.size(); ++triId) {
        const TriangleIndex& triIndex = tris[triId];
        Vector3f a = verts[triIndex[1]] - verts[triIndex[0]];
        Vector3f b = verts[triIndex[2]] - verts[triIndex[0]];
        area += 0.5 * Vector3f::cross(a, b).length();
    }
    return area;
}

void Mesh::computeBoxesAndCentroids() {
    int n = tris.size();
    triBoxes.resize(n);
    triCentroids.resize(n);
    for (int triId = 0; triId < n; ++triId) {
        triBoxes[triId] = AABB(verts[tris[triId][0]], verts[tris[triId][1]], verts[tris[triId][2]]);
        triCentroids[triId] = (verts[tris[triId][0]] + verts[tris[triId][1]] + verts[tris[triId][2]]) / 3.0f;
    }
}

BVHNode* Mesh::buildBVH(int start, int end) {
    if (start >= end) {
        return nullptr;
    }

    BVHNode* node = new BVHNode();

    int firstTriId = triIds[start];
    node->box = triBoxes[firstTriId];
    AABB centroidBounds = AABB(triCentroids[firstTriId]);
    for (int i = start; i < end; ++i) {
        int triId = triIds[i];
        node->box.expand(triBoxes[triId]);
        centroidBounds.expand(AABB(triCentroids[triId]));
    }

    int count = end - start;
    if (count <= 4) {
        node->start = start;
        node->count = count;
        return node;
    }
    
    int axis = centroidBounds.longestAxis();
    int mid = (start + end) / 2;
    std::nth_element(
        triIds.begin() + start, 
        triIds.begin() + mid,
        triIds.begin() + end,
        [&](int a, int b) {
            return triCentroids[a][axis] < triCentroids[b][axis];
        }
    );
    node->left = buildBVH(start, mid);
    node->right = buildBVH(mid, end);
    return node;
}

bool Mesh::intersectTriangle(int triId, const Ray& ray,  Hit& hit , float tmin) const {
    Vector3f E1 = verts[tris[triId][1]] - verts[tris[triId][0]];
    Vector3f E2 = verts[tris[triId][2]] - verts[tris[triId][0]];
    Vector3f O = ray.getOrigin();
    Vector3f D = ray.getDirection();
    Vector3f DE2 = Vector3f::cross(D, E2);
    float det = Vector3f::dot(E1, DE2);
    if (std::abs(det) < 1e-6) {
        return false;
    }
    float inv = 1.0f / det;
    Vector3f S = O - verts[tris[triId][0]];
    float u = inv * Vector3f::dot(S, DE2);
    if (u < 0 || u > 1.0f) {
        return false;
    }
    Vector3f SE1 = Vector3f::cross(S, E1);
    float v = inv * Vector3f::dot(D, SE1);
    if (v < 0 || u + v > 1.0f) {
        return false;
    }
    float t = inv * Vector3f::dot(E2, SE1);
    if (t < tmin || t > hit.getT()) {
        return false;
    }
    hit.set(t, getTriangleMaterial(triId), normals[triId]);
    return true;
}

Material *Mesh::getTriangleMaterial(int triId) const {
    if (triId >= 0 && triId < (int)tris.size()) {
        int matIndex = tris[triId].matIndex;
        if (matIndex >= 0 && matIndex < (int)mtl_materials.size()) {
            return mtl_materials[matIndex];
        }
    }
    return material;
}
