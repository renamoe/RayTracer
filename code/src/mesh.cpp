#include "mesh.hpp"
#include "triangle.hpp"
#include "texture.hpp"
#include <iostream>
#include <algorithm>
#include <array>
#include <numeric>
#include <string>

namespace {

constexpr int BVH_STACK_SIZE = 128;

std::string directoryOf(const std::string &filename) {
    std::string::size_type pos = std::string::npos;
    for (std::string::size_type i = filename.size(); i > 0; --i) {
        char c = filename[i - 1];
        if (c == '/' || c == '\\') {
            pos = i - 1;
            break;
        }
    }
    if (pos == std::string::npos) {
        return "";
    }
    return filename.substr(0, pos + 1);
}

} // namespace

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {
    if (!bvhRoot) return false;

    bool result = false;
    std::array<BVHNode*, BVH_STACK_SIZE> stack;
    int stackSize = 0;
    stack[stackSize++] = bvhRoot;

    while (stackSize > 0) {
        BVHNode* node = stack[--stackSize];
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
                stack[stackSize++] = node->left;
            }
            if (node->right != nullptr) {
                stack[stackSize++] = node->right;
            }
        }
    }
    return result;
}

bool Mesh::occluded(const Ray &r, float tmin, float tmax) {
    if (!bvhRoot || tmax < tmin) return false;

    std::array<BVHNode*, BVH_STACK_SIZE> stack;
    int stackSize = 0;
    stack[stackSize++] = bvhRoot;

    while (stackSize > 0) {
        BVHNode* node = stack[--stackSize];
        if (!node->box.intersect(r, tmin, tmax)) {
            continue;
        }
        if (node->count > 0) {
            for (int i = 0; i < node->count; ++i) {
                int triId = triIds[node->start + i];
                if (occludedTriangle(triId, r, tmin, tmax)) {
                    return true;
                }
            }
        } else {
            if (node->left != nullptr) {
                stack[stackSize++] = node->left;
            }
            if (node->right != nullptr) {
                stack[stackSize++] = node->right;
            }
        }
    }
    return false;
}

bool Mesh::getBoundingBox(AABB &box) const {
    if (bvhRoot == nullptr) {
        return false;
    }
    box = bvhRoot->box;
    return true;
}

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

Mesh::Mesh(const char *filename, Material *material) : Object3D(material) {
    std::string objFilename = normalizeTexturePath(filename);
    std::string baseDir = directoryOf(objFilename);

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = baseDir;
    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(objFilename, reader_config)) {
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
        auto *mat = new Material(
            Vector3f(m.diffuse[0], m.diffuse[1], m.diffuse[2]),
            Vector3f(m.specular[0], m.specular[1], m.specular[2]),
            Vector3f(m.emission[0], m.emission[1], m.emission[2]),
            m.shininess
        );
        if (!m.diffuse_texname.empty()) {
            mat->setDiffuseTexture(Texture::load(resolveRelativePath(objFilename, m.diffuse_texname)));
        }
        mtl_materials.push_back(mat);
    }

    // Copy vertices
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        verts.push_back(Vector3f(attrib.vertices[i], attrib.vertices[i+1], attrib.vertices[i+2]));
    }

    for (size_t i = 0; i + 1 < attrib.texcoords.size(); i += 2) {
        texcoords.push_back(Vector2f(attrib.texcoords[i], attrib.texcoords[i + 1]));
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
                    trig.texcoord[v_idx] = idx.texcoord_index;
                }
                trig.matIndex = shapes[s].mesh.material_ids[f];
                tris.push_back(trig);
            }
            index_offset += fv;
        }
    }

    computeTriangleData();
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

void Mesh::computeTriangleData() {
    normals.resize(tris.size());
    triangleData.resize(tris.size());
    for (int triId = 0; triId < (int) tris.size(); ++triId) {
        const TriangleIndex& triIndex = tris[triId];
        TriangleData &data = triangleData[triId];

        data.v0 = verts[triIndex[0]];
        data.e1 = verts[triIndex[1]] - data.v0;
        data.e2 = verts[triIndex[2]] - data.v0;

        Vector3f normal = Vector3f::cross(data.e1, data.e2);
        float normalLength = normal.length();
        data.normal = normalLength > 0.0f ? normal / normalLength : Vector3f::ZERO;
        data.area = 0.5f * normalLength;
        data.centroid = (data.v0 + verts[triIndex[1]] + verts[triIndex[2]]) / 3.0f;
        data.box = AABB(data.v0, verts[triIndex[1]], verts[triIndex[2]]);
        data.material = getTriangleMaterial(triId);
        data.hasTexCoords = true;
        for (int i = 0; i < 3; ++i) {
            int texcoordIndex = triIndex.texcoord[i];
            if (texcoordIndex >= 0 && texcoordIndex < (int)texcoords.size()) {
                data.texCoord[i] = texcoords[texcoordIndex];
            } else {
                data.texCoord[i] = Vector2f::ZERO;
                data.hasTexCoords = false;
            }
        }

        normals[triId] = data.normal;
    }
}

float Mesh::getArea() const {
    float area = 0;
    for (const TriangleData &data : triangleData) {
        area += data.area;
    }
    return area;
}

BVHNode* Mesh::buildBVH(int start, int end) {
    if (start >= end) {
        return nullptr;
    }

    BVHNode* node = new BVHNode();

    int firstTriId = triIds[start];
    node->box = triangleData[firstTriId].box;
    AABB centroidBounds = AABB(triangleData[firstTriId].centroid);
    for (int i = start; i < end; ++i) {
        int triId = triIds[i];
        node->box.expand(triangleData[triId].box);
        centroidBounds.expand(triangleData[triId].centroid);
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
            return triangleData[a].centroid[axis] < triangleData[b].centroid[axis];
        }
    );
    node->left = buildBVH(start, mid);
    node->right = buildBVH(mid, end);
    return node;
}

bool Mesh::intersectTriangle(int triId, const Ray& ray,  Hit& hit , float tmin) const {
    const TriangleData &data = triangleData[triId];
    Vector3f O = ray.getOrigin();
    Vector3f D = ray.getDirection();
    Vector3f DE2 = Vector3f::cross(D, data.e2);
    float det = Vector3f::dot(data.e1, DE2);
    if (std::abs(det) < 1e-6) {
        return false;
    }
    float inv = 1.0f / det;
    Vector3f S = O - data.v0;
    float u = inv * Vector3f::dot(S, DE2);
    if (u < 0 || u > 1.0f) {
        return false;
    }
    Vector3f SE1 = Vector3f::cross(S, data.e1);
    float v = inv * Vector3f::dot(D, SE1);
    if (v < 0 || u + v > 1.0f) {
        return false;
    }
    float t = inv * Vector3f::dot(data.e2, SE1);
    if (t < tmin || t > hit.getT()) {
        return false;
    }
    if (data.hasTexCoords) {
        Vector2f uv = data.texCoord[0] * (1.0f - u - v) +
                      data.texCoord[1] * u +
                      data.texCoord[2] * v;
        hit.set(t, data.material, data.normal, uv);
    } else {
        hit.set(t, data.material, data.normal);
    }
    return true;
}

bool Mesh::occludedTriangle(int triId, const Ray& ray, float tmin, float tmax) const {
    const TriangleData &data = triangleData[triId];
    Vector3f O = ray.getOrigin();
    Vector3f D = ray.getDirection();
    Vector3f DE2 = Vector3f::cross(D, data.e2);
    float det = Vector3f::dot(data.e1, DE2);
    if (std::abs(det) < 1e-6) {
        return false;
    }
    float inv = 1.0f / det;
    Vector3f S = O - data.v0;
    float u = inv * Vector3f::dot(S, DE2);
    if (u < 0 || u > 1.0f) {
        return false;
    }
    Vector3f SE1 = Vector3f::cross(S, data.e1);
    float v = inv * Vector3f::dot(D, SE1);
    if (v < 0 || u + v > 1.0f) {
        return false;
    }
    float t = inv * Vector3f::dot(data.e2, SE1);
    return t >= tmin && t <= tmax;
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
