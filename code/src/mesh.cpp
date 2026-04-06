#include "mesh.hpp"
#include "triangle.hpp"
#include <iostream>

bool Mesh::intersect(const Ray &r, Hit &h, float tmin) {

    // Optional: Change this brute force method into a faster one.
    bool result = false;
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        Material* tri_mat = material;
        if (!t_matIndices.empty() && t_matIndices[triId] >= 0 && t_matIndices[triId] < (int)mtl_materials.size()) {
            tri_mat = mtl_materials[t_matIndices[triId]];
        }
        Triangle triangle(v[triIndex[0]],
                          v[triIndex[1]], v[triIndex[2]], tri_mat);
        triangle.normal = n[triId];
        result |= triangle.intersect(r, h, tmin);
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
    // if (!reader.Warning().empty()) {
    //     std::cout << "TinyObjReader: " << reader.Warning();
    // }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Parse MTL materials
    for (const auto& m : materials) {
        mtl_materials.push_back(new Material(
            Vector3f(m.diffuse[0], m.diffuse[1], m.diffuse[2]),
            Vector3f(m.specular[0], m.specular[1], m.specular[2]),
            m.shininess
        ));
    }

    // Copy vertices
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        v.push_back(Vector3f(attrib.vertices[i], attrib.vertices[i+1], attrib.vertices[i+2]));
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
                t.push_back(trig);
                t_matIndices.push_back(shapes[s].mesh.material_ids[f]);
            }
            index_offset += fv;
        }
    }

    computeNormal();
}

Mesh::~Mesh() {
    for (auto mat : mtl_materials) {
        delete mat;
    }
}

void Mesh::computeNormal() {
    n.resize(t.size());
    for (int triId = 0; triId < (int) t.size(); ++triId) {
        TriangleIndex& triIndex = t[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        n[triId] = b / b.length();
    }
}
