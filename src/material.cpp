//
// Created by lasagnaphil on 8/21/21.
//

#include "artsim/material.h"

namespace artsim {

void MaterialDB::clear() {
    materials.clear();
    material_pairs.clear();
}

Id<Material>
MaterialDB::add_material(real default_friction, real default_restitution, real default_restitution_threshold) {
    auto id = materials.make();
    auto ptr = materials.get(id);
    ptr->friction = default_friction;
    ptr->restitution = default_restitution;
    ptr->restitution_threshold = default_restitution_threshold;
    return id;
}

void MaterialDB::set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id, real friction, real restitution,
                                   real restitution_threshold) {
    material_pairs[std::make_pair(mat1_id, mat2_id)] = Material{friction, restitution, restitution_threshold};
}

Material MaterialDB::get_material_pair(Id<Material> mat1_id, Id<Material> mat2_id) {
    auto it = material_pairs.find({mat1_id, mat2_id});
    if (it == material_pairs.end()) {
        Material* mat1 = materials.get(mat1_id);
        Material* mat2 = materials.get(mat2_id);
        Material mat;
        mat.friction = glm::max(mat1->friction, mat2->friction);
        mat.restitution = glm::min(mat1->restitution, mat2->restitution);
        mat.restitution_threshold = glm::max(mat1->restitution_threshold, mat2->restitution_threshold);
        return mat;
    }
    else {
        return it->second;
    }
}

}