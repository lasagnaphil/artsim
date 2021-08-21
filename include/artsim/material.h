//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_MATERIAL_H
#define ARTSIM_MATERIAL_H

#include <artsim/types.h>
#include <artsim/core/arena.h>
#include <unordered_map>

namespace artsim {

struct Material {
    real friction = 1.0f;
    real restitution = 0.0f;
    real restitution_threshold = 0.01f;
};

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (std::pair<T1, T2> const &v) const
    {
        using std::hash;
        return hash<T1>()(v.first) ^ (hash<T2>()(v.second) << 1);
    }
};

struct MaterialDB {
    Arena<Material> materials;
    std::unordered_map<std::pair<Id<Material>, Id<Material>>, Material, pair_hash> material_pairs;

    void clear();

    Id<Material> add_material(real default_friction = 1.0f,
                              real default_restitution = 0.0f,
                              real default_restitution_threshold = 0.01f);
    Material* get_material(Id<Material> id) { return materials.get(id); }
    void remove_material(Id<Material> id) { materials.release(id); }

    void set_material_pair(Id<Material> mat1_id, Id<Material> mat2_id,
                           real friction, real restitution, real restitution_threshold);

    Material get_material_pair(Id<Material> mat1_id, Id<Material> mat2_id);
};

}

#endif //ARTSIM_MATERIAL_H
