//
// Created by lasagnaphil on 8/21/21.
//

#ifndef ARTSIM_COLLISION_SHAPE_H
#define ARTSIM_COLLISION_SHAPE_H

#include <artsim/types.h>
#include <artsim/core/arena.h>

#include <Discregrid/cubic_lagrange_discrete_grid.hpp>
#include <Discregrid/geometry/mesh_distance.hpp>
#include <Discregrid/mesh/triangle_mesh.hpp>

#include <memory>

struct btCollisionShape;

namespace artsim {

struct CollisionMesh {
    std::unique_ptr<Discregrid::TriangleMesh> mesh;
    std::unique_ptr<Discregrid::MeshDistance> bvh;
    std::unique_ptr<Discregrid::CubicLagrangeDiscreteGrid> sdf;
    enum class Type { BVH, SDF } type;

    void init_bvh(const char* objfile);
    void init_sdf(const char* objfile, real sdf_grid_size);
};

struct CollisionShape {
    enum class Type {
        Ground, Box, Sphere, Mesh
    };
    Type type;
    glm::rvec3 scale;
    glm::rvec3 color; // For debug rendering purposes

    real mass(real density) const;
    glmx::tsmat3x3<real> inertia(real density) const;
};

}
#endif //ARTSIM_COLLISION_SHAPE_H
