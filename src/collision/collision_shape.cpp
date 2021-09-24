//
// Created by lasagnaphil on 8/21/21.
//

#include <artsim/collision/collision_shape.h>
#include <artsim/math/eigen.h>
#include <artsim/math/bullet.h>

#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>

using namespace artsim;
using namespace glmx;

void CollisionMesh::init_bvh(const char* objfile) {
    type = CollisionMesh::Type::BVH;
    printf("Loading mesh %s...\n", objfile);
    mesh = std::make_unique<Discregrid::TriangleMesh>(objfile);
    printf("Initializing BVH structure for mesh %s...\n", objfile);
    bvh = std::make_unique<Discregrid::MeshDistance>(mesh.get());
}

void CollisionMesh::init_sdf(const char* objfile, real sdf_grid_size) {
    if (!bvh) {
        init_bvh(objfile);
    }

    type = CollisionMesh::Type::SDF;

    Eigen::AlignedBox<real, 3> domain;
    domain.setEmpty();
    for (auto const& x : mesh->vertices())
    {
        domain.extend(x);
    }
    domain.max() += 1.0e-3 * domain.diagonal().norm() * Eigen::Vector3r::Ones();
    domain.min() -= 1.0e-3 * domain.diagonal().norm() * Eigen::Vector3r::Ones();

    Eigen::Vector3i res = ((domain.max() - domain.min()) / sdf_grid_size).array().ceil().cast<int>();
    printf("Generating SDF of size (%d, %d, %d) for %s...\n", res[0], res[1], res[2], objfile);
    sdf = std::make_unique<Discregrid::CubicLagrangeDiscreteGrid>(
            domain, Eigen::Vector3r(sdf_grid_size, sdf_grid_size, sdf_grid_size));
    // auto cell_size = sdf_grid.cellSize();
    // fmt::print("Cell size = ({}, {}, {})\n", cell_size[0], cell_size[1], cell_size[2]);
    auto func = [this](Eigen::Vector3r const& xi) {return bvh->signedDistanceCached(xi); };
    sdf->addFunction(func, true);
}

real CollisionShape::mass(real density) const {
    switch (type) {
        case Type::Box: return density * scale.x * scale.y * scale.z;
        case Type::Sphere: return real(4.0 / 3.0) * density * glm::pi<real>() * scale.x * scale.y * scale.z;
        default: return real(0);
    }
}

tsmat3x3<real> CollisionShape::inertia(real density) const {
    switch (type) {
        case Type::Box: {
            const glm::tvec3<real>& s = scale;
            glm::vec3 I = mass(density) * glm::tvec3<real>(s.y*s.y + s.z*s.z, s.z*s.z + s.x*s.x, s.x*s.x + s.y*s.y) / real(12);
            return tsmat3x3<real>(I.x, I.y, I.z, 0, 0, 0);
        }
        case Type::Sphere: {
            real r = scale.x;
            glm::vec3 I = real(0.4) * mass(density) * glm::tvec3<real>(r*r);
            return tsmat3x3<real>(I.x, I.y, I.z, 0, 0, 0);
        }
        default: return tsmat3x3<real>(0, 0, 0, 0, 0, 0);
    }
}
