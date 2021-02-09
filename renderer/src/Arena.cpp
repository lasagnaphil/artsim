//
// Created by lasagnaphil on 19. 3. 7.
//

#include "gengine/Arena.h"
#include "gengine/Transform.h"
#include "gengine/Shader.h"
#include "gengine/Texture.h"
#include "gengine/Image.h"
#include "gengine/PhongRenderer.h"
#include "gengine/PBRenderer.h"
#include "gengine/Mesh.h"
#include "gengine/LineMesh.h"

#ifdef USE_SHARED_PTR
#else
#define LIST_OF_VARIABLES \
    GLOBAL_TYPE(Transform, 1, 16) \
    GLOBAL_TYPE(Shader, 2, 16) \
    GLOBAL_TYPE(Texture, 3, 16) \
    GLOBAL_TYPE(Image, 4, 16) \
    GLOBAL_TYPE(Mesh, 5, 16) \
    GLOBAL_TYPE(LineMesh, 6, 16) \
    GLOBAL_TYPE(PhongMaterial, 7, 16) \
    GLOBAL_TYPE(LineMaterial, 8, 16) \
    GLOBAL_TYPE(PBRMaterial, 9, 16)

#define GLOBAL_TYPE(__Type, __id, __capacity) \
template <> uint16_t TypeRegistry::getID<__Type>() { return __id; } \
template <> __Type *Ref<__Type>::operator->() const { return Resources::get(*this); } \
template <> __Type &Ref<__Type>::operator*() const { return *Resources::get(*this); } \
template <> __Type *Ref<__Type>::get() { return Resources::get(*this); } \
template <> __Type *Ref<__Type>::tryGet() { return Resources::tryGet(*this); } \
template <> void Ref<__Type>::release() { Resources::release(*this); } \
static Arena<__Type> __Type##Storage; \
template <> Arena<__Type>& Resources::getStorage() { return __Type##Storage; }

LIST_OF_VARIABLES
#undef GLOBAL_TYPE

#define GLOBAL_TYPE(__Type, __id, __capacity) \
__Type##Storage = GenAllocator<__Type>(__capacity);

#undef GLOBAL_TYPE

#endif
