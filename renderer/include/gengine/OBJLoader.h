//
// Created by lasagnaphil on 21. 8. 31..
//

#ifndef EOS_SCAN_TO_HUMAN_OBJLOADER_H
#define EOS_SCAN_TO_HUMAN_OBJLOADER_H

#include <gengine/Mesh.h>
#include <gengine/PBRenderer.h>

class OBJLoader {
public:
    struct Object {
        Ref<Mesh> mesh;
        Ref<PBRMaterial> mat;
    };
    static Object loadPBRSingle(const char* filename);
};

#endif //EOS_SCAN_TO_HUMAN_OBJLOADER_H
