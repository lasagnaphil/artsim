//
// Created by lasagnaphil on 2019-03-10.
//

#ifndef MOTION_EDITING_POSEDATA_H
#define MOTION_EDITING_POSEDATA_H

#include <artsim/math/pose.h>
#include <artsim/anim/pose_tree.h>
#include <artsim/anim/motion_clip.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/ext.hpp>
#include <span.hpp>
#include <stack>

namespace artsim {

struct BVHData {
    PoseTree pose_tree;
    MotionClip clip;

    // Motion data
    enum class ChannelType : uint8_t {
        Xrot, Yrot, Zrot, Xpos, Ypos, Zpos
    };

    static bool loadFromFile(const char* filename, BVHData& bvhData, float scale = 1.0f);

    void print() const;

    void saveToFile(const std::string& filename, int eulerOrd);

    void switchZToYUp();

    bool removeJoint(uint32_t nodeIdx);

    bool removeJoint(const std::string& nodeName);

    bool removeCMUPhantomJoints();

    bool checkValidity();

    // DEPRECATED
    glmx::pose samplePose(float time);

private:
    void printRecursive(uint32_t jointID, int depth) const;
    void saveToFileRecursive(uint32_t jointID, std::ostream& ofs, int depth, int eulerOrd);
};

}

#endif //MOTION_EDITING_POSEDATA_H
