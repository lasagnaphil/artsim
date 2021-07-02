//
// Created by lasagnaphil on 20. 2. 12..
//

#include "gengine/PoseFK.h"

glmx::quat_transform calcFK(const PoseTree &poseTree, glmx::const_pose_view pose, uint32_t mIdx) {
    uint32_t i = mIdx;
    if (poseTree[i].isEndSite()) {
        i = poseTree[i].parent;
    }

    glmx::quat_transform t = glmx::quat_transform(glmx::IDENTITY);

    while (true) {
        auto& node = poseTree[i];
        if (poseTree[i].isEndSite()) {
            t = glmx::quat_transform(node.offset) * t;
        }
        else {
            t = glmx::quat_transform(node.offset, pose.q(i)) * t;
        }
        if (i == 0) {
            t = glmx::quat_transform(pose.v()) * t;
            break;
        }
        else {
            i = node.parent;
        }
    }
    return t;
}

std::vector<glmx::quat_transform> calcFK(const PoseTree& poseTree, glmx::const_pose_view pose) {
    std::vector<glmx::quat_transform> transforms(poseTree.numNodes);
    std::stack<std::tuple<uint32_t, uint32_t>> recursionStack;

    transforms[0] = glmx::quat_transform(glmx::IDENTITY);
    recursionStack.push({0, 0});

    while (!recursionStack.empty()) {
        auto[idx, parentIdx] = recursionStack.top();
        recursionStack.pop();

        auto& node = poseTree[idx];
        if (poseTree[idx].isEndSite()) {
            transforms[idx] = transforms[parentIdx] * glmx::quat_transform(node.offset);
        } else {
            transforms[idx] = transforms[parentIdx] * glmx::quat_transform(node.offset, pose.q(idx));
        }

        for (uint32_t childIdx : node.childJoints) {
            recursionStack.push({childIdx, idx});
        }
    }

    return transforms;
}


