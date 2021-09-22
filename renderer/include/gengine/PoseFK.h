//
// Created by lasagnaphil on 20. 2. 12..
//

#ifndef DEEPMIMIC_POSEFK_H
#define DEEPMIMIC_POSEFK_H

#include <artsim/math/pose.h>
#include <artsim/math/se3.h>
#include <artsim/anim/pose_tree.h>

namespace artsim {

glmx::quat_transform calcFK(const PoseTree& poseTree, glmx::const_pose_view pose, uint32_t mIdx);

std::vector<glmx::quat_transform> calcFK(const PoseTree& poseTree, glmx::const_pose_view pose);

}
#endif //DEEPMIMIC_POSEFK_H
