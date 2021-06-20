//
// Created by lasagnaphil on 2019-03-10.
//

#include "gengine/BVHData.h"

#include <iomanip>
#include <fstream>
#include <sstream>
#include <iostream>
#include <optional>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/quaternion.hpp>

std::optional<BVHData::ChannelType> stringToChannelType(const std::string& name) {
    switch (name[0]) {
        case 'X':
            if (name == "Xposition") return BVHData::ChannelType::Xpos;
            else if (name == "Xrotation") return BVHData::ChannelType::Xrot;
            else return {};
        case 'Y':
            if (name == "Yposition") return BVHData::ChannelType::Ypos;
            else if (name == "Yrotation") return BVHData::ChannelType::Yrot;
            else return {};
        case 'Z':
            if (name == "Zposition") return BVHData::ChannelType::Zpos;
            else if (name == "Zrotation") return BVHData::ChannelType::Zrot;
            else return {};
        default:
            return {};
    }
}

bool BVHData::loadFromFile(const char* filename, BVHData& data, float scale) {
    std::ifstream file(filename);

    std::string line;
    std::string keyword;

    enum class ParseState { Root, Joint, EndSite, Motion };
    ParseState state = ParseState::Root;
    int lineCount = 0;

    uint32_t curJointID = {};
    uint32_t childJointID = {};

    std::istringstream iss;
    auto newLine = [&]() -> std::istringstream& {
        std::getline(file, line);
        lineCount++;
        iss = std::istringstream(line);
        return iss;
    };

    // temporary storage for end sites
    std::vector<PoseTreeNode> endSites;

    // type of each channel
    std::vector<ChannelType> channelTypeData;

    int numBVHChannels = 0;

    bool finished = false;
    bool error = false;
    while (!error && !finished) {
        switch (state) {
            case ParseState::Root: {
                PoseTreeNode curJoint;
                curJointID = 0; // root joint id

                curJoint.parent = 0;

                newLine() >> keyword;

                newLine() >> keyword >> curJoint.name;

                newLine() >> keyword;
                if (keyword != "{") {
                    error = true;
                    break;
                }
                newLine() >> keyword >> curJoint.offset.x >> curJoint.offset.y >> curJoint.offset.z; // OFFSET
                curJoint.offset *= scale;
                int channels;
                std::string channelNames[6];
                newLine() >> keyword >> channels
                          >> channelNames[0] >> channelNames[1] >> channelNames[2]
                          >> channelNames[3] >> channelNames[4] >> channelNames[5];
                if (channels != 6) {
                    fprintf(stderr, "Only root node with 6 channels supported!\n");
                    error = true;
                    break;
                }
                for (int i = 0; i < 6; i++) {
                    if (auto channelType = stringToChannelType(channelNames[i])) {
                        channelTypeData.push_back(*channelType);
                    }
                    else {
                        fprintf(stderr, "Invalid channel type.\n");
                        error = true;
                        break;
                    }
                }
                numBVHChannels += 6; data.poseTree.numJoints++; data.poseTree.numNodes++; // CHANNELS
                childJointID = curJointID;

                newLine() >> keyword;
                if (keyword == "JOINT") {
                    state = ParseState::Joint;
                }
                else if (keyword == "End") {
                    state = ParseState::EndSite;
                }
                data.poseTree.allNodes.push_back(curJoint);
                break;
            }
            case ParseState::Joint: {
                PoseTreeNode childJoint;
                childJointID = data.poseTree.allNodes.size();
                PoseTreeNode& curJoint = data.poseTree.allNodes[curJointID];

                iss >> childJoint.name;
                newLine() >> keyword;
                if (keyword != "{") {
                    fprintf(stderr, "Error while parsing joint.\n");
                    error = true;
                    break;
                }
                newLine() >> keyword >> childJoint.offset.x >> childJoint.offset.y
                          >> childJoint.offset.z; // OFFSET
                childJoint.offset *= scale;

                int channels;
                std::vector<std::string> channelNames;
                newLine() >> keyword >> channels;
                if (channels > 0 && channels <= 6) {
                    channelNames.resize(channels);
                }
                else {
                    fprintf(stderr, "Invalid number of channels. (%d)\n", channels);
                    error = true;
                    break;
                }
                for (int c = 0; c < channels; c++) {
                    iss >> channelNames[c];
                }
                for (int c = 0; c < channels; c++) {
                    if (auto channelType = stringToChannelType(channelNames[c])) {
                        channelTypeData.push_back(*channelType);
                    }
                    else {
                        fprintf(stderr, "Invalid channel type %s.\n", channelNames[c].c_str());
                        error = true;
                        break;
                    }
                }
                numBVHChannels += channels; data.poseTree.numJoints++; data.poseTree.numNodes++; // CHANNELS
                curJoint.childJoints.push_back(childJointID);
                childJoint.parent = curJointID;
                curJointID = childJointID;

                newLine() >> keyword;
                if (keyword == "JOINT") {
                    state = ParseState::Joint;
                }
                else if (keyword == "End") {
                    state = ParseState::EndSite;
                }
                data.poseTree.allNodes.push_back(childJoint);
                break;
            }
            case ParseState::EndSite: {
                PoseTreeNode childJoint;
                curJointID = childJointID;
                uint32_t endSiteID = endSites.size();
                PoseTreeNode& curJoint = data.poseTree.allNodes[curJointID];

                childJoint.name = "End Site";
                newLine() >> keyword;
                if (keyword != "{") {
                    fprintf(stderr, "Error while parsing end site.\n");
                    error = true;
                    break;
                }
                newLine() >> keyword >> childJoint.offset.x >> childJoint.offset.y >> childJoint.offset.z;
                childJoint.offset *= scale;
                data.poseTree.numNodes++;
                // The most significant bit tells if the id is an end site or not
                curJoint.childJoints.push_back(endSiteID + (1 << 31));
                childJoint.parent = curJointID;
                curJointID = childJointID;

                newLine() >> keyword;
                if (keyword != "}") {
                    fprintf(stderr, "Error while parsing end site.\n");
                    error = true;
                    break;
                }
                newLine() >> keyword;
                while (keyword == "}") {
                    childJointID = curJointID;
                    curJointID = data.poseTree.allNodes[curJointID].parent;
                    newLine() >> keyword;
                }
                if (keyword == "JOINT") {
                    state = ParseState::Joint;
                }
                else if (keyword == "MOTION") {
                    state = ParseState::Motion;
                }

                endSites.push_back(childJoint);
                break;
            }
            case ParseState::Motion: {
                uint32_t numFrames;
                float frameTime;
                newLine() >> keyword >> numFrames;
                std::string _;
                newLine() >> keyword >> _ >> frameTime;

                data.clip = MotionClip::empty(data.poseTree.numJoints, numFrames, frameTime);
                float num;
                int offset = 0;
                float* dataPtr = data.clip.data.data();
                for (int f = 0; f < numFrames; f++) {
                    int rotCount = 0;
                    newLine();
                    glm::quat rot = glm::identity<glm::quat>();
                    for (int c = 0; c < numBVHChannels; c++) {
                        iss >> num;
                        if (!iss) {
                            fprintf(stderr, "Error while parsing Motion segment.\n");
                            error = true;
                            break;
                        }
                        if (c < 2) {
                            dataPtr[offset] = num * scale;
                            offset++;
                        }
                        else if (c == 2) {
                            dataPtr[offset] = num * scale;
                            offset += 2;
                        }
                        else {
                            switch (channelTypeData[c]) {
                                case ChannelType::Xrot:
                                    rot = glm::rotate(rot, glm::radians(num), {1, 0, 0});
                                    rotCount++;
                                    break;
                                case ChannelType::Yrot:
                                    rot = glm::rotate(rot, glm::radians(num), {0, 1, 0});
                                    rotCount++;
                                    break;
                                case ChannelType::Zrot:
                                    rot = glm::rotate(rot, glm::radians(num), {0, 0, 1});
                                    rotCount++;
                                    break;
                                default:
                                    // Ignore Xposition, Yposition, Zposition
                                    break;
                            }
                            if (rotCount == 3) {
                                dataPtr[offset] = rot.x;
                                dataPtr[offset+1] = rot.y;
                                dataPtr[offset+2] = rot.z;
                                dataPtr[offset+3] = rot.w;
                                offset += 4;
                                rot = glm::identity<glm::quat>();
                                rotCount = 0;
                            }
                        }
                    }
                    if (error) break;
                }
                finished = true;
                break;
            }
        }

    }

    if (error) {
        return false;
    }

    // Now we combine the joint data and end site data into one std::vector.
    // Body node indices need to be in 0...numJoints-1,
    // and end site indices need to be in numJoints...numNodes-1.
    // So need to make sure the highest bit of the end site ids are set back to zero,
    // and then add numJoints to set the index in the right location.
    data.poseTree.allNodes.reserve(data.poseTree.allNodes.size() + endSites.size());
    data.poseTree.allNodes.insert(data.poseTree.allNodes.end(), endSites.begin(), endSites.end());
    for (uint32_t i = 0; i < data.poseTree.numJoints; i++) {
        auto& joint = data.poseTree.allNodes[i];
        for (auto& childJointID : joint.childJoints) {
            if (childJointID & (1 << 31)) {
                childJointID = (childJointID & ~(1 << 31)) + data.poseTree.numJoints;
            }
        }
    }

    data.poseTree.constructNodeNameMapping();

    return true;
}

void BVHData::print() const {
    printRecursive(0 /* root joint id */, 0);
}

void BVHData::printRecursive(uint32_t jointID, int depth) const {
    const PoseTreeNode& joint = poseTree.allNodes[jointID];
    if (!joint.isEndSite()) {
        for (int i = 0; i < depth; i++) { std::cout << "    "; }
        std::cout << "Name: " << joint.name << std::endl;
        for (int i = 0; i < depth; i++) { std::cout << "    "; }
        std::cout << "Offset: " << joint.offset.x << " " << joint.offset.y << " " << joint.offset.z << std::endl;
        for (auto child : joint.childJoints) {
            printRecursive(child, depth + 1);
        }
    }
    else {
        for (int i = 0; i < depth; i++) { std::cout << "    "; }
        std::cout << "End Site Offset: " << joint.offset.x << " " << joint.offset.y << " " << joint.offset.z << std::endl;
    }
}

void BVHData::saveToFile(const std::string& filename, int eulerOrd) {
    using std::endl;
    std::ofstream ofs(filename);
    ofs << std::fixed;
    ofs.precision(6);

    saveToFileRecursive(0, ofs, 0, eulerOrd);
    ofs << "MOTION" << endl;
    ofs << "Frames: " << clip.numFrames << endl;
    ofs << "Frame Time: " << clip.frameTime << endl;
    for (int f = 0; f < clip.numFrames; f++) {
        glmx::pose_view pose = clip.getFrame(f);
        ofs << pose.v().x << " " << pose.v().y << " " << pose.v().z << " ";
        for (int i = 0; i < pose.size(); i++) {
            glm::vec3 e = glmx::quatToEuler(pose.q(i), eulerOrd);
            switch (eulerOrd) {
                case EulOrdXYZr:
                case EulOrdXYXr:
                case EulOrdXZYr:
                case EulOrdXZXr:
                case EulOrdYZXr:
                case EulOrdYZYr:
                case EulOrdYXZr:
                case EulOrdYXYr:
                case EulOrdZXYr:
                case EulOrdZXZr:
                case EulOrdZYXr:
                case EulOrdZYZr:
                    ofs << glm::degrees(e.x) << " " << glm::degrees(e.y) << " " << glm::degrees(e.z) << " "; break;
                case EulOrdXYZs:
                case EulOrdXYXs:
                case EulOrdXZYs:
                case EulOrdXZXs:
                case EulOrdYZXs:
                case EulOrdYZYs:
                case EulOrdYXZs:
                case EulOrdYXYs:
                case EulOrdZXYs:
                case EulOrdZXZs:
                case EulOrdZYXs:
                case EulOrdZYZs:
                    ofs << glm::degrees(e.z) << " " << glm::degrees(e.y) << " " << glm::degrees(e.x) << " "; break;
            }
        }
        ofs << endl;
    }
}

void BVHData::saveToFileRecursive(uint32_t jointIdx, std::ostream& ofs, int depth, int eulerOrd) {
    using std::endl;
    PoseTreeNode& node = poseTree[jointIdx];

    std::string tabs;
    for (int i = 0; i < depth; i++) {
        tabs += "\t";
    }
    if (jointIdx == 0) {
        ofs << tabs << "HIERARCHY" << endl;
        ofs << tabs << "ROOT " << node.name << endl;
        ofs << tabs << "{" << endl;
        ofs << tabs << "\tOFFSET " << node.offset.x << " " << node.offset.y << " " << node.offset.z << endl;
        ofs << tabs << "\tCHANNELS 6 Xposition Yposition Zposition";
        switch (eulerOrd) {
            case EulOrdXYZr: ofs << " Xrotation Yrotation Zrotation" << endl; break;
            case EulOrdXYXr: ofs << " Xrotation Yrotation Xrotation" << endl; break;
            case EulOrdXZYr: ofs << " Xrotation Zrotation Yrotation" << endl; break;
            case EulOrdXZXr: ofs << " Xrotation Zrotation Xrotation" << endl; break;
            case EulOrdYZXr: ofs << " Yrotation Zrotation Xrotation" << endl; break;
            case EulOrdYZYr: ofs << " Yrotation Zrotation Yrotation" << endl; break;
            case EulOrdYXZr: ofs << " Yrotation Xrotation Zrotation" << endl; break;
            case EulOrdYXYr: ofs << " Yrotation Xrotation Yrotation" << endl; break;
            case EulOrdZXYr: ofs << " Zrotation Xrotation Yrotation" << endl; break;
            case EulOrdZXZr: ofs << " Zrotation Xrotation Zrotation" << endl; break;
            case EulOrdZYXr: ofs << " Zrotation Yrotation Xrotation" << endl; break;
            case EulOrdZYZr: ofs << " Zrotation Yrotation Zrotation" << endl; break;
            case EulOrdXYZs: ofs << " Zrotation Yrotation Xrotation" << endl; break;
            case EulOrdXYXs: ofs << " Xrotation Yrotation Xrotation" << endl; break;
            case EulOrdXZYs: ofs << " Yrotation Zrotation Xrotation" << endl; break;
            case EulOrdXZXs: ofs << " Xrotation Zrotation Xrotation" << endl; break;
            case EulOrdYZXs: ofs << " Xrotation Zrotation Yrotation" << endl; break;
            case EulOrdYZYs: ofs << " Yrotation Zrotation Yrotation" << endl; break;
            case EulOrdYXZs: ofs << " Zrotation Xrotation Yrotation" << endl; break;
            case EulOrdYXYs: ofs << " Yrotation Xrotation Yrotation" << endl; break;
            case EulOrdZXYs: ofs << " Yrotation Xrotation Zrotation" << endl; break;
            case EulOrdZXZs: ofs << " Zrotation Xrotation Zrotation" << endl; break;
            case EulOrdZYXs: ofs << " Xrotation Yrotation Zrotation" << endl; break;
            case EulOrdZYZs: ofs << " Zrotation Yrotation Zrotation" << endl; break;
        }
    }
    else if (!node.isEndSite()) {
        ofs << tabs << "JOINT " << node.name << endl;
        ofs << tabs << "{" << endl;
        ofs << tabs << "\tOFFSET " << node.offset.x << " " << node.offset.y << " " << node.offset.z << endl;
        ofs << tabs << "\tCHANNELS 3";
        switch (eulerOrd) {
            case EulOrdXYZr: ofs << " Xrotation Yrotation Zrotation" << endl; break;
            case EulOrdXYXr: ofs << " Xrotation Yrotation Xrotation" << endl; break;
            case EulOrdXZYr: ofs << " Xrotation Zrotation Yrotation" << endl; break;
            case EulOrdXZXr: ofs << " Xrotation Zrotation Xrotation" << endl; break;
            case EulOrdYZXr: ofs << " Yrotation Zrotation Xrotation" << endl; break;
            case EulOrdYZYr: ofs << " Yrotation Zrotation Yrotation" << endl; break;
            case EulOrdYXZr: ofs << " Yrotation Xrotation Zrotation" << endl; break;
            case EulOrdYXYr: ofs << " Yrotation Xrotation Yrotation" << endl; break;
            case EulOrdZXYr: ofs << " Zrotation Xrotation Yrotation" << endl; break;
            case EulOrdZXZr: ofs << " Zrotation Xrotation Zrotation" << endl; break;
            case EulOrdZYXr: ofs << " Zrotation Yrotation Xrotation" << endl; break;
            case EulOrdZYZr: ofs << " Zrotation Yrotation Zrotation" << endl; break;
            case EulOrdXYZs: ofs << " Zrotation Yrotation Xrotation" << endl; break;
            case EulOrdXYXs: ofs << " Xrotation Yrotation Xrotation" << endl; break;
            case EulOrdXZYs: ofs << " Yrotation Zrotation Xrotation" << endl; break;
            case EulOrdXZXs: ofs << " Xrotation Zrotation Xrotation" << endl; break;
            case EulOrdYZXs: ofs << " Xrotation Zrotation Yrotation" << endl; break;
            case EulOrdYZYs: ofs << " Yrotation Zrotation Yrotation" << endl; break;
            case EulOrdYXZs: ofs << " Zrotation Xrotation Yrotation" << endl; break;
            case EulOrdYXYs: ofs << " Yrotation Xrotation Yrotation" << endl; break;
            case EulOrdZXYs: ofs << " Yrotation Xrotation Zrotation" << endl; break;
            case EulOrdZXZs: ofs << " Zrotation Xrotation Zrotation" << endl; break;
            case EulOrdZYXs: ofs << " Xrotation Yrotation Zrotation" << endl; break;
            case EulOrdZYZs: ofs << " Zrotation Yrotation Zrotation" << endl; break;
        }
    }
    else {
        ofs << tabs << "End Site" << endl;
        ofs << tabs << "{" << endl;
        ofs << tabs << "\tOFFSET " << node.offset.x << " " << node.offset.y << " " << node.offset.z << endl;
    }
    for (uint32_t childIdx : node.childJoints) {
        saveToFileRecursive(childIdx, ofs, depth + 1, eulerOrd);
    }
    ofs << tabs << "}" << endl;
}

void BVHData::switchZtoYup() {
    for (int f = 0; f < clip.numFrames; f++) {
        glmx::pose_view pose = clip.getFrame(f);
        pose.v() = glmx::Rx_quat<float>(-M_PI/2) * pose.v();
        for (int i = 0; i < pose.size(); i++) {
            pose.q(i) = glmx::Rx_quat<float>(-M_PI/2) * pose.q(i) * glmx::Rx_quat<float>(M_PI/2);
        }
    }
    for (auto& node : poseTree.allNodes) {
        node.offset = glmx::Rx(-M_PI/2) * node.offset;
    }
}

bool BVHData::removeJoint(uint32_t nodeIdx) {
    if (nodeIdx == (uint32_t)-1) {
        fprintf(stderr, "Tring to remove invalid joint!\n");
        return false;
    }

    auto& node = poseTree[nodeIdx];
    std::string nodeName = node.name;
    uint32_t parentIdx = poseTree[nodeIdx].parent;
    auto& parentNode = poseTree[parentIdx];

    uint32_t numChildren = poseTree[nodeIdx].childJoints.size();
    if (numChildren != 1) {
        fprintf(stderr, "Only joints with one child can be removed!\n");
        return false;
    }

    uint32_t childIdx = poseTree[nodeIdx].childJoints[0];
    poseTree[childIdx].parent = parentIdx;

    for (uint32_t& i : parentNode.childJoints) {
        if (i == nodeIdx) {
            i = childIdx;
            break;
        }
    }
    poseTree.numJoints--;
    poseTree.numNodes--;
    poseTree.allNodes.erase(poseTree.allNodes.begin() + nodeIdx);

    for (int f = 0; f < clip.numFrames; f++) {
        if (childIdx < poseTree.numJoints) {
            glmx::pose_view pose = clip.getFrame(f);
            pose.q(childIdx) = pose.q(nodeIdx) * pose.q(childIdx);
        }
    }

    clip.removeJoint(nodeIdx);

    for (auto& node : poseTree.allNodes) {
        if (node.parent > nodeIdx) {
            node.parent--;
        }
        for (uint32_t& i : node.childJoints) {
            if (i > nodeIdx) {
                i--;
            }
        }
    }

    poseTree.nodeNameMap.erase(nodeName);
    for (auto& [key, i] : poseTree.nodeNameMap) {
        if (i > nodeIdx) {
            i--;
        }
    }

    return true;
}

bool BVHData::removeJoint(const std::string& nodeName) {
    uint32_t nodeIdx = poseTree.findIdx(nodeName);
    if (nodeIdx != (uint32_t)-1) {
        return removeJoint(nodeIdx);
    }
    else {
        fprintf(stderr, "Trying to remove invalid joint %s!\n", nodeName.c_str());
        return false;
    }
}

bool BVHData::removeCMUPhantomJoints() {
    bool success = true;
    success &= removeJoint("LHipJoint");
    success &= removeJoint("RHipJoint");
    success &= removeJoint("LowerBack");
    success &= removeJoint("Neck");
    success &= removeJoint("LeftShoulder");
    success &= removeJoint("LeftFingerBase");
    success &= removeJoint("LThumb");
    success &= removeJoint("RightShoulder");
    success &= removeJoint("RightFingerBase");
    success &= removeJoint("RThumb");
    return success;
}

bool BVHData::checkValidity() {
    for (auto& node : poseTree.allNodes) {
        if (std::isnan(node.offset.x) || std::isnan(node.offset.y) || std::isnan(node.offset.z)) {
            return false;
        }
    }
    return true;
}

// DEPRECATED
glmx::pose BVHData::samplePose(float time) {
    glmx::pose pose = glmx::pose::empty(clip.getFrame(0).size());
    uint32_t u = time / clip.frameTime;
    if (u <= 0) {
        pose = glmx::pose(clip.getFrame(0));
    }
    else if (u >= clip.numFrames - 1) {
        pose = glmx::pose(clip.getFrame(clip.numFrames - 1));
    }
    else {
        float t = std::fmod(time, clip.frameTime);
        glmx::slerp(clip.getFrame(u), clip.getFrame(u + 1), t, pose.getView());
    }
    return pose;
}

