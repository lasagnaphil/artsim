//
// Created by lasagnaphil on 19. 11. 12..
//

#ifndef GENGINE_POSETREE_H
#define GENGINE_POSETREE_H

#include <algorithm>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>
#include <glm/vec3.hpp>

struct PoseTreeNode {
    std::string name;
    glm::vec3 offset;
    uint32_t parent;
    std::vector<uint32_t> childJoints;

    bool isRoot() const { return parent == 0; }
    bool isEndSite() const { return childJoints.size() == 0; }
};

struct PoseTree {
    std::vector<PoseTreeNode> allNodes;
    std::unordered_map<std::string, uint32_t> nodeNameMap;

    uint32_t numJoints = 0;
    uint32_t numNodes = 0;

    void constructNodeNameMapping() {
        for (int i = 0; i < allNodes.size(); i++) {
            nodeNameMap[allNodes[i].name] = i;
        }
    }

    const PoseTreeNode& operator[](uint32_t idx) const { return allNodes[idx]; }
    PoseTreeNode& operator[](uint32_t idx) { return allNodes[idx]; }

    const PoseTreeNode* operator[](const std::string& name) const {
        auto it = nodeNameMap.find(name);
        if (it != nodeNameMap.end()) {
            return &allNodes[it->second];
        }
        else {
            return nullptr;
        }
    }

    PoseTreeNode* operator[](const std::string& name) {
        return const_cast<PoseTreeNode*>(const_cast<const PoseTree*>(this)->operator[](name));
    }

    uint32_t findIdx(const std::string& name) const {
        auto it = nodeNameMap.find(name);
        if (it != nodeNameMap.end()) {
            return it->second;
        }
        else {
            return (uint32_t)-1;
        }
    }

    const PoseTreeNode& getRootNode() const {
        return allNodes[0];
    }

    PoseTreeNode& getRootNode() {
        return allNodes[0];
    }

    uint32_t getChildIdx(const PoseTreeNode& node, const std::string& name) const {
        for (uint32_t childIdx : node.childJoints) {
            if (allNodes[childIdx].name == name) {
                return childIdx;
            }
        }
        return (uint32_t)-1;
    }

    uint32_t getChildIdx(const uint32_t nodeIdx, const std::string& name) const {
        for (uint32_t childIdx : allNodes[nodeIdx].childJoints) {
            if (allNodes[childIdx].name == name) {
                return childIdx;
            }
        }
        return (uint32_t)-1;
    }

    const PoseTreeNode& getParentOfNode(const PoseTreeNode& node) const {
        return allNodes[node.parent];
    }

    PoseTreeNode& getParentOfNode(const PoseTreeNode& node) {
        return allNodes[node.parent];
    }

    template <typename Fun>
    void iterateChildrenOfNode(const PoseTreeNode& node, Fun fun) const {
        for (uint32_t childIdx : node.childJoints) {
            fun(allNodes[childIdx]);
        }
    }

    template <typename Fun>
    void iterateChildrenOfNode(PoseTreeNode& node, Fun fun) {
        for (uint32_t childIdx : node.childJoints) {
            fun(allNodes[childIdx]);
        }
    }

    template <class Fun>
    void iterateDFS(Fun fun) {
        std::stack<uint32_t> indices;
        indices.push(0);
        while (!indices.empty()) {
            uint32_t idx = indices.top();
            indices.pop();
            PoseTreeNode& node = allNodes[idx];
            fun(node, idx);
            for (int i = node.childJoints.size() - 1; i >= 0; i--) {
                indices.push(node.childJoints[i]);
            }
        }
    }
};

#endif //GENGINE_POSETREE_H
