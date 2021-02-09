//
// Created by lasagnaphil on 2017-03-27.
//

#pragma once

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <glad/glad.h>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "gengine/Arena.h"

struct PhongMaterial;
struct PBRMaterial;
struct Camera;

class Shader {
public:

    Shader(const char* name = "") : name(name) {};

    void compileFromFile(const char* vertexPath, const char* fragmentPath, const char* geometryPath = nullptr);
    void compileFromString(const char* vertexSrc, const char* fragmentSrc, const char* geomSrc = nullptr);
    void use() const;

    void setBool(const char* name, bool value) const;
    void setBool(GLint uniID, bool value) const;
    void setInt(const char* name, int value) const;
    void setInt(GLint uniID, int value) const;
    void setFloat(const char* name, float value) const;
    void setFloat(GLint uniID, float value) const;
    void setMat4(const char* name, const glm::mat4& value) const;
    void setMat4(GLint uniID, const glm::mat4& value) const;
    void setVec3(const char* name, const glm::vec3& value) const;
    void setVec3(GLint uniID, const glm::vec3& value) const;
    void setVec4(const char* name, const glm::vec4& value) const;
    void setVec4(GLint uniID, const glm::vec4& value) const;
    void setPhongMaterial(const PhongMaterial& material) const;
    void setPBRMaterial(const PBRMaterial& material) const;
    void setCamera(const Camera* camera) const;

    GLint getUniformLocation(const char* name);

    GLuint program;
    std::string name;
};

class Shaders {
public:
    static Ref<Shader> line2D;
    static Ref<Shader> line3D;
    static Ref<Shader> point;
    static Ref<Shader> phong;
    static Ref<Shader> depth;
    static Ref<Shader> depthDebug;
    static Ref<Shader> pbr;

    static void init();
};

