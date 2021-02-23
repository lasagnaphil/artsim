//
// Created by lasagnaphil on 2017-03-27.
//

#include "gengine/Shader.h"
#include "gengine/GLUtils.h"
#include "gengine/FlyCamera.h"
#include "gengine/PhongRenderer.h"
#include "gengine/PBRenderer.h"

#include <glm/gtc/type_ptr.hpp>

#include "shaders/line2d.vert.h"
#include "shaders/line2d.frag.h"
#include "shaders/line3d.vert.h"
#include "shaders/line3d.frag.h"
#include "shaders/point.vert.h"
#include "shaders/point.frag.h"
#include "shaders/phong_shadows.vert.h"
#include "shaders/phong_shadows.frag.h"
#include "shaders/depth.vert.h"
#include "shaders/depth.frag.h"
#include "shaders/depth_debug.vert.h"
#include "shaders/depth_debug.frag.h"
#include "shaders/pbr.vert.h"
#include "shaders/pbr.frag.h"

GLuint compileShader(GLenum type, const GLchar *source) {
    GLuint shader = glCreateShader(type);

    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

    if (status == GL_TRUE) {
        // std::cout << "Shader (type " << type << ") is compiled successfully!" << std::endl;
    } else {
        std::cout << "Shader (type " << type << ") compile failed!" << std::endl;
        std::cout << "Compile log: " << std::endl;
        char compileInfo[512];
        glGetShaderInfoLog(shader, 512, NULL, compileInfo);
        std::cout << compileInfo << std::endl;
    }

    return shader;
}

std::string loadFile(const std::string& path) {
    std::string contents;
    std::ifstream fs;

    fs.open(path, std::ios::in);
    if (!fs) {
        std::cerr << "Error while loading file " << path << ":" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::stringstream buf;
    buf << fs.rdbuf();
    fs.close();
    return buf.str();
}

void Shader::compileFromFile(const char *vertexPath, const char *fragmentPath, const char *geometryPath) {
    bool hasGeom = geometryPath != nullptr;

    std::string vertexCode = loadFile(vertexPath);
    std::string fragmentCode = loadFile(fragmentPath);

    std::string geometryCode;
    if (hasGeom) {
        geometryCode = loadFile(geometryPath);
    }

    const GLchar* vertexCodePtr = vertexCode.data();
    const GLchar* fragmentCodePtr = fragmentCode.data();
    const GLchar* geometryCodePtr = hasGeom? geometryCode.data() : nullptr;

    compileFromString(vertexCodePtr, fragmentCodePtr, geometryCodePtr);
}

void Shader::compileFromString(const char* vertexSrc, const char* fragmentSrc, const char* geomSrc) {

    program = glCreateProgram();

    GLuint vertexShaderPtr = compileShader(GL_VERTEX_SHADER, vertexSrc);
    GLuint fragmentShaderPtr = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
    glAttachShader(program, vertexShaderPtr);
    glAttachShader(program, fragmentShaderPtr);

    GLuint geometryShaderPtr;
    if (geomSrc) {
        geometryShaderPtr = compileShader(GL_GEOMETRY_SHADER, geomSrc);
        glAttachShader(program, geometryShaderPtr);
    }

    glLinkProgram(program);
    GLint success;
    GLchar infoLog[512];
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cout << "Error: shader program linking failed" << infoLog << std::endl;
    }

    glDeleteShader(vertexShaderPtr);
    glDeleteShader(fragmentShaderPtr);
    if (geomSrc) glDeleteShader(geometryShaderPtr);
}

void Shader::use() const {
    glUseProgram(this->program);
}

void Shader::setBool(const char* name, bool value) const {
    glUniform1i(glGetUniformLocation(program, name), (int)value);
}

void Shader::setBool(GLint uniID, bool value) const {
    glUniform1i(uniID, (int)value);
}

void Shader::setInt(const char* name, int value) const {
    glUniform1i(glGetUniformLocation(program, name), value);
}

void Shader::setInt(GLint uniID, int value) const {
    glUniform1i(uniID, value);
}

void Shader::setFloat(const char* name, float value) const {
    glUniform1f(glGetUniformLocation(program, name), value);
}

void Shader::setFloat(GLint uniID, float value) const {
    glUniform1f(uniID, value);
}

void Shader::setMat4(const char* name, const glm::mat4& value) const {
    glUniformMatrix4fv(glGetUniformLocation(program, name), 1, GL_FALSE, glm::value_ptr(value));
}

void Shader::setMat4(GLint uniID, const glm::mat4& value) const {
    glUniformMatrix4fv(uniID, 1, GL_FALSE, glm::value_ptr(value));
}

void Shader::setVec3(const char* name, const glm::vec3& value) const {
    glUniform3fv(glGetUniformLocation(program, name), 1, glm::value_ptr(value));
}

void Shader::setVec3(GLint uniID, const glm::vec3& value) const {
    glUniform3fv(uniID, 1, glm::value_ptr(value));
}

void Shader::setVec4(const char* name, const glm::vec4& value) const {
    glUniform4fv(glGetUniformLocation(program, name), 1, glm::value_ptr(value));
}

void Shader::setVec4(GLint uniID, const glm::vec4& value) const {
    glUniform4fv(uniID, 1, glm::value_ptr(value));
}

void Shader::setPhongMaterial(const PhongMaterial &material) const {
    setVec4("material.ambient", material.ambient);
    setFloat("material.shininess", material.shininess);
    setBool("material.useTexDiffuse", (bool)material.texDiffuse);
    setBool("material.useTexSpecular", (bool)material.texSpecular);
    if (material.texDiffuse) {
        glActiveTexture(GL_TEXTURE0);
        material.texDiffuse->bind();
    }
    else {
        setVec4("material.diffuse", material.diffuse);
    }
    if (material.texSpecular) {
        glActiveTexture(GL_TEXTURE1);
        material.texSpecular->bind();
    }
    else {
        setVec4("material.specular", material.specular);
    }
    setInt("material.texDiffuse", 0);
    setInt("material.texSpecular", 1);
}

void Shader::setPBRMaterial(const PBRMaterial &material) const {
    glActiveTexture(GL_TEXTURE0);
    material.texAlbedo->bind();

    glActiveTexture(GL_TEXTURE1);
    material.texMetallic->bind();

    glActiveTexture(GL_TEXTURE2);
    material.texRoughness->bind();

    glActiveTexture(GL_TEXTURE3);
    material.texAO->bind();

    setInt("mat.texAlbedo", 0);
    setInt("mat.texMetallic", 1);
    setInt("mat.texRoughness", 2);
    setInt("mat.texAO", 3);
    setFloat("mat.alpha", material.alpha);
}

void Shader::setCamera(const Camera* camera) const {
    setMat4("proj", camera->getPerspectiveMatrix());
    setMat4("view", camera->getViewMatrix());
    setVec3("viewPos", camera->getPosition());
}

GLint Shader::getUniformLocation(const char* name) {
    return glGetUniformLocation(program, name);
}


Ref<Shader> Shaders::line2D = {};
Ref<Shader> Shaders::line3D = {};
Ref<Shader> Shaders::point = {};
Ref<Shader> Shaders::phong = {};
Ref<Shader> Shaders::depth = {};
Ref<Shader> Shaders::depthDebug = {};
Ref<Shader> Shaders::pbr = {};

void Shaders::init() {
    Shaders::line2D = Resources::make<Shader>("line2d");
    Shaders::line2D->compileFromString(line2d_vert_shader, line2d_frag_shader);

    Shaders::line3D = Resources::make<Shader>("line3d");
    Shaders::line3D->compileFromString(line3d_vert_shader, line3d_frag_shader);

    Shaders::point = Resources::make<Shader>("point");
    Shaders::point->compileFromString(point_vert_shader, point_frag_shader);

    Shaders::phong = Resources::make<Shader>("phong");
    Shaders::phong->compileFromString(phong_shadows_vert_shader, phong_shadows_frag_shader);

    Shaders::depth = Resources::make<Shader>("depth");
    Shaders::depth->compileFromString(depth_vert_shader, depth_frag_shader);

    Shaders::depthDebug = Resources::make<Shader>("depth_debug");
    Shaders::depthDebug->compileFromString(depth_debug_vert_shader, depth_debug_frag_shader);

    Shaders::pbr = Resources::make<Shader>("pbr");
    Shaders::pbr->compileFromString(pbr_vert_shader, pbr_frag_shader);
}
