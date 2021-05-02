#include "gengine/InputManager.h"
#include "gengine/App.h"
#include "gengine/PBRenderer.h"
#include "gengine/FlyCamera.h"

class MyApp : public App {
public:
    MyApp() : App(AppSettings::defaultPBR()) {}

    void loadResources() override {
        FlyCamera* camera = dynamic_cast<FlyCamera*>(this->camera.get());
        Ref<Transform> cameraTransform = camera->transform;
        cameraTransform->move({0.0f, 17.5f, 0.0f});

        pbRenderer.dirLightProjVolume = {
                {-20.f, -20.f, 0.f}, {20.f, 20.f, 1000.f}
        };
        pbRenderer.shadowFramebufferSize = {2048, 2048};

        pbRenderer.dirLight.enabled = true;
        pbRenderer.dirLight.direction = glm::normalize(glm::vec3 {2.0f, -3.0f, -2.0f});
        pbRenderer.dirLight.color = {0.5f, 0.5f, 0.5f};

        pbRenderer.pointLights[0].enabled = true;
        pbRenderer.pointLights[0].position = {-10.0f, 10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[0].color = {100.f, 100.f, 100.f};

        pbRenderer.pointLights[1].enabled = true;
        pbRenderer.pointLights[1].position = {10.0f, 10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[1].color = {100.f, 100.f, 100.f};

        pbRenderer.pointLights[2].enabled = true;
        pbRenderer.pointLights[2].position = {-10.0f, -10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[2].color = {100.f, 100.f, 100.f};

        pbRenderer.pointLights[3].enabled = true;
        pbRenderer.pointLights[3].position = {10.0f, -10.0f + 17.5f, 10.0f};
        pbRenderer.pointLights[3].color = {100.f, 100.f, 100.f};

        ground_mat = PBRMaterial::quick(
                "resources/textures/mossy-ground1-albedo.png",
                "resources/textures/mossy-ground1-metal.png",
                "resources/textures/mossy-ground1-roughness.png",
                "resources/textures/mossy-ground1-ao.png");

        ground_mesh = Mesh::makePlane(1000.0f, 100.0f);

        setDemo1();

        sphereTransforms.resize(N * N);

        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                int id = N*i + j;
                sphereTransforms[id] = Resources::make<Transform>();
                Transform::addChildToParent(sphereTransforms[id], rootTransform);
                sphereTransforms[id]->setPosition({2.5f * (i - N/2), 2.5f * (j + N/2), 0.0f});
                sphereTransforms[id]->setRotationEuler({M_PI/2, 0.0f, 0.0f});
                sphereTransforms[id]->setScale({1.0f, 1.0f, 1.0f});
            }
        }

        sphereMesh = Mesh::makeSphere();
    }

    void setDemo1() {
        sphereMats.resize(N * N);
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                int id = N*i + j;
                sphereMats[id] = Resources::make<PBRMaterial>();
                sphereMats[id]->texAlbedo = Texture::fromSingleColor({0.5f, 0.0f, 0.0f});
                sphereMats[id]->texAO = Texture::fromSingleColor({1.0f, 0.0f, 0.0f});
                sphereMats[id]->texMetallic =
                        Texture::fromSingleColor({(float)j / (float)N, 0.0f, 0.0f});
                sphereMats[id]->texRoughness =
                        Texture::fromSingleColor({glm::clamp((float)i / (float)N, 0.05f, 1.0f), 0.0f, 0.0f});
                // sphereMats[id]->transparent = true;
                // sphereMats[id]->alpha = 0.8f;
            }
        }
    }

    void setDemo2() {

        auto sphereMat = PBRMaterial::quick(
                "resources/textures/rustediron2_basecolor.png",
                "resources/textures/rustediron2_metallic.png",
                "resources/textures/rustediron2_roughness.png",
                "resources/textures/rustediron2_ao.png");

        sphereMats.resize(N * N);
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                int id = N*i + j;
                sphereMats[id] = sphereMat;
            }
        }
    }

    void processInput(SDL_Event &event) override {
    }

    void update(float dt) override {
        auto inputMgr = InputManager::get();
        if (inputMgr->isKeyEntered(SDL_SCANCODE_1)) {
            setDemo1();
        }
        if (inputMgr->isKeyEntered(SDL_SCANCODE_2)) {
            setDemo2();
        }
    }

    void render() override {
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pbRenderer.queueRender({ground_mesh, ground_mat, rootTransform->getWorldTransform()});

        for (int i = 0; i < sphereTransforms.size(); i++) {
            pbRenderer.queueRender({sphereMesh, sphereMats[i], sphereTransforms[i]->getWorldTransform()});
        }
        pbRenderer.render();

        for (auto& light : pbRenderer.pointLights) {
            if (light.enabled) {
                imRenderer.drawPoint(light.position, colors::Yellow, 4.0f, true);
            }
        }
        imRenderer.render();

        pbRenderer.renderImGui();
    }

    void release() override {
    }

private:
    const int N = 7;

    Ref<PBRMaterial> ground_mat;
    std::vector<Ref<PBRMaterial>> sphereMats;
    Ref<Mesh> ground_mesh;
    Ref<Mesh> sphereMesh;
    std::vector<Ref<Transform>> sphereTransforms;
};

int main(int argc, char** argv) {
    MyApp app;
    app.load();
    app.startMainLoop();
    app.release();

    return 0;
}
