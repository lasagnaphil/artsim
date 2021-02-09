//
// Created by lasagnaphil on 2/8/18.
//

#ifndef GENGINE_INPUTMANAGER_H
#define GENGINE_INPUTMANAGER_H

#include <glm/ext.hpp>
#include <SDL2/SDL_events.h>

struct InputManager {
    Uint8 currKeys[SDL_NUM_SCANCODES] = {};
    Uint8 prevKeys[SDL_NUM_SCANCODES] = {};

    Uint32 currMouse = {};
    Uint32 prevMouse = {};
    glm::ivec2 currMousePos = {};
    glm::ivec2 prevMousePos = {};

    static InputManager inst;
    static InputManager* get() {
        return &InputManager::inst;
    }

    void update();

    bool isKeyPressed(SDL_Scancode key);
    bool isKeyEntered(SDL_Scancode key);
    bool isKeyExited(SDL_Scancode key);

    bool isMousePressed(Uint8 button);
    bool isMouseEntered(Uint8 button);
    bool isMouseExited(Uint8 button);

    bool isMouseOnImGui();

    glm::ivec2 getMousePos();
    glm::ivec2 getRelMousePos();
};


#endif //GENGINE_INPUTMANAGER_H
