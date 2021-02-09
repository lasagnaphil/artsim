//
// Created by lasagnaphil on 2/8/18.
//

#include "gengine/InputManager.h"

#include <imgui.h>
#include <imgui_internal.h>

InputManager InputManager::inst = {};

void InputManager::update() {
    // update keyboard state
    memcpy(prevKeys, currKeys, SDL_NUM_SCANCODES * sizeof(Uint8));
    memcpy(currKeys, SDL_GetKeyboardState(NULL), SDL_NUM_SCANCODES * sizeof(Uint8));

    // update mouse state
    prevMouse = currMouse;
    prevMousePos = currMousePos;
    currMouse = SDL_GetMouseState(&currMousePos.x, &currMousePos.y);
}

bool InputManager::isKeyPressed(SDL_Scancode key){
    auto& io = ImGui::GetIO();
    return currKeys[key] && !io.WantCaptureKeyboard;
}

bool InputManager::isKeyEntered(SDL_Scancode key) {
    auto& io = ImGui::GetIO();
    return currKeys[key] && !prevKeys[key] && !io.WantCaptureKeyboard;
}

bool InputManager::isKeyExited(SDL_Scancode key) {
    auto& io = ImGui::GetIO();
    return !currKeys[key] && prevKeys[key] && !io.WantCaptureKeyboard;
}

bool InputManager::isMousePressed(Uint8 button) {
    auto& io = ImGui::GetIO();
    return (currMouse & SDL_BUTTON(button)) != 0 && !io.WantCaptureMouse;
}

bool InputManager::isMouseEntered(Uint8 button) {
    auto& io = ImGui::GetIO();
    return (currMouse & SDL_BUTTON(button)) != 0 &&
           (prevMouse & SDL_BUTTON(button)) == 0 &&
           !io.WantCaptureMouse;
}

bool InputManager::isMouseExited(Uint8 button) {
    auto& io = ImGui::GetIO();
    return (currMouse & SDL_BUTTON(button)) == 0 &&
           (prevMouse & SDL_BUTTON(button)) != 0 &&
           !io.WantCaptureMouse;
}

bool InputManager::isMouseOnImGui() {
    ImGuiContext* g = ImGui::GetCurrentContext();
    if (g->HoveredWindow != NULL) return true;
    return false;
}

glm::ivec2 InputManager::getMousePos() {
    return currMousePos;
}

glm::ivec2 InputManager::getRelMousePos() {
    glm::ivec2 pos;
    SDL_GetRelativeMouseState(&pos.x, &pos.y);
    return pos;
}
