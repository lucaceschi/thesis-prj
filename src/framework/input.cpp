#include "framework/app.hpp"
#include "framework/input.hpp"
#include "framework/debug.hpp"

using namespace frmwrk;


Input::Input()
    : cursorFlag_(GLFW_CURSOR_NORMAL)
{}

void Input::init(GLFWwindow * window)
{
    window_ = window;

    glfwSetCursorPosCallback(window, Input::cursorPositionCallback);
    glfwSetMouseButtonCallback(window, Input::mouseButtonCallback);
    glfwSetScrollCallback(window, Input::mouseScrollCallback);
    glfwSetKeyCallback(window, Input::keyCallback);
}

void Input::clean()
{
    pressedKeys_.clear();
    releasedKeys_.clear();
    mouseScrollOffset_ = 0;
    pressedMouseButtons_.clear();
    releasedMouseButtons_.clear();
}

// GLFW input event callbacks - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void Input::cursorPositionCallback(GLFWwindow * window, double xpos, double ypos)
{
    App* app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
    Input* input = &(app->input_);

    input->cursorPos_(0) = xpos;
    input->cursorPos_(1) = ypos;
}

void Input::mouseButtonCallback(GLFWwindow * window, int button, int action, int mods)
{
    App* app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
    Input* input = &(app->input_);

    if(action == GLFW_PRESS)
    {
        input->pressedMouseButtons_[button] = true;
        input->heldMouseButtons_[button] = true;
    }
    else
    {
        input->releasedMouseButtons_[button] = true;
        input->heldMouseButtons_[button] = false;
    }
}

void Input::mouseScrollCallback(GLFWwindow * window, double xoffset, double yoffset)
{
    App* app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
    Input* input = &(app->input_);

    input->mouseScrollOffset_ = yoffset;
}

void Input::keyCallback(GLFWwindow * window, int key, int scancode, int action, int mods)
{    
    App* app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
    Input* input = &(app->input_);

    if(action == GLFW_PRESS)
    {
        input->pressedKeys_[key] = true;
        input->heldKeys_[key] = true;
    }
    else if(action == GLFW_RELEASE)
    {
        input->releasedKeys_[key] = true;
        input->heldKeys_[key] = false;
    }
}

// Mouse getters - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Eigen::Vector2d Input::getCursorPos() const
{
    return cursorPos_;
}

int Input::getMouseScrollOffset()
{
    return mouseScrollOffset_;
}

bool Input::isMouseButtonPressed(int button)
{
    return pressedMouseButtons_[button];
}

bool Input::isMouseButtonReleased(int button)
{
    return releasedMouseButtons_[button];
}

bool Input::isMouseButtonHeld(int button)
{
    return heldMouseButtons_[button];
}

// Mouse setters - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void Input::disableCursor(bool disable)
{
    if(disable)
    {
        if(cursorFlag_ == GLFW_CURSOR_DISABLED) return;

        cursorFlag_ = GLFW_CURSOR_DISABLED;
    }
    else
    {
        cursorFlag_ = GLFW_CURSOR_NORMAL;
    }

    glfwSetInputMode(window_, GLFW_CURSOR, cursorFlag_);
}

void Input::hideCursor(bool hide)
{
    if(hide)
    {
        if(cursorFlag_ == GLFW_CURSOR_HIDDEN || cursorFlag_ == GLFW_CURSOR_DISABLED) return;

        cursorFlag_ = GLFW_CURSOR_HIDDEN;
    }
    else
    {
        if(cursorFlag_ == GLFW_CURSOR_DISABLED || cursorFlag_ == GLFW_CURSOR_NORMAL) return;

        cursorFlag_ = GLFW_CURSOR_NORMAL;
    }

    glfwSetInputMode(window_, GLFW_CURSOR, cursorFlag_);
}

// Keyboard getters - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

bool Input::isKeyPressed(int key)
{
    return pressedKeys_[key];
}

bool Input::isKeyReleased(int key)
{
    return releasedKeys_[key];
}

bool Input::isKeyHeld(int key)
{
    return heldKeys_[key];
}