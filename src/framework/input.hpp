#ifndef FRAMEWORK_INPUT_H_
#define FRAMEWORK_INPUT_H_

#include <map>

#include <GLFW/glfw3.h>
#include <Eigen/Dense>


namespace frmwrk
{
    class Input
    {
    public:
        Input();

        void init(GLFWwindow * window);

        void clean();

        // Mouse getters

        Eigen::Vector2d getCursorPos() const;
        int getMouseScrollOffset();
        bool isMouseButtonPressed(int button);
        bool isMouseButtonReleased(int button);
        bool isMouseButtonHeld(int button);

        // Mouse setters

        void disableCursor(bool disable);
        void hideCursor(bool hide);

        // Keyboard getters

        bool isKeyPressed(int key);
        bool isKeyReleased(int key);
        bool isKeyHeld(int key);

    private:
        GLFWwindow* window_;

        //Mouse

        Eigen::Vector2d cursorPos_;
        double mouseScrollOffset_;
        int cursorFlag_;
        std::map<int, bool> heldMouseButtons_;
        std::map<int, bool> pressedMouseButtons_;
        std::map<int, bool> releasedMouseButtons_;

        //Keyboard

        std::map<int, bool> heldKeys_;
        std::map<int, bool> pressedKeys_;
        std::map<int, bool> releasedKeys_;

        // GLFW input event callbacks

        static void cursorPositionCallback(GLFWwindow * window, double xpos, double ypos);
        static void mouseButtonCallback(GLFWwindow * window, int button, int action, int mods);
        static void mouseScrollCallback(GLFWwindow * window, double xoffset, double yoffset);
        static void keyCallback(GLFWwindow * window, int key, int scancode, int action, int mods);
    };
}


#endif