#ifndef FRAMEWORK_APP_H_
#define FRAMEWORK_APP_H_

#include <string>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

#include "framework/input.hpp"


namespace frmwrk
{
    class App
    {
        friend Input;

    public:
        App(const std::string & appName, const Eigen::Vector2i & windowSize, const bool resizableWindow);
        virtual ~App();

        void run();
        void stop();

        // Window getters

        Eigen::Vector2i getWindowSize() const;
        Eigen::Vector2i getFramebufferSize() const;
        Eigen::Vector2f getWindowContentScale() const;

    protected:
        Input input_;

    private:
        bool initialized_;
        bool isRunning_;

        std::string appName_;
        Eigen::Vector2i windowSize_;
        Eigen::Vector2i framebufferSize_;
        Eigen::Vector2f contentScale_;
        bool resizableWindow_;
        GLFWwindow* window_;

        bool init();

        virtual bool initApp();
        virtual bool mainLoop(double delta);

        static void windowSizeCallback(GLFWwindow* window, int width, int height);
    };

}


#endif