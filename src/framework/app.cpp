#include "framework/app.hpp"
#include "framework/debug.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>

using namespace frmwrk;


App::App(const std::string & appName, const Eigen::Vector2i & windowSize, const bool resizableWindow)
    : initialized_(false),
      appName_(appName),
      windowSize_(windowSize),
      resizableWindow_(resizableWindow)
{}

App::~App()
{
    if(initialized_)
    {
        ImGui_ImplOpenGL2_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(window_);
        glfwTerminate();
    }

}

void App::run()
{    
    if(!init() || !initApp())
    {
        Debug::logError("An error occurred while initializing the app, cannot enter the main loop");
        return;
    }

    double oldTime, currTime;
    currTime = glfwGetTime();

    isRunning_ = true;
    while(!glfwWindowShouldClose(window_) && isRunning_)
    {
        oldTime = currTime;
        currTime = glfwGetTime();

        input_.clean();
        glfwPollEvents();

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if(!mainLoop(currTime - oldTime))
        {
            break;
        }

        ImGui::Render();
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
    }
}

bool App::init()
{
    Debug::logInfo("Initializing app...");

    glfwSetErrorCallback(Debug::logGlfwError);
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_RESIZABLE, resizableWindow_? GL_TRUE : GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 8);

    window_ = glfwCreateWindow(windowSize_(0), windowSize_(1), appName_.c_str(), NULL, NULL);
    if (!window_)
    {
        glfwTerminate();
        return false;
    }
    glfwSetWindowUserPointer(window_, this);
    glfwSetWindowSizeCallback(window_, App::windowSizeCallback);
    App::windowSizeCallback(window_, windowSize_(0), windowSize_(1));
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    input_.init(window_);

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        Debug::logError("GLEW ERROR: %s", glewGetErrorString(err));
        glfwDestroyWindow(window_);
        glfwTerminate();
        return false;
    }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    Debug::logSuccess("Using OpenGL %s", glGetString(GL_VERSION));

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL2_Init();

    initialized_ = true;

    return true;
}

void App::stop()
{
    isRunning_ = false;
}

bool App::initApp()
{
    return true;
}

bool App::mainLoop(double delta)
{
    return true;
}

// Window getters - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

Eigen::Vector2i App::getWindowSize() const
{
    return windowSize_;
}

Eigen::Vector2i App::getFramebufferSize() const
{
    return framebufferSize_;
}

Eigen::Vector2f App::getWindowContentScale() const
{
    return contentScale_;
}

// GLFW window event callbacks - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void App::windowSizeCallback(GLFWwindow* window, int width, int height)
{
    App* app = reinterpret_cast<App *>(glfwGetWindowUserPointer(window));
    int ix, iy;
    float fx, fy;
    
    app->windowSize_ = Eigen::Vector2i(width, height);

    glfwGetFramebufferSize(window, &ix, &iy);
    app->framebufferSize_ = Eigen::Vector2i{ix, iy};
    glViewport(0, 0, ix, iy);

    glfwGetWindowContentScale(window, &fx, &fy);
    app->contentScale_ = Eigen::Vector2f{fx, fy};
}