#include "framework/app.hpp"
#include "framework/debug.hpp"
#include "grid.hpp"
#include "constraints.hpp"

#include <stdio.h>
#include <vector>
#include <array>
#include <unordered_map>

#include <Eigen/Dense>
#include <imgui.h>
#include <wrap/gui/trackball.h>

#define MAX_GRID_COLS 1


struct Pick
{
    Pick(int nodeIdx, GLfloat depth)
        : pick(true), nodeIdx(nodeIdx), depth(depth) {};
    Pick() : pick(false) {};
    
    bool pick;
    int nodeIdx;
    GLfloat depth;
};


class TestApp : public frmwrk::App
{
public:
    TestApp()
        : App("sphere-segment intesection", Eigen::Vector2i{1200, 800}, true),
          bgColorRender_{0xff, 0xff, 0xff},
          bgColorPicking_{0x00, 0x00, 0xff},
          SColor_{
            {0xff, 0x00, 0x00},
            {0x00, 0x00, 0xff}
          },
          S_{
            Eigen::Vector3d(2, 0, 0),
            Eigen::Vector3d(1.5, 0, 0),
          },
          collPos_(3, 2),
          nColls_(0),
          sphCenter_(0, 0, 0),
          sphRad_(1)
    {}

private:

    const GLubyte bgColorRender_[3];
    const GLubyte bgColorPicking_[3];
    const GLubyte SColor_[2][3];

    vcg::Trackball trackball_;
    Pick pick_;

    Eigen::Vector3d S_[2];
    Eigen::Matrix3Xd collPos_;
    int nColls_;

    Eigen::Vector3d sphCenter_;
    double sphRad_;


    virtual bool initApp()
    {
        glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
        return true;
    }


    virtual bool mainLoop(double deltaTime)
    {        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        GLfloat framebufferRatio = (GLfloat)getFramebufferSize()(0)/getFramebufferSize()(1);
        gluPerspective(40, framebufferRatio, 0.1, 100);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0,0,-4,  0,0,0,   0,1,0);

        trackball_.GetView();
        trackball_.Apply();

        Eigen::Vector2d curPos = input_.getCursorPos();
        curPos(0) = curPos(0) * getWindowContentScale()(0);
        curPos(1) = getFramebufferSize()(1) - curPos(1) * getWindowContentScale()(1);

        if(!ImGui::GetIO().WantCaptureMouse)
        {
            if(input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT) || input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT))
            {
                GLubyte pickedColor[3];
                GLfloat pickedDepth;
                int pickedNodeIdx;

                drawPick();
                glReadPixels(curPos(0), curPos(1), 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &pickedColor);
                glReadPixels(curPos(0), curPos(1), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &pickedDepth);
                pickedNodeIdx = color2node(pickedColor[0], pickedColor[1], pickedColor[2]);

                if(pickedNodeIdx != -1 && input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
                    pick_ = Pick(pickedNodeIdx, pickedDepth);

                if(pickedNodeIdx == -1)
                    trackball_.MouseDown((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
            }

            if(input_.isMouseButtonHeld(GLFW_MOUSE_BUTTON_LEFT))
            {
                if(pick_.pick)
                {
                    GLdouble modelview[16];
                    GLdouble projection[16];
                    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
                    glGetDoublev(GL_PROJECTION_MATRIX, projection);
                    Eigen::Map<Eigen::Matrix4d> modelviewMat(modelview);
                    Eigen::Map<Eigen::Matrix4d> projectionMat(projection);

                    Eigen::Vector2d curPos = input_.getCursorPos();
                    Eigen::Vector4d curPosHom = {
                        ((curPos(0) / getFramebufferSize()(0)) - 0.5)*2,
                        -((curPos(1) / getFramebufferSize()(1)) - 0.5)*2,
                        pick_.depth * 2.0 - 1.0,
                        1
                    };

                    Eigen::Vector4d worldPos = (projectionMat * modelviewMat).inverse() * curPosHom;

                    S_[pick_.nodeIdx] = Eigen::Vector3d{
                        worldPos(0) / worldPos(3),
                        worldPos(1) / worldPos(3),
                        worldPos(2) / worldPos(3)
                    };
                }
                else
                    trackball_.MouseMove((int)curPos(0), (int)curPos(1));
            }

            if(input_.isMouseButtonReleased(GLFW_MOUSE_BUTTON_LEFT))
            {
                pick_ = Pick();
                trackball_.MouseUp((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
            }

            if(input_.getMouseScrollOffset() != 0)
                trackball_.MouseWheel(input_.getMouseScrollOffset());
        }

        if(!ImGui::GetIO().WantCaptureKeyboard)
        {
            if(input_.isKeyPressed(GLFW_KEY_LEFT_SHIFT))
                trackball_.ButtonDown(vcg::Trackball::KEY_SHIFT);
            if(input_.isKeyReleased(GLFW_KEY_LEFT_SHIFT))
                trackball_.ButtonUp(vcg::Trackball::KEY_SHIFT);
            if(input_.isKeyPressed(GLFW_KEY_LEFT_CONTROL))
                trackball_.ButtonDown(vcg::Trackball::KEY_CTRL);
            if (input_.isKeyReleased(GLFW_KEY_LEFT_CONTROL))
                trackball_.ButtonUp(vcg::Trackball::KEY_CTRL);
            if (input_.isKeyPressed(GLFW_KEY_LEFT_ALT))
                trackball_.ButtonDown(vcg::Trackball::KEY_ALT);
            if (input_.isKeyReleased(GLFW_KEY_LEFT_ALT))
                trackball_.ButtonUp(vcg::Trackball::KEY_ALT);
        }


        Eigen::Vector3d V = S_[1] - S_[0];
        double segLength = V.norm();
        V /= segLength;
        Eigen::Vector3d EO = sphCenter_ - S_[0];
        double v = EO.dot(V);
        double disc = std::pow(sphRad_, 2) - EO.squaredNorm() + std::pow(v, 2);
        if(disc < 0)
            nColls_ = 0;
        else
        {
            nColls_ = 1;
            double len = (v - std::sqrt(disc));
            collPos_.col(0) = S_[0] + len * V;
            
            if(len <= segLength)
                std::cout << len << std::endl;
        }

        drawRender();
        trackball_.DrawPostApply();

        return true;
    }


    void drawRender()
    {
        glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw sphere
/*         GLfloat lightPos[] = {2.0f, 2.0f, 0.0f, 0.0f};
        glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
        GLfloat lightAmbCol[] = {0.0f, 0.0f, 0.0f, 1.0f};
        glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbCol);
        GLfloat lightDiffCol[] = {1.0f, 1.0f, 1.0f, 1.0f};
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffCol);
        GLfloat lightSpecCol[] = {1.0f, 1.0f, 1.0f, 1.0f};
        glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecCol);

        GLfloat lightModelAmbCol[] = {0.2f, 0.2f, 0.2f, 1.0f};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lightModelAmbCol);

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

        GLfloat matSpecCol[] = {1.0f, 1.0f, 1.0f, 1.0f};
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, matSpecCol);
        GLfloat matEmissCol[] = {0.0f, 0.0f, 0.0f, 1.0f};
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, matEmissCol);

        glColor3f(1, 1, 1);

        glEnable(GL_RESCALE_NORMAL);

        glDisable(GL_LIGHTING);
 */

        // Draw edge len constraints
          
        glColor3ub(SColor_[1][0], SColor_[1][1], SColor_[1][2]);
        glLineWidth(3.0f);

        glBegin(GL_LINES);
        glVertex3d(S_[0][0], S_[0][1], S_[0][2]);
        glVertex3d(S_[1][0], S_[1][1], S_[1][2]);
        glEnd();

        // Draw grid nodes

        glPointSize(18.0f);

        glBegin(GL_POINTS);
        for(int n = 0; n < 2; n++)
        {
            if(pick_.pick && pick_.nodeIdx == n)
                glColor3ub(0xff, 0x00, 0x00);
            else
                glColor3ub(SColor_[n][0], SColor_[n][1], SColor_[n][2]);

            glVertex3d(S_[n](0), S_[n](1), S_[n](2));
        }
        glEnd();

        // Draw coll points

        glColor3f(0, 0, 0);
        glBegin(GL_POINTS);
        for(int n = 0; n < nColls_; n++)
            glVertex3d(collPos_.col(n)[0], collPos_.col(n)[1], collPos_.col(n)[2]);
        glEnd();
    }

    void drawPick()
    {
        glClearColor(bgColorPicking_[0]/255.0, bgColorPicking_[1]/255.0, bgColorPicking_[2]/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glPointSize(30.0f);

        glBegin(GL_POINTS);
        for(int n = 0; n < 2; n++)
        {
            GLubyte r, g, b;
            std::tie(r, g, b) = node2color(n);
            glColor3ub(r, g, b);

            const Eigen::Vector3d nodePos = S_[n];
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        glEnd();
    }

    std::tuple<GLubyte, GLubyte, GLubyte> node2color(int i)
    {
        int row = i / MAX_GRID_COLS;
        int col = i - (row * MAX_GRID_COLS);
        return {(GLubyte)row+1, (GLubyte)col+1, 0};
    }

    int color2node(GLubyte r, GLubyte g, GLubyte b)
    {
        if(b != 0)
            return -1;

        return (r-1) * MAX_GRID_COLS + (g-1);
    }
};

// Entry point - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int main(int argc, char const *argv[])
{
    TestApp app = TestApp();
    app.run();

    fflush(stdin);

    return 0;
}
