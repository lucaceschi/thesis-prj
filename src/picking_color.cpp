#include "framework/app.hpp"
#include "framework/debug.hpp"
#include "grid.hpp"

#include <stdio.h>

#include <Eigen/Dense>
#include <imgui.h>
#include <wrap/gui/trackball.h>

#define GRID_WIDTH  5
#define GRID_HEIGHT 5
#define GRID_EDGE_LEN 0.2
#define SIM_TOL 1e-8


class MainApp : public frmwrk::App
{
public:
    MainApp()
        : App("Grid", Eigen::Vector2i{640, 480}, true)
    {}

private:
    const GLubyte bgColorRender_[3] = {0xfa, 0xfa, 0xfa};
    const GLubyte bgColorPicking_[3] = {0x00, 0x00, 0xff};

    Grid grid_ = Grid(Eigen::Vector3d{1, 0, 0}, Eigen::Vector3d{0, 0, 1}, GRID_HEIGHT, GRID_WIDTH, GRID_EDGE_LEN);

    bool picking_ = false;
    int pickedNode_ = -1;
    GLfloat pickedDepth_;

    vcg::Trackball trackball_;

    std::vector<float> fixGridDeltas;


    virtual bool initApp()
    {
        glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
        return true;
    }

    virtual bool mainLoop(double delta)
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        vcg::Point3f center = {
            (float)(GRID_WIDTH -  1) * GRID_EDGE_LEN / 2.0,
            0,
            (float)(GRID_HEIGHT -  1) * GRID_EDGE_LEN / 2.0,
        };
        gluLookAt(2,2,2,   center[0],center[1],center[2],   0,1,0);
        trackball_.SetPosition(center);
        trackball_.GetView();
        trackball_.Apply();

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(40, (GLdouble)getFramebufferSize()(0)/getFramebufferSize()(1), 0.1, 100);

        Eigen::Vector2d curPos = input_.getCursorPos();
        curPos(0) = curPos(0) * getWindowContentScale()(0);
        curPos(1) = getFramebufferSize()(1) - curPos(1) * getWindowContentScale()(1);

        if(!ImGui::GetIO().WantCaptureMouse)
        {
            if(input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
            {
                draw(true);

                GLubyte pickedColor_[3];
                glReadPixels(curPos(0), curPos(1), 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &pickedColor_);
                glReadPixels(curPos(0), curPos(1), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &pickedDepth_);
                pickedNode_ = color2node(pickedColor_[0], pickedColor_[1], pickedColor_[2]);
                
                if(pickedNode_ != -1)
                    picking_ = true;
                else
                    trackball_.MouseDown((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
            }

            if(input_.isMouseButtonHeld(GLFW_MOUSE_BUTTON_LEFT))
            {
                if(picking_)
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
                        pickedDepth_ * 2.0 - 1.0,
                        1
                    };

                    Eigen::Vector4d worldPos = (projectionMat * modelviewMat).inverse() * curPosHom;

                    grid_.setNodePos(pickedNode_, {
                        worldPos(0) / worldPos(3),
                        worldPos(1) / worldPos(3),
                        worldPos(2) / worldPos(3)
                    });
                }
                else
                    trackball_.MouseMove((int)curPos(0), (int)curPos(1));
            }

            if(input_.isMouseButtonReleased(GLFW_MOUSE_BUTTON_LEFT))
            {
                picking_ = false;
                trackball_.MouseUp((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
            }
        }

        draw();
        trackball_.DrawPostApply();
        ImGui::Begin("Sim");
        if(ImGui::Button("Fix it!"))
            fixGrid();
        ImGui::PlotLines("Deltas", fixGridDeltas.data(), fixGridDeltas.size());
        ImGui::End();

        return true;
    }


    void draw(bool pickingMode = false)
    {
        if(pickingMode)
            glClearColor(bgColorPicking_[0]/255.0, bgColorPicking_[1]/255.0, bgColorPicking_[2]/255.0, 1.0f);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(!pickingMode)
        {
            glColor3ub(0x0e, 0x0e, 0x0e);
            glLineWidth(3.0f);

            glBegin(GL_LINES);
            for(int e = 0; e < grid_.getNEdges(); e++)
            {
                const Eigen::Array2i edgeNodes = grid_.getEdge(e);
                const Eigen::Vector3d nodePosA = grid_.getNodePos(edgeNodes(0));
                const Eigen::Vector3d nodePosB = grid_.getNodePos(edgeNodes(1));
                glVertex3d(nodePosA(0), nodePosA(1), nodePosA(2));
                glVertex3d(nodePosB(0), nodePosB(1), nodePosB(2));
            }
            glEnd();
        }

        if(pickingMode)
            glPointSize(30.0f);
        else
            glPointSize(18.0f);

        glBegin(GL_POINTS);
        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            if(pickingMode)
            {
                GLubyte r, g, b;
                std::tie(r, g, b) = node2color(n);
                glColor3ub(r, g, b);
            }
            else if(!picking_ || (picking_ && pickedNode_ != n))
                glColor3ub(0x0e, 0x0e, 0x0e);
            else
                continue;

            const Eigen::Vector3d nodePos = grid_.getNodePos(n);
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        if(!pickingMode && picking_ && pickedNode_ != -1)
        {
            glColor3ub(0xff, 0x00, 0x00);
            const Eigen::Vector3d nodePos = grid_.getNodePos(pickedNode_);
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        glEnd();

        if(pickingMode)
            glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
    }


    std::tuple<GLubyte, GLubyte, GLubyte> node2color(int i)
    {
        int row = i / GRID_WIDTH;
        int col = i - (row * GRID_WIDTH);
        return {(GLubyte)row+1, (GLubyte)col+1, 0};
    }

    int color2node(GLubyte r, GLubyte g, GLubyte b)
    {
        if(b != 0)
            return -1;

        return (r-1) * GRID_WIDTH + (g-1);
    }

    void fixGrid()
    {
        int nIters = 0;
        bool stop = false;
        Eigen::Array2i currEdge;
        Eigen::Vector3d p1, p2, v;
        double dist, delta, totDeltas;

        fixGridDeltas.clear();

        while(!stop)
        {
            nIters++;
            stop = true;
            totDeltas = 0;

            for(int e = 0; e < grid_.getNEdges(); e++)
            {
                currEdge = grid_.getEdge(e);
                p1 = grid_.getNodePos(currEdge[0]);
                p2 = grid_.getNodePos(currEdge[1]);

                v = p1 - p2;
                dist = v.norm();
                v.normalize();

                if(GRID_EDGE_LEN - dist > SIM_TOL)
                    stop = false;

                delta = (GRID_EDGE_LEN - dist) / 2.0;
                totDeltas += delta;
                p1 = p1 + delta * v;
                p2 = p2 - delta * v;

                grid_.setNodePos(currEdge[0], p1);
                grid_.setNodePos(currEdge[1], p2);
            }

            fixGridDeltas.push_back((float)totDeltas);
        }

        frmwrk::Debug::log("N iters: %i", nIters);
    }
};

// Entry point - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int main(int argc, char const *argv[])
{
    MainApp app = MainApp();
    app.run();

    fflush(stdin);

    return 0;
}
