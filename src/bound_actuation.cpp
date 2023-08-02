#include "framework/app.hpp"
#include "framework/debug.hpp"
#include "grid.hpp"

#include <stdio.h>
#include <unordered_map>

#include <Eigen/Dense>
#include <imgui.h>
#include <wrap/gui/trackball.h>


#define GRID_COLS  10
#define GRID_ROWS 10
#define GRID_EDGE_LEN 0.2
#define GRID_CENTER Eigen::Vector3d{0, 0.5, 0}
#define GRID_X_TANG_VEC Eigen::Vector3d{1, 0, 0}
#define GRID_Y_TANG_VEC Eigen::Vector3d{0, 0, 1}

#define SIM_GRAV_SHIFT 1e-2
#define SIM_SPHERE_RADIUS 0.5
#define SIM_TOL 1e-2
#define SIM_MAX_ITERS 1000


class MainApp : public frmwrk::App
{
public:
    MainApp()
        : App("Boundary actuation", Eigen::Vector2i{1000, 800}, true)
    {}

private:
    Grid grid_ = Grid(GRID_CENTER, GRID_ROWS, GRID_COLS, GRID_X_TANG_VEC, GRID_Y_TANG_VEC, GRID_EDGE_LEN);
    
    const GLubyte bgColorRender_[3] = {0xff, 0xff, 0xff};
    const GLubyte bgColorPicking_[3] = {0x00, 0x00, 0xff};
    GLubyte pickedColor_[3];
    int pickedNode_ = -1;
    GLfloat pickedDepth_;

    vcg::Trackball trackball_;

    SphereCollConstr sphereC_ = SphereCollConstr(grid_, SIM_SPHERE_RADIUS);
    bool playSim_ = false;
    bool simCollision_ = false;
    int simIters_ = 0;
    std::vector<float> simDeltas_;


    virtual bool initApp()
    {
        glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
        return true;
    }


    virtual bool mainLoop(double deltaTime)
    {        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(3,3,3,   0,0,0,   0,1,0);
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

                glReadPixels(curPos(0), curPos(1), 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &pickedColor_);
                glReadPixels(curPos(0), curPos(1), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &pickedDepth_);
                pickedNode_ = color2node(pickedColor_[0], pickedColor_[1], pickedColor_[2]);
                
                if(pickedNode_ == -1)
                    trackball_.MouseDown((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
                else
                    grid_.fixedNodes.insert(pickedNode_);
            }

            if(input_.isMouseButtonHeld(GLFW_MOUSE_BUTTON_LEFT))
            {
                if(pickedNode_ != -1)
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

                    grid_.getNodePos(pickedNode_) = Eigen::Vector3d{
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
                grid_.fixedNodes.erase(pickedNode_);
                pickedNode_ = -1;
                trackball_.MouseUp((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
            }
        }

        draw();

        if(playSim_)
            simIters_ = simGrid(deltaTime);

        ImGui::Begin("Sim");
        ImGui::Text("N iters: %i", simIters_);
        ImGui::PlotLines("Deltas", simDeltas_.data(), simDeltas_.size());
        if(ImGui::Button("Play / Pause"))
            playSim_ = !playSim_;
        if(ImGui::Button("Enable / Disable collision"))
            simCollision_ = !simCollision_;
        if(ImGui::Button("Cut"))
            cut();
        ImGui::End();

        return true;
    }


    void draw(bool pickingMode = false)
    {       
        // Set background color
        
        if(pickingMode)
            glClearColor(bgColorPicking_[0]/255.0, bgColorPicking_[1]/255.0, bgColorPicking_[2]/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw edge len constraints

        if(!pickingMode)
        {           
            glColor3ub(0x0e, 0x0e, 0x0e);
            glLineWidth(3.0f);

            glBegin(GL_LINES);
            for(const EdgeLenConstr& e : grid_.edgeLenC)
            {
                const Eigen::Vector3d nodePosA = grid_.getNodePos(e.getNodeIdxs().first);
                const Eigen::Vector3d nodePosB = grid_.getNodePos(e.getNodeIdxs().second);
                glVertex3d(nodePosA(0), nodePosA(1), nodePosA(2));
                glVertex3d(nodePosB(0), nodePosB(1), nodePosB(2));
            }
            glEnd();
        }

        // Draw grid nodes

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
            else if(pickedNode_ == n || grid_.isNodeFixed(n))
                glColor3ub(0xff, 0x00, 0x00);
            else
                glColor3ub(0x0e, 0x0e, 0x0e);

            const Eigen::Vector3d nodePos = grid_.getNodePos(n);
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        glEnd();

        if(pickingMode)
            glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
        else
            trackball_.DrawPostApply();
    }


    std::tuple<GLubyte, GLubyte, GLubyte> node2color(int i)
    {
        int row = i / GRID_COLS;
        int col = i - (row * GRID_COLS);
        return {(GLubyte)row+1, (GLubyte)col+1, 0};
    }

    int color2node(GLubyte r, GLubyte g, GLubyte b)
    {
        if(b != 0)
            return -1;

        return (r-1) * GRID_COLS + (g-1);
    }


    int simGrid(double deltaTime)
    {
        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            if(grid_.isNodeFixed(n))
                continue;

            grid_.getNodePos(n) -= Eigen::Vector3d{0, SIM_GRAV_SHIFT, 0};
        }
        
        bool stop = false;
        int nIters = 0;
        double totDelta;

        simDeltas_.clear();
        while(!stop)
        {
            stop = true;
            totDelta = 0;
            
            for(const EdgeLenConstr& e : grid_.edgeLenC)
                totDelta += e.resolve();

            if(simCollision_)
                totDelta += sphereC_.resolve();

            simDeltas_.push_back(totDelta);

            if(totDelta > SIM_TOL)
                stop = false;

            if(nIters > SIM_MAX_ITERS)
            {
                frmwrk::Debug::logWarning("Sim iters exceeded, stopping");
                stop = true;
                playSim_ = false;
            }

            nIters++;
        }

        return nIters;
    }

    void cut()
    {
        const Eigen::Vector3d cutPlaneNormal = {0, 1, 0};
        
        std::vector<Eigen::Vector3d> newNodes;
        std::unordered_map<int, int> nodeIndexMap;
        std::vector<EdgeLenConstr> newEdgeLenC;
        std::unordered_set<int> newFixedNodes;

        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            const Eigen::Vector3d p = grid_.getNodePos(n);

            // only preserve nodes above the cutting plane
            if(p(1) < 0)
                continue;

            newNodes.push_back(p);
            nodeIndexMap[n] = newNodes.size() - 1;
        }
        
        for(const EdgeLenConstr& e : grid_.edgeLenC)
        {
            auto p1IndexPair = nodeIndexMap.find(e.getNodeIdxs().first);
            auto p2IndexPair = nodeIndexMap.find(e.getNodeIdxs().second);

            // scrap edges below the cutting plane
            if(p1IndexPair == nodeIndexMap.end() && p2IndexPair == nodeIndexMap.end())
                continue;

            int p1Index, p2Index;
            double edgeLength;
            
            if(p1IndexPair != nodeIndexMap.end() && p2IndexPair != nodeIndexMap.end())
            {
                // edge is above the cutting plane, preserve it
                p1Index = p1IndexPair->second;
                p2Index = p2IndexPair->second;
                edgeLength = e.getLength();
            }
            else
            {
                // edge crosses the cutting plane: retrieve the node below the cutting plane
                // and move it to the closest intersection between the edge and the plane

                Eigen::Vector3d edgeVec = (grid_.getNodePos(e.getNodeIdxs().first) - grid_.getNodePos(e.getNodeIdxs().second)).normalized();
                Eigen::Vector3d newNode;

                if(p1IndexPair == nodeIndexMap.end())
                {
                    newNode = grid_.getNodePos(e.getNodeIdxs().first);
                    p1Index = newNodes.size();
                    p2Index = p2IndexPair->second;
                }
                else
                {
                    newNode = grid_.getNodePos(e.getNodeIdxs().second);
                    p1Index = p1IndexPair->second;
                    p2Index = newNodes.size();
                }

                double deltaLength = newNode.dot(cutPlaneNormal) / edgeVec.dot(cutPlaneNormal);

                newNode -= (deltaLength * edgeVec);
                edgeLength = e.getLength() - abs(deltaLength);
                newNodes.push_back(newNode);
                newFixedNodes.insert(newNodes.size() - 1);
            }

            newEdgeLenC.emplace_back(grid_, p1Index, p2Index, edgeLength);
        }

        Eigen::Matrix3Xd newNodeMatrix(3, newNodes.size());
        for(int n = 0; n < newNodes.size(); n++)
            newNodeMatrix.col(n) = newNodes[n];

        grid_ = Grid(newNodeMatrix, newEdgeLenC, newFixedNodes);
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
