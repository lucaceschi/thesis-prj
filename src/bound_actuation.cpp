#include "framework/app.hpp"
#include "framework/debug.hpp"
#include "grid.hpp"
#include "constraints.hpp"

#include <stdio.h>
#include <unordered_map>

#include <Eigen/Dense>
#include <imgui.h>
#include <wrap/gui/trackball.h>


#define N_GRIDS 2
#define MAX_GRID_COLS 10

#define GRID_EDGE_LEN 0.2
#define GRID_1_COLS 6
#define GRID_1_ROWS 4
#define GRID_1_CENTER Eigen::Vector3d{0, 1, 0}
#define GRID_1_X_TANG_VEC Eigen::Vector3d{1, 0, 0}
#define GRID_1_Y_TANG_VEC Eigen::Vector3d{0, 0, 1}
#define GRID_2_COLS 4
#define GRID_2_ROWS 10
#define GRID_2_CENTER Eigen::Vector3d{0, 0.5, 0}
#define GRID_2_X_TANG_VEC Eigen::Vector3d{1, 0, 1}
#define GRID_2_Y_TANG_VEC Eigen::Vector3d{-1, 0, 1}

#define SIM_GRAV_SHIFT 1e-2
#define SIM_SPHERE_RADIUS 0.5
#define SIM_TOL 1e-2
#define SIM_MAX_ITERS 1000


struct Pick
{
    Pick(int gridIdx, int nodeIdx, GLfloat depth)
        : pick(true), gridIdx(gridIdx), nodeIdx(nodeIdx), depth(depth) {};
    Pick() : pick(false) {};
    
    bool pick;
    int gridIdx;
    int nodeIdx;
    GLfloat depth;
};


class MainApp : public frmwrk::App
{
public:
    MainApp()
        : App("Boundary actuation", Eigen::Vector2i{1000, 800}, true),
          grids_{
              Grid(GRID_1_CENTER, GRID_1_ROWS, GRID_1_COLS, GRID_1_X_TANG_VEC, GRID_1_Y_TANG_VEC, GRID_EDGE_LEN),
              Grid(GRID_2_CENTER, GRID_2_ROWS, GRID_2_COLS, GRID_2_X_TANG_VEC, GRID_2_Y_TANG_VEC, GRID_EDGE_LEN)
          },
          edgeLenCs_{
              EdgeLenConstr(&grids_[0], 0.2),
              EdgeLenConstr(&grids_[1], 0.2)
          },
          sphereCollCs_{
              SphereCollConstr(&grids_[0], Eigen::Vector3d{0.2, 0, 0}, 0.5),
              SphereCollConstr(&grids_[1], Eigen::Vector3d{0.2, 0, 0}, 0.5),
              SphereCollConstr(&grids_[0], Eigen::Vector3d{-0.2, 0, 0}, 0.5),
              SphereCollConstr(&grids_[1], Eigen::Vector3d{-0.2, 0, 0}, 0.5)
          },
          gridColors_{
              {0x68, 0x2b, 0x68},
              {0x2b, 0x2b, 0x78}
          },
          bgColorRender_{0xff, 0xff, 0xff},
          bgColorPicking_{0x00, 0x00, 0xff},
          playSim_(false),
          simCollision_(false),
          simIters_(0)
    {

    }

private:
    Grid grids_[N_GRIDS];
    const GLubyte gridColors_[N_GRIDS][3];
    EdgeLenConstr edgeLenCs_[N_GRIDS];
    SphereCollConstr sphereCollCs_[2 * N_GRIDS];

    const GLubyte bgColorRender_[3];
    const GLubyte bgColorPicking_[3];
    Pick pick_;

    vcg::Trackball trackball_;

    bool playSim_;
    bool simCollision_;
    int simIters_;
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
                GLubyte pickedColor[3];
                GLfloat pickedDepth;
                int pickedNodeIdx;

                for(int g = 0; g < N_GRIDS; g++)
                {
                    drawGridPick(g);
                    glReadPixels(curPos(0), curPos(1), 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &pickedColor);
                    glReadPixels(curPos(0), curPos(1), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &pickedDepth);
                    pickedNodeIdx = color2node(pickedColor[0], pickedColor[1], pickedColor[2]);

                    if(pickedNodeIdx == -1)
                        trackball_.MouseDown((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
                    else
                    {
                        pick_ = Pick(g, pickedNodeIdx, pickedDepth);
                        grids_[g].fixedNodes.insert(pickedNodeIdx);
                        break;
                    }
                }
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

                    grids_[pick_.gridIdx].getNodePos(pick_.nodeIdx) = Eigen::Vector3d{
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
                if(pick_.pick)
                    grids_[pick_.gridIdx].fixedNodes.erase(pick_.nodeIdx);
                pick_ = Pick();
                trackball_.MouseUp((int)curPos(0), (int)curPos(1), vcg::Trackball::BUTTON_LEFT);
            }
        }

        if(playSim_)
            simIters_ = simGrids(deltaTime);

        for(int g = 0; g < N_GRIDS; g++)
            drawGridRender(g, (g == 0));

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


    void drawGridRender(int gridIdx, bool clearBuffer)
    {
        Grid& grid = grids_[gridIdx];

        if(clearBuffer)
        {
            glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        }

        // Draw edge len constraints
          
        glColor3ub(gridColors_[gridIdx][0], gridColors_[gridIdx][1], gridColors_[gridIdx][2]);
        glLineWidth(3.0f);

        glBegin(GL_LINES);
        for(int e = 0; e < grid.getNEdges(); e++)
        {
            Eigen::Vector3d nodePosA = grid.getNodePos(grid.edge(e)[0]);
            Eigen::Vector3d nodePosB = grid.getNodePos(grid.edge(e)[1]);
            glVertex3d(nodePosA(0), nodePosA(1), nodePosA(2));
            glVertex3d(nodePosB(0), nodePosB(1), nodePosB(2));
        }
        glEnd();

        // Draw grid nodes

        glPointSize(18.0f);

        glBegin(GL_POINTS);
        for(int n = 0; n < grid.getNNodes(); n++)
        {
            if(grid.isNodeFixed(n) || (pick_.pick && pick_.gridIdx == gridIdx && pick_.nodeIdx == n))
                glColor3ub(0xff, 0x00, 0x00);
            else
                glColor3ub(gridColors_[gridIdx][0], gridColors_[gridIdx][1], gridColors_[gridIdx][2]);

            const Eigen::Vector3d nodePos = grid.getNodePos(n);
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        glEnd();

        trackball_.DrawPostApply();
    }

    void drawGridPick(int gridIdx)
    {
        Grid& grid = grids_[gridIdx];
        
        glClearColor(bgColorPicking_[0]/255.0, bgColorPicking_[1]/255.0, bgColorPicking_[2]/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw grid nodes

        glPointSize(30.0f);

        glBegin(GL_POINTS);
        for(int n = 0; n < grid.getNNodes(); n++)
        {
            GLubyte r, g, b;
            std::tie(r, g, b) = node2color(n);
            glColor3ub(r, g, b);

            const Eigen::Vector3d nodePos = grid.getNodePos(n);
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


    int simGrids(double deltaTime)
    {        
        for(int g = 0; g < N_GRIDS; g++)
            for(int n = 0; n < grids_[g].getNNodes(); n++)
            {
                if(grids_[g].isNodeFixed(n))
                    continue;

                grids_[g].getNodePos(n) -= Eigen::Vector3d{0, SIM_GRAV_SHIFT, 0};
            }
        
        bool stop = false;
        int nIters = 0;
        double totDelta;

        simDeltas_.clear();
        while(!stop)
        {
            stop = true;
            totDelta = 0;
            
            for(const EdgeLenConstr& e : edgeLenCs_)
                totDelta += e.resolve();

            if(simCollision_)
                for(const SphereCollConstr& s : sphereCollCs_)
                    totDelta += s.resolve();

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
        for(int g = 0; g < N_GRIDS; g++)
        {
            Grid& grid = grids_[g];
            
            const Eigen::Vector3d cutPlaneNormal = {0, 1, 0};
            
            std::vector<Eigen::Vector3d> newNodes;
            std::unordered_map<int, int> nodeIndexMap;
            std::vector<Eigen::Array2i> newEdges;
            std::vector<double> newEdgeLengths;
            std::unordered_set<int> newFixedNodes;

            for(int n = 0; n < grid.getNNodes(); n++)
            {
                const Eigen::Vector3d p = grid.getNodePos(n);

                // only preserve nodes above the cutting plane
                if(p(1) < 0)
                    continue;

                newNodes.push_back(p);
                nodeIndexMap[n] = newNodes.size() - 1;
            }
            
            for(int e = 0; e < grid.getNEdges(); e++)
            {
                auto p1IndexPair = nodeIndexMap.find(grid.edge(e)[0]);
                auto p2IndexPair = nodeIndexMap.find(grid.edge(e)[1]);

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
                    edgeLength = edgeLenCs_[g].getLength(e);
                }
                else
                {
                    // edge crosses the cutting plane: retrieve the node below the cutting plane
                    // and move it to the closest intersection between the edge and the plane

                    Eigen::Vector3d edgeVec = (grid.getNodePos(grid.edge(e)[0]) - grid.getNodePos(grid.edge(e)[1])).normalized();
                    Eigen::Vector3d newNodePos;

                    if(p1IndexPair == nodeIndexMap.end())
                    {
                        newNodePos = grid.getNodePos(grid.edge(e)[0]);
                        p1Index = newNodes.size();
                        p2Index = p2IndexPair->second;
                    }
                    else
                    {
                        newNodePos = grid.getNodePos(grid.edge(e)[1]);
                        p1Index = p1IndexPair->second;
                        p2Index = newNodes.size();
                    }

                    double deltaLength = newNodePos.dot(cutPlaneNormal) / edgeVec.dot(cutPlaneNormal);

                    newNodePos -= (deltaLength * edgeVec);
                    edgeLength = edgeLenCs_[g].getLength(e) - abs(deltaLength);
                    newNodes.push_back(newNodePos);
                    newFixedNodes.insert(newNodes.size() - 1);
                }

                newEdges.emplace_back(p1Index, p2Index);
                newEdgeLengths.push_back(edgeLength);
            }

            Eigen::Matrix3Xd newNodeMatrix(3, newNodes.size());
            for(int n = 0; n < newNodes.size(); n++)
                newNodeMatrix.col(n) = newNodes[n];

            Eigen::Array2Xi newEdgesMatrix(2, newEdges.size());
            for(int e = 0; e < newEdges.size(); e++)
                newEdgesMatrix.col(e) = newEdges[e];

            grids_[g] = Grid(newNodeMatrix, newEdgesMatrix, newFixedNodes);
            edgeLenCs_[g] = EdgeLenConstr(&grids_[g], newEdgeLengths);
        }
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
