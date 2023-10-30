#include "framework/app.hpp"
#include "framework/debug.hpp"
#include "grid.hpp"
#include "constraints.hpp"
#include "forces.hpp"

#include <stdio.h>
#include <vector>
#include <array>
#include <unordered_map>

#include <Eigen/Dense>
#include <imgui.h>
#include <wrap/gui/trackball.h>


#define MAX_GRID_COLS 10
#define INIT_GRID_EDGE_LEN 0.1

#define SIM_GRAV_SHIFT 5e-3
#define SIM_INIT_ABS_TOL 1e-10
#define SIM_INIT_REL_TOL 2.22e-16
#define SIM_MAX_ITERS 100000
#define SIM_SCISSOR_EE_MIN_DIST 1e-3
#define SIM_SCISSOR_CC_MIN_DIST 1e-2
#define SIM_SCISSOR_CN_MIN_DIST 1e-2


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

enum ViewPoint
{
    TOP,
    FRONT,
    RIGHT
};


class MainApp : public frmwrk::App
{
public:
    MainApp()
        : App("Boundary actuation", Eigen::Vector2i{1200, 800}, true),
          grids_{
              Grid({0, 1, 0}, 16, 16, {1, 0, 0}, {0, 0, 1},  INIT_GRID_EDGE_LEN),
              Grid({0, 1, 0}, 16, 16, {1, 0, 1}, {-1, 0, 1}, INIT_GRID_EDGE_LEN)
          },
          edgeLenCs_{
              EdgeLenConstr(grids_, 0, INIT_GRID_EDGE_LEN),
              EdgeLenConstr(grids_, 1, INIT_GRID_EDGE_LEN)
          },
          shearingCs_{
              ShearLimitConstr(grids_, 0, INIT_GRID_EDGE_LEN, M_PI_4),
              ShearLimitConstr(grids_, 1, INIT_GRID_EDGE_LEN, M_PI_4)
          },
          sphereCollCs_{
              SphereCollConstr(0, Eigen::Vector3d{0, 0, 0}, 0.5),
              SphereCollConstr(1, Eigen::Vector3d{0, 0, 0}, 0.5)
          },
          fixCs_{
              FixedNodeConstr(0),
              FixedNodeConstr(1)
          },
          gravForce_(Eigen::Vector3d{0, -SIM_GRAV_SHIFT, 0}),
          gridColors_{
              {0x60, 0x60, 0xde},
              {0x60, 0xbc, 0xc0}
          },
          bgColorRender_{0xff, 0xff, 0xff},
          bgColorPicking_{0x00, 0x00, 0xff},
          orthoCamera_(false),
          viewPoint_(ViewPoint::FRONT),
          playSim_(false),
          absTolSimSlider_(SIM_INIT_ABS_TOL),
          absTolSim_(SIM_INIT_ABS_TOL * INIT_GRID_EDGE_LEN),
          relTolSimSlider_(SIM_INIT_REL_TOL),
          relTolSim_(SIM_INIT_REL_TOL * INIT_GRID_EDGE_LEN),
          doNStepsSim_(1),
          gravSim_(true),
          edgeSim_(true),
          simCollision_(true),
          simScissors_(true),
          addScissors_(false),
          simIters_(0)
    {}

private:
    std::vector<Grid> grids_;
    std::vector<std::array<GLubyte, 3>> gridColors_;
    std::vector<EdgeLenConstr> edgeLenCs_;
    std::vector<ShearLimitConstr> shearingCs_;
    std::vector<SphereCollConstr> sphereCollCs_;
    std::vector<PlaneCollConstr> planeCollCs_;
    std::vector<FixedNodeConstr> fixCs_;
    std::vector<ScissorConstr> scissorCs_;

    ConstantForce gravForce_;

    const GLubyte bgColorRender_[3];
    const GLubyte bgColorPicking_[3];
    Pick pick_;

    vcg::Trackball trackball_;
    bool orthoCamera_;
    ViewPoint viewPoint_;

    bool playSim_;
    float absTolSim_, absTolSimSlider_;
    float relTolSim_, relTolSimSlider_;
    int doNStepsSim_;
    bool gravSim_;
    bool edgeSim_;
    bool simCollision_;
    bool simScissors_;
    bool addScissors_;
    int simIters_;
    std::vector<float> simTotDispls;
    std::vector<float> simMaxDeltas_;
    std::vector<Eigen::Matrix3Xd> prevNodePos_;


    virtual bool initApp()
    {
        glClearColor(bgColorRender_[0]/255.0, bgColorRender_[1]/255.0, bgColorRender_[2]/255.0, 1.0f);
        prevNodePos_.resize(grids_.size());
        return true;
    }


    void setViewpoint(ViewPoint newViewpoint)
    {
        viewPoint_ = newViewpoint;
        trackball_.Reset();
    }


    virtual bool mainLoop(double deltaTime)
    {        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        GLfloat framebufferRatio = (GLfloat)getFramebufferSize()(0)/getFramebufferSize()(1);
        if(orthoCamera_)
            glOrtho(-1.25 * framebufferRatio, 1.25 * framebufferRatio, -1.25, 1.25, -10, 10);
        else
            gluPerspective(40, framebufferRatio, 0.1, 100);
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        switch(viewPoint_)
        {
            case ViewPoint::TOP:
                gluLookAt(0,4,0,   0,0,0,   0,0,1);
                break;
            case ViewPoint::RIGHT:
                gluLookAt(4,0,0,   0,0,0,   0,1,0);
                break;
            case ViewPoint::FRONT:
            default:
                gluLookAt(0,0,-4,  0,0,0,   0,1,0);
                break;
        }
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

                for(int g = 0; g < grids_.size(); g++)
                {
                    drawGridPick(g);
                    glReadPixels(curPos(0), curPos(1), 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &pickedColor);
                    glReadPixels(curPos(0), curPos(1), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &pickedDepth);
                    pickedNodeIdx = color2node(pickedColor[0], pickedColor[1], pickedColor[2]);

                    if(pickedNodeIdx != -1 && input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT))
                    {
                        //vcg::Point3f pickedNodePos;
                        //pickedNodePos.FromEigenVector(grids_[g].nodePos(pickedNodeIdx));
                        //trackball_.Translate(vcg::Point3f(1, 0, 0));
                        if(!fixCs_[g].isNodeFixed(pickedNodeIdx))
                            fixCs_[g].fixNode(grids_, pickedNodeIdx);
                        else
                            fixCs_[g].freeNode(pickedNodeIdx);
                        break;
                    }
                    else if(pickedNodeIdx != -1 && input_.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
                    {
                        pick_ = Pick(g, pickedNodeIdx, pickedDepth);
                        fixCs_[g].fixNode(grids_, pickedNodeIdx);
                        break;
                    }
                }

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

                    grids_[pick_.gridIdx].nodePos(pick_.nodeIdx) = Eigen::Vector3d{
                        worldPos(0) / worldPos(3),
                        worldPos(1) / worldPos(3),
                        worldPos(2) / worldPos(3)
                    };
                    fixCs_[pick_.gridIdx].fixNode(grids_, pick_.nodeIdx);
                }
                else
                    trackball_.MouseMove((int)curPos(0), (int)curPos(1));
            }

            if(input_.isMouseButtonReleased(GLFW_MOUSE_BUTTON_LEFT))
            {
                if(pick_.pick)
                    fixCs_[pick_.gridIdx].freeNode(pick_.nodeIdx);
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

            if(input_.isKeyPressed(GLFW_KEY_SPACE))
                playSim_ = !playSim_; 
        }

        if(ImGui::BeginMainMenuBar())
        {
            ImGuiID exportPopupId = ImGui::GetID("export_grids_popup");
            if(ImGui::BeginPopup("export_grids_popup"))
            {
                ImGui::SeparatorText("Export grids");
                static char filename[20] = "";
                bool pressedEnter = ImGui::InputTextWithHint("", "base file name", filename, 64, ImGuiInputTextFlags_EnterReturnsTrue);
                ImGui::SameLine();
                if(pressedEnter || ImGui::Button("Export"))
                {
                    for(int g = 0; g < grids_.size(); g++)
                        grids_[g].exportPly(std::string(filename) + "_" + std::to_string(g) + ".ply");
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
            
            if(ImGui::BeginMenu("File"))
            {
                if(ImGui::MenuItem("Export grids"))
                {
                    ImGui::OpenPopup(exportPopupId);
                }
                ImGui::EndMenu();
            }
            if(ImGui::BeginMenu("Camera"))
            {
                ImGui::MenuItem("Ortho proj", nullptr, &orthoCamera_);
                ImGui::Separator();
                if(ImGui::MenuItem("Top viewpoint"))
                    setViewpoint(ViewPoint::TOP);
                if(ImGui::MenuItem("Front viewpoint"))
                    setViewpoint(ViewPoint::FRONT);
                if(ImGui::MenuItem("Right viewpoint"))
                    setViewpoint(ViewPoint::RIGHT);
                ImGui::EndMenu();
            }
            ImGui::EndMainMenuBar();
        }

        ImGui::Begin("Sim");
        ImGui::Text("N iters: %i", simIters_);
        ImGui::PlotLines("Tot displ", simTotDispls.data(), simTotDispls.size(), 0, nullptr, FLT_MAX, FLT_MAX, {200, 30});
        ImGui::PlotLines("Max delta", simMaxDeltas_.data(), simMaxDeltas_.size(), 0, nullptr, FLT_MAX, FLT_MAX, {200, 30});
        ImGui::DragFloat("Abs tol", &absTolSimSlider_, 1e-2, 0.1, 1e-18, "%.2e");
        ImGui::DragFloat("Rel tol", &relTolSimSlider_, 1e-2, 0.1, 1e-18, "%.2e");
        absTolSim_ = absTolSimSlider_ * INIT_GRID_EDGE_LEN;
        relTolSim_ = relTolSimSlider_ * INIT_GRID_EDGE_LEN;
        
        ImGui::Checkbox("Play sim", &playSim_);
        if(ImGui::Button("Do full iteration"))
            simIters_ = simGrids();
        if(ImGui::Button("Do N steps"))
            simIters_ = simGrids(doNStepsSim_);
        ImGui::DragInt("N", &doNStepsSim_, 1, 1, 1000);
        ImGui::End();

        ImGui::Begin("Constraints");
        ImGui::Checkbox("Gravity", &gravSim_);
        ImGui::Checkbox("Edge Lenght", &edgeSim_);
        ImGui::Checkbox("Collisions", &simCollision_);
        ImGui::Checkbox("Scissors", &simScissors_);
        ImGui::Checkbox("Add scissors", &addScissors_);
        if(ImGui::Button("Cut"))
            cut();
        ImGui::End();

        if(playSim_)
            simIters_ = simGrids();

        if(addScissors_)
        {
            int nAdded = addScissorC();
            if(nAdded > 0)
                frmwrk::Debug::log("Added %i sciss constr", nAdded);
        }

        for(int g = 0; g < grids_.size(); g++)
            drawGridRender(g, (g == 0));
        trackball_.DrawPostApply();

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
        glLineWidth(2.0f * trackball_.track.sca);

        glBegin(GL_LINES);
        for(int e = 0; e < grid.getNEdges(); e++)
        {
            Eigen::Vector3d nodePosA = grid.nodePos(grid.edge(e)[0]);
            Eigen::Vector3d nodePosB = grid.nodePos(grid.edge(e)[1]);
            glVertex3d(nodePosA(0), nodePosA(1), nodePosA(2));
            glVertex3d(nodePosB(0), nodePosB(1), nodePosB(2));
        }
        glEnd();

        // Draw grid nodes

        glPointSize(10.0f * trackball_.track.sca);

        glBegin(GL_POINTS);
        for(int n = 0; n < grid.getNNodes(); n++)
        {
            if(fixCs_[gridIdx].isNodeFixed(n) || (pick_.pick && pick_.gridIdx == gridIdx && pick_.nodeIdx == n))
                glColor3ub(0xff, 0x00, 0x00);
            else
                glColor3ub(gridColors_[gridIdx][0], gridColors_[gridIdx][1], gridColors_[gridIdx][2]);

            const Eigen::Vector3d nodePos = grid.nodePos(n);
            glVertex3d(nodePos(0), nodePos(1), nodePos(2));
        }
        glEnd();

        // Draw scissor constr

        glPointSize(10.0f);
        glColor3ub(0xff, 0x00, 0xff);
        glBegin(GL_POINTS);
        for(const ScissorConstr& s : scissorCs_)
        {
            Eigen::Vector3d cpA = s.getCrossPointA(grids_);
            Eigen::Vector3d cpB = s.getCrossPointB(grids_);
            glVertex3d(cpA(0), cpA(1), cpA(2));
            glVertex3d(cpB(0), cpB(1), cpB(2));
        }
        glEnd();
    }

    void drawGridPick(int gridIdx)
    {
        Grid& grid = grids_[gridIdx];
        
        glClearColor(bgColorPicking_[0]/255.0, bgColorPicking_[1]/255.0, bgColorPicking_[2]/255.0, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw grid nodes

        glPointSize(24.0f * trackball_.track.sca);

        glBegin(GL_POINTS);
        for(int n = 0; n < grid.getNNodes(); n++)
        {
            GLubyte r, g, b;
            std::tie(r, g, b) = node2color(n);
            glColor3ub(r, g, b);

            const Eigen::Vector3d nodePos = grid.nodePos(n);
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


    float smoothstep(float edge0, float edge1, float x)
    {
        // Scale, and clamp x to 0..1 range
        x = clamp((x - edge0) / (edge1 - edge0));
        return x * x * (3.0f - 2.0f * x);
    }

    float clamp(float x, float lowerlimit = 0.0f, float upperlimit = 1.0f)
    {
        if (x < lowerlimit) return lowerlimit;
        if (x > upperlimit) return upperlimit;
        return x;
    }

    int simGrids(int doNIters = std::numeric_limits<int>::max())
    {
        if(gravSim_)
            for(int g = 0; g < grids_.size(); g++)
                for(int n = 0; n < grids_[g].getNNodes(); n++)
                    grids_[g].nodePos(n) += gravForce_.F(grids_[g].nodePos(n));
        
        bool stop = false;
        int nIters = 0;
        double totDisplacement, totPrevNorm;
        double maxDelta;

        simTotDispls.clear();
        simMaxDeltas_.clear();
        while(!stop)
        {
            maxDelta = 0;
            for(int g = 0; g < grids_.size(); g++)
                prevNodePos_[g] = Eigen::Matrix3Xd(grids_[g].pos);

            for(const FixedNodeConstr& f : fixCs_)
                maxDelta = std::max(maxDelta, f.resolve(grids_));
            
            if(edgeSim_)
            {
                for(const EdgeLenConstr& e : edgeLenCs_)
                    maxDelta = std::max(maxDelta, e.resolve(grids_));
                for(const ShearLimitConstr& s : shearingCs_)
                    maxDelta = std::max(maxDelta, s.resolve(grids_));
            }

            if(simScissors_)
                for(const ScissorConstr& s : scissorCs_)
                    maxDelta = std::max(maxDelta, s.resolve(grids_));

            if(simCollision_)
            {
                for(const SphereCollConstr& s : sphereCollCs_)
                    maxDelta = std::max(maxDelta, s.resolve(grids_));
                for(const PlaneCollConstr& p : planeCollCs_)
                        maxDelta = std::max(maxDelta, p.resolve(grids_));
            }

            simMaxDeltas_.push_back(maxDelta);

            totDisplacement = totPrevNorm = 0;
            for(int g = 0; g < grids_.size(); g++)
            {
                totDisplacement += (prevNodePos_[g] - grids_[g].pos).squaredNorm();
                totPrevNorm += prevNodePos_[g].squaredNorm();
            }
            simTotDispls.push_back(totDisplacement);

            if(doNIters == std::numeric_limits<int>::max())
            {
                if(totDisplacement <= absTolSim_ + totPrevNorm * relTolSim_)
                    stop = true;

                if(nIters > SIM_MAX_ITERS)
                {
                    frmwrk::Debug::logWarning("Sim iters exceeded, stopping");
                    stop = true;
                    playSim_ = false;
                }
            }

            nIters++;

            if(nIters >= doNIters)
                break;
        }

        return nIters;
    }

    void cut()
    {
        std::vector<std::unordered_map<int, int>> edgeIndexMaps(grids_.size());
        
        for(int g = 0; g < grids_.size(); g++)
        {
            Grid& grid = grids_[g];
            
            const Eigen::Vector3d cutPlaneNormal = {0, 1, 0};
            
            std::vector<Eigen::Vector3d> newNodes;
            std::unordered_map<int, int> nodeIndexMap;
            std::vector<Eigen::Array2i> newEdges;
            std::vector<double> newEdgeLengths;
            FixedNodeConstr newFixCs_(g);

            for(int n = 0; n < grid.getNNodes(); n++)
            {
                const Eigen::Vector3d p = grid.nodePos(n);

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

                    Eigen::Vector3d edgeVec = (grid.nodePos(grid.edge(e)[0]) - grid.nodePos(grid.edge(e)[1])).normalized();
                    Eigen::Vector3d newNodePos;

                    if(p1IndexPair == nodeIndexMap.end())
                    {
                        newNodePos = grid.nodePos(grid.edge(e)[0]);
                        p1Index = newNodes.size();
                        p2Index = p2IndexPair->second;
                    }
                    else
                    {
                        newNodePos = grid.nodePos(grid.edge(e)[1]);
                        p1Index = p1IndexPair->second;
                        p2Index = newNodes.size();
                    }

                    double deltaLength = newNodePos.dot(cutPlaneNormal) / edgeVec.dot(cutPlaneNormal);

                    newNodePos -= (deltaLength * edgeVec);
                    edgeLength = edgeLenCs_[g].getLength(e) - abs(deltaLength);
                    newNodes.push_back(newNodePos);
                    newFixCs_.fixNode(newNodes.size() - 1, newNodePos);
                }

                newEdges.emplace_back(p1Index, p2Index);
                edgeIndexMaps[g][e] = newEdges.size() - 1;
                newEdgeLengths.push_back(edgeLength);
            }

            Eigen::Matrix3Xd newNodeMatrix(3, newNodes.size());
            for(int n = 0; n < newNodes.size(); n++)
                newNodeMatrix.col(n) = newNodes[n];

            Eigen::Array2Xi newEdgesMatrix(2, newEdges.size());
            for(int e = 0; e < newEdges.size(); e++)
                newEdgesMatrix.col(e) = newEdges[e];

            grids_[g] = Grid(newNodeMatrix, newEdgesMatrix);

            edgeLenCs_[g] = EdgeLenConstr(grids_, g, newEdgeLengths);

            for(FixedNodeConstr::const_iterator it = fixCs_[g].cbegin(); it != fixCs_[g].cend(); it++)
            {
                auto newIndex = nodeIndexMap.find(it->first);
                if(newIndex != nodeIndexMap.end())
                    newFixCs_.fixNode(grids_, newIndex->second);
            }
            fixCs_[g] = newFixCs_;
        }

        // recreate scissor constraints
        // WARN: exact parametric positions of each scissor are not preserved
        std::vector<ScissorConstr> newScissors;
        for(const ScissorConstr& s : scissorCs_)
        {
            std::unordered_map<int, int>::iterator edgeAIt = edgeIndexMaps[s.getGridAIdx()].find(s.getEdgeAIdx());
            std::unordered_map<int, int>::iterator edgeBIt = edgeIndexMaps[s.getGridBIdx()].find(s.getEdgeBIdx());

            if(edgeAIt != edgeIndexMaps[s.getGridAIdx()].end() &&
               edgeBIt != edgeIndexMaps[s.getGridBIdx()].end())
                newScissors.emplace_back(grids_, s.getGridAIdx(), edgeAIt->second, s.getGridBIdx(), edgeBIt->second);
        }
        scissorCs_.swap(newScissors);
    }

    int addScissorC()
    {
        int nAdded = 0;
        for(int gA = 0; gA < grids_.size()-1; gA++)
            for(int gB = gA+1; gB < grids_.size(); gB++)
                for(int eA = 0; eA < grids_[gA].getNEdges(); eA++)
                    for(int eB = 0; eB < grids_[gB].getNEdges(); eB++)
                    {                        
                        ScissorConstr s = ScissorConstr(grids_, gA, eA, gB, eB);

                        if(s.getDist() > SIM_SCISSOR_EE_MIN_DIST ||
                           std::abs(s.getAlpha() - 0.5) > (0.5 - SIM_SCISSOR_CN_MIN_DIST) ||
                           std::abs(s.getBeta()  - 0.5) > (0.5 - SIM_SCISSOR_CN_MIN_DIST))
                            continue;


                        Eigen::Vector3d crossPoint = (s.getCrossPointA(grids_) + s.getCrossPointB(grids_)) / 2;
                        bool acceptable = true;
                        for(const ScissorConstr& otherS : scissorCs_)
                        {
                            Eigen::Vector3d otherCrossPoint = (otherS.getCrossPointA(grids_) + otherS.getCrossPointB(grids_)) / 2;
                            if((crossPoint - otherCrossPoint).norm() < SIM_SCISSOR_CC_MIN_DIST)
                            {
                                acceptable = false;
                                break;
                            }
                        }

                        if(acceptable)
                        {
                            scissorCs_.push_back(s);
                            nAdded++;
                        }
                    }
        
        return nAdded;
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
