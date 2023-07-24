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

#define SIM_TOL 1e-4


class MainApp : public frmwrk::App
{
public:
    MainApp()
        : App("Boundary actuation", Eigen::Vector2i{1000, 800}, true)
    {}

private:
    Grid grid_ = Grid({0, 0.5, 0}, {1, 0, 0}, {0, 0, 1}, GRID_ROWS, GRID_COLS, GRID_EDGE_LEN);

    const GLubyte bgColorRender_[3] = {0xff, 0xff, 0xff};
    const GLubyte bgColorPicking_[3] = {0x00, 0x00, 0xff};
    GLubyte pickedColor_[3];
    bool picking_ = false;
    int pickedNode_ = -1;
    GLfloat pickedDepth_;

    vcg::Trackball trackball_;

    bool shapeConstraint_ = true;
    bool fixedGridNodes_[GRID_ROWS * GRID_COLS] = {false};
    int simSteps_ = 10;
    int simIters_ = 0;
    std::vector<float> pbdDeltas_;


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

        ImGui::Begin("Sim");
        ImGui::Text("N iters: %i", simIters_);
        ImGui::PlotLines("Deltas", pbdDeltas_.data(), pbdDeltas_.size());
        ImGui::InputInt("Steps", &simSteps_);
        if(ImGui::Button("Go!"))
            for(int i = 0; i < simSteps_; i++)
                simIters_ = simGrid(deltaTime);
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

        // Draw grid edges

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
            else if(picking_ && pickedNode_ == n)
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
            if(fixedGridNodes_[n])
                continue;
            
            Eigen::Vector3d pos = grid_.getNodePos(n);
            grid_.setNodePos(n, pos - Eigen::Vector3d{0, SIM_TOL, 0});
        }
        
        int nIters = 0;
        bool stop = false;
        Eigen::Array2i currEdge;
        Eigen::Vector3d p1, p2, v;
        double edgeLen, dist, delta, totDeltas;

        pbdDeltas_.clear();

        while(!stop)
        {
            nIters++;
            stop = true;
            totDeltas = 0;

            for(int e = 0; e < grid_.getNEdges(); e++)
            {
                currEdge = grid_.getEdge(e);
                edgeLen = grid_.getEdgeLen(e);
                p1 = grid_.getNodePos(currEdge[0]);
                p2 = grid_.getNodePos(currEdge[1]);

                v = p1 - p2;
                dist = v.norm();
                v.normalize();

                if(abs(edgeLen - dist) > SIM_TOL)
                    stop = false;

                delta = (edgeLen - dist);
                totDeltas += abs(delta);

                if(fixedGridNodes_[currEdge[0]] || (picking_ && currEdge[0] == pickedNode_))
                    p2 = p2 - delta * v;
                else if(fixedGridNodes_[currEdge[1]] || (picking_ && currEdge[1] == pickedNode_))
                    p1 = p1 + delta * v;
                else
                {
                    p1 = p1 + delta/2.0 * v;
                    p2 = p2 - delta/2.0 * v;
                }

                if(shapeConstraint_)
                {
                    if(p1.squaredNorm() < 0.2)
                    {
                        stop = false;
                        while(p1.squaredNorm() < 0.2)
                            p1 += (p1 * SIM_TOL);
                    }

                    if(p2.squaredNorm() < 0.2)
                    {
                        stop = false;
                        while(p2.squaredNorm() < 0.2)
                            p2 += (p2 * SIM_TOL);
                    }
                }

                grid_.setNodePos(currEdge[0], p1);
                grid_.setNodePos(currEdge[1], p2);
            }

            pbdDeltas_.push_back((float)totDeltas);
        }

        return nIters;
    }

    /*int simGrid(double deltaTime)
    {
        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            if(fixedGridNodes_[n])
                continue;

            grid_.setNodePos(n, grid_.getNodePos(n) - Eigen::Vector3d{0, SIM_TOL, 0});
        }
        
        int nIters = 0;
        bool stop = false;
        Eigen::Array2i currEdge;
        Eigen::Vector3d p1, p2, v;
        double edgeLen, dist, delta, totDeltas;
        std::vector<Eigen::Vector3d> newPositions[grid_.getNNodes()];

        while(!stop)
        {
            nIters++;
            stop = true;
            totDeltas = 0;

            for(int e = 0; e < grid_.getNEdges(); e++)
            {                
                currEdge = grid_.getEdge(e);
                edgeLen = grid_.getEdgeLen(e);
                p1 = grid_.getNodePos(currEdge[0]);
                p2 = grid_.getNodePos(currEdge[1]);

                v = p1 - p2;
                dist = v.norm();
                v.normalize();

                if(abs(edgeLen - dist) > SIM_TOL)
                    stop = false;
                
                delta = (edgeLen - dist);
                totDeltas += abs(delta);

                if(fixedGridNodes_[currEdge[0]] || (picking_ && currEdge[0] == pickedNode_))
                    newPositions[currEdge[1]].push_back(p2 - delta * v);
                else if(fixedGridNodes_[currEdge[1]] || (picking_ && currEdge[1] == pickedNode_))
                    newPositions[currEdge[0]].push_back(p1 + delta * v);
                else
                {
                    newPositions[currEdge[0]].push_back(p1 + delta/2.0 * v);
                    newPositions[currEdge[1]].push_back(p2 - delta/2.0 * v);
                }
            }

            for(int n = 0; n < grid_.getNNodes(); n++)
            {
                if(!newPositions[n].empty())
                {
                    v = {0, 0, 0};
                    for(const Eigen::Vector3d &p : newPositions[n])
                        v += p;
                    v /= newPositions[n].size();
                    grid_.setNodePos(n, v);
                }
                newPositions[n].clear();
            }

            if(shapeConstraint_)
                for(int e = 0; e < grid_.getNEdges(); e++)
                {
                    currEdge = grid_.getEdge(e);
                    p1 = grid_.getNodePos(currEdge[0]);
                    p2 = grid_.getNodePos(currEdge[1]);

                    if(p1.squaredNorm() < 0.2)
                    {
                        stop = false;
                        while(p1.squaredNorm() < 0.2)
                            p1 += (p1 * SIM_TOL);
                    }
                    grid_.setNodePos(currEdge[0], p1);

                    if(p2.squaredNorm() < 0.2)
                    {
                        stop = false;
                        while(p2.squaredNorm() < 0.2)
                            p2 += (p2 * SIM_TOL);
                    }
                    grid_.setNodePos(currEdge[1], p2);
                }
        }

        pbdDeltas_.clear();       

        return nIters;
    }*/

    void cut()
    {
        const Eigen::Vector3d cutPlaneNormal = {0, 1, 0};
        
        std::vector<Eigen::Vector3d> nodes;
        std::unordered_map<int, int> nodeIndexMap;
        std::vector<std::pair<int, int>> edges;
        std::vector<double> edgeLengths;

        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            Eigen::Vector3d p = grid_.getNodePos(n);

            // only preserve nodes above the cutting plane
            if(p(1) < 0)
                continue;

            nodes.push_back(p);
            nodeIndexMap[n] = nodes.size() - 1;
        }
        
        for(int e = 0; e < grid_.getNEdges(); e++)
        {
            Eigen::Array2i edge = grid_.getEdge(e);
            auto p1IndexPair = nodeIndexMap.find(edge[0]);
            auto p2IndexPair = nodeIndexMap.find(edge[1]);

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
                edgeLength = grid_.getEdgeLen(e);
            }
            else
            {
                // edge crosses the cutting plane: retrieve the node below the cutting plane
                // and move it to the closest intersection between the edge and the plane

                Eigen::Vector3d edgeVec = (grid_.getNodePos(edge[0]) - grid_.getNodePos(edge[1])).normalized();
                Eigen::Vector3d newNode;

                if(p1IndexPair == nodeIndexMap.end())
                {
                    newNode = grid_.getNodePos(edge[0]);
                    p1Index = nodes.size();
                    p2Index = p2IndexPair->second;
                }
                else
                {
                    newNode = grid_.getNodePos(edge[1]);
                    p1Index = p1IndexPair->second;
                    p2Index = nodes.size();
                }

                double deltaLength = newNode.dot(cutPlaneNormal) / edgeVec.dot(cutPlaneNormal);

                newNode -= (deltaLength * edgeVec);
                edgeLength = grid_.getEdgeLen(e) - abs(deltaLength);
                nodes.push_back(newNode);
                fixedGridNodes_[nodes.size() - 1] = true;
            }

            edges.push_back({p1Index, p2Index});
            edgeLengths.push_back(edgeLength);
        }

        Eigen::Matrix3Xd nodeMatrix(3, nodes.size());
        for(int n = 0; n < nodes.size(); n++)
            nodeMatrix.col(n) = nodes[n];
        
        Eigen::Array2Xi edgeMatrix(2, edges.size());
        Eigen::ArrayXd edgeLengthsArray(edgeLengths.size());
        for(int e = 0; e < edges.size(); e++)
        {
            edgeMatrix.col(e) = Eigen::Array2i{edges[e].first, edges[e].second};
            edgeLengthsArray[e] = edgeLengths[e];
        }

        grid_ = Grid(nodeMatrix, edgeMatrix, edgeLengthsArray);
        shapeConstraint_ = false;
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
