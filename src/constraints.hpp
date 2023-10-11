#ifndef CONSTRAINTS_HPP_
#define CONSTRAINTS_HPP_

#include "grid.hpp"
#include "param_distance3.hpp"
#include "framework/debug.hpp"

#include <unordered_map>
#include <Eigen/Dense>

#define SMALL_NUM 1e-16


class HardConstraint
{
public:
    virtual double resolve(std::vector<Grid>& grids) const = 0;
};


class EdgeLenConstr : public HardConstraint
{
public:
    EdgeLenConstr(std::vector<Grid>& grids, int gridIdx, double length)
        : gridIdx_(gridIdx),
          lens_(grids[gridIdx].getNEdges(), length)
    {}

    EdgeLenConstr(std::vector<Grid>& grids, int gridIdx, std::vector<double> lengths)
        : gridIdx_(gridIdx),
          lens_(lengths)
    {}

    virtual double resolve(std::vector<Grid>& grids) const
    {
        Grid& g = grids[gridIdx_];
        double maxDelta = 0.0;
        
        for(int e = 0; e < g.getNEdges(); e++)
        {
            Eigen::Vector3d v = g.nodePos(g.edge(e)[0]) - g.nodePos(g.edge(e)[1]);
            double dist = v.norm();
            v.normalize();
            double delta = (lens_[e] - dist) / 2.0;

            g.nodePos(g.edge(e)[0]) += delta * v;
            g.nodePos(g.edge(e)[1]) -= delta * v;

            maxDelta = std::max(maxDelta, std::abs(delta));
        }

        return maxDelta;
    }

    double getLength(int e) const { return lens_[e]; }

private:
    int gridIdx_;
    std::vector<double> lens_;
};


class SphereCollConstr : public HardConstraint
{
public:
    SphereCollConstr(int gridIdx, Eigen::Vector3d centerPos, double radius)
        : gridIdx_(gridIdx),
          originPos_(centerPos),
          radius_(radius)
    {}

    void genCollisions(std::vector<Grid>& grids, std::vector<Eigen::Matrix3Xd> prevNodePos)
    {
        Grid& g = grids[gridIdx_];
        Eigen::Matrix3Xd prevPos = prevNodePos[gridIdx_];
        
        coll_.clear();

        for(int n = 0; n < g.getNNodes(); n++)
        {
            Eigen::Vector3d OP = g.nodePos(n) - originPos_;

            if(OP.squaredNorm() > std::pow(radius_, 2))
                continue; // the updated position of the particle is already out of the sphere

            Eigen::Vector3d V = g.nodePos(n) - prevPos.col(n);
            double segLength = V.norm();
            V /= segLength;
            Eigen::Vector3d EO = originPos_ - prevPos.col(n);
            double v = EO.dot(V);
            double disc = std::pow(radius_, 2) - EO.squaredNorm() + std::pow(v, 2);
            if(disc > 0)
            {
                double len = (v - std::sqrt(disc));
                if(len < 0)
                {
                    // static collision handling
                    Eigen::Vector3d normal = OP.normalized();
                    coll_.emplace_back(n, originPos_ + normal * radius_, normal);
                }
                else
                {
                    // continuous collision handling
                    Eigen::Vector3d entryPoint = prevPos.col(n) + len * V;
                    coll_.emplace_back(n, entryPoint, (entryPoint - originPos_).normalized());
                }
            }
        }
    }

    virtual double resolve(std::vector<Grid>& grids) const
    {
        Grid& g = grids[gridIdx_];
        int n;
        Eigen::Vector3d entryPoint, normal;
        double currDist, currDelta, maxDelta = 0;
        
        for(const std::tuple<int, Eigen::Vector3d, Eigen::Vector3d>& c : coll_)
        {
            std::tie(n, entryPoint, normal) = c;
            
            currDelta = (entryPoint - g.nodePos(n)).dot(normal);
            
            if(currDelta > 0)
            {
                g.nodePos(n) += (currDelta * normal);
                maxDelta = std::max(maxDelta, currDelta);
            }
        }

        return maxDelta;
    }

private:
    Eigen::Vector3d originPos_;
    int gridIdx_;
    double radius_;
    std::vector<std::tuple<int, Eigen::Vector3d, Eigen::Vector3d>> coll_;
};


class PlaneCollConstr : public HardConstraint
{
public:
    PlaneCollConstr(int gridIdx, Eigen::Vector3d origin, Eigen::Vector3d normal)
        : gridIdx_(gridIdx),
          origin_(origin),
          normal_(normal.normalized())
    {}

    virtual double resolve(std::vector<Grid>& grids) const
    {
        Grid& g = grids[gridIdx_];
        double currDelta, maxDelta = 0;

        for(int n = 0; n < g.getNNodes(); n++)
        {
            currDelta = (origin_ - g.nodePos(n)).dot(normal_);
            
            if(currDelta > 0)
            {
                g.nodePos(n) += (currDelta * normal_);
                maxDelta = std::max(maxDelta, currDelta);
            }
        }

        return maxDelta;
    }

private:
    int gridIdx_;
    Eigen::Vector3d origin_, normal_;
};


class FixedNodeConstr : public HardConstraint
{    
public:
    FixedNodeConstr(int gridIdx)
        : gridIdx_(gridIdx)
    {}

    void fixNode(int nodeIdx, Eigen::Vector3d pos)
    {
        fixedPos_[nodeIdx] = pos;
    }

    void fixNode(std::vector<Grid>& grids, int nodeIdx)
    {
        fixNode(nodeIdx, grids[gridIdx_].nodePos(nodeIdx));
    }

    void freeNode(int nodeIdx)
    {
        typedef std::unordered_map<int, Eigen::Vector3d>::iterator Iterator;
        Iterator it = fixedPos_.find(nodeIdx);
        if(it != fixedPos_.end())
            fixedPos_.erase(it);
    }

    bool isNodeFixed(int nodeIdx) const
    {
        return fixedPos_.find(nodeIdx) != fixedPos_.cend();
    }

    virtual double resolve(std::vector<Grid>& grids) const
    {
        typedef std::unordered_map<int, Eigen::Vector3d>::const_iterator CIterator;
        Grid& g = grids[gridIdx_];

        double maxDelta = 0;
        for(CIterator it = fixedPos_.cbegin(); it != fixedPos_.cend(); it++)
        {
            maxDelta = std::max(maxDelta, (g.nodePos(it->first) - it->second).squaredNorm());
            g.nodePos(it->first) = it->second;
        }

        return maxDelta;
    }

    typedef std::unordered_map<int, Eigen::Vector3d>::const_iterator const_iterator;
    const_iterator cbegin() const
    {
        return fixedPos_.cbegin();
    }
    const_iterator cend() const 
    {
        return fixedPos_.cend();
    }

private:
    int gridIdx_;
    std::unordered_map<int, Eigen::Vector3d> fixedPos_;
};


class ScissorConstr : public HardConstraint
{
public:
    ScissorConstr(std::vector<Grid>& grids, int gridAIdx, int edgeAIdx, int gridBIdx, int edgeBIdx)
        : gridAIdx_(gridAIdx),
          edgeAIdx_(edgeAIdx),
          gridBIdx_(gridBIdx),
          edgeBIdx_(edgeBIdx)
    {
        Grid& gridA = grids[gridAIdx];
        Grid& gridB = grids[gridBIdx];
        
        vcg::Segment3d segA = vcg::Segment3d(
            vcg::Point3d(gridA.nodePos(gridA.edge(edgeAIdx)[0]).data()),
            vcg::Point3d(gridA.nodePos(gridA.edge(edgeAIdx)[1]).data())
        );

        vcg::Segment3d segB = vcg::Segment3d(
            vcg::Point3d(gridB.nodePos(gridB.edge(edgeBIdx)[0]).data()),
            vcg::Point3d(gridB.nodePos(gridB.edge(edgeBIdx)[1]).data())
        );

        bool parallel;
        vcg::Point3d midpointA, midpointB;
        vcg::SegmentSegmentDistancePar(segA, segB, dist_, parallel, alpha_, beta_, midpointA, midpointB);
        omegaA_ = 1.0 / (std::pow(alpha_, 2) + std::pow(1 - alpha_, 2));
        omegaB_ = 1.0 / (std::pow(beta_,  2) + std::pow(1 - beta_,  2));

        if(parallel)
            frmwrk::Debug::logWarning("Found parallel edges while adding a new scissor constraint");
    }

    virtual double resolve(std::vector<Grid>& grids) const
    {
        Grid& gridA = grids[gridAIdx_];
        Grid& gridB = grids[gridBIdx_];
        
        Eigen::Vector3d shiftDir = getCrossPointB(grids) - getCrossPointA(grids);
        double delta = shiftDir.norm(); //(shiftDir.norm() - dist_);
        double deltaA = delta * omegaB_ / (omegaA_ + omegaB_);
        double deltaB = delta * omegaA_ / (omegaA_ + omegaB_);
        double deltaA0 = deltaA * (1 - alpha_) * omegaA_;
        double deltaA1 = deltaA * (alpha_)     * omegaA_;
        double deltaB0 = deltaB * (1 - beta_)  * omegaB_;
        double deltaB1 = deltaB * (beta_)      * omegaB_;
        shiftDir.normalize();

        gridA.nodePos(gridA.edge(edgeAIdx_)[0]) += deltaA0 * shiftDir;
        gridA.nodePos(gridA.edge(edgeAIdx_)[1]) += deltaA1 * shiftDir;
        gridB.nodePos(gridB.edge(edgeBIdx_)[0]) -= deltaB0 * shiftDir;
        gridB.nodePos(gridB.edge(edgeBIdx_)[1]) -= deltaB1 * shiftDir;

        return std::max({std::abs(deltaA0), std::abs(deltaA1), std::abs(deltaB0), std::abs(deltaB1)});
    }

    int getGridAIdx()  const { return gridAIdx_; }
    int getGridBIdx()  const { return gridBIdx_; }
    int getEdgeAIdx() const { return edgeAIdx_; }
    int getEdgeBIdx() const { return edgeBIdx_; }
    double getDist() const { return dist_; }
    double getAlpha() const { return alpha_; }
    double getBeta() const { return beta_; }

    inline Eigen::Vector3d getCrossPointA(std::vector<Grid>& grids) const
    {
        Grid& gridA = grids[gridAIdx_];
        Eigen::Vector3d n0 = gridA.nodePos(gridA.edge(edgeAIdx_)[0]);
        Eigen::Vector3d n1 = gridA.nodePos(gridA.edge(edgeAIdx_)[1]);
        return n0 * (1 - alpha_) + n1 * alpha_;
    }

    inline Eigen::Vector3d getCrossPointB(std::vector<Grid>& grids) const
    {
        Grid& gridB = grids[gridBIdx_];
        Eigen::Vector3d n0 = gridB.nodePos(gridB.edge(edgeBIdx_)[0]);
        Eigen::Vector3d n1 = gridB.nodePos(gridB.edge(edgeBIdx_)[1]);
        return n0 * (1 - beta_) + n1 * beta_;
    }

private:
    int gridAIdx_, edgeAIdx_;
    double alpha_, omegaA_;
    int gridBIdx_, edgeBIdx_;
    double beta_, omegaB_;
    double dist_;
};


#endif