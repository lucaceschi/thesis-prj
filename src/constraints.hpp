#ifndef CONSTRAINTS_HPP_
#define CONSTRAINTS_HPP_

#include "grid.hpp"

#include <Eigen/Dense>

#define SMALL_NUM 1e-12


class HardConstraint
{
public:
    virtual double value() const = 0;
    virtual double resolve() const = 0;
};


class EdgeLenConstr : public HardConstraint
{
public:
    EdgeLenConstr(Grid* grid, double length)
        : grid_(grid),
          lens_(grid->getNEdges(), length)
    {}

    EdgeLenConstr(Grid* grid, std::vector<double> lengths)
        : grid_(grid),
          lens_(lengths)
    {}

    virtual double value() const
    {
        double v = 0.0;
        for(int e = 0; e < grid_->getNEdges(); e++)
            v += (grid_->nodePos(grid_->edge(e)[0]) - grid_->nodePos(grid_->edge(e)[1])).norm();

        return v;
    }

    virtual double resolve() const
    {
        double totDelta = 0.0;
        
        for(int e = 0; e < grid_->getNEdges(); e++)
        {
            Eigen::Vector3d v = grid_->nodePos(grid_->edge(e)[0]) - grid_->nodePos(grid_->edge(e)[1]);
            double dist = v.norm();
            v.normalize();
            double delta = (lens_[e] - dist);

            if(grid_->isNodeFixed(grid_->edge(e)[0]))
                grid_->nodePos(grid_->edge(e)[1]) -= delta * v;
            else if(grid_->isNodeFixed(grid_->edge(e)[1]))
                grid_->nodePos(grid_->edge(e)[0]) += delta * v;
            else
            {
                grid_->nodePos(grid_->edge(e)[0]) += delta/2.0 * v;
                grid_->nodePos(grid_->edge(e)[1]) -= delta/2.0 * v;
            }

            totDelta += std::abs(delta);
        }

        return totDelta;
    }

    double getLength(int e) const { return lens_[e]; }

private:
    Grid* grid_;
    std::vector<double> lens_;
};


class SphereCollConstr : public HardConstraint
{
public:
    SphereCollConstr(Grid* grid, Eigen::Vector3d centerPos, double radius)
        : grid_(grid),
          centerPos_(centerPos),
          radius_(radius)
    {}

    virtual double value() const
    {
        double currDelta, totValue = 0;
        
        for(int n = 0; n < grid_->getNNodes(); n++)
        {
            currDelta = radius_ - (grid_->nodePos(n) - centerPos_).squaredNorm(); 
            if(currDelta > 0)
                totValue += currDelta;
        }

        return totValue;
    }

    virtual double resolve() const
    {
        double currDist, currDelta, totValue = 0;
        
        for(int n = 0; n < grid_->getNNodes(); n++)
        {
            if(grid_->isNodeFixed(n))
                continue;
            
            currDist = (grid_->nodePos(n) - centerPos_).norm();
            currDelta = radius_ - currDist; 
            if(currDelta > 0)
            {
                totValue += currDelta;
                
                if((grid_->nodePos(n) - centerPos_).isZero())
                    grid_->nodePos(n) += Eigen::Vector3d{0, radius_, 0};
                else
                    grid_->nodePos(n) = (grid_->nodePos(n) - centerPos_) * (radius_ / currDist) + centerPos_;
            }
        }

        return totValue;
    }

    Eigen::Vector3d centerPos_;
private:
    Grid* grid_;
    double radius_;
};


class ScissorConstr : public HardConstraint
{
public:
    ScissorConstr(Grid* gridA, int nodeA1Indx, int nodeA2Indx,
                  Grid* gridB, int nodeB1Indx, int nodeB2Indx)
        : gridA_(gridA),
          nodeA1_(nodeA1Indx),
          nodeA2_(nodeA2Indx),
          gridB_(gridB),
          nodeB1_(nodeB1Indx),
          nodeB2_(nodeB2Indx)
    {
        Eigen::Vector3d u = gridA->nodePos(nodeA2Indx) - gridA->nodePos(nodeA1Indx);
        Eigen::Vector3d v = gridB->nodePos(nodeB2Indx) - gridB->nodePos(nodeB1Indx);
        Eigen::Vector3d w = gridA->nodePos(nodeA1Indx) - gridB->nodePos(nodeB1Indx);
        double a = u.dot(u);
        double b = u.dot(v);
        double c = v.dot(v);
        double d = u.dot(w);
        double e = v.dot(w);
        double D = a*c - b*b;
        double sc, sN, sD = D;
        double tc, tN, tD = D;

        if (D < SMALL_NUM) {
            sN = 0.0;
            sD = 1.0;
            tN = e;
            tD = c;
        }
        else {
            sN = (b*e - c*d);
            tN = (a*e - b*d);
            if (sN < 0.0) {
                sN = 0.0;
                tN = e;
                tD = c;
            }
            else if (sN > sD) {
                sN = sD;
                tN = e + b;
                tD = c;
            }
        }

        if (tN < 0.0) {
            tN = 0.0;
 
            if (-d < 0.0)
                sN = 0.0;
            else if (-d > a)
                sN = sD;
            else {
                sN = -d;
                sD = a;
            }
        }
        else if (tN > tD) {
            tN = tD;
 
            if ((-d + b) < 0.0)
                sN = 0;
            else if ((-d + b) > a)
                sN = sD;
            else {
                sN = (-d +  b);
                sD = a;
            }
        }
 
        alpha_ = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
        beta_ = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);
        dist_ = (w + (alpha_ * u) - (beta_ * v)).norm();
    }

    virtual double value() const
    {
        return (getMidpointA() - getMidpointB()).norm();
    }

    virtual double resolve() const
    {
        Eigen::Vector3d midpointA = getMidpointA();
        Eigen::Vector3d midpointB = getMidpointB();

        Eigen::Vector3d shiftDir = midpointB - midpointA;
        double delta = (shiftDir.norm() - dist_);
        shiftDir.normalize();

        gridA_->nodePos(nodeA1_) += delta/2 * shiftDir;
        gridA_->nodePos(nodeA2_) += delta/2 * shiftDir;
        gridB_->nodePos(nodeB1_) -= delta/2 * shiftDir;
        gridB_->nodePos(nodeB2_) -= delta/2 * shiftDir;

        return delta * 2;
    }

    Grid* getGridA() const { return gridA_; }
    Grid* getGridB() const { return gridB_; }
    int getNodeA1Indx() const { return nodeA1_; }
    int getNodeA2Indx() const { return nodeA2_; }
    int getNodeB1Indx() const { return nodeB1_; }
    int getNodeB2Indx() const { return nodeB2_; }
    std::pair<int, int> getAlphaBeta() const { return std::make_pair(alpha_, beta_); }
    double getDist() const { return dist_; }

    inline Eigen::Vector3d getMidpointA() const
    {
        return gridA_->nodePos(nodeA1_) + alpha_ * (gridA_->nodePos(nodeA2_) - gridA_->nodePos(nodeA1_));
    }

    inline Eigen::Vector3d getMidpointB() const
    {
        return gridB_->nodePos(nodeB1_) + beta_  * (gridB_->nodePos(nodeB2_) - gridB_->nodePos(nodeB1_));
    }

private:
    Grid* gridA_;
    int nodeA1_, nodeA2_;
    double alpha_;
    Grid* gridB_;
    int nodeB1_, nodeB2_;
    double beta_;
    double dist_;
};


#endif