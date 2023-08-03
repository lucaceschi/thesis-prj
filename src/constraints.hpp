#ifndef CONSTRAINTS_HPP_
#define CONSTRAINTS_HPP_

#include "grid.hpp"

#include <Eigen/Dense>


class HardConstraint
{
public:
    virtual double value() const = 0;
    virtual double resolve() const = 0;
};


class EdgeLenConstr : public HardConstraint
{
public:
    EdgeLenConstr(Grid& grid, int nodeAIdx, int nodeBIdx, double length)
        : grid_(grid),
          nodeIdxs_(nodeAIdx, nodeBIdx),
          len_(length)
    {}

    virtual double value() const
    {
        return (grid_.getNodePos(nodeIdxs_.first) - grid_.getNodePos(nodeIdxs_.second)).norm();
    }

    virtual double resolve() const
    {
        Eigen::Vector3d v = grid_.getNodePos(nodeIdxs_.first) - grid_.getNodePos(nodeIdxs_.second);
        double dist = v.norm();
        v.normalize();
        double delta = (len_ - dist);

        if(grid_.isNodeFixed(nodeIdxs_.first))
            grid_.getNodePos(nodeIdxs_.second) -= delta * v;
        else if(grid_.isNodeFixed(nodeIdxs_.second))
            grid_.getNodePos(nodeIdxs_.first) += delta * v;
        else
        {
            grid_.getNodePos(nodeIdxs_.first) += delta/2.0 * v;
            grid_.getNodePos(nodeIdxs_.second) -= delta/2.0 * v;
        }

        return std::abs(delta);
    }

    std::pair<int, int> getNodeIdxs() const { return nodeIdxs_; }
    double getLength() const { return len_; }

private:
    Grid& grid_;
    std::pair<int, int> nodeIdxs_;
    double len_;
};


class SphereCollConstr : public HardConstraint
{
public:
    SphereCollConstr(Grid& grid, double radius)
        : grid_(grid),
          radius_(radius)
    {}

    virtual double value() const
    {
        double currDelta, totValue = 0;
        
        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            currDelta = radius_ - grid_.getNodePos(n).squaredNorm(); 
            if(currDelta > 0)
                totValue += currDelta;
        }

        return totValue;
    }

    virtual double resolve() const
    {
        double currDist, currDelta, totValue = 0;
        
        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            if(grid_.isNodeFixed(n))
                continue;
            
            currDist = grid_.getNodePos(n).norm();
            currDelta = radius_ - currDist; 
            if(currDelta > 0)
            {
                totValue += currDelta;
                
                if(grid_.getNodePos(n).isZero())
                    grid_.getNodePos(n) = Eigen::Vector3d{0, radius_, 0};
                else
                    grid_.getNodePos(n) *= (radius_ / currDist);
            }
        }

        return totValue;
    }
          

private:
    Grid& grid_;
    double radius_;
};


#endif