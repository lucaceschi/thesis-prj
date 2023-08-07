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
        return (grid_.nodePos(nodeIdxs_.first) - grid_.nodePos(nodeIdxs_.second)).norm();
    }

    virtual double resolve() const
    {
        Eigen::Vector3d v = grid_.nodePos(nodeIdxs_.first) - grid_.nodePos(nodeIdxs_.second);
        double dist = v.norm();
        v.normalize();
        double delta = (len_ - dist);

        if(grid_.isNodeFixed(nodeIdxs_.first))
            grid_.nodePos(nodeIdxs_.second) -= delta * v;
        else if(grid_.isNodeFixed(nodeIdxs_.second))
            grid_.nodePos(nodeIdxs_.first) += delta * v;
        else
        {
            grid_.nodePos(nodeIdxs_.first) += delta/2.0 * v;
            grid_.nodePos(nodeIdxs_.second) -= delta/2.0 * v;
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
    SphereCollConstr(Grid& grid, Eigen::Vector3d centerPos, double radius)
        : grid_(grid),
          centerPos_(centerPos),
          radius_(radius)
    {}

    virtual double value() const
    {
        double currDelta, totValue = 0;
        
        for(int n = 0; n < grid_.getNNodes(); n++)
        {
            currDelta = radius_ - (grid_.nodePos(n) - centerPos_).squaredNorm(); 
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
            
            currDist = (grid_.nodePos(n) - centerPos_).norm();
            currDelta = radius_ - currDist; 
            if(currDelta > 0)
            {
                totValue += currDelta;
                
                if((grid_.nodePos(n) - centerPos_).isZero())
                    grid_.nodePos(n) += Eigen::Vector3d{0, radius_, 0};
                else
                    grid_.nodePos(n) = (grid_.nodePos(n) - centerPos_) * (radius_ / currDist) + centerPos_;
            }
        }

        return totValue;
    }
          

private:
    Grid& grid_;
    Eigen::Vector3d centerPos_;
    double radius_;
};


#endif