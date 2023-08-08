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
            v += (grid_->getNodePos(grid_->edge(e)[0]) - grid_->getNodePos(grid_->edge(e)[1])).norm();

        return v;
    }

    virtual double resolve() const
    {
        double totDelta = 0.0;
        
        for(int e = 0; e < grid_->getNEdges(); e++)
        {
            Eigen::Vector3d v = grid_->getNodePos(grid_->edge(e)[0]) - grid_->getNodePos(grid_->edge(e)[1]);
            double dist = v.norm();
            v.normalize();
            double delta = (lens_[e] - dist);

            if(grid_->isNodeFixed(grid_->edge(e)[0]))
                grid_->getNodePos(grid_->edge(e)[1]) -= delta * v;
            else if(grid_->isNodeFixed(grid_->edge(e)[1]))
                grid_->getNodePos(grid_->edge(e)[0]) += delta * v;
            else
            {
                grid_->getNodePos(grid_->edge(e)[0]) += delta/2.0 * v;
                grid_->getNodePos(grid_->edge(e)[1]) -= delta/2.0 * v;
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
          radius_(radius)
    {}

    virtual double value() const
    {
        double currDelta, totValue = 0;
        
        for(int n = 0; n < grid_->getNNodes(); n++)
        {
            currDelta = radius_ - (grid_->getNodePos(n) - centerPos_).squaredNorm(); 
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
            
            currDist = (grid_->getNodePos(n) - centerPos_).norm();
            currDelta = radius_ - currDist; 
            if(currDelta > 0)
            {
                totValue += currDelta;
                
                if((grid_->getNodePos(n) - centerPos_).isZero())
                    grid_->getNodePos(n) += Eigen::Vector3d{0, radius_, 0};
                else
                    grid_->getNodePos(n) = (grid_->getNodePos(n) - centerPos_) * (radius_ / currDist) + centerPos_;
            }
        }

        return totValue;
    }
          

    Eigen::Vector3d centerPos_;
private:
    Grid* grid_;
    double radius_;
};


#endif