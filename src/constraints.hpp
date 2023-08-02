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
    EdgeLenConstr(Eigen::Matrix3Xd& pos, int nodeAIdx, int nodeBIdx, double length, int& pickedNode)
        : pos_(pos),
          nodeIdxs_(nodeAIdx, nodeBIdx),
          len_(length),
          pickedNode_(pickedNode)
    {}

    virtual double value() const
    {
        return (pos_.col(nodeIdxs_.first) - pos_.col(nodeIdxs_.second)).norm();
    }

    virtual double resolve() const
    {
        Eigen::Vector3d v = pos_.col(nodeIdxs_.first) - pos_.col(nodeIdxs_.second);
        double dist = v.norm();
        v.normalize();
        double delta = (len_ - dist);

        if(nodeIdxs_.first == pickedNode_)
            pos_.col(nodeIdxs_.second) -= delta * v;
        else if(nodeIdxs_.second == pickedNode_)
            pos_.col(nodeIdxs_.first) += delta * v;
        else
        {
            pos_.col(nodeIdxs_.first) += delta/2.0 * v;
            pos_.col(nodeIdxs_.second) -= delta/2.0 * v;
        }

        return std::abs(delta);
    }

    std::pair<int, int> getNodeIdxs() { return nodeIdxs_; }

private:
    Eigen::Matrix3Xd& pos_;
    std::pair<int, int> nodeIdxs_;
    double len_;
    int& pickedNode_;
};


class SphereCollConstr : public HardConstraint
{
public:
    SphereCollConstr(Eigen::Matrix3Xd& pos, double radius, int& pickedNode)
        : pos_(pos),
          radius_(radius),
          pickedNode_(pickedNode)
    {}

    virtual double value() const
    {
        double currDelta, totValue = 0;
        
        for(int n = 0; n < pos_.cols(); n++)
        {
            currDelta = radius_ - pos_.col(n).squaredNorm(); 
            if(currDelta > 0)
                totValue += currDelta;
        }

        return totValue;
    }

    virtual double resolve() const
    {
        double currDist, currDelta, totValue = 0;
        
        for(int n = 0; n < pos_.cols(); n++)
        {
            if(n == pickedNode_)
                continue;
            
            currDist = pos_.col(n).squaredNorm();
            currDelta = radius_ - currDist; 
            if(currDelta > 0)
            {
                totValue += currDelta;
                
                if(pos_.col(n).isZero())
                    pos_.col(n) = Eigen::Vector3d{0, radius_, 0};
                else
                    pos_.col(n) *= (radius_ / currDist);
            }
        }

        return totValue;
    }
          

private:
    Eigen::Matrix3Xd& pos_;
    double radius_;
    int& pickedNode_;
};