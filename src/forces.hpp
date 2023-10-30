#ifndef FORCES_HPP
#define FORCES_HPP

#include <Eigen/Dense>
#include <vcg/complex/allocate.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/GridOperators.h>


class UnaryForce
{
public:
    virtual Eigen::Vector3d F(Eigen::Vector3d pos) const = 0;
};


class ConstantForce : public UnaryForce
{
public:
    ConstantForce(Eigen::Vector3d f)
        : f_(f)
    {}

    virtual Eigen::Vector3d F(Eigen::Vector3d pos) const
    {
        return f_;
    }

private:
    Eigen::Vector3d f_;
};


#endif