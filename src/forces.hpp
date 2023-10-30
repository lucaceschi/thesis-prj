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


class SDFAttractionForce : public UnaryForce
{
public:
    using SDF = double(Eigen::Vector3d);
    using dSDF = Eigen::Vector3d(Eigen::Vector3d);

    SDFAttractionForce(SDF* sdf, dSDF* dsdf)
        : sdf_(sdf),
          dsdf_(dsdf)
    {}

    virtual Eigen::Vector3d F(Eigen::Vector3d pos) const
    {
        return -dsdf_(pos) * sdf_(pos);
    }

private:
    SDF* sdf_;
    dSDF* dsdf_;
};


class SphereAttractionForce : public SDFAttractionForce
{
public:
    SphereAttractionForce()
        : SDFAttractionForce(this->sdf, this->dsdf)
    {}

private:
    static double sdf(Eigen::Vector3d pos)
    {
        return pos.norm() - 0.5;
    }

    static Eigen::Vector3d dsdf(Eigen::Vector3d pos)
    {
        return pos.normalized();
    }
};

#endif