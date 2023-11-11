#ifndef FORCES_HPP
#define FORCES_HPP

#include "framework/debug.hpp"

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


class DiscreteSDFAttractionForce : public UnaryForce
{
using Vec3d = openvdb::math::Vec3d;
using Coord = openvdb::Coord;
using Transform = openvdb::math::Transform;
using FloatGrid = openvdb::FloatGrid;
using FloatGradient = openvdb::tools::Gradient<FloatGrid>;

public:
    DiscreteSDFAttractionForce()
        : ready_(false)
    {}

    DiscreteSDFAttractionForce(std::string vdbPath)
        : ready_(false)
    {
        load(vdbPath);
    }

    DiscreteSDFAttractionForce(FloatGrid::Ptr sdfGrid)
        : ready_(false)
    {
        load(sdfGrid);
    }

    bool load(FloatGrid::Ptr sdfGrid)
    {
        openvdb::initialize();
        
        sdfGrid_ = sdfGrid;
        transform_ = sdfGrid->transformPtr();
        
        computeGradientGrid();

        ready_ = true;
        return true;
    }

    bool load(std::string vdbPath)
    {
        openvdb::initialize();
        
        openvdb::io::File vdbFile(vdbPath);
        try
        {
            vdbFile.open(false);
        }
        catch(std::exception& e)
        {
            frmwrk::Debug::logError("%s: %s", vdbPath.c_str(), e.what());
            return false;
        }
        if (vdbFile.getGrids()->empty())
        {
            frmwrk::Debug::logWarning("%s: no grid found");
            return false;
        }
        //openvdb::GridBase::Ptr grid = vdbFile.getGrids()->front();
        sdfGrid_ = openvdb::gridPtrCast<openvdb::FloatGrid>(vdbFile.getGrids()->front());
        vdbFile.close();

        transform_ = sdfGrid_->transformPtr();
        computeGradientGrid();

        ready_ = true;
        return true;
    }

    virtual Eigen::Vector3d F(Eigen::Vector3d pos) const
    {
        if(!ready_)
            return Eigen::Vector3d::Zero();

        FloatGradient::OutGridType::ConstAccessor gradGridAcc = gradGrid_->getConstAccessor();
        Coord c = transform_->worldToIndexCellCentered(Vec3d(pos.data()));
        return -Eigen::Vector3f(gradGridAcc.getValue(c).asPointer()).cast<double>() * std::min(5e-3f, distFromSurface(pos));
    }

    float distFromSurface(Eigen::Vector3d pos) const
    {
        FloatGrid::ConstAccessor sdfGridAcc = sdfGrid_->getConstAccessor();
        Coord c = transform_->worldToIndexCellCentered(Vec3d(pos.data()));
        return sdfGridAcc.getValue(c);
    }

private:
    void computeGradientGrid()
    {
        FloatGradient grad(*sdfGrid_);
        gradGrid_ = grad.process();
    }

    bool ready_;
    Transform::Ptr transform_;
    FloatGrid::Ptr sdfGrid_;
    FloatGradient::OutGridType::Ptr gradGrid_;
};


template <class MeshType>
class MeshAttractionForce : public UnaryForce
{
typedef typename MeshType::VertexType VertexType;
typedef typename MeshType::FaceType FaceType;
using Vec3s = openvdb::math::Vec3s;
using Vec3I = openvdb::Vec3I;
using Vec4I = openvdb::Vec4I;
using Transform = openvdb::math::Transform;
using FloatGrid = openvdb::FloatGrid;

static inline std::vector<Vec4I> emptyVecOfVec4I = std::vector<Vec4I>();

public:
    MeshAttractionForce()
        : ready_(false)
    {}

    MeshAttractionForce(MeshType& mesh, double voxelSize, float externalBandWidth, float internalBandWidth)
    {
        load(mesh);
    }

    bool load(MeshType& mesh, double voxelSize, float externalBandWidth, float internalBandWidth)
    {
        openvdb::initialize();
        
        meshVerts_.reserve(mesh.VN());
        for(const VertexType& v : mesh.vert)
            meshVerts_.emplace_back(v.cP().X(),
                                    v.cP().Y(),
                                    v.cP().Z());

        meshFaceIs_.reserve(mesh.FN());
        for(const FaceType& f : mesh.face)
            meshFaceIs_.emplace_back(vcg::tri::Index(mesh, f.cV(0)),
                                     vcg::tri::Index(mesh, f.cV(1)),
                                     vcg::tri::Index(mesh, f.cV(2)));

        Transform::Ptr transform = Transform::createLinearTransform(voxelSize);
        FloatGrid::Ptr sdfGrid = openvdb::tools::meshToSignedDistanceField<FloatGrid>(*transform,
                                                                                      meshVerts_,
                                                                                      meshFaceIs_,
                                                                                      emptyVecOfVec4I,
                                                                                      externalBandWidth,
                                                                                      internalBandWidth);
        openvdb::io::File vdbFile("../data/out.vdb");
        vdbFile.write({sdfGrid});
        vdbFile.close();

        sdfForce_.load(sdfGrid);
        
        ready_ = true;
        return true;
    }

    virtual Eigen::Vector3d F(Eigen::Vector3d pos) const
    {
        if(!ready_) return Eigen::Vector3d::Zero();
        return sdfForce_.F(pos);
    }

    float distFromSurface(Eigen::Vector3d pos) const
    {
        if(!ready_) return 0;
        return sdfForce_.distFromSurface(pos);
    }

private:
    bool ready_;
    std::vector<Vec3s> meshVerts_;
    std::vector<Vec3I> meshFaceIs_;
    DiscreteSDFAttractionForce sdfForce_;
};


#endif