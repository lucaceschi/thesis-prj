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


template <class MeshType>
class MeshAttractionForce : public UnaryForce
{
typedef typename MeshType::VertexType VertexType;
typedef typename MeshType::FaceType FaceType;
using Vec3d = openvdb::math::Vec3d;
using Vec3s = openvdb::math::Vec3s;
using Vec3I = openvdb::Vec3I;
using Vec4I = openvdb::Vec4I;
using Coord = openvdb::Coord;
using Transform = openvdb::math::Transform;
using FloatGrid = openvdb::FloatGrid;
using FloatCpt = openvdb::tools::Cpt<FloatGrid>;

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

        transform_ = Transform::createLinearTransform(voxelSize);

        FloatGrid::Ptr sdfGrid;
        sdfGrid = openvdb::tools::meshToSignedDistanceField<FloatGrid>(*transform_,
                                                                       meshVerts_,
                                                                       meshFaceIs_,
                                                                       emptyVecOfVec4I,
                                                                       externalBandWidth,
                                                                       internalBandWidth);

        FloatCpt cpt(*sdfGrid);
        cptGrid_ = cpt.process();
        
        ready_ = true;
        return true;
    }

    virtual Eigen::Vector3d F(Eigen::Vector3d pos) const
    {
        if(!ready_)
            return Eigen::Vector3d::Zero();

        FloatCpt::OutGridType::ConstAccessor cptGridAcc_ = cptGrid_->getConstAccessor();
        Coord c = transform_->worldToIndexCellCentered(Vec3d(pos.data()));
        Eigen::Vector3f closestPoint(cptGridAcc_.getValue(c).asPointer());        

        return closestPoint.cast<double>() - pos;
    }

private:
    bool ready_;
    std::vector<Vec3s> meshVerts_;
    std::vector<Vec3I> meshFaceIs_;

    Transform::Ptr transform_;
    FloatCpt::OutGridType::Ptr cptGrid_;
};


#endif