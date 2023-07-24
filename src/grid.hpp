#ifndef GRID_H_
#define GRID_H_

#include <Eigen/Dense>


class Grid
{
public:
    Grid(Eigen::Vector3d center,
         Eigen::Vector3d xTangVec, Eigen::Vector3d yTangVec,
         int nRows, int nCols,
         double edgeLength);

    Grid(Eigen::Matrix3Xd nodePos, Eigen::Array2Xi edges, Eigen::ArrayXd edgeLengths);

    int getNNodes() const;
    Eigen::Vector3d getNodePos(int i) const;
    Eigen::Vector3d getNodePos(int row, int col) const;

    int getNEdges() const;
    Eigen::Array2i getEdge(int e) const;
    double getEdgeLen(int e) const;

    void setNodePos(int i, Eigen::Vector3d pos);

private:
    Eigen::Vector3d xTangVec_;
    Eigen::Vector3d yTangVec_;
    int nRows_;
    int nCols_;
    Eigen::Matrix3Xd nodePos_;
    Eigen::Array2Xi edges_;
    Eigen::ArrayXd edgeLengths_;

};


#endif