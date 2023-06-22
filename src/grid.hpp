#ifndef GRID_H_
#define GRID_H_

#include <Eigen/Dense>


class Grid
{
public:
    Grid(Eigen::Vector3d xTangVec, Eigen::Vector3d yTangVec, int width, int height, float edgeLength);

    int getNNodes() const;
    Eigen::Vector3d getNodePos(int i) const;
    Eigen::Vector3d getNodePos(int row, int col) const;

    int getNEdges() const;
    Eigen::Array2i getEdge(int e) const;

    void setNodePos(int i, Eigen::Vector3d pos);

private:
    Eigen::Vector3d xTangVec_;
    Eigen::Vector3d yTangVec_;
    int nRows_;
    int nCols_;
    float edgeLength_;
    Eigen::Matrix3Xd nodePos_;
    Eigen::Array2Xi edges_;

};


#endif