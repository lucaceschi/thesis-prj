#include "grid.hpp"
#include <iostream>


Grid::Grid(Eigen::Vector3d center,
           Eigen::Vector3d xTangVec, Eigen::Vector3d yTangVec,
           int nRows, int nCols,
           double edgeLength)
    : xTangVec_(xTangVec),
      yTangVec_(yTangVec),
      nRows_(nRows),
      nCols_(nCols),
      nodePos_(3, nRows * nCols),
      edges_(2, 2 * nRows * nCols - nRows - nCols),
      edgeLengths_(2 * nRows * nCols - nRows - nCols)
{
    double width  = (double)(nCols -  1) * edgeLength / 2.0;
    double height = (double)(nRows -  1) * edgeLength / 2.0;
    Eigen::Vector3d gridOrigin = center - (width * xTangVec) - (height * yTangVec);
    
    for(int n = 0, e = 0; n < getNNodes(); n++)
    {
        // poor spatial locality? https://en.wikipedia.org/wiki/Z-order_curve
        int row = n / nCols;
        int col = n - (row * nCols);

        nodePos_.col(n) = gridOrigin + (row * edgeLength) * xTangVec + (col * edgeLength) * yTangVec;

        if(col != nCols - 1)
            edges_.col(e++) = Eigen::Array2i{n, n+1};
        if(row != nRows - 1)
            edges_.col(e++) = Eigen::Array2i{n, n+nCols};
    }

    edgeLengths_.fill(edgeLength);
}

Grid::Grid(Eigen::Matrix3Xd nodePos, Eigen::Array2Xi edges, Eigen::ArrayXd edgeLengths)
    : nodePos_(nodePos),
      edges_(edges),
      edgeLengths_(edgeLengths)
{}

int Grid::getNNodes() const
{
    return nodePos_.cols();
}

Eigen::Vector3d Grid::getNodePos(int i) const
{
    return nodePos_.col(i);
}

Eigen::Vector3d Grid::getNodePos(int row, int col) const
{
    return getNodePos(row * nCols_ + col);
}

void Grid::setNodePos(int i, Eigen::Vector3d pos)
{
    nodePos_.col(i) = pos;
}

int Grid::getNEdges() const
{
    return edges_.cols();
}

Eigen::Array2i Grid::getEdge(int e) const
{
    return edges_.col(e);
}

double Grid::getEdgeLen(int e) const
{
    return edgeLengths_(e);
}