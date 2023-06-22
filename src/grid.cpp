#include "grid.hpp"
#include <iostream>

Grid::Grid(Eigen::Vector3d xTangVec, Eigen::Vector3d yTangVec, int nRows, int nCols, float edgeLength)
    : xTangVec_(xTangVec),
      yTangVec_(yTangVec),
      nRows_(nRows),
      nCols_(nCols),
      edgeLength_(edgeLength),
      nodePos_(3, nRows * nCols),
      edges_(2, 2 * nRows * nCols - nRows - nCols)
{   
    for(int n = 0, e = 0; n < getNNodes(); n++)
    {
        // poor spatial locality? https://en.wikipedia.org/wiki/Z-order_curve
        int row = n / nCols;
        int col = n - (row * nCols);

        nodePos_.col(n) = (row * edgeLength) * xTangVec + (col * edgeLength) * yTangVec;

        if(col != nCols - 1)
            edges_.col(e++) = Eigen::Array2i{n, n+1};
        if(row != nRows - 1)
            edges_.col(e++) = Eigen::Array2i{n, n+nCols};
    }
}

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