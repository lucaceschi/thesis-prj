#ifndef GRID_H_
#define GRID_H_

#include <vector>
#include <unordered_set>
#include <Eigen/Dense>


class EdgeLenConstr;

struct Grid
{
    Grid(Eigen::Matrix3Xd nodePos, Eigen::Array2Xi edges)
        : pos(nodePos),
          edges(edges)
    {}
    
    Grid(Eigen::Vector3d center, int nRows, int nCols,
         Eigen::Vector3d xTangVec, Eigen::Vector3d yTangVec, double edgeLength)
        : pos(3, nRows * nCols),
          edges(2, 2 * nRows * nCols - nRows - nCols)
    {
        double width  = (double)(nCols -  1) * edgeLength;
        double height = (double)(nRows -  1) * edgeLength;
        xTangVec.normalize();
        yTangVec.normalize();
        Eigen::Vector3d gridOrigin = center - (width/2.0 * xTangVec) - (height/2.0 * yTangVec);

        for(int n = 0, e = 0; n < (nRows * nCols); n++)
        {
            // poor spatial locality? https://en.wikipedia.org/wiki/Z-order_curve
            int row = n / nCols;
            int col = n - (row * nCols);

            pos.col(n) = gridOrigin + (row * edgeLength) * yTangVec + (col * edgeLength) * xTangVec;

            if(col != nCols - 1)
                edges.col(e++) = Eigen::Array2i{n, n+1};
            if(row != nRows - 1)
                edges.col(e++) = Eigen::Array2i{n, n+nCols};
        }
    }

    int getNNodes() const { return pos.cols(); }
    int getNEdges() const { return edges.cols(); }

    inline Eigen::Block<Eigen::Matrix3Xd, 3, 1, true> nodePos(int idx) { return pos.col(idx); }
    inline Eigen::Block<Eigen::Array2Xi, 2, 1, true> edge(int idx) { return edges.col(idx); }
    
    Eigen::Matrix3Xd pos;
    Eigen::Array2Xi edges;
};


#include "constraints.hpp"

#endif