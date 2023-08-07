#ifndef GRID_H_
#define GRID_H_

#include <vector>
#include <unordered_set>
#include <Eigen/Dense>


class EdgeLenConstr;

struct Grid
{
    Grid(Eigen::Matrix3Xd nodePos, std::vector<EdgeLenConstr> edgeLenConstraints, std::unordered_set<int> fixedNodes)
        : pos(nodePos),
          edgeLenCs(edgeLenConstraints),
          fixedNodes(fixedNodes)
    {}
    
    Grid(Eigen::Vector3d center, int nRows, int nCols,
         Eigen::Vector3d xTangVec, Eigen::Vector3d yTangVec, double edgeLength)
        : pos(3, nRows * nCols)
    {
        double width  = (double)(nCols -  1) * edgeLength;
        double height = (double)(nRows -  1) * edgeLength;
        xTangVec.normalize();
        yTangVec.normalize();
        Eigen::Vector3d gridOrigin = center - (width/2.0 * xTangVec) - (height/2.0 * yTangVec);

        edgeLenCs.reserve(2 * nRows * nCols - nRows - nCols);

        for(int n = 0; n < (nRows * nCols); n++)
        {
            // poor spatial locality? https://en.wikipedia.org/wiki/Z-order_curve
            int row = n / nCols;
            int col = n - (row * nCols);

            pos.col(n) = gridOrigin + (row * edgeLength) * yTangVec + (col * edgeLength) * xTangVec;

            if(col != nCols - 1)
                edgeLenCs.emplace_back(*this, n, n+1, edgeLength);
            if(row != nRows - 1)
                edgeLenCs.emplace_back(*this, n, n+nCols, edgeLength);
        }
    }

    int getNNodes() const { return pos.cols(); }

    Eigen::Block<Eigen::Matrix3Xd, 3, 1, true> nodePos(int idx) { return pos.col(idx); }
    bool isNodeFixed(int idx) const {
        auto it = fixedNodes.find(idx);
        return it != fixedNodes.end();
    }
    
    std::vector<EdgeLenConstr> edgeLenCs;
    Eigen::Matrix3Xd pos;
    std::unordered_set<int> fixedNodes;
};


#include "constraints.hpp"

#endif