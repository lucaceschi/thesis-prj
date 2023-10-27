#ifndef GRID_H_
#define GRID_H_

#include <vector>
#include <unordered_set>
#include <fstream>
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
          edges(2, 2 * nRows * nCols - nRows - nCols),
          diags(2, 2 * (nCols - 1) * (nRows - 1))
    {
        double width  = (double)(nCols -  1) * edgeLength;
        double height = (double)(nRows -  1) * edgeLength;
        xTangVec.normalize();
        yTangVec.normalize();
        Eigen::Vector3d gridOrigin = center - (width/2.0 * xTangVec) - (height/2.0 * yTangVec);

        for(int n = 0, e = 0, d = 0; n < (nRows * nCols); n++)
        {
            // poor spatial locality? https://en.wikipedia.org/wiki/Z-order_curve
            int row = n / nCols;
            int col = n - (row * nCols);

            pos.col(n) = gridOrigin + (row * edgeLength) * yTangVec + (col * edgeLength) * xTangVec;

            if(col != nCols - 1)
                edges.col(e++) = Eigen::Array2i{n, n+1};
            if(row != nRows - 1)
                edges.col(e++) = Eigen::Array2i{n, n+nCols};

            if(row != nRows - 1 && col != nCols - 1)
                diags.col(d++) = Eigen::Array2i(n, n+nCols+1);
            if(row != 0 && col != nCols - 1)
                diags.col(d++) = Eigen::Array2i(n, n-nCols+1);
        }
    }

    int getNNodes() const { return pos.cols(); }
    int getNEdges() const { return edges.cols(); }
    int getNDiags() const { return diags.cols(); }

    inline Eigen::Block<Eigen::Matrix3Xd, 3, 1, true> nodePos(int idx) { return pos.col(idx); }
    inline Eigen::Block<Eigen::Array2Xi, 2, 1, true> edge(int idx) { return edges.col(idx); }
    inline Eigen::Block<Eigen::Array2Xi, 2, 1, true> diag(int idx) { return diags.col(idx); }

    void exportPly(std::string filename) const
    {
        Eigen::IOFormat eigenFmt(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ");
        std::ofstream f(filename, std::ios::out | std::ios::trunc);

        // Header
        f << "ply" << std::endl << "format ascii 1.0" << std::endl;
        f << "element vertex " << getNNodes() << std::endl;
        f << "property double x" << std::endl;
        f << "property double y" << std::endl;
        f << "property double z" << std::endl;
        f << "element edge " << getNEdges() << std::endl;
        f << "property int vertex1" << std::endl;
        f << "property int vertex2" << std::endl;
        f << "end_header" << std::endl;

        // Vertex list
        for(int n = 0; n < getNNodes(); n++)
            f << pos.col(n).format(eigenFmt) << std::endl;

        // Edge list
        for(int e = 0; e < getNEdges(); e++)
            f << edges.col(e).format(eigenFmt) << std::endl;

        f.close();
    }
    
    Eigen::Matrix3Xd pos;
    Eigen::Array2Xi edges;
    Eigen::Array2Xi diags;
};


#include "constraints.hpp"

#endif