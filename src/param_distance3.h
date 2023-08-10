#include <limits>

#include <vcg/space/point3.h>
#include <vcg/space/segment3.h>
#include <vcg/space/line3.h>
#include <vcg/space/distance3.h>


namespace vcg
{

    template <class ScalarType> 
    void SegmentPointSquaredDistancePar(const Segment3<ScalarType>& s,
                                        const Point3<ScalarType>& p,
                                        ScalarType& t,
                                        ScalarType& sqr_dist) 
    {
        Point3<ScalarType> e = s.P1() - s.P0();
        ScalarType eSquaredNorm = e.SquaredNorm();
        if (eSquaredNorm < std::numeric_limits<ScalarType>::min())
        {
            t = 0.5;
            Point3<ScalarType> closest = s.MidPoint();
            sqr_dist = SquaredDistance(closest, p);
        }
        else
        {
            t = ((p - s.P0()) * e) / eSquaredNorm;
            if(t < 0)      t = 0;
            else if(t > 1) t = 1;

            Point3<ScalarType> closest = s.P0() * (1.0 - t) + s.P1() * t;
            sqr_dist = SquaredDistance(p,closest);
            assert(!math::IsNAN(sqr_dist));
        }
    }

    template <class ScalarType>
    void SegmentSegmentDistancePar(const vcg::Segment3<ScalarType>& s0,
                                   const vcg::Segment3<ScalarType>& s1,
                                   ScalarType& dist,
                                   bool& parallel,
                                   ScalarType& alpha,
                                   ScalarType& beta,
                                   vcg::Point3<ScalarType>& closest0,
                                   vcg::Point3<ScalarType>& closest1)

    {
        typedef typename vcg::Point3<ScalarType> CoordType;

        vcg::Line3<ScalarType> l0,l1;

        ///construct two lines
        l0.SetOrigin(s0.P0());
        l0.SetDirection((s0.P1()-s0.P0()).Normalize());

        l1.SetOrigin(s1.P0());
        l1.SetDirection((s1.P1()-s1.P0()).Normalize());

        ///then find closest point
        ScalarType line_dist;
        CoordType closest_test0,closest_test1;
        LineLineDistance(l0,l1,parallel,line_dist,closest_test0,closest_test1);
        ///special case if the two lines are parallel
        if (parallel)
        {
            ///find the minimum distance between extremes to segments
            ScalarType dist_test;
            ScalarType clos_test;

            ///find combination of distances between all extremes and segments
            SegmentPointSquaredDistancePar(s0,s1.P0(),clos_test,dist);
            alpha = clos_test;
            beta = 0.0;
            ///and find the minimum updating coherently the closest points
            SegmentPointSquaredDistancePar(s0,s1.P1(),clos_test,dist_test);
            if (dist_test<dist)
            {
                dist=dist_test;
                alpha=clos_test;
                beta=1.0;
            }
            SegmentPointSquaredDistancePar(s1,s0.P0(),clos_test,dist_test);
            if (dist_test<dist)
            {
                dist=dist_test;
                alpha=0.0;
                beta=clos_test;
            }
            SegmentPointSquaredDistancePar(s1,s0.P1(),clos_test,dist_test);
            if (dist_test<dist)
            {
                dist=dist_test;
                alpha=1.0;
                beta=clos_test;
            }
            dist=sqrt(dist);
            return;
        }

        ///then ffind the closest segments points... 
        ///means that if it is not an extreme then take 
        ///the closest extreme
        ScalarType sqr_dist0;
        SegmentPointSquaredDistancePar(s0,closest_test0,alpha,sqr_dist0);
        ScalarType sqr_dist1;
        SegmentPointSquaredDistancePar(s1,closest_test1,beta,sqr_dist1);

        ///then return the distance
        closest0 = s0.P0() * (1 - alpha) + s0.P1() * alpha;
        closest1 = s1.P0() * (1 - beta)  + s1.P1() * beta;
        dist=(closest0-closest1).Norm();
    }
}