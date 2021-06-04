#pragma once

#include "AStarOnVorDiag.h"
typedef Vertex Vector3D;
#include "collision detection.hpp"

//----------------------------------------------------------------------------
class BezierCurve3D
{
public:
    BezierCurve3D(
        const Vertex& a, 
        const Vertex& b,
        const Vertex& c,
        const Vertex& d);


    BezierCurve3D(
        const Vertex& p0,
        const Vector3D& n0,
        const Vector3D& d0,
        const Vertex& p1,
        const Vector3D& n1,
        const Vector3D& d1);

    static BezierCurve3D make_flipped(const BezierCurve3D& other)
    {
        return BezierCurve3D(other.d, other.c, other.b, other.a);
    }

    void _get_quadric_bezier_ctrl_pts(Vertex& a_tag, 
                                      Vertex& b_tag, 
                                      Vertex& c_tag) const
    {
        a_tag = (b - a) * 3.;
        b_tag = (c - b) * 3.;
        c_tag = (d - c) * 3.;
    }

    Vector3D der(double t) const;
    Vector3D second_der(double t) const;
    Vector3D norm(double t) const;
    Vertex eval(double t) const;

private:
    Vertex a, b, c, d;
};

//----------------------------------------------------------------------------
class PntTng
{
public:
    PntTng(const Vertex& pt = Vertex(), const Vector3D& nr = Vector3D());
    void setNormal(const Vector3D& n) { nr = n; }
    double getDistanceTo(const PntTng& other)  const { return pt.dist(other.pt); }
    void setTangents(const PntTng& prev_pt, const PntTng& next_pt);
    void setNaiveNorm(const PntTng& prev_pt, const PntTng& next_pt);
    void setEndpointNormAndTang(const PntTng& other, bool b_other_is_next);

public:
    Vertex pt;
    Vector3D nr;
    Vector3D right_tg;
    Vector3D left_tg;
};

//----------------------------------------------------------------------------
class BezierQuasiAverage
{
public:
    BezierQuasiAverage(vector<cd::pointCollisionTester>& collisionTesters) :
        collisionTesters_(collisionTesters) {}
    PntTng operator() (double t0, const PntTng& p0, const PntTng& p1) const;
private:
    vector<cd::pointCollisionTester>& collisionTesters_;
};

//----------------------------------------------------------------------------
template<typename PointsMetaClass, typename AveraginFunctor>
vector<PointsMetaClass>
double_polygon(const vector<PointsMetaClass>& pnts, 
               bool b_open,
               bool b_preserve, 
               const AveraginFunctor& fnAvg)
{
    size_t N = pnts.size();
    size_t NN = b_open ? (N - 1) : N;
    vector<PointsMetaClass> res;// (NN * (b_preserve ? 2 : 1));

    for (size_t i = 0; i < NN; ++i)
    {
        PointsMetaClass r = fnAvg(0.5, pnts[i], pnts[(i + 1) % N]);

        if (b_preserve)
            res.push_back(pnts[i]);
        res.push_back(r);
    }
    if (b_preserve && b_open)
        res.push_back(*pnts.rbegin());

    return res;
}

//----------------------------------------------------------------------------
template<typename PointsMetaClass, 
         typename AveraginFunctor>
vector<PointsMetaClass>
perform_Lane_Riesenfeld_algorithm(  const vector<PointsMetaClass>&  pnts,
                                    bool                            b_open,
                                    unsigned int                    n_iterations,
                                    unsigned int                    n_smoothing_inner_steps,
                                    const AveraginFunctor&          fnAvg)
{
    vector<PointsMetaClass> res(pnts);
    while (n_iterations-- > 0)
    {
        res = double_polygon<PointsMetaClass, AveraginFunctor>( res,
                                                                b_open,
                                                                true,
                                                                fnAvg);
        for (unsigned int m = 0; m < n_smoothing_inner_steps; ++m)
            res = double_polygon<PointsMetaClass, AveraginFunctor>( res,
                                                                    b_open,
                                                                    false,
                                                                    fnAvg);
    }
    return res;    
}
//----------------------------------------------------------------------------
using namespace cd;
class cd::pointCollisionTester;
vector<Vertex>
subd_smoothing(const vector<Vertex>& vecVertices,
               vector<cd::pointCollisionTester>& collisionTesters,
               bool b_open = false,
               int n_iterations = 1,
               int n_smoothing_inner_steps = 0);
//============================ END OF FILE ===================================
