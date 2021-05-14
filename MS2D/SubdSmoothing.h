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
using namespace cd;
class cd::pointCollisionTester;
vector<Vertex>
subd_smoothing(const vector<Vertex>& vecVertices,
               vector<cd::pointCollisionTester>& collisionTesters,
               int n_of_iterations = 1,
               bool b_open = false);
//============================ END OF FILE ===================================
