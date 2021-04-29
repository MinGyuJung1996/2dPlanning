#include "SubdSmoothing.h"

//----------------------------------------------------------------------------
BezierCurve3D::BezierCurve3D(
    const Vertex& e,
    const Vertex& f,
    const Vertex& g,
    const Vertex& h) :
    a(e), b(f), c(g), d(h) {}

BezierCurve3D::BezierCurve3D(
    const Vertex& p0,
    const Vector3D& n0,
    const Vector3D& d0,
    const Vertex& p1,
    const Vector3D& n1,
    const Vector3D& d1):
a(p0),
d(p1)
{
    double theta = acos(n0.dot(n1));
    double p0p1_dist = (p0 - p1).norm();
    double tang_len = p0p1_dist / (3. * pow(cos(theta / 4.), 2));
    b = p0 + d0 * tang_len;
    c = p1 + d1 * tang_len;
}


Vector3D BezierCurve3D::der(double t) const
{
    double t2 = t * t;
    double mt = 1 - t;
    double mt2 = mt * mt;
    Vertex a_tag(0., 0.), b_tag(0., 0.), c_tag(0., 0.);
    _get_quadric_bezier_ctrl_pts(a_tag, b_tag, c_tag);
    Vector3D der = a_tag * mt2 + b_tag * 2 * mt * t + c_tag * t2;
    return der;
}

Vector3D BezierCurve3D::second_der(double t) const
{
    Vertex a_tag(0., 0.), b_tag(0., 0.), c_tag(0., 0.);
    _get_quadric_bezier_ctrl_pts(a_tag, b_tag, c_tag);
    Vector3D a_dtag = (b_tag - a_tag) * 2.;
    Vector3D b_dtag = (c_tag - b_tag) * 2.;
    Vector3D sec_der = a_dtag * t + b_dtag * (1 - t);
    return sec_der;
}

Vector3D BezierCurve3D::norm(double t) const
{
    Vector3D fir_der = der(t);
    Vector3D sec_der = second_der(t);
    Vector3D bnorm = fir_der.cross(sec_der);
    bnorm.normalize();
    fir_der.normalize();
    Vector3D nrm = fir_der.cross(bnorm);
    nrm.normalize();
    return nrm;
}

Vertex BezierCurve3D::eval(double t) const
{
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;
    Vertex pt = a * mt3 + b * 3. * mt2 * t \
        + c * 3. * mt * t2 + d * t3;
    return pt;
}
//----------------------------------------------------------------------------
PntTng::PntTng(const Vertex& p, const Vector3D& n):
pt(p), nr(n), right_tg(), left_tg()
{}


void PntTng::setTangents(const PntTng& prev_pt, const PntTng& next_pt)
{
    Vector3D v_prev = prev_pt.pt - pt;
    v_prev.normalize();
    Vector3D v_next = next_pt.pt - pt;
    v_next.normalize();
    Vector3D bnorm = v_prev.cross(v_next);
    this->right_tg = this->nr.cross(bnorm);
    this->left_tg = this->right_tg * (-1);
}

void PntTng::setNaiveNorm(const PntTng& prev_pt, const PntTng& next_pt)
{
    Vector3D v_prev = prev_pt.pt - pt;
    double prev_len = v_prev.norm();
    v_prev.normalize();
    Vector3D v_next = next_pt.pt - pt;
    double next_len = v_next.norm();
    v_next.normalize();
    Vector3D bnorm = v_prev.cross(v_next);
    Vector3D prev_perp = v_prev.cross(bnorm);
    Vector3D next_perp = bnorm.cross(v_next);
    double w = prev_len / (prev_len + next_len);
    this->nr = prev_perp.geodesic_avg(next_perp, w);
}

//-----------------------------------------------------------------------------
void init_norms_and_tangents(vector<PntTng>& pnts, bool b_open)
{
    //@@TODO: only closed polgons are supported now
    size_t N = pnts.size();
    for (size_t i = 0; i < N; ++i)
    {
        pnts[i].setNaiveNorm(pnts[(i - 1 + N) % N], pnts[(i + 1) % N]);
        pnts[i].setTangents(pnts[(i - 1 + N) % N], pnts[(i + 1) % N]);
    }
}

//-----------------------------------------------------------------------------
Vertex test_collision_and_correct(const Vertex& p)
{
    return p;
}

//-----------------------------------------------------------------------------
PntTng bqa_3D(double t0, const PntTng& p0, const PntTng& p1)
{
    BezierCurve3D bez_crv = BezierCurve3D(p0.pt, p0.nr, p0.right_tg, p1.pt, p1.nr, p1.left_tg);
    Vertex res_pos = bez_crv.eval(t0);
    Vertex free_pos = test_collision_and_correct(res_pos);
    Vector3D res_norm = bez_crv.norm(t0);
    PntTng res_obj(res_pos, res_norm);
    if (!(free_pos == res_pos))
        res_obj.setNaiveNorm(p0, p1);
    res_obj.setTangents(p0, p1);
    return res_obj;
}

//----------------------------------------------------------------------------
vector<PntTng> 
double_polygon_bqa(const vector<PntTng>& pnts, bool b_preserve, bool b_open)
{
    //@@TODO: only closed polgons are supported now
    size_t N = pnts.size();
    size_t NN = b_open ? (N - 1) : N;
    vector<PntTng> res(NN*(b_preserve?2:1));

    for (size_t i = 0; i < NN; ++i)
    {
        PntTng r = bqa_3D(0.5, pnts[i], pnts[(i + 1) % N]);

        if (b_preserve)
            res.push_back(pnts[i]);
        res.push_back(r);
    }
    if (b_preserve && b_open)
        res.push_back(*pnts.rbegin());

    return res;
}

//-----------------------------------------------------------------------------
vector<PntTng> pts_to_pnp(const vector<Vertex>& vecVertices)
{
    vector<PntTng> res(vecVertices.size());
    for (auto& v : vecVertices)
        res.push_back(PntTng(v));
    return res;
}

//-----------------------------------------------------------------------------
vector<Vertex> pnp_to_pts(const vector<PntTng>& vecPNPs)
{
    vector<Vertex> res(vecPNPs.size());
    for (auto& v : vecPNPs)
        res.push_back(Vertex(v.pt));
    return res;
}

//-----------------------------------------------------------------------------
vector<Vertex>
subd_smoothing(const vector<Vertex>& vecVertices, 
               int n_of_iterations = 1, 
               bool b_open = false)
{
    vector<PntTng> pnts = pts_to_pnp(vecVertices);
    init_norms_and_tangents(pnts, b_open);

    for (int i = 0; i < n_of_iterations; ++i)
        pnts = double_polygon_bqa(pnts, true, b_open);

    return pnp_to_pts(pnts);
}

//============================ END OF FILE ===================================
