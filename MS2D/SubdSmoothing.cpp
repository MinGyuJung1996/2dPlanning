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
    bnorm.normalize();
    this->right_tg = this->nr.cross(bnorm);
    right_tg.normalize();
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
    bnorm.normalize();
    Vector3D prev_perp = v_prev.cross(bnorm);
    prev_perp.normalize();
    Vector3D next_perp = bnorm.cross(v_next);
    next_perp.normalize();
    double w = prev_len / (prev_len + next_len);
    this->nr = prev_perp.geodesic_avg(next_perp, w);
}

void PntTng::setEndpointNormAndTang(const PntTng& other, bool b_other_is_next)
{
    const Vector3D& other_der = b_other_is_next ? other.left_tg : other.right_tg;
    Vector3D segm = pt - other.pt;
    Vector3D bnorm = other_der.cross(segm);
    nr = bnorm.cross(segm * (-1));
    nr.normalize();
    if (b_other_is_next)
    {
        right_tg = bnorm.cross(nr);
        right_tg.normalize();
    }
    else
    {
        left_tg = nr.cross(bnorm);
        left_tg.normalize();
    }
}

//-----------------------------------------------------------------------------
void init_norms_and_tangents(vector<PntTng>& pnts, bool b_open)
{
    size_t N = pnts.size();
    size_t idxStart = 0;
    size_t idxEnd = N - 1;
    if (b_open)
    {
        ++idxStart;
        --idxEnd;
    }
    for (size_t i = idxStart; i <= idxEnd; ++i)
    {
        pnts[i].setNaiveNorm(pnts[(i - 1 + N) % N], pnts[(i + 1) % N]);
        pnts[i].setTangents(pnts[(i - 1 + N) % N], pnts[(i + 1) % N]);
    }
    if (b_open)
    {
        pnts[0].setEndpointNormAndTang(pnts[1], true);
        pnts[N-1].setEndpointNormAndTang(pnts[N-2], false);
    }
    if (2 == N)
    {
        //exotic case: we have just one segment, i.e. 2 vertices 
        // to define data for.
        Vector3D xaxis(1., 0., 0.);
        Vector3D seg = pnts[1].pt - pnts[0].pt;
        seg.normalize();
        Vector3D bnorm = xaxis.cross(seg);
        Vector3D norm = bnorm.cross(seg);
        pnts[0].nr = norm;
        pnts[0].right_tg = bnorm;
        pnts[1].nr = norm;
        pnts[1].left_tg = bnorm * (-1);
    }

}

//-----------------------------------------------------------------------------
PntTng 
BezierQuasiAverage::operator()(double t0, const PntTng& p0, const PntTng& p1) const
{
    BezierCurve3D bez_crv = BezierCurve3D(p0.pt, p0.nr, p0.right_tg, 
                                          p1.pt, p1.nr, p1.left_tg);
    Vertex res_pos = bez_crv.eval(t0);
    cd::Point res_ptn_2D(res_pos.x, res_pos.y);
    cd::Point free_ptn_2D;
    bool b_collides = 
        this->collisionTesters_[res_pos.z].testPrecise(res_ptn_2D, 
                                                       free_ptn_2D);
    if (b_collides)
    {
        res_pos.x = free_ptn_2D.x();
        res_pos.y = free_ptn_2D.y();
    }
    Vector3D res_norm = bez_crv.norm(t0);
    PntTng res_obj(res_pos, res_norm);
    res_obj.setTangents(p0, p1);
    return res_obj;
}

//-----------------------------------------------------------------------------
vector<PntTng> pts_to_pnp(const vector<Vertex>& vecVertices)
{
    vector<PntTng> res(vecVertices.size());
    for(int i = 0; i < vecVertices.size(); ++i)
        res[i] = PntTng(vecVertices[i]);
    return res;
}

//-----------------------------------------------------------------------------
vector<Vertex> pnp_to_pts(const vector<PntTng>& vecPNPs)
{
    vector<Vertex> res(vecPNPs.size());
    for (int i = 0; i < vecPNPs.size(); ++i)
        res[i] = Vertex(vecPNPs[i].pt);
    return res;
}

//-----------------------------------------------------------------------------
vector<Vertex>
subd_smoothing( const vector<Vertex>& vecVertices,
                vector<cd::pointCollisionTester>& collisionTesters,
                bool b_open,
                int n_iterations,
                int n_smoothing_inner_steps)
{
    vector<PntTng> pnts = pts_to_pnp(vecVertices);
    init_norms_and_tangents(pnts, b_open);
    BezierQuasiAverage fnBQA(collisionTesters);

    for (int i = 0; i < n_iterations; ++i)
        pnts = perform_Lane_Riesenfeld_algorithm<PntTng, BezierQuasiAverage>(
                  pnts, b_open, n_iterations, n_smoothing_inner_steps, fnBQA);

    return pnp_to_pts(pnts);
}

//============================ END OF FILE ===================================
