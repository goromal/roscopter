#include "ekf/meas.h"

namespace roscopter{
  namespace ekf{
    namespace meas
{

Base::Base()
{
    type = BASE;
    handled = false;
}

std::string Base::Type() const
{
    std::string type_str;
    switch (type)
    {
    case BASE:
        type_str = "Base";
        break;
    case GNSS:
        type_str = "Gnss";
        break;
    case IMU:
        type_str = "Imu";
        break;
    case BARO:
        type_str = "Baro";
        break;
    case RANGE:
        type_str = "Range";
        break;
    case MOCAP:
        type_str = "Mocap";
        break;
    case ZERO_VEL:
        type_str = "ZeroVel";
        break;
    case POS:
        type_str = "Pos";
        break;
    case VEL:
        type_str = "Vel";
        break;
#ifdef RELATIVE
    case REL_HEADING:
        type_str = "RelHeading";
        break;
#endif
    }
    return type_str;
}

bool basecmp(const Base* a, const Base* b)
{
    return a->t < b->t;
}



Imu::Imu(double _t, const Vector6d &_z, const Matrix6d &_R)
{
    t = _t;
    z = _z;
    R = _R;
    type = IMU;
}

Baro::Baro(double _t, const double &_z, const double &_R, const double& _temp)
{
    t = _t;
    z(0) = _z;
    R(0) = _R;
    temp = _temp;
    type = RANGE;
}

Pos::Pos(double _t, const Eigen::Vector3d& _z, const Eigen::Matrix3d& _R)
{
    t = _t;
    z = _z;
    R = _R;
    type = POS;
}

Vel::Vel(double _t, const Eigen::Vector3d& _z, const Eigen::Matrix3d& _R)
{
    t = _t;
    z = _z;
    R = _R;
    type = VEL;
}

Range::Range(double _t, const double &_z, const double &_R)
{
    t = _t;
    z(0) = _z;
    R(0) = _R;
    type = RANGE;
}

#ifdef RELATIVE
RelativeHeading::RelativeHeading(double _t, const double& _z, const double& _R)
{
    t = _t;
    z(0) = _z;
    Eigen::Vector3d Rdiag;
    Rdiag << 0.001, 0.001, R;
    R = Rdiag.asDiagonal();
    R(0) = _R;
    type = REL_HEADING;
}
#endif

Gnss::Gnss(double _t, const Vector6d& _z, const Matrix6d& _R) :
    p(z.data()),
    v(z.data()+3)
{
    t = _t;
    type = GNSS;
    z = _z;
    R = _R;
}

Mocap::Mocap(double _t, const xform::Xformd &_z, const Matrix6d &_R) :
    z(_z),
    R(_R)
{
    t = _t;
    type = MOCAP;
}

ZeroVel::ZeroVel(double _t)
{
    t = _t;
    type = ZERO_VEL;
}
}}}
