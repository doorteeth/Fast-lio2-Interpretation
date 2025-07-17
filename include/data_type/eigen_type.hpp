#include <Eigen/Dense>
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;