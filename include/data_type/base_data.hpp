#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;

enum LID_TYPE
{
    AVIA = 1,
    VELO16,
    OUST64,
    MARSIM
}; //{1, 2, 3}
enum TIME_UNIT
{
    SEC = 0,
    MS = 1,
    US = 2,
    NS = 3
};
enum Feature
{
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};
enum Surround
{
    Prev,
    Next
};
enum E_jump
{
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
};