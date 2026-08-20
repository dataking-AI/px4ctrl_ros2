#include "px4_ctrl_ros2/controller.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace px4_ctrl_ros2
{
namespace
{

void expectSameRotation(const Eigen::Quaterniond & lhs, const Eigen::Quaterniond & rhs)
{
  EXPECT_NEAR(
    std::abs(lhs.normalized().dot(rhs.normalized())),
    1.0, 1.0e-12);
}

}  // namespace

TEST(CoordinateConversion, RoundTripsNedFrdAndNwuFlu)
{
  const Eigen::Quaterniond q_ned_frd(
    Eigen::AngleAxisd(0.37, Eigen::Vector3d(1.0, -2.0, 3.0).normalized()));

  const Eigen::Quaterniond q_nwu_flu = ned_frd_to_nwu_flu(q_ned_frd);
  const Eigen::Quaterniond round_trip = nwu_flu_to_ned_frd(q_nwu_flu);

  expectSameRotation(round_trip, q_ned_frd);
}

TEST(CoordinateConversion, AppliesFixedNwuFluToNedFrdAxisTransform)
{
  const Eigen::Quaterniond q_nwu_flu(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d nwu_to_ned;
  nwu_to_ned << 1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0;

  const Eigen::Matrix3d expected =
    nwu_to_ned * q_nwu_flu.toRotationMatrix() * nwu_to_ned.transpose();
  const Eigen::Quaterniond actual = nwu_flu_to_ned_frd(q_nwu_flu);

  expectSameRotation(actual, Eigen::Quaterniond(expected));
}

}  // namespace px4_ctrl_ros2
