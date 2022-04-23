#pragma once
#include "quadruped_controllers/quadruped_types.hpp"
#include <chrono>

template <typename T>
inline T square(T a)
{
  return a * a;
}

namespace quadruped_controllers {
/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
template <typename T> Vec3<T> quatToRPY(const Eigen::Quaternion<T> &q) {
  Vec3<T> rpy;
  T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  rpy(2) =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                 square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  rpy(1) = std::asin(as);
  rpy(0) =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                 square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return rpy;
}
} // namespace quadruped_controllers

template <typename T> Eigen::Quaternion<T> RpyToQuat(Vec3<T> rpy) {
  using namespace Eigen;
  // Abbreviations for the various angular functions
  double cy = cos(rpy(2) * 0.5);
  double sy = sin(rpy(2) * 0.5);
  double cp = cos(rpy(1) * 0.5);
  double sp = sin(rpy(1) * 0.5);
  double cr = cos(rpy(0) * 0.5);
  double sr = sin(rpy(0) * 0.5);

  Eigen::Quaternion<T> q;
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}

class Timer{
  public:
    Timer(){
      tic();
    }
    inline void tic(){
      start_time_ = std::chrono::high_resolution_clock::now();
    }
    inline double toc(){
      return std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::high_resolution_clock::now() - start_time_).count();
    }
  private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};