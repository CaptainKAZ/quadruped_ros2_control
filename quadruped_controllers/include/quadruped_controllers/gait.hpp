#pragma once

#include "quadruped_controllers/quadruped_types.hpp"
#include <cstddef>

namespace quadruped_controllers {
template <typename T> class OffsetDurationGait {
public:
  OffsetDurationGait(T cycle, const Vec4<T> &offsets, const Vec4<T> &durations)
      : cycle_(cycle), offsets_(offsets), durations_(durations) {}

  void update(const rclcpp::Time &time) {
    phase_ = std::fmod(time.seconds() / cycle_, 1.);
  }

  DVec<T> getMpcTable(int horizon) {
    DVec<T> mpc_table(4 * horizon);
    int iteration = phase_ * horizon;

    for (int i = 0; i < horizon; i++) {
      int iter = (i + iteration) % horizon;
      for (int j = 0; j < 4; j++) {
        int progress = iter - offsets_[j] * horizon;
        if (progress < 0)
          progress += horizon;
        if (progress >= durations_[j] * horizon)
          mpc_table[i * 4 + j] = 0;
        else
          mpc_table[i * 4 + j] = 1;
      }
    }
    return mpc_table;
  }

  Vec4<T> getSwingTime() {
    Vec4<T> ones;
    ones.setOnes();
    return (ones - durations_) * cycle_;
  }

  double getSwingTime(size_t leg) { return cycle_ * (1. - durations_[leg]); }

  Vec4<T> getStanceTime() {
    Vec4<T> ones;
    return durations_ * cycle_;
  }

  double getStanceTime(size_t leg) { return durations_[leg] * cycle_; }

  T getCycle() { return cycle_; }

private:
  T cycle_, phase_;

  Vec4<T> offsets_;   // offset in 0.0 ~ 1.0
  Vec4<T> durations_; // duration of step in 0.0 ~ 1.0
};

class MpcGait : public OffsetDurationGait<double> {
public:
  MpcGait(double cycle, const Vec4<double> &offsets,
          const Vec4<double> &durations, double mpc_dt, int mpc_horizon)
      : OffsetDurationGait<double>(cycle, offsets, durations), mpc_dt_(mpc_dt),
        mpc_horizon_(mpc_horizon) {}
  double mpc_dt_;
  int mpc_horizon_;
};

} // namespace quadruped_controllers