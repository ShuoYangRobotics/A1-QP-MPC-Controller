
// This filter comes from
// https://github.com/google-research/tiny-differentiable-simulator/blob/master/examples/whole_body_control/com_velocity_estimator.hpp
// hence we must include Apache License 2.0 too
#pragma once

#include <cassert>
#include <iostream>
#include <utility>
#include <deque>
// A stable O(1) moving filter for incoming data streams. Implements the
// Neumaier's algorithm to calculate the moving window average,
// which is numerically stable.
class MovingWindowFilter {
 public:

  MovingWindowFilter() {}

  MovingWindowFilter(int window_size) : window_size_(window_size) {
    assert(window_size_ > 0);
    sum_ = 0.0;
    correction_ = 0.0;
  }

  // Computes the moving window average.
  double CalculateAverage(double new_value) {
    if (value_deque_.size() < window_size_) {
      // pass
    } else {
      // The left most value needs to be subtracted from the moving sum first.
      UpdateNeumaierSum(-value_deque_.front());
      value_deque_.pop_front();
    }
    // Add the new value.
    UpdateNeumaierSum(new_value);
    value_deque_.push_back(new_value);

    return (sum_ + correction_) / double(window_size_);
  }

  std::deque<double> GetValueQueue() {
    return value_deque_;
  }
 private:
  int window_size_;
  double sum_, correction_;
  std::deque<double> value_deque_;

  // Updates the moving window sum using Neumaier's algorithm.
  //
  // For more details please refer to:
  // https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
  void UpdateNeumaierSum(double value) {
    double new_sum = sum_ + value;
    if (std::abs(sum_) >= std::abs(value)) {
      // If previous sum is bigger, low-order digits of value are lost.
      correction_ += (sum_ - new_sum) + value;
    } else {
      correction_ += (value - new_sum) + sum_;
    }
    sum_ = new_sum;
  }
};