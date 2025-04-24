#pragma once
#include <cmath>

namespace math_utils {

template <typename dtype>
class InterpolationFn {
 public:
  // For polynomial interpolation
  virtual void setPolyInterpKernel(double period, dtype start_point,
                                   dtype end_point, int n = 1000) {};
  // For power smooth function
  virtual void setPowerSmoothKernel(dtype end_point, double k, double alpha) {};

  virtual dtype solve(double t) = 0;
  virtual dtype step() = 0;
  virtual dtype d(double t) = 0;
  virtual dtype dd(double t) = 0;
  virtual dtype operator()(double t) = 0;
};

template <typename dtype>
class CubicInterpolationFn : public InterpolationFn<dtype> {
 public:
  void setPolyInterpKernel(double period, dtype start_point, dtype end_point,
                           int n = 1000) override {
    period_ = period;
    start_point_ = start_point;
    end_point_ = end_point;
    dtype d_length = end_point - start_point;
    coefficient_a_ = -2 * d_length / std::pow(period, 3);
    coefficient_b_ = 3 * d_length / (period * period);
    num_points_ = n;
    step_period_ = period_ / num_points_;
    step_ = 0;
    t_ = 0.0;
  }

  dtype solve(double t) override {
    if (t > period_) t = period_;
    return coefficient_a_ * t * t * t + coefficient_b_ * t * t + start_point_;
  }

  dtype step() override {
    ++step_;
    if (step_ <= num_points_) t_ += step_period_;
    return coefficient_a_ * t_ * t_ * t_ + coefficient_b_ * t_ * t_ +
           start_point_;
  }

  dtype d(double t) override {
    if (t > period_) t = period_;
    return 3 * coefficient_a_ * t * t + 2 * coefficient_b_ * t;
  }
  dtype dd(double t) override {
    if (t > period_) t = period_;
    return 6 * coefficient_a_ * t + 2 * coefficient_b_;
  }

  dtype operator()(double t) override {
    if (t > period_) t = period_;
    return coefficient_a_ * t * t * t + coefficient_b_ * t * t + start_point_;
  }

 private:
  dtype start_point_, end_point_;
  dtype coefficient_a_, coefficient_b_;
  int num_points_, step_;
  double step_period_, period_, t_;
};
/**
 * @brief Quintic interpolation function
 * @details Use this function to obtain a curve with
 *          continuous acceleration. And it satisfies:
 *          f(0) = 0, f(t) = l, f(t) = a * t^5 + b * t^4 + c * t^3
 */
template <typename dtype>
class QuinticInterpolationFn : public InterpolationFn<dtype> {
 public:
  void setPolyInterpKernel(double period, dtype start_point, dtype end_point,
                           int n = 1000) override {
    period_ = period;
    start_point_ = start_point;
    end_point_ = end_point;
    dtype d_length = end_point - start_point;
    coefficient_a_ = 6 * d_length / std::pow(period, 5);
    coefficient_b_ = -2.5 * coefficient_a_ * period;
    coefficient_c_ = 5 * coefficient_a_ * period * period / 3;
    num_points_ = n;
    step_period_ = period_ / num_points_;
    step_ = 0;
  }
  dtype solve(double t) override {
    if (t > period_) t = period_;
    return coefficient_a_ * std::pow(t, 5) + coefficient_b_ * std::pow(t, 4) +
           coefficient_c_ * std::pow(t, 3) + start_point_;
  }

  dtype step() override {
    ++step_;
    if (step_ <= num_points_) t_ += step_period_;
    return coefficient_a_ * std::pow(t_, 5) + coefficient_b_ * std::pow(t_, 4) +
           coefficient_c_ * std::pow(t_, 3) + start_point_;
  }

  dtype d(double t) override {
    if (t > period_) t = period_;
    return 5 * coefficient_a_ * std::pow(t, 4) +
           4 * coefficient_b_ * std::pow(t, 3) +
           3 * coefficient_c_ * std::pow(t, 2);
  }

  dtype dd(double t) override {
    if (t > period_) t = period_;
    return 20 * coefficient_a_ * std::pow(t, 3) +
           12 * coefficient_b_ * std::pow(t, 2) + 6 * coefficient_c_ * t;
  }

  dtype operator()(double t) override {
    if (t > period_) t = period_;
    return coefficient_a_ * std::pow(t, 5) + coefficient_b_ * std::pow(t, 4) +
           coefficient_c_ * std::pow(t, 3) + start_point_;
  }

 private:
  dtype start_point_, end_point_;
  dtype coefficient_a_, coefficient_b_, coefficient_c_;
  int num_points_, step_;
  double step_period_, period_, t_;
};
}  // namespace math_utils
