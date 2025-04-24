#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <vector>

namespace sdk {

struct ArmLowCmd {
  ArmLowCmd(unsigned int dof = 7) {
    this->dof = dof;
    reset();
  };

  void reset() {
    q.assign(dof, 0.);
    dq.assign(dof, 0.);
    tau.assign(dof, 0.);
    kp.assign(dof, 0.);
    kd.assign(dof, 0.);
  }

  void setControlGain(const float& kp, const float& kd) {
    this->kp.assign(dof, kp);
    this->kd.assign(dof, kd);
  };
  void setControlGain(const std::vector<float>& kp,
                      const std::vector<float>& kd) {
    assert(this->kp.size() == kp.size());
    assert(this->kd.size() == kd.size());
    this->kp = kp;
    this->kd = kd;
  };
  void setControlGain(const Eigen::Ref<const Eigen::VectorXf>& kp,
                      const Eigen::Ref<const Eigen::VectorXf>& kd) {
    assert(this->kp.size() == kp.size());
    assert(this->kd.size() == kd.size());
    std::copy(kp.data(), kp.data() + kp.size(), this->kp.data());
    std::copy(kd.data(), kd.data() + kd.size(), this->kd.data());
  }
  void getControlGain(Eigen::Ref<Eigen::VectorXf> kp,
                      Eigen::Ref<Eigen::VectorXf> kd) const {
    kp = Eigen::Map<const Eigen::VectorXf>(this->kp.data(), this->kp.size());
    kd = Eigen::Map<const Eigen::VectorXf>(this->kd.data(), this->kd.size());
  };
  Eigen::Map<const Eigen::VectorXf> getControlGainKp() const {
    return Eigen::Map<const Eigen::VectorXf>(this->kp.data(), this->kp.size());
  };
  Eigen::Map<const Eigen::VectorXf> getControlGainKd() const {
    return Eigen::Map<const Eigen::VectorXf>(this->kd.data(), this->kd.size());
  };
  void setQ(const Eigen::Ref<const Eigen::VectorXf>& q) {
    assert(this->q.size() == q.size());
    std::copy(q.data(), q.data() + q.size(), this->q.data());
  };
  void getQ(Eigen::Ref<Eigen::VectorXf> q) const {
    q = Eigen::Map<const Eigen::VectorXf>(this->q.data(), this->q.size());
  }
  [[nodiscard]] Eigen::Map<const Eigen::VectorXf> getQ() const {
    return Eigen::Map<const Eigen::VectorXf>(this->q.data(), this->q.size());
  }
  void setDq(const Eigen::Ref<const Eigen::VectorXf>& dq) {
    assert(this->dq.size() == dq.size());
    std::copy(dq.data(), dq.data() + dq.size(), this->dq.data());
  };
  void getDq(Eigen::Ref<Eigen::VectorXf> dq) const {
    dq = Eigen::Map<const Eigen::VectorXf>(this->dq.data(), this->dq.size());
  }
  [[nodiscard]] Eigen::Map<const Eigen::VectorXf> getDq() const {
    return Eigen::Map<const Eigen::VectorXf>(this->dq.data(), this->dq.size());
  }
  void setTau(const Eigen::Ref<const Eigen::VectorXf>& tau) {
    assert(this->tau.size() == tau.size());
    std::copy(tau.data(), tau.data() + tau.size(), this->tau.data());
  };
  void getTau(Eigen::Ref<Eigen::VectorXf> tau) const {
    tau = Eigen::Map<const Eigen::VectorXf>(this->tau.data(), this->tau.size());
  }
  [[nodiscard]] Eigen::Map<const Eigen::VectorXf> getTau() const {
    return Eigen::Map<const Eigen::VectorXf>(this->tau.data(),
                                             this->tau.size());
  }
  std::vector<float> q;
  std::vector<float> dq;
  std::vector<float> tau;
  std::vector<float> kp;
  std::vector<float> kd;
  size_t dof;
};

struct G1DualArmLowCmd {
  ArmLowCmd left_arm, right_arm;
  bool enable{false};
  void setQ(Eigen::Ref<const Eigen::VectorXf> q) {
    left_arm.setQ(q.head<7>());
    right_arm.setQ(q.tail<7>());
  }
  [[nodiscard]] const Eigen::VectorXf getQ() const {
    Eigen::VectorXf q(14);
    q.head<7>() = left_arm.getQ();
    q.tail<7>() = right_arm.getQ();
    return q;
  }
  void setDq(Eigen::Ref<const Eigen::VectorXf> dq) {
    left_arm.setDq(dq.head<7>());
    right_arm.setDq(dq.tail<7>());
  }
  [[nodiscard]] const Eigen::VectorXf getDq() const {
    Eigen::VectorXf dq(14);
    dq.head<7>() = left_arm.getDq();
    dq.tail<7>() = right_arm.getDq();
    return dq;
  }
  void setControlGain(const float& kp, const float& kd) {
    left_arm.setControlGain(kp, kd);
    right_arm.setControlGain(kp, kd);
  }
  void setControlGain(Eigen::Ref<const Eigen::VectorXf> kp,
                      Eigen::Ref<const Eigen::VectorXf> kd) {
    left_arm.setControlGain(kp.head<7>(), kd.head<7>());
    right_arm.setControlGain(kp.tail<7>(), kd.tail<7>());
  }
  [[nodiscard]] Eigen::VectorXf getControlGainKp() const {
    Eigen::VectorXf kp(14);
    kp << left_arm.getControlGainKp(), right_arm.getControlGainKp();
    return kp;
  }
  [[nodiscard]] Eigen::VectorXf getControlGainKd() const {
    Eigen::VectorXf kd(14);
    kd << left_arm.getControlGainKd(), right_arm.getControlGainKd();
    return kd;
  }
  void setTau(Eigen::Ref<const Eigen::VectorXf> tau) {
    left_arm.setTau(tau.head<7>());
    right_arm.setTau(tau.tail<7>());
  }
  [[nodiscard]] const Eigen::VectorXf getTau() const {
    Eigen::VectorXf tau(14);
    tau << left_arm.getTau(), right_arm.getTau();
    return tau;
  }
};

struct ArmLowState {
  ArmLowState(unsigned int dof = 7) {
    this->dof = dof;
    q.assign(dof, 0.);
    dq.assign(dof, 0.);
    ddq.assign(dof, 0.);
    tau.assign(dof, 0.);
    end_posture.setZero();
  };

  void getQ(Eigen::Ref<Eigen::VectorXf> q) {
    q = Eigen::Map<Eigen::VectorXf>(this->q.data(), this->q.size());
  }
  [[nodiscard]] Eigen::Map<const Eigen::VectorXf> getQ() {
    return Eigen::Map<const Eigen::VectorXf>(this->q.data(), this->q.size());
  }
  void getDq(Eigen::Ref<Eigen::VectorXf> dq) {
    dq = Eigen::Map<Eigen::VectorXf>(this->dq.data(), this->dq.size());
  }
  [[nodiscard]] Eigen::Map<const Eigen::VectorXf> getDq() {
    return Eigen::Map<const Eigen::VectorXf>(this->dq.data(), this->dq.size());
  }
  void getDdq(Eigen::Ref<Eigen::VectorXf> ddq) {
    ddq = Eigen::Map<Eigen::VectorXf>(this->ddq.data(), this->ddq.size());
  }
  [[nodiscard]] Eigen::Map<const Eigen::VectorXf> getDdq() {
    return Eigen::Map<const Eigen::VectorXf>(this->ddq.data(),
                                             this->ddq.size());
  }
  void getTau(Eigen::Ref<Eigen::VectorXf> tau) {
    tau = Eigen::Map<Eigen::VectorXf>(this->tau.data(), this->tau.size());
  }
  [[nodiscard]] Eigen::Map<const Eigen::VectorXf> getTau() {
    return Eigen::Map<const Eigen::VectorXf>(this->tau.data(),
                                             this->tau.size());
  }
  std::vector<float> q;
  std::vector<float> dq;
  std::vector<float> ddq;
  std::vector<float> tau;
  Eigen::Matrix<double, 6, 1> end_posture;
  size_t dof;
};

struct G1DualArmLowState {
  ArmLowState left_arm, right_arm;
  uint32_t tick{0};
  void getQ(Eigen::Ref<Eigen::VectorXf> q) {
    left_arm.getQ(q.head<7>());
    right_arm.getQ(q.tail<7>());
  }
  [[nodiscard]] const Eigen::VectorXf getQ() {
    Eigen::VectorXf q(14);
    q << left_arm.getQ(), right_arm.getQ();
    return q;
  }
  void getDq(Eigen::Ref<Eigen::VectorXf> dq) {
    left_arm.getDq(dq.head<7>());
    right_arm.getDq(dq.tail<7>());
  }
  [[nodiscard]] const Eigen::VectorXf getDq() {
    Eigen::VectorXf dq(14);
    dq << left_arm.getDq(), right_arm.getDq();
    return dq;
  }
  void getDdq(Eigen::Ref<Eigen::VectorXf> ddq) {
    left_arm.getDdq(ddq.head<7>());
    right_arm.getDdq(ddq.tail<7>());
  }
  [[nodiscard]] const Eigen::VectorXf getDdq() {
    Eigen::VectorXf ddq(14);
    ddq << left_arm.getDdq(), right_arm.getDdq();
    return ddq;
  }
  void getTau(Eigen::Ref<Eigen::VectorXf> tau) {
    left_arm.getTau(tau.head<7>());
    right_arm.getTau(tau.tail<7>());
  }
  [[nodiscard]] Eigen::VectorXf getTau() {
    Eigen::VectorXf tau(14);
    tau << left_arm.getTau(), right_arm.getTau();
    return tau;
  }
};

}  // namespace sdk
