#pragma once
#include <stdint.h>
#include <math.h>

class EnergySavingControl {
 public:
  static constexpr uint32_t idleMs = 30UL * 60UL * 1000UL;
  static constexpr float standbyC = 100.0f;
  static constexpr float readyDeltaC = 5.0f;

  void update(bool enabled, float target, float temperature, bool healthy,
              bool moving, uint32_t now) {
    if (enabled != enabled_ || fabsf(target - target_) > 0.01f) {
      ready_ = false;
      eco_ = false;
      // Only enabling heat starts a new inactivity interval.
      if (enabled && !enabled_) lastCommandMs_ = now;
    }
    enabled_ = enabled;
    target_ = target;
    if (!enabled) {
      ready_ = false;
      eco_ = false;
      return;
    }
    if (target > standbyC && !moving && now - lastCommandMs_ >= idleMs) {
      eco_ = true;
      ready_ = false;
    }
    if (!healthy) ready_ = false;
    else if (!eco_ && temperature >= target - readyDeltaC) ready_ = true;
  }

  // An exit from standby never queues or executes the triggering movement.
  bool lidCommand(uint32_t now) {
    lastCommandMs_ = now;
    if (eco_) {
      eco_ = false;
      ready_ = false;
      return false;
    }
    return !enabled_ || ready_;
  }

  float effectiveTarget() const { return eco_ ? standbyC : target_; }
  bool eco() const { return eco_; }
  bool ready() const { return ready_; }
  uint16_t idleSecondsLeft(uint32_t now) const {
    if (!enabled_ || target_ <= standbyC || eco_) return 0;
    uint32_t elapsed = now - lastCommandMs_;
    return elapsed >= idleMs ? 0 : (idleMs - elapsed + 999) / 1000;
  }

 private:
  bool enabled_ = false, eco_ = false, ready_ = false;
  float target_ = 100.0f;
  uint32_t lastCommandMs_ = 0;
};
