#pragma once

#include <math.h>
#include <stdint.h>

namespace ThermostatDefaults {
constexpr float targetC = 100.0f;
constexpr float hysteresisC = 2.0f;
constexpr uint16_t minOnSeconds = 10;
constexpr uint16_t minOffSeconds = 10;
}

// Learns residual temperature travel after switching from completed extrema.
// Horizons are estimates, not a physical model of the heater.
class ThermostatAnticipation {
 public:
  static constexpr float initialOffSeconds = 30.0f;
  static constexpr float initialOnSeconds = 15.0f;
  static constexpr float maxOffAdvanceC = 10.0f;
  static constexpr float maxOnAdvanceC = 5.0f;

  void reset() {
    offSeconds_ = initialOffSeconds;
    onSeconds_ = initialOnSeconds;
    learnedCycles_ = 0;
    resetSamples();
  }

  void resetSamples() {
    hasSample_ = false;
    rate_ = 0.0f;
    cancelCycle();
  }

  void cancelCycle() {
    tracking_ = false;
    reversalSamples_ = 0;
  }

  void sample(float temperature, uint32_t now) {
    if (!isfinite(temperature)) {
      resetSamples();
      return;
    }
    if (!hasSample_ || now - sampleMs_ > 5000UL) {
      resetSamples();
      hasSample_ = true;
      sampleMs_ = now;
      sampleTemperature_ = temperature;
      return;
    }
    uint32_t elapsed = now - sampleMs_;
    if (elapsed < 2000UL) return;

    float dt = elapsed / 1000.0f;
    float measuredRate = (temperature - sampleTemperature_) / dt;
    sampleMs_ = now;
    sampleTemperature_ = temperature;
    if (fabsf(measuredRate) > 1.0f) {
      // A sudden disturbance is not a usable thermal coast-down cycle.
      resetSamples();
      return;
    }
    rate_ += (dt / (4.0f + dt)) * (measuredRate - rate_);
    if (!tracking_) return;
    if (now - switchMs_ > 180000UL) {
      cancelCycle();
      return;
    }

    if (trackPeak_ ? temperature > extremum_ : temperature < extremum_) {
      extremum_ = temperature;
      reversalSamples_ = 0;
    }
    bool reversed = trackPeak_ ? (extremum_ - temperature >= 0.3f && rate_ < -0.005f)
                               : (temperature - extremum_ >= 0.3f && rate_ > 0.005f);
    reversalSamples_ = reversed ? reversalSamples_ + 1 : 0;
    if (reversalSamples_ < 3) return;

    float travel = trackPeak_ ? extremum_ - switchTemperature_ : switchTemperature_ - extremum_;
    float measuredSeconds = bounded(travel / switchSpeed_, 0.0f, 120.0f);
    float& estimate = trackPeak_ ? offSeconds_ : onSeconds_;
    estimate += bounded(0.25f * (measuredSeconds - estimate), -10.0f, 10.0f);
    if (learnedCycles_ < 65535) ++learnedCycles_;
    cancelCycle();
  }

  void switched(bool heaterOn, float temperature, uint32_t now) {
    cancelCycle();
    float speed = heaterOn ? -rate_ : rate_;
    if (!hasSample_ || !isfinite(temperature) || speed < 0.01f) return;
    tracking_ = true;
    trackPeak_ = !heaterOn;
    switchSpeed_ = speed;
    switchTemperature_ = extremum_ = temperature;
    switchMs_ = now;
  }

  float offThreshold(float target) const {
    return target - bounded(rate_ * offSeconds_, 0.0f, maxOffAdvanceC);
  }

  float onThreshold(float target, float hysteresis) const {
    return fminf(target, target - hysteresis +
                 bounded(-rate_ * onSeconds_, 0.0f, maxOnAdvanceC));
  }

  bool demand(bool heaterOn, float temperature, float target, float hysteresis) const {
    if (!isfinite(temperature)) return false;
    if (heaterOn) return temperature < offThreshold(target);
    if (temperature >= target) return false;
    // Allow the residual rise to finish after an anticipated OFF.
    if (tracking_ && trackPeak_ && rate_ > 0.005f &&
        temperature >= target - maxOffAdvanceC) return false;
    return temperature <= onThreshold(target, hysteresis);
  }

  float rateCPerSecond() const { return rate_; }
  float offSeconds() const { return offSeconds_; }
  float onSeconds() const { return onSeconds_; }
  uint16_t learnedCycles() const { return learnedCycles_; }

 private:
  static float bounded(float v, float low, float high) {
    return fminf(high, fmaxf(low, v));
  }

  float offSeconds_ = initialOffSeconds;
  float onSeconds_ = initialOnSeconds;
  float rate_ = 0.0f;
  float sampleTemperature_ = 0.0f;
  uint32_t sampleMs_ = 0;
  bool hasSample_ = false;
  bool tracking_ = false;
  bool trackPeak_ = false;
  float switchTemperature_ = 0.0f;
  float switchSpeed_ = 0.0f;
  float extremum_ = 0.0f;
  uint32_t switchMs_ = 0;
  uint8_t reversalSamples_ = 0;
  uint16_t learnedCycles_ = 0;
};
