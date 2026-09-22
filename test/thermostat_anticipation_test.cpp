#include "ThermostatAnticipation.h"
#include <assert.h>
#include <stdio.h>

static void trend(ThermostatAnticipation& control, float start, float rate, uint32_t offset = 0) {
  for (uint32_t t = 0; t <= 20000; t += 500) {
    control.sample(start + rate * (t / 1000.0f), offset + t);
  }
}

static void testPredictionAndRecovery() {
  ThermostatAnticipation control;
  assert(control.demand(false, 20, 100, 2));
  assert(!control.demand(false, NAN, 100, 2));
  trend(control, 80, 0.5f);
  assert(!control.demand(true, 95, 100, 2));  // Stop while rising, below the setpoint.
  assert(control.offThreshold(100) >= 90);
  control.switched(false, 90, 20000);
  assert(!control.demand(false, 92, 100, 2)); // Do not reheat during residual rise.
  control.resetSamples();
  trend(control, 101, -0.1f);
  assert(control.demand(false, 99, 100, 2));  // Start while falling, above the lower bound.
  assert(!control.demand(false, 100.1f, 100, 2));
  assert(control.onThreshold(100, 2) <= 100);
  control.sample(99, 30000);                 // Gap discards the stale derivative.
  assert(control.rateCPerSecond() == 0);
  assert(!control.demand(false, 99, 100, 2));
  control.reset();
  trend(control, 80, 0.3f, UINT32_MAX - 15000); // Unsigned clock rollover.
  assert(control.rateCPerSecond() > 0.2f);
  control.sample(NAN, 21000);
  assert(control.rateCPerSecond() == 0);
}

static void testLearningAndNoise() {
  ThermostatAnticipation control;
  trend(control, 70, 0.5f);
  control.switched(false, 80, 20000);
  for (uint32_t t = 20500; t <= 80000; t += 500) {
    float dt = (t - 20000) / 1000.0f;
    float temp = dt <= 20 ? 80 + 0.5f * dt : 90 - 0.15f * (dt - 20);
    control.sample(temp, t);
  }
  assert(control.learnedCycles() == 1);
  assert(control.offSeconds() < 30 && control.offSeconds() > 20);
  control.reset();
  trend(control, 70, 0.5f);
  control.switched(false, 80, 20000);
  control.cancelCycle(); // Manual stop, changed settings or moving lid.
  for (uint32_t t = 20500; t <= 60000; t += 500) control.sample(80, t);
  assert(control.learnedCycles() == 0);
  control.reset();
  for (uint32_t t = 0; t < 120000; t += 500) {
    control.sample(99 + 0.05f * sinf(t * 0.007f), t);
    assert(!control.demand(false, 99, 100, 2));
    assert(control.demand(true, 99, 100, 2));
  }
}

struct Result {
  float low = 1000;
  float high = -1000;
  float error = 0;
  int samples = 0;
  int changes = 0;
};

// Illustrative lagged heater, not an identification of the real grill.
static Result simulate(bool predictive, float lagSeconds) {
  ThermostatAnticipation control;
  float temperature = 20, filtered = 20, heatFlow = 0;
  bool on = false;
  uint32_t lastSwitch = 0;
  Result result;
  for (uint32_t ms = 0; ms <= 3600000; ms += 500) {
    heatFlow += (0.5f / lagSeconds) * ((on ? 0.9f : 0) - heatFlow);
    temperature += 0.5f * (heatFlow - 0.003f * (temperature - 20));
    filtered += 0.25f * (temperature - filtered);
    control.sample(filtered, ms);
    bool wanted = predictive ? control.demand(on, filtered, 100, 2)
                              : (on ? filtered < 100 : filtered <= 95);
    if (wanted != on && ms - lastSwitch >= 10000) {
      on = wanted;
      lastSwitch = ms;
      if (predictive) control.switched(on, filtered, ms);
      if (ms >= 1800000) ++result.changes;
    }
    if (ms >= 1800000) {
      result.low = fminf(result.low, temperature);
      result.high = fmaxf(result.high, temperature);
      result.error += fabsf(temperature - 100);
      ++result.samples;
    }
  }
  result.error /= result.samples;
  printf("%s lag=%.0fs: %.2f..%.2f C, mean |error|=%.2f C, switches/30min=%d\n",
         predictive ? "predictive" : "baseline  ", lagSeconds,
         result.low, result.high, result.error, result.changes);
  return result;
}

int main() {
  testPredictionAndRecovery();
  testLearningAndNoise();
  const float lags[] = {10.0f, 30.0f, 60.0f};
  for (float lag : lags) {
    Result baseline = simulate(false, lag);
    Result predictive = simulate(true, lag);
    assert(predictive.high - predictive.low < baseline.high - baseline.low);
    assert(predictive.error < baseline.error);
  }
  puts("Thermostat anticipation tests passed");
}
