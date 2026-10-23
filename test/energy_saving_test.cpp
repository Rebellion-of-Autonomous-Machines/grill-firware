#include "EnergySavingControl.h"
#include <cassert>
#include <cstdio>

int main() {
  EnergySavingControl e;
  e.update(false, 180, 20, true, false, 0);
  assert(e.lidCommand(0)); // Cold manual operation with heating disabled.
  e.update(true, 180, 20, true, false, 100);
  assert(!e.ready() && !e.lidCommand(100));
  e.update(true, 180, 174.9f, true, false, 200);
  assert(!e.ready());
  e.update(true, 180, 175, true, false, 300);
  assert(e.ready());
  e.update(true, 180, 170, true, false, 400);
  assert(e.ready()); // Normal thermostat cycling keeps readiness latched.
  e.update(true, 180, 175, true, false, 1800099);
  assert(!e.eco());
  e.update(true, 180, 175, true, false, 1800100);
  assert(e.eco() && !e.ready() && e.effectiveTarget() == 100);
  assert(!e.lidCommand(1800100)); // Wake must be retried, even when already hot.
  assert(!e.eco() && e.effectiveTarget() == 180);
  e.update(true, 180, 100, true, false, 1800200);
  assert(!e.ready());
  assert(!e.lidCommand(1800300)); // Rejected command refreshes idle timer.
  e.update(true, 180, 175, true, false, 1800400);
  assert(e.lidCommand(1800400));
  e.update(true, 200, 175, true, false, 1800500);
  assert(!e.ready());
  e.update(true, 200, 195, true, false, 1800600);
  assert(e.ready());
  e.update(true, 200, 195, false, false, 1800700);
  assert(!e.ready()); // Invalid/stale sensor must not authorize movement.
  e.update(false, 200, 20, false, false, 1800800);
  assert(e.lidCommand(1800800));
  e.update(true, 100, 95, true, false, 1800900);
  e.update(true, 100, 95, true, false, 9000000);
  assert(!e.eco() && e.ready());
  // Millis rollover, and an in-progress movement is never interrupted by standby.
  EnergySavingControl rollover;
  uint32_t start = UINT32_MAX - 5000;
  rollover.update(true, 180, 180, true, false, start);
  rollover.update(true, 180, 180, true, true, start + EnergySavingControl::idleMs);
  assert(!rollover.eco());
  rollover.update(true, 180, 180, true, false, start + EnergySavingControl::idleMs + 1);
  assert(rollover.eco());
  puts("Energy saving: timeout, wake, readiness, retry, heat OFF and rollover passed");
}
