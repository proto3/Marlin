#include "Marlin.h"
#include "plasma.h"

#define TURN_PLASMA_ON  digitalWrite(PLASMA_CONTROL_PIN, PLASMA_CONTROL_INVERTING ? LOW : HIGH);
#define TURN_PLASMA_OFF digitalWrite(PLASMA_CONTROL_PIN, PLASMA_CONTROL_INVERTING ? HIGH : LOW);
#define IS_PLASMA_TRANSFERRED (digitalRead(PLASMA_TRANSFER_PIN) == (PLASMA_TRANSFER_INVERTING ? LOW : HIGH))

PlasmaState Plasma::state = Locked;

void Plasma::init() {
  pinMode(PLASMA_CONTROL_PIN, OUTPUT);
  pinMode(PLASMA_TRANSFER_PIN, INPUT);
  stop();
}

bool Plasma::start() {
  CRITICAL_SECTION_START
  if(state != Locked)
  {
    state = Ignition;
    TURN_PLASMA_ON
  }
  CRITICAL_SECTION_END
  return state != Locked;
}

void Plasma::stop() {
  if(state != Locked)
  {
    state = Off;
  }
  TURN_PLASMA_OFF
}

void Plasma::lock() {
  stop();
  state = Locked;
}

void Plasma::unlock() {
  state = Off;
}

PlasmaState Plasma::update_state() {
  switch(state)
  {
    case Locked:
      break;
    case Off:
      break;
    case Ignition:
      if(IS_PLASMA_TRANSFERRED)
        state = Established;
      break;
    case Established:
      if(!IS_PLASMA_TRANSFERRED)
      {
        stop();
        state = Lost;
      }
      break;
    case Lost:
      break;
  }
  return state;
}

PlasmaState Plasma::get_state() {
  return state;
}
