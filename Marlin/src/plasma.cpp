#include "Marlin.h"
#include "plasma.h"
#include "torch_height_control.h"

#define TURN_PLASMA_ON  digitalWrite(PLASMA_CONTROL_PIN, PLASMA_CONTROL_INVERTING ? LOW : HIGH);
#define TURN_PLASMA_OFF digitalWrite(PLASMA_CONTROL_PIN, PLASMA_CONTROL_INVERTING ? HIGH : LOW);
#define IS_PLASMA_TRANSFERRED (digitalRead(PLASMA_TRANSFER_PIN) == (PLASMA_TRANSFER_INVERTING ? LOW : HIGH))

PlasmaState Plasma::state = Off;
bool Plasma::stop_pending = false;
bool Plasma::locked_flag = true;
bool Plasma::lost_flag = false;

uint32_t Plasma::_pierce_time_ms = 500;
uint32_t Plasma::_cutting_feedrate_mm_m = 5000;

//----------------------------------------------------------------------------//
void Plasma::init() {
  pinMode(PLASMA_CONTROL_PIN, OUTPUT);
  #if PLASMA_TRANSFER_PULLUP
    pinMode(PLASMA_TRANSFER_PIN, INPUT_PULLUP);
  #else
    pinMode(PLASMA_TRANSFER_PIN, INPUT);
  #endif
  TURN_PLASMA_OFF
}
//----------------------------------------------------------------------------//
bool Plasma::ignite()
{
  CRITICAL_SECTION_START
  lost_flag = false;
  bool ok = !locked_flag && state == Off;
  if(ok)
  {
    state = Ignition;
    TURN_PLASMA_ON
  }
  CRITICAL_SECTION_END
  return ok;
}
//----------------------------------------------------------------------------//
bool Plasma::activate_thc()
{
  CRITICAL_SECTION_START
  bool ok = !locked_flag && state == Established;
  if(ok)
  {
    state = Established_THC;
    torchHeightController.enable();
  }
  CRITICAL_SECTION_END
  return ok;
}
//----------------------------------------------------------------------------//
void Plasma::stop()
{
  CRITICAL_SECTION_START
  TURN_PLASMA_OFF
  switch (state)
  {
    case Off:
    case Slowdown_THC:
      break;
    case Ignition:
    case Established:
      state = Off;
      break;
    case Established_THC:
      state = Slowdown_THC;
      break;
  }
  stop_pending = false;
  CRITICAL_SECTION_END
}
//----------------------------------------------------------------------------//
void Plasma::stop_after_move()
{
  stop_pending = true;
}
  //----------------------------------------------------------------------------//
void Plasma::lock()
{
  CRITICAL_SECTION_START
  stop();
  locked_flag = true;
  CRITICAL_SECTION_END
}
//----------------------------------------------------------------------------//
void Plasma::unlock()
{
  locked_flag = false;
}
//----------------------------------------------------------------------------//
PlasmaState Plasma::update()
{
  switch (state)
  {
    case Off:
      break;
    case Slowdown_THC:
      if(torchHeightController.disable())
        state = Off;
      break;
    case Ignition:
      if(IS_PLASMA_TRANSFERRED)
        state = Established;
      break;
    case Established:
    case Established_THC:
      if(!IS_PLASMA_TRANSFERRED)
      {
        stop();
        lost_flag = true;
      }
      else if(stop_pending && !planner.blocks_queued())
      {
        stop();
      }
      break;
  }
  return state;
}
//----------------------------------------------------------------------------//
PlasmaState Plasma::get_state()
{
  return state;
}
//----------------------------------------------------------------------------//
bool Plasma::is_lost()
{
  return lost_flag;
}
//----------------------------------------------------------------------------//
