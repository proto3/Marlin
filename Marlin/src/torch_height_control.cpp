#include "torch_height_control.h"
#include "ADS1015.h"
#include "stepper.h"

#define PAUSE_TIMER4  TCCR4B = _BV(WGM43);
#define RESUME_TIMER4 TCCR4B = _BV(WGM43) | _BV(CS41) | _BV(CS40);

bool TorchHeightController::_counting_up = true;
int32_t TorchHeightController::_target_speed = 0;
int16_t TorchHeightController::_speed = 0;
int16_t TorchHeightController::_max_acc = 0;
int8_t TorchHeightController::_dir = 1;
uint16_t TorchHeightController::_max_stopping_distance = 0xFFFF;
unsigned long TorchHeightController::_retract_mm = PLASMA_THC_RETRACT_MM;
long TorchHeightController::_z_top_pos = 0;
long TorchHeightController::_z_bottom_pos = 0;
long TorchHeightController::_safe_pos = 0;

int16_t TorchHeightController::_new_target_speed = 25000;
int16_t TorchHeightController::_counter = 0;

//----------------------------------------------------------------------------//
void TorchHeightController::init()
{
  ADS1015_device.init();

  _z_top_pos = sw_endstop_max[Z_AXIS] * planner.axis_steps_per_mm[Z_AXIS];
  _z_bottom_pos = 0;

  // Set maximal period (considered as speed = 0)
  ICR4 = 0xFFFF;

  // Phase correct PWM mode with ICR4 on TOP, prescaler 64
  TCCR4A = _BV(WGM41);
  TCCR4B = _BV(WGM43);

  // Init counter to maxval to mean last Z step is at least this old
  TCNT4 = 0xFFFF;

  // allow interrupts
  TIMSK4 = _BV(TOIE4) | _BV(ICF4);
}
//----------------------------------------------------------------------------//
void TorchHeightController::enable()
{
  stepper.leave_control_on(Z_AXIS);
  Z_ENABLE_WRITE(Z_ENABLE_ON);

  _reset_PID();
}
//----------------------------------------------------------------------------//
bool TorchHeightController::disable()
{
  if(stepper.position(Z_AXIS) == _safe_pos)
  {
    // Z have stopped, give back Z control to stepper class
    current_position[Z_AXIS] = _safe_pos / planner.axis_steps_per_mm[Z_AXIS];
    planner.set_z_position_step(_safe_pos);
    stepper.take_control_on(Z_AXIS);
    return true;
  }
  else
  {
    return false;
  }
}
//----------------------------------------------------------------------------//
void TorchHeightController::_step_to_safe_pos()
{
  long z_pos = stepper.position(Z_AXIS);
  // Z is far enough from top, use max speed
  if(_safe_pos - z_pos > _max_stopping_distance)
  {
    _target_speed = PLASMA_MAX_THC_STEP_S;
  }
  else // Z approch safe pos, move one step per update()
  {
    _target_speed = 0;
    // do a step if Z below top
    if(_speed == 0 && z_pos < _safe_pos)
    {
      _dir = 1;
      Z_DIR_WRITE(INVERT_Z_DIR ^ (_dir > 0));
      Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
      delayMicroseconds(2);
      stepper.shift_z_position(_dir);
      Z_STEP_WRITE(INVERT_Z_STEP_PIN);
    }
  }
}
//----------------------------------------------------------------------------//
void TorchHeightController::update(PlasmaState plasma_state)
{
  int32_t voltage_mv = ADS1015_device.read();
  long z_pos = stepper.position(Z_AXIS);

  switch(plasma_state)
  {
    case Off:
    case Ignition:
    case Established:
      _target_speed = 0;
      break;
    case Established_THC:
      // check for software endstop overrun
      if(z_pos > _z_top_pos || z_pos < _z_bottom_pos)
      {
        kill("Stop: Z overrun.");
      }

      //PID--------------//
      if(_counter == 200)
      {
        _counter = 0;
        _new_target_speed = -_new_target_speed;
      }
      else
      {
        _counter++;
      }
      _target_speed = _new_target_speed;
      //--------------//

      _safe_pos = min(_z_top_pos, z_pos + _retract_mm * planner.axis_steps_per_mm[Z_AXIS]);
      break;
    case Slowdown_THC:
      if(z_pos < _safe_pos)
      {
        _step_to_safe_pos();
      }
      break;
  }

  int32_t acc = _target_speed - _speed;

  // don't touch anything, speed is fine
  if(acc == 0)
    return;

  // clip acceleration to _max_acc
  if(acc > _max_acc)
    acc = _max_acc;
  else if(acc < -_max_acc)
    acc = -_max_acc;

  // apply acceleration
  _speed = _speed + acc;

  int16_t max_speed = PLASMA_MAX_THC_STEP_S;
  if(_speed > max_speed)
    _speed = max_speed;
  else if(_speed < -max_speed)
    _speed = -max_speed;

  // compute step direction and frequency
  uint16_t freq;
  if(_speed >= 0)
  {
    freq = _speed;
    _dir = 1;
  }
  else
  {
    freq = -_speed;
    _dir = -1;
  }

  // compute interrupts period for step generation
  uint16_t period;
  if(freq == 0)
    period = 0xFFFF;
  else
    period = min(250000 / freq, 0xFFFF);

  PAUSE_TIMER4;
  Z_DIR_WRITE(INVERT_Z_DIR ^ (_dir > 0));
  uint16_t elapsed = _counting_up ? TCNT4 : ICR4 - TCNT4;
  uint16_t rest = elapsed > period ? 0 : period - elapsed;
  ICR4 = period;
  TCNT4 = _counting_up ? period - rest : rest;
  RESUME_TIMER4;
}
//----------------------------------------------------------------------------//
void TorchHeightController::set_mm_to_retract(unsigned long mm)
{
  _retract_mm = mm;
}
//----------------------------------------------------------------------------//
void TorchHeightController::set_max_acc_step_s2(unsigned long max_acc)
{
  // store acceleration in milliseconds as update will be called at 1kHz
  _max_acc = max_acc / 1000;
  _max_stopping_distance = pow((uint32_t)PLASMA_MAX_THC_STEP_S, 2) / (2 * max_acc) + 10;
}
//----------------------------------------------------------------------------//
void TorchHeightController::_reset_PID()
{
  _new_target_speed = 25000;
  _counter = 100;
}
//----------------------------------------------------------------------------//
ISR(TIMER4_OVF_vect){ TorchHeightController::ovf_isr(); }
void TorchHeightController::ovf_isr()
{
  if(ICR4 == 0xFFFF)
  {
    PAUSE_TIMER4;
  }
  else
  {
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
    _counting_up = true;
    stepper.shift_z_position(_dir);
    delayMicroseconds(2);
    Z_STEP_WRITE(INVERT_Z_STEP_PIN);
  }
}
//----------------------------------------------------------------------------//
ISR(TIMER4_CAPT_vect){ TorchHeightController::capt_isr(); }
void TorchHeightController::capt_isr()
{
  if(ICR4 == 0xFFFF)
  {
    PAUSE_TIMER4;
  }
  else
  {
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
    _counting_up = false;
    stepper.shift_z_position(_dir);
    delayMicroseconds(2);
    Z_STEP_WRITE(INVERT_Z_STEP_PIN);
  }
}
//----------------------------------------------------------------------------//
