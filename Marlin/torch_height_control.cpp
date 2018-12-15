#include "torch_height_control.h"
#include "stepper.h"
#include "temperature.h"

#define PAUSE_TIMER4  TCCR4B = _BV(WGM43);
#define RESUME_TIMER4 TCCR4B = _BV(WGM43) | _BV(CS41) | _BV(CS40);

bool TorchHeightController::_enabled = false;
bool TorchHeightController::_disabling = false;
bool TorchHeightController::_counting_up = true;
int32_t TorchHeightController::_target_speed = 0;
int16_t TorchHeightController::_speed = 0;
int16_t TorchHeightController::_max_acc = 0;
int8_t TorchHeightController::_dir = 1;
uint16_t TorchHeightController::_max_stopping_distance = 0xFFFF;
long TorchHeightController::_z_top_pos = 0;
long TorchHeightController::_z_bottom_pos = 0;

int16_t TorchHeightController::_new_target_speed = 25000;
int16_t TorchHeightController::_counter = 0;

//----------------------------------------------------------------------------//
void TorchHeightController::init()
{
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

  _enabled = true;
  _disabling = false;
}
//----------------------------------------------------------------------------//
void TorchHeightController::disable()
{
  _enabled = false;
  _disabling = true;
}
//----------------------------------------------------------------------------//
bool TorchHeightController::is_disabling()
{
  return _disabling;
}
//----------------------------------------------------------------------------//
void TorchHeightController::update(bool plasma_on)
{
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

  if(_disabling && _speed == 0)
  {
    planner.set_z_position_step(stepper.position(Z_AXIS));
    stepper.take_control_on(Z_AXIS);
    _disabling = false;
  }

  if(!_enabled)
  {
    _target_speed = 0;
  }
  else
  {
    long z_pos = stepper.position(Z_AXIS);
    if(z_pos > _z_top_pos || z_pos < _z_bottom_pos)
    {
      kill("Stop: Z overrun.");
    }
    if(!plasma_on)
    {
      if(_z_top_pos - z_pos < _max_stopping_distance)
      {
        _target_speed = 0;
        if(_speed == 0 && z_pos < _z_top_pos)
        {
          _dir = 1;
          Z_DIR_WRITE(INVERT_Z_DIR ^ (_dir > 0));
          Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
          delayMicroseconds(2);
          Z_STEP_WRITE(INVERT_Z_STEP_PIN);
          stepper.shift_z_position(_dir);
        }
      }
      else
      {
        _target_speed = PLASMA_MAX_THC_STEP_S;
      }
    }
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
void TorchHeightController::set_max_acc_step_s2(unsigned long max_acc)
{
  // store acceleration is milliseconds as update will be called at 1kHz
  _max_acc = max_acc / 1000;

  unsigned long max_freq = PLASMA_MAX_THC_STEP_S;
  _max_stopping_distance = pow(max_freq, 2) / max_acc;
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
    _counting_up = true;
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
    delayMicroseconds(2);
    Z_STEP_WRITE(INVERT_Z_STEP_PIN);
    stepper.shift_z_position(_dir);
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
    _counting_up = false;
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
    delayMicroseconds(2);
    Z_STEP_WRITE(INVERT_Z_STEP_PIN);
    stepper.shift_z_position(_dir);
  }
}
//----------------------------------------------------------------------------//
