#include "torch_height_control.h"
#include "ADS1015.h"
#include "stepper.h"

#define PAUSE_TIMER   FTM2_SC = 0x00
#define RESUME_TIMER  FTM2_SC = (FTM_SC_CLKS(0b1) & FTM_SC_CLKS_MASK) | (FTM_SC_PS(FTM2_TIMER_PRESCALE_BITS) & FTM_SC_PS_MASK) | FTM_SC_TOIE

int16_t TorchHeightController::_target_voltage = 130;
float TorchHeightController::_voltage = -1.0;
int32_t TorchHeightController::_target_speed = 0;
int16_t TorchHeightController::_speed = 0;
int16_t TorchHeightController::_max_acc = 0;
int8_t TorchHeightController::_dir = 1;
uint16_t TorchHeightController::_max_stopping_distance = 0xFFFF;
uint32_t TorchHeightController::_retract_mm = PLASMA_THC_RETRACT_MM;
int32_t TorchHeightController::_z_top_pos = 0;
int32_t TorchHeightController::_z_bottom_pos = 0;
int32_t TorchHeightController::_safe_pos = 0;
uint16_t TorchHeightController::target_modulus = 0xFFFF;
bool TorchHeightController::pending_modulus_update = false;

int16_t TorchHeightController::_new_target_speed = 25000;
int16_t TorchHeightController::_counter = 0;

//----------------------------------------------------------------------------//
void TorchHeightController::init()
{
  ADS1015_device.init();

  _z_top_pos = sw_endstop_max[Z_AXIS] * planner.axis_steps_per_mm[Z_AXIS];
  _z_bottom_pos = 0;

  HAL_timer_start(THC_TIMER_NUM, 0x1234);
  HAL_timer_set_compare(THC_TIMER_NUM, 0xFFFF);
  ENABLE_THC_INTERRUPT();
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
  int32_t z_pos = stepper.position(Z_AXIS);
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
      delayMicroseconds(3);
      stepper.shift_z_position(_dir);
      Z_STEP_WRITE(INVERT_Z_STEP_PIN);
    }
  }
}
//----------------------------------------------------------------------------//
void TorchHeightController::update(PlasmaState plasma_state)
{
  int32_t voltage_mv = ADS1015_device.read();
  int32_t z_pos = stepper.position(Z_AXIS);

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
    period = min(THC_TIMER_RATE / freq, 0xFFFF);

  PAUSE_TIMER;
  Z_DIR_WRITE(INVERT_Z_DIR ^ (_dir > 0));
  uint16_t elapsed = FTM2_CNT - 1;
  uint16_t rest = elapsed > period ? 0 : period - elapsed;

  if(rest == 0)
  {
    target_modulus = period;
    pending_modulus_update = true;
    FTM2_MOD = FTM2_CNT;
    FTM2_CNT = 0x1234;
  }
  else
  {
    FTM2_MOD = period;
  }

  RESUME_TIMER;
}
//----------------------------------------------------------------------------//
void TorchHeightController::set_mm_to_retract(uint32_t mm)
{
  _retract_mm = mm;
}
//----------------------------------------------------------------------------//
void TorchHeightController::set_max_acc_step_s2(uint32_t max_acc)
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
HAL_THC_TIMER_ISR {
  HAL_timer_isr_prologue(THC_TIMER_NUM);
  TorchHeightController::isr();
}
void TorchHeightController::isr()
{
  if(FTM2_MOD == 0xFFFF)
  {
      PAUSE_TIMER;
  }
  else
  {
    if(pending_modulus_update)
    {
      PAUSE_TIMER;
      FTM2_MOD = target_modulus;
      pending_modulus_update = false;
      RESUME_TIMER;
    }
    Z_STEP_WRITE(!INVERT_Z_STEP_PIN);
    stepper.shift_z_position(_dir);
    delayMicroseconds(3);
    Z_STEP_WRITE(INVERT_Z_STEP_PIN);
  }
}
//----------------------------------------------------------------------------//
