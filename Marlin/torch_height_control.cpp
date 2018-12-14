#include "torch_height_control.h"
#include "stepper.h"

bool TorchHeightController::_enabled = false;
//----------------------------------------------------------------------------//
void TorchHeightController::enable()
{
  stepper.leave_control_on(Z_AXIS);
  _enabled = true;
}
//----------------------------------------------------------------------------//
void TorchHeightController::disable()
{
  _enabled = false;
  planner.set_z_position_step(stepper.position(Z_AXIS));
  stepper.take_control_on(Z_AXIS);
}
//----------------------------------------------------------------------------//
