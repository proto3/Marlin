#include "state.h"
#include "plasma.h"
#include "cardreader.h"

MachineState State::state = waiting_file;

//----------------------------------------------------------------------------//
bool State::start(const char *name)
{
  CRITICAL_SECTION_START
  if(state != waiting_file || !card.openFile(name, true))
  {
    CRITICAL_SECTION_END
    return false;
  }

  print_job_timer.start();
  plasmaManager.unlock();
  state = running;

  CRITICAL_SECTION_END
  return true;
}
//----------------------------------------------------------------------------//
bool State::stop()
{
  CRITICAL_SECTION_START
  if(state == waiting_file)
  {
    CRITICAL_SECTION_END
    return false;
  }

  plasmaManager.lock();
  quickstop_stepper();
  print_job_timer.stop();
  card.closefile(false);
  state = waiting_file;

  CRITICAL_SECTION_END
  return true;
}
//----------------------------------------------------------------------------//
bool State::suspend()
{
  CRITICAL_SECTION_START
  if(state != running && state != pause_pending)
  {
    CRITICAL_SECTION_END
    return false;
  }

  plasmaManager.lock();
  print_job_timer.pause();
  state = suspended;

  CRITICAL_SECTION_END
  return true;
}
//----------------------------------------------------------------------------//
bool State::resume()
{
  CRITICAL_SECTION_START
  if(state != suspended)
  {
    CRITICAL_SECTION_END
    return false;
  }

  plasmaManager.unlock();
  print_job_timer.start();
  state = running;

  CRITICAL_SECTION_END
  return true;
}
//----------------------------------------------------------------------------//
bool State::set_pending_pause()
{
  CRITICAL_SECTION_START
  if(state != running)
  {
    CRITICAL_SECTION_END
    return false;
  }

  state = pause_pending;

  CRITICAL_SECTION_END
  return true;
}
//----------------------------------------------------------------------------//
bool State::suspend_if_pending()
{
  CRITICAL_SECTION_START
  if(state != pause_pending)
  {
    CRITICAL_SECTION_END
    return false;
  }

  suspend();

  CRITICAL_SECTION_END
  return true;
}
//----------------------------------------------------------------------------//
MachineState State::get_state()
{
  return state;
}
//----------------------------------------------------------------------------//
