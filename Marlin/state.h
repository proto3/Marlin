#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

//TODO Marlin.h not necessary but enum.h doesn't include its own dependencies...
#include "Marlin.h"
#include "enum.h"

#define IS_WAITING_FILE  stateManager.get_state() == waiting_file
#define IS_RUNNING       stateManager.get_state() == running
#define IS_PAUSE_PENDING stateManager.get_state() == pause_pending
#define IS_SUSPENDED     stateManager.get_state() == suspended

class State {

  public:
    bool start(const char *name);
    bool stop();
    bool suspend();
    bool resume();
    bool set_pending_pause();
    bool suspend_if_pending();
    MachineState get_state();

  private:
    static MachineState state;
};

extern State stateManager;

#endif
