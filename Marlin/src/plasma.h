#ifndef PLASMA_H
#define PLASMA_H

#include "enum.h"

class Plasma {

  public:
    void init();
    bool ignite();
    bool activate_thc();
    void stop();
    void stop_after_move();
    void lock();
    void unlock();
    PlasmaState update();
    PlasmaState get_state();
    bool is_lost();

  private:
    static PlasmaState state;
    static bool stop_pending;
    static bool locked_flag;
    static bool lost_flag;
};

extern Plasma plasmaManager;

#endif
