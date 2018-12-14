#ifndef PLASMA_H
#define PLASMA_H

#include "enum.h"

class Plasma {

  public:
    void init();
    bool start();
    void stop();
    void stop_after_move();
    void lock();
    void unlock();
    PlasmaState update();
    PlasmaState get_state();

  private:
    static PlasmaState state;
    static bool stop_pending;
};

extern Plasma plasmaManager;

#endif
