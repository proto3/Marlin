#ifndef PLASMA_H
#define PLASMA_H

#include "enum.h"

class Plasma {

  public:
    void init();
    bool start();
    void stop();
    void lock();
    void unlock();
    PlasmaState update_state();
    PlasmaState get_state();

  private:
    static PlasmaState state;
};

extern Plasma plasmaManager;

#endif
