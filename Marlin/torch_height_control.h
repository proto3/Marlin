#ifndef TORCH_HEIGHT_CONTROL_H
#define TORCH_HEIGHT_CONTROL_H

#include "Marlin.h"

class TorchHeightController {
  public:
    static void enable();
    static void disable();
    static void update();

  private:
    static bool _enabled;
};

extern TorchHeightController torchHeightController;

#endif
