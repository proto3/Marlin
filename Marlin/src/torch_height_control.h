#ifndef TORCH_HEIGHT_CONTROL_H
#define TORCH_HEIGHT_CONTROL_H

#include "Marlin.h"

class TorchHeightController {
  public:
    static void init();
    static void enable();
    static bool disable();
    static void set_max_acc_step_s2(unsigned long);

    static void update(PlasmaState);
    static void ovf_isr();
    static void capt_isr();

  private:
    static void _reset_PID();
    static void _step_to_safe_pos();

    static bool _counting_up;
    static int32_t _target_speed;
    static int16_t _speed;
    static int16_t _max_acc;
    static int8_t _dir;
    static uint16_t _max_stopping_distance;
    static long _z_top_pos;
    static long _z_bottom_pos;

    static int16_t _new_target_speed;
    static int16_t _counter;
};

extern TorchHeightController torchHeightController;

#endif
