#ifndef TORCH_HEIGHT_CONTROL_H
#define TORCH_HEIGHT_CONTROL_H

#include "Marlin.h"

class TorchHeightController {
  public:
    static void init();
    static void enable();
    static void disable();
    static bool is_enabled();
    static bool is_disabling();
    static void update(bool);
    static void set_max_acc_step_s2(unsigned long);

    static void ovf_isr();
    static void capt_isr();

  private:
    static void _reset_PID();

    static bool _enabled;
    static bool _disabling;
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
