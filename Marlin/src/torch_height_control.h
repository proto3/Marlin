#ifndef TORCH_HEIGHT_CONTROL_H
#define TORCH_HEIGHT_CONTROL_H

#include "Marlin.h"

class TorchHeightController {
  public:
    static void init();
    static void enable();
    static bool disable();
    static void set_mm_to_retract(uint32_t);
    static void set_max_acc_step_s2(uint32_t);

    static void update(PlasmaState);
    static void isr();

    static int16_t _target_voltage;
    static float _voltage;

  private:
    static void _reset_PID();
    static void _step_to_safe_pos();

    static int32_t _target_speed;
    static int16_t _speed;
    static int16_t _max_acc;
    static int8_t _dir;
    static uint32_t _retract_mm;
    static uint16_t _max_stopping_distance;
    static int32_t _z_top_pos;
    static int32_t _z_bottom_pos;
    static int32_t _safe_pos;
    static uint16_t target_modulus;
    static bool pending_modulus_update;

    static int16_t _new_target_speed;
    static int16_t _counter;
};

extern TorchHeightController torchHeightController;

#endif
