#ifndef ADS1015_H
#define ADS1015_H

#include "Marlin.h"

class ADS1015 {
  public:
    static void init();
    static int32_t read();

  private:
    typedef enum{ CONV_REQ, READ_REQ, READ_BUF } adc_state_t;
    static adc_state_t adc_state;
    static bool requested;
};

extern ADS1015 ADS1015_device;

#endif
