#ifndef _FASTIO_ARDUINO_H
#define _FASTIO_ARDUINO_H

#include <Arduino.h>

#define SET_INPUT(IO) if(IO != -1) {pinMode(IO, INPUT);}
#define SET_INPUT_PULLUP(IO) if(IO != -1) {pinMode(IO, INPUT_PULLUP);}
#define SET_OUTPUT(IO) if(IO != -1) {pinMode(IO, OUTPUT);}

#define READ(IO) (IO != -1 ? digitalRead(IO) : LOW)
#define WRITE(IO, v) if(IO != -1) {digitalWrite(IO, v);}
#define OUT_WRITE(IO, v) if(IO != -1) {pinMode(IO, OUTPUT); digitalWrite(IO, v);}

#define TOGGLE(IO) WRITE(IO, !READ(IO))

#endif /* _FASTIO_ARDUINO_H */
