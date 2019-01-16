#!/usr/bin/env python3
import os, sys, time
import serial
import saleae
from enum import Enum

class Port(Enum):
    B = 1
    D = 2

class Player:
    clock_period_ms = 10
    portb = 255
    portd = 255

    def __init__(self, output_basename, device='/dev/ttyUSB0', baudrate=115200, use_cache=False):
        self.output_basename = output_basename
        self.events = list()
        self.next_slot = 0
        self.nb_step_loaded = -1
        self.event_count = 0
        self.use_cache = use_cache

        if(self.use_cache):
            return

        # try to connect on serial port
        try:
            self.ser = serial.Serial(device, baudrate, timeout = None)
        except Exception as e:
            print('Serial player not connected.', file=sys.stderr)
            sys.exit(0)

        # wait for ready message
        self._receive()

        # reset target board
        self.reset_target()
        self._send("start")
        time.sleep(4)
        self._reset_player()

        # launch logic if needed
        self.logic_was_running = saleae.Saleae.is_logic_running()
        self.s = saleae.Saleae(quiet=True)

        # check saleae device is connected
        if(len(self.s.get_connected_devices()) == 4):
            self.ser.close()
            print('Saleae device not connected.', file=sys.stderr)
            sys.exit(0)

        # configure capture
        self.s.set_trigger_one_channel(0, saleae.Trigger.Posedge)
        self.s.set_sample_rate((1000000, 0))

        # prepare event file
        self.event_file = open(self.output_basename + '.event', "w")

        self.nb_step_loaded = 0

    def _send(self, str):
        self.ser.write((str+'\n').encode('utf-8'))
        time.sleep(0.01)

    def _receive(self):
        return self.ser.readline().decode('utf-8')

    def _reset_player(self):
        self._send("reset")
        self.next_slot = 0

    def _set_pin(self, port, pin, val):
        if not val is None:
            if(port == Port.B):
                if(val):
                    self.portb |= 1 << pin
                else:
                    self.portb &= ~(1 << pin)
            elif(port == Port.D):
                if(val):
                    self.portd |= 1 << pin
                else:
                    self.portd &= ~(1 << pin)

    def _add_step(self, timestamp,
        rot_push = None,
        rot_a    = None,
        rot_b    = None,
        kill     = None,
        reset    = None,
        end_x    = None,
        end_y    = None,
        end_z    = None,
        transfer = None,
        ohm_pr   = None,
        t_mnt    = None):
        self.nb_step_loaded += 1

        self._set_pin(Port.B, 0, rot_push)
        self._set_pin(Port.B, 2, rot_a)
        self._set_pin(Port.B, 1, rot_b)
        self._set_pin(Port.B, 3, kill)
        self._set_pin(Port.B, 4, reset)

        self._set_pin(Port.D, 2, ohm_pr)
        self._set_pin(Port.D, 3, end_z)
        self._set_pin(Port.D, 4, end_y)
        self._set_pin(Port.D, 5, end_x)
        self._set_pin(Port.D, 6, t_mnt)
        self._set_pin(Port.D, 7, transfer)

        if(not self.use_cache):
            self._send(str(timestamp)  + " " + str(self.portb) + " " + str(self.portd))
            time.sleep(0.1)

    def run(self):
        if(self.use_cache):
            return self.events

        duration = float(self.next_slot) / 100
        self.s.set_capture_seconds(duration + 0.5)
        self.s.capture_start()

        #leave some time for capture to start
        time.sleep(0.5)

        self.ser.reset_input_buffer()
        self._send("start")
        assert self.nb_step_loaded == int(self._receive())

        time.sleep(duration)

        done = False
        for i in range(5):
            if not self.s.is_processing_complete():
                time.sleep(0.5)
            else:
                done = True
                break

        if(not done):
            self.s.capture_stop()
            print("ERROR : Capture has not completed in time.")
            sys.exit(0)

        save_path = os.path.abspath(self.output_basename + '.bin')
        self.s.export_data2(save_path, format='binary', each_sample=False)

        if not self.logic_was_running:
            saleae.Saleae.kill_logic()

        return self.events

    def close(self):
        if(not self.use_cache):
            self.ser.close()
            self.event_file.close()

    def add_event(self, event):
        self.events.append(self.next_slot * 10)
        if(not self.use_cache):
            self.event_file.write(str(self.next_slot) + " [" + str(self.event_count) + "] " + event + "\n")
        self.event_count += 1

    def wait_ms(self, t):
        self.next_slot += int(t / self.clock_period_ms)

    def wait_until_ms(self, t):
        target = int(t / self.clock_period_ms)
        if(self.next_slot < target):
            self.next_slot = target

    def reset_target(self):
        self._add_step(self.next_slot,   reset=0)
        self._add_step(self.next_slot+1, reset=1)
        self.next_slot += 600

    def kill(self):
        self.add_event("kill")
        self._add_step(self.next_slot,   kill=0)
        self._add_step(self.next_slot+1, kill=1)
        self.next_slot += 1

    def click(self):
        self.add_event("click")
        self._add_step(self.next_slot,   rot_push=0)
        self._add_step(self.next_slot+20, rot_push=1)
        self.next_slot += 30

    def move_up(self):
        self.add_event("up")
        self._add_step(self.next_slot,   rot_a=0)
        self._add_step(self.next_slot+1, rot_b=0)
        self._add_step(self.next_slot+2, rot_a=1)
        self._add_step(self.next_slot+3, rot_b=1)
        self.next_slot += 30

    def move_down(self):
        self.add_event("down")
        self._add_step(self.next_slot,   rot_b=0)
        self._add_step(self.next_slot+1, rot_a=0)
        self._add_step(self.next_slot+2, rot_b=1)
        self._add_step(self.next_slot+3, rot_a=1)
        self.next_slot += 30

    def endstop_x(self):
        self.add_event("end_x")
        self._add_step(self.next_slot,   end_x=0)
        self._add_step(self.next_slot+1, end_x=1)
        self.next_slot += 2

    def endstop_y(self):
        self.add_event("end_y")
        self._add_step(self.next_slot,   end_y=0)
        self._add_step(self.next_slot+1, end_y=1)
        self.next_slot += 2

    def endstop_z(self):
        self.add_event("end_z")
        self._add_step(self.next_slot,   end_z=0)
        self._add_step(self.next_slot+1, end_z=1)
        self.next_slot += 2

    def transfer_on(self):
        self.add_event("transfer")
        self._add_step(self.next_slot, transfer=0)
        self.next_slot += 1

    def transfer_off(self):
        self.add_event("transfer loss")
        self._add_step(self.next_slot, transfer=1)
        self.next_slot += 1

    def ohmic_probe(self):
        self.add_event("ohmic probe")
        self._add_step(self.next_slot, ohm_pr=0)
        self.next_slot += 10
        self._add_step(self.next_slot, ohm_pr=1)
        self.next_slot += 1

    def torch_dismount(self):
        self.add_event("torch dismount")
        self._add_step(self.next_slot, t_mnt=0)
        self.next_slot += 1

    def torch_mount(self):
        self.add_event("torch mount")
        self._add_step(self.next_slot, t_mnt=1)
        self.next_slot += 1
