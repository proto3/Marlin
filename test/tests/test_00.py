#!/usr/bin/env python3
import os
import utils.player as player
import utils.decoder as decoder

class Test:
    def setup(self):
        global events
        global timeline
        self.events = events
        self.timeline = timeline

    # z move up only
    def test_z_move_up(self):
        slice = decoder.from_to(self.timeline, 0, self.events[5])
        assert decoder.move_cumul_down(slice, 3) == 0
        assert decoder.move_diff(slice, 3) > 100

    # z stop has stopped moving up 2ms after enstop
    def test_z_stop_1(self):
        slice = decoder.from_to(self.timeline, self.events[5] + 1, self.events[5] + 100)
        assert decoder.move_cumul_up(slice, 3) == 0

    # z move down 500 and move up
    def test_z_back(self):
        slice = decoder.from_to(self.timeline, self.events[5], self.events[6])
        assert decoder.move_cumul_down(slice, 3) == 500
        assert decoder.move_cumul_up(slice, 3) > 100

    # z stop has stopped moving up 2ms after enstop
    def test_z_stop_2(self):
        slice = decoder.from_to(self.timeline, self.events[6] + 1, self.events[6] + 200)
        assert decoder.move_cumul_up(slice, 3) == 0

    # x move down only
    def test_x_move_down(self):
        slice = decoder.from_to(self.timeline, self.events[6], self.events[7])
        assert decoder.move_cumul_up(slice, 1) == 0
        assert decoder.move_diff(slice, 1) < -100

    # x stop has stopped moving down 2ms after enstop
    def test_x_stop_1(self):
        slice = decoder.from_to(self.timeline, self.events[7] + 1, self.events[7] + 100)
        assert decoder.move_cumul_down(slice, 1) == 0

    # x move up 500 and move down
    def test_x_back(self):
        slice = decoder.from_to(self.timeline, self.events[7], self.events[8])
        assert decoder.move_cumul_up(slice, 1) == 500
        assert decoder.move_cumul_down(slice, 1) > 100

    # x stop has stopped moving down 2ms after enstop
    def test_x_stop_2(self):
        slice = decoder.from_to(self.timeline, self.events[8] + 1, self.events[8] + 200)
        assert decoder.move_cumul_down(slice, 1) == 0

    # y move down only
    def test_y_move_down(self):
        slice = decoder.from_to(self.timeline, self.events[8], self.events[9])
        assert decoder.move_cumul_up(slice, 2) == 0
        assert decoder.move_diff(slice, 2) < -100

    # y stop has stopped moving down 2ms after enstop
    def test_y_stop_1(self):
        slice = decoder.from_to(self.timeline, self.events[9] + 1, self.events[9] + 100)
        assert decoder.move_cumul_down(slice, 2) == 0

    # y move up 500 and move down
    def test_y_back(self):
        slice = decoder.from_to(self.timeline, self.events[9], self.events[10])
        assert decoder.move_cumul_up(slice, 2) == 500
        assert decoder.move_cumul_down(slice, 2) > 100

    # y stop has stopped moving down 2ms after enstop
    def test_y_stop_2(self):
        slice = decoder.from_to(self.timeline, self.events[10] + 1, self.events[10] + 200)
        assert decoder.move_cumul_down(slice, 2) == 0

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
p.move_down()
p.click()
p.move_down()
p.click()
p.endstop_z()
p.wait_ms(300)
p.endstop_z()
p.wait_ms(100)
p.endstop_x()
p.wait_ms(300)
p.endstop_x()
p.wait_ms(100)
p.endstop_y()
p.wait_ms(300)
p.endstop_y()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
