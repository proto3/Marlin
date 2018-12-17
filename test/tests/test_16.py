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

    def test_M6_no_M3_and_G28_with_thc(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, events[22] + 2)
        self.timeline[3] += 10000

        # no plasma
        assert decoder.plasma_is_always(self.timeline, 'off')

        slice = decoder.from_to(self.timeline, events[22], decoder.end(timeline))
        a = decoder.when_is_position_reached(slice, 1000, 1000, 5000)
        assert a != -1

        slice = decoder.from_to(self.timeline, a, decoder.end(timeline))
        b = decoder.when_is_position_reached(slice, 2000, 3000, 5000)
        assert b != -1

        slice = decoder.from_to(self.timeline, b, decoder.end(timeline))
        c = decoder.when_is_position_reached(slice, 1000, 1500, 6000)
        assert c != -1

        slice = decoder.from_to(self.timeline, a, b)
        assert decoder.move_cumul(slice, 3) == 0

        slice = decoder.from_to(self.timeline, c, decoder.end(timeline))
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
p.move_down()
p.move_down()
p.move_down()
p.click()
p.move_down()
p.move_down()
p.move_down()
p.move_down()
p.move_down()
p.move_down()
p.move_down()
p.move_down()
p.move_down()
p.move_down()
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

p.wait_ms(5000)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
