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

    def test_clamp_min(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, events[16] + 2)
        self.timeline[3] += 10000

        # motors reach max pos
        max_pos = decoder.when_is_position_reached(self.timeline, 500, 500, 500)
        assert max_pos != -1

        # no moves after clamp abort
        slice = decoder.from_to(self.timeline, max_pos, decoder.end(self.timeline))
        assert max_pos + 300 < (self.timeline[0][-1]/1000)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
for i in range(3):
    p.move_down()
p.click()
for i in range(5):
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
p.wait_ms(500)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
