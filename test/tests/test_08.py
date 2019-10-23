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

    def test_unhomed_move(self):
        # no moves after clamp abort
        assert events[12] + 300 < (self.timeline[0][-1]/1000)
        assert decoder.move_cumul(self.timeline, 1) == 0
        assert decoder.move_cumul(self.timeline, 2) == 0
        assert decoder.move_cumul(self.timeline, 3) == 0

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
for i in range(3):
    p.move_down()
p.click()
for i in range(6):
    p.move_down()
p.click()
p.transfer_on()
p.wait_ms(300)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
