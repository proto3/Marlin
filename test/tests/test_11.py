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

    def test_unhomed_M3(self):
        # plasma off
        assert decoder.plasma_is_always(self.timeline, 'off')

        # nothing moves
        assert decoder.move_cumul(self.timeline, 1) == 0
        assert decoder.move_cumul(self.timeline, 2) == 0
        assert decoder.move_cumul(self.timeline, 3) == 0

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
for i in range(3):
    p.move_down()
p.click()
for i in range(7):
    p.move_down()
p.click()
p.wait_ms(700)
p.ohmic_probe()
p.wait_ms(100)
p.transfer_on()
p.wait_ms(100)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
