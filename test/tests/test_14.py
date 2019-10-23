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

    def test_speed(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, self.events[21] + 2)
        self.timeline[3] += 10000

        slice = decoder.from_to(self.timeline, self.events[22], self.events[23])
        assert decoder.plasma_is_always(slice, 'on')
        assert decoder.move_cumul(slice, 3) > 1000

        slice = decoder.from_to(self.timeline, self.events[23] + 2, decoder.end(self.timeline))
        assert decoder.plasma_is_always(slice, 'off')
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
for i in range(3):
    p.move_down()
p.click()
for i in range(10):
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
p.wait_ms(1500)
p.ohmic_probe()
p.wait_ms(100)
p.transfer_on()
p.wait_ms(4000)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
