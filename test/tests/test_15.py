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

    def test_M6_no_G28(self):
        # plasma on start
        slice = decoder.from_to(self.timeline, self.events[15], self.events[16])
        assert not decoder.plasma_is_always(slice, 'off')

        # plasma stop
        slice = decoder.from_to(self.timeline, self.events[16] + 2, decoder.end(self.timeline))
        assert decoder.plasma_is_always(slice, 'off')

        # nothing moves
        assert decoder.move_cumul(self.timeline, 1) == 0
        assert decoder.move_cumul(self.timeline, 2) == 0
        assert decoder.move_cumul(self.timeline, 3) == 0

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
p.click()
p.wait_ms(100)
p.transfer_on()
p.wait_ms(1000)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
