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

    def test_kill_switch(self):
        # motors reach firing position
        self.plasma_start = decoder.when_is_position_reached(timeline, 500, 500, 0)
        assert self.plasma_start != -1

        # no plasma before cut
        slice = decoder.from_to(self.timeline, 0, self.plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # plasma fired all along
        slice = decoder.from_to(self.timeline, self.plasma_start + 4, events[8])
        assert decoder.plasma_is_always(slice, 'on')

        # everything has stopped after kill
        slice = decoder.from_to(self.timeline, events[8] + 1, decoder.end(timeline))
        assert decoder.plasma_is_always(slice, 'off')
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
p.click()
p.transfer_on()
p.wait_ms(150)
p.kill()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
