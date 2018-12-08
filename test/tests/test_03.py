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

    def test_M3_G4(self):
        # motors reach firing position
        self.plasma_start = decoder.when_is_position_reached(timeline, 500, 500, 0)
        assert self.plasma_start != -1

        # motors reach cutoff position
        self.plasma_stop = decoder.when_is_position_reached(timeline, 1000, 1000, 0)
        assert self.plasma_stop != -1

        # no plasma before cut
        slice = decoder.from_to(self.timeline, 0, self.plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # plasma fired all along
        slice = decoder.from_to(self.timeline, self.plasma_start + 5, self.plasma_stop)
        assert decoder.plasma_is_always(slice, 'on')

        # no plasma after cut off
        slice = decoder.from_to(self.timeline, self.plasma_stop + 2, decoder.end(timeline))
        assert decoder.plasma_is_always(slice, 'off')

        # no move during G4
        slice = decoder.from_to(self.timeline, self.events[9] + 5, self.events[9] + 500 - 5)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

        # move after G4
        slice = decoder.from_to(self.timeline, self.events[9] + 500 + 5, self.events[9] + 520)
        assert decoder.move_cumul(slice, 1) > 0

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
p.click()
p.transfer_on()
p.wait_ms(900)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
