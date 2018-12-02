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

    def test_plasma_timeout(self):
        # motors reach firing position
        self.plasma_start = decoder.when_is_position_reached(timeline, 500, 500, 0)
        assert self.plasma_start != -1

        # motors reach cutoff position
        self.plasma_stop = decoder.when_is_position_reached(timeline, 1000, 1000, 0)
        assert self.plasma_stop != -1

        # plasma during 3sec
        slice = decoder.from_to(self.timeline, self.plasma_start + 4, self.plasma_start + 2990)
        assert decoder.plasma_is_always(slice, 'on')

        # plasma stop when timeout
        slice = decoder.from_to(self.timeline,self.plasma_start + 3010, self.plasma_start + 4000)
        assert decoder.plasma_is_always(slice, 'off')

        # plasma restart on resume
        slice = decoder.from_to(self.timeline, events[12], self.plasma_stop)
        assert decoder.plasma_is_always(slice, 'on')

        # plasma stop after cutoff
        slice = decoder.from_to(self.timeline, self.plasma_stop + 1, decoder.end(timeline))
        assert decoder.plasma_is_always(slice, 'off')


export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
p.move_down()
p.move_down()
p.move_down()
p.click()
p.move_down()
p.click()
p.wait_ms(3100)
p.click()
p.move_down()
p.move_down()
p.move_down()
p.click()
p.transfer_on()
p.wait_ms(350)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
