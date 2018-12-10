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
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, events[12] + 2)

        # motors reach firing position
        self.plasma_start = decoder.when_is_position_reached(timeline, 500, 500, 0)
        assert self.plasma_start != -1

        # no plasma before cut
        slice = decoder.from_to(self.timeline, 0, self.plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # plasma fired all along
        slice = decoder.from_to(self.timeline, self.plasma_start + 70, events[14])
        assert decoder.plasma_is_always(slice, 'on')

        # everything has stopped after kill
        slice = decoder.from_to(self.timeline, events[14] + 1, decoder.end(timeline))
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
p.wait_ms(200)
p.transfer_on()
p.wait_ms(150)
p.kill()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
