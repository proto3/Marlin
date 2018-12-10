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

    def test_transfer_loss(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, events[15] + 2)

        # motors reach firing position
        self.plasma_start_1 = decoder.when_is_position_reached(timeline, 500, 500, 0)
        assert self.plasma_start_1 != -1

        # motors reach firing position
        self.plasma_start_2 = decoder.when_is_position_reached(timeline, 4000, 6000, 0)
        assert self.plasma_start_2 != -1

        # motors reach cutoff position
        self.plasma_stop = decoder.when_is_position_reached(timeline, 8000, 5000, 0)
        assert self.plasma_stop != -1

        # plasma on until tranfer loss
        slice = decoder.from_to(self.timeline, self.plasma_start_1 + 5, events[17])
        assert decoder.plasma_is_always(slice, 'on')

        # plasma off at tranfer loss
        slice = decoder.from_to(self.timeline, events[17] + 2, events[22])
        assert decoder.plasma_is_always(slice, 'off')

        # plasma restart on resume
        slice = decoder.from_to(self.timeline, self.plasma_start_2 + 70, self.plasma_stop)
        assert decoder.plasma_is_always(slice, 'on')

        # plasma stop after cutoff
        slice = decoder.from_to(self.timeline, self.plasma_stop + 2, decoder.end(timeline))
        assert decoder.plasma_is_always(slice, 'off')

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
p.wait_ms(100)
p.transfer_on()
p.wait_ms(400)
p.transfer_off()
p.wait_ms(400)
p.click()
p.move_down()
p.move_down()
p.move_down()
p.click()
p.wait_ms(250)
p.transfer_on()
p.wait_ms(450)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
