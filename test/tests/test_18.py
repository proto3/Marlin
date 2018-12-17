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

    def test_THC_clamp(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, self.events[24] + 2)
        self.timeline[3] += 10000

        slice = decoder.from_to(self.timeline, self.events[24], self.events[25])
        before_cut = decoder.when_is_position_reached(slice, 1000, 1000, 9800)
        assert before_cut != -1

        slice = decoder.from_to(self.timeline, self.events[24], self.events[25])
        assert not decoder.plasma_is_always(slice, 'off')

        slice = decoder.from_to(self.timeline, self.events[25] + 100, decoder.end(self.timeline))
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
p.wait_ms(800)
p.transfer_on()
p.wait_ms(1000)
p.transfer_off()


events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
