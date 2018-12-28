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

    def test_no_M7_no_M5(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, self.events[23] + 2)
        self.timeline[3] += 10000

        slice = decoder.from_to(self.timeline, self.events[24], self.events[24] + 800)
        assert decoder.plasma_is_always(slice, 'on')
        assert decoder.move_cumul(slice, 3) > 4000

        slice = decoder.from_to(self.timeline, self.events[25], self.events[25] + 1000)
        after_cut = decoder.when_is_position_reached(slice, 2000, 3000, 6000)
        assert after_cut != -1

        slice = decoder.from_to(self.timeline, self.events[25], self.events[26])
        assert decoder.max_pos(slice, 3) == 6000

        slice = decoder.from_to(self.timeline, after_cut, self.events[26])
        assert decoder.move_cumul_up(slice, 3) == 0
        assert decoder.move_cumul_down(slice, 3) == 1000

        slice = decoder.from_to(self.timeline, self.events[26], self.events[26] + 800)
        assert decoder.plasma_is_always(slice, 'on')
        assert decoder.move_cumul(slice, 3) > 4000

        slice = decoder.from_to(self.timeline, self.events[27], decoder.end(self.timeline))
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0
        assert slice[1][-1] == 2000
        assert slice[2][-1] == 3000

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
p.wait_ms(2200)
p.transfer_on()
p.wait_ms(1000)
p.transfer_off()
p.wait_ms(1200)
p.transfer_on()
p.wait_ms(1000)
p.transfer_off()


events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
