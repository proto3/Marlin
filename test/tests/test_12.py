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

    def test_G28_during_cut(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, self.events[19] + 2)
        self.timeline[3] += 10000

        # motors reach probing position
        slice = decoder.from_to(self.timeline, self.events[19], self.events[20])
        assert decoder.move_cumul_up(slice, 1) == 1000
        assert decoder.move_cumul_up(slice, 2) == 1500

        # define probing time
        probing_time = decoder.when_is_position_reached(self.timeline, 1000, 1500, 10000)
        assert probing_time != -1

        # z move down
        slice = decoder.from_to(self.timeline, probing_time, self.events[20])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z stopped and move 3mm up
        slice = decoder.from_to(self.timeline, self.events[13], self.events[20] + 2)
        min_pos_time = decoder.when_is_lowest_position_reached(slice, 3)
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[21])
        assert decoder.move_cumul_down(slice, 3) == 0
        assert decoder.move_cumul_up(slice, 3) == 300

        # no plasma before cut
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[21])
        plasma_start = decoder.when_is_highest_position_reached(slice, 3)
        cut_start = self.events[21] + 200
        slice = decoder.from_to(self.timeline, 0, plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # no move during piercing
        slice = decoder.from_to(self.timeline, plasma_start + 1, cut_start - 1)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0
        assert decoder.plasma_is_always(slice, 'on')

        # stop after piercing
        slice = decoder.from_to(self.timeline, cut_start + 50, decoder.end(self.timeline))
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0
        assert decoder.plasma_is_always(slice, 'off')

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename, use_cache=False)

p.click()
for i in range(3):
    p.move_down()
p.click()
for i in range(8):
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
p.wait_ms(700)
p.ohmic_probe()
p.wait_ms(100)
p.transfer_on()
p.wait_ms(5000)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
