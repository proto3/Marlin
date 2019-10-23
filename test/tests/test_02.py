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

    def test_M3_no_M5(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, self.events[13] + 2)
        self.timeline[3] += 10000

        # motors reach probing position
        slice = decoder.from_to(self.timeline, self.events[13], self.events[14])
        assert decoder.move_cumul_up(slice, 1) == 1000
        assert decoder.move_cumul_up(slice, 2) == 1000

        # define probing time
        probing_time = decoder.when_is_position_reached(self.timeline, 1000, 1000, 10000)
        assert probing_time != -1

        # z move down
        slice = decoder.from_to(self.timeline, probing_time, self.events[14])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z stopped and move 3mm up
        slice = decoder.from_to(self.timeline, self.events[14], self.events[14] + 2)
        min_pos_time = decoder.when_is_lowest_position_reached(slice, 3)
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[15])
        assert decoder.move_cumul_down(slice, 3) == 0
        assert decoder.move_cumul_up(slice, 3) == 300

        # no plasma before cut
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[15])
        plasma_start = decoder.when_is_highest_position_reached(slice, 3)
        cut_start = self.events[15] + 200
        slice = decoder.from_to(self.timeline, 0, plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # no move during piercing
        slice = decoder.from_to(self.timeline, plasma_start + 1, cut_start)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

        # reach intermediate positions
        slice = decoder.from_to(self.timeline, plasma_start, decoder.end(self.timeline))
        assert decoder.when_is_position_reached(self.timeline, 3000, 1000, None) != -1
        assert decoder.when_is_position_reached(self.timeline, 3000, 3000, None) != -1

        # reach cutoff position
        slice = decoder.from_to(self.timeline, self.events[15], decoder.end(self.timeline))
        plasma_stop = decoder.when_is_position_reached(slice, 0, 0, None)
        assert plasma_stop != -1

        # plasma fired all along and z move enough with THC
        slice = decoder.from_to(self.timeline, plasma_start + 5, plasma_stop)
        assert decoder.plasma_is_always(slice, 'on')
        assert decoder.move_cumul(slice, 3) > 4000

        # no plasma after cut off
        slice = decoder.from_to(self.timeline, plasma_stop + 80, decoder.end(self.timeline))
        assert decoder.plasma_is_always(slice, 'off')

        # final position
        assert decoder.get_position(self.timeline, decoder.end(self.timeline)) == (0, 0, 10000)

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
for i in range(3):
    p.move_down()
p.click()
for i in range(2):
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
p.wait_ms(50)
p.transfer_on()
p.wait_ms(1200)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
