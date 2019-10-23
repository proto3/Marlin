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

    def test_pause(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, self.events[14] + 2)
        self.timeline[3] += 10000

        # motors reach probing position
        slice = decoder.from_to(self.timeline, self.events[14], self.events[18])
        assert decoder.move_cumul_up(slice, 1) == 500
        assert decoder.move_cumul_up(slice, 2) == 500

        # define probing time
        probing_time = decoder.when_is_position_reached(self.timeline, 500, 500, 10000)
        assert probing_time != -1

        # z move down
        slice = decoder.from_to(self.timeline, probing_time, self.events[18])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z stopped and move 3mm up
        slice = decoder.from_to(self.timeline, self.events[18], self.events[18] + 2)
        min_pos_time = decoder.when_is_lowest_position_reached(slice, 3)
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[19])
        assert decoder.move_cumul_down(slice, 3) == 0
        assert decoder.move_cumul_up(slice, 3) == 300

        # no plasma before cut
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[19])
        plasma_start = decoder.when_is_highest_position_reached(slice, 3)
        cut_start = self.events[19] + 200
        slice = decoder.from_to(self.timeline, 0, plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # no move during piercing
        slice = decoder.from_to(self.timeline, plasma_start + 1, cut_start)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

        # reach cutoff position
        plasma_stop = decoder.when_is_position_reached(self.timeline, 5000, 3000, None)
        assert plasma_stop != -1

        # plasma fired all along and z move enough with THC
        slice = decoder.from_to(self.timeline, plasma_start + 5, plasma_stop)
        assert decoder.plasma_is_always(slice, 'on')
        assert decoder.move_cumul(slice, 3) > 3000

        # wait for resume
        slice = decoder.from_to(self.timeline, plasma_stop, self.events[22])
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0

        # z goes back to top
        slice = decoder.from_to(self.timeline, plasma_stop, self.events[22])
        assert decoder.max_pos(slice, 3) == 10000

        # define probing time
        probing_time = decoder.when_is_position_reached(self.timeline, 4000, 6000, 10000)
        assert probing_time != -1

        # z move down
        slice = decoder.from_to(self.timeline, probing_time, self.events[23])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z stopped and move 3mm up
        slice = decoder.from_to(self.timeline, self.events[23], self.events[23] + 2)
        min_pos_time = decoder.when_is_lowest_position_reached(slice, 3)
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[24])
        assert decoder.move_cumul_down(slice, 3) == 0
        assert decoder.move_cumul_up(slice, 3) == 300

        # no plasma before cut
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[24])
        plasma_start = decoder.when_is_highest_position_reached(slice, 3)
        cut_start = self.events[24] + 200
        slice = decoder.from_to(self.timeline, self.events[21], plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # no move during ignition
        cut_start = self.events[24] + 200
        slice = decoder.from_to(self.timeline, self.events[24], cut_start)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

        # plasma stop after cutoff
        slice = decoder.from_to(self.timeline, self.events[25], decoder.end(self.timeline))
        assert decoder.plasma_is_always(slice, 'off')

        # final position
        assert decoder.get_position(self.timeline, decoder.end(self.timeline)) == (0, 0, 10000)

export_basename = 'tmp/' + os.path.splitext(os.path.basename(__file__))[0]
p = player.Player(export_basename)

p.click()
for i in range(3):
    p.move_down()
p.click()
for i in range(3):
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
p.click()
p.move_down()
p.move_down()
p.wait_ms(100)
p.ohmic_probe()
p.wait_ms(100)
p.transfer_on()
p.wait_ms(400)
p.click()
p.wait_ms(150)
p.transfer_off()
p.wait_ms(1500)

# resume
p.click()

p.wait_ms(1000)
p.ohmic_probe()
p.wait_ms(100)
p.transfer_on()
p.wait_ms(700)
p.transfer_off()
p.wait_ms(1000)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
