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
        decoder.apply_autohome(self.timeline, self.events[14] + 2)
        self.timeline[3] += 10000

        # motors reach probing position
        slice = decoder.from_to(self.timeline, self.events[14], self.events[15])
        assert decoder.move_cumul_up(slice, 1) == 500
        assert decoder.move_cumul_up(slice, 2) == 500

        # define probing time
        probing_time = decoder.when_is_position_reached(self.timeline, 500, 500, 10000)
        assert probing_time != -1

        # z move down
        slice = decoder.from_to(self.timeline, probing_time, self.events[15])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z stopped and move 3mm up
        slice = decoder.from_to(self.timeline, self.events[15], self.events[15] + 2)
        min_pos_time = decoder.when_is_lowest_position_reached(slice, 3)
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[16])
        assert decoder.move_cumul_down(slice, 3) == 0
        assert decoder.move_cumul_up(slice, 3) == 300

        # no plasma before cut
        slice = decoder.from_to(self.timeline, min_pos_time, self.events[16])
        plasma_start = decoder.when_is_highest_position_reached(slice, 3)
        cut_start = self.events[16] + 200
        slice = decoder.from_to(self.timeline, 0, plasma_start)
        assert decoder.plasma_is_always(slice, 'off')

        # no move during piercing
        slice = decoder.from_to(self.timeline, plasma_start + 1, cut_start)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

        # plasma until transfer loss
        slice = decoder.from_to(self.timeline, plasma_start + 5, self.events[17])
        assert decoder.plasma_is_always(slice, 'on')
        assert decoder.move_cumul(slice, 3) > 1000

        # plasma stop when transfer lost
        slice = decoder.from_to(self.timeline, self.events[17] + 2, self.events[22])
        assert decoder.plasma_is_always(slice, 'off')

        paused = decoder.when_is_position_reached(self.timeline, 5000, 3000, 10000)
        assert paused != -1

        # define probing time
        probing_time_2 = decoder.when_is_position_reached(self.timeline, 4000, 6000, 10000)
        assert probing_time_2 != -1

        # z move down
        slice = decoder.from_to(self.timeline, probing_time_2, self.events[22])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z stopped and move 3mm up
        slice = decoder.from_to(self.timeline, self.events[22], self.events[22] + 2)
        min_pos_time_2 = decoder.when_is_lowest_position_reached(slice, 3)
        slice = decoder.from_to(self.timeline, min_pos_time_2, self.events[23])
        assert decoder.move_cumul_down(slice, 3) == 0
        assert decoder.move_cumul_up(slice, 3) == 300

        # plasma restart on resume and z move enough with THC
        plasma_stop_2 = decoder.when_is_position_reached(self.timeline, 8000, 5000, None)
        assert plasma_stop_2 != -1
        slice = decoder.from_to(self.timeline, self.events[23], plasma_stop_2)
        assert decoder.plasma_is_always(slice, 'on')
        assert decoder.move_cumul(slice, 3) > 2000

        # no move during ignition
        cut_start_2 = self.events[23] + 200
        slice = decoder.from_to(self.timeline, self.events[23], cut_start_2)
        assert decoder.move_cumul(slice, 1) == 0
        assert decoder.move_cumul(slice, 2) == 0
        assert decoder.move_cumul(slice, 3) == 0

        # plasma stop after cutoff
        slice = decoder.from_to(self.timeline, plasma_stop_2 + 2, decoder.end(self.timeline))
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
p.wait_ms(700)
p.ohmic_probe()
p.wait_ms(50)
p.transfer_on()
p.wait_ms(500)
p.transfer_off()
p.wait_ms(100)
p.click()
p.move_down()
p.move_down()
p.click()
p.wait_ms(600)
p.ohmic_probe()
p.wait_ms(100)
p.transfer_on()
p.wait_ms(700)
p.transfer_off()
p.wait_ms(800)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
