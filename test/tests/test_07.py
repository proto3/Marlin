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

    def test_ohmic_probe(self):
        # shift positions according to autohome
        decoder.apply_autohome(self.timeline, events[16] + 2)
        self.timeline[3] += 10000

        # z move down
        slice = decoder.from_to(self.timeline, self.events[16], self.events[17])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z stopped
        slice = decoder.from_to(self.timeline, self.events[17] + 2, self.events[17] + 100)
        assert decoder.move_cumul_down(slice, 3) == 0

        # z move 3mm up
        slice = decoder.from_to(self.timeline, self.events[17] + 2, self.events[17] + 200)
        assert decoder.move_cumul_up(slice, 3) == 300

        # z move down to 80mm and up to 100mm
        slice = decoder.from_to(self.timeline, self.events[17], self.events[17] + 700)
        assert decoder.min_pos(slice, 3) == 8000
        assert decoder.max_pos(slice, 3) == 10000

        # z reach 0
        slice = decoder.from_to(self.timeline, self.events[17], decoder.end(self.timeline))
        assert decoder.min_pos(slice, 3) == 0

        # z stop at 0
        z_zero = decoder.when_is_position_reached(self.timeline, 0, 0, 0)
        assert z_zero != -1

        # z move 3mm up
        slice = decoder.from_to(self.timeline, z_zero, z_zero + 100)
        assert decoder.move_cumul_up(slice, 3) == 300

        # z move to 20mm
        slice = decoder.from_to(self.timeline, z_zero + 100, decoder.end(self.timeline))
        assert decoder.move_cumul_up(slice, 3) == 1700

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
p.ohmic_probe()

p.wait_ms(3000)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
