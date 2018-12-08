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
        # z move up
        slice = decoder.from_to(self.timeline, self.events[10], self.events[11])
        assert decoder.move_cumul_up(slice, 3) > 100
        assert decoder.move_cumul_down(slice, 3) == 0

        # z move back
        slice = decoder.from_to(self.timeline, self.events[11] + 2, self.events[12])
        assert decoder.move_cumul_down(slice, 3) == 500
        assert decoder.move_cumul_up(slice, 3) > 100

        # z move down
        slice = decoder.from_to(self.timeline, self.events[16] + 2, self.events[17])
        assert decoder.move_cumul_down(slice, 3) > 100
        assert decoder.move_cumul_up(slice, 3) == 0

        # z move back 3mm
        slice = decoder.from_to(self.timeline, self.events[17] + 2, self.events[17] +300)
        assert decoder.move_cumul_up(slice, 3) == 300
        assert decoder.move_cumul_down(slice, 3) == 0

        # z stop on home pos when moving up
        z_home = decoder.from_to(self.timeline, self.events[13], self.events[14])[3][0]
        z_home_return = decoder.from_to(self.timeline, self.events[18], decoder.end(self.timeline))[3][0]
        assert z_home == z_home_return

        # z to 280mm
        z_min = decoder.min_pos(timeline, 3)
        z_min == z_home - 2000


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

p.wait_ms(2000)
p.transfer_off()

events = p.run()

timeline = decoder.decode(export_basename + '.bin')

p.close()
