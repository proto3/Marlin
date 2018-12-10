#!/usr/bin/env python3
from PyQt5 import QtCore, QtGui, Qt
import pyqtgraph as pg
import sys, os
import numpy as np

import decoder

class MainWidget(QtGui.QWidget):
    def __init__(self):
        super().__init__()
        plot = pg.PlotWidget()

        plot.addLegend(size=(80,50))
        self.curve_x = pg.PlotCurveItem(data[0], data[1], pen=pg.mkPen(color=(0x2E, 0x86, 0xAB), width=2), name='X')
        self.curve_y = pg.PlotCurveItem(data[0], data[2], pen=pg.mkPen(color=(0xF0, 0x87, 0x00), width=2), name='Y')
        self.curve_z = pg.PlotCurveItem(data[0], data[3], pen=pg.mkPen(color=(0x8A, 0xC9, 0x26), width=2), name='Z')

        self.curve_p = pg.PlotCurveItem(data[0], (1-data[4])*5000, pen=pg.mkPen(color=(46, 40, 171), width=1))
        self.fillLevel = pg.PlotCurveItem([0, data[0][-1]], [0, 0])
        self.cut = pg.FillBetweenItem(curve1=self.curve_p, curve2=self.fillLevel, brush=pg.mkBrush(color=(0xFF, 0x64, 0x16, 80)))

        plot.addItem(self.curve_x)
        plot.addItem(self.curve_y)
        plot.addItem(self.curve_z)
        plot.addItem(self.cut)

        for event in events:
            self.marker = pg.InfiniteLine(pos=int(event[0])*10000, pen=pg.mkPen(color=(0xF0, 0x57, 0x00), width=1), label=event[1])
            plot.addItem(self.marker)

        self.marker = pg.InfiniteLine(pos=data[0][-1], pen=pg.mkPen(color=(0xF0, 0x57, 0x00), width=1), movable=True, label='t : {value:.0f}')
        plot.addItem(self.marker)

        grid_alpha = 70
        x = plot.getAxis("bottom")
        y = plot.getAxis("left")
        x.setGrid(grid_alpha)
        y.setGrid(grid_alpha)

        layout = QtGui.QVBoxLayout()
        layout.addWidget(plot)
        self.setLayout(layout)

if __name__ == '__main__':
    if(len(sys.argv) < 2):
        print("usage : python3 visualizer.py record.bin")
        sys.exit(0)
    test_name = os.path.splitext(sys.argv[1])[0]

    data = decoder.decode(test_name + '.bin')

    # duplicate values to not interpolate value but draw stair-step curves instead
    a = np.delete(np.repeat(np.take(data, [0], axis=0), 2, axis=1),0, axis=1)
    b = np.delete(np.repeat(np.delete(data, 0, axis=0), 2, axis=1),-1, axis=1)
    data = np.concatenate((a,b), axis=0)

    event_file = open(test_name + '.event', "r")
    events = list()
    for line in event_file:
        words = line.split(' ', 1)
        words[1] = words[1].replace("\n", "")
        words[1] = words[1].replace(" ", "\n")
        events.append(words)
    event_file.close()

    pg.setConfigOption('background', (0x35, 0x40, 0x45))
    pg.setConfigOption('foreground', (0xB0, 0xB0, 0xB0))
    pg.setConfigOption('antialias', True)
    app = QtGui.QApplication([])

    main_widget = MainWidget()
    main_widget.show()

sys.exit(app.exec_())
