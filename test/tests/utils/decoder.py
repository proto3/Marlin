#!/usr/bin/env python3
import sys, struct
import numpy as np

def _rising_edges(serie):
    shifted = np.roll(serie, 1)
    shifted[0] = serie[0]
    return (serie - shifted) > 0

def decode(filename):
    # load raw data from capture file
    file = open(filename, 'rb')
    raw = [[], []]
    while(True):
        next_long = file.read(8)
        if(next_long == b''):
            break
        time = struct.unpack('q', next_long)[0]
        value = struct.unpack('h', file.read(2))[0]
        raw[0].append(time)
        raw[1].append(value)
    data = np.array(raw)

    data[0] += 10

    # split capture channels into rows
    # [time, clk, step_x, dir_x, step_y, dir_y, step_z, dir_z, cut]
    for i in range(2,9):
        mask = (1<<(i-1))
        data = np.insert(data, i, (data[1] & mask)>0, axis=0)
    data[1] = data[1] & 0x01

    # look for first clock pulse and delete what's preceeding
    prev_val = 0
    for i, val in enumerate(data[1]):
        # rising edge
        if prev_val < val:
            prev_rise = i
        # falling edge and pulse shorter than 15µs
        elif prev_val > val and data[0][i] - data[0][prev_rise] < 15:
            clk_start_idx = prev_rise
            break
        prev_val = val
    data = np.delete(data, np.s_[0:clk_start_idx], axis=1)

    # update t0
    data[0] -= data[0][0]

    # replace clock value by clock edges (1 : rising, 0 : constant, -1 : falling)
    prev_clk = np.roll(data[1], 1)
    prev_clk[0] = 0
    data[1] -= prev_clk

    # get number of ticks
    nb_clk_cycles = np.count_nonzero(data[1] == 1) - 1 # -1 because we count cycles, not rising edges

    # compute duration from first to last tick
    rev = data[1][::-1]
    last_tick_idx = len(rev) - np.argmax(rev == 1) - 1
    duration = data[0][last_tick_idx]

    # print("duration :", duration/1000)
    # print("ticks :", nb_clk_cycles)
    # print("average ticks duration :", duration / nb_clk_cycles/1000)

    # normalize time scale on clock
    time_normalized = data[0].astype(float)
    cycle_reel_duration = 10000
    time_normalized *= cycle_reel_duration * nb_clk_cycles / duration
    data[0] = time_normalized.round().astype(int)

    # accumulate stepper position from step and dir
    for i in range(2,7,2):
        data[i] = np.cumsum(_rising_edges(data[i]) * (data[i+1]*2-1))

    # drop clock and direction data
    data = np.delete(data, [1,3,5,7], 0)

    # look for duplicates indices in timeline
    tr = data[1:].transpose()
    to_delete = []
    for i in range(1, len(tr)-1):
        if(np.array_equal(tr[i], tr[i-1])):
            to_delete.append(i)

    # delete duplicates
    data = np.delete(data, to_delete, axis=1)

    return data

def begin(timeline):
    return timeline[0][0] / 1000

def end(timeline):
    return timeline[0][-1] / 1000

def from_to(timeline, a, b):
    assert a >= 0
    assert b > 0
    assert a < b

    a *= 1000
    b *= 1000

    start = np.argmax(timeline[0] > a) - 1
    stop = np.argmax(timeline[0] >= b)

    if(stop == 0):
        stop = len(timeline[0])

    slice = np.copy(np.split(timeline, [start, stop], axis=1)[1])
    slice[0][0] = a

    slice = np.pad(slice, ((0, 0), (0, 1)), 'edge')
    slice[0][-1] = b

    return slice

def move_diff(timeline, axis):
    return timeline[axis][-1] - timeline[axis][0]

def move_cumul(timeline, axis):
    shifted = np.roll(timeline[axis], 1)
    shifted[0] = shifted[1]
    diff = timeline[axis] - shifted
    return np.sum(np.absolute(diff))

def move_cumul_up(timeline, axis):
    shifted = np.roll(timeline[axis], 1)
    shifted[0] = shifted[1]
    diff = timeline[axis] - shifted
    return np.sum(np.clip(diff, 0, None))

def move_cumul_down(timeline, axis):
    shifted = np.roll(timeline[axis], 1)
    shifted[0] = shifted[1]
    diff = timeline[axis] - shifted
    return -np.sum(np.clip(diff, None, 0))

def min_pos(timeline, axis):
    return np.min(timeline[axis])

def max_pos(timeline, axis):
    return np.max(timeline[axis])

def when_is_highest_position_reached(timeline, axis):
    return timeline[0][np.argmax(timeline[axis])] / 1000

def when_is_lowest_position_reached(timeline, axis):
    return timeline[0][np.argmin(timeline[axis])] / 1000

def when_is_position_reached(timeline, x, y, z):
    if(x is None):
        x_valid = np.ones_like(timeline[0])
    else:
        x_valid = np.take(timeline, 1, axis=0) == x
    if(y is None):
        y_valid = np.ones_like(timeline[0])
    else:
        y_valid = np.take(timeline, 2, axis=0) == y
    if(z is None):
        z_valid = np.ones_like(timeline[0])
    else:
        z_valid = np.take(timeline, 3, axis=0) == z
    valid = np.logical_and(np.logical_and(x_valid, y_valid), z_valid)
    if(not np.any(valid)):
        return -1
    else:
        return timeline[0][np.argmax(valid)] / 1000

def get_position(timeline, t):
    index = np.argmax(timeline[0] > t * 1000) - 1
    return (timeline[1][index], timeline[2][index], timeline[3][index])

def plasma_is_always(timeline, state):
    val = -1
    if(state == 'on'):
        val = 0
    elif(state == 'off'):
        val = 1
    else:
        assert val != -1

    plasma = np.take(timeline, 4, axis=0)
    return np.all(plasma == val)

def apply_autohome(timeline, t):
    timeline[1] -= get_position(timeline, t)[0]
    timeline[2] -= get_position(timeline, t)[1]
    timeline[3] -= get_position(timeline, t)[2]
