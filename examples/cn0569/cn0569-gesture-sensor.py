# Copyright (C) 2021 Analog Devices, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     - Neither the name of Analog Devices, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#     - The use of this software may or may not infringe the patent rights
#       of one or more patent holders.  This license does not release you
#       from the requirement that you obtain separate licenses from these
#       patent holders to use this software.
#     - Use of the software either in source or binary form, must be run
#       on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
# RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import adi
import time
import math
import simpleaudio as sa
import numpy as np

sample_rate = 44100
T = 1
t = np.linspace(0, T, T * sample_rate, False)

freq_c4 = 65.41
freq_d4 = 73.42
freq_e4 = 82.41
freq_f4 = 87.31
freq_g4 = 98.00

freq_c5 = 130.81
freq_d5 = 146.83
freq_e5 = 164.81
freq_f5 = 174.61
freq_g5 = 196.00

note_c4 = np.sin(freq_c4 * t * 2 * np.pi)
note_d4 = np.sin(freq_d4 * t * 2 * np.pi)
note_e4 = np.sin(freq_e4 * t * 2 * np.pi)
note_f4 = np.sin(freq_f4 * t * 2 * np.pi)
note_g4 = np.sin(freq_g4 * t * 2 * np.pi)

note_c5 = np.sin(freq_c5 * t * 2 * np.pi)
note_d5 = np.sin(freq_d5 * t * 2 * np.pi)
note_e5 = np.sin(freq_e5 * t * 2 * np.pi)
note_f5 = np.sin(freq_f5 * t * 2 * np.pi)
note_g5 = np.sin(freq_g5 * t * 2 * np.pi)

norm_c4 = note_c4 * 32767 / np.max(np.abs(note_c4))
norm_c4 = norm_c4.astype(np.int16)
norm_d4 = note_d4 * 32767 / np.max(np.abs(note_d4))
norm_d4 = norm_d4.astype(np.int16)
norm_e4 = note_e4 * 32767 / np.max(np.abs(note_e4))
norm_e4 = norm_e4.astype(np.int16)
norm_f4 = note_f4 * 32767 / np.max(np.abs(note_f4))
norm_f4 = norm_f4.astype(np.int16)
norm_g4 = note_g4 * 32767 / np.max(np.abs(note_g4))
norm_g4 = norm_g4.astype(np.int16)

norm_c5 = note_c5 * 32767 / np.max(np.abs(note_c5))
norm_c5 = norm_c5.astype(np.int16)
norm_d5 = note_d5 * 32767 / np.max(np.abs(note_d5))
norm_d5 = norm_d5.astype(np.int16)
norm_e5 = note_e5 * 32767 / np.max(np.abs(note_e5))
norm_e5 = norm_e5.astype(np.int16)
norm_f5 = note_f5 * 32767 / np.max(np.abs(note_f5))
norm_f5 = norm_f5.astype(np.int16)
norm_g5 = note_g5 * 32767 / np.max(np.abs(note_g5))
norm_g5 = norm_g5.astype(np.int16)

theremin_args0 = {
        "CLICK":norm_c4,
        "DOWN":norm_d4,
        "UP":norm_e4,
        "LEFT":norm_f4,
        "RIGHT":norm_g4
        }
theremin_args1 = {
        "CLICK":norm_c5,
        "DOWN":norm_d5,
        "UP":norm_e5,
        "LEFT":norm_f5,
        "RIGHT":norm_g5
        }

if __name__ == "__main__":

    # Set up ADPD1080
    adpd1080 = adi.adpd1080(uri="serial:COM4")
    adpd1080._ctrl.context.set_timeout(0)
    adpd1080.rx_buffer_size = 8
    adpd1080.sample_rate = 10
 
    avg = [0, 0, 0, 0, 0, 0, 0, 0]
    for _ in range(5):
        data = adpd1080.rx()
        for idx, val in enumerate(data):
            avg[idx] += val.sum()
    avg = [int(x / 5) for x in avg]

#    for i in range(8):
#        adpd1080.channel[i].offset = adpd1080.channel[i].offset + avg[i]

#    adpd1080.sample_rate = 512

    g0_incr = 0
    g1_incr = 0
    has_gest0 = False
    has_gest1 = False
    algo_time0 = False
    algo_time1 = False
    while True:
        data = adpd1080.rx()
        for i, ch in enumerate(data):
            ch -= min(ch, avg[i])
        print(data)

        L0 = data[0].sum() + data[1].sum() + data[2].sum() + data[3].sum()
        L1 = data[4].sum() + data[5].sum() + data[6].sum() + data[7].sum()

        if L0 > 1000 and not has_gest0:
            has_gest0 = True
            start_x0 = (int(data[1].sum()) - int(data[0].sum())) / (int(data[1].sum()) + int(data[0].sum()))
            start_y0 = (int(data[3].sum()) - int(data[2].sum())) / (int(data[3].sum()) + int(data[2].sum()))
        if L1 > 1000 and not has_gest1:
            has_gest1 = True
            start_x1 = (int(data[5].sum()) - int(data[4].sum())) / (int(data[5].sum()) + int(data[4].sum()))
            start_y1 = (int(data[7].sum()) - int(data[6].sum())) / (int(data[7].sum()) + int(data[6].sum()))

        if L0 < 1000 and has_gest0 and g0_incr >= 5:
            has_gest0 = False
            try:
                end_x0 = (int(data[1].sum()) - int(data[0].sum())) / (int(data[1].sum()) + int(data[0].sum()))
                end_y0 = (int(data[3].sum()) - int(data[2].sum())) / (int(data[3].sum()) + int(data[2].sum()))
            except ZeroDivisionError:
                end_x0 = 0.000001
                end_y0 = 0.000001
            algo_time0 = True
        if L1 < 1000 and has_gest1 and g1_incr >= 5:
            has_gest1 = False
            try:
                end_x1 = (int(data[5].sum()) - int(data[4].sum())) / (int(data[5].sum()) + int(data[4].sum()))
                end_y1 = (int(data[7].sum()) - int(data[6].sum())) / (int(data[7].sum()) + int(data[6].sum()))
            except ZeroDivisionError:
                end_x1 = 0.000001
                end_y1 = 0.000001
            algo_time1 = True

        if L0 >= 1000:
            g0_incr += 1
        else:
            g0_incr = 0
        if L1 >= 1000:
            g1_incr += 1
        else:
            g1_incr = 0

        if algo_time0:
            algo_time0 = False
            m = (start_y0 - end_y0) / (start_x0 - end_x0 + 0.000001)
            d = math.sqrt((start_x0 - end_x0)**2 + (start_y0 - end_y0)**2)
            if d < 0.07:
                gesture0 = "CLICK"
            else:
                if abs(m) > 1:
                    if start_y0 < end_y0:
                        gesture0 = "UP"
                    else:
                        gesture0 = "DOWN"
                elif abs(m) < 1:
                    if start_x0 > end_x0:
                        gesture0 = "LEFT"
                    else:
                        gesture0 = "RIGHT"
                else:
                    gesture0 = "CLICK"
            if gesture0 != "":
                print("Gesture0: " + gesture0)
                sa.play_buffer(theremin_args0[gesture0], 1, 2, sample_rate)

        if algo_time1:
            algo_time1 = False
            m = (start_y1 - end_y1) / (start_x1 - end_x1 + 0.000001)
            d = math.sqrt((start_x1 - end_x1)**2 + (start_y1 - end_y1)**2)
            if d < 0.07:
                gesture1 = "CLICK"
            else:
                if abs(m) > 1:
                    if start_y1 < end_y1:
                        gesture1 = "UP"
                    else:
                        gesture1 = "DOWN"
                elif abs(m) < 1:
                    if start_x1 > end_x1:
                        gesture1 = "LEFT"
                    else:
                        gesture1 = "RIGHT"
                else:
                    gesture1 = "CLICK"
            if gesture1 != "":
                print("Gesture1: " + gesture1)
                sa.play_buffer(theremin_args1[gesture1], 1, 2, sample_rate)
