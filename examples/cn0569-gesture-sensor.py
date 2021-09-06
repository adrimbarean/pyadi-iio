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
import collections as col
import math
import winsound as ws

theremin_args0 = [
        [262, 500],
        [294, 500],
        [330, 500],
        [349, 500],
        [392, 500]
        ]
theremin_args1 = [
        [523, 500],
        [587, 500],
        [659, 500],
        [698, 500],
        [784, 500]
        ]
theremin_dict = {
        "CLICK":0,
        "DOWN":1,
        "UP":2,
        "LEFT":3,
        "RIGHT":4
        }

if __name__ == "__main__":

    # Set up ADPD1080
    adpd1080 = adi.adpd1080(uri="serial:COM23")
    adpd1080._ctrl.context.set_timeout(0)
    adpd1080.rx_buffer_size = 8
    adpd1080.sample_rate = 10
 
    avg = [0, 0, 0, 0, 0, 0, 0, 0]
    for _ in range(5):
        data = adpd1080.rx()
        for idx, val in enumerate(data):
            avg[idx] += val.sum()
    avg = [int(x / 5) for x in avg]

    for i in range(8):
        adpd1080.channel[i].offset = adpd1080.channel[i].offset + avg[i]

    L0_tab = col.deque(5*[0], 5)
    L1_tab = col.deque(5*[0], 5)
    front0 = 0
    front1 = 0
    has_gest0 = False
    has_gest1 = False
    algo_time0 = False
    algo_time1 = False
    while True:
        data = adpd1080.rx()

        L0 = data[0].sum() + data[1].sum() + data[2].sum() + data[3].sum()
        L1 = data[4].sum() + data[5].sum() + data[6].sum() + data[7].sum()
        L0_tab.appendleft(L0)
        L1_tab.appendleft(L1)
        front0 = sum(L0_tab) / 5
        front1 = sum(L1_tab) / 5

        if front0 > 1000 and not has_gest0:
            has_gest0 = True
            start_x0 = (int(data[1].sum()) - int(data[0].sum())) / (int(data[1].sum()) + int(data[0].sum()))
            start_y0 = (int(data[3].sum()) - int(data[2].sum())) / (int(data[3].sum()) + int(data[2].sum()))
        if front1 > 1000 and not has_gest1:
            has_gest1 = True
            start_x1 = (int(data[5].sum()) - int(data[4].sum())) / (int(data[5].sum()) + int(data[4].sum()))
            start_y1 = (int(data[7].sum()) - int(data[6].sum())) / (int(data[7].sum()) + int(data[6].sum()))

        if front0 < 1000 and has_gest0:
            has_gest0 = False
            try:
                end_x0 = (int(data[1].sum()) - int(data[0].sum())) / (int(data[1].sum()) + int(data[0].sum()))
                end_y0 = (int(data[3].sum()) - int(data[2].sum())) / (int(data[3].sum()) + int(data[2].sum()))
            except ZeroDivisionError:
                end_x0 = 0.000001
                end_y0 = 0.000001
            algo_time0 = True
        if front1 < 1000 and has_gest1:
            has_gest1 = False
            try:
                end_x1 = (int(data[5].sum()) - int(data[4].sum())) / (int(data[5].sum()) + int(data[4].sum()))
                end_y1 = (int(data[7].sum()) - int(data[6].sum())) / (int(data[7].sum()) + int(data[6].sum()))
            except ZeroDivisionError:
                end_x1 = 0.000001
                end_y1 = 0.000001
            algo_time1 = True

        if algo_time0:
            algo_time0 = False
            m = (start_y0 - end_y0) / (start_x0 - end_x0 + 0.000001)
            d = math.sqrt((start_x0 - end_x0)**2 + (start_y0 - end_y0)**2)
            if d < 0.07:
                gesture0 = "CLICK"
            else:
                if abs(m) > 1:
                    if start_y0 > end_y0:
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
                ws.Beep(theremin_args0[theremin_dict[gesture0]][0], theremin_args0[theremin_dict[gesture1]][1])

        if algo_time1:
            algo_time1 = False
            m = (start_y1 - end_y1) / (start_x1 - end_x1 + 0.000001)
            d = math.sqrt((start_x1 - end_x1)**2 + (start_y1 - end_y1)**2)
            if d < 0.07:
                gesture1 = "CLICK"
            else:
                if abs(m) > 1:
                    if start_y1 > end_y1:
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
                ws.Beep(theremin_args1[theremin_dict[gesture1]][0], theremin_args1[theremin_dict[gesture1]][1])
