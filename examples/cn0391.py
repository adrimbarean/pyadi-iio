# Copyright (C) 2019 Analog Devices, Inc.
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
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
import math
from time import sleep

class cn0391(adi.ad7124):

    def __init__(self, uri=""):
        adi.ad7124.__init__(self, uri)

        self.rx_enabled_channels = [1]

    _th_types_available = {
            "TYPE_T" : 0,
            "TYPE_J" : 1,
            "TYPE_K" : 2,
            "TYPE_E" : 3,
            "TYPE_S" : 4,
            "TYPE_R" : 5,
            "TYPE_N" : 6,
            "TYPE_B" : 7
            }
    _th_type = [0, 0, 0, 0]

    @property
    def th0_type(self):
            return self._th_type[0]
    @th0_type.setter
    def th0_type(self, value=0):
        if type(value) is str:
            try:
                self._th_type[0] = self._th_types_available[value]
            except:
                print("Possible thermistor types are: " + self._th_types_available)
        elif type(value) is int:
            if value < self._th_types_available.len():
                self._th_type[0] = value
            else:
                print("Possible types are integers from 0 to %d".format(self._th_types_available.len()))
        else:
            print("Invalid argument. Integer or string needed.")

    @property
    def th1_type(self):
            return self._th_type[1]
    @th1_type.setter
    def th1_type(self, value=0):
        if type(value) is str:
            try:
                self._th_type[1] = self._th_types_available[value]
            except:
                print("Possible thermistor types are: " + self._th_types_available)
        elif type(value) is int:
            if value < self._th_types_available.len():
                self._th_type[1] = value
            else:
                print("Possible types are integers from 0 to %d".format(self._th_types_available.len()))
        else:
            print("Invalid argument. Integer or string needed.")

    @property
    def th2_type(self):
            return self._th_type[2]
    @th2_type.setter
    def th2_type(self, value=0):
        if type(value) is str:
            try:
                self._th_type[2] = self._th_types_available[value]
            except:
                print("Possible thermistor types are: " + self._th_types_available)
        elif type(value) is int:
            if value < self._th_types_available.len():
                self._th_type[2] = value
            else:
                print("Possible types are integers from 0 to %d".format(self._th_types_available.len()))
        else:
            print("Invalid argument. Integer or string needed.")

    @property
    def th3_type(self):
            return self._th_type[3]
    @th3_type.setter
    def th3_type(self, value=0):
        if type(value) is str:
            try:
                self._th_type[3] = self._th_types_available[value]
            except:
                print("Possible thermistor types are: " + self._th_types_available)
        elif type(value) is int:
            if value < self._th_types_available.len():
                self._th_type[3] = value
            else:
                print("Possible types are integers from 0 to %d".format(self._th_types_available.len()))
        else:
            print("Invalid argument. Integer or string needed.")

    class temp_range():
        def __init__(self, neg_temp, pos_temp1, pos_temp2, neg_volt, pos_volt1, pos_volt2, pos_volt3):
            self.neg_temp = neg_temp
            self.pos_temp1 = pos_temp1
            self.pos_temp2 = pos_temp2
            self.neg_volt = neg_volt
            self.pos_volt1 = pos_volt1
            self.pos_volt2 = pos_volt2
            self.pos_volt3 = pos_volt3

    _temp_ranges = [
            temp_range([0.0, 0.387481063e-1, 0.441944343e-4, 0.118443231e-6, 0.200329735e-7, 0.901380195e-9, 0.226511565e-10, 0.360711542e-12, 0.384939398e-14, 0.282135219e-16, 0.142515947e-18, 0.487686622e-21, 0.107955392e-23, 0.139450270e-26, 0.797951539e-30], [0.0, 0.387481063e-1, 0.332922278e-4, 0.206182434e-6, -0.218822568e-8, 0.109968809e-10, -0.308157587e-13, 0.454791352e-16, -0.275129016e-19], [], [0.0, 2.5949192e+1, -2.1316967e-1, 7.9018692e-1, 4.2527777e-1, 1.3304473e-1, 2.0241446e-2, 1.2668171e-3], [0.0, 2.592800e+1, -7.602961e-1, 4.637791e-2, -2.165394e-3, 6.048144e-5, -7.293422e-7], [], []),
            temp_range([0.0, 0.503811878e-1, 0.304758369e-4, -0.856810657e-7, 0.132281952e-9, -0.170529583e-12, 0.209480906e-15, -0.125383953e-18, 0.156317256e-22], [0.296456256e+3, -0.149761277e+1, 0.317871039e-2, -0.318476867e-5, 0.157208190e-8, -0.306913690e-12], [], [0.0, 1.9528268e+1, -1.2286185e+0, -1.0752178e+0, -5.9086933e-1, -1.7256713e-1, -2.8131513e-2, -2.3963370e-3, -8.3823321e-5], [0.0, 1.978425e+1, -2.001204e-1, 1.036969e-2, -2.549687e-4, 3.585153e-6, -5.344285e-8, 5.099890e-10, 0.0], [-3.11358187e+3, 3.00543684e+2, -9.94773230e+0, 1.70276630e-1, -1.43033468e-3, 4.73886084e-6], []),
            temp_range([0.0, 0.394501280e-1, 0.236223735e-4, -0.328589067e-6, -0.499048287e-8, -0.675090591e-10, -0.574103274e-12, -0.310888728e-14, -0.104516093e-16, -0.198892668e-19, -0.163226974e-22], [-0.176004136e-1, 0.389212049e-1, 0.185587700e-4, -0.994575928e-7, 0.318409457e-9, -0.560728448e-12, 0.560750590e-15, -0.320207200e-18, 0.971511471e-22, -0.121047212e-25], [], [0.0, 2.5173462e+1, -1.1662878e+0, -1.0833638e+0, -8.9773540e-1, -3.7342377e-1, -8.6632643e-2, -1.0450598e-2, -5.19205771e-4, 0.0], [0.0, 2.508355e+1, 7.860106e-2, -2.503131e-1, 8.315270e-2, -1.228034e-2, 9.804036e-4, -4.413030e-5, 1.057734e-6, -1.052755e-8], [-1.318058e+2, 4.830222e+1, -1.646031e+0,  5.464731e-2, -9.650715e-4, 8.802193e-6, -3.110810e-8], []),
            temp_range([0.0, 0.586655087e-1, 0.454109771e-4, -0.779980486e-6, -0.258001608e-7, -0.594525830e-9, -0.932140586e-11, -0.102876055e-12, -0.803701236e-15, -0.439794973e-17, -0.164147763e-19, -0.396736195e-22, -0.558273287e-25, -0.346578420e-28], [0.0, 0.586655087e-1, 0.450322755e-4, 0.289084072e-7, -0.330568966e-9, 0.650244032e-12, -0.191974955e-15, -0.125366004e-17, 0.214892175e-20, -0.143880417e-23, 0.359608994e-27], [], [0.0, 1.6977288e+1, -4.3514970e-1, -1.5859697e-1, -9.2502871e-2, -2.6084314e-2, -4.1360199e-3, -3.4034030e-4, -1.1564890e-5, 0.0], [0.0, 1.7057035e+1, -2.3301759e-1, 6.5435585e-3, -7.3562749e-5, -1.7896001e-6, 8.4036165e-8, -1.3735879e-9, 1.0629823e-11, -3.2447087e-14], [], []),
            temp_range([0.0, 0.540313308e-2, 0.125934289e-4, -0.232477968e-7, 0.3220288238e-10, -0.331465196e-13, 0.255744251e-16, -0.125068871e-19,  0.271443176e-23], [0.132900444e+1, 0.334509311e-2, 0.654805192e-5, -0.164856259e-8, 0.129989605e-13], [0.146628232e+3, -0.258430516e+0, 0.163693574e-3, -0.330439046e-7, -0.943223690e-14], [0.0, 1.84949460e+2, -8.00504062e+1, 1.02237430e+2, -1.52248592e+2, 1.88821343e+2, -1.59085941e+2, 8.23027880e+1,  -2.34181944e+1, 2.79786260e+0], [1.291507177e+1, 1.466298863e+2, -1.534713402e+1, 3.145945973e+0, -4.163257839e-1, 3.187963771e-2, -1.291637500e-3, 2.183475087e-5, -1.447379511e-7, 8.211272125e-9], [-8.087801117e+1, 1.621573104e+2, -8.536869453e+0,  4.7196869763e-1, -1.441693666e-2, 2.081618890e-4], [5.333875126e+4, -1.235892298e+4, 1.092657613e+3,  -4.265693686e+1, 6.247205420e-1]),
            temp_range([0.0, 0.528961729e-2, 0.139166589e-4, -0.238855693e-7, 0.356916001e-10, -0.462347666e-13, 0.500777441e-16, -0.373105886e-19,  0.157716482e-22, -0.281038625e-26], [0.29515792e+1, -0.252061251e-2, 0.159564501e-4, -0.764085947e-8, 0.205305291e-11, -0.293359668e-15], [0.152232118e+3, -0.268819888e+0, 0.171280280e-3, -0.345895706e-7, -0.934633971e-14], [0.0, 1.8891380e+2, -9.3835290e+1, 1.3068619e+2, -2.2703580e+2, 3.5145659e+2, -3.8953900e+2, 2.8239471e+2,  -1.2607281e+2, 3.1353611e+1, -3.3187769e+0], [1.334584505e+1, 1.472644573e+2, -1.844024844e+1, 4.031129726e+0, -6.249428360e-1, 6.468412046e-2, -4.458750426e-3, 1.994710149e-4, -5.313401790e-6, 6.481976217e-8], [-8.199599416e+1, 1.553962042e+2, -8.342197663e+0,  4.279433549e-1, -1.191577910e-2, 1.492290091e-4], [3.406177836e+4, -7.023729171e+3, 5.582903813e+2,  -1.952394635e+1, 2.560740231e-1]),
            temp_range([0.0, 0.261591059e-1, 0.109574842e-4, -0.938411115e-7, -0.464120397e-10, -0.675090591e-10, -0.263033577e-11, -0.226534380e-13, -0.760893007e-16, -0.934196678e-19], [0.0, 0.259293946e-1, 0.157101418e-4, 0.438256272e-7, -0.252611697e-9, 0.643118193e-12, -0.100634715e-14, 0.997453389e-18, -0.608632456e-21, 0.208492293e-24, -0.306821961e-28], [], [0.0, 3.8436847e+1, 1.1010485e+0, 5.2229312e+0, 7.2060525e+0, 5.8488586e+0, 2.7754916e+0, 7.7075166e-1, 1.1582665e-1, 7.3138868e-3], [0.0, 3.86896e+1, -1.08267e+0, 4.70205e-2, -2.12169e-6, -1.17272e-4, 5.39280e-6, -7.98156e-8], [1.972485e+1, 3.300943e+1, -3.915159e-1,  9.855391e-3, -1.274371e-4, 7.767022e-7], []),
            temp_range([0.0, -0.246508183e-3, 0.590404211e-5, -0.13257931e-8, 0.156682919e-11, -0.169445292e-14, 0.629903470e-18], [-0.389381686e+1, 0.285717474e-1, -0.848851047e-4, 0.157852801e-6, -0.168353448e-9, 0.111097940e-12, -0.445154310e-16, 0.989756408e-20, -0.937913302e-24], [], [9.8423321e+1, 6.9971500e+2, -8.4765304e+2, 1.0052644e+3, -8.3345952e+2, 4.5508542e+2, -1.5523037e+2, 2.9886750e+1, -2.4742860e+0], [2.1315071e+2, 2.8510504e+2, -5.2742887e+1, 9.9160804e+0, -1.2965303e+0, 1.1195870e-1, -6.0625199e-3, 1.8661696e-4, -2.4878585e-6], [], [])
            ]
    _cj_temp_range = [
            [-200, 0, 400, -300],
            [-210, 760, 1200, -300],
            [-270, 0, 1372, -300],
            [-270, 0, 1000, -300],
            [-50, 1064.18, 1664.5, 1768.1],
            [-50, 1064.18, 1664.5, 1768.1],
            [-270, 0, 1300, -300],
            [0, 630.615, 1820, -300]
            ]

    _th_temp_range = [
            [-200, 400],
            [-210, 1200],
            [-200, 1372],
            [-200, 1000],
            [-50, 1768],
            [-50, 1768],
            [-200, 1300],
            [250, 1820]
            ]

    _th_volt_range = [
            [-5.603, 0, 20.872, -10, -10],
            [-8.095, 0, 42.919, 69.553, -10],
            [-5.891, 0, 20.644, 54.886, -10],
            [-8.825, 0, 76.373, -10, -10],
            [-0.235, 1.874, 10.332, 17.536, 18.693],
            [-0.226, 1.923, 11.361, 19.739, 21.103],
            [-3.990, 0, 20.613, 47.513, -10],
            [0.291, 2.431, 13.820, -10, -10]
            ]

    _2_23 = 8388608.0
    _gain_rtd = 1
    _gain_th = 32
    _r5 = 1600.0
    _r0 = 1000.0
    _i_ext = 0.75
    _vref_int = 2500.0
    _vref_ext = (_r5 * _i_ext)
    _tc_offset_voltage = 0.0

    _coeff_a = 3.9083e-3
    _coeff_b = -5.775e-7
    _coeff_a_a = (_coeff_a * _coeff_a)
    _coeff_4b_r0 = (4 * _coeff_b / _r0)
    _coeff_2b = (2 * _coeff_b)

    _coeff_k_a0 = 0.1185976
    _coeff_k_a1 = -0.1183432e-3
    _coeff_k_a2 = 0.1269686e+3

    _cj_poly_coeff = [-242.02, 2.2228, 0.00259, -0.00000483, -0.0000000282, 0.000000000152]

    _adc_value0 = [0, 0, 0, 0]
    _adc_value1 = [0, 0, 0, 0]
    _rtd_value = [0, 0, 0, 0]
    _rtd_temp = [0, 0, 0, 0]
    _th_temp = [0, 0, 0, 0]

    def enable_current_source(self, curr_source_ch):
        io_ctrl1_reg = self._rxadc.reg_read(0x3)
        io_ctrl1_reg &= ~0xF
        io_ctrl1_reg |= (0xF & (2 * curr_source_ch + 1))
        self._rxadc.reg_write(0x3, io_ctrl1_reg)

    def data_to_res(self, data):
        return (self._r5 * (data - self._2_23))/(self._2_23 * self._gain_rtd)

    def poly_calc(self, inval, poly_coeff):
        retval = 0.0
        exp_val = 1.0

        for coeff in poly_coeff:
            retval += coeff * exp_val
            exp_val *= inval

        return retval

    def calc_rtd_temp(self, i):
        if self._rtd_value[i] > self._r0:
            return (-self._coeff_a + math.sqrt(self._coeff_a_a - self._coeff_4b_r0*(self._r0 - self._rtd_value[i]))) / self._coeff_2b
        else:
            return self.poly_calc(self._rtd_value[i] / 10, self._cj_poly_coeff)

    def calc_th_temp(self, i):
        th_idx = self._th_type[i]
        th_coeff = self._temp_ranges[th_idx]

        th_voltage = ((self._vref_int*(self._adc_value1[i] - self._2_23))/(self._2_23*self._gain_th)) + self._tc_offset_voltage

        if self._rtd_temp[i] < self._cj_temp_range[th_idx][1]:
            cj_voltage = self.poly_calc(self._rtd_temp[i], th_coeff.neg_temp)
        elif self._rtd_temp[i] <= self._cj_temp_range[th_idx][2]:
            cj_voltage = self.poly_calc(self._rtd_temp[i], th_coeff.pos_temp1)
            if th_idx == self._th_types_available["TYPE_K"]:
                cj_voltage += self._coeff_k_a0 * math.exp(self._coeff_k_a1 * (self._rtd_temp[i] - self._coeff_k_a2) * (self._rtd_temp[i] - self._coeff_k_a2))
        else:
            cj_voltage = self.poly_calc(self._rtd_temp[i], th_coeff.pos_temp2)

        th_voltage += cj_voltage

        if th_voltage < self._th_volt_range[th_idx][0]:
            return -1
        elif th_voltage <= self._th_volt_range[th_idx][1]:
            return self.poly_calc(th_voltage, th_coeff.neg_volt)
        elif th_voltage <= self._th_volt_range[th_idx][2]:
            return self.poly_calc(th_voltage, th_coeff.pos_volt1)
        elif th_voltage <= self._th_volt_range[th_idx][3] and len(th_coeff.pos_volt2) is not 0:
            return self.poly_calc(th_voltage, th_coeff.pos_volt2)
        elif len(th_coeff.pos_volt3) is not 0:
            if th_voltage <= self._th_volt_range[th_idx][4]:
                return self.poly_calc(th_voltage, th_coeff.pos_volt3)
            else:
                return -1
        else:
            return -1


    def get_data(self):
        for i in range(4):
            self.enable_current_source(i)
            self._adc_value0[i] = self.channel[i].raw
            self._adc_value1[i] = self.channel[i + 4].raw

            self._rtd_value[i] = self.data_to_res(self._adc_value0[i])

            self._rtd_temp[i] = self.calc_rtd_temp(i)
            self._th_temp[i] = self.calc_th_temp(i)

        return [self._rtd_temp, self._th_temp]

    def rtd_calibration(self):
        self.enable_current_source(1)
        temp = self._rxadc.reg_read(0x9)
        temp |= 0x8000
        self._rxadc.reg_write(0x9, temp)
        self._rxadc.reg_write(0x1, 0x514)
        while True:
            if self._rxadc.reg_read(0x1) == 0x510:
                break
        temp = self._rxadc.reg_read(0x9)
        temp &= ~0x8000
        self._rxadc.reg_write(0x9, temp)
        self._rxadc.reg_write(0x1, 0x5C0)

    def th_calibration(self):
        temp = self._rxadc.reg_read(0xd)
        temp |= 0x8000
        self._rxadc.reg_write(0xd, temp)
        self._rxadc.reg_write(0x29, 0x800000)

        self._rxadc.reg_write(0x1, 0x518)
        while True:
            if self._rxadc.reg_read(0x1) == 0x510:
                break

        self._rxadc.reg_write(0x1, 0x514)
        while True:
            if self._rxadc.reg_read(0x1) == 0x510:
                break

        temp = self._rxadc.reg_read(0xd)
        temp &= ~0x8000
        self._rxadc.reg_write(0xd, temp)
        self._rxadc.reg_write(0x1, 0x5C0)

    def calibration(self):
        self.rtd_calibration()
        self.th_calibration()

if __name__ == "__main__":
    # Set up CN0391
    cn0391_dev = cn0391(uri="serial:COM5")

    cn0391_dev.calibration()

    cn0391_dev.th0_type = "TYPE_R"

    data = cn0391_dev.get_data()
    print(data)
