#!/usr/bin/env python

import sys
import math
from operator import add

class encoder(object):
    def __init__(self):

        ## MORF NO PRIOR KNOWLEDGE ##
        self.set_BC_morf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.set_CF_morf = [0.5044372080402942, 0.11179573179088269, 0.22336131133168322, 0.5238323908959595, 0.4897837537401469, 0.23336879209210276, 0.156900132397411, 0.2705401047539295, 0.5045759819857375, 0.4904423579823461, 0.2409376907772342, 0.1710147855545038, 0.2840915832472842, 0.5079500341720962, 0.4593346845127408, 0.21835216964225468, 0.1690032511888188, 0.3258838945822318, 0.5074750373643965, 0.2620683981985637]
        self.set_FT_morf = [-0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04, -0.04]

    def get_init_parameter_set(self, robot, encoding, numKernals):

        if numKernals == 20:
            if robot == 'MORF':
                set_BC = self.set_BC_morf
                set_CF = self.set_CF_morf
                set_FT = self.set_FT_morf
            else:
                print('[ ERROR] Unknown robot')

        if encoding == "indirect":
            init_parameter_set = set_BC + set_CF + set_FT
        else:
            print('[ ERROR] Unknown encoding')

        # Sensor Parameter Set
        sens_set_BC = [0.0] * (numKernals)  # 0.03
        sens_set_CF = [0.0] * (numKernals)  # 0.0
        sens_set_FT = [0.0] * (numKernals)  # -0.3

        if encoding == "indirect":
            init_sensor_parameter_set = sens_set_BC + sens_set_CF + sens_set_FT
        else:
            print('[ ERROR] Unknown encoding')

        return init_parameter_set, init_sensor_parameter_set
