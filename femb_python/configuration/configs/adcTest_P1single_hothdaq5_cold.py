#!/usr/bin/env python33

"""
Just like adcTest_P1single_hothdaq4, just with self.COLD = True
"""

from femb_python.configuration.configs import adcTest_P1single_hothdaq5

class FEMB_CONFIG(adcTest_P1single_hothdaq5.FEMB_CONFIG):
    def __init__(self,exitOnError=True):
        super().__init__(exitOnError=exitOnError)
        self.COLD = True
