#!/usr/bin/env python33

"""
Configuration for P1 ADC single-chip board
"""

from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import
from builtins import super
from builtins import int
from builtins import range
from builtins import hex
from builtins import str
from future import standard_library
standard_library.install_aliases()
from builtins import object
import sys 
import string
import time
import copy
import os.path
import subprocess
from femb_python.femb_udp_P1A01 import FEMB_UDP
from femb_python.test_measurements.adc_clk_tst.femb_udp_cmdline import FPGA_UDP
from femb_python.configuration.config_base import FEMB_CONFIG_BASE, FEMBConfigError, SyncADCError, InitBoardError, ConfigADCError, ReadRegError
from femb_python.configuration.adc_asic_reg_mapping_P1 import ADC_ASIC_REG_MAPPING
from femb_python.test_measurements.adc_clk_tst.adc_asic_reg_mapping import ADC_ASIC_REG_MAPPING_NEW# fix/update!
from femb_python.test_instrument_interface.keysight_33600A import Keysight_33600A
from femb_python.test_instrument_interface.rigol_dp800 import RigolDP800

class FEMB_CONFIG(FEMB_CONFIG_BASE):

    def __init__(self,exitOnError=True):
        super().__init__(exitOnError=exitOnError)
        #declare board specific registers
        self.FEMB_VER = "adctestP1single_clkTest"
        self.REG_RESET = 0                  # checked (good)
        self.REG_ASIC_RESET = 1             # checked (good)
        self.REG_ASIC_SPIPROG = 2           # checked (good)
        self.REG_SEL_CH = 7                 # checked (good)
        self.REG_HS = 17                    # checked (wtf)
        self.REG_ADCSPI_BASE = 0x200        # 512 in decimal updated (good)
        self.REG_ADCSPI_RDBACK_BASE = 0x250 # 592 in decimal updated (good)

        self.REG_LATCHLOC1_4 = 4            # checked (good) latch_loc0(7 downto 0)
        self.REG_CLKPHASE = 6               # checked (good) is it the same as phase control?
        """
        self.REG_LATCHLOC1_4_data_warm = 0x6060604 # 
        self.REG_CLKPHASE_data_warm = 0xfffc0000
        self.REG_LATCHLOC1_4_data_1MHz_warm = 0x6060604
        self.REG_CLKPHASE_data_1MHz_warm = 0xffff0000
        self.REG_LATCHLOC1_4_data_cold = 0x6060604
        self.REG_CLKPHASE_data_cold = 0xfffc0000
        self.REG_LATCHLOC1_4_data_1MHz_cold = 0x6060604
        self.REG_CLKPHASE_data_1MHz_cold = 0xfffc0001
        """
        self.REG_LATCHLOC1_4_data_1MHz = 0x6060604 # update
        self.REG_CLKPHASE_data_1MHz = 0x15   # update
        self.REG_LATCHLOC1_4_data_2MHz = 0x6060604 # update
        self.REG_CLKPHASE_data_2MHz = 0x15   # update

        self.ADC_TESTPATTERN = [0x12, 0x345, 0x678, 0xf1f, 0xad, 0xc01, 0x234, 0x567, 0x89d, 0xeca, 0xff0, 0x123, 0x456, 0x789, 0xabc, 0xdef]

        ##################################
        # external clock control registers
        ##################################
        self.FPGA_FREQ_MHZ = 200 # frequency of FPGA clock in MHz
        self.REG_EXTCLK_PERIOD = 20

        self.REG_EXTCLK_INV       = 21 # checked (good)
        self.REG_EXTCLK_RST_OFF   = 22 # checked (good) 
        self.REG_EXTCLK_RST_WID   = 23 # checked (good)
        self.REG_EXTCLK_READ_OFF  = 24 # checked (good)
        self.REG_EXTCLK_READ_WID  = 25 # checked (good)
        self.REG_EXTCLK_IDXM_OFF  = 26 # checked (good)
        self.REG_EXTCLK_IDXM_WID  = 27 # checked (good)
        self.REG_EXTCLK_IDXL_OFF  = 28 # checked (good)
        self.REG_EXTCLK_IDXL_WID  = 29 # checked (good)
        self.REG_EXTCLK_IDL1_OFF  = 30 # checked (good)
        self.REG_EXTCLK_IDL1_WID  = 31 # checked (good)
        self.REG_EXTCLK_IDL2_OFF  = 32 # checked (good)
        self.REG_EXTCLK_IDL2_WID  = 33 # checked (good)
        self.REG_EXTCLK_PLL_STEP0 = 34 # checked (good)
        self.REG_EXTCLK_PLL_STEP1 = 35 # checked (good)
        self.REG_EXTCLK_PLL_STEP2 = 36 # checked (good)

        self.EC_RST_OFF = 0
        self.EC_RST_WID = 50
        self.EC_RD_OFF = 480
        self.EC_RD_WID = 20
        self.EC_IDXM_OFF = 230
        self.EC_IDXM_WID = 270 #270
        self.EC_IDXL_OFF = 480
        self.EC_IDXL_WID = 20
        self.EC_IDL1_OFF = 50 #40
        self.EC_IDL1_WID = 190
        self.EC_IDL2_OFF = 480
        self.EC_IDL2_WID = 20
        self.EC_PLL_STEP0 =  0x0014000D #0x000b000f # found in labview
        self.EC_PLL_STEP1 =  0x00060012 #0x000e0008 # found in labview
        self.EC_PLL_STEP2 =  0x80190009 # found in labview
        self.inv_rst=True
        self.inv_read=False
        self.inv_idxm=False
        self.inv_idxl=False
        self.inv_idl=False
        #self.inv_clk_dis=True

        ##################################
        ##################################

        self.NASICS = 1
        self.NCHNS = 16
        self.FUNCGENINTER = Keysight_33600A("/dev/usbtmc1",1)
        self.POWERSUPPLYINTER = RigolDP800("/dev/usbtmc0",["CH2","CH3","CH1"]) # turn on CH2 first
        self.F2DEFAULT = 0
        self.CLKDEFAULT = "fifo"

        self.frame_size = 0x0efb
        self.BPS = 13 #Bytes per sample

        ## Firmware update related variables
        self.FIRMWAREVERSION = "A01"
        self.FIRMWAREPATH2MHZ = "/home/oper/Documents/CarlosForkedRepo/femb_python/femb_python/test_measurements/adc_clk_tst/S_SKT_ADC_CHP_TST.sof"
        self.FIRMWAREPATH1MHZ = "/home/oper/Documents/CarlosForkedRepo/femb_python/femb_python/test_measurements/adc_clk_tst/S_SKT_ADC_CHP_TST.sof"
        self.FIRMWAREPROGEXE = "/opt/sw/intelFPGA/17.0/qprogrammer/bin/quartus_pgm"
        #self.FIRMWAREPROGCABLE = "USB-Blaster"
        self.FIRMWAREPROGCABLE = "USB-BlasterII"
        self.SAMPLERATE = 2e6

        #initialize FEMB UDP object
        self.femb = FEMB_UDP()
        self.adc_reg = ADC_ASIC_REG_MAPPING()
        self.adc_reg_new = ADC_ASIC_REG_MAPPING_NEW()
        self.femb_eh = FPGA_UDP()
        self.PC_IP = self.femb_eh.PC_IP
        self.FPGA_IP = self.femb_eh.FPGA_IP


    def selectChannel(self,asic,chan,hsmode=1,singlechannelmode=None):
        """
        asic is chip number 0 to 7
        chan is channel within asic from 0 to 15
        hsmode: if 0 then streams all channels of a chip, if 1 only te selected channel. defaults to 1
        singlechannelmode: not implemented
        """
        hsmodeVal = int(hsmode) & 1;
        asicVal = int(asic)
        if (asicVal < 0 ) or (asicVal >= self.NASICS ) :
                print( "femb_config_femb : selectChan - invalid ASIC number, only 0 to {} allowed".format(self.NASICS-1))
                return
        chVal = int(chan)
        if (chVal < 0 ) or (chVal > 15 ) :
                print("femb_config_femb : selectChan - invalid channel number, only 0 to 15 allowed")
                return

        #print( "Selecting ASIC " + str(asicVal) + ", channel " + str(chVal))
        self.femb.write_reg(8, 0x80000001)                  # WIB_MODE   <= reg8_p(0) 
        """
        self.femb.write_reg ( self.REG_HS, hsmodeVal)
        regVal = (chVal << 8 ) + asicVal
        self.femb.write_reg( self.REG_SEL_CH, regVal)
        """

    def programFirmware(self, firmware):
        """
        Programs the FPGA using the firmware file given.
        """
        if self.FIRMWAREPROGCABLE == "USB-BlasterII":
            # this programmer is too fast for our board 
            # (or linux or something) so we have to slow it down
            jtagconfig_commandline = os.path.dirname(self.FIRMWAREPROGEXE)
            jtagconfig_commandline = os.path.join(jtagconfig_commandline,"jtagconfig")
            jtagconfig_commandline += " --setparam 1 JtagClock 6M"
            print(jtagconfig_commandline)
            subprocess.run(jtagconfig_commandline.split(),check=True)
        commandline = "{} -c {} -m jtag -o p;{}".format(self.FIRMWAREPROGEXE,self.FIRMWAREPROGCABLE,firmware)
        commandlinelist = commandline.split()
        print(commandline)
        print(commandlinelist)
        subprocess.run(commandlinelist,check=True)

    def checkFirmwareProgrammerStatus(self):
        """
        Prints a debug message for the firmware programmer
        """
        jtagconfig_commandline = os.path.dirname(self.FIRMWAREPROGEXE)
        jtagconfig_commandline = os.path.join(jtagconfig_commandline,"jtagconfig")
        if self.FIRMWAREPROGCABLE == "USB-BlasterII":
            # this programmer is too fast for our board 
            # (or linux or something) so we have to slow it down
            jtagconfig_commandline_speed = jtagconfig_commandline +" --setparam 1 JtagClock 6M"
            print(jtagconfig_commandline_speed)
            subprocess.run(jtagconfig_commandline_speed.split())
        subprocess.run(jtagconfig_commandline.split())

    def programFirmware1Mhz(self):
        self.programFirmware(self.FIRMWAREPATH1MHZ)
        self.SAMPLERATE = 1e6

    def programFirmware2Mhz(self):
        self.programFirmware(self.FIRMWAREPATH2MHZ)
        self.SAMPLERATE = 2e6

    def getClockStr(self):
        latchloc1 = self.femb.read_reg(self.REG_LATCHLOC1_4)
        #latchloc5 = self.femb.read_reg(self.REG_LATCHLOC5_8)
        clkphase = self.femb.read_reg(self.REG_CLKPHASE)
        if latchloc1 is None:
            return "Register Read Error"
        #if latchloc5 is None:
        #    return "Register Read Error"
        if clkphase is None:
            return "Register Read Error"
        return "Latch Loc: {:#010x} Clock Phase: {:#010x}".format(latchloc1,clkphase) #,latchloc5

    def getSyncStatus(self):
        return [None],[True],None

    """
    testing
    """

    def get_data_chipXchnX(self, chip, chn, packets = 1):
        
        if (chn < -1 ) or (chn > self.NCHNS ):
            print ("FEMB CONFIG --> get_data_chipXchnX() -> Error: Channel must be between 0 and 15, or -1 for all channels")
            return
        
        if (chip < 0 ) or (chip > self.NASICS ):
            print ("FEMB CONFIG --> get_data_chipXchnX() -> Error: Chip must be between 0 and {}".format(self.chip_num))
            return

        badSync = 0
        expVal = 1 #expected value
        k = 0
        for i in range(15):
            self.femb.write_reg(7, 0)      # CHN_select <= reg7_p(7-0)       
            time.sleep(0.01)
            self.femb.write_reg(7, chn)    # CHN_select <= reg7_p(7-0)       
            time.sleep(0.01)
            data = self.femb_eh.get_data_packets(data_type = "int", num = packets, header = False)
            #data1 = self.femb.get_data(num = packets)
            #print("normal: {} ".format(data[0:20]))
            #print("new:    {} ".format(data1[0:20]))
            try:
                if (k > 0):
                    print ("get_data_chipXchnX --> Now doing another test")
                    #print (data[0:13])
                if (data[0] != 0xFACE):
                #If FACE isn't the first 2 bytes, turn WIB mode off and then on and try again
                    self.femb.write_reg(8,0) # Turn WIB Mode Off
                    time.sleep(0.01)
                    self.femb.write_reg(8,1) # Turn WIB Mode On
                    time.sleep(0.01)

                    if (k > 14):
                        #print ("chipXchnX--> Error in get_data_chipXchnX: return None")
                        #print (hex(data[0]))
                        #print (data)
                        data = None
                        badSync = 1
                        expVal = 0
                        return data, bs, ex # data none causes errors
                    else:
                        #print ("chipXchnX--> Error in get_data_chipXchnX: Packet format error")
                        #print ("k = {}".format(k))
                        #print (data[0:13])
                        k += 1
                else:
                    #print ("chipXchnX--> Found header, break loop")
                    #print ("k = {}".format(k))
                    #print (data[0:13])
                    break
            except IndexError:
                print ("FEMB CONFIG --> Something was wrong with the incoming data")
                print (data)
            
        test_length = len(data)
        full_samples = test_length // self.BPS
        chn_data = []
        
        for i in range (full_samples):
            if (chn == 7):
                chn_data.append(data[(self.BPS*i)+1] & 0x0FFF)
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 6):
                chn_data.append(((data[(self.BPS*i)+2] & 0x00FF) << 4) + ((data[(self.BPS*i)+1] & 0xF000) >> 12))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 5):
                chn_data.append(((data[(self.BPS*i)+3] & 0x000F) << 8) + ((data[(self.BPS*i)+2] & 0xFF00) >> 8))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 4):
                chn_data.append(((data[(self.BPS*i)+3] & 0xFFF0) >> 4))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 3):
                chn_data.append(data[(self.BPS*i)+4] & 0x0FFF)
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 2):
                chn_data.append(((data[(self.BPS*i)+5] & 0x00FF) << 4) + ((data[(self.BPS*i)+4] & 0xF000) >> 12))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 1):
                chn_data.append(((data[(self.BPS*i)+6] & 0x000F) << 8) + ((data[(self.BPS*i)+5] & 0xFF00) >> 8))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 0):
                chn_data.append(((data[(self.BPS*i)+6] & 0xFFF0) >> 4))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 15):
                chn_data.append(data[(self.BPS*i)+7] & 0x0FFF)
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 14):
                chn_data.append(((data[(self.BPS*i)+8] & 0x00FF) << 4) + ((data[(self.BPS*i)+7] & 0xF000) >> 12))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 13):
                chn_data.append(((data[(self.BPS*i)+9] & 0x000F) << 8) + ((data[(self.BPS*i)+8] & 0xFF00) >> 8))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 12):
                chn_data.append(((data[(self.BPS*i)+9] & 0xFFF0) >> 4))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 11):
                chn_data.append(data[(self.BPS*i)+10] & 0x0FFF)
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 10):
                chn_data.append(((data[(self.BPS*i)+11] & 0x00FF) << 4) + ((data[(self.BPS*i)+10] & 0xF000) >> 12))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 9):
                chn_data.append(((data[(self.BPS*i)+12] & 0x000F) << 8) + ((data[(self.BPS*i)+11] & 0xFF00) >> 8))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == 8):
                chn_data.append(((data[(self.BPS*i)+12] & 0xFFF0) >> 4))
                #print("chn: {} val: {}".format(chn,chn_data))
            if (chn == -1):
                return (data)
            
        return chn_data, badSync, expVal

    def fixUnsyncNew(self, adc):

        print("\n femb_config_sbnd.py -> fixUnsyncNew() -> adc = " + str(adc) + "\n")        
        
        adcNum = int(adc)
        if (adcNum < 0) or (adcNum > 3):
                print ("FEMB_CONFIG--> femb_config_femb : testLink - invalid asic number")
                return

        initLATCH1_4 = self.femb.read_reg( self.REG_LATCHLOC1_4)
        initPHASE = self.femb.read_reg( self.REG_CLKPHASE)
        
        #loop through sync parameters
        shiftMask = (0xFF << 8*adcNum)
        initSetting = (initLATCH1_4 & shiftMask) >> (8*adcNum)
        print ("FEMB_CONFIG--> First testing around default value of {}".format(initSetting))
        for phase in range(0,4,1):
            clkMask = (0x3 << (adcNum * 2))
            testPhase = ( (initPHASE & ~(clkMask)) | (phase << (adcNum * 2)) ) 
            self.femb.write_reg(self.REG_CLKPHASE, testPhase)
            #print ("Init Setting is {}".format(hex(initSetting)))
            #print ("Will test {}, {}, and {}".format(initSetting - 1, initSetting, initSetting + 1))
            for shift in range(initSetting - 1,initSetting + 2,1):
                print ("\n\tfemb_config_sbnd.py -> fixUnsyncNew -> This time, we're testing {}\n".format(shift))
                testShift = ( (initLATCH1_4 & ~(shiftMask)) | (shift << 8*adcNum) )
                self.femb.write_reg(self.REG_LATCHLOC1_4, testShift)
                print ("FEMB_CONFIG--> Trying to sync Chip {} with Latch Lock:{} and Phase:{}".format(adcNum, 
                       hex(testShift), hex(testPhase)))
                       
                #test link
                unsync, syncDicts = self.testUnsyncNew(adcNum)
                if unsync == True :
                    print ("FEMB_CONFIG--> ADC {} synchronized".format(adc))
                    self.REG_LATCHLOC1_4_data = testShift
                    self.REG_CLKPHASE_data = testPhase
                    return True
            #Then try all settings
        print ("FEMB_CONFIG--> Now testing the rest of them")
        for phase in range(0,4,1):
            clkMask = (0x3 << (adcNum * 2))
            testPhase = ( (initPHASE & ~(clkMask)) | (phase << (adcNum * 2)) ) 
            self.femb.write_reg(self.REG_CLKPHASE, testPhase)
            #First test latch lock settings close to default values
            shiftMask = (0xFF << 8*adcNum)
            initSetting = initLATCH1_4 & shiftMask
            for shift in range(0,16,1):
                testShift = ( (initLATCH1_4 & ~(shiftMask)) | (shift << 8*adcNum) )
                self.femb.write_reg(self.REG_LATCHLOC1_4, testShift)
                print ("FEMB_CONFIG--> Trying to sync Chip {} with Latch Lock:{} and Phase:{}".format(adcNum, 
                       hex(testShift), hex(testPhase)))
                       
                #test link
                unsync, syncDicts = self.testUnsyncNew(adcNum)
                if (unsync == 0):
                    print ("FEMB_CONFIG--> ADC {} synchronized".format(adc))
                    self.REG_LATCHLOC1_4_data = testShift
                    self.REG_CLKPHASE_data = testPhase
                    return True
        #if program reaches here, sync has failed
        print ("FEMB_CONFIG--> ADC SYNC process failed for ADC # " + str(adc))
        return False


    def testUnsyncNew(self, adc):
        print("Starting testUnsync adc: ",adc)
        adcNum = int(adc)
        badSync = 0 #
        syncDicts = [{}]*16 #
        syncDataCounts = [{} for i in range(16)] # data for each channel

        if (adcNum < 0 ) or (adcNum > 3 ):
            print ("testUnsyncNew()--> asic number must be between 0 and 3")
            badSync = 1
            return badSync
            
        for j in range(100):    #reset sync error
            self.femb_eh.write_reg(11, 1) # ERROR_RESET <= reg11_p(0)
            time.sleep(0.01)
            self.femb_eh.write_reg(11, 0) # ERROR_RESET <= reg11_p(0)
            time.sleep(0.01)
            
            if (adc == 0):
                conv_error = self.femb_eh.read_reg(12)     # reg12_i => x"" & CONV_ERROR 
                header_error = self.femb_eh.read_reg(13)   # reg13_i => HDR2_ERROR & HDR1_ERROR 
                
            error = False
            
            if (conv_error != 0):
                error = True # conv error
                
            if (header_error != 0):
                error = True # header error

            if (error == True):               
                print ("testUnsyncNew() -> conv error: {} header error: {} : {}".format(conv_error,header_error, j)) # no errors

            if (error == False):
                print ("testUnsyncNew() -> conv error: {} header error: {} : {}".format(conv_error,header_error, j)) #break loop if no error found
                break

            elif (j > 60):
                badSync = 1
                break
            else:
                self.configAdcAsicNew(False)
                #print ("Loop {}".format(j))
           
        for ch in range(0,16,1):
            syncDicts[ch]["data"] = True
            syncDicts[ch]["maxCodeMatchesExpected"] = True
            for test in range(0,1,1):
                data, bs, ex = self.get_data_chipXchnX(chip = adcNum, chn = ch, packets = 1)
                if (ex == 0):
                    syncDicts[ch]["maxCodeMatchesExpected"] = False                    
                if (data == None):
                    badSync = 1 # Sync response didn't come in
                    syncDicts[ch]["data"] = False
                    continue
                if (bs == 0):
                    syncDicts[ch]["data"] = True
                    syncDicts[ch]["maxCodeMatchesExpected"] = True
                    for samp in data[0:len(data)]:
                        if samp != self.ADC_TESTPATTERN[ch]:
                            print ("testUnsyncNew() -> chip {} chn {} looking for {}, found {}".format(adcNum, ch, hex(self.ADC_TESTPATTERN[ch]), hex(samp)))
                            badSync = 1
                            break
                        else:
                            print ("testUnsyncNew() -> chip {} chn {} looking for {}, found {}".format(adcNum, ch, hex(self.ADC_TESTPATTERN[ch]), hex(samp)))
                            badSync = 0
                            break
                if (badSync == 1 ):
                   break

        return badSync, syncDicts


    def FinalSyncCheck(self):
        print ("FEMB_CONFIG--> Final sync check to make sure")
        for a in range(0,self.NASICS,1):
            
            self.adc_reg_new.set_adc_global(chip = a, f5 = 1) # Test DATA
            self.configAdcAsicNew(False)

            # quad board settings:
            #self.select_chip(chip = a)
            
            # single board settings:
            #select_chip is not necessary

            badSync = 0
            expV = 1
            for ch in range(0,16,1):
                for test in range(0,100,1):
                    data, badSync, expV = self.get_data_chipXchnX(chip = a, chn = ch, packets = 1)
                    if (len(data) == 0):
                        print ("FEMB_CONFIG--> Sync response bad.  Exiting()")
                        return 1
                    if (badSync == 0):
                        for samp in data[0:len(data)]:
                            if samp != self.ADC_TESTPATTERN[ch]:
                                badSync = 1 
                                print ("FinalSync--> chp {} chn {} looking for {}, found {}".format(a, ch, hex(self.ADC_TESTPATTERN[ch]), hex(samp)))
                            else:
                                badSync = 0 
                                print ("FinalSync--> chp {} chn {} looking for {}, found {}".format(
                                    a, ch, hex(self.ADC_TESTPATTERN[ch]), hex(samp)))
                                break

                            if badSync == 1:
                                break
                    #print("time after checking the sample {}".format(time.time()))
                    if badSync == 0:
                        break

                #print("time after checking 100 samples {}".format(time.time()))
                if badSync == 1:
                    self.femb.write_reg( 9, 1)
                    sys.exit("FEMB_CONFIG--> Failed final check looking at channel test values")

            self.configAdcAsicNew(False)
            print ("FEMB_CONFIG--> Chip {} recieved the correct test values!".format(a))
        
            badSync, syncDicts = self.testUnsyncNew(a) #checks conv error & header error
            if (badSync == 1):
                self.femb.write_reg(9, 1)
                sys.exit("FEMB_CONFIG--> Sync failed in the final check")                

        
        self.femb.write_reg(14,0)
        # clk    # 00-internal (0), 01-external, 10-internal monostable (2), 11-internal
        # frqc   # 0-2MHz Monostable, 1-1MHz monostable
        # en_gr  # 0 disable offset, 1 enable offset
        # slsb   # 0 full current steering, 1 test data
        # f0     # 0 fifo write pointer internal IDL, 1 external IDL
        # f1     # 0 normal, 1 obsolete
        # f2     # 0 signal gen on, 1 signal gen off
        # f3     # 0 IDL delayed 5-10 ns, 1 IDL delay one clk cycle
        # f4     # 
        # f5     # 0 adc data, 1 test data
        self.adc_reg_new.set_adc_chip(chip = a, d = 6, pcsr=1, pdsr=0, slp=0, tstin=1, clk = 2, frqc = 1, f0 = 0, f4 = 0, f5 = 0)
        self.configAdcAsicNew(True)


        for j in range(100):    #reset sync error
            self.femb_eh.write_reg(11, 1) # ERROR_RESET <= reg11_p(0)
            time.sleep(0.01)
            self.femb_eh.write_reg(11, 0) # ERROR_RESET <= reg11_p(0)
            time.sleep(0.01)
            conv_error = self.femb_eh.read_reg(12)     # reg12_i => x"" & CONV_ERROR 
            header_error = self.femb_eh.read_reg(13)   # reg13_i => HDR2_ERROR & HDR1_ERROR 
            error = False
            
            if (conv_error != 0):
                error = True # conv error
                
            if (header_error != 0):
                error = True # header error

            if (error == True):               
                print ("testUnsyncNew() -> conv error: {} header error: {} : {}".format(conv_error,header_error, j)) # no errors

            if (error == False):
                print ("testUnsyncNew() -> conv error: {} header error: {} : {}".format(conv_error,header_error, j)) #break loop if no error found
                break

            elif (j > 60):
                badSync = 1
                break
            else:
                self.configAdcAsicNew(False)
                #print ("Loop {}".format(j))

    def configAdcAsicNew(self, to_print = True, enableOffsetCurrent=None,offsetCurrent=None,testInput=None,
                            freqInternal=None,sleep=None,pdsr=None,pcsr=None,
                            clockMonostable=None,clockExternal=None,clockFromFIFO=None,
                            sLSB=None,f0=None,f1=None,f2=None,f3=None,f4=None,f5=None):

        Adcasic_regs = self.adc_reg_new.REGS
        #Adcasic_regs = self.adc_reg.REGS
        #ADC ASIC SPI registers
        self.femb.write_reg ( self.REG_ASIC_SPIPROG, 0x40)
        self.femb.write_reg ( self.REG_ASIC_SPIPROG, 0)
        time.sleep(0.01)
        self.femb.write_reg ( self.REG_ASIC_SPIPROG, 0x20)
        self.femb.write_reg ( self.REG_ASIC_SPIPROG, 0)
        time.sleep(0.01)
        self.femb.write_reg ( self.REG_ASIC_SPIPROG, 0x40)
        self.femb.write_reg ( self.REG_ASIC_SPIPROG, 0)
        time.sleep(0.01)

        """
        % ADC ASIC CUSTOMIZATION
        """
        if enableOffsetCurrent is None:
            enableOffsetCurrent=0
        if offsetCurrent is None:
            offsetCurrent=0
        else:
            offsetCurrent = int("{:04b}".format(offsetCurrent)[::-1],2) # need to reverse bits, use string/list tricks
        if testInput is None:
            testInput=-1
        if freqInternal is None:
            freqInternal=-1
        if sleep is None:
            sleep=-1
        if pdsr is None:
            pdsr=-1
        if pcsr is None:
            pcsr=-1
        if sLSB is None:
            sLSB = -1
        if f1 is None:
            f1 = -1
        if f2 is None:
            f2 = -1
        if f3 is None:
            f3 = -1
        if f4 is None:
            f4 = -1
        if f5 is None:
            f5 = -1
        if not (clockMonostable or clockExternal or clockFromFIFO):
            clockExternal=True
        clk=0
        if clockExternal:
            clk=2
        elif clockFromFIFO:
            clk=0
        if f0 is None:
            if clockExternal:
                f0 = 1
            else:
                f0 = 0
        if clockExternal:
            self.extClock(enable=True)
        #else:
            #self.extClock(enable=False)

        #self.adc_reg_new.set_adc_chip(chip = 0, d = 6, pcsr=1, pdsr=0, slp=0, tstin=1, clk = clk, frqc = 1, f4 = 0, f5 = 0)

        #self.adc_reg_new.set_adc_chip(en_gr=enableOffsetCurrent,d=offsetCurrent,tstin=testInput,frqc=freqInternal,slp=sleep,pdsr=pdsr,pcsr=pcsr,clk=clk,f0=f0,f1=f1,f2=f2,f3=f3,f4=f4,f5=f5,slsb=sLSB)

        """
        % END ADC ASIC CUSTOMIZATION
        """
        if (to_print == True):
            print ("configAdcAsicNew -> Config ADC ASIC SPI")
        for k in range(10):            
            i = 0
            for regNum in range(self.REG_ADCSPI_BASE,self.REG_ADCSPI_BASE+len(Adcasic_regs),1):
                    self.femb.write_reg( regNum, Adcasic_regs[i])
                    i = i + 1
                    
            if (to_print == True): # Write ADC ASIC SPI to FPGA
                print ("\t FEMB_CONFIG--> CONFIG() -> Program ADC ASIC SPI")
            self.femb.write_reg ( self.REG_ASIC_SPIPROG, 0)
            time.sleep(.05)
            self.femb.write_reg ( self.REG_ASIC_SPIPROG, 1)
            time.sleep(.05)
            self.femb.write_reg ( self.REG_ASIC_SPIPROG, 1)
            time.sleep(.05)

            if (to_print == True): # Print to screen
                print ("FEMB_CONFIG--> Check ADC ASIC SPI")
                
            readback_regs = []
            i = 0
            for regNum in range(self.REG_ADCSPI_RDBACK_BASE,self.REG_ADCSPI_RDBACK_BASE+len(Adcasic_regs),1):
                readback_regs.append(self.femb.read_reg(regNum))
                print("configAdcAsicNew() -> rb reg: {:#010x} from base: {:#010x}".format(regNum, Adcasic_regs[i]))
                i = i + 1
            
            i=0
            for i in range (0,len(readback_regs),1): #Why is this reading out extra adc asic registers?
                if (Adcasic_regs[i] != readback_regs[i]):
                    print ("\t FEMB_CONFIG --> CONFIG() ERROR -> configAdcAsicNew() -> Sent Adcasic_reg not correct = " + str(hex(Adcasic_regs[i])) + " , rb_reg = " + str(hex(readback_regs[i])) )
                else:
                    continue
                    print ("femb_config_sbnd.py -> configAdcAsicNew() -> Adcasic_reg correct = " + str(hex(Adcasic_regs[i])) + " , rb_reg = " + str(hex(readback_regs[i])) )
            
#            val = self.femb.read_reg ( self.REG_ASIC_SPIPROG ) 
            wrong = False
            if (wrong == True and k == 9):
                print ("\tFEMB_CONFIG--> CONFIG() -> SPI_Status")
                print (hex(val))
                sys.exit("\tFEMB_CONFIG--> CONFIG() -> femb_config_femb : Wrong readback. ADC SPI failed")
                return
                
            elif (wrong == 0): 
                if (to_print == True):
                    print("FEMB_CONFIG--> ADC ASIC SPI is OK")
                break
          
    def extClock(self, enable=True):
        self.femb_eh.write_reg(14, 0) #set 200MHz adc clock      

        clock = 1./ self.FPGA_FREQ_MHZ * 1000. # clock now in ns (5*10^(-9))
        mult = 1
        denominator = clock/mult
        cl1 = self.EC_RST_OFF // denominator
        cl2 = self.EC_RST_WID // denominator
        cl3 = self.EC_RD_OFF // denominator
        cl4 = self.EC_RD_WID // denominator
        cl5 = self.EC_IDXM_OFF // denominator
        cl6 = self.EC_IDXM_WID // denominator
        cl7 = self.EC_IDXL_OFF // denominator
        cl8 = self.EC_IDXL_WID // denominator
        cl9 = self.EC_IDL1_OFF // denominator
        cl10 = self.EC_IDL1_WID // denominator
        cl11 = self.EC_IDL2_OFF // denominator
        cl12 = self.EC_IDL2_WID // denominator
        inv=0
        if self.inv_rst:      #INV_RST
            inv += 1 << 0
        if self.inv_read:     #INV_READ
            inv += 1 << 1
        if self.inv_idxm:     #INV_IDXM
            inv += 1 << 2
        if self.inv_idxl:     #INV_IDXL
            inv += 1 << 3
        if self.inv_idl:      #INV_IDL
            inv += 1 << 4
        #Coarse Control
        self.femb_eh.write_reg(21, inv)
        self.femb_eh.write_reg(22, cl1) # RESET Offset      
        self.femb_eh.write_reg(23, cl2) # RESET Width
        self.femb_eh.write_reg(24, cl3) # READ Offset
        self.femb_eh.write_reg(25, cl4) # READ Width
        self.femb_eh.write_reg(26, cl5)#self.EC_IDXM_OFF)  # IDXM Offset
        self.femb_eh.write_reg(27, cl6)#self.EC_IDXM_WID)  # IDXM Width        
        self.femb_eh.write_reg(28, cl7)#self.EC_IDXL_OFF)  # IDXL Offset
        self.femb_eh.write_reg(29, cl8)#self.EC_IDXL_WID)  # IDXL Width
        self.femb_eh.write_reg(30, cl9)#self.EC_IDL1_OFF)  # IDL1 Offset
        self.femb_eh.write_reg(31, cl10)#self.EC_IDL1_WID)  # IDL1 Width         
        self.femb_eh.write_reg(32, cl11)#self.EC_IDL2_OFF)  # IDL2 Offset
        self.femb_eh.write_reg(33, cl12)#self.EC_IDL2_WID)  # IDL2 Width
        #Fine Control
        self.femb_eh.write_reg(34, self.EC_PLL_STEP0) # C0 & C1 fine clock settings   
        self.femb_eh.write_reg(35, self.EC_PLL_STEP1) # C2 & C3 fine clock settings
        self.femb_eh.write_reg(36, self.EC_PLL_STEP2) # C2 & C3 fine clock settings

    def syncADCNew(self,iASIC=None):
        print("\nsyncADCNew - BEGIN\n")
        alreadySynced = True
        for a in range(0,self.NASICS,1):
            print("FEMB_CONFIG--> Test ADC " + str(a))
            self.adc_reg_new.set_adc_global(chip = a, f5 = 1) # Test DATA
            self.configAdcAsicNew(False)
            unsync, syncDicts = self.testUnsyncNew(a)
            if unsync != 0:
                alreadySynced = False
                print("FEMB_CONFIG--> ADC not synced, try to fix")
                response = self.fixUnsyncNew(a)
                if (response != True):
                    sys.exit (" FEMB_CONFIG --> ADC {} could not sync".format(a))
            elif (unsync == 1):
                print ("FEMB_CONFIG--> ADC {} synced!".format(a))
                
            self.adc_reg_new.set_adc_global(chip = a, f5 = 1) # Test DATA
            self.configAdcAsicNew(True)
            
        self.FinalSyncCheck()
        latchloc1_4 = self.femb.read_reg(self.REG_LATCHLOC1_4) 
        clkphase    = self.femb.read_reg(self.REG_CLKPHASE)
        if self.SAMPLERATE == 1e6:
            self.REG_LATCHLOC1_4_data_1MHz = latchloc1_4
            self.REG_CLKPHASE_data_1MHz    = clkphase
        else: # 2 MHz
            self.REG_LATCHLOC1_4_data_2MHz = latchloc1_4
            self.REG_CLKPHASE_data_2MHz    = clkphase

        print ("FEMB_CONFIG--> Final Latch latency " + str(hex(latchloc1_4)))
        print ("FEMB_CONFIG--> Final Phase Shift " +   str(hex(clkphase)))
        print ("FEMB_CONFIG--> ADC passed Sync Test!")

        return not alreadySynced,latchloc1_4,clkphase 

    def resetBoardNew(self): #cp 1/17/18
        #
        #   resetFEMBBoard()
        #       sends a 1 to register 1 for "instantaneous" ASIC reset.
        #       sends a 0 to register 1 to ensure reset is not locked in.
        #
        #       procedure:
        #           line 29: FPGA to ASIC reset
        #           line 30: wait 5 ms
        #           line 31: FPGA to ASIC disable reset
        #
        
        print ("resetBoardNew - BEGIN")
        #Reset FEMB system
        self.femb.write_reg ( self.REG_RESET, 1)
        time.sleep(5.)
        self.femb.write_reg ( self.REG_RESET, 0)
        time.sleep(2.)
        print ("FEMB_CONFIG--> Reset FEMB is DONE\n")


    def initBoardNew(self):
        #
        # initBoard()
        # 	set up default registers. 
        # 	initializes clock settings
        #
        print("\ninitBoardNew - BEGIN\n")

        # Frame size is multiple of 13, so 0xFACE is consistently the first 2 bytes.
        frame_size = self.frame_size
        if (frame_size%13 != 0):
            frame_size = 13 * (frame_size//13)
            
        self.femb.write_reg(10, frame_size)
        time.sleep(0.1)
    
        if (self.femb.read_reg(10) != frame_size):
            sys.exit("FEMB_CONFIG--> initBoard() -> Frame Size not set correctly, something wrong with FPGA communication")
        
        print ("FEMB_CONFIG--> initBoard() -> Chip tester version {}".format(hex(self.femb.read_reg(0x101))))

        # Set to WIB Mode and start by reading out chip 1
        # Channel Setting is irrelevant in WIB mode
        self.femb.write_reg(8, 0x80000001)                  # WIB_MODE   <= reg8_p(0)
        self.femb.write_reg(7, 0x80000000)                  # CHN_select <= reg7_p(7 downto 0)

        """ SHIFT DATA BUS """
        # LATCH_LOC_0 <= reg4_p(7 downto 0)
        self.femb.write_reg(self.REG_LATCHLOC1_4, self.REG_LATCHLOC1_4_data_2MHz)
        
        """ SHIFT DATA BUS """
        # CLK_selectx <= reg6_p(7 downto 0)
        self.femb.write_reg(self.REG_CLKPHASE, self.REG_CLKPHASE_data_2MHz)
        inv=0
        if self.inv_rst:      #INV_RST
            inv += 1 << 0
        if self.inv_read:     #INV_READ
            inv += 1 << 1
        if self.inv_idxm:     #INV_IDXM
            inv += 1 << 2
        if self.inv_idxl:     #INV_IDXL
            inv += 1 << 3
        if self.inv_idl:      #INV_IDL
            inv += 1 << 4

        clock = 1./ self.FPGA_FREQ_MHZ * 1000. # clock now in ns (5*10^(-9))
        mult = 1
        denominator = clock/mult
        cl1 = self.EC_RST_OFF // denominator
        cl2 = self.EC_RST_WID // denominator
        cl3 = self.EC_RD_OFF // denominator
        cl4 = self.EC_RD_WID // denominator
        cl5 = self.EC_IDXM_OFF // denominator
        cl6 = self.EC_IDXM_WID // denominator
        cl7 = self.EC_IDXL_OFF // denominator
        cl8 = self.EC_IDXL_WID // denominator
        cl9 = self.EC_IDL1_OFF // denominator
        cl10 = self.EC_IDL1_WID // denominator
        cl11 = self.EC_IDL2_OFF // denominator
        cl12 = self.EC_IDL2_WID // denominator
        """
        print("set RSET CLK OFF: {}".format(cl1))
        print("set RSET CLK WID: {}".format(cl2))
        print("set READ CLK OFF: {}".format(cl3))
        print("set READ CLK WID: {}".format(cl4))
        print("set IDXM CLK OFF: {}".format(cl5))
        print("set IDXM CLK WID: {}".format(cl6))
        print("set IDXL CLK OFF: {}".format(cl7))
        print("set IDXL CLK WID: {}".format(cl8))
        print("set IDL1 CLK OFF: {}".format(cl9))
        print("set IDL1 CLK WID: {}".format(cl10))
        print("set IDL2 CLK OFF: {}".format(cl11))
        print("set IDL2 CLK WID: {}".format(cl12))
        """
        self.femb_eh.write_reg(21, inv)
        self.femb_eh.write_reg(14, 0) #set 200MHz adc clock      
        self.femb_eh.write_reg(22, cl1) # RESET Offset      
        self.femb_eh.write_reg(23, cl2) # RESET Width
        self.femb_eh.write_reg(24, cl3) # READ Offset
        self.femb_eh.write_reg(25, cl4) # READ Width
        self.femb_eh.write_reg(26, cl5)#self.EC_IDXM_OFF)  # IDXM Offset
        self.femb_eh.write_reg(27, cl6)#self.EC_IDXM_WID)  # IDXM Width        
        self.femb_eh.write_reg(28, cl7)#self.EC_IDXL_OFF)  # IDXL Offset
        self.femb_eh.write_reg(29, cl8)#self.EC_IDXL_WID)  # IDXL Width
        self.femb_eh.write_reg(30, cl9)#self.EC_IDL1_OFF)  # IDL1 Offset
        self.femb_eh.write_reg(31, cl10)#self.EC_IDL1_WID)  # IDL1 Width         
        self.femb_eh.write_reg(32, cl11)#self.EC_IDL2_OFF)  # IDL2 Offset
        self.femb_eh.write_reg(33, cl12)#self.EC_IDL2_WID)  # IDL2 Width
        #Fine Control
        self.femb_eh.write_reg(34, self.EC_PLL_STEP0) # C0 & C1 fine clock settings   
        self.femb_eh.write_reg(35, self.EC_PLL_STEP1) # C2 & C3 fine clock settings
        self.femb_eh.write_reg(36, self.EC_PLL_STEP2) # C2 & C3 fine clock settings

        #set default value to FEMB ADCs
        #clk = 2 is external
        #clk = 0 is internal
        self.adc_reg_new.set_adc_board(d=6, pcsr=1, pdsr=0, slp=0, tstin=1,
                 clk = 0, frqc = 1, en_gr = 0, f0 = 0, f1 = 0, 
                 f2 = 0, f3 = 0, f4 = 0, f5 = 1, slsb = 0) # set to internal 1/31/18 cp
                 
        self.femb.write_reg( self.REG_ASIC_SPIPROG, 0x30)
        self.femb.write_reg( self.REG_ASIC_SPIPROG, 0)
        self.configAdcAsicNew(False)
