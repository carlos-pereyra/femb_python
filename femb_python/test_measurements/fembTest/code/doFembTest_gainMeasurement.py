from __future__ import print_function
from __future__ import unicode_literals
from __future__ import division
from __future__ import absolute_import
from builtins import hex
from builtins import open
from builtins import range
from builtins import int
from builtins import str
from future import standard_library
standard_library.install_aliases()
from builtins import object
import sys
import string
from subprocess import call
from time import sleep
import os
import ntpath
import glob
import struct
import json

from femb_python.configuration import CONFIG
from femb_python.write_data import WRITE_DATA
from femb_python.configuration.cppfilerunner import CPP_FILE_RUNNER

#specify location of femb_udp package

class FEMB_TEST_GAIN(object):

    def __init__(self, datadir="data", outlabel="gainMeasurement",fembNum=0):
        #set internal variables
        self.outlabel = outlabel
        self.outpathlabel = os.path.join(datadir, outlabel)
        self.fembNum = int(fembNum)

        print( "FEMB # " + str(fembNum) )

        #import femb_udp modules from femb_udp package
        self.femb_config = CONFIG()
        self.write_data = WRITE_DATA(datadir)
        #set appropriate packet size
        self.write_data.femb.MAX_PACKET_SIZE = 8000
        self.cppfr = CPP_FILE_RUNNER()
	
        #set status variables
        self.status_check_setup = 0
        self.status_record_data = 0
        self.status_do_analysis = 0
        self.status_archive_results = 0

        #misc variables
        self.gain = 0
        self.shape = 0
        self.base = 0
        self.leakage = 0
        self.leakagex10 = 0
        self.buffer = 0
        self.acdc = 0

        #json output, note module version number defined here
        self.jsondict = {'type':'fembTest_gain'}
        self.jsondict['version'] = '1.0'
        self.jsondict['timestamp']  = str(self.write_data.date)

    def check_setup(self):
        #CHECK STATUS AND INITIALIZATION
        print("GAIN MEASUREMENT - CHECKING READOUT STATUS")
        self.status_check_setup = 0

        #make sure output directory exists
        self.write_data.assure_filedir()

        #check if register interface is working
        print("Checking register interface")
        regVal = self.femb_config.femb.read_reg(6)
        if (regVal == None):
            print("Error running doFembTest - FEMB register interface is not working.")
            print(" Turn on or debug FEMB UDP readout.")       
            return
        if ( regVal < 0 ):
            print("Error running doFembTest - FEMB register interface is not working.")
            print(" Turn on or debug FEMB UDP readout.")       
            return
        print("Read register 6, value = " + str( hex( regVal ) ) )

        #check that femb number is valid
        if ( int(self.fembNum) < 0 ) or ( int( self.fembNum) >= self.femb_config.NFEMBS ):
            print("Error running doFembTest - Invalid FEMB # specified.")
            return    

        #do firmware version check HERE
        self.femb_config.selectFemb(self.fembNum)

        #initialize FEMB to known state
        print("Initializing board")
        #self.femb_config.initBoard()
        self.femb_config.initFemb(self.fembNum)

        #check if data streaming is working
        print("Checking data streaming")
        testData = self.write_data.femb.get_data_packets(1)
        if testData == None:
            print("Error running doFembTest - FEMB is not streaming data.")
            print(" Turn on and initialize FEMB UDP readout.")
            #print(" This script will exit now")
            #sys.exit(0)
            return
        if len(testData) == 0:
            print("Error running doFembTest - FEMB is not streaming data.")
            print(" Turn on and initialize FEMB UDP readout.")
            #print(" This script will exit now")
            #sys.exit(0)
            return

        print("Received data packet " + str(len(testData[0])) + " bytes long")

        #check for analysis executables
        if not self.cppfr.exists('test_measurements/fembTest/code/parseBinaryFile'):    
            print('parseBinaryFile not found, run setup.sh')
            #sys.exit(0)
            return

        print("GAIN MEASUREMENT - READOUT STATUS OK" + "\n")
        self.status_check_setup = 1

    def record_data(self):
        if self.status_check_setup == 0:
            print("Please run check_setup method before trying to take data")
            return
        if self.status_record_data == 1:
            print("Data already recorded. Reset/restat GUI to begin a new measurement")
            return
        #MEASUREMENT SECTION
        print("GAIN MEASUREMENT - RECORDING DATA")

        #wait to make sure HS link is back on after check_setup
        sleep(0.5)

        #setup output file
        self.write_data.filename = self.outlabel+".bin"
        print("Recording " + self.write_data.filename )

        isOpen = self.write_data.open_file()
        if isOpen == 0 :
            print( "Error running doFembTest - Could not open output data file for writing, ending test" )
        subrun = 0

        #config FE ASICs
        print("FE ASIC Settings: Gain " + str(self.gain) + ", Shaping Time " + str(self.shape) + ", Baseline " + str(self.base) )
        print("FE ASIC Settings: Leakage Current " + str(self.leakage) + ", Leakage x10 " + str(self.leakagex10) )
        print("FE ASIC Settings: Output Buffer " + str(self.buffer) + ", AC/DC " + str(self.acdc) )
        self.femb_config.feasicGain = self.gain
        self.femb_config.feasicShape = self.shape
        self.femb_config.feasicBaseline = self.base
        self.femb_config.feasicLeakageVal = self.leakage
        self.femb_config.feasicLeakagex10Val = self.leakagex10
        self.femb_config.bufVal = self.buffer
        self.femb_config.acdcVal = self.acdc
        self.femb_config.feasicEnableTestInput = 0
        self.femb_config.configFeAsic()

        #disable pulser
        self.femb_config.setFpgaPulser(0,0x0)

        #record data
        self.write_data.numpacketsrecord = 1000
        self.write_data.run = 0
        self.write_data.runtype = 0
        self.write_data.runversion = 0

        #take initial noise data run
        subrun = 0
        asicCh = 0
        for asic in range(0,self.femb_config.NASICS,1):
          self.femb_config.selectChannel(asic,asicCh)
          self.write_data.record_data(subrun, asic, asicCh)

        #turn ASICs back on, start pulser section
        self.femb_config.feasicEnableTestInput = 1
        self.femb_config.configFeAsic()
        subrun = 1

        #loop over pulser configurations, each configuration is it's own subrun
        for p in range(0,20,1):
            pVal = int(p)
            self.femb_config.setFpgaPulser(1,pVal)
            print("Pulse amplitude " + str(pVal) )

            #loop over channels
            for asic in range(0,8,1):
                self.femb_config.selectChannel(asic,asicCh)
                self.write_data.record_data(subrun, asic, asicCh)

            #increment subrun, important
            subrun = subrun + 1

        #close file
        self.write_data.close_file()

        #turn off FEMB

        print("GAIN MEASUREMENT - DONE RECORDING DATA" + "\n")
        self.status_record_data = 1

    def do_analysis(self):
        if self.status_record_data == 0:
            print("Please record data before analysis")
            return
        if self.status_do_analysis == 1:
            print("Analysis already complete")
            return
        #ANALYSIS SECTION
        print("GAIN MEASUREMENT - ANALYZING AND SUMMARIZING DATA")

        #parse binary
        self.cppfr.run("test_measurements/fembTest/code/parseBinaryFile", [self.write_data.data_file_path])

        #run analysis program
        parseBinaryFile = self.outpathlabel + "-parseBinaryFile.root"
        call(["mv", "output_parseBinaryFile.root" , parseBinaryFile])
        self.cppfr.run("test_measurements/fembTest/code/processNtuple_gainMeasurement",  [parseBinaryFile])

        processNtupleFile = self.outpathlabel + "-processNtupleFile.root"
        call(["mv", "output_processNtuple_gainMeasurement.root" , processNtupleFile])

        summaryPlot = self.outpathlabel + "-summaryPlot.png"
        call(["mv", "summaryPlot_gainMeasurement.png" , summaryPlot])

        resultsFile = self.outpathlabel + "-results.list"
        call(["mv", "output_processNtuple_gainMeasurement.list" , resultsFile])

        print("GAIN MEASUREMENT - DONE ANALYZING AND SUMMARIZING DATA" + "\n")
        self.status_do_analysis = 1

    def archive_results(self):
        #if self.status_do_analysis == 0:
        #    print("Please analyze data before archiving results")
        #    return
        #if self.status_archive_results == 1:
        #    print("Results already archived")
        #    return
        #ARCHIVE SECTION
        print("GAIN MEASUREMENT - ARCHIVE")

        #add summary variables to output
        self.jsondict['status_check_setup'] = str( self.status_check_setup )
        self.jsondict['status_record_data'] = str( self.status_record_data )
        self.jsondict['status_do_analysis'] = str( self.status_do_analysis )
        self.jsondict['filedir'] = str( self.write_data.filedir )
        self.jsondict['config_gain'] = str( self.gain )
        self.jsondict['config_shape'] = str( self.shape )
        self.jsondict['config_base'] = str( self.base )

        if self.status_do_analysis == 1:
          #parse the output results, kind of messy
          listFile = self.outpathlabel + "-results.list"

          lines = []
          with open( listFile ) as infile:
            for line in infile:
                line = line.strip('\n')
                line = line.split(',') #measurements separated by commas
                parseline = {}
                for n in range(0,len(line),1):
                    word = line[n].split(' ')
                    if( len(word) != 2 ):
                        continue
                    parseline[ str(word[0]) ] = str(word[1])
                lines.append(parseline)
            self.jsondict['results'] = lines

        #dump results into json
        jsonFile = self.outpathlabel + "-results.json"
        with open( jsonFile , 'w') as outfile:
            json.dump( self.jsondict, outfile, indent=4)

        print("GAIN MEASUREMENT - DONE STORING RESULTS IN DATABASE" + "\n")
        self.status_archive_results = 1

def main():
    datadir = "data"
    fembNum = 1
    gain = 2
    shape = 1
    base = 0
    if len(sys.argv) == 2 :
      params = json.loads(open(sys.argv[1]).read())
      datadir = params['datadir']
      fembNum = params['fembNum']
      gain = params['gain']
      shape = params['shape']
      base = params['base']

    femb_test = FEMB_TEST_GAIN(datadir,"gainMeasurement",fembNum)
    femb_test.gain = gain
    femb_test.shape = shape
    femb_test.base = base

    femb_test.check_setup()
    femb_test.record_data()
    femb_test.do_analysis()
    femb_test.archive_results()

if __name__ == '__main__':
    main()
