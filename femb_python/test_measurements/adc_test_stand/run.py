from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals
from __future__ import absolute_import
from builtins import range
from future import standard_library
standard_library.install_aliases()
from ...femb_udp import FEMB_UDP
from ...test_instrument_interface import RigolDG4000
from ...write_root_tree import WRITE_ROOT_TREE
import time
import datetime
import glob
from uuid import uuid1 as uuid
import json
import numpy
import matplotlib.pyplot as plt
import ROOT
from .collect_data import COLLECT_DATA 
from .static_tests import STATIC_TESTS
from .dynamic_tests import DYNAMIC_TESTS
from .summary_plots import SUMMARY_PLOTS

class ADC_TEST_SUMMARY(object):

    def __init__(self,allStatsRaw,testTime):
        self.allStatsRaw = allStatsRaw
        self.testTime = testTime
        self.allStats = None
        self.makeSummaries()

    def makeSummaries(self):
        """
        makes keys:
        self.staticSummary[chipSerial][offset][statistic][channel]

        from:
        self.allStatsRaw[offset][chipSerial]
        """
        allStatsRaw = self.allStatsRaw
        offsets = sorted(allStatsRaw.keys())
        chipSerials = []
        for offset in offsets:
            chipSerials = sorted(allStatsRaw[offset].keys())
            break
        staticSummary = {}
        for chipSerial in chipSerials:
            staticSummary[chipSerial]={}
            for offset in offsets:
                staticSummary[chipSerial][offset] = self.makeStaticSummary(allStatsRaw[offset][chipSerial]["static"])
        self.staticSummary = staticSummary

    def makeStaticSummary(self,stats):
        """
        returns:
        result[statistic][channel]

        from:
        stats[channel][statistic]
        """
        statNames = stats[0].keys()
        result = {}
        for statName in statNames:
            result[statName] = []
            for iChan in range(16):
                result[statName].append(stats[iChan][statName])
        return result

    def get_serials(self):
        return self.staticSummary.keys()

    def get_summary(self,serial):
        return {"static":self.staticSummary[serial],"serial":serial,"time":self.testTime}

    def write_jsons(self,fileprefix):
        for serial in self.staticSummary:
            filename = fileprefix + "_" + str(serial) + ".json"
            data = self.get_summary(serial)
            with open(filename,"w") as f:
                json.dump(data,f)

def main():
    from ...configuration.argument_parser import ArgumentParser
    from ...configuration import CONFIG
    ROOT.gROOT.SetBatch(True)
    parser = ArgumentParser(description="Runs ADC tests")
    parser.addNPacketsArgs(False,100)
    args = parser.parse_args()
  
    config = CONFIG()

    collect_data = COLLECT_DATA(config,args.nPackets)
    static_tests = STATIC_TESTS(config)
    dynamic_tests = DYNAMIC_TESTS(config)
    startDateTime = datetime.datetime.now().replace(microsecond=0).isoformat()

    allStatsRaw = {}
    #for offset in range(-1,16):
    for offset in [-1,0]:
      offsetStats = {}
      if offset <=0:
        config.configAdcAsic(enableOffsetCurrent=0,offsetCurrent=0)
      else:
        config.configAdcAsic(enableOffsetCurrent=1,offsetCurrent=offset)
      for iChip in range(config.NASICS):
          chipStats = {}
          fileprefix = "adcTestData_{}_chip{}_offset{}".format(startDateTime,iChip,offset)
          collect_data.getData(fileprefix,iChip,adcOffset=offset)
          static_fns = list(glob.glob(fileprefix+"_functype3_*.root"))
          assert(len(static_fns)==1)
          static_fn = static_fns[0]
          staticStats = static_tests.analyzeLinearity(static_fn,diagnosticPlots=False)
          dynamicStats = dynamic_tests.analyze(fileprefix,diagnosticPlots=False)
          chipStats["static"] = staticStats
          chipStats["dynamic"] = dynamicStats
          offsetStats[iChip] = chipStats
      allStatsRaw[offset] = offsetStats
    summary = ADC_TEST_SUMMARY(allStatsRaw,startDateTime)
    summary.write_jsons("adcTest_{}".format(startDateTime))
    for serial in summary.get_serials():
      SUMMARY_PLOTS(summary.get_summary(serial),"adcTest_{}_{}".format(startDateTime,serial),plotAll=True)
    