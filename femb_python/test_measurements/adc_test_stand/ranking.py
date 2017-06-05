"""
Code for ranking ADCs. More generally analyzing multiple summary json files
produced in ADC tests.
"""

from __future__ import print_function
from __future__ import division
from __future__ import unicode_literals
from __future__ import absolute_import
from builtins import range
from future import standard_library
standard_library.install_aliases()
import os
import os.path
import time
import datetime
import glob
from uuid import uuid1 as uuid
import json
import numpy
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
from matplotlib.figure import Figure

class RANKING(object):

    def __init__(self,inglobstrs):
        """
        inglobstr is a string that can be parsed by glob to make a list of
        directories containing json files or json files to be analyzed.
        Directories found in the glob will be walked to find subdirectories 
        containing json files.
        """
        jsonpaths = []
        for inglobstr in inglobstrs:
            inpathlist = glob.glob(inglobstr)
            for inpath in inpathlist:
                for directory, subdirs, filenames in os.walk(inpath):
                    for filename in filenames:
                        if os.path.splitext(filename)[1] == ".json" and "adcTest" == filename[:7] and not ("Raw" in filename):
                            jsonpath = os.path.join(directory,filename)
                            jsonpaths.append(jsonpath)
                            print(jsonpath)
        datadicts = []
        for jsonpath in jsonpaths:
            with open(jsonpath) as jsonfile:
                datadict = json.load(jsonfile)
                datadicts.append(datadict)
        self.datadicts = datadicts

        # 0 for min 1 for max
        self.statsToDraw = {
            "DNL75perc400": 1,
            "DNLmax400": 1,
            "INLabs75perc400": 1,
            "INLabsMax400": 1,
            "codeAtZeroV": 1,
            "minCode": 1,
            "minCodeV": 1,
            "maxCode": 0,
            "maxCodeV": 0,
            "inputPin_mean": 1,
            "meanCodeFor0.2V": 1,
            "meanCodeFor1.6V": 0,
            "sinads_f62365.0_a0.6": 0,
            "sinads_f951512.5_a0.6": 0,
            "stuckCodeFrac400": 1,
        }

    def rank(self):
        datadicts = self.getlatestdata()
        minMaxDicts = {}
        statNames = set()
        for datadict in datadicts:
            asic_stats = {}
            serial = datadict["serial"]
            print(serial)
            static = datadict["static"]
            asic_stats.update(self.getminmaxstats(static))
            dynamic = datadict["dynamic"]
            asic_stats.update(self.getminmaxstats(dynamic,True))
            try:
                inputPin = datadict["inputPin"]
            except KeyError:
                print("Warning: now input pin stats for chip",serial)
            else:
                asic_stats.update(self.getminmaxstats(inputPin,inputPin=True))
            try:
                dc = datadict["dc"]
            except KeyError:
                print("Warning: now input pin stats for chip",serial)
            else:
                asic_stats.update(self.getminmaxstats(dc))
            minMaxDicts[serial] = asic_stats
            for stat in asic_stats:
                statNames.add(stat)
        statNames = sorted(list(statNames))
        print("AllStatNames:")
        print(sorted(statNames))
        maxVals = {}
        minVals = {}
        for statName in statNames:
            minVals[statName] = []
            maxVals[statName] = []
            for serial in minMaxDicts:
                try:
                    minVals[statName].append(minMaxDicts[serial][statName][0])
                    maxVals[statName].append(minMaxDicts[serial][statName][1])
                except KeyError as e:
                    print("Warning KeyError for asic: ",serial," stat: ",statName," error: ",e)
                    pass
        statNamesToDraw = sorted(self.statsToDraw)
        nStats = len(statNamesToDraw)
        print("statNamesToDraw:")
        print(statNamesToDraw)
        nx = 4
        ny = 4
        nPerFig = nx*ny
        for iFig in range(int(numpy.ceil(nStats/nPerFig))):
            fig, axes2D = plt.subplots(4,4,figsize=(12,12))
            #fig.subplots_adjust(left=0.07,right=0.93,bottom=0.05,top=0.95,wspace=0.25)
            axes = [y for x in axes2D for y in x]
            for iAx, ax in enumerate(axes):
                try:
                    statName = statNamesToDraw[iFig*nPerFig+iAx]
                except IndexError:
                    ax.axis("off")
                    continue
                else:
                    ax.set_ylabel("ASICs / bin")
                    try:
                        if self.statsToDraw[statName] == 0: # min
                            ax.hist(minVals[statName],histtype="step")
                            ax.set_xlabel("min({})".format(statName))
                        else: #max
                            ax.hist(maxVals[statName],histtype="step")
                            ax.set_xlabel("max({})".format(statName))
                    except KeyError as e:
                        print("Warning: Could not find stat to draw",e)
                    self.set_xticks(ax)
                    ax.set_ylim(0,ax.get_ylim()[1]*1.2)
            plt.tight_layout()
            fig.savefig("ADC_ranking_page{}.png".format(iFig))
            fig.savefig("ADC_ranking_page{}.pdf".format(iFig))

    def set_xticks(self,ax):
        xlim = ax.get_xlim()
        xticks = numpy.linspace(xlim[0],xlim[1],4)
        xticklabels = ["{:.1g}".format(x) for x in xticks]
        ax.set_xticks(xticks)

    def getminmaxstats(self,data,dynamic=False,inputPin=False):
        result = {}
        stats = []
        clocks = sorted(data.keys())
        offsets = []
        amps = []
        freqs = []
        for clock in clocks:
            offsets = sorted(data[clock])
            for offset in offsets:
                stats = sorted(data[clock][offset])
                if dynamic:
                    for stat in stats: 
                        amps = data[clock][offset][stat]
                        for amp in amps:
                            freqs = data[clock][offset][stat][amp]
                    break
                break
            break
        for stat in stats:
            minVal = 1e20
            maxVal = -1e20
            if dynamic:
                for amp in amps:
                    for freq in freqs:
                        minVal = 1e20
                        maxVal = -1e20
                        for clock in clocks:
                            for offset in offsets:
                                for chan in range(16):
                                    val = data[clock][offset][stat][amp][freq][chan]
                                    minVal = min(val,minVal)
                                    maxVal = max(val,maxVal)
                        result["{}_f{:.1f}_a{:.1f}".format(stat,float(freq),float(amp))] = [minVal,maxVal]
            else:
                minVal = 1e20
                maxVal = -1e20
                for clock in clocks:
                    for offset in offsets:
                          for chan in range(16):
                               val = data[clock][offset][stat][chan]
                               minVal = min(val,minVal)
                               maxVal = max(val,maxVal)
                if inputPin:
                    print("stat:",stat)
                    result["inputPin_"+stat] = [minVal,maxVal]
                else:
                    result[stat] = [minVal,maxVal]
        return result

    def getlatestdata(self):
        """
        Returns list of data dicts, only the 
        latest timesamped one per serial.
        """
        datadicts = self.datadicts
        #for datadict in datadicts:
        #    print(datadict["serial"],datadict["timestamp"])
        resultdict = {} # key is serial, value is datadict
        for datadict in datadicts:
            serial = datadict["serial"]
            try:
                olddata = resultdict[serial]
            except KeyError:
                resultdict[serial] = datadict
            else:
                oldtimestamp = olddata["timestamp"]
                newtimestamp = datadict["timestamp"]
                oldtimestamp = datetime.datetime.strptime(oldtimestamp,"%Y-%m-%dT%H:%M:%S")
                newtimestamp = datetime.datetime.strptime(newtimestamp,"%Y-%m-%dT%H:%M:%S")
                if newtimestamp > oldtimestamp:
                  resultdict[serial] = datadict
        #print("result:")
        sortedserials = None
        try:
            sortedserials = sorted(resultdict,key=lambda x: int(x))
        except:
            sortedserials = sorted(resultdict)
        result = []
        for serial in sortedserials:
            datadict = resultdict[serial]
        #    print(datadict["serial"],datadict["timestamp"])
            result.append(datadict)
        return result

def main():
    from ...configuration.argument_parser import ArgumentParser
    parser = ArgumentParser(description="Plots a ranking of ADCs")
    parser.add_argument("infilename",help="Input json file names and/or glob string.",nargs="+")
    args = parser.parse_args()
  
    #from ...configuration import CONFIG
    #config = CONFIG()

    globstr =  "/home/jhugon/dune/coldelectronics/femb_python/hothdaq*"
    ranking = RANKING(args.infilename)
    ranking.rank()
