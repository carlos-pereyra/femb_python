import os
import sys
import time
import random
import json

from femb_python.configuration import CONFIG

class TEST_QUAD_EPCS(object):

    def __init__(self, datadir="data", outlabel="OscillatorTesting"):
        self.outpathlabel = os.path.join(datadir, outlabel)
        self.femb_config = CONFIG()

        self.nFlashes = 4
        self.nPages = 10
        self.nTriesWrite = 3
        self.printData = False
        
    def doTesting(self):
        #Initialize board
        self.femb_config.initBoard()

        #QUICK FIX- status of each chip as it passes through test
        flashToSkip = [False]*self.nFlashes # previously defined before erase section

        #Loop through flashes and check status; if status is bad, exit
        for iFlash in range(self.nFlashes):
            #Check the status
            print("\nChecking status of flash %s" %(iFlash))
            boardStatus = self.femb_config.readStatus(iFlash)
            if(boardStatus != 0):
                 print("Flash %s has a problem. Will skip this flash!\n" %(iFlash))
                 flashToSkip[iFlash] = True
                 #return

        #Loop through flashes and erase
        eraseTried = [0]*self.nFlashes
        for iFlash in range(self.nFlashes):
            if flashToSkip[iFlash] == True:
                print("Flash %s has a problem. Will skip this flash!\n" %(iFlash))
                eraseTried[iFlash] = 9999999
                continue
            #Erase Flash 
            self.femb_config.eraseFlash(iFlash)
            boardStatus = self.femb_config.readStatus(iFlash)
            iTries = 1
            while(boardStatus != 0 and iTries <= 3):
                iTries +=1
                print("Error!! Status after erasing is bad. Trying again. Try no. %s" %(iTries))
                self.femb_config.eraseFlash(iFlash)
                boardStatus = self.femb_config.readStatus(iFlash)
            eraseTried[iFlash] = iTries
                
            if(iTries > 3):
                print("Flash %s has a problem. Will skip this flash!\n" %(iFlash))
                flashToSkip[iFlash] = True

            #Check a page (page 5 here) to make sure if things make sense; memory should be erased to 0xFFFFFFFF
            outputData = self.femb_config.readFlash(iFlash, 5)
            print("\nPrinting page 5 of flash %s, expecting all 0xffffffff\n" %(iFlash))
            outputDataHex = [hex(x) for x in outputData]
            print(outputDataHex)
            
        print("\nDone erasing flashes! Begining the tests.\n")

        #Loop over flashes and pages
        failedPages = [0]*self.nFlashes
        programTried = [[0 for iP in range(0, self.nPages)] for iF in range(0, self.nFlashes)]
        for iFlash in range(self.nFlashes):
            if flashToSkip[iFlash]:
                failedPages[iFlash] = 9999999
                print("Flash %s has a problem. Will skip this flash!\n" %(iFlash))
                continue
            for iPage in range(self.nPages):
                inputData = []
                for iNum in range(0, 64):
                    num = random.randint(1, 10000000)
                    inputData.append(num)
                
                self.femb_config.programFlash(iFlash, iPage, inputData)
                outputData = self.femb_config.readFlash(iFlash, iPage)

                if self.printData:
                    inputDataHex = [hex(x) for x in inputData]
                    outputDataHex = [hex(x) for x in outputData]
                    print("\nInput data is:")
                    print(inputDataHex)
                    print()
                    print("\nOutput data is:")
                    print(outputDataHex)
                
                isMatch = set(inputData) == set(outputData)
                iTries = 1
                while (not isMatch and iTries <= self.nTriesWrite):
                    iTries += 1
                    print("*" * 75)
                    print("Input and output data don't match, trying again!\nTry no. %s" %(iTries))
                    print("*" * 75)
                    self.femb_config.programFlash(iFlash, iPage, inputData)
                    outputData = self.femb_config.readFlash(iFlash, iPage)
                    isMatch = set(inputData) == set(outputData)

                    if self.printData:
                        outputDataHex = [hex(x) for x in outputData]
                        print("\nOutput data is:")
                        print(outputDataHex)
                    
                programTried[iFlash][iPage] = iTries
                
                if(iTries > self.nTriesWrite):
                    print("*" * 75)
                    print("Writing to flash %s, page %s failed!" %(iFlash, iPage))
                    print("*" * 75)                    
                    failedPages[iFlash] += 1
                else:
                    print("*" * 75)
                    print("Input and output data match for flash %s, page %s!" %(iFlash, iPage))
                    print("*" * 75)
                    
        #Print the results
        print("*" * 75)
        print("\nPrinting results:")
        print("Tested %s flashes over %s pages." %(self.nFlashes, self.nPages))

        print("\nInfo on Erase: ")
        for iFlash in range(self.nFlashes):
            print("\nNo. of tries to erase flash %s was %s" %(iFlash, eraseTried[iFlash]))
            
        flashSuccess = [False]*self.nFlashes
        print("\nInfo on Write: ")
        print("Note: Will print no. of write tries if > 1 for first 5 page.\nNote: Failing on 9999999 pages means failed to erase and skipped.")
        for iFlash in range(self.nFlashes):
            if(failedPages[iFlash] == 0):
               flashSuccess[iFlash] = True 
               print("\nFlash %s passed!!" %(iFlash))
               for iPage in range(self.nPages):
                   if(programTried[iFlash][iPage] > 1 and iPage < 5):
                       print("No. of program tries for page %s is %s" %(iPage, programTried[iFlash][iPage]))
            else:
               print("\nFlash %s failed!!" %(iFlash))
               print("It failed on %s pages even after %s tries." %(failedPages[iFlash], self.nTriesWrite))
        print("*" * 75)

        #Save the results
        with open(self.outpathlabel+".json", 'w') as outFile:
            json.dump({'No. of tries to erase flash: ':eraseTried}, outFile)
            json.dump({'Passed? : ':flashSuccess}, outFile)
            json.dump({'No. of failed pages: ':failedPages}, outFile)
            json.dump({'No. of program tries for a page: ':programTried}, outFile)
            
def main():
    datadir = sys.argv[1]
    outlabel = sys.argv[2]

    epcsTest = TEST_QUAD_EPCS(datadir, outlabel)

    #Begin testing
    epcsTest.doTesting()


if __name__ == '__main__':
    main()
