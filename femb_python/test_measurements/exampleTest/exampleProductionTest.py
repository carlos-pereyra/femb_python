#!/usr/bin/env python3
'''
This provides a command line interface to WIB + FEMB production tests.

It manages a runpolicy object in order to run the testing script
in a manner consistent with other CE testing stations

It maintains a state over a test sequence.
'''

from femb_python import runpolicy

class Test(object):
    def __init__(self, **params):
        self._params = params;
        pass

    def runparams(self):
        '''
        Return parameters that should be passed to a runner's run.
        ''' 
        return self._params

    def __call__(self, runner):
        '''
        Perform the test.
        '''
        params = runner.resolve(**self._params)
        runner(**params)

class Sequencer(object):
    def __init__(self, tests, runner):
        self.tests = tests      # the tests to perform
        self.runner = runner      # a runpolicy object

    def run(self):
        for test in self.tests:
            test(self.runner)

def main(**params):
    '''
    Main entry to the test script.
    '''
    print( "EXAMPLE PRODUCTION TEST - START")

    use_sumatra = False
    test_category = "example"      # pick something

    #parameters specific for a general test, more are defined by runpolicy runner
    #this example uses replacement fields to make it easier to define each individual test
    #main_params = dict(params)
    #main_params.update(
    #    executable = "femb_example_test",      # the program or script actually running the test
    #    #argstr = "{datadir} {outlabel}",      #command line arguments to exectuable
    #    argstr="{paramfile}",        #provide parameter file as argument
    #    datadir = "exampleTest_test_{test}",      # use easy to guess sub directory for each test, recommend defining it here
    #    outlabel = "exampleTest_test_{test}",       # likewise, easy to guess files, recommend defining it here
    #)                                               # note: "test" is filled in the loop below

    # define the tests to perform
    #tests = [Test(test=n, **main_params) for n in range(1,4)]

    #Explicitly define list of production tests to perform
    tests = []

    #Test 0
    params_test_0 = dict(params)
    params_test_0.update(
        executable = "femb_example_test",      # the program or script actually running the test
        argstr="{paramfile}",        #provide parameter file as argument
        datadir = "exampleTest_test_0",      # use easy to guess sub directory for each test, recommend defining it here
        outlabel = "exampleTest_test_0",       # likewise, easy to guess files, recommend defining it here
    )                                               # note: "test" is filled in the loop below
    tests.append( Test(**params_test_0) )

    #Test 1
    params_test_1 = dict(params)
    params_test_1.update( executable = "femb_example_test", argstr="{paramfile}", datadir = "exampleTest_test_1", outlabel = "exampleTest_test_1",)
    tests.append( Test(**params_test_1) )

    ##add more test as needed

    #actually run tests here
    r = runpolicy.make_runner(test_category, use_sumatra, **params)
    if r == None:
      print("EXAMPLE PRODUCTION TEST - ERROR: runpolicy runner could not be defined, production test not started.")
      return
    s = Sequencer(tests, r)
    s.run()

    print( "EXAMPLE PRODUCTION TEST - DONE")
    
if '__main__' == __name__:
    main()