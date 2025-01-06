import os

import sys
sys.path = ["../"] + sys.path
import CppHeaderParser

def inc (ddir):
    try:
        cppHeader = CppHeaderParser.CppHeader(ddir)
    except CppHeaderParser.CppParseError as e:
        print(e)
        sys.exit(1)
    #print("\n#includes are:")
    for incl in cppHeader.includes:
        print("" + ddir +          "    %s"%incl)
    #print("\n#defines are:")
    #for define in cppHeader.defines:
    #    print("define    " + ddir + "             %s"%define)

def traverse(headDir):
    # traverse root directory, and list directories as dirs and files as files
    for root, dirs, files in os.walk("."):
        path = root.split(os.sep)
        print((len(path) - 1) * '---', os.path.basename(root))
      
        for file in files:
            if file.endswith(".h") or file.endswith(".cpp"): 
                # print(os.path.join(directory, filename))
                inc(file)
                continue

                                                        # def travsingleDir(headDir):
                                                            # #ddir = "SampleClass.h"
                                                            # #cwd = os.getcwd()
                                                            # #directory = os.fsencode(cwd)
                                                                
                                                            # for file in os.listdir(headDir):
                                                                # filename = os.fsdecode(file)
                                                                # if filename.endswith(".h") or filename.endswith(".h"): 
                                                                    # # print(os.path.join(directory, filename))
                                                                    # inc (filename)
                                                                    # continue
                                                                # else:
                                                                    # continue
                                                        

headDir = "."
traverse(headDir)
