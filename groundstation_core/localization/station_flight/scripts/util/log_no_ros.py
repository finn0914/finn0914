"""
Copied from repo `drone_core main c0fa0af`.

Used in files that potentially runs without ros, e.g. my_tcp.py
"""

def loginfo(x):
    print(x)
    
def logwarn(s):
    print("\033[93m%s\033[0m"%s) # yellow

def logerr(s):
    print("\033[91m%s\033[0m"%s) # red

def logfatal(s):
    logerr(s)