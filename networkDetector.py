import sys, getopt
import subprocess
import re
import time

class DetectedNetwork:
    def __init__(self):
        self.name = None
        self.address = None
        self.channel = None
        self.quality = None
        self.signal = None
    
    def fill(self, detectedNetworkText):
        for line in detectedNetworkText.split("\n"):
            self.name = self.name or self.get_name(line)
            self.address = self.address or self.get_address(line)
            self.channel = self.channel or self.get_Channel(line)
            self.signal = self.signal or self.get_Signal(line)
            self.quality = self.quality or self.get_Quality(line)

    def get_name(self, line):
        return self.getValue(line,"ESSID:")

    def get_address(self, line):
        return self.getValue(line, "Address: ")

    def get_Channel(self, line):
        return self.getValue(line, "Channel:")

    def get_Quality(self, line):
        qualityLine = self.getValue(line, "Quality=")
        if qualityLine is not None:
            qualityFactor = qualityLine.split()[0].split('/')
            return float(qualityFactor[0]) / float(qualityFactor[1]) * 100
        else:
            return None

    def get_Signal(self, line):
        signalLine = self.getValue(line, "Quality=")
        if signalLine is not None:
            signalDbm = signalLine.split('Signal level=')[1] # get -83 dBm
            return float(re.sub(' dBm$', '', signalDbm.strip()))
        else:
            return None


    def getValue(self, line, keyword):
        line=line.lstrip()
        length=len(keyword)
        if line[:length] == keyword:
            return line[length:]
        else:
            return None

    def __str__(self):
        return "name: {x.name:25}, address: {x.address:10}, channel: {x.channel:2}, quality: {x.quality:2.2f}, signal: {x.signal: 2.2f}".format(x=self)


def mainToConsole():
    proc = subprocess.Popen(["iwlist", "wlan0", "scan"],stdout=subprocess.PIPE, universal_newlines=True)
    out, err = proc.communicate()

    detectedNetworks = []

    for detectedNetworkText in out.split("Cell ")[1:]:
        dn = DetectedNetwork()
        dn.fill(detectedNetworkText[5:])
        detectedNetworks.append(dn)

    for dn in detectedNetworks:
        print(dn)

def mainToFile():
    fo = open("wireless.dat", "w", 1)

    while True:
        proc = subprocess.Popen(["iwlist", "wlan0", "scan"],stdout=subprocess.PIPE, universal_newlines=True)
        out, err = proc.communicate()
        
        detectedNetworks = []
        
        for detectedNetworkText in out.split("Cell ")[1:]:
            dn = DetectedNetwork()
            dn.fill(detectedNetworkText[5:])
            detectedNetworks.append(dn)

        for dn in detectedNetworks:
            fo.write("{dn.name}\t{dn.quality}\t{dn.signal}\n".format(dn=dn))

        time.sleep(0.5)

def main(argv):

    type = None

    try:
        opts, args = getopt.getopt(argv,"hfc",["file","console"])
    except getopt.GetoptError:
        print("sudo networkDetectory.py --file")
        print("sudo networkDetectory.py --console")
        sys.exit(2)
    
    for opt, arg in opts:
        if opt == '-h':
            print("sudo networkDetectory.py --file")
            print("sudo networkDetectory.py --console")
            sys.exit()
        elif opt in ("-f", "--file"):
            type = "file"
        elif opt in ("-c", "--console"):
            type = "console"
    
    if type is None:
        print("sudo networkDetectory.py --file")
        print("sudo networkDetectory.py --console")
    elif type == "console":
        mainToConsole()
    elif type == "file":
        mainToFile()

if __name__ == "__main__":
   main(sys.argv[1:])

