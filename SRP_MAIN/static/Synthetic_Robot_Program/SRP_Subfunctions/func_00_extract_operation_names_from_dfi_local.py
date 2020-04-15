import tkinter as tk
from tkinter import filedialog
from tkinter import *
import json

class dfi_to_skils:
    
    def __init__(self, master):
        frame = Frame(master)
        frame.pack()


        self.importFileButton = Button(frame, text = "import file", command = self.importFile)
        #self.importFileButton.pack(side=LEFT)

        self.quitButton = Button(frame, text = "Quit", command=frame.quit)
        self.quitButton.pack(side=LEFT)

        self.processingButton = Button(frame, text = "processing", command= self.processFile)
        self.processingButton.pack(side=LEFT)

    def importFile(self):
        filename = filedialog.askopenfilename()
        print('Selected:', filename)
    
    def processFile(self):
        filename = filedialog.askopenfilename()
        print('Selected:', filename)


        import xml.etree.ElementTree as ET
        tree = ET.parse(str(filename)) #DFI file
        root = tree.getroot()

        index = 0
        # Parsing through ".../proceduralStep/figure" to find attribute "title"
        for figure in root.findall("content/procedure/mainProcedure/proceduralStep/figure"): #structure inside xml file until the subelement before the information we are seeking
            try:
                title = figure.find('title').text
            except:
                title = None

        data = {}

        # Parsing through ".../proceduralStep/fiSentence" to find attribute "original"
        for fiSentence in root.findall("content/procedure/mainProcedure/proceduralStep/proceduralStep/fiSentence"): 
            
            try:
                original = fiSentence.find('original').text
            except:
                original = None

            # Parsing through ".../proceduralStep/fiSentence/components" to find the rest of the attributes "operation", "target", "targetState" etc
            for components in fiSentence.findall("components"): 
            
                try:
                    operation = components.find('operation').text
                except:
                    operation = None
                try:
                    target = components.find('target').text
                except:
                    target = None
                try:
                    targetState = components.find('targetState').text
                except:
                    targetState = None
                try:
                    referenceToSpecification = components.find('referenceToSpecification').text
                except:
                    referenceToSpecification = None
                try:
                    relation = components.find('relation').text
                except:
                    relation = None
                try:
                    operationManner = components.find('operationManner').text
                except:
                    operationManner = None
                try:
                    aim = components.find('aim').text
                except:
                    aim = None
            
        # Store data
            tempDict = {"original":original, "operation":operation, "target":target, "targetState":targetState, "referenceToSpecification":referenceToSpecification, "relation":relation, "operationManner":operationManner, "aim":aim}
            tempDict = {k: v for k, v in tempDict.items() if v != None}
            data["procedural_step_"+str(index)] = tempDict

            index+=1
        #data["number_of_procedural_steps:"] = index

        print(json.dumps(data, indent = 4))
    

root = Tk()
b = dfi_to_skils(root)
root.mainloop()