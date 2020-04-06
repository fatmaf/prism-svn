from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

from CanvasWorld import CanvasWorld

from GoalsUtilities import GoalsFileReader
from GoalsUtilities import GridGuiGoalsDialog
from HelperClasses import GridFileReader
from HelperClasses import GridGuiStaTraDialog
#import pyscreenshot as ImageGrab

class MainApp(object):

    StapuCanvasSide = 'left'
    SsiCanvasSide = 'right'
    def loadGridFile(self):
        fn = tkFileDialog.askopenfilename(initialfile="smallerShelfDepot_r10_g10_a1_fs79_fsp_90_6_d_1_.grid",initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Grid File", filetypes=(("grid",".grid"),("all files","*.*")))
        foldername = fn[:fn.rfind("/")+1]
        filename = fn[fn.rfind("/")+1:]
        filename = filename.replace('.grid','')
        self.folderguide = foldername
        self.fnguide = filename 
        print ("Loading "+ filename)
        gfr = GridFileReader()
        (row,column,size,xydict)=gfr.readGridFile(fn)
        self.initialiseStapuSsiCanvases(size,column,row)
        #so now we can fill a canvas with xy dict
        self.stapucanvas.fillCanvasWithCells(xydict)
        self.stapucanvas.packCanvas(MainApp.StapuCanvasSide)
        self.ssicanvas.fillCanvasWithCells(xydict)
        self.ssicanvas.packCanvas(MainApp.SsiCanvasSide)


    def loadGoalsFile(self):
        if self.folderguide is None:
            fn = tkFileDialog.askdirectory(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Properties File Directory")
        else:
            fn = tkFileDialog.askdirectory(initialdir=self.folderguide,title="Open Properties File Directory")
        folder = fn +'/'
        if self.fnguide is None:
            fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Goals File", filetypes=(("prop",".prop"),("all files","*.*")))
            filename = fn 
        else:
            filename = folder + self.fnguide+'.prop'
        print ("Loading file "+filename)
        gfr = GoalsFileReader()
        goals = gfr.readPropsFile(filename)
        print ("Goals "+str(goals))
        if self.fnguide is None:
            fnguide = filename[filename.rfind('/')+1:filename.rfind('.prop')]
        else:
            fnguide = self.fnguide
        print (fnguide)

        #you can either enter the goals by hand or select a file for them
        
        goalsHere = None
        
        #if(d.goalsArray is None):
        if self.folderguide is None:
            fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/results/logs/",title="Open Goals File", filetypes=(("stapu",fnguide+"*_stapu.txt"),("all files","*.*")))
        else:
            fn = tkFileDialog.askopenfilename(initialdir=self.folderguide+"results/logs/",title="Open Goals File", filetypes=(("stapu",fnguide+"*_stapu.txt"),("all files","*.*")))
            #we just parse the filename
        if fn:
            import re
            regex = ur"_G:\d-\[([\d\s?,]*)\]"
            matches = re.findall(regex,fn)
            print (matches)
            matches = '['+matches[0]+']'
            goalsHere = eval(matches)
        else:
            d = GridGuiGoalsDialog(self.app)
            goalsHere = d.goalsArray

        #now we just do the goals
        #set the goals for both stapu and ssi
        self.stapucanvas.setCurrentGoalsAndGoalDict(goals,goalsHere)
        self.ssicanvas.setCurrentGoalsAndGoalDict(goals,goalsHere)
        #now we've got all the goals
        #now we set them up
        #we just update the canvas with them 
                
        #print(d.goalsArray)
        
    def loadStaTra(self):
        if self.folderguide is None:
            d = GridGuiStaTraDialog(self.app)
        else:
            print ("folderguide "+self.folderguide)
            GridGuiStaTraDialog.defaultFolder =self.folderguide+"results/logs/debugRes/extras"
            d = GridGuiStaTraDialog(self.app)
        staputraname = d.staputraname
        stapustaname = d.stapustaname
        ssitraname = d.ssitraname
        ssistaname = d.ssistaname
        if(stapustaname is not None):
            self.stapucanvas.setPolicyObj(stapustaname,staputraname)
        if ssistaname is not None:
            self.ssicanvas.setPolicyObj(ssistaname,ssitraname)
        
    def initialiseStapuSsiCanvases(self,updatedSize=50,column=10,row=10):
        self.stapucanvas.initialiseOrResetCanvas(self.app,updatedSize,column,row)
        self.ssicanvas.initialiseOrResetCanvas(self.app,updatedSize,column,row)
        self.stapucanvas.fillCanvasWithCells()
        self.ssicanvas.fillCanvasWithCells()
        self.stapucanvas.packCanvas(MainApp.StapuCanvasSide)
        self.ssicanvas.packCanvas(MainApp.SsiCanvasSide)

    def listPaths(self):
        self.stapucanvas.listPaths()
        self.ssicanvas.listPaths()

    def setNumGoalsAndAgents(self):
        numgoals = tkSimpleDialog.askinteger("Input","Goals",parent=self.app,minvalue=1,maxvalue = 10)
        if numgoals is not None:
            print (numgoals)
        else:
            numgoals = 4
            print ("Setting Num Goals to default")
        numagents = tkSimpleDialog.askinteger("Input","Robots",parent=self.app,minvalue=1,maxvalue=10)
        if numagents is not None:
            print(numagents)
        else:
            numagents = 4
            print("Setting num robots to default")

        self.stapucanvas.showRandomAgentsAndGoals(numgoals,numagents)
        self.ssicanvas.showRandomAgentsAndGoals(numgoals,numagents)
            

    def exitApp(self):
        self.app.destroy()
        
    def generateMenu(self):
        menu = Menu(self.app)
        menu.add_command(label='Grid File',command=self.loadGridFile)
        menu.add_command(label='Goals File',command=self.loadGoalsFile)
        menu.add_command(label='Policy Files',command=self.loadStaTra)
        menu.add_command(label='Set Num Goals and Agents',command = self.setNumGoalsAndAgents)
        menu.add_command(label='Move Agents',command=self.moveAgentsOnBothCanvases)
        menu.add_command(label='Start Animation',command=self.animateAgentsOnBothCanvases)
        menu.add_command(label='Stop Animation',command=self.stopAnimationOnBothCanvases)
        menu.add_command(label='Increase Animation Speed',command=self.increaseAnimationSpeed)
        menu.add_command(label='Load MDP For Simulation',command=self.loadMDPFile)
        menu.add_command(label='Simulate Policy Animation',command=self.moveAgentsSimulatePolicies)
        menu.add_command(label='Simulate Policy Step',command=self.moveAgentsSimulatePoliciesStep)
        menu.add_command(label='List Paths',command=self.listPaths)
        menu.add_command(label='Screenshot',command=self.screenshotCanvases)
        menu.add_command(label='Exit',command=self.exitApp)
        self.app.config(menu=menu)

    #def grabCanvas(self,canvasobj,name):
    #    canvas = canvasobj.canvasbbox()  # Get Window Coordinates of Canvas
    #    self.grabcanvas = ImageGrab.grab(bbox=canvas).save(name+".pdf")
        
    def screenshotCanvases(self):
        #self.grabCanvas(self.stapucanvas,"stapucanvas")
        #self.grabCanvas(self.ssicanvas,"ssicanvas")
        self.stapucanvas.canvas.postscript(file="stapucanvas.ps",colormode='color')
        self.ssicanvas.canvas.postscript(file="ssicanvas.ps",colormode='color')

    def loadMDPFile(self):
        if self.folderguide is None:
            fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/results/logs/",title="Open MDP File", filetypes=(("sta","*.sta"),("all files","*.*")))
        else:
            fn = tkFileDialog.askopenfilename(initialdir=self.folderguide+"results/logs/",title="Open MDP File", filetypes=(("sta","*.sta"),("all files","*.*")))
        fn = fn.replace('.sta','')
        self.stapucanvas.setPolicySimObj(fn)
        self.ssicanvas.setPolicySimObj(fn)
        #self.stapucanvas.doPolicySim = True
        #self.ssicanvas.doPolicySim = True
        self.maxPolSims = 1000
        self.polRunNum = 0
        self.numGoalsStapu = 0
        self.numGoalsSSI = 0 
        

    def moveAgentsSimulatePolicies(self):
        doMove = self.moveAgentsSimulatePoliciesStep()
        if(self.polRunNum < self.maxPolSims):
            self.stapucanvas.doPolicySim = True
            self.ssicanvas.doPolicySim = True
            doMove = False 
        if not doMove:
            self.app.after(self.animationSpeed,self.moveAgentsSimulatePolicies)
            
    def moveAgentsSimulatePoliciesStep(self):
        if(self.polRunNum < self.maxPolSims):
            self.stapucanvas.doPolicySim = True
            self.ssicanvas.doPolicySim = True
            moveagentsstapu = self.stapucanvas.moveAgents()
            moveagentsssi = self.ssicanvas.moveAgents()
            if (moveagentsstapu and moveagentsssi):
                #reset things
                numGoalSTAPU = self.stapucanvas.resetEverythingForPolSim()
                numGoalSSI = self.ssicanvas.resetEverythingForPolSim()
                self.numGoalsStapu = self.numGoalsStapu + numGoalSTAPU
                self.numGoalsSSI = self.numGoalsStapu + numGoalSSI
                self.polRunNum = self.polRunNum + 1
                self.stapucanvas.doPolicySim = False
                self.ssicanvas.doPolicySim = False
            if self.polRunNum == self.maxPolSims:
                stapuavg = float(self.numGoalsStapu)/float(self.polRunNum)
                print ("STAPU avg: "+str(stapuavg))
                ssiavg = float(self.numGoalsSSI)/float(self.polRunNum)
                print ("SSI avg: "+str(ssiavg))
        return (moveagentsstapu and moveagentsssi)
        
        

    def shiftPaths(self):
        self.stapucanvas.shiftAgentPaths(10)
    
        
    def moveAgentsOnBothCanvases(self):
        if self.animateBothCanvases:
            moveagentstapu = self.stapucanvas.moveAgents()
            moveagentsssi = self.ssicanvas.moveAgents()
            if(moveagentstapu and moveagentsssi):
                self.animateBothCanvases = False
                print ("Stopping animation")
            self.app.after(self.animationSpeed,self.moveAgentsOnBothCanvases)
            
        else:
            self.stapucanvas.moveAgents()
            self.ssicanvas.moveAgents()


    def animateAgentsOnBothCanvases(self):
        self.animateBothCanvases = True


    def stopAnimationOnBothCanvases(self):
        self.animateBothCanvases = False
            

    def increaseAnimationSpeed(self):
        if self.animationSpeed > self.maxSpeed:
            self.animationSpeed = self.animationSpeed/self.increaseSpeedBy
            print("Increasing Animation speed to every "+str(self.animationSpeed)+" ms")
        
    def __init__(self):
        self.animationSpeed = 100
        self.increaseSpeedBy = 5
        self.maxSpeed = 0
        self.animateBothCanvases = False
        self.fnguide = None
        self.folderguide = None
        self.app = Tk()
        self.stapucanvas = CanvasWorld('STAPU')
        self.ssicanvas = CanvasWorld('Auctioning')
        #lets load the main window
        self.initialiseStapuSsiCanvases()
        self.generateMenu()
        self.app.title('STAPU/Auctioning Visualiser')

        
        self.app.mainloop()

    
        
        


if __name__=="__main__":
    ma = MainApp()
    
