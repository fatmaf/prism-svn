from tkinter import *
import Tkinter, Tkconstants, tkFileDialog, tkSimpleDialog

from CanvasWorld import CanvasWorld

from GoalsUtilities import GoalsFileReader
from GoalsUtilities import GridGuiGoalsDialog
from HelperClasses import GridFileReader
from HelperClasses import GridGuiStaTraDialog

class MainApp(object):

    StapuCanvasSide = 'left'
    SsiCanvasSide = 'right'
    def loadGridFile(self):
        fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Grid File", filetypes=(("grid",".grid"),("all files","*.*")))
        filename = fn[fn.rfind("/")+1:]
        filename = filename.replace('.grid','')
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
        fn = tkFileDialog.askdirectory(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/",title="Open Properties File Directory")
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
        fn = tkFileDialog.askopenfilename(initialdir="/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/smallerwhdoors/results/logs/",title="Open Goals File", filetypes=(("stapu",fnguide+"*_stapu.txt"),("all files","*.*")))
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

    
    def exitApp(self):
        self.app.destroy()
        
    def generateMenu(self):
        menu = Menu(self.app)
        menu.add_command(label='Grid File',command=self.loadGridFile)
        menu.add_command(label='Goals File',command=self.loadGoalsFile)
        menu.add_command(label='Policy Files',command=self.loadStaTra)
        
        menu.add_command(label='Move Agents',command=self.moveAgentsOnBothCanvases)
        menu.add_command(label='Start Animation',command=self.animateAgentsOnBothCanvases)
        menu.add_command(label='Stop Animation',command=self.stopAnimationOnBothCanvases)
        menu.add_command(label='Shift Path', command=self.shiftPaths)
        menu.add_command(label='Exit',command=self.exitApp)
        self.app.config(menu=menu)

        

    def shiftPaths(self):
        self.stapucanvas.shiftAgentPaths(10)
    
        
    def moveAgentsOnBothCanvases(self):
        if self.animateBothCanvases:
            self.stapucanvas.moveAgents()
            self.ssicanvas.moveAgents()
            self.app.after(100,self.moveAgentsOnBothCanvases)
        else:
            self.stapucanvas.moveAgents()
            self.ssicanvas.moveAgents()


    def animateAgentsOnBothCanvases(self):
        self.animateBothCanvases = True


    def stopAnimationOnBothCanvases(self):
        self.animateBothCanvases = False
            

        
    def __init__(self):
        self.animateBothCanvases = False
        self.fnguide = None 
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
    
