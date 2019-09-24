package demos;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceReach;
import automata.DA;
import explicit.LTLModelChecker;
import explicit.MDPModelChecker;
import explicit.Model;
import explicit.ProbModelChecker;
import explicit.LTLModelChecker.LTLProduct;
import explicit.MDP;
import parser.State;
import parser.VarList;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismException;
import prism.PrismSettings;
import simulator.ModulesFileModelGenerator;

public class SingleAgentNestedProductMDPClean
{

	HashMap<Integer,AutomatonInformation> internalAutomatonInfo; 
	Model finalProduct; 
	HashMap<Integer,Integer> vlMapping; 
	boolean homogenous = false; 
	int numTotalAgents; 
	ArrayList<ArrayList<Integer>> initialStates; 
	
	
	public SingleAgentNestedProductMDPClean(Prism prism,ArrayList<AutomatonInformation> automatonList, ModulesFile modulesFile, PropertiesFile propertiesFile, Model model, boolean allStatesInDFA) throws PrismException
	{
		//step1 we create our own automaton list 
		//step2 we create the nested product 
		//no we just interleave the two 
		internalAutomatonInfo = new HashMap<Integer,AutomatonInformation>(); 
		
		MDPModelChecker mc = new MDPModelChecker(prism); 
		mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));
		LTLModelChecker ltlmc = new LTLModelChecker(mc);
	
	
		//lets do this bois 
		//gurls ? 
		//people!!! 
		 
		Model nestedProductModel = (MDP)model; 
		////profile
		long startTime = System.currentTimeMillis();
		for(int daNum = 0; daNum<automatonList.size(); daNum++)
		{
			long startTime1 = System.currentTimeMillis();
			AutomatonInformation daInfo = new AutomatonInformation(automatonList.get(daNum)); 

			Vector<BitSet> labelBS = daInfo.generateDA(ltlmc, mc, nestedProductModel);
			
			

			LTLProduct<Model> product = ltlmc.constructProductModel(daInfo.da, nestedProductModel, labelBS, null, allStatesInDFA);
			nestedProductModel = product.getProductModel(); 
			String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
			//System.getProperty("user.dir");
			String modelLocation = dir;
			String resLoc = dir + "results/";
			nestedProductModel.exportToDotFile(resLoc+"pda_"+daNum+".dot");
			daInfo.associatedIndexInMDP = (automatonList.size()-1)-daNum; //check if this is it, possibly a better way to do this 
			daInfo.daLabel = "da"+daInfo.associatedIndexInMDP; 
			daInfo.originalLabel = nestedProductModel.getVarList().getName(0);
			internalAutomatonInfo.put(daInfo.associatedIndexInMDP,daInfo);
			long stopTime2 = System.currentTimeMillis();
			long runTime2 = stopTime2 - startTime;
			System.out.println("\nProduct with DA"+daNum+":" + runTime2 + "ms" + "(" + TimeUnit.SECONDS.convert(runTime2, TimeUnit.MILLISECONDS) + "s)\n");
			System.out.println( nestedProductModel.infoStringTable());

		}

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		System.out.println("\nCreated Nested Product: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
		//end profiling
		finalProduct = nestedProductModel; 
		mdpToProductModelStateMapping((MDP)model);
		//now lets just have a map for mdp states to product states 
		
	}
	
	void mdpToProductModelStateMapping(MDP originalModel)
	{
		
		//so we're only going to match the mdp states 
		//step 1 we get the varlist and exclude all variables with da or _da 
		VarList pvl = finalProduct.getVarList(); 
		VarList mvl = originalModel.getVarList(); 
		
		vlMapping = new HashMap<Integer,Integer>(); 
		for(int v = 0; v< pvl.getNumVars(); v++)
		{
			String name = pvl.getName(v); 
			if(mvl.exists(name))
			{
			int mdpv = mvl.getIndex(name); 
			vlMapping.put(v, mdpv); 
			}
		}
		System.out.println(vlMapping);
		
		//lets just keep this mapping cuz we'll need it. 
	}
	
	State createMDPStateFromProductState(int productStateNum)
	{
		State productState = finalProduct.getStatesList().get(productStateNum);
		State newState = new State(vlMapping.size()); 
		for(int pI : vlMapping.keySet())
		{
			int mI = vlMapping.get(pI); 
			newState.setValue(mI, productState.varValues[pI]);
		}
		
		return newState; 
	}

	public void setHomogenous(ArrayList<Model> models)
	{
		homogenous = true; 
		numTotalAgents = models.size(); 
		//go over each model and get the corresponding model states
		
		
	}
	
	

}
