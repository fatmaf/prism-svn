package demos;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Stack;
import java.util.Vector;
import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import common.IterableStateSet;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.Model;
import explicit.ModelCheckerPartialSatResult;
import parser.ast.Expression;
import parser.ast.ExpressionProb;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;
import parser.State;
import parser.VarList;

public class ContrastiveXAI
{

	Prism prism;
	PrismLog mainLog;
	MDPModelChecker mc;
	MDP mdp;
	DA<BitSet, ? extends AcceptanceOmega> da;
	BitSet accStates;
	BitSet sinkStates;
	int daVarInd = -1;
	PropertiesFile propertiesFile;
	ModulesFile modulesFile;

	//main function 
	public static void main(String[] args)
	{
		new ContrastiveXAI().run();
	}

//	public ArrayList<BitSet> setStateLabels(Vector<BitSet> labelBS, MDP mdp)
//	{
//		//initialise to get labels for actions 
//		int numAPs = da.getAPList().size();
//		BitSet s_labels = new BitSet(numAPs);
//		//associate a label with each state in the mdp 
//		ArrayList<BitSet> statelabels = new ArrayList<BitSet>();
//		for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
//			s_labels = new BitSet(numAPs);
//			// Get BitSet representing APs (labels) satisfied by state s_0
//			for (int k = 0; k < numAPs; k++) {
//				s_labels.set(k, labelBS.get(Integer.parseInt(da.getAPList().get(k).substring(1))).get(s_0));
//			}
//			// Find corresponding initial state in DA
//			int q_0 = da.getEdgeDestByLabel(da.getStartState(), s_labels);
//			mainLog.println(mdp.getStatesList().get(s_0) + " " + s_0 + "," + q_0 + ":" + s_labels.toString());
//			statelabels.add(s_labels);
//		}
//		return statelabels;
//	}
//
//	public HashMap<State, HashMap<Object, ArrayList<BitSet>>> getStateActionLabels(ArrayList<BitSet> statelabels, MDP mdp)
//	{
//		//attach a label to an action of the mdp 
//		//basically that of the next state 
//		HashMap<State, HashMap<Object, ArrayList<BitSet>>> actionLabels = new HashMap<State, HashMap<Object, ArrayList<BitSet>>>();
//		for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
//			//go over all these states again 
//			//assign labels to actions as those for the next state 
//			//but what about probabilistic ones ? 
//			int numChoices = mdp.getNumChoices(s_0);
//			State state = mdp.getStatesList().get(s_0);
//			if (!actionLabels.containsKey(state)) {
//				actionLabels.put(state, new HashMap<Object, ArrayList<BitSet>>());
//			}
//
//			for (int i = 0; i < numChoices; i++) {
//				Object action = mdp.getAction(s_0, i);
//				if (!actionLabels.get(state).containsKey(action))
//					actionLabels.get(state).put(action, new ArrayList<BitSet>());
//
//				Iterator<Entry<Integer, Double>> trIter = mdp.getTransitionsIterator(s_0, i);
//				while (trIter.hasNext()) {
//					Entry<Integer, Double> nextSP = trIter.next();
//					int ns = nextSP.getKey();
//					actionLabels.get(state).get(action).add(statelabels.get(ns));
//
//				}
//			}
//		}
//		return actionLabels;
//	}
//
//	//so only has state actions that lead to a sink state 
//	//we dont care about the label really 
//
//	public HashMap<State, ArrayList<Object>> getStateActionLabelsSinkStates(ArrayList<BitSet> statelabels, MDP mdp) throws PrismException
//	{
//		//attach a label to an action of the mdp 
//		//basically that of the next state 
//		HashMap<State, ArrayList<Object>> actionLabels = new HashMap<State, ArrayList<Object>>();
//		for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
//			//go over all these states again 
//			//assign labels to actions as those for the next state 
//			//but what about probabilistic ones ? 
//			int numChoices = mdp.getNumChoices(s_0);
//			State state = mdp.getStatesList().get(s_0);
//			int daState = getDAStateVar(mdp, state);
//
//			for (int i = 0; i < numChoices; i++) {
//				Object action = mdp.getAction(s_0, i);
//
//				Iterator<Entry<Integer, Double>> trIter = mdp.getTransitionsIterator(s_0, i);
//				while (trIter.hasNext()) {
//					Entry<Integer, Double> nextSP = trIter.next();
//					int ns = nextSP.getKey();
//					State nextState = mdp.getStatesList().get(ns);
//					int nextDaState = getDAStateVar(mdp, nextState);
//					if (sinkStates.get(nextDaState)) {
//						if (!actionLabels.containsKey(state)) {
//							actionLabels.put(state, new ArrayList<Object>());
//						}
//						if (!actionLabels.get(state).contains(action)) {
//							actionLabels.get(state).add(action);
//						}
//					}
//
//				}
//			}
//		}
//		return actionLabels;
//	}

	//
	public void readModel(String saveplace, String filename) throws FileNotFoundException, PrismException
	{
		mainLog = new PrismFileLog("stdout");

		// Initialise PRISM engine 
		prism = new Prism(mainLog);

		prism.initialise();
		prism.setEngine(Prism.EXPLICIT);

		//load the mdp for a single model 

		String modelFileName = saveplace + filename + "0.prism";
		modulesFile = prism.parseModelFile(new File(modelFileName));
		prism.loadPRISMModel(modulesFile);
		propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + "_rew.props"));
		prism.buildModel();
		mdp = (MDP) prism.getBuiltModelExplicit();

		//set up the model checker 
		mc = new MDPModelChecker(prism);
		mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));

	}

	public void setMCExportOptionsAll(String saveplace, String filename)
	{
		mc.setExportAdv(true);
		mc.setExportAdvFilename(saveplace + "results/xai_" + filename + ".dot");
		mc.setExportProductVector(true);
		mc.setExportProductVectorFilename(saveplace + "results/xai_" + filename + "_res.tra");
		mc.setGenStrat(true);
		mc.setExportProductStates(true);
		mc.setExportProductTrans(true);
		mc.setExportProductStatesFilename(saveplace + "results/xai_" + filename + ".sta");
		mc.setExportProductTransFilename(saveplace + "results/xai_" + filename + ".tra");
	}
//
//	public BitSet getAccStatesReachorRabin(DA<BitSet, ? extends AcceptanceOmega> da)
//	{
//		BitSet acc = null;
//		AcceptanceOmega acceptance = da.getAcceptance();
//		//Check if its a DFA
//		if (acceptance instanceof AcceptanceReach) {
//			acc = ((AcceptanceReach) acceptance).getGoalStates();
//		} else if (acceptance instanceof AcceptanceRabin) {
//			acc = da.getRabinAccStates();
//		}
//		return acc;
//	}
//
//	public BitSet getSinkStates(DA<BitSet, ? extends AcceptanceOmega> da, BitSet accStates)
//	{
//		BitSet sinkStates = new BitSet();
//		for (int i = 0; i < da.size(); i++) {
//			boolean isSinkState = false;
//			int numEdges = da.getNumEdges(i);
//			int numEdgesSrcToSrc = da.getNumEdges(i, i);
//			if (numEdges == numEdgesSrcToSrc) {
//				isSinkState = true;
//
//			}
//			if (numEdges == 0)
//				isSinkState = true;
//			if (isSinkState) {
//				if (accStates.get(i) == false)
//					sinkStates.set(i);
//			}
//		}
//		return sinkStates;
//	}
//
//	public HashMap<Integer, List<BitSet>> getStateLabelPairsLeadingToSinkStates(BitSet sinkStates)
//	{
//		HashMap<Integer, List<BitSet>> stateLabelPairsLeadingToSinkStates = new HashMap<Integer, List<BitSet>>();
//
//		//what if you had multiple sink states ?
//
//		//now get labels that make us transition to sink states 
//		int nextSinkState = sinkStates.nextSetBit(0);
//		while (nextSinkState != -1) {
//			for (int src = 0; src < da.size(); src++) {
//				if (!sinkStates.get(src)) {
//					int numEdges = da.getNumEdges(src, nextSinkState);
//					if (numEdges > 0) {
//						//all the states that lead here 
//						mainLog.println(numEdges + " between " + src + " - " + nextSinkState);
//
//						List<BitSet> edgeLabels = da.getEdgeLabels(src, nextSinkState);
//						mainLog.println(edgeLabels.toString());
//						if (!stateLabelPairsLeadingToSinkStates.containsKey(src)) {
//							stateLabelPairsLeadingToSinkStates.put(src, edgeLabels);
//						} else {
//							stateLabelPairsLeadingToSinkStates.get(src).addAll(edgeLabels);
//						}
//
//					}
//				}
//			}
//			nextSinkState = sinkStates.nextSetBit(nextSinkState + 1);
//		}
//		return stateLabelPairsLeadingToSinkStates;
//	}
//
//	public HashMap<Integer, List<BitSet>> convertPropertyToDA(Expression expr, Vector<BitSet> labelBS) throws PrismException
//	{
//		//convert property to da 
//		LTLModelChecker mcLTL = new LTLModelChecker(prism);
//		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
//
//		da = mcLTL.constructDAForLTLFormula(mc, mdp, expr, labelBS, allowedAcceptance);
//		mainLog.println(da.size());
//		//get sink states 
//		//for each state in the da 
//		//if it has no outward edges or only one that leads to itself its a deadend 
//		accStates = getAccStatesReachorRabin(da);
//		sinkStates = getSinkStates(da, accStates);
//		HashMap<Integer, List<BitSet>> stateLabelPairsLeadingToSinkStates = getStateLabelPairsLeadingToSinkStates(sinkStates);
//
//		mainLog.println("Acc States " + accStates.toString());
//		mainLog.println("Sink States " + sinkStates.toString());
//		return stateLabelPairsLeadingToSinkStates;
//	}

	int getDAStateVar(MDP mdp, State s) throws PrismException
	{
		//assumes only one da var here!!! 

		//find the variable which is the da 
		VarList vl = mdp.getVarList();
		if (daVarInd == -1) {
			for (int i = 0; i < vl.getNumVars(); i++) {
				String name = vl.getName(i);
				if (name.contains("da")) {
					daVarInd = i;
					break;
				}
			}
		}
		if (daVarInd != -1) {
			return (int) s.varValues[daVarInd];
		} else {
			throw new PrismException("no da variable!!!");
		}
	}

	int getStateIndex(MDP mdp, State s)
	{
		List<State> sl = mdp.getStatesList();
		int sI = -1;
		for (int i = 0; i < sl.size(); i++) {
			if (sl.get(i).compareTo(s) == 0) {
				sI = i;
				break;
			}
		}
		return sI;
	}

	State stringToState(String str)
	{
		//remove brackets 
		//split on , 
		String newStr = str.replace("(", "");
		newStr = newStr.replace(")", "");
		String[] strSplit = newStr.split(",");
		int stateSize = strSplit.length;
		State toret = new State(stateSize);
		for (int i = 0; i < stateSize; i++) {
			int sN = Integer.parseInt(strSplit[i]);
			toret.setValue(i, sN);
		}
		return toret;
	}

	int getActionIndex(int s, Object a, MDP mdp)
	{
		int aI = -1;
		int numChoices = mdp.getNumChoices(s);
		for (int i = 0; i < numChoices; i++) {
			Object ao = mdp.getAction(s, i);
			if (ao.toString().contentEquals(a.toString())) {
				aI = i;
				break;
			}
		}
		return aI;
	}

	
	public void doStuff() throws PrismException, FileNotFoundException
	{
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		String filename = "xai_r1_d1_g1_fs1notgoal_noback";//"g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";

		readModel(saveplace, filename);

		ExpressionReward exprRew = (ExpressionReward) propertiesFile.getProperty(0);
		mainLog.println(exprRew.toString());

		setMCExportOptionsAll(saveplace, filename);

		Expression expr = exprRew.getExpression();

		NVIExposed nvi = new NVIExposed();
		ModelCheckerPartialSatResult res = nvi.nviexposed(prism, mainLog, mdp, expr, exprRew, null, modulesFile, mc);
		PrismFileLog pfl = new PrismFileLog(saveplace + "results/" + "xai_" + filename + "_prodmpd.dot");
		nvi.productmdp.exportToDotFile(pfl, null, true);
		pfl.close();
		PathCreator pathO = new PathCreator(); 
		pathO.createPathPolicy(0,nvi.productmdp, res.strat, saveplace + "results/", "xai_" + filename + "_path" +0);
		
		
		//state1 action 2 the door action 
		PathCreator pathC = new PathCreator(); 
		ArrayList<Integer> prefixStates = new ArrayList<Integer>(); 
		ArrayList<Integer> prefixActions = new ArrayList<Integer>(); 
		prefixStates.add(0); prefixActions.add(0); 
		prefixStates.add(1); prefixActions.add(1); 
 		pathC.creatPath(prefixStates, prefixActions, nvi.productmdp, res.strat, saveplace + "results/", "xai_" + filename + "_path" +prefixStates.toString() );
		
 		//now lets compare these paths 
 		//we can go home if we do this. 
 	
// 		printBestStateValuesAlongPath
 		bestStatePathDFS	(pathO,nvi);
// 		printBestStateValuesAlongPath
 		bestStatePathDFS	(pathC,nvi);
 		//so we go over each path 
 		//and identify the states that had the most influence 
 		//where influence is just weights 

 	

	}

	public void bestStatePathDFS(PathCreator pathC, NVIExposed nvi)
	{
		//lets start with path 0 
 		//start with the initial state 
 		//for a path with no forks this is easy 
 		//lets see what happens 
		mainLog.println("********************** Path *******************");
 		MDPSimple mdp = pathC.pc.mdpCreator.mdp; 
 		int startState = mdp.getFirstInitialState(); 
 		Stack<Entry<Entry<Integer,Integer>,Double>> stateStackProb = new Stack<Entry<Entry<Integer,Integer>,Double>>();
 		Stack<Entry<Entry<Integer,Integer>,Double>> stateStackProg = new Stack<Entry<Entry<Integer,Integer>,Double>>();
 		Stack<Entry<Entry<Integer,Integer>,Double>> stateStackCost = new Stack<Entry<Entry<Integer,Integer>,Double>>();
 	
 		ArrayList<Integer> seen = new ArrayList<Integer>(); 
 		bspdfs(startState,mdp,nvi.productmdp,nvi,stateStackProb,stateStackProg,stateStackCost,0,true,seen);
		
		
	}
	
	public void bspdfs(int s, MDPSimple mdp, MDP productmdp,NVIExposed nvi,
			Stack<Entry<Entry<Integer,Integer>,Double>> stateStackProb,	
			Stack<Entry<Entry<Integer,Integer>,Double>>stateStackProg,
			Stack<Entry<Entry<Integer,Integer>,Double>> stateStackCost, 
			double parentCost,boolean startState, ArrayList<Integer> seen)
	{
		State sState = mdp.getStatesList().get(s);
		int sInMDP = getStateIndex(nvi.productmdp, sState); 
		if(!seen.contains(s)) {
			seen.add(s);
			//now we follow this path 
			//all the way to the end 
			int numChoices = mdp.getNumChoices(s); 
			for(int i = 0; i<numChoices; i++)
			{
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, i); 
				Object a = mdp.getAction(s, i); 
				int aInMDP = getActionIndex(sInMDP,a,productmdp); 
				Entry<Integer,Integer> saPair = new AbstractMap.SimpleEntry<Integer,Integer>(sInMDP,aInMDP); 
				double val = nvi.stateActionProbValues.get(sInMDP).get(aInMDP);
				Entry<Entry<Integer,Integer>,Double> sav = new AbstractMap.SimpleEntry<Entry<Integer,Integer>,Double>(saPair,val);
				String mls = "";
				mls+= sav.toString()+",";
				stateStackProb.push(sav);
				val = nvi.stateActionProgValues.get(sInMDP).get(aInMDP);
				sav = new AbstractMap.SimpleEntry<Entry<Integer,Integer>,Double>(saPair,val);
				
				mls+= sav.toString()+",";
				stateStackProg.push(sav);
				val = nvi.stateActionCostValues.get(sInMDP).get(aInMDP);
				sav = new AbstractMap.SimpleEntry<Entry<Integer,Integer>,Double>(saPair,val);
				mls+=sav.toString();
				stateStackCost.push(sav);
				
				if(!startState)
					mls+= " cost diff = "+(parentCost-val);
				mainLog.println(mls);
				while(tranIter.hasNext())
				{
					Entry<Integer, Double> next = tranIter.next(); 
					int nextState = next.getKey(); 
					bspdfs(nextState,mdp,productmdp,nvi,stateStackProb,stateStackProg,stateStackCost,val,false,seen);
					
		
					
					
				}
			}
		}	}
	public void printBestStateValuesAlongPath(PathCreator pathC, NVIExposed nvi)
	{
 	
 		//lets start with path 0 
 		//start with the initial state 
 		//for a path with no forks this is easy 
 		//lets see what happens 
		mainLog.println("********************** Path *******************");
 		MDPSimple mdp = pathC.pc.mdpCreator.mdp; 
 		int startState = mdp.getFirstInitialState(); 
 		Queue<Integer> statesQ = new LinkedList<Integer>(); 
 		statesQ.add(startState); 
 		Stack<Entry<Entry<Integer,Integer>,Double>> stateStackProb = new Stack<Entry<Entry<Integer,Integer>,Double>>();
 		Stack<Entry<Entry<Integer,Integer>,Double>> stateStackProg = new Stack<Entry<Entry<Integer,Integer>,Double>>();
 		Stack<Entry<Entry<Integer,Integer>,Double>> stateStackCost = new Stack<Entry<Entry<Integer,Integer>,Double>>();
 		while(!statesQ.isEmpty())
 		{
 			int s = statesQ.remove();
 			State sState = mdp.getStatesList().get(s);
			int sInMDP = getStateIndex(nvi.productmdp, sState); 
 			//now we follow this path 
 			//all the way to the end 
 			int numChoices = mdp.getNumChoices(s); 
 			for(int i = 0; i<numChoices; i++)
 			{
 				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, i); 
 				Object a = mdp.getAction(s, i); 
 				int aInMDP = getActionIndex(sInMDP,a,nvi.productmdp); 
 				Entry<Integer,Integer> saPair = new AbstractMap.SimpleEntry<Integer,Integer>(sInMDP,aInMDP); 
 				double val = nvi.stateActionProbValues.get(sInMDP).get(aInMDP);
 				Entry<Entry<Integer,Integer>,Double> sav = new AbstractMap.SimpleEntry<Entry<Integer,Integer>,Double>(saPair,val);
 				String mls = "";
 				mls+= sav.toString()+",";
 				stateStackProb.push(sav);
 				val = nvi.stateActionProgValues.get(sInMDP).get(aInMDP);
 				sav = new AbstractMap.SimpleEntry<Entry<Integer,Integer>,Double>(saPair,val);
 				
 				mls+= sav.toString()+",";
 				stateStackProg.push(sav);
 				val = nvi.stateActionCostValues.get(sInMDP).get(aInMDP);
 				sav = new AbstractMap.SimpleEntry<Entry<Integer,Integer>,Double>(saPair,val);
 				mainLog.println(mls+sav.toString());
 				stateStackCost.push(sav);
 				while(tranIter.hasNext())
 				{
 					Entry<Integer, Double> next = tranIter.next(); 
 					int nextState = next.getKey(); 
 					statesQ.add(nextState);
 		
 					
 					
 				}
 			}
 			
 		}
	}
	public void openDotFile(String location, String fn) throws IOException
	{
		//		String comm = "dot -Tsvg " + location + fn + "-o " + location + fn + ".dot";
		//		comm += " & ";
		//		comm += "google-chrome-stable " + location + fn + ".dot";
		//		runCommand(comm);
		Runtime.getRuntime().exec(new String[] { "bash", "-c", "dot -Tsvg " + location + fn + "-o " + location + fn + ".svg" });
		Runtime.getRuntime().exec(new String[] { "bash", "-c", "google-chrome " + location + fn + ".svg &" });
	}

	//from the internet somewhere 
	public void runCommand(String command)
	{
		try {
			Runtime rt = Runtime.getRuntime();
			//Process pr = rt.exec("cmd /c dir");
			Process pr = rt.exec(command);

			BufferedReader input = new BufferedReader(new InputStreamReader(pr.getInputStream()));

			String line = null;

			while ((line = input.readLine()) != null) {
				System.out.println(line);
			}

			int exitVal = pr.waitFor();
			System.out.println("Exited with error code " + exitVal);

		} catch (Exception e) {
			System.out.println(e.toString());
			e.printStackTrace();
		}
	}

	//run function 

	public void run()
	{

		try {

			doStuff();

		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
