package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import acceptance.AcceptanceOmega;
import automata.DA;
import demos.XAIPolicyPath.PathNode;
import demos.XAIStateInformation.ValueLabel;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.ModelCheckerPartialSatResult;
import parser.ast.Expression;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import simulator.ModulesFileModelGenerator;
import strat.Strategy;
import parser.State;
import parser.VarList;

public class XAIStateAnalyser
{

	Prism prism;
	PrismLog mainLog;
	MDPModelChecker mc;
	MDP mdp;
	DA<BitSet, ? extends AcceptanceOmega> da;
	BitSet accStates;
	BitSet dasinkStates;
	int daVarInd = -1;
	PropertiesFile propertiesFile;
	ModulesFile modulesFile;
	ArrayList<String> nonMDPDistVars;
	String currentSaveplace = "";
	String currentFilename = "";
	HashMap<String,Double> variableWeights ;
	//main function 
	public static void main(String[] args)
	{
		new XAIStateAnalyser().run();
	}

	//
	public void readModel(String saveplace, String filename, boolean noappend) throws FileNotFoundException, PrismException
	{
		mainLog = new PrismFileLog("stdout");

		// Initialise PRISM engine 
		prism = new Prism(mainLog);

		prism.initialise();
		prism.setEngine(Prism.EXPLICIT);

		//load the mdp for a single model 

		currentSaveplace = saveplace;
		currentFilename = filename;
		String modelFileName = saveplace + filename;
		if (!noappend)
			modelFileName += "0.prism";
		else
			modelFileName += ".prism";
		modulesFile = prism.parseModelFile(new File(modelFileName));
		prism.loadPRISMModel(modulesFile);
		String propFileName = saveplace + filename;
		if (!noappend)
			propFileName += "_rew.props";
		else
			propFileName += ".props";
		propertiesFile = prism.parsePropertiesFile(modulesFile, new File(propFileName));
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

	int getDAStateVar(MDP mdp, State s) throws PrismException
	{
		//assumes only one da var here!!! 

		//find the variable which is the da 
		VarList vl = mdp.getVarList();
		setDAVarInd(vl);
		if (daVarInd != -1) {
			return (int) s.varValues[daVarInd];
		} else {
			throw new PrismException("no da variable!!!");
		}
	}

	public void setDAVarInd(VarList vl)
	{

		if (daVarInd == -1) {
			for (int i = 0; i < vl.getNumVars(); i++) {
				String name = vl.getName(i);
				if (name.contains("da")) {
					daVarInd = i;
					break;
				}
			}
		}
	}

	public int getDAVarInd(VarList vl)
	{
		int daVarIndHere = -1;

		for (int i = 0; i < vl.getNumVars(); i++) {
			String name = vl.getName(i);
			if (name.contains("da")) {
				daVarIndHere = i;
				break;
			}
		}
		return daVarIndHere;
	}

	State getMDPState(VarList vl, State s)
	{
		//		if (daVarInd == -1)
		int daVarIndHere = getDAVarInd(vl);
		if (daVarIndHere == -1)
			return s;
		State toret = new State(vl.getNumVars() - 1);
		int j = 0;
		for (int i = 0; i < vl.getNumVars(); i++) {
			if (i != daVarIndHere) {
				toret.setValue(j, s.varValues[i]);
				j++;
			}
		}
		return toret;
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
		ArrayList<String> actionStrings = new ArrayList<String>();
		int aI = -1;
		if (a != null) {
			int numChoices = mdp.getNumChoices(s);
			for (int i = 0; i < numChoices; i++) {
				Object ao = mdp.getAction(s, i);
				if (ao != null)
					actionStrings.add(ao.toString());
				else
					actionStrings.add("null");
				if (ao.toString().contentEquals(a.toString())) {
					aI = i;
					break;
				}
			}
		}
		if (aI == -1)
			System.out.println(actionStrings.toString());
		return aI;
	}

	public void listClosestStatesToState(XAIStateInformation ssTop, HashMap<State, ArrayList<Entry<State, Double>>> optimalPolicyStateDistances,
			XAIPathCreator optimalPolicyPaths)
	{
		System.out.println("Listing distances from " + ssTop.toString());
		//for these two we want the closest states 
		int numStatesListed = 0;
		int maxStatesToList = 10;
		double threshold = Math.sqrt((double) ssTop.getState().varValues.length);
		if (optimalPolicyStateDistances.containsKey(ssTop.getState())) {
			ArrayList<Entry<State, Double>> ssTopDistances = optimalPolicyStateDistances.get(ssTop.getState());
			for (Entry<State, Double> e : ssTopDistances) {
				State s = e.getKey();
				String aStr = "";
				if (optimalPolicyPaths != null) {

					Object a = optimalPolicyPaths.getStateAction(s);

					if (a != null)
						aStr = a.toString();
				}
				if (Math.abs(e.getValue()) > threshold) {

					if (numStatesListed >= maxStatesToList)
						break;
					numStatesListed++;
				}

				System.out.println(e + ":" + aStr);

			}

		} else
			System.out.println("State " + ssTop.getState() + " does not exist in distance structure");

	}

	public void listClosestStatesToStateOnTheFly(XAIStateInformation ssTop, VarList otherVL, HashMap<State, ArrayList<Entry<State, Double>>> optimalPolicyStateDistances,
			XAIPathCreator optimalPolicyPaths, HashMap<State, HashMap<State, Double>> dijkstraDists, MDP originalMDP, MDP otherMDP,
			ModelCheckerPartialSatResult nvires) throws Exception
	{
		System.out.println("Listing distances from " + ssTop.toString());
		//for these two we want the closest states 

		double threshold = Math.sqrt((double) ssTop.getState().varValues.length);
		System.out.println("Threshold " + threshold);
		int numStatesListed = 0;
		int maxStatesToList = 10;
		if (optimalPolicyStateDistances.containsKey(ssTop.getState())) {
			ArrayList<Entry<State, Double>> ssTopDistances = optimalPolicyStateDistances.get(ssTop.getState());
			for (Entry<State, Double> e : ssTopDistances) {
				State s = e.getKey();
				String aStr = "";
				if (optimalPolicyPaths != null) {

					Object a = optimalPolicyPaths.getStateAction(s);

					if (a != null)
						aStr = a.toString();
				}
				if (Math.abs(e.getValue()) > threshold) {
					if (numStatesListed >= maxStatesToList)
						break;
					numStatesListed++;
				}
				System.out.println(e + ":" + aStr);

			}

		} else {
			VarList vl = optimalPolicyPaths.pc.mdpCreator.mdp.getVarList();

			if(otherVL == null)
				otherVL = vl;
			State istate = ssTop.getState();
			State mdpState = this.getMDPState(vl, istate);
			if (dijkstraDists == null || !dijkstraDists.containsKey(mdpState)) {

				int oms = this.getStateIndex(originalMDP, mdpState);
				if (oms == -1) {
					//so we cant find this 
					//we need to find something close to this state in the mdp 
					//close how ? 
					//close in terms of distance 
					calculateSingleStateDistances(mdpState, dijkstraDists, originalMDP.getStatesList(), otherVL,originalMDP.getVarList(), originalMDP.getNumStates());
					optimalPolicyPaths.pc.mdpCreator.saveMDPstatra(originalMDP, currentSaveplace + "results/", "xai_" + currentFilename + "_orignialmdp", null);
					optimalPolicyPaths.pc.mdpCreator.saveMDP(originalMDP, currentSaveplace + "results/", "xai_" + currentFilename + "_orignialmdp", null);
					System.out.println("bug here");
					throw new Exception("no such state in orignial mdp: " + mdpState.toString());
				}
				dijkstraDists = mdpDijkstraUpdate(oms, originalMDP, dijkstraDists);

			}
			ArrayList<Entry<State, Double>> pq;
			if (otherMDP == null)
				pq = this.calculateSingleStateDistances(istate, dijkstraDists, optimalPolicyPaths.pc.mdpCreator.mdp.getStatesList(), vl,vl,
						optimalPolicyPaths.pc.mdpCreator.mdp.getNumStates());
			else
				pq = this.calculateSingleStateDistances(istate, dijkstraDists, otherMDP.getStatesList(), vl,vl, otherMDP.getNumStates());
			optimalPolicyStateDistances.put(istate, pq);
			ArrayList<Entry<State, Double>> ssTopDistances = optimalPolicyStateDistances.get(ssTop.getState());
			for (Entry<State, Double> e : ssTopDistances) {
				State s = e.getKey();
				String aStr = "";
				if (optimalPolicyPaths != null) {

					Object a = optimalPolicyPaths.getStateAction(s);

					if (a != null)
						aStr = a.toString();
					else {
						if (nvires != null) {
							Strategy strat = nvires.strat;
							int si = this.getStateIndex(otherMDP, s);
							strat.initialise(si);
							a = strat.getChoiceAction();
							if (a != null)
								aStr = a.toString();
						}
					}
				}
				if (Math.abs(e.getValue()) > threshold) {
					if (numStatesListed >= maxStatesToList)
						break;
					numStatesListed++;
				}

				System.out.println(e + ":" + aStr);

			}

		}
	}

	public XAIPathCreator createAlternatePath(String filename, XAIvi vi, ModelCheckerPartialSatResult optimalPolicy, String saveplace) throws PrismException
	{
		XAIPathCreator alternativePolicyPath = new XAIPathCreator();
		ArrayList<Integer> prefixStates = new ArrayList<Integer>();
		ArrayList<Integer> prefixActions = new ArrayList<Integer>();
		if (filename.contains("key")) {
			prefixStates.add(0);
			prefixActions.add(0);
			prefixStates.add(2);
			prefixActions.add(3);
			prefixStates.add(5);
			prefixActions.add(3);

		} else {
			prefixStates.add(0);
			prefixActions.add(2);
			prefixStates.add(3);
			prefixActions.add(6);
			prefixStates.add(4);
			prefixActions.add(2);
		}
		alternativePolicyPath.creatPathFlex(prefixStates, prefixActions, vi.productmdp, optimalPolicy.strat, saveplace + "results/",
				"xai_" + filename + "alternatePath" + prefixStates.toString(), vi.progRewards, vi.costRewards, vi.accStates);
		return alternativePolicyPath;
	}

	public XAIPathCreator createAlternatePath(XAIPathCreator pathToFollow, String filename, XAIvi vi, ModelCheckerPartialSatResult optimalPolicy,
			String saveplace) throws PrismException
	{
		XAIPathCreator alternativePolicyPath = new XAIPathCreator();
		ArrayList<Integer> prefixStates = new ArrayList<Integer>();
		ArrayList<Integer> prefixActions = new ArrayList<Integer>();
		MDP pfmdp = pathToFollow.pc.mdpCreator.mdp;
		int initialState = pfmdp.getFirstInitialState();
		Queue<Integer> statesQ = new LinkedList<Integer>();
		BitSet seenStates = new BitSet();
		statesQ.add(initialState);
		while (!statesQ.isEmpty()) {
			int i = statesQ.remove();
			if (!seenStates.get(i)) {
				seenStates.set(i);
				State s = pfmdp.getStatesList().get(i);
				int sI = this.getStateIndex(vi.productmdp, s);
				Object a = pfmdp.getAction(i, 0);
				if (a != null) {
					int aI = this.getActionIndex(sI, a, vi.productmdp);
					if (aI != -1) {
						prefixStates.add(sI);
						prefixActions.add(aI);
					} else {
						System.out.println("No action " + a.toString() + " for " + s.toString());
					}
				}
				Iterator<Entry<Integer, Double>> tranIter = pfmdp.getTransitionsIterator(i, 0);
				while (tranIter.hasNext()) {
					Entry<Integer, Double> sp = tranIter.next();
					statesQ.add(sp.getKey());
				}

			}
		}
		alternativePolicyPath.creatPathFlex(prefixStates, prefixActions, vi.productmdp, optimalPolicy.strat, saveplace + "results/",
				"xai_" + filename + "alternatePath" + prefixStates.toString(), vi.progRewards, vi.costRewards, vi.accStates);
		return alternativePolicyPath;
	}

	public ArrayList<XAIStateInformation> getSwingStatesListByCost(XAIvi vi, double discount, XAIPathCreator policyPaths) throws PrismException
	{
		ModelCheckerPartialSatResult policyVI = vi.doPathNVI(prism, mainLog, policyPaths, mc, discount);
		ModelCheckerPartialSatResult policyOccFreq = vi.doPathOccupancyFreq(prism, mainLog, policyPaths, mc, discount);

		XAIPolicyPath policyPathTree = traversePath(policyPaths, policyVI, policyOccFreq);
		doActionValuesDifference(policyPathTree);
		ArrayList<XAIStateInformation> mostInfluentialStatesQ = getSwingStatesListByCost(policyPathTree);
		return mostInfluentialStatesQ;
	}

	public void doStuff() throws Exception
	{
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/xaiTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		String filename = "xai_r1_d1_g1_key";//"xai_r1_d1_g1_key";//_fs1notgoal_noback";//"g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";

		HashMap<String, Double> discountVals = new HashMap<String, Double>();
		discountVals.put("xai_r1_d1_g1", 1.0);
		discountVals.put("xai_r1_d1_g1_key", 0.9);

		double discount = discountVals.get(filename);
		readModel(saveplace, filename, false);

		//just testing the distance stuff 
		//		double[] dists = mdpDijkstraSingleState(mdp, mdp.getFirstInitialState());
		//		System.out.println(Arrays.toString(dists));
		HashMap<State, HashMap<State, Double>> mdpDistshm = mdpDijkstra(mdp);
		ExpressionReward exprRew = (ExpressionReward) propertiesFile.getProperty(0);
		mainLog.println(exprRew.toString());

		setMCExportOptionsAll(saveplace, filename);

		Expression expr = exprRew.getExpression();

		XAIvi nvi = new XAIvi();
		ModelCheckerPartialSatResult res = nvi.nviexposed(prism, mainLog, mdp, expr, exprRew, null, modulesFile, mc, discount);
		PrismFileLog pfl = new PrismFileLog(saveplace + "results/" + "xai_" + filename + "_prodmpd.dot");
		nvi.productmdp.exportToDotFile(pfl, null, true);
		pfl.close();
		XAIPathCreator pathO = new XAIPathCreator();
		pathO.createPathPolicy(0, nvi.productmdp, res.strat, saveplace + "results/", "xai_" + filename + "_path" + 0, nvi.progRewards, nvi.costRewards,
				nvi.accStates);
		HashMap<State, ArrayList<Entry<State, Double>>> pathOdists = calculateStateDistances(pathO, mdpDistshm);
		ModelCheckerPartialSatResult pathONVI = nvi.doPathNVI(prism, mainLog, pathO, mc, discount);
		ModelCheckerPartialSatResult pathOoccfreq = nvi.doPathOccupancyFreq(prism, mainLog, pathO, mc, discount);
		//state1 action 2 the door action 
		XAIPathCreator pathC = new XAIPathCreator();
		ArrayList<Integer> prefixStates = new ArrayList<Integer>();
		ArrayList<Integer> prefixActions = new ArrayList<Integer>();
		if (filename.contains("key")) {
			prefixStates.add(1);
			prefixActions.add(0);
			prefixStates.add(2);
			prefixActions.add(3);
			prefixStates.add(5);
			prefixActions.add(3);

		} else {
			prefixStates.add(0);
			prefixActions.add(2);
			prefixStates.add(3);
			prefixActions.add(6);
			prefixStates.add(4);
			prefixActions.add(2);
		}
		pathC.creatPath(prefixStates, prefixActions, nvi.productmdp, res.strat, saveplace + "results/", "xai_" + filename + "_path" + prefixStates.toString(),
				nvi.progRewards, nvi.costRewards, nvi.accStates);
		ModelCheckerPartialSatResult pathCNVI = nvi.doPathNVI(prism, mainLog, pathC, mc, discount);
		ModelCheckerPartialSatResult pathCoccfreq = nvi.doPathOccupancyFreq(prism, mainLog, pathC, mc, discount);
		XAIPolicyPath pathCPath = traversePath(pathC, pathCNVI, pathCoccfreq);
		doActionValuesDifference(pathCPath);

		printMostInfluentialStates(pathCPath);

		XAIPolicyPath pathOPath = traversePath(pathO, pathONVI, pathOoccfreq);
		doActionValuesDifference(pathOPath);
		printMostInfluentialStates(pathOPath);
		//now lets compare these paths 
		//we can go home if we do this. 

	}

	public HashMap<State, ArrayList<Entry<State, Double>>> calculateStateDistances(XAIPathCreator pathC, HashMap<State, HashMap<State, Double>> dijkstraDists)
			throws Exception
	{
		MDP pmdp = pathC.pc.mdpCreator.mdp;
		HashMap<State, ArrayList<Entry<State, Double>>> toret = calculateStateDistances(pmdp, dijkstraDists);

		return toret;
	}

	public HashMap<State, ArrayList<Entry<State, Double>>> calculateStateDistancesOnTheFly(XAIPathCreator pathC,
			HashMap<State, HashMap<State, Double>> dijkstraDists, MDP originalMDP) throws Exception
	{
		MDP pmdp = pathC.pc.mdpCreator.mdp;
		HashMap<State, ArrayList<Entry<State, Double>>> toret = calculateStateDistancesOnTheFly(pmdp, dijkstraDists, originalMDP);
		return toret;
	}

	public HashMap<State, ArrayList<Entry<State, Double>>> calculateStateDistancesOnTheFly(MDP m, HashMap<State, HashMap<State, Double>> dijkstraDists,
			MDP originalMDP) throws Exception
	{

		int numStates = m.getNumStates();
		List<State> sl = m.getStatesList();
		VarList vl = m.getVarList();
		HashMap<State, ArrayList<Entry<State, Double>>> toret = new HashMap<State, ArrayList<Entry<State, Double>>>();
		for (int i = 0; i < numStates; i++) {
			State istate = sl.get(i);
			State mdpState = this.getMDPState(vl, istate);
			if (dijkstraDists == null || !dijkstraDists.containsKey(mdpState)) {

				int oms = this.getStateIndex(originalMDP, mdpState);
				dijkstraDists = mdpDijkstraUpdate(oms, originalMDP, dijkstraDists);
			}
			ArrayList<Entry<State, Double>> pq = this.calculateSingleStateDistances(istate, dijkstraDists, sl,vl, vl, numStates);
			toret.put(istate, pq);
		}
		return toret;
	}

	public ArrayList<Entry<State, Double>> calculateSingleStateDistances(State istate, HashMap<State, HashMap<State, Double>> dijkstraDists, List<State> sl,
			VarList iStatevl,VarList oStatevl, int numStates) throws Exception
	{
		ArrayList<Entry<State, Double>> pq = new ArrayList<Entry<State, Double>>();
		for (int j = 0; j < numStates; j++) {
			State jstate = sl.get(j);
			double dist = stateDist(istate, jstate, iStatevl,oStatevl, dijkstraDists);
			Entry<State, Double> e = new AbstractMap.SimpleEntry<State, Double>(jstate, dist);
			pq.add(e);
		}
		Collections.sort(pq, (a, b) -> (a.getValue().compareTo(b.getValue())));
		return pq;
	}

	public HashMap<State, ArrayList<Entry<State, Double>>> calculateStateDistancesOnTheFly(XAIPathCreator path, MDP m,
			HashMap<State, HashMap<State, Double>> dijkstraDists, MDP originalMDP) throws Exception
	{
		MDP pathMDP = path.pc.mdpCreator.mdp;
		int numStates = m.getNumStates();
		List<State> sl = m.getStatesList();
		VarList vl = m.getVarList();
		HashMap<State, ArrayList<Entry<State, Double>>> toret = new HashMap<State, ArrayList<Entry<State, Double>>>();
		for (int i = 0; i < pathMDP.getNumStates(); i++) {

			int mdpSI = path.pc.policyStateToOriginalMDPMap.get(i);
			State istate = sl.get(mdpSI);

			State mdpState = this.getMDPState(vl, istate);
			if (dijkstraDists == null || !dijkstraDists.containsKey(mdpState)) {

				int oms = this.getStateIndex(originalMDP, mdpState);
				dijkstraDists = mdpDijkstraUpdate(oms, originalMDP, dijkstraDists);
			}
			ArrayList<Entry<State, Double>> pq = this.calculateSingleStateDistances(istate, dijkstraDists, sl,vl, vl, numStates);
			toret.put(istate, pq);
		}
		return toret;
	}

	public HashMap<State, ArrayList<Entry<State, Double>>> calculateStateDistances(XAIPathCreator pathC, MDP m,
			HashMap<State, HashMap<State, Double>> dijkstraDists) throws Exception
	{

		MDP pathMDP = pathC.pc.mdpCreator.mdp;

		int numStates = m.getNumStates();
		List<State> sl = m.getStatesList();
		VarList vl = m.getVarList();
		HashMap<State, ArrayList<Entry<State, Double>>> toret = new HashMap<State, ArrayList<Entry<State, Double>>>();
		for (int i = 0; i < pathMDP.getNumStates(); i++) {

			int mdpSI = pathC.pc.policyStateToOriginalMDPMap.get(i);
			State istate = sl.get(mdpSI);
			ArrayList<Entry<State, Double>> pq = this.calculateSingleStateDistances(istate, dijkstraDists, sl, vl,vl, numStates);
			toret.put(istate, pq);
		}
		return toret;
	}

	public HashMap<State, ArrayList<Entry<State, Double>>> calculateStateDistances(MDP m, HashMap<State, HashMap<State, Double>> dijkstraDists) throws Exception
	{

		int numStates = m.getNumStates();
		List<State> sl = m.getStatesList();
		VarList vl = m.getVarList();
		HashMap<State, ArrayList<Entry<State, Double>>> toret = new HashMap<State, ArrayList<Entry<State, Double>>>();
		for (int i = 0; i < numStates; i++) {
			State istate = sl.get(i);
			ArrayList<Entry<State, Double>> pq = this.calculateSingleStateDistances(istate, dijkstraDists, sl, vl,vl, numStates);
			toret.put(istate, pq);
		}
		return toret;
	}

	public XAIPolicyPath traversePath(XAIPathCreator pathC, ModelCheckerPartialSatResult pathCNVI, ModelCheckerPartialSatResult pathCoccfreq)
	{
		//just traverse the path 
		//we're making a tree really 
		//its a linked list 
		//so I should have a path data structure 
		//uff toba I do this all the time 
		//but this is different cuz i'm creating a tree. 
		MDPSimple mdp = pathC.pc.mdpCreator.mdp;
		int startState = mdp.getFirstInitialState();
		Queue<Integer> stateQ = new LinkedList<Integer>();
		stateQ.add(startState);
		BitSet seen = new BitSet();
		XAIPolicyPath path = null;
		while (!stateQ.isEmpty()) {
			int currentState = stateQ.remove();

			if (!seen.get(currentState)) {
				Object action = null;
				seen.set(currentState);
				//we have the values for this state 
				HashMap<ValueLabel, Double> stateValues = new HashMap<ValueLabel, Double>();
				stateValues.put(ValueLabel.frequency, pathCoccfreq.solnProb[currentState]);
				stateValues.put(ValueLabel.probability, pathCNVI.solnProb[currentState]);
				stateValues.put(ValueLabel.progression, pathCNVI.solnProg[currentState]);
				stateValues.put(ValueLabel.cost, pathCNVI.solnCost[currentState]);
				//now we create a new state 
				//also we know that each state only has one action here
				State currentStateState = mdp.getStatesList().get(currentState);
				if (mdp.getNumChoices(currentState) > 0) {
					action = mdp.getAction(currentState, 0);

					XAIStateInformation s = new XAIStateInformation(currentStateState, currentState, action);
					s.addValuesForState(action, stateValues);

					if (path == null) {
						path = new XAIPolicyPath(s);

					}

					//update the state 
					s = path.addToStatesList(s).root;
					//					System.out.println(s);
					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(currentState, 0);
					while (tranIter.hasNext()) {
						Entry<Integer, Double> spPair = tranIter.next();
						int nextS = spPair.getKey();
						double prob = spPair.getValue();
						State nextState = mdp.getStatesList().get(nextS);
						XAIStateInformation nextSI = new XAIStateInformation(nextState, nextS, null);
						nextSI.addParent(s);
						path.addChild(s, nextSI, prob);
						stateQ.add(nextS);
					}

				}
				if (action == null) {
					stateValues = new HashMap<ValueLabel, Double>();
					stateValues.put(ValueLabel.frequency, pathCoccfreq.solnProb[currentState]);
					stateValues.put(ValueLabel.probability, pathCNVI.solnProb[currentState]);
					stateValues.put(ValueLabel.progression, pathCNVI.solnProg[currentState]);
					stateValues.put(ValueLabel.cost, pathCNVI.solnCost[currentState]);
					XAIStateInformation s = new XAIStateInformation(currentStateState, currentState, action);
					s.addValuesForState(action, stateValues);
					path.addToStatesList(s);
				}

			}
		}
		return path;

	}

	public void printMostInfluentialStates(XAIPolicyPath p)
	{
		HashMap<ValueLabel, Double> minVals = new HashMap<ValueLabel, Double>();
		minVals.put(ValueLabel.probability, 1.0);
		minVals.put(ValueLabel.cost, 1000000.0);
		minVals.put(ValueLabel.frequency, 1000000.0);
		minVals.put(ValueLabel.progression, 10000.0);
		HashMap<ValueLabel, Double> maxVals = new HashMap<ValueLabel, Double>();
		maxVals.put(ValueLabel.probability, 0.0);
		maxVals.put(ValueLabel.cost, 0.0);
		maxVals.put(ValueLabel.frequency, 0.0);
		maxVals.put(ValueLabel.progression, 0.0);
		HashMap<ValueLabel, XAIStateInformation> maxPNs = new HashMap<ValueLabel, XAIStateInformation>();
		HashMap<ValueLabel, XAIStateInformation> minPNs = new HashMap<ValueLabel, XAIStateInformation>();

		List<State> statesSeen = new ArrayList<State>();
		PathNode pn = p.root;
		Stack<PathNode> pQ = new Stack<PathNode>();
		if (pn.children != null) {
			for (PathNode pnc : pn.children.keySet()) {
				pQ.push(pnc);
			}
		}

		while (!pQ.isEmpty()) {
			pn = pQ.pop();
			if (!statesSeen.contains(pn.root.getState()))
				statesSeen.add(pn.root.getState());
			else
				continue;
			HashMap<ValueLabel, Double> pnVals = pn.root.actionValuesDifference.get(pn.root.getChosenAction());
			for (ValueLabel vl : pnVals.keySet()) {
				if (pnVals.get(vl) < minVals.get(vl)) {
					minPNs.put(vl, pn.root);
					minVals.put(vl, pnVals.get(vl));
				}
				if (pnVals.get(vl) > maxVals.get(vl)) {
					maxPNs.put(vl, pn.root);
					maxVals.put(vl, pnVals.get(vl));
				}
			}
			if (pn.children != null) {
				for (PathNode pnc : pn.children.keySet()) {
					if (pnc != pn)
						pQ.push(pnc);
				}
			}
		}
		statesSeen = new ArrayList<State>();
		pn = p.root;
		pQ = new Stack<PathNode>();
		if (pn.children != null) {
			for (PathNode pnc : pn.children.keySet()) {
				pQ.push(pnc);
			}
		}
		HashMap<ValueLabel, ArrayList<XAIStateInformation>> minPNsAll = new HashMap<ValueLabel, ArrayList<XAIStateInformation>>();
		HashMap<ValueLabel, ArrayList<XAIStateInformation>> maxPNsAll = new HashMap<ValueLabel, ArrayList<XAIStateInformation>>();

		PriorityQueue<XAIStateInformation> siRankedbycost = new PriorityQueue<XAIStateInformation>(new XAIStateInformationRelativeCostComparator());
		double rangeRadius = 0.0001;
		while (!pQ.isEmpty()) {
			pn = pQ.pop();
			if (!statesSeen.contains(pn.root.getState()))
				statesSeen.add(pn.root.getState());
			else
				continue;
			HashMap<ValueLabel, Double> pnVals = pn.root.actionValuesDifference.get(pn.root.getChosenAction());
			siRankedbycost.add(pn.root);
			for (ValueLabel vl : pnVals.keySet()) {
				if (!minPNsAll.containsKey(vl))
					minPNsAll.put(vl, new ArrayList<XAIStateInformation>());
				if (!maxPNsAll.containsKey(vl))
					maxPNsAll.put(vl, new ArrayList<XAIStateInformation>());
				if (Math.abs(pnVals.get(vl) - minVals.get(vl)) < rangeRadius) {
					minPNsAll.get(vl).add(pn.root);
				}
				if (Math.abs(pnVals.get(vl) - maxVals.get(vl)) < rangeRadius) {

					maxPNsAll.get(vl).add(pn.root);
				}
			}
			if (pn.children != null) {
				for (PathNode pnc : pn.children.keySet()) {
					if (pnc != pn)
						pQ.push(pnc);
				}
			}
		}
		for (ValueLabel vl : maxPNs.keySet()) {
			if (vl == ValueLabel.cost) {
				System.out.println(vl.toString());
				System.out.println("Max");
				for (XAIStateInformation sI : maxPNsAll.get(vl))
					System.out.println(sI);

				System.out.println("Min");
				for (XAIStateInformation sI : minPNsAll.get(vl))
					System.out.println(sI);
			}
		}
		//		while(!siRankedbycost.isEmpty())
		//		{
		//			System.out.println(siRankedbycost.remove().toString());
		//		}

	}

	public ArrayList<XAIStateInformation> getSwingStatesListByCost(XAIPolicyPath p)
	{
		List<State> statesSeen = new ArrayList<State>();
		PathNode pn = p.root;
		Stack<PathNode> pQ = new Stack<PathNode>();
		if (pn.children != null) {
			for (PathNode pnc : pn.children.keySet()) {
				pQ.push(pnc);
			}
		}

		ArrayList<XAIStateInformation> siRankedbycost = new ArrayList<XAIStateInformation>();
		while (!pQ.isEmpty()) {
			pn = pQ.pop();
			if (!statesSeen.contains(pn.root.getState()))
				statesSeen.add(pn.root.getState());
			else
				continue;
			siRankedbycost.add(pn.root);
			if (pn.children != null) {
				for (PathNode pnc : pn.children.keySet()) {
					if (pnc != pn)
						pQ.push(pnc);
				}
			}
		}

		Collections.sort(siRankedbycost, new XAIStateInformationRelativeCostComparator());
		return siRankedbycost;

	}

	//so now lets go over the whole path and do weights 
	//the idea is to just break things down 
	//so like for a straight path its weight is just that whole path 
	//for probability you just multiply the whole thing with the prob 
	//why not do a simple difference 
	//lets not weight at all 
	//hmmm 
	public void doActionValuesDifference(XAIPolicyPath p)
	{
		//so we have a path p 
		//the difference is from the root to the successor 
		//and its just the probability multiplied by the cost of the parent - this nodes cost 
		PathNode pn = p.root;
		//assuming a single action for each state 
		//we should not see the same state twice 
		//if we do well thats a problem 
		List<State> statesSeen = new ArrayList<State>();
		double rootCost = pn.root.actionValues.get(pn.root.getChosenAction()).get(ValueLabel.cost);
		if (pn != null) {
			statesSeen.add(pn.root.getState());
			for (PathNode pnc : pn.children.keySet()) {
				doActionValuesDifference(pn, pnc, 1.0, rootCost, statesSeen);
			}
		}

	}

	public void doActionValuesDifference(PathNode p, PathNode c, double pProb, double rootCost, List<State> statesSeen)
	{
		//		System.out.println(p.root);
		//		System.out.println(c.root);
		if (!statesSeen.contains(c.root.getState())) {
			statesSeen.add(c.root.getState());

			if (!p.root.isEqual(c.root.state, c.root.chosenAction)) {
				HashMap<ValueLabel, Double> parentActionValues = p.root.actionValues.get(p.root.chosenAction);
				//		if (c.root.chosenAction != null) {
				HashMap<ValueLabel, Double> childActionValues = c.root.actionValues.get(c.root.chosenAction);
				HashMap<ValueLabel, Double> diff = new HashMap<ValueLabel, Double>();
				double cProb = p.children.get(c);

				for (ValueLabel vl : parentActionValues.keySet()) {
					if (vl == ValueLabel.probability) {
						double probRatio = cProb / pProb;
						if (probRatio == 1)
							probRatio = 0.0;
						diff.put(vl, probRatio);

					} else {
						if (vl == ValueLabel.frequency) {
							diff.put(vl, childActionValues.get(vl));
						} else {

							double trydiff = (parentActionValues.get(vl)) - childActionValues.get(vl);
							//							if(trydiff < 0)
							//								System.out.println("Negative costs here?");
							//negative costs are okay because like there may be other paths that get us to there 
							//so you know its not bad at all 
							//some states are better than others 
							//its not monotonic 

							if (vl == ValueLabel.progression) {
								diff.put(vl, trydiff);
							} else {
								if (vl == ValueLabel.cost) {
									double trydiffrel = trydiff / rootCost;
									diff.put(vl, trydiffrel);
								}
							}
						}
					}
				}
				c.root.actionValuesDifference.put(c.root.getChosenAction(), diff);
				if (c.children != null) {
					for (PathNode cc : c.children.keySet()) {
						doActionValuesDifference(c, cc, cProb, rootCost, statesSeen);
					}
				}
			}
		}
	}

	//run function 
	public HashMap<State, HashMap<State, Double>> mdpDijkstraUpdate(int mdpStateNum, MDP mdp, HashMap<State, HashMap<State, Double>> mdpDistshm)
	{

		double[] dists = mdpDijkstraSingleState(mdp, mdpStateNum);
		if (dists[mdpStateNum] != 0)
			System.out.println("Error");

		//going to normalise the distances so each var has equal influence
		if (mdpDistshm == null)
			mdpDistshm = new HashMap<State, HashMap<State, Double>>();

		boolean normalize = true;

		State s = mdp.getStatesList().get(mdpStateNum);
		int numstates = mdp.getNumStates();
		HashMap<State, Double> hm = new HashMap<State, Double>();
		for (int j = 0; j < numstates; j++) {
			State ns = mdp.getStatesList().get(j);
			if (normalize)
				hm.put(ns, dists[j] / (double) numstates);
			else
				hm.put(ns, dists[j]);

		}
		mdpDistshm.put(s, hm);

		return mdpDistshm;

	}

	public HashMap<State, HashMap<State, Double>> mdpDijkstra(MDP mdp)
	{
		double[][] mdpDists = new double[mdp.getNumStates()][mdp.getNumStates()];
		int numstates = mdp.getNumStates();
		for (int i = 0; i < mdp.getNumStates(); i++) {
			double[] dists = mdpDijkstraSingleState(mdp, i);
			if (dists[i] != 0)
				System.out.println("Error");
			mdpDists[i] = dists;

		}
		boolean normalize = true;
		//going to normalise the distances so each var has equal influence
		HashMap<State, HashMap<State, Double>> mdpDistshm = new HashMap<State, HashMap<State, Double>>();
		for (int i = 0; i < numstates; i++) {

			State s = mdp.getStatesList().get(i);
			HashMap<State, Double> hm = new HashMap<State, Double>();
			for (int j = 0; j < numstates; j++) {
				State ns = mdp.getStatesList().get(j);
				if (normalize)
					hm.put(ns, mdpDists[i][j] / (double) numstates);
				else
					hm.put(ns, mdpDists[i][j]);

			}
			mdpDistshm.put(s, hm);
		}
		return mdpDistshm;
		//		System.out.print("\t\t");
		//		for (int i = 0; i < mdp.getNumStates(); i++) {
		//			System.out.print(mdp.getStatesList().get(i) + "\t");
		//		}
		//		System.out.print("\n");
		//		for (int i = 0; i < mdp.getNumStates(); i++) {
		//			System.out.println(mdp.getStatesList().get(i) + "\t" + Arrays.toString(mdpDists[i]));
		//		}

	}

	public double[] mdpDijkstraSingleState(MDP mdp, int s)
	{
		double maxDist = mdp.getNumStates();
		double[] dist = new double[mdp.getNumStates()];
		double[] prevdist = new double[mdp.getNumStates()];
		for (int i = 0; i < mdp.getNumStates(); i++) {

			dist[i] = maxDist;
			prevdist[i] = maxDist;
		}
		dist[s] = 0;
		prevdist[s] = 0;
		Queue<Integer> sQ = new LinkedList<Integer>();
		sQ.add(s);
		BitSet visited = new BitSet();
		while (!sQ.isEmpty()) {
			int currents = sQ.remove();
			if (!visited.get(currents)) {
				visited.set(currents);
				int numChoices = mdp.getNumChoices(currents);
				for (int i = 0; i < numChoices; i++) {
					Iterator<Entry<Integer, Double>> traniter = mdp.getTransitionsIterator(currents, i);
					while (traniter.hasNext()) {
						Entry<Integer, Double> sp = traniter.next();
						int nexts = sp.getKey();
						double cost = 1 + dist[currents];
						if (cost < dist[nexts]) {
							dist[nexts] = cost;
						}
						sQ.add(nexts);
					}
				}
			}
		}
		return dist;
	}

	public State createNewStateUsingVL(VarList origvl, VarList newvl, State oldState)
	{
		State newState = new State(newvl.getNumVars());
		for (int i = 0; i < newvl.getNumVars(); i++) {
			String varname = newvl.getName(i);
			//does it exist in the original state ? 
			int oldVlIndex = origvl.getIndex(varname);
			if (oldVlIndex != -1) {
				newState.setValue(i, oldState.varValues[oldVlIndex]);
			}

		}
		return newState;
	}

	public double stateDist(State s1p, State s2p, VarList s1vl, VarList s2vl, HashMap<State, HashMap<State, Double>> mdpDijkstraDists) throws Exception
	{
		double dist = 0;

		boolean mdpBitsDone = false;
		//we're doing a kind of euclidean distance 
		//for each variable in vl 
		//step 1 
		//we check which vl is longer 
		VarList vl = s1vl;

		if (s1vl.getNumVars() != s2vl.getNumVars()) {
			if (s1vl.getNumVars() > s2vl.getNumVars()) {
				//if s1 is bigger 
				//well we need to remove states 
				//so the distance is just a distance 
				//if we remove states we're alright 
				//so we'd rather remove states than unremove them ? 
				//cuz that would be filling them in with something 
				vl = s2vl;
				//now we must equal them 
				//so we go over all the states in s1vl 
				//and get rid of any names not in s1vl 
				//and create a new state 
				s1p = this.createNewStateUsingVL(s1vl, s2vl, s1p);
			} else {
				vl = s1vl;
				s2p = this.createNewStateUsingVL(s2vl, s1vl, s2p);
			}
		}
		State s1 = this.getMDPState(vl, s1p);
		State s2 = this.getMDPState(vl, s2p);
		int numVars = vl.getNumVars();
		double weight = 1.0; 
		if (!s1p.toString().contentEquals(s2p.toString())) {

			double[] dists = new double[numVars];
			//so we'll do all vars that are in the special list or part of the da 
			for (int var = 0; var < numVars; var++) {
				String vname = vl.getName(var);
				if(variableWeights.containsKey(vname))
					weight = variableWeights.get(vname); 
				else 
					weight = 1.0; 
				int s1var = 0;
				if (s1p.varValues[var] instanceof Boolean) {
					if ((Boolean) s1p.varValues[var] == true)
						s1var = 1;
				} else
					s1var = (int) s1p.varValues[var];
				int s2var = 0;
				if (s2p.varValues[var] instanceof Boolean) {
					if ((Boolean) s2p.varValues[var] == true)
						s2var = 1;
				} else
					s2var = (int) s2p.varValues[var];
				if (vname.contains("da")) {
					dists[var] = Math.abs(s1var - s2var);
					//dists[var] = Math.abs(da.getDistsToAcc().get(s1var) - da.getDistsToAcc().get(s2var));
					//TODO:the distance to a sink state should be a very large number 
					if (this.dasinkStates.get(s2var))
						dists[var] = da.size();
				} else {
					if (nonMDPDistVars.contains(vname)) {

						if (s1var == s2var)
							dists[var] = 0;
						else {
							if (vname.contains("turn"))
								dists[var] = mdp.getNumStates();
							else
								dists[var] = 1;
						}
					} else {
						if (!mdpBitsDone) {
							//because we're only using dijkstras if the s variable is different 
							boolean sameValues = false;
							Object s1sval = s1p.varValues[var];
							Object s2sval = s2p.varValues[var];

							if (s1sval instanceof Boolean) {
								boolean s1varh = (Boolean) s1sval;
								boolean s2varh = (Boolean) s2sval;
								sameValues = s1varh == s2varh;
							} else {
								int s1varh = (int) s1sval;
								int s2varh = (int) s2sval;
								sameValues = s1varh == s2varh;
							}
							if (!sameValues) {
								if (mdpDijkstraDists.containsKey(s1)) {
									if (mdpDijkstraDists.get(s1).containsKey(s2)) {
										dists[var] = mdpDijkstraDists.get(s1).get(s2);
									} else {
										throw new Exception("no distance?");
									}
								} else if (mdpDijkstraDists.containsKey(s2)) {
									if (mdpDijkstraDists.get(s2).containsKey(s1)) {
										dists[var] = mdpDijkstraDists.get(s2).get(s1);
									} else {
										throw new Exception("no distance?");
									}
								}
								mdpBitsDone = true;
								weight = variableWeights.get("mdp");
							}

						} else {
							dists[var] = 0;
						}
					}
				}
				//				}
				dists[var] *= (dists[var]*weight);
				dist += dists[var];
			}
			dist = Math.sqrt(dist);
		}
		return dist;
	}

	public void run()
	{

		try {

			doStuff();
			//			identifySwingStates();
			//			identifySwingStatesContext();

		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
