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
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import java.util.Vector;
import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import cern.colt.Arrays;
import common.IterableStateSet;
import demos.PolicyPath.PathNode;
import demos.StateInformation.ValueLabel;
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
		if (a != null) {
			int numChoices = mdp.getNumChoices(s);
			for (int i = 0; i < numChoices; i++) {
				Object ao = mdp.getAction(s, i);
				if (ao.toString().contentEquals(a.toString())) {
					aI = i;
					break;
				}
			}
		}
		return aI;
	}

	public void doStuff() throws Exception
	{
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/xaiTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		String filename = "xai_r1_d1_g1_key";//"xai_r1_d1_g1_key";//_fs1notgoal_noback";//"g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";

		HashMap<String, Double> discountVals = new HashMap<String, Double>();
		discountVals.put("xai_r1_d1_g1", 1.0);
		discountVals.put("xai_r1_d1_g1_key", 0.9);

		double discount = discountVals.get(filename);
		readModel(saveplace, filename);

		//just testing the distance stuff 
		//		double[] dists = mdpDijkstraSingleState(mdp, mdp.getFirstInitialState());
		//		System.out.println(Arrays.toString(dists));
		HashMap<State, HashMap<State, Double>> mdpDistshm = mdpDijkstra(mdp);
		ExpressionReward exprRew = (ExpressionReward) propertiesFile.getProperty(0);
		mainLog.println(exprRew.toString());

		setMCExportOptionsAll(saveplace, filename);

		Expression expr = exprRew.getExpression();

		NVIExposed nvi = new NVIExposed();
		ModelCheckerPartialSatResult res = nvi.nviexposed(prism, mainLog, mdp, expr, exprRew, null, modulesFile, mc, discount);
		PrismFileLog pfl = new PrismFileLog(saveplace + "results/" + "xai_" + filename + "_prodmpd.dot");
		nvi.productmdp.exportToDotFile(pfl, null, true);
		pfl.close();
		PathCreator pathO = new PathCreator();
		pathO.createPathPolicy(0, nvi.productmdp, res.strat, saveplace + "results/", "xai_" + filename + "_path" + 0, nvi.progRewards, nvi.costRewards,
				nvi.accStates);
		HashMap<State, PriorityQueue<Entry<State, Double>>> pathOdists = calculateStateDistances(pathO, mdpDistshm);
		ModelCheckerPartialSatResult pathONVI = nvi.doPathNVI(prism, mainLog, pathO, mc, discount);
		ModelCheckerPartialSatResult pathOoccfreq = nvi.doPathOccupancyFreq(prism, mainLog, pathO, mc, discount);
		//state1 action 2 the door action 
		PathCreator pathC = new PathCreator();
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
		PolicyPath pathCPath = traversePath(pathC, pathCNVI, pathCoccfreq);
		doActionValuesDifference(pathCPath);

		printMostInfluentialStates(pathCPath);

		PolicyPath pathOPath = traversePath(pathO, pathONVI, pathOoccfreq);
		doActionValuesDifference(pathOPath);
		printMostInfluentialStates(pathOPath);
		//now lets compare these paths 
		//we can go home if we do this. 

	}

	public HashMap<State, PriorityQueue<Entry<State, Double>>> calculateStateDistances(PathCreator pathC, HashMap<State, HashMap<State, Double>> dijkstraDists)
			throws Exception
	{
		MDP pmdp = pathC.pc.mdpCreator.mdp;
		int numStates = pmdp.getNumStates();
		List<State> sl = pmdp.getStatesList();
		VarList vl = pmdp.getVarList();
		HashMap<State, PriorityQueue<Entry<State, Double>>> toret = new HashMap<State, PriorityQueue<Entry<State, Double>>>();
		for (int i = 0; i < numStates; i++) {
			State istate = sl.get(i);
			PriorityQueue<Entry<State, Double>> pq = new PriorityQueue<Entry<State, Double>>((a, b) -> (a.getValue().compareTo(b.getValue())));
			for (int j = 0; j < numStates; j++) {
				State jstate = sl.get(j);
				double dist = stateDist(istate, jstate, vl, dijkstraDists);
				Entry<State, Double> e = new AbstractMap.SimpleEntry<State, Double>(jstate, dist);
				pq.add(e);
			}
			toret.put(istate, pq);
		}
		return toret;
	}

	public PolicyPath traversePath(PathCreator pathC, ModelCheckerPartialSatResult pathCNVI, ModelCheckerPartialSatResult pathCoccfreq)
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
		PolicyPath path = null;
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

					StateInformation s = new StateInformation(currentStateState, currentState, action);
					s.addValuesForState(action, stateValues);
					if (path == null) {
						path = new PolicyPath(s);

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
						StateInformation nextSI = new StateInformation(nextState, nextS, null);
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
					StateInformation s = new StateInformation(currentStateState, currentState, action);
					s.addValuesForState(action, stateValues);
					path.addToStatesList(s);
				}

			}
		}
		return path;

	}

	public void printMostInfluentialStates(PolicyPath p)
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
		HashMap<ValueLabel, StateInformation> maxPNs = new HashMap<ValueLabel, StateInformation>();
		HashMap<ValueLabel, StateInformation> minPNs = new HashMap<ValueLabel, StateInformation>();

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
		HashMap<ValueLabel, ArrayList<StateInformation>> minPNsAll = new HashMap<ValueLabel, ArrayList<StateInformation>>();
		HashMap<ValueLabel, ArrayList<StateInformation>> maxPNsAll = new HashMap<ValueLabel, ArrayList<StateInformation>>();

		PriorityQueue<StateInformation> siRankedbycost = new PriorityQueue<StateInformation>(new StateInformationRelativeCostComparator());
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
					minPNsAll.put(vl, new ArrayList<StateInformation>());
				if (!maxPNsAll.containsKey(vl))
					maxPNsAll.put(vl, new ArrayList<StateInformation>());
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
				for (StateInformation sI : maxPNsAll.get(vl))
					System.out.println(sI);

				System.out.println("Min");
				for (StateInformation sI : minPNsAll.get(vl))
					System.out.println(sI);
			}
		}
		//		while(!siRankedbycost.isEmpty())
		//		{
		//			System.out.println(siRankedbycost.remove().toString());
		//		}

	}

	//so now lets go over the whole path and do weights 
	//the idea is to just break things down 
	//so like for a straight path its weight is just that whole path 
	//for probability you just multiply the whole thing with the prob 
	//why not do a simple difference 
	//lets not weight at all 
	//hmmm 
	public void doActionValuesDifference(PolicyPath p)
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
		//going to normalise the distances so each var has equal influence
		HashMap<State, HashMap<State, Double>> mdpDistshm = new HashMap<State, HashMap<State, Double>>();
		for (int i = 0; i < numstates; i++) {

			State s = mdp.getStatesList().get(i);
			HashMap<State, Double> hm = new HashMap<State, Double>();
			for (int j = 0; j < numstates; j++) {
				State ns = mdp.getStatesList().get(j);
				hm.put(ns, mdpDists[i][j] / (double) numstates);

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
		State state = mdp.getStatesList().get(s);
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

	public double stateDist(State s1, State s2, VarList vl, HashMap<State, HashMap<State, Double>> mdpDijkstraDists) throws Exception
	{
		double dist = 0;
		//we're doing a kind of euclidean distance 
		//for each variable in vl 
		int numVars = vl.getNumVars();
		if (!s1.toString().contentEquals(s2.toString())) {

			double[] dists = new double[numVars];
			for (int var = 0; var < numVars; var++) {
				String vname = vl.getName(var);
				//so  the distance measures are different 
				//if the varname has door or key we just do like a change //edit distance 
				//if it s we use mdpDijkstraDist 
				if (vname.contentEquals("s")) {

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

				} else {

					int s1var = (int) s1.varValues[var];
					int s2var = (int) s2.varValues[var];
					if (vname.contains("da"))
						dists[var] = Math.abs(s1var - s2var);
					else {
						if (s1var == s2var)
							dists[var] = 0;
						else
							dists[var] = 1;
					}
				}
				dists[var] *= dists[var];
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
