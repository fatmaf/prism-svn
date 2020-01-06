package demos;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
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

public class XAITemp
{

	Prism prism;
	PrismLog mainLog;
	MDPModelChecker mc;
	MDP mdp;
	DA<BitSet, ? extends AcceptanceOmega> da;
	BitSet accStates;
	BitSet sinkStates;
	int daVarInd = -1;
	ModulesFile modulesFile=null; 
	
	//main function 
	public static void main(String[] args)
	{
		new XAITemp().run();
	}

	public ArrayList<BitSet> setStateLabels(Vector<BitSet> labelBS, MDP mdp)
	{
		//initialise to get labels for actions 
		int numAPs = da.getAPList().size();
		BitSet s_labels = new BitSet(numAPs);
		//associate a label with each state in the mdp 
		ArrayList<BitSet> statelabels = new ArrayList<BitSet>();
		for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
			s_labels = new BitSet(numAPs);
			// Get BitSet representing APs (labels) satisfied by state s_0
			for (int k = 0; k < numAPs; k++) {
				s_labels.set(k, labelBS.get(Integer.parseInt(da.getAPList().get(k).substring(1))).get(s_0));
			}
			// Find corresponding initial state in DA
			int q_0 = da.getEdgeDestByLabel(da.getStartState(), s_labels);
			mainLog.println(mdp.getStatesList().get(s_0) + " " + s_0 + "," + q_0 + ":" + s_labels.toString());
			statelabels.add(s_labels);
		}
		return statelabels;
	}

	public HashMap<State, HashMap<Object, ArrayList<BitSet>>> getStateActionLabels(ArrayList<BitSet> statelabels, MDP mdp)
	{
		//attach a label to an action of the mdp 
		//basically that of the next state 
		HashMap<State, HashMap<Object, ArrayList<BitSet>>> actionLabels = new HashMap<State, HashMap<Object, ArrayList<BitSet>>>();
		for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
			//go over all these states again 
			//assign labels to actions as those for the next state 
			//but what about probabilistic ones ? 
			int numChoices = mdp.getNumChoices(s_0);
			State state = mdp.getStatesList().get(s_0);
			if (!actionLabels.containsKey(state)) {
				actionLabels.put(state, new HashMap<Object, ArrayList<BitSet>>());
			}

			for (int i = 0; i < numChoices; i++) {
				Object action = mdp.getAction(s_0, i);
				if (!actionLabels.get(state).containsKey(action))
					actionLabels.get(state).put(action, new ArrayList<BitSet>());

				Iterator<Entry<Integer, Double>> trIter = mdp.getTransitionsIterator(s_0, i);
				while (trIter.hasNext()) {
					Entry<Integer, Double> nextSP = trIter.next();
					int ns = nextSP.getKey();
					actionLabels.get(state).get(action).add(statelabels.get(ns));

				}
			}
		}
		return actionLabels;
	}

	//so only has state actions that lead to a sink state 
	//we dont care about the label really 

	public HashMap<State, ArrayList<Object>> getStateActionLabelsSinkStates(ArrayList<BitSet> statelabels, MDP mdp) throws PrismException
	{
		//attach a label to an action of the mdp 
		//basically that of the next state 
		HashMap<State, ArrayList<Object>> actionLabels = new HashMap<State, ArrayList<Object>>();
		for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
			//go over all these states again 
			//assign labels to actions as those for the next state 
			//but what about probabilistic ones ? 
			int numChoices = mdp.getNumChoices(s_0);
			State state = mdp.getStatesList().get(s_0);
			int daState = getDAStateVar(mdp, state);

			for (int i = 0; i < numChoices; i++) {
				Object action = mdp.getAction(s_0, i);

				Iterator<Entry<Integer, Double>> trIter = mdp.getTransitionsIterator(s_0, i);
				while (trIter.hasNext()) {
					Entry<Integer, Double> nextSP = trIter.next();
					int ns = nextSP.getKey();
					State nextState = mdp.getStatesList().get(ns);
					int nextDaState = getDAStateVar(mdp, nextState);
					if (sinkStates.get(nextDaState)) {
						if (!actionLabels.containsKey(state)) {
							actionLabels.put(state, new ArrayList<Object>());
						}
						if (!actionLabels.get(state).contains(action)) {
							actionLabels.get(state).add(action);
						}
					}

				}
			}
		}
		return actionLabels;
	}

	public HashMap<Object, ArrayList<BitSet>> getActionLabelsOnly(ArrayList<BitSet> statelabels, MDP mdp)
	{
		//attach a label to an action of the mdp 
		//basically that of the next state 
		HashMap<Object, ArrayList<BitSet>> actionLabels = new HashMap<Object, ArrayList<BitSet>>();
		for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
			//go over all these states again 
			//assign labels to actions as those for the next state 
			//but what about probabilistic ones ? 
			int numChoices = mdp.getNumChoices(s_0);
			for (int i = 0; i < numChoices; i++) {
				Object action = mdp.getAction(s_0, i);
				if (!actionLabels.containsKey(action)) {
					actionLabels.put(action, new ArrayList<BitSet>());
				}
				Iterator<Entry<Integer, Double>> trIter = mdp.getTransitionsIterator(s_0, i);
				while (trIter.hasNext()) {
					Entry<Integer, Double> nextSP = trIter.next();
					int ns = nextSP.getKey();
					actionLabels.get(action).add(statelabels.get(ns));

				}
			}
		}
		return actionLabels;
	}

	public void mdpActionsToDAPropsNoProduct() throws PrismException, FileNotFoundException
	{
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		String filename = "g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";

		PropertiesFile propertiesFile = readModel(saveplace, filename);

		ExpressionReward exprReward = (ExpressionReward) propertiesFile.getProperty(0);
		Expression expr = exprReward.getExpression();
		mainLog.println(exprReward.toString());

		Vector<BitSet> labelBS = new Vector<BitSet>();
		convertPropertyToDA(expr, labelBS);

		mainLog.println(da.getAPList().toString());
		mainLog.println(expr.toString());

		ArrayList<BitSet> statelabels = setStateLabels(labelBS, mdp);

		HashMap<Object, ArrayList<BitSet>> actionLabels = getActionLabelsOnly(statelabels, mdp);

		for (Object a : actionLabels.keySet()) {
			mainLog.println(a.toString() + " " + actionLabels.get(a).toString());
		}
		//map da stuff to actions 
		//so essentially form bins 

	}

	public PropertiesFile readModel(String saveplace, String filename) throws FileNotFoundException, PrismException
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
		PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + "_rew.props"));
		prism.buildModel();
		mdp = (MDP) prism.getBuiltModelExplicit();

		//set up the model checker 
		mc = new MDPModelChecker(prism);
		mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));
		return propertiesFile;

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

	public BitSet getAccStatesReachorRabin(DA<BitSet, ? extends AcceptanceOmega> da)
	{
		BitSet acc = null;
		AcceptanceOmega acceptance = da.getAcceptance();
		//Check if its a DFA
		if (acceptance instanceof AcceptanceReach) {
			acc = ((AcceptanceReach) acceptance).getGoalStates();
		} else if (acceptance instanceof AcceptanceRabin) {
			acc = da.getRabinAccStates();
		}
		return acc;
	}

	public BitSet getSinkStates(DA<BitSet, ? extends AcceptanceOmega> da, BitSet accStates)
	{
		BitSet sinkStates = new BitSet();
		for (int i = 0; i < da.size(); i++) {
			boolean isSinkState = false;
			int numEdges = da.getNumEdges(i);
			int numEdgesSrcToSrc = da.getNumEdges(i, i);
			if (numEdges == numEdgesSrcToSrc) {
				isSinkState = true;

			}
			if (numEdges == 0)
				isSinkState = true;
			if (isSinkState) {
				if (accStates.get(i) == false)
					sinkStates.set(i);
			}
		}
		return sinkStates;
	}

	public HashMap<Integer, List<BitSet>> getStateLabelPairsLeadingToSinkStates(BitSet sinkStates)
	{
		HashMap<Integer, List<BitSet>> stateLabelPairsLeadingToSinkStates = new HashMap<Integer, List<BitSet>>();

		//what if you had multiple sink states ?

		//now get labels that make us transition to sink states 
		int nextSinkState = sinkStates.nextSetBit(0);
		while (nextSinkState != -1) {
			for (int src = 0; src < da.size(); src++) {
				if (!sinkStates.get(src)) {
					int numEdges = da.getNumEdges(src, nextSinkState);
					if (numEdges > 0) {
						//all the states that lead here 
						mainLog.println(numEdges + " between " + src + " - " + nextSinkState);

						List<BitSet> edgeLabels = da.getEdgeLabels(src, nextSinkState);
						mainLog.println(edgeLabels.toString());
						if (!stateLabelPairsLeadingToSinkStates.containsKey(src)) {
							stateLabelPairsLeadingToSinkStates.put(src, edgeLabels);
						} else {
							stateLabelPairsLeadingToSinkStates.get(src).addAll(edgeLabels);
						}

					}
				}
			}
			nextSinkState = sinkStates.nextSetBit(nextSinkState + 1);
		}
		return stateLabelPairsLeadingToSinkStates;
	}
	public HashMap<Integer, List<BitSet>> processDA(	DA<BitSet, ? extends AcceptanceOmega> da, Vector<BitSet> labelBS) throws PrismException
	{
		this.da = da; 
		mainLog.println(da.size());
		//get sink states 
		//for each state in the da 
		//if it has no outward edges or only one that leads to itself its a deadend 
		accStates = getAccStatesReachorRabin(da);
		sinkStates = getSinkStates(da, accStates);
		HashMap<Integer, List<BitSet>> stateLabelPairsLeadingToSinkStates = getStateLabelPairsLeadingToSinkStates(sinkStates);

		mainLog.println("Acc States " + accStates.toString());
		mainLog.println("Sink States " + sinkStates.toString());
		return stateLabelPairsLeadingToSinkStates;
	}
	public HashMap<Integer, List<BitSet>> convertPropertyToDA(Expression expr, Vector<BitSet> labelBS) throws PrismException
	{
		//convert property to da 
		LTLModelChecker mcLTL = new LTLModelChecker(prism);
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };

		da = mcLTL.constructDAForLTLFormula(mc, mdp, expr, labelBS, allowedAcceptance);
		mainLog.println(da.size());
		//get sink states 
		//for each state in the da 
		//if it has no outward edges or only one that leads to itself its a deadend 
		accStates = getAccStatesReachorRabin(da);
		sinkStates = getSinkStates(da, accStates);
		HashMap<Integer, List<BitSet>> stateLabelPairsLeadingToSinkStates = getStateLabelPairsLeadingToSinkStates(sinkStates);

		mainLog.println("Acc States " + accStates.toString());
		mainLog.println("Sink States " + sinkStates.toString());
		return stateLabelPairsLeadingToSinkStates;
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

	public void mdpActionsToDAPropsProduct() throws PrismException, FileNotFoundException
	{
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		String filename = "xai_r1_d1_g1_fs1notgoal";//"g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";

		PropertiesFile propertiesFile = readModel(saveplace, filename);

		ExpressionReward exprRew = (ExpressionReward) propertiesFile.getProperty(0);
		mainLog.println(exprRew.toString());

		setMCExportOptionsAll(saveplace, filename);

		Expression expr = exprRew.getExpression();
		Entry<MDP, MDStrategy> prodStratPair = mc.checkPartialSatExprReturnStrategy(mdp, expr, exprRew, null);
		MDStrategy strat = prodStratPair.getValue();
		MDP productMDP = prodStratPair.getKey();
		PolicyCreator pc = new PolicyCreator();
		pc.createPolicy(productMDP, strat);
		pc.savePolicy(saveplace + "results/", "xai_" + filename + "_policy");
		Vector<BitSet> labelBS = new Vector<BitSet>();
		convertPropertyToDA(expr, labelBS);

		mdp = productMDP;
		//initialise to get labels for actions 

		mainLog.println(expr.toString());

		ArrayList<BitSet> statelabels = setStateLabels(labelBS, mdp);

		HashMap<State, HashMap<Object, ArrayList<BitSet>>> actionLabels = getStateActionLabels(statelabels, mdp);
		HashMap<State, ArrayList<Object>> sinkStateActionLabels = getStateActionLabelsSinkStates(statelabels, mdp);
		try {
			openDotFile(saveplace + "results/", "xai_" + filename + "_policy.dot");
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		try {
			String userInput = "";
			String exitString = "exit";
			while (!userInput.contentEquals(exitString)) {
				BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
				System.out.println("List actions in state: (exit to exit)");

				userInput = reader.readLine();
				if (!userInput.contentEquals(exitString)) {
					System.out.println("Displaying actions for first state that resembles " + userInput);
					State chosenState = null;
					//
					for (State stateHere : actionLabels.keySet()) {

						if (stateHere.toString().contains(userInput)) {

							mainLog.println(actionLabels.get(stateHere).keySet().toString());
							chosenState = stateHere;
							break;
						}
					}
					System.out.println("Why not action?");
					userInput = reader.readLine();
					System.out.println("Why not action " + userInput);
					if (chosenState != null) {
						Object chosenAction = null;
						HashMap<Object, ArrayList<BitSet>> actionLabelsForThisState = actionLabels.get(chosenState);
						for (Object a : actionLabelsForThisState.keySet()) {
							if (a.toString().contains(userInput)) {
								chosenAction = a;
								break;
							}
						}
						if (chosenAction != null) {
							//					System.out.println(actionLabelsForThisState.get(chosenAction).toString());
							boolean actionOkay = true;
							if (sinkStateActionLabels.containsKey(chosenState)) {
								if (sinkStateActionLabels.get(chosenState).contains(chosenAction)) {
									actionOkay = false;
								}
							}
							if (!actionOkay) {
								mainLog.println(chosenAction.toString() + " violates the LTL specification in state " + chosenState.toString());
							} else {
								mainLog.println(chosenAction.toString() + " DOES NOT violate the LTL specification in state " + chosenState.toString()
										+ " so there's probably another reason!"
										+ "\nOr it was chosen.");

							}
						}
					}
				}
			}

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
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
			System.out.println("No Product");
			mdpActionsToDAPropsNoProduct();
			System.out.println("No Product");
			System.out.println("Product");
			mdpActionsToDAPropsProduct();
			System.out.println("Product");
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
