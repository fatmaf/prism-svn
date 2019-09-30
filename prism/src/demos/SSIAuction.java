package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;

import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.ModelCheckerResult;
import explicit.StateValues;
import parser.State;
import parser.VarList;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismLog;
import prism.Result;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;
import strat.Strategy;
import strat.Strategy.Choice;

public class SSIAuction
{

	public Entry<Integer, Double> getSingleAgentBid(MDP agentMDP, ArrayList<Expression> taskSet, Expression agentTasks, ExpressionReward rewardExpr,
			MDPModelChecker mc) throws PrismException
	{

		Expression bidPick = null;
		double bidValue = 100000;
		//		Expression robotTasks = null;
		Expression currentTask = null;
		int indexOfChosenTask = -1;

		for (int exprNum = 0; exprNum < taskSet.size(); exprNum++) {
			currentTask = taskSet.get(exprNum);
			if (agentTasks != null)
				currentTask = Expression.And(agentTasks, currentTask);
			double costInInitState = mc.checkPartialSatExposed(agentMDP, currentTask, rewardExpr, null)[2];
			
			if (costInInitState < bidValue) {
				bidPick = taskSet.get(exprNum);
				bidValue = costInInitState;
				indexOfChosenTask = exprNum;
			}

		}
		return new AbstractMap.SimpleEntry<Integer, Double>(indexOfChosenTask, bidValue);
	}

	public Entry<ExpressionReward, ArrayList<Expression>> processProperties(PropertiesFile propertiesFile, PrismLog mainLog)
	{
		// Model check the first property from the file using the model checker
		for (int i = 0; i < propertiesFile.getNumProperties(); i++)
			mainLog.println(propertiesFile.getProperty(i));
		ExpressionFunc expr = (ExpressionFunc) propertiesFile.getProperty(0);
		int numOp = expr.getNumOperands();
		ArrayList<Expression> ltlExpressions = new ArrayList<Expression>(numOp);

		ExpressionReward rewExpr = null;
		for (int exprNum = 0; exprNum < numOp; exprNum++) {
			ltlExpressions.add((expr.getOperand(exprNum)));
			//full expression 
			mainLog.println(ltlExpressions.get(exprNum));
			if (ltlExpressions.get(exprNum) instanceof ExpressionReward)
				rewExpr = (ExpressionReward) ltlExpressions.get(exprNum);

		}
		Expression safetyExpr = ((ExpressionQuant) ltlExpressions.get(numOp - 1)).getExpression();
		ArrayList<Expression> taskSet = new ArrayList<Expression>();

		for (int exprNum = 0; exprNum < numOp - 1; exprNum++) {

			Expression currentExpr = ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression();
			Expression currentExprWithSafetyExpr = Expression.And(currentExpr, safetyExpr);
			taskSet.add(currentExprWithSafetyExpr);
		}
		return new AbstractMap.SimpleEntry<ExpressionReward, ArrayList<Expression>>(rewExpr, taskSet);
	}

	public ArrayList<Expression> auctionTasks(ArrayList<Expression> taskSet, int numRobots, ArrayList<MDP> mdps, ExpressionReward rewExpr,
			ArrayList<MDPModelChecker> mcs) throws PrismException
	{
		ArrayList<Expression> robotsTasks = new ArrayList<Expression>();

		for (int i = 0; i < numRobots; i++)
			robotsTasks.add(null);

		while (!taskSet.isEmpty()) {

			int bestBidIndex = -1;
			double bestBidValue = 10000;
			Expression bestBidExpression = null;
			int bestBidRobotIndex = -1;

			for (int i = 0; i < numRobots; i++) {
				Entry<Integer, Double> bid = getSingleAgentBid(mdps.get(i), taskSet, robotsTasks.get(i), rewExpr, mcs.get(i));
				int bidTaskIndex = bid.getKey();
				double bidValue = bid.getValue();

				if (bidValue < bestBidValue) {
					bestBidValue = bidValue;
					bestBidIndex = bidTaskIndex;
					bestBidRobotIndex = i;
					bestBidExpression = taskSet.get(bestBidIndex);
				}
			}
			Expression robotTasks = robotsTasks.get(bestBidRobotIndex);
			if (robotTasks != null) {
				bestBidExpression = Expression.And(robotTasks, bestBidExpression);

			}
			robotsTasks.set(bestBidRobotIndex, bestBidExpression);
			taskSet.remove(bestBidIndex);

		}
		return robotsTasks;
	}

	public void getSingleAgentPlansUsingNVI(int numRobots, ArrayList<MDP> mdps, ExpressionReward rewExpr, ArrayList<MDPModelChecker> mcs,
			ArrayList<Expression> robotsTasks, ArrayList<MDStrategy> nviStrategies, ArrayList<MDP> productMDPs, PrismLog mainLog, String saveplace,
			String filename) throws PrismException
	{
		
		for (int i = 0; i < numRobots; i++) {

			mcs.get(i).setGenStrat(true);
			int initState = mdps.get(i).getFirstInitialState();
			Entry<MDP, MDStrategy> prodStratPair = mcs.get(i).checkPartialSatExprReturnStrategy(mdps.get(i), robotsTasks.get(i), rewExpr, null);
			MDP productMDP = prodStratPair.getKey();
			productMDPs.add(productMDP);
			MDStrategy nviStrategy = prodStratPair.getValue();
			initState = productMDP.getFirstInitialState();
			nviStrategy.initialise(initState);
			Object action = nviStrategy.getChoiceAction();
			mainLog.println(i + ":" + initState + "->" + action.toString());
			mainLog.println(i + ":" + robotsTasks.get(i).toString());
			PolicyCreator pc = new PolicyCreator();
			pc.createPolicy(productMDP, nviStrategy);
			pc.savePolicy(saveplace + "results/", filename + "_" + i);
			nviStrategies.add(nviStrategy);
		}
	}

	public PropertiesFile loadFiles(Prism prism, String saveplace, String filename, int numRobots, ArrayList<MDP> mdps, ArrayList<MDPModelChecker> mcs)
			throws FileNotFoundException, PrismException
	{
		PropertiesFile propertiesFile = null;
		//load the files 
		//could be a separate function 
		for (int i = 0; i < numRobots; i++) {
			String modelFileName = saveplace + filename + i + ".prism";
			ModulesFile modulesFile = prism.parseModelFile(new File(modelFileName));
			prism.loadPRISMModel(modulesFile);
			propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + ".prop"));
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit();
			mdps.add(mdp);
			MDPModelChecker mc = new MDPModelChecker(prism);
			mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));
			mcs.add(mc);
		}
		return propertiesFile;
	}

	public void run()
	{
		try {
			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
			String filename = "g5_r2_t3_d2_fs1";//"g7_r5_t6_d3_fs2";//"g7x3_r2_t3_d0_fs1";//"robot";
			ArrayList<String> ssNames = new ArrayList<String>();
			ssNames.add("door1");
			ssNames.add("door0");

			// Create a log for PRISM output (hidden or stdout)
			//PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);

			prism.initialise();
			prism.setEngine(Prism.EXPLICIT);

			int numRobots = 2; //numRobots

			ArrayList<MDP> mdps = new ArrayList<MDP>();
			ArrayList<MDPModelChecker> mcs = new ArrayList<MDPModelChecker>();

			PropertiesFile propertiesFile = loadFiles(prism, saveplace, filename, numRobots, mdps, mcs);

			//do things to the properties 

			Entry<ExpressionReward, ArrayList<Expression>> processedProperties = processProperties(propertiesFile, mainLog);
			ExpressionReward rewExpr = processedProperties.getKey();

			ArrayList<Expression> taskSet = processedProperties.getValue();

			ArrayList<Expression> robotsTasks = auctionTasks(taskSet, numRobots, mdps, rewExpr, mcs);
			mainLog.println("\n\nAssigned Tasks");
			for(Expression rexpr:robotsTasks)
			{
				mainLog.println(rexpr.toString());
			}
			//print task distribution and get strategy 
			ArrayList<MDStrategy> nviStrategies = new ArrayList<MDStrategy>();
			ArrayList<MDP> productMDPs = new ArrayList<MDP>();

			getSingleAgentPlansUsingNVI(numRobots, mdps, rewExpr, mcs, robotsTasks, nviStrategies, productMDPs, mainLog, saveplace, "ssi_"+filename);

			Queue<State> potentialReallocStates = createJointPolicy(numRobots, mainLog, productMDPs, nviStrategies, saveplace, "ssi_"+filename, ssNames);

			mainLog.println("Potential ReallocStates");
			while (!potentialReallocStates.isEmpty()) {
				State s = potentialReallocStates.remove();
				mainLog.println(s);
			}
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	int doJointStateCreationSetup(int numRobots, ArrayList<MDP> productMDPs, int[] currentStates, int numSS, ArrayList<String> ssNames,
			ArrayList<ArrayList<Integer>> daIndices, HashMap<String, ArrayList<Integer>> ssIndices, ArrayList<ArrayList<Integer>> privateIndices)
	{

		int numStateVars = 0;

		for (int i = 0; i < numRobots; i++) {

			//get the varlists and the da indices 

			MDP mdp = productMDPs.get(i);

			VarList vl = mdp.getVarList();
			//so lets go over all the varlist things and if they have the words da, 
			//we put them in the da indices list 
			//and if they match stuf in ss we put them in ss 
			//and if neither we put them in private 

			for (int j = 0; j < vl.getNumVars(); j++) {
				String name = vl.getName(j);
				if (name.contains("da")) {
					if (daIndices.size() <= i)
						daIndices.add(new ArrayList<Integer>());
					daIndices.get(i).add(j);
				} else {
					boolean hasSS = false;
					if (numSS > 0) {

						if (ssNames.contains(name)) {
							if (!ssIndices.containsKey(name)) {
								ssIndices.put(name, new ArrayList<Integer>());
							}
							ssIndices.get(name).add(j);
							hasSS = true;
						}
					}

					if (!hasSS) {
						if (privateIndices.size() <= i)
							privateIndices.add(new ArrayList<Integer>());
						privateIndices.get(i).add(j);
					}
				}
			}

			currentStates[i] = mdp.getFirstInitialState();
			numStateVars += (mdp.getVarList().getNumVars() - numSS);

			//		
		}
		numStateVars += numSS;
		return numStateVars;
	}

	State createJointState(int numStateVars,
			//			int numRobots, 
			ArrayList<MDP> productMDPs, int[] currentStates, ArrayList<ArrayList<Integer>> daIndices, HashMap<String, ArrayList<Integer>> ssIndices,
			ArrayList<ArrayList<Integer>> privateIndices, HashMap<Integer, int[]> jsToRobotState, State parentState, HashMap<String, Integer> jsSSIndices)
			throws PrismException
	{

		State jointState = new State(numStateVars);
		//so we have the dastate indices 
		//we have the private state indices 
		//we have the shared state vars 
		//for now i'm ignoring the shared state vars 
		//TODO: add shared state stuff 

		//so we do this for each robot really 
		//its easier to do the da first and then the ss then the ps 
		int jsIndex = 0;
		ArrayList<State> robotStateStates = new ArrayList<State>();
		for (int i = 0; i < daIndices.size(); i++) {

			State rs = productMDPs.get(i).getStatesList().get(currentStates[i]);
			robotStateStates.add(rs);

			//now lets add these in succession 
			for (int j = 0; j < daIndices.get(i).size(); j++) {
				int daInd = daIndices.get(i).get(j);
				jointState.setValue(jsIndex, rs.varValues[daInd]);
				if (parentState == null) {
					int[] arr = new int[2];
					arr[0] = i;
					arr[1] = daInd;
					jsToRobotState.put(jsIndex, arr.clone());
				}
				jsIndex++;

			}

		}
		//ss stuff 

		if (ssIndices != null) {
			for (String ss : ssIndices.keySet()) {
				//to resolve here - race conditions for the shared state 
				//so we need an original or reference 

				//we have no reference so we just take both
				int r = 0;
				State rs = robotStateStates.get(r);
				int currentJSValue = (int) rs.varValues[ssIndices.get(ss).get(r)];
				for (r = 1; r < productMDPs.size(); r++) {
					rs = robotStateStates.get(r);
					int nextJSValue = (int) rs.varValues[ssIndices.get(ss).get(r)];
					if (currentJSValue != nextJSValue) {
						if (parentState != null) {
							//if they're not the same 
							//use reference 
							if (nextJSValue != (int) parentState.varValues[jsSSIndices.get(ss)])

							{
								currentJSValue = nextJSValue;
							}
						} else {
							throw new PrismException("Shared State values aren't the same in the initial state!!");
						}
					}

				}
				jointState.setValue(jsIndex, currentJSValue);
				if (parentState == null) {
					int arr[] = new int[productMDPs.size() + 1];
					arr[0] = -1;
					ArrayList<Integer> ssInds = ssIndices.get(ss);
					for (int i = 0; i < ssInds.size(); i++) {
						arr[i + 1] = ssInds.get(i);
					}
					jsToRobotState.put(jsIndex, arr.clone());
					jsSSIndices.put(ss, jsIndex);

				}
				jsIndex++;
			}
		}
		for (int i = 0; i < privateIndices.size(); i++) {

			State rs = robotStateStates.get(i);

			//now lets add these in succession 
			for (int j = 0; j < privateIndices.get(i).size(); j++) {
				int daInd = privateIndices.get(i).get(j);
				jointState.setValue(jsIndex, rs.varValues[daInd]);
				if (parentState == null) {
					int[] arr = new int[2];
					arr[0] = i;
					arr[1] = daInd;
					jsToRobotState.put(jsIndex, arr.clone());
				}
				jsIndex++;
			}

		}

		return jointState;

	}

	Queue<State> createJointPolicy(int numRobots, PrismLog mainLog, ArrayList<MDP> productMDPs, ArrayList<MDStrategy> nviStrategies, String saveplace,
			String filename, ArrayList<String> ssNames) throws PrismException
	{
		Queue<State> possibleReallocStates = new LinkedList<State>();

		MDPCreator mdpCreator = new MDPCreator(mainLog);

		ArrayList<ArrayList<Integer>> daIndices = new ArrayList<ArrayList<Integer>>();
		HashMap<String, ArrayList<Integer>> ssIndices = new HashMap<String, ArrayList<Integer>>();
		ArrayList<ArrayList<Integer>> privateIndices = new ArrayList<ArrayList<Integer>>();
		HashMap<Integer, int[]> jsToRobotState = new HashMap<Integer, int[]>();
		State parentState = null;
		HashMap<String, Integer> jsSSIndices = new HashMap<String, Integer>();
		//get the actions 
		Object[] actions = new Object[numRobots];

		//for each robot mdp get initial state 
		int[] currentStates = new int[numRobots];
		Queue<State> statesQueues = new LinkedList<State>();
		//= new LinkedList<int[]>();
		Queue<Double> statesProbQueue = new LinkedList<Double>();

		int numSS = 0;
		if (ssNames != null) {
			numSS = ssNames.size();
		}

		int numStateVars = doJointStateCreationSetup(numRobots, productMDPs, currentStates, numSS, ssNames, daIndices, ssIndices, privateIndices);

		State jointState = createJointState(numStateVars, productMDPs, currentStates, daIndices, ssIndices, privateIndices, jsToRobotState, parentState,
				jsSSIndices);
//		mainLog.println(jointState.toString());

		//break joint state into current states 
		//what i need to do 
		//need a map really for each robot 
		//would make things so much easier 

		statesQueues.add(jointState);
		statesProbQueue.add(1.0);

		ArrayList<State> visited = new ArrayList<State>();
		State currentJointState;
		try {
			while (!statesQueues.isEmpty()) {
				currentJointState = statesQueues.remove();
				if (!visited.contains(currentJointState))
					visited.add(currentJointState);
				else
					continue;
				parentState = currentJointState;
				currentStates = jointStatetoRobotStates(numRobots, currentJointState, jsToRobotState, productMDPs, mainLog);

				//createJointState(numStateVars, productMDPs, currentStates, numSS, ssNames, daIndices, ssIndices, privateIndices, null);

//				mainLog.println(jointState.toString());
				ArrayList<ArrayList<Integer>> robotStates = new ArrayList<ArrayList<Integer>>();
				ArrayList<ArrayList<Double>> robotStatesProbs = new ArrayList<ArrayList<Double>>();
				String infoString = "";

				for (int i = 0; i < numRobots; i++) {
					ArrayList<Integer> nextRobotStates = new ArrayList<Integer>();

					ArrayList<Double> nextRobotStatesProbs = new ArrayList<Double>();

					MDStrategy strat = nviStrategies.get(i);
					MDP mdp = productMDPs.get(i);

					int actionChoice = strat.getChoiceIndex(currentStates[i]);
					if (actionChoice > -1) {

						actions[i] = strat.getChoiceAction(currentStates[i]);

						infoString += i + ":" + currentStates[i] + mdp.getStatesList().get(currentStates[i]).toString() + "->" + actions[i].toString() + " ";
						//						//printing out the state 
						//						mainLog.println(mdp.getStatesList().get(currentStates[i]));
						//now how do we get the next action 
						Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(currentStates[i], actionChoice);
						//we need these cuz we have to use them 
						while (tranIter.hasNext()) {
							Entry<Integer, Double> currentIter = tranIter.next();
							nextRobotStates.add(currentIter.getKey());
							nextRobotStatesProbs.add(currentIter.getValue());

						}
					} else {
						actions[i] = "*";
					}

					robotStatesProbs.add(nextRobotStatesProbs);
					robotStates.add(nextRobotStates);
				}
//				mainLog.println(infoString);
				//put them all together in the array 
				ArrayList<Integer> robotStateNums = new ArrayList<Integer>();
				boolean allZero = true;
				for (int i = 0; i < robotStates.size(); i++) {
					if (robotStates.get(i).size() != 0)
						allZero = false;
					else {
						robotStates.get(i).add(currentStates[i]);
						robotStatesProbs.get(i).add(1.0);

					}
					robotStateNums.add(robotStates.get(i).size());
				}
				String ja = createJointAction(actions);
//				mainLog.println(ja);

				if (!allZero) {
					ArrayList<Entry<State, Double>> successorsWithProbs = new ArrayList<Entry<State, Double>>();

					ArrayList<int[]> combinationsList = generateCombinations(robotStateNums, mainLog);
					//now we need the stupid combination generator 
					//now we have to make these combinations and add them to our queue 
					for (int i = 0; i < combinationsList.size(); i++) {
						int[] currentCombination = combinationsList.get(i);
						int[] nextStatesCombo = new int[numRobots];
						double comboProb = 1;
						for (int j = 0; j < currentCombination.length; j++) {
							nextStatesCombo[j] = robotStates.get(j).get(currentCombination[j] - 1);
							comboProb *= robotStatesProbs.get(j).get(currentCombination[j] - 1);
						}

						State nextJointState = createJointState(numStateVars, productMDPs, nextStatesCombo, daIndices, ssIndices, privateIndices,
								jsToRobotState, parentState, jsSSIndices);

						statesQueues.add(nextJointState);
						//					statesProbQueue.add(comboProb);
						successorsWithProbs.add(new AbstractMap.SimpleEntry<State, Double>(nextJointState, comboProb));
					}
					mdpCreator.addAction(currentJointState, ja, successorsWithProbs);

				} else {
					//lets add these to a queue!!! 
					//add the current joint state to the possible realloc queue
					possibleReallocStates.add(currentJointState);
				}
			}
		} catch (PrismException e) {
			mdpCreator.saveMDP(saveplace + "results/", filename + "_jp");
			throw e;
		}
		mdpCreator.saveMDP(saveplace + "results/", filename + "_jp");

		return possibleReallocStates;

	}

	int[] jointStatetoRobotStates(int numRobots, State jointState, HashMap<Integer, int[]> jsToRobotState, ArrayList<MDP> productMDPs, PrismLog mainLog)
			throws PrismException
	{
		int[] currentStates = new int[numRobots];
		ArrayList<State> jsToRobotStateStates = new ArrayList<State>();
		for (int key : jsToRobotState.keySet()) {
			int[] rIndCombo = jsToRobotState.get(key);
			int rnum = rIndCombo[0];
			int indVal = rIndCombo[1];
			//say -1 is for an ss 
			if (rnum > -1) {
				//so jstorobotstate gives us the index 
				//of the index in the robot state 

				while (jsToRobotStateStates.size() <= rnum)
					jsToRobotStateStates.add(new State(productMDPs.get(rnum).getVarList().getNumVars()));
				State s = jsToRobotStateStates.get(rnum);
				s.setValue(indVal, jointState.varValues[key]);
			} else {
				//its a shared state 
				for (int r = 0; r < rIndCombo.length - 1; r++) {
					indVal = rIndCombo[r + 1];
					while (jsToRobotStateStates.size() <= r)
						jsToRobotStateStates.add(new State(productMDPs.get(rnum).getVarList().getNumVars()));
					State s = jsToRobotStateStates.get(r);
					s.setValue(indVal, jointState.varValues[key]);
				}
			}
		}
		for (int r = 0; r < numRobots; r++) {

			State rs = jsToRobotStateStates.get(r);
			//			mainLog.println(rs.toString());
			//we need to find the corresponding state 
			MDP mdp = productMDPs.get(r);
			int matchingS = findMatchingStateNum(mdp, rs);
			currentStates[r] = matchingS;
//			mainLog.println(matchingS);

		}
		return currentStates;
	}

	int findMatchingStateNum(MDP mdp, State rs) throws PrismException
	{

		int snum = -1;
		List<State> sl = mdp.getStatesList();
		//now just go over all the states and find the one that matches 
		for (int s = 0; s < sl.size(); s++) {
			if (rs.compareTo(sl.get(s)) == 0) {
				snum = s;
				break;
			}
		}
		if (snum == -1)
			throw new PrismException("Unable to find matching state in states list for " + rs.toString());
		return snum;

	}

	String createJointAction(Object[] actions)
	{
		String sep = ",";
		String ja = "";
		for (int i = 0; i < actions.length; i++) {
			ja += actions[i].toString();
			if (i != actions.length - 1)
				ja += sep;

		}
		return ja;
	}

	ArrayList<int[]> generateCombinations(ArrayList<Integer> numItemsPerGroup, PrismLog mainLog) throws PrismException
	{
		ArrayList<int[]> res = new ArrayList<int[]>();
		int[] counter = new int[numItemsPerGroup.size()];
		for (int i = 0; i < counter.length; i++)
			counter[i] = numItemsPerGroup.get(i);
		int[] original = counter.clone();
		int numP = generateCombinations(counter, 0, original.length - 1, original, 0, res);
		int estimatedC = getNumberOfCombinations(original);
		if (res.size() != estimatedC) {
			mainLog.println("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
			throw new PrismException("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
		}
		return res;
	}

	void generateCombinations(int counter[], int original[], ArrayList<int[]> res, PrismLog mainLog) throws PrismException
	{
		int numP = generateCombinations(counter, 0, original.length - 1, original, 0, res);
		int estimatedC = getNumberOfCombinations(original);
		if (res.size() != estimatedC) {
			mainLog.println("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
			throw new PrismException("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
		}
	}

	int getNumberOfCombinations(int[] arr)
	{
		int num = 1;
		for (int i = 0; i < arr.length; i++)
			num *= arr[i];
		return num;
	}

	int generateCombinations(int[] arr, int start, int end, int[] orig, int numC, ArrayList<int[]> res)
	{
		if (start == end) {
			while (arr[start] != 0) {

				res.add(arr.clone());
				arr[start]--;
				numC++;
			}
			arr[start] = orig[start];
		} else {
			while (arr[start] != 0) {
				numC = generateCombinations(arr, start + 1, end, orig, numC, res);
				arr[start]--;
			}
			arr[start] = orig[start];
		}
		return numC;
	}

	public static void main(String[] args)
	{

		new SSIAuction().run();
	}
}
