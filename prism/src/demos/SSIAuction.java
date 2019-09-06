package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map.Entry;
import java.util.Queue;

import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.ModelCheckerResult;
import explicit.StateValues;
import parser.State;
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
			StateValues nvicosts = mc.checkPartialSatExposed(agentMDP, currentTask, rewardExpr, null);
			double costInInitState = nvicosts.getDoubleArray()[agentMDP.getFirstInitialState()];
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

	public void run()
	{
		try {
			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
			String filename = "g7x3_r2_t3_d0_fs1";//"robot";
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

			//do things to the properties 

			ArrayList<Expression> robotsTasks = new ArrayList<Expression>();
			for (int i = 0; i < numRobots; i++)
				robotsTasks.add(null);
			Entry<ExpressionReward, ArrayList<Expression>> processedProperties = processProperties(propertiesFile, mainLog);
			ExpressionReward rewExpr = processedProperties.getKey();

			ArrayList<Expression> taskSet = processedProperties.getValue();

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

			//print task distribution and get strategy 
			ArrayList<MDStrategy> nviStrategies = new ArrayList<MDStrategy>();
			ArrayList<MDP> productMDPs = new ArrayList<MDP>();
			mainLog.println("Assigned Tasks");
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
			createJointPolicy(numRobots,mainLog, productMDPs,nviStrategies);

		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	void createJointPolicy(int numRobots, PrismLog mainLog, ArrayList<MDP> productMDPs, ArrayList<MDStrategy> nviStrategies) throws PrismException
	{
		//get the actions 
		Object[] actions = new Object[numRobots];

		//for each robot mdp get initial state 
		int[] currentStates = new int[numRobots];
		Queue<int[]> statesQueues = new LinkedList<int[]>();
		Queue<Double> statesProbQueue = new LinkedList<Double>();
		int numStateVars = 0; 
		int numSS = 0; 
	
		for (int i = 0; i < numRobots; i++) {

			MDP mdp = productMDPs.get(i);

			currentStates[i] = mdp.getFirstInitialState();
			numStateVars +=(mdp.getVarList().getNumVars()-numSS); 

			//		
		}
		numStateVars += numSS;
		State jointState = new State(numStateVars);
		
		//now we create the joint state 
		for(int i = 0; i<currentStates.length; i++)
		{
			//take the da bits 
			//take the 
		}
		
		statesQueues.add(currentStates.clone());
		statesProbQueue.add(1.0);

		while (!statesQueues.isEmpty()) {
			currentStates = statesQueues.remove();
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
				}
				robotStatesProbs.add(nextRobotStatesProbs);
				robotStates.add(nextRobotStates);
			}
			mainLog.println(infoString);
			//put them all together in the array 
			ArrayList<Integer> robotStateNums = new ArrayList<Integer>();
			boolean allZero = true;
			for (int i = 0; i < robotStates.size(); i++) {
				if (robotStates.get(i).size() != 0)
					allZero = false;
				else
					robotStates.get(i).add(currentStates[i]);
				robotStateNums.add(robotStates.get(i).size());
			}
			if (!allZero) {
				ArrayList<int[]> combinationsList = generateCombinations(robotStateNums, mainLog);
				//now we need the stupid combination generator 
				//now we have to make these combinations and add them to our queue 
				for (int i = 0; i < combinationsList.size(); i++) {
					int[] currentCombination = combinationsList.get(i);
					int[] nextStatesCombo = new int[numRobots];
					double comboProb = 1;
					for (int j = 0; j < currentCombination.length; j++) {
						nextStatesCombo[j] = robotStates.get(j).get(currentCombination[j] - 1);
						comboProb *= robotStatesProbs.get(i).get(currentCombination[j] - 1);
					}
					statesQueues.add(nextStatesCombo.clone());
					statesProbQueue.add(comboProb);
				}
			}
		}

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
