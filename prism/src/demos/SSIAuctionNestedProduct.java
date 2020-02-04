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
import java.util.PriorityQueue;

import acceptance.AcceptanceType;

import java.util.Queue;
import java.util.concurrent.TimeUnit;

import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.MDPSparse;
import explicit.Model;
import explicit.ModelCheckerMultipleResult;
import explicit.ModelCheckerResult;
import explicit.StateValues;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.DeclarationIntUnbounded;
import parser.ast.DeclarationType;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import parser.ast.RewardStruct;
import prism.Prism;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismLog;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;

public class SSIAuctionNestedProduct
{
	Queue<Entry<StateExtended, ArrayList<Expression>>> possibleReallocStates;
	MDPCreator jointPolicyCreator;
	private HashMap<Integer, int[]> jsToRobotState;
	private MDPCreator mdpCreator = null;
	public boolean doingReallocs = true;
	String fnPrefix = "ssi";
	HashMap<Expression, String> expressionLabels = null;
	private ArrayList<String> ssNames;
	public boolean debugSSI = false;
	public long totalTimeDuration = 0;

	public long firstSolDuration = 0;
	public long allReplanningDuration = 0;

	public ArrayList<MDPRewardsSimple> createMaxExpTaskRewStruct(SingleAgentNestedProductMDP saMDP, MDPRewardsSimple costsModel)
	{
		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>();

		MDP mdp = saMDP.finalProduct.getProductModel();

		int numStates = mdp.getNumStates();
		MDPRewardsSimple progressionRewards = new MDPRewardsSimple(numStates);
		MDPRewardsSimple costs = new MDPRewardsSimple(numStates);
		for (int s = 0; s < mdp.getNumStates(); s++) {
			int singleAgentState = saMDP.productStateToMDPState.get(s);
			double rewardHere = 0;
			if (costsModel != null)
				rewardHere = costsModel.getStateReward(singleAgentState);
			costs.addToStateReward(s, rewardHere);
			//for each state go over all the other states 
			int numChoices = mdp.getNumChoices(s);
			for (int c = 0; c < numChoices; c++) {

				rewardHere = 0;
				if (costsModel != null)
					rewardHere = costsModel.getTransitionReward(singleAgentState, c);
				costs.addToTransitionReward(s, c, rewardHere);

				double choiceRew = 0;
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, c);
				while (tranIter.hasNext()) {
					Entry<Integer, Double> currentPair = tranIter.next();
					int nextS = currentPair.getKey();
					double prob = currentPair.getValue();
					boolean[] essAcc = saMDP.addRewardForTaskCompletion(nextS, s);
					if (essAcc[0]) {
						choiceRew += prob;
					}
				}
				progressionRewards.addToTransitionReward(s, c, choiceRew);

			}

		}
		rewards.add(progressionRewards);
		rewards.add(costs);
		return rewards;
	}

	//copied from STAPU 
	protected ArrayList<DAInfo> initializeDAInfoFromLTLExpressions(ArrayList<Expression> exprs, PrismLog mainLog)
	{
		int numExprs = exprs.size();
		ArrayList<DAInfo> daInfoList = new ArrayList<DAInfo>(numExprs);

		for (int daNum = 0; daNum < numExprs; daNum++) {
			boolean hasReward = exprs.get(daNum) instanceof ExpressionReward;
			Expression thisExpr;
			if (hasReward)
				thisExpr = exprs.get(daNum);
			else {
				if (exprs.get(daNum) instanceof ExpressionQuant)
					thisExpr = ((ExpressionQuant) exprs.get(daNum)).getExpression();
				else
					thisExpr = exprs.get(daNum);
			}
			DAInfo daInfo = new DAInfo(mainLog, thisExpr, hasReward);
			daInfoList.add(daInfo);
		}

		return daInfoList;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDPModelChecker mc, MDP mdp, BitSet target, BitSet statesToAvoid,
			ArrayList<MDPRewardsSimple> rewards, ArrayList<Boolean> minRewards, int probPreference, Prism prism, PrismLog mL) throws PrismException
	{

		ModelCheckerMultipleResult res2 = computeNestedValIterFailurePrint(mc, mdp, target, statesToAvoid, rewards, minRewards, probPreference, null, prism,
				mL);// computeNestedValIterFailurePrint(mdp, target, statesToAvoid,
		// rewards,minRewards,target,probPreference,null);

		return res2;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDPModelChecker mc, MDP mdp, BitSet target, BitSet statesToAvoid,
			ArrayList<MDPRewardsSimple> rewards, ArrayList<Boolean> minRewards, int probPreference, double[] probInitVal, Prism prismC, PrismLog mainLog)
			throws PrismException
	{

		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		if (mc == null) {
			mc = new MDPModelChecker(prismC);

		}
		statesToAvoid.or(target);
		if (mc.getGenStrat() == false)
			mc.setGenStrat(true);
		ModelCheckerMultipleResult res2 = mc.computeNestedValIterArray(mdp, target, statesToRemainIn, rewards, null, minRewards, statesToAvoid, probPreference,
				probInitVal);
		getModelCheckerMultipleResultInInitState(res2, (MDPSimple) mdp, mainLog);

		return res2;
	}

	protected double[] getModelCheckerMultipleResultInInitState(ModelCheckerMultipleResult res2, MDPSimple mdp, PrismLog mainLog)
	{
		double[] resultVals = resultValues(res2, mdp, null);
		int mdpInitState = mdp.getFirstInitialState();
		mainLog.println("\nFor state " + mdpInitState + " p = " + resultVals[0] + ", max exp tasks completed: " + resultVals[1] + ", min sum of costs: "
				+ resultVals[2]);
//		System.out.println("\nFor state " + mdpInitState + " p = " + resultVals[0] + ", max exp tasks completed: " + resultVals[1] + ", min sum of costs: "
//				+ resultVals[2]);
		return resultVals;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDPModelChecker mc, MDP mdp, BitSet target, BitSet statesToAvoid,
			ArrayList<MDPRewardsSimple> rewards, int probPreference, boolean doMaxTasks, Prism p, PrismLog m) throws PrismException
	{

		ArrayList<Boolean> minMaxRew = new ArrayList<Boolean>();
		int rewinit = 0;
		if (doMaxTasks) {
			minMaxRew.add(false);
			rewinit++;
		}
		for (int rew = rewinit; rew < rewards.size(); rew++)
			minMaxRew.add(true);
		return computeNestedValIterFailurePrint(mc, mdp, target, statesToAvoid, rewards, minMaxRew, probPreference, p, m);
	}

	public Entry<Entry<Integer, double[]>, Entry<SingleAgentNestedProductMDP, ModelCheckerMultipleResult>> getSingleAgentBidNestedProductIncremental(
			SingleAgentNestedProductMDP prev, MDP agentMDP, ArrayList<Expression> taskSet, ArrayList<Expression> agentTasks, ExpressionReward rewardExpr,
			MDPModelChecker mc, double[] sumSoFar, MDPRewardsSimple costsModel, PrismLog mainLog, Prism prism, String saveplace, String filename)
			throws PrismException
	{

		Expression bidPick = null;
		double[] bidValue = new double[] { 0, 0, 10000 };

		int indexOfChosenTask = -1;
		Entry<SingleAgentNestedProductMDP, ModelCheckerMultipleResult> chosenNPSol = null;
		for (int exprNum = 0; exprNum < taskSet.size(); exprNum++) {
			ArrayList<Expression> currentAgentTasks = new ArrayList<Expression>();
			if (prev == null)
				currentAgentTasks.addAll(agentTasks);

			currentAgentTasks.add(0, getInnerExpression(taskSet.get(exprNum)));

			Entry<SingleAgentNestedProductMDP, ModelCheckerMultipleResult> npSol = getSingleAgentSolNVINestedProductIncremental(prev, currentAgentTasks,
					rewardExpr, mainLog, mc, (MDPSimple) agentMDP, costsModel, saveplace, filename, prism);
			double[] nvicosts = resultValues(npSol.getValue(), (MDPSimple) npSol.getKey().finalProduct.getProductModel(), null);
			double[] nvicostsinc = nvicosts.clone();

			nvicostsinc[1] -= sumSoFar[0];
			nvicostsinc[2] -= sumSoFar[1];

			//			nvicosts[2] +=sumOfCosts;
			if (progCostIsBetter(nvicostsinc, bidValue)) {
				bidPick = taskSet.get(exprNum);
				bidValue = nvicostsinc.clone();
				indexOfChosenTask = exprNum;
				chosenNPSol = npSol;
			}

		}
		//		bidValue[2]-=sumOfCosts;
		SimpleEntry<Integer, double[]> bidBits = new AbstractMap.SimpleEntry<Integer, double[]>(indexOfChosenTask, bidValue);
		return new AbstractMap.SimpleEntry<Entry<Integer, double[]>, Entry<SingleAgentNestedProductMDP, ModelCheckerMultipleResult>>(bidBits, chosenNPSol);

	}

	Entry<SingleAgentNestedProductMDP, ModelCheckerMultipleResult> getSingleAgentSolNVINestedProductIncremental(SingleAgentNestedProductMDP prev,
			ArrayList<Expression> robotsTasksBroken, ExpressionReward rewExpr, PrismLog mainLog, MDPModelChecker mc, MDPSimple mdp, MDPRewardsSimple costsModel,
			String saveplace, String filename, Prism prism) throws PrismException
	{

		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(robotsTasksBroken, mainLog);

		SingleAgentNestedProductMDP res = buildSingleAgentNestedProductMDPIncremental(prev, "r", mdp, daList, null, prism, mc, mainLog);

		ArrayList<MDPRewardsSimple> rewards = createMaxExpTaskRewStruct(res, costsModel);

		//		new MDPCreator().saveMDP(res.finalProduct.getProductModel(), saveplace + "results/" + fnPrefix, filename + "_prod_" + 0, res.combinedAcceptingStates);
		//		mainLog.println(res.numMDPVars);
		ModelCheckerMultipleResult nviSol = computeNestedValIterFailurePrint(mc, res.finalProduct.getProductModel(), res.combinedAcceptingStates,
				res.combinedStatesToAvoid, rewards, 0, true, prism, mainLog);
		double[] resVals = resultValues(nviSol, (MDPSimple) res.finalProduct.getProductModel(), null);
		return new AbstractMap.SimpleEntry<SingleAgentNestedProductMDP, ModelCheckerMultipleResult>(res, nviSol);//resultValues(nviSol, (MDPSimple) res.finalProduct.getProductModel());

	}

	boolean progCostIsBetter(double[] newValues, double[] oldValues)
	{
		boolean doUpdate = false;
		if (newValues[1] > oldValues[1]) {
			doUpdate = true;
		} else {
			if (newValues[1] == oldValues[1]) {
				if (newValues[2] < oldValues[2]) {
					doUpdate = true;
				}
			}
		}

		return doUpdate;
	}

	public Entry<ExpressionReward, Entry<Expression, ArrayList<Expression>>> processProperties(PropertiesFile propertiesFile, PrismLog mainLog, int numGoals,
			ArrayList<Integer> goalNumbers)
	{
		expressionLabels = new HashMap<Expression, String>();

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
			//just initialise the hashmap here 

		}
		Expression safetyExpr = ltlExpressions.get(numOp - 1);
		ArrayList<Expression> taskSet = new ArrayList<Expression>();

		int lastExprNum = 0;
		if (goalNumbers == null) {
			for (int exprNum = 0; exprNum < numGoals - 1; exprNum++) {

				Expression currentExpr = ltlExpressions.get(exprNum);
				//			Expression currentExprWithSafetyExpr = Expression.And(currentExpr, safetyExpr);
				taskSet.add(currentExpr);

				expressionLabels.put(getInnerExpression(ltlExpressions.get(exprNum)), "da" + exprNum);
				lastExprNum = exprNum;
			}
			lastExprNum++;
		} else {

			for (int exprNum : goalNumbers) {
				Expression currentExpr = ltlExpressions.get(exprNum);
				//			Expression currentExprWithSafetyExpr = Expression.And(currentExpr, safetyExpr);
				taskSet.add(currentExpr);

				expressionLabels.put(getInnerExpression(ltlExpressions.get(exprNum)), "da" + lastExprNum);
				lastExprNum++;
			}
		}
		//		lastExprNum++;
		expressionLabels.put(Expression.Not(getInnerExpression(safetyExpr)), "da" + lastExprNum);

		//		taskSet.add(safetyExpr);
		return new AbstractMap.SimpleEntry<ExpressionReward, Entry<Expression, ArrayList<Expression>>>(rewExpr,
				new SimpleEntry<Expression, ArrayList<Expression>>(safetyExpr, taskSet));
	}

	public Expression getInnerExpression(Expression expr)
	{
		return ((ExpressionQuant) expr).getExpression();
	}

	public Entry<ArrayList<ArrayList<Expression>>, Entry<ArrayList<SingleAgentNestedProductMDP>, ArrayList<ModelCheckerMultipleResult>>> auctionTasksSumOfCostsNestedProductIncremental(
			ArrayList<Expression> taskSet, int numRobots, ArrayList<MDPSimple> mdps, ExpressionReward rewExpr, Expression safetyExpr,
			ArrayList<MDPModelChecker> mcs, ArrayList<MDPRewardsSimple> costsModels, Prism prism, String saveplace, String filename, PrismLog mainLog,
			PrismLog fileLog) throws PrismException
	{

		////profile
		long startTime = System.currentTimeMillis();

		long maxBidDuration = 0;

		//so basically we start with the mdps 
		//then we auction one task using the nested product thing 
		//then we reuse that mdp for the next task 
		//got it ? 

		//		ArrayList<Expression> robotsTasks = new ArrayList<Expression>();
		ArrayList<ArrayList<Expression>> robotsTasksBroken = new ArrayList<ArrayList<Expression>>();
		ArrayList<SingleAgentNestedProductMDP> prevNPs = new ArrayList<SingleAgentNestedProductMDP>();
		ArrayList<ModelCheckerMultipleResult> results = new ArrayList<ModelCheckerMultipleResult>();
		double[][] previousCost = new double[numRobots][2];

		//so what we've got to do here is 
		//we got to mix the single agent bid with the sum of costs 
		//so we have this current sum of costs 
		//then we get the bid for one agent -
		//if its less than the current sum, that agent gets the bid 
		// so technically we're merging the whole process 

		for (int i = 0; i < numRobots; i++) {
			//			robotsTasks.add(getInnerExpression(safetyExpr));
			robotsTasksBroken.add(new ArrayList<Expression>());
			robotsTasksBroken.get(i).add(getInnerExpression(safetyExpr));
			previousCost[i][0] = 0;
			previousCost[i][1] = 0;
			prevNPs.add(null);
			results.add(null);
		}

		int auctionRound = 0;
		//TODO:your code here 

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		totalTimeDuration += runTime;
		fileLog.println("Auction Initialization: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

		while (!taskSet.isEmpty()) {
			////profile
			long startTimex = System.currentTimeMillis();

			int bestBidIndex = -1;
			double[] bestBidValues = new double[] { 0, 0, 10000 };
			Expression bestBidExpression = null;
			int bestBidRobotIndex = -1;
			double[] bestBidValuesFull = new double[] { 0, 0, 10000 };
			long maxBidCalculationForTask = 0;
			SingleAgentNestedProductMDP bestNP = null;
			ModelCheckerMultipleResult bestNPSol = null;

			//TODO:your code here 

			long stopTimex = System.currentTimeMillis();
			long runTimex = stopTimex - startTimex;
			totalTimeDuration += runTimex;
			fileLog.println("Auction Initialization: " + getTimeString(runTimex));
			fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

			for (int i = 0; i < numRobots; i++) {
				startTimex = System.currentTimeMillis();
				Entry<Entry<Integer, double[]>, Entry<SingleAgentNestedProductMDP, ModelCheckerMultipleResult>> returnStuff = getSingleAgentBidNestedProductIncremental(
						prevNPs.get(i), mdps.get(i), taskSet, robotsTasksBroken.get(i), rewExpr, mcs.get(i), previousCost[i], costsModels.get(i), mainLog,
						prism, saveplace, filename);
				Entry<Integer, double[]> bid = returnStuff.getKey();

				int bidTaskIndex = bid.getKey();
				double[] bidValues = bid.getValue();

				if (progCostIsBetter(bidValues, bestBidValues)) {
					bestBidValues = bidValues;
					bestBidIndex = bidTaskIndex;
					bestBidRobotIndex = i;
					bestBidExpression = getInnerExpression(taskSet.get(bestBidIndex));
					bestNP = returnStuff.getValue().getKey();
					bestNPSol = returnStuff.getValue().getValue();

				}
				stopTimex = System.currentTimeMillis();
				runTimex = stopTimex - startTimex;
				if (maxBidDuration < runTimex)
					maxBidDuration = runTimex;
				fileLog.println("Robot " + i + " bid: " + getTimeString(runTimex));
				fileLog.println("Max Bid Time" + getTimeString(maxBidDuration));
				fileLog.println("XXX,A1,"+System.currentTimeMillis());

			}
			totalTimeDuration += maxBidDuration;
			startTimex = System.currentTimeMillis();
			previousCost[bestBidRobotIndex][0] += bestBidValues[1];
			previousCost[bestBidRobotIndex][1] += bestBidValues[2];

			robotsTasksBroken.get(bestBidRobotIndex).add(bestBidExpression);
			prevNPs.set(bestBidRobotIndex, bestNP);
			results.set(bestBidRobotIndex, bestNPSol);

			taskSet.remove(bestBidIndex);

			auctionRound++;
			stopTimex = System.currentTimeMillis();
			runTimex = stopTimex - startTimex;
			totalTimeDuration += runTimex;
			fileLog.println("Auction cleanup: " + getTimeString(runTimex));
			fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
			fileLog.println("XXX,A2,"+System.currentTimeMillis());

		}
		startTime = System.currentTimeMillis();
		SimpleEntry<ArrayList<SingleAgentNestedProductMDP>, ArrayList<ModelCheckerMultipleResult>> stuff = new AbstractMap.SimpleEntry<ArrayList<SingleAgentNestedProductMDP>, ArrayList<ModelCheckerMultipleResult>>(
				prevNPs, results);
		SimpleEntry<ArrayList<ArrayList<Expression>>, Entry<ArrayList<SingleAgentNestedProductMDP>, ArrayList<ModelCheckerMultipleResult>>> toret = new AbstractMap.SimpleEntry<ArrayList<ArrayList<Expression>>, Entry<ArrayList<SingleAgentNestedProductMDP>, ArrayList<ModelCheckerMultipleResult>>>(
				robotsTasksBroken, stuff);
		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		totalTimeDuration += runTime;
		fileLog.println("Auction cleanup: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
		fileLog.println("XXX,A3,"+System.currentTimeMillis());
		return toret;//robotsTasksBroken;
	}

	public PropertiesFile loadFiles(Prism prism, String saveplace, String filename, int numRobots, ArrayList<MDPSimple> mdps, ArrayList<MDPModelChecker> mcs,
			ArrayList<Integer> robotNumbers) throws FileNotFoundException, PrismException
	{
		PropertiesFile propertiesFile = null;
		//load the files 
		//could be a separate function 
		int fnNumber = 0;
		for (int i = 0; i < numRobots; i++) {
			if (robotNumbers == null)
				fnNumber = i;
			else
				fnNumber = robotNumbers.get(i);
			String modelFileName = saveplace + filename + fnNumber + ".prism";
			ModulesFile modulesFile = prism.parseModelFile(new File(modelFileName));
			prism.loadPRISMModel(modulesFile);
			propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + ".prop"));
			prism.buildModel();
			MDPSimple mdp = new MDPSimple((MDPSparse) prism.getBuiltModelExplicit());
			mdps.add(mdp);
			MDPModelChecker mc = new MDPModelChecker(prism);
			mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));
			mcs.add(mc);
		}
		return propertiesFile;
	}

	/**
	 * @param model
	 *            - the mdp model
	 * 
	 * @param exprs
	 *            - array list of expressions
	 * 
	 * @param statesOfInterest
	 *            - states to care about - we care about everything so we don't
	 *            really need this
	 * 
	 */
	protected SingleAgentNestedProductMDP buildSingleAgentNestedProductMDPIncremental(SingleAgentNestedProductMDP prev, String name, Model model,
			ArrayList<DAInfo> daList, BitSet statesOfInterest, Prism prism, MDPModelChecker mc, PrismLog mainLog) throws PrismException
	{
		// return the list of daInfo and the product mdp
		LTLModelChecker mcLTL = new LTLModelChecker(prism); // is this okay ?

		MDP productMDP = null;
		SingleAgentNestedProductMDP res = null;
		DAInfo lastResdaInfo = null;
		int lastResdaAssociatedIndex = -1;
		int maxResDAs = 0;

		if (prev == null) {
			res = new SingleAgentNestedProductMDP(mainLog);
			res.setNumMDPVars(model.getVarList().getNumVars());
			res.initializeProductToMDPStateMapping((MDP) model);
			res.daList = new ArrayList<DAInfo>();
			productMDP = (MDP) model;
		} else {
			//otherwise copy it 
			//cuz warna maslay hotay hain 
			//			throw new PrismException("Copy nested product please");
			res = new SingleAgentNestedProductMDP(prev, mcLTL);
			productMDP = res.finalProduct.getProductModel();
			lastResdaInfo = res.daList.get(res.daList.size() - 1);
			lastResdaAssociatedIndex = lastResdaInfo.associatedIndexInProduct;
			maxResDAs = res.daList.size();
		}
		LTLProduct<MDP> product = null;

		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };

		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		// all the states are states of interest
		bsInit.set(0, numStates);

		for (int daNum = 0; daNum < daList.size(); daNum++) {
			DAInfo daInfo = new DAInfo(daList.get(daNum));
			//			if (lastResdaAssociatedIndex != -1 && daInfo.associatedIndexInProduct == -1)
			//				daInfo.associatedIndexInProduct = lastResdaAssociatedIndex + 1;
			//			else {
			daInfo.associatedIndexInProduct++; //should go to zero from -1 
			//			}

			product = daInfo.constructDAandProductModel(mcLTL, mc, allowedAcceptance, productMDP, null, true);
			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);

			// update state numbers
			for (int otherDAs = 0; otherDAs < (daNum + maxResDAs); otherDAs++) {
				res.daList.get(otherDAs).updateStateNumbers(product);
				res.daList.get(otherDAs).associatedIndexInProduct++; //and everyone else also gets shifted once. 

			}
			res.updateProductToMDPStateMapping(product);
			res.daList.add(daInfo);
		}

		res.setDAListAndFinalProduct(product);
		return res;
	}

	public double[] run()
	{
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";

		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";

		boolean reallocOnFirstRobotDeadend = false;
		PrismLog fileLog = new PrismDevNullLog();
		double[] res = run(saveplace, fn, numRobots, numGoals, numDoors, null, null, reallocOnFirstRobotDeadend, fileLog, null);
		fileLog.close();
		return res;
	}

	VarList createJointVarList(ArrayList<String> ssNames, ArrayList<MDPSimple> mdps, ArrayList<Integer> daIndices) throws PrismLangException
	{
		//create the varlist here 
		//cuz this is where we're fixing things okay 
		//okay yeah 
		//cool :P 
		//now we take the da list 
		VarList jvl = new VarList();
		int varC = 0;
		//well really its just everything in the hasmap 
		for (Expression expr : expressionLabels.keySet()) {
			String name = expressionLabels.get(expr);
			jvl.addVar(varC, new Declaration(name, new DeclarationIntUnbounded()), 1, null);
			daIndices.add(varC);
			varC++;
		}
		//for each shared var we need to do stuff 
		for (String ss : ssNames) {
			String name = ss;
			jvl.addVar(varC, new Declaration(name, new DeclarationIntUnbounded()), 1, null);
			varC++;
		}
		//now just the private states 
		//which are the states in the mdps that are not in ss 
		for (int i = 0; i < mdps.size(); i++) {
			VarList svl = mdps.get(i).getVarList();
			for (int j = 0; j < svl.getNumVars(); j++) {
				String name = svl.getName(j);
				if (!ssNames.contains(name)) {
					//					Declaration decl = svl.getDeclaration(j);

					//					decl.setName(decl.getName() + "_r" + i);
					jvl.addVar(varC++, new Declaration(name + "_r" + i, new DeclarationIntUnbounded()), 1, null);
				}
			}
		}
		return jvl;
	}

	HashMap<Integer, HashMap<Integer, Integer>> mapJointVarListToRobotVarList(ArrayList<String> ssNames, ArrayList<ArrayList<DAInfo>> finalDAList, VarList jvl,
			ArrayList<MDP> productMDPs, HashMap<Integer, DAInfo> jvlDAMap, ArrayList<MDPSimple> originalMDPs
	//			ArrayList<HashMap<Integer,Integer>> svlTojvl

	) throws PrismException
	{
		//so like now we do all the book keeping 
		//i'm just going to repeat this create joint policy thing again 
		//cuz i'm tired 
		//so now we've got to match the da stuff 
		//this is super important okay 
		//and we've got to match the ss stuff 
		//and we've got to match the individual state stuff 
		//lets do this shit 
		//let us begin my friend 
		//we know that the expression's name is what is in our label 
		//so for the da bit for each robot, we just the finaldalist 
		HashMap<Integer, HashMap<Integer, Integer>> jvlTosvl = new HashMap<Integer, HashMap<Integer, Integer>>();

		for (int i = 0; i < finalDAList.size(); i++) {
			//			svlTojvl.add(new HashMap<Integer,Integer>()); 

			ArrayList<DAInfo> robotDAList = finalDAList.get(i);
			if (robotDAList != null) {
				VarList svl = productMDPs.get(i).getVarList();
				//so now we've got the dainfo thingi 
				for (int j = 0; j < robotDAList.size(); j++) {
					DAInfo daInfo = robotDAList.get(j);
					Expression expr = daInfo.daExpr;
					int nestedProductIndex = daInfo.associatedIndexInProduct;
					String jvlName = expressionLabels.get(expr);
					int jvlIndex = jvl.getIndex(jvlName);
					if (!jvlTosvl.containsKey(jvlIndex)) {
						jvlTosvl.put(jvlIndex, new HashMap<Integer, Integer>());
					}
					jvlTosvl.get(jvlIndex).put(i, nestedProductIndex);
					if (jvlDAMap != null)
						jvlDAMap.put(jvlIndex, daInfo);
					//				svlTojvl.get(i).put(nestedProductIndex,jvlIndex);

				}

				//we've got to do the same thing for shared variables 
				for (int j = 0; j < ssNames.size(); j++) {
					String svlName = ssNames.get(j);
					int svlIndex = svl.getIndex(svlName);
					int jvlIndex = jvl.getIndex(svlName);
					if (svlIndex == -1 | jvlIndex == -1)
						throw new PrismException("Shared state does not exist");
					if (!jvlTosvl.containsKey(jvlIndex)) {
						jvlTosvl.put(jvlIndex, new HashMap<Integer, Integer>());
					}
					jvlTosvl.get(jvlIndex).put(i, svlIndex);
					//				svlTojvl.get(i).put(svlIndex,jvlIndex);

				}
				//lastly for the individual variables 
				//so like we know they're at the end 
				//so i'm just going to start from the end 
				for (int j = svl.getNumVars() - 1; j >= 0; j--) {
					String svlName = svl.getName(j);
					if (ssNames.contains(svlName))
						continue;
					if (svlName.contains("da"))
						continue;
					String jvlName = svlName + "_r" + i;
					int jvlIndex = jvl.getIndex(jvlName);
					int svlIndex = j;
					if (svlIndex == -1 | jvlIndex == -1)
						throw new PrismException("Private state does not exist - " + svlName + " " + jvlName + "," + svlIndex + " " + jvlIndex);
					if (!jvlTosvl.containsKey(jvlIndex)) {
						jvlTosvl.put(jvlIndex, new HashMap<Integer, Integer>());
					}
					jvlTosvl.get(jvlIndex).put(i, svlIndex);
					//				svlTojvl.get(i).put(svlIndex,jvlIndex);
				}
			} else {
				//if it is null then what do we do ?
				if (originalMDPs != null) {
					VarList svl = originalMDPs.get(i).getVarList();
					//we've got to do the same thing for shared variables 
					for (int j = 0; j < ssNames.size(); j++) {
						String svlName = ssNames.get(j);
						int svlIndex = svl.getIndex(svlName);
						int jvlIndex = jvl.getIndex(svlName);
						if (svlIndex == -1 | jvlIndex == -1)
							throw new PrismException("Shared state does not exist");
						if (!jvlTosvl.containsKey(jvlIndex)) {
							jvlTosvl.put(jvlIndex, new HashMap<Integer, Integer>());
						}
						jvlTosvl.get(jvlIndex).put(i, svlIndex);
						//				svlTojvl.get(i).put(svlIndex,jvlIndex);

					}
					//lastly for the individual variables 
					//so like we know they're at the end 
					//so i'm just going to start from the end 
					for (int j = svl.getNumVars() - 1; j >= 0; j--) {
						String svlName = svl.getName(j);
						if (ssNames.contains(svlName))
							continue;
						if (svlName.contains("da"))
							continue;
						String jvlName = svlName + "_r" + i;
						int jvlIndex = jvl.getIndex(jvlName);
						int svlIndex = j;
						if (svlIndex == -1 | jvlIndex == -1)
							throw new PrismException("Private state does not exist - " + svlName + " " + jvlName + "," + svlIndex + " " + jvlIndex);
						if (!jvlTosvl.containsKey(jvlIndex)) {
							jvlTosvl.put(jvlIndex, new HashMap<Integer, Integer>());
						}
						jvlTosvl.get(jvlIndex).put(i, svlIndex);
						//				svlTojvl.get(i).put(svlIndex,jvlIndex);
					}
				}
			}
		}
		return jvlTosvl;
	}

	public String getTimeString(long time)
	{
		String timeString = time + "ms" + "(" + TimeUnit.SECONDS.convert(time, TimeUnit.MILLISECONDS) + "s)";
		return timeString;
	}

	public double[] run(String saveplace, String fn, int numRobots, int numGoals, int numDoors, ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers,
			boolean stopReallocationWhenAnyRobotDeadends, PrismLog fileLog, String mainLogFile)
	{
		fileLog.println("XXX,-A,"+System.currentTimeMillis());
		////profile
		long startTime = System.currentTimeMillis();
		long stopTime;
		long runTime;
		double[] results = null;

		ArrayList<double[]> planningValuesSSI = null;
		ArrayList<double[]> planningValuesJP = null;
		if (debugSSI) {
			planningValuesSSI = new ArrayList<double[]>();
			planningValuesJP = new ArrayList<double[]>();
		}

		fnPrefix += "r" + numRobots + "_g" + numGoals + "d" + numDoors;

		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;

		totalTimeDuration += runTime;
		fileLog.println("Initialization: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

		try {
			
			startTime = System.currentTimeMillis();
			String filename = fn;
			ssNames = new ArrayList<String>();

			for (int i = 0; i < numDoors; i++)
				ssNames.add("door" + i);

			//			boolean stopReallocationWhenAnyRobotDeadends = true;//false;
			// Create a log for PRISM output (hidden or stdout)
			//PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");
			if (mainLogFile != null) {
				mainLog = new PrismFileLog(mainLogFile);
			}

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);

			prism.initialise();
			prism.setEngine(Prism.EXPLICIT);
			Queue<StateExtended> reallocStatesPQ = new PriorityQueue<StateExtended>();
			HashMap<State, Integer> reallocStatesMapToList = new HashMap<State, Integer>();

			ArrayList<MDPSimple> mdps = new ArrayList<MDPSimple>();
			ArrayList<MDPModelChecker> mcs = new ArrayList<MDPModelChecker>();
			PropertiesFile propertiesFile = loadFiles(prism, saveplace, filename, numRobots, mdps, mcs, robotNumbers);
			int[] mdpInitialStates = new int[mdps.size()];
			for (int i = 0; i < mdpInitialStates.length; i++) {
				mdpInitialStates[i] = mdps.get(i).getFirstInitialState();
			}
			//do things to the properties 
			Entry<ExpressionReward, Entry<Expression, ArrayList<Expression>>> processedProperties = processProperties(propertiesFile, mainLog, numGoals,
					goalNumbers);
			ExpressionReward rewExpr = processedProperties.getKey();
			Entry<Expression, ArrayList<Expression>> safetyExprAndList = processedProperties.getValue();
			Expression safetyExpr = safetyExprAndList.getKey();
			ArrayList<Expression> taskSet = safetyExprAndList.getValue();
			ArrayList<Expression> taskSetToEdit = new ArrayList<Expression>();
			taskSetToEdit.addAll(taskSet);

			ArrayList<MDPRewardsSimple> costsModels = new ArrayList<MDPRewardsSimple>();
			getCostsModels(numRobots, rewExpr, mcs, mdps, costsModels);

			stopTime = System.currentTimeMillis();
			runTime = stopTime - startTime;
			totalTimeDuration += runTime;
			fileLog.println("Initialization: " + getTimeString(runTime));
			fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

			Entry<ArrayList<ArrayList<Expression>>, Entry<ArrayList<SingleAgentNestedProductMDP>, ArrayList<ModelCheckerMultipleResult>>> meh = auctionTasksSumOfCostsNestedProductIncremental(
					taskSetToEdit, numRobots, mdps, rewExpr, safetyExpr, mcs, costsModels, prism, saveplace, filename, mainLog, fileLog);
			fileLog.println("XXX,B,"+System.currentTimeMillis());
			startTime = System.currentTimeMillis();

			ArrayList<ArrayList<Expression>> robotsTasksBroken = meh.getKey();

			ArrayList<SingleAgentNestedProductMDP> samdps = meh.getValue().getKey();
			ArrayList<ModelCheckerMultipleResult> nviSols = meh.getValue().getValue();

			if (rewExpr == null)
				throw new PrismException("No reward function");

			ArrayList<MDStrategy> nviStrategies = new ArrayList<MDStrategy>();
			ArrayList<MDP> productMDPs = new ArrayList<MDP>();
			ArrayList<ArrayList<DAInfo>> finalDAList = new ArrayList<ArrayList<DAInfo>>();
			ArrayList<MDPRewardsSimple> costRewards = new ArrayList<MDPRewardsSimple>();
			ArrayList<MDPRewardsSimple> expTaskRewards = new ArrayList<MDPRewardsSimple>();
			ArrayList<BitSet> singleAgentNPAccStates = new ArrayList<BitSet>();

			stopTime = System.currentTimeMillis();
			runTime = stopTime - startTime;

			totalTimeDuration += runTime;
			fileLog.println("Solution Prep: " + getTimeString(runTime));
			fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

			ArrayList<BitSet> statesToAvoid = getRobotPlansUsingNVINestedProductIncremental(numRobots, samdps, nviSols, costsModels, productMDPs, mainLog,
					costRewards, saveplace, filename, prism, nviStrategies, finalDAList, singleAgentNPAccStates, fileLog, expTaskRewards, planningValuesSSI);
			fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
			startTime = System.currentTimeMillis();
			fileLog.println("XXX,C,"+System.currentTimeMillis());
			if (debugSSI) {

				for (int i = 0; i < productMDPs.size(); i++) {
					if (productMDPs.get(i) != null) {
						PolicyCreator pc = new PolicyCreator();

						pc.createPolicyWithRewardsStructuresAsLabels(productMDPs.get(i).getFirstInitialState(), productMDPs.get(i), nviStrategies.get(i),
								expTaskRewards.get(i), costRewards.get(i), singleAgentNPAccStates.get(i));
						pc.savePolicy(saveplace + "results/" + fnPrefix, filename + "_0_init_ptRews" + i);
					}
				}
			}

			ArrayList<Integer> daIndices = new ArrayList<Integer>();

			VarList jvl = createJointVarList(ssNames, mdps, daIndices);

			mdpCreator = new MDPCreator(mainLog);
			mdpCreator.setVarList(jvl);
			//			ArrayList<HashMap<Integer,Integer>> svlTojvl = new ArrayList<HashMap<Integer,Integer>>(); 
			HashMap<Integer, DAInfo> jvlDAMap = new HashMap<Integer, DAInfo>();
			HashMap<Integer, HashMap<Integer, Integer>> jvlTosvl = mapJointVarListToRobotVarList(ssNames, finalDAList, jvl, productMDPs, jvlDAMap, mdps);

			double[] resultvalues = createJointPolicy(daIndices, jvlDAMap, jvlTosvl, costRewards, productMDPs, mainLog, nviStrategies, saveplace, filename,
					ssNames, prism, null, singleAgentNPAccStates, stopReallocationWhenAnyRobotDeadends, 1.0, statesToAvoid, 0, mdps, fileLog);
			if (debugSSI) {
				planningValuesJP.add(resultvalues);
			}
			ArrayList<ArrayList<Expression>> remainingTasks = new ArrayList<ArrayList<Expression>>();
			ArrayList<int[]> correspondingMDPInitialStates = new ArrayList<int[]>();
			ArrayList<State> correspondingJointStates = new ArrayList<State>();
			int numPlanning = 0;
			//			doingReallocs = false; 
			stopTime = System.currentTimeMillis();
			runTime = stopTime - startTime;
			//			System.out.println("\nRun time: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");

			totalTimeDuration += runTime;
			this.firstSolDuration = totalTimeDuration;
			fileLog.println("Joint Policy Building: " + getTimeString(runTime));
			fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
			fileLog.println("XXX,D,"+System.currentTimeMillis());
			if (doingReallocs) {
				startTime = System.currentTimeMillis();
				processReallocations(numRobots, taskSet, remainingTasks, correspondingMDPInitialStates, correspondingJointStates, productMDPs, mdps, jvlTosvl,
						mdpInitialStates, mainLog, reallocStatesPQ, reallocStatesMapToList);
				//so now we just repeat for each remainingTasks thing 
				stopTime = System.currentTimeMillis();
				runTime = stopTime - startTime;
				//			System.out.println("\nRun time: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");

				totalTimeDuration += runTime;

				fileLog.println("Reallocation Prep: " + getTimeString(runTime));
				fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
				while (!reallocStatesPQ.isEmpty()) {
					startTime = System.currentTimeMillis();

					numPlanning++;
					fileLog.println("Reallocation " + numPlanning);
					StateExtended currentSE = reallocStatesPQ.remove();
					State ps = currentSE.getChildStateState();
					double stateProb = currentSE.parentToChildTransitionProbability;

					int stateIndex = reallocStatesMapToList.get(ps);

					ArrayList<Expression> currentTaskSet = remainingTasks.get(stateIndex);
					int[] currentMDPInitialStates = correspondingMDPInitialStates.get(stateIndex);
					//if they're both deadends just skip them 
					boolean allDeadends = true;
					for (int i = 0; i < currentMDPInitialStates.length; i++) {
						if (currentMDPInitialStates[i] != -1) {
							boolean thisDeadend = this.stateIsDeadend(mdps.get(i), currentMDPInitialStates[i]);
							allDeadends = thisDeadend & allDeadends;
							if (!allDeadends)
								break;
						}
					}
					stopTime = System.currentTimeMillis();
					runTime = stopTime - startTime;
					totalTimeDuration += runTime;
					fileLog.println("Reallocation Prep: " + getTimeString(runTime));
					fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
					if (allDeadends)
						continue;
					startTime = System.currentTimeMillis();
					//sanity check 
					State sanityCheckS = correspondingJointStates.get(stateIndex);
					if (ps.compareTo(sanityCheckS) != 0)
						throw new PrismException("Whoops the states dont match");
					String ms = ps.toString() + " - ";
					stopTime = System.currentTimeMillis();
					runTime = stopTime - startTime;
					totalTimeDuration += runTime;
					fileLog.println("Reallocation Prep: " + getTimeString(runTime));
					fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

					if (currentTaskSet.size() > 0) {
						startTime = System.currentTimeMillis();
						for (int i = 0; i < mdps.size(); i++) {

							((MDPSimple) mdps.get(i)).clearInitialStates();
							if (currentMDPInitialStates[i] != -1) {
								((MDPSimple) mdps.get(i)).addInitialState(currentMDPInitialStates[i]);
								ms += ((MDPSimple) mdps.get(i)).getStatesList().get(currentMDPInitialStates[i]).toString() + "=";
								ms += ((MDPSimple) mdps.get(i)).getStatesList().get(((MDPSimple) mdps.get(i)).getFirstInitialState()).toString() + " ";
							}
						}
						stopTime = System.currentTimeMillis();
						runTime = stopTime - startTime;
						//			System.out.println("\nRun time: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");

						totalTimeDuration += runTime;
						fileLog.println("Reallocation Prep: " + getTimeString(runTime));
						fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

						meh = auctionTasksSumOfCostsNestedProductIncremental(currentTaskSet, numRobots, mdps, rewExpr, safetyExpr, mcs, costsModels, prism,
								saveplace, filename, mainLog, fileLog);
						fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
						fileLog.println("XXX,B,"+System.currentTimeMillis());
						startTime = System.currentTimeMillis();
						robotsTasksBroken = meh.getKey();
						samdps = meh.getValue().getKey();
						nviSols = meh.getValue().getValue();

						//lets process these 
						for (int i = 0; i < robotsTasksBroken.size(); i++) {
							if (robotsTasksBroken.get(i).size() == 1) {
								//								if (robotsTasksBroken.get(i).contains(safetyExpr)) {
								//									robotsTasksBroken.get(i).remove(0);
								//
								//								}
								robotsTasksBroken.get(i).remove(0);
							}
						}
						mainLog.println("\n\nUpdated Assigned Tasks");
						mainLog.println(robotsTasksBroken.toString());

						//torepeat
						//print task distribution and get strategy 
						nviStrategies = new ArrayList<MDStrategy>();
						productMDPs = new ArrayList<MDP>();
						finalDAList = new ArrayList<ArrayList<DAInfo>>();
						costRewards = new ArrayList<MDPRewardsSimple>();
						singleAgentNPAccStates = new ArrayList<BitSet>();
						expTaskRewards = new ArrayList<MDPRewardsSimple>();
						//					if (robotsTasksBroken.get(1).size() == 1)
						//						continue;
						stopTime = System.currentTimeMillis();
						runTime = stopTime - startTime;
						//			System.out.println("\nRun time: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");

						totalTimeDuration += runTime;
						fileLog.println("Reallocation Prep: " + getTimeString(runTime));
						fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

						statesToAvoid = getRobotPlansUsingNVINestedProductIncremental(numRobots, samdps, nviSols, costsModels, productMDPs, mainLog,
								costRewards, saveplace, filename, prism, nviStrategies, finalDAList, singleAgentNPAccStates, fileLog, expTaskRewards,
								planningValuesSSI);
						fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

						startTime = System.currentTimeMillis();
						for (int i = 0; i < productMDPs.size(); i++) {
							if (productMDPs.get(i) != null)
								ms += productMDPs.get(i).getStatesList().get(productMDPs.get(i).getFirstInitialState()) + " ";
						}
						fileLog.println("XXX,C,"+System.currentTimeMillis());
						if (debugSSI) {

							for (int i = 0; i < productMDPs.size(); i++) {
								if (productMDPs.get(i) != null) {
									//			String pfn = saveplace + "results/" + fnPrefix, filename + "_jp"
									PolicyCreator pc = new PolicyCreator();

									pc.createPolicyWithRewardsStructuresAsLabels(productMDPs.get(i).getFirstInitialState(), productMDPs.get(i),
											nviStrategies.get(i), expTaskRewards.get(i), costRewards.get(i), singleAgentNPAccStates.get(i));
									//(productMDPs.get(i), nviStrategies.get(i));
									pc.savePolicy(saveplace + "results/" + fnPrefix, filename + "_" + numPlanning + "_" + ps.toString() + "_ptRews" + i);
								}
							}
						}

						jvlTosvl = mapJointVarListToRobotVarList(ssNames, finalDAList, jvl, productMDPs, null, null);
						resultvalues = createJointPolicy(daIndices, jvlDAMap, jvlTosvl, costRewards, productMDPs, mainLog, nviStrategies, saveplace, filename,
								ssNames, prism, ps, singleAgentNPAccStates, stopReallocationWhenAnyRobotDeadends, stateProb, statesToAvoid, numPlanning, null,
								fileLog);
						if (debugSSI) {
							planningValuesJP.add(resultvalues);
						}
						processReallocations(numRobots, taskSet, remainingTasks, correspondingMDPInitialStates, correspondingJointStates, productMDPs, mdps,
								jvlTosvl, currentMDPInitialStates, mainLog, reallocStatesPQ, reallocStatesMapToList);
						fileLog.println("XXX,D,"+System.currentTimeMillis());
					}
					stopTime = System.currentTimeMillis();
					runTime = stopTime - startTime;
					//			System.out.println("\nRun time: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");

					totalTimeDuration += runTime;
					fileLog.println("Joint Policy Building: " + getTimeString(runTime));
					fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

				}
			}
			allReplanningDuration = totalTimeDuration - firstSolDuration;
			startTime = System.currentTimeMillis();
			ModelCheckerMultipleResult nviSol = computeNestedValIterFailurePrint(mcs.get(0), mdpCreator.mdp, mdpCreator.accStates, new BitSet(),
					mdpCreator.getRewardsInArray(), 0, true, prism, mainLog);

			mainLog.println("Reallocated " + numPlanning + " times");
//			mdpCreator.saveMDP(saveplace + "results/" + fnPrefix, filename + "_jp");
			if (debugSSI) {
				for (int i = 0; i < planningValuesSSI.size(); i++) {
					mainLog.println(i + ":" + "P:" + Arrays.toString(planningValuesSSI.get(i)) + " C:" + Arrays.toString(planningValuesJP.get(i)));
				}
			}
			results = resultValues(nviSol, mdpCreator.mdp, fileLog);
			fileLog.println("XXX,E,"+System.currentTimeMillis());
			//			return resultvalues;
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			fileLog.println(e.getStackTrace().toString());

		}
		//TODO:your code here 
		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		//			System.out.println("\nRun time: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");

		totalTimeDuration += runTime;
		
		fileLog.println("Final Clean up: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(totalTimeDuration));
		fileLog.println("XXX,F,"+System.currentTimeMillis());
		return results;
	}

	void getCostsModels(int numRobots, ExpressionReward rewExpr, ArrayList<MDPModelChecker> mcs, ArrayList<MDPSimple> mdps,
			ArrayList<MDPRewardsSimple> costsModels) throws PrismException
	{
		for (int rnum = 0; rnum < numRobots; rnum++) {
			RewardStruct costStruct = (rewExpr).getRewardStructByIndexObject(mcs.get(rnum).getModulesFile(),
					mcs.get(rnum).getModulesFile().getConstantValues());
			MDPRewardsSimple costsModel = (MDPRewardsSimple) mcs.get(rnum).constructRewards(mdps.get(rnum), costStruct);
			costsModels.add(costsModel);
			MDPSimple model = mdps.get(rnum);
			//			System.out.println("Initial State: " + model.getStatesList().get(model.getFirstInitialState()));
			//			System.out.println("First action name: " + model.getAction(model.getFirstInitialState(), 0).toString());
			//			System.out.println("Reward for first action " + costsModel.getTransitionReward(model.getFirstInitialState(), 0));

		}
	}

	ArrayList<BitSet> getRobotPlansUsingNVINestedProductIncremental(int numRobots, ArrayList<SingleAgentNestedProductMDP> nps,
			ArrayList<ModelCheckerMultipleResult> nviSols, ArrayList<MDPRewardsSimple> costsModels, ArrayList<MDP> productMDPs, PrismLog mainLog,
			ArrayList<MDPRewardsSimple> costRewards, String saveplace, String filename, Prism prism, ArrayList<MDStrategy> nviStrategies,
			ArrayList<ArrayList<DAInfo>> finalDAList, ArrayList<BitSet> singleAgentNPAccStates, PrismLog fileLog, ArrayList<MDPRewardsSimple> expTaskRews,
			ArrayList<double[]> planningValuesSSI) throws PrismException
	{
		////profile
		long startTime = System.currentTimeMillis();

		double[] finalResVals = new double[] { 1.0, 0.0, 0.0 };
		double[][] resValsRobots = new double[numRobots][3];
		ArrayList<BitSet> statesToAvoid = new ArrayList<BitSet>();
		//TODO:your code here 

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		totalTimeDuration += runTime;
		fileLog.println("Solution Prep: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(totalTimeDuration));

		long maxTime = 0;
		for (int rnum = 0; rnum < numRobots; rnum++) {
			////profile
			long startTimex = System.currentTimeMillis();

			//			System.out.println("\nRun time: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
			//end profiling
			SingleAgentNestedProductMDP sanp = nps.get(rnum);
			if (sanp != null) {

				SingleAgentNestedProductMDP res = sanp;
				MDPRewardsSimple costsModel = costsModels.get(rnum);
				ArrayList<MDPRewardsSimple> rewards = createMaxExpTaskRewStruct(res, costsModel);
				costRewards.add(rewards.get(1)); //cuz its always 2 for now 

				expTaskRews.add(rewards.get(0));
				if (debugSSI)
					new MDPCreator().saveMDP(res.finalProduct.getProductModel(), saveplace + "results/" + fnPrefix, filename + "_prod_" + rnum,
							res.combinedAcceptingStates);
				//				mainLog.println(res.numMDPVars);
				ModelCheckerMultipleResult nviSol = nviSols.get(rnum);
				double[] resVals = this.resultValues(nviSol, (MDPSimple) res.finalProduct.getProductModel(), null);

				for (int i = 0; i < resVals.length; i++) {
					if (i == 0)
						finalResVals[i] *= resVals[i];
					else
						finalResVals[i] += resVals[i];
				}
				resValsRobots[rnum] = resVals.clone();
				statesToAvoid.add((BitSet) res.combinedStatesToAvoid.clone());
				//for each da in daList ;
				//lets see what we have 
				singleAgentNPAccStates.add((BitSet) res.combinedAcceptingStates.clone());
				productMDPs.add(res.finalProduct.getProductModel());
				nviStrategies.add(nviSol.strat);
				finalDAList.add(res.daList);
				if (debugSSI) {
					PolicyCreator pc = new PolicyCreator();
					pc.createPolicy(res.finalProduct.getProductModel(), nviSol.strat);
					pc.savePolicy(saveplace + "results/" + fnPrefix, filename + "p" + rnum);
				}

			} else {
				statesToAvoid.add(null);
				singleAgentNPAccStates.add(null);
				productMDPs.add(null);
				nviStrategies.add(null);
				finalDAList.add(null);
				costRewards.add(null);
				expTaskRews.add(null);
			}
			//TODO:your code here 

			long stopTimex = System.currentTimeMillis();
			long runTimex = stopTimex - startTimex;
			if (maxTime < runTimex)
				maxTime = runTimex;
			fileLog.println("Robot " + rnum + " Solution: " + getTimeString(runTimex));
			fileLog.println("Max: " + getTimeString(maxTime));
		}
		totalTimeDuration += maxTime;
		startTime = System.currentTimeMillis();
		if (debugSSI) {
			for (int rnum = 0; rnum < numRobots; rnum++) {
				System.out.println(rnum + ": " + Arrays.toString(resValsRobots[rnum]));
			}

			planningValuesSSI.add(finalResVals);
		}
		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		totalTimeDuration += runTime;
		return statesToAvoid;

	}

	int updateMDPStateUsingSS(HashMap<Integer, HashMap<Integer, Integer>> jvlTosvl, MDPSimple mdp, int state, ArrayList<String> ssNames, State js, VarList jvl)
	{

		State stateState = mdp.getStatesList().get(state);
		VarList svl = mdp.getVarList();
		for (String ss : ssNames) {
			int svlIndex = svl.getIndex(ss);
			int jvlIndex = jvl.getIndex(ss);
			Object jsVal = js.varValues[jvlIndex];
			stateState.setValue(svlIndex, jsVal);

		}
		return this.findStateIndexFromState(mdp, stateState);
	}

	void processReallocations(int numRobots, ArrayList<Expression> taskSet, ArrayList<ArrayList<Expression>> remainingTasks,
			ArrayList<int[]> correspondingMDPInitialStates, ArrayList<State> correspondingJointStates, ArrayList<MDP> productMDPs, ArrayList<MDPSimple> mdps,
			HashMap<Integer, HashMap<Integer, Integer>> jvlTosvl, int[] mdpInitialStatesFromBefore, PrismLog mainLog, Queue<StateExtended> reallocStatesPQ,
			HashMap<State, Integer> reallocStatesMapToList) throws PrismException
	{
		VarList jvl = mdpCreator.mdp.getVarList();
		mainLog.println("Processing Reallocation States");
		while (!possibleReallocStates.isEmpty()) {
			String mlString = "";
			Entry<StateExtended, ArrayList<Expression>> reallocStateTaskPair = possibleReallocStates.remove();
			StateExtended se = reallocStateTaskPair.getKey();
			reallocStatesPQ.add(se);
			State s = se.getChildStateState();
			//			boolean bc = false;
			//			if (s.toString().contains("0,0,1,0,0,1,-1,-1"))
			//				bc = true;
			ArrayList<Expression> completedTasks = reallocStateTaskPair.getValue();
			ArrayList<Expression> newTaskSet = new ArrayList<Expression>();
			for (int i = 0; i < taskSet.size(); i++) {
				if (!completedTasks.contains(getInnerExpression(taskSet.get(i)))) {
					newTaskSet.add(taskSet.get(i));
				}
			}

			int[] currentStates = jointStateToRobotStates(productMDPs, s, jvlTosvl, mainLog);
			correspondingJointStates.add(s);
			int[] mdpStates = new int[currentStates.length];
			for (int i = 0; i < productMDPs.size(); i++) {
				if (productMDPs.get(i) != null) {
					mdpStates[i] = productStateToMDPState(currentStates[i], productMDPs.get(i), mdps.get(i));

				} else {
					mdpStates[i] = updateMDPStateUsingSS(jvlTosvl, mdps.get(i), mdpInitialStatesFromBefore[i], ssNames, s, jvl);
				}
			}

			remainingTasks.add(newTaskSet);
			correspondingMDPInitialStates.add(mdpStates);
			//saving the value 
			reallocStatesMapToList.put(s, remainingTasks.size() - 1);
			mlString += s.toString() + " - ";
			for (int i = 0; i < productMDPs.size(); i++) {
				if (productMDPs.get(i) != null)
					mlString += productMDPs.get(i).getStatesList().get(currentStates[i]) + ":" + mdps.get(i).getStatesList().get(mdpStates[i]) + ", ";
				else
					mlString += "x : " + mdps.get(i).getStatesList().get(mdpStates[i]) + ", ";
			}
			mainLog.println(mlString);

		}
		mainLog.println("Done Processing Reallocation States");

	}

	protected double[] resultValues(ModelCheckerMultipleResult res2, MDPSimple mdp, PrismLog fileLog)
	{
		double[] result = null;
		ArrayList<double[]> solns = res2.solns;
		double[] solnProb = solns.get(solns.size() - 1);
		StateValues probsProduct = StateValues.createFromDoubleArray(solnProb, mdp);

		// Get final prob result
		if (mdp.getFirstInitialState() != -1) {
			result = new double[solns.size()];
			double maxProb = probsProduct.getDoubleArray()[mdp.getFirstInitialState()];
			result[0] = maxProb;
			String resString = "";
			for (int i = 0; i < solns.size() - 1; i++) {

				StateValues costsProduct = StateValues.createFromDoubleArray(res2.solns.get(i), mdp);

				double minCost = costsProduct.getDoubleArray()[mdp.getFirstInitialState()];
				resString += i + ":" + minCost + " ";
				result[i + 1] = minCost;
			}
			//			mainLog.println("\nFor p = " + maxProb + ", rewards " + resString);
			if (fileLog != null) {
				fileLog.println("\nFor p = " + maxProb + ", rewards " + resString);
			}
		}
		return result;

	}

	int productStateToMDPState(int productState, MDP productMDP, MDP mdp)
	{
		//get the varlists 
		VarList pvl = productMDP.getVarList();
		VarList mvl = mdp.getVarList();
		//just the things that have the same names 
		//the order is the mvl 
		State productStateState = productMDP.getStatesList().get(productState);
		State mdpStateState = new State(mvl.getNumVars());
		for (int i = 0; i < mvl.getNumVars(); i++) {
			String name = mvl.getName(i);
			int pI = pvl.getIndex(name);
			mdpStateState.setValue(i, productStateState.varValues[pI]);
		}

		return findStateIndexFromState(mdp, mdpStateState);
	}

	int findStateIndexFromState(MDP mdp, State mdpStateState)
	{
		//now we've got to find this state 
		int mdpState = -1;
		List<State> mdpStatesList = mdp.getStatesList();
		for (int s = 0; s < mdpStatesList.size(); s++) {
			if (mdpStatesList.get(s).compareTo(mdpStateState) == 0) {
				mdpState = s;
				break;
			}
		}
		return mdpState;
	}

	Object[] createJointState(ArrayList<String> ssNames, ArrayList<Integer> daIndices, HashMap<Integer, HashMap<Integer, Integer>> jvlTosvl,
			HashMap<Integer, DAInfo> processedDAList, ArrayList<MDP> productMDPs, ArrayList<MDPSimple> originalMDPs, int[] currentStates, State ps)
			throws PrismException
	{

		Object[] res = new Object[3];

		boolean isAcc = false;
		int numAcc = 0; //so why do we have ess and acc 
		//well cuz ess is like we've completed a new task 
		//acc is like we've completed all the tasks 

		int numDAs = daIndices.size() - 1;
		int numEss = 0;
		VarList jvl = mdpCreator.mdp.getVarList();
		int numVars = jvl.getNumVars();
		State jointState = new State(numVars);
		//so we have the dastate indices 
		//we have the private state indices 
		//we have the shared state vars 
		//for now i'm ignoring the shared state vars 
		//TODO: add shared state stuff 

		//so we do this for each robot really 
		//its easier to do the da first and then the ss then the ps 

		ArrayList<State> robotStateStates = new ArrayList<State>();
		for (int i = 0; i < productMDPs.size(); i++) {
			if (productMDPs.get(i) != null) {
				robotStateStates.add(productMDPs.get(i).getStatesList().get(currentStates[i]));
			} else {
				if (originalMDPs != null) {
					robotStateStates.add(originalMDPs.get(i).getStatesList().get(currentStates[i]));
				} else
					robotStateStates.add(null);
			}
		}
		for (int jvlIndex = 0; jvlIndex < numVars; jvlIndex++) {
			String name = jvl.getName(jvlIndex);
			boolean isSS = ssNames.contains(name);
			boolean isDA = daIndices.contains(jvlIndex);

			Object valToSet = null;
			if (jvlTosvl.containsKey(jvlIndex)) {

				HashMap<Integer, Integer> hm = jvlTosvl.get(jvlIndex);
				for (int r : hm.keySet()) {
					State rs = robotStateStates.get(r);
					int svlIndex = hm.get(r);
					if (rs != null) {

						if (isSS) {
							if (valToSet == null) {
								valToSet = rs.varValues[svlIndex];
							} else {
								int rVal = (int) rs.varValues[svlIndex];
								if (ps == null) {
									if ((int) valToSet != rVal) {
										throw new PrismException("Seems the shared states dont have the same value even without a parent " + rs.toString());
									}
								} else {
									int psVal = (int) ps.varValues[jvlIndex];
									if ((psVal != rVal)) //if this value and the parent value are not the same  
									{
										//do we take this one ? 
										//only if it is not the same as the one we already have 
										if ((int) valToSet != rVal) {
											valToSet = rs.varValues[svlIndex];
										}

									}
								}

							}
						} else {
							valToSet = rs.varValues[svlIndex];
						}
					} else {
						if (ps != null)
							valToSet = ps.varValues[jvlIndex];
						else {
							throw new PrismException("Seems like we dont have a value for this index?? " + jvlIndex + " " + robotStateStates.toString());
						}
					}

				}
			} else {
				if (ps != null)
					valToSet = ps.varValues[jvlIndex];
				else {
					throw new PrismException("Seems like we dont have a value for this index?? " + jvlIndex + " " + robotStateStates.toString());
				}

			}
			jointState.setValue(jvlIndex, valToSet);
			if (isDA & ps != null) {
				int psVal = (int) ps.varValues[jvlIndex];
				if (processedDAList.containsKey(jvlIndex)) {
					DAInfo daInfo = processedDAList.get(jvlIndex);
					if (!daInfo.isSafeExpr) {
						if (daInfo.daAccStates.get((int) (valToSet))) {
							numAcc++;
							if (!daInfo.daAccStates.get(psVal))
								numEss++;
						}
					}

				}
			}
		}

		if (numAcc == numDAs)
			isAcc = true;
		//		if (numEss > 0) {
		//			System.out.println("Essential State: " + jointState.toString());
		//
		//		}
		//		if (isAcc)
		//			System.out.println("Accepting State: " + jointState.toString());
		res[0] = jointState;
		res[1] = numEss;
		res[2] = isAcc;

		return res;

	}

	double[] createJointPolicy(ArrayList<Integer> daIndices, HashMap<Integer, DAInfo> jvlDAMap, HashMap<Integer, HashMap<Integer, Integer>> jvlTosvl,
			ArrayList<MDPRewardsSimple> costRewards, ArrayList<MDP> productMDPs, PrismLog mainLog, ArrayList<MDStrategy> nviStrategies, String saveplace,
			String filename, ArrayList<String> ssNames, Prism prism, State parentState, ArrayList<BitSet> singleAgentNPAcceptingStates,
			boolean stopAtFirstDeadend, double initStateProb, ArrayList<BitSet> statesToAvoid, int numPlanning, ArrayList<MDPSimple> originalMDPs,
			PrismLog fileLog) throws PrismException
	{

		int numRobots = productMDPs.size();
		possibleReallocStates = new LinkedList<Entry<StateExtended, ArrayList<Expression>>>();

		//get the actions 
		Object[] actions = new Object[numRobots];

		//for each robot mdp get initial state 
		int[] currentStates = new int[numRobots];
		for (int i = 0; i < numRobots; i++) {
			if (productMDPs.get(i) != null) {
				currentStates[i] = productMDPs.get(i).getFirstInitialState();
			} else {
				if (originalMDPs != null) {
					currentStates[i] = originalMDPs.get(i).getFirstInitialState();
				} else
					currentStates[i] = -1;
			}
		}
		Queue<State> statesQueues = new LinkedList<State>();
		Queue<Double> statesProbQueue = new LinkedList<Double>();

		Object[] jsPlusEssAccFlags = createJointState(ssNames, daIndices, jvlTosvl, jvlDAMap, productMDPs, originalMDPs, currentStates, parentState);

		State jointState = (State) jsPlusEssAccFlags[0];

		boolean isAcc = (boolean) jsPlusEssAccFlags[2];

		if (isAcc)
			return new double[] { 1.0, 0.0, 0.0 };
		statesQueues.add(jointState);
		statesProbQueue.add(initStateProb);

		ArrayList<State> visited = new ArrayList<State>();
		State currentJointState;
		double currentJointStateProb;
		try {
			while (!statesQueues.isEmpty()) {
				currentJointState = statesQueues.remove();
				currentJointStateProb = statesProbQueue.remove();
				if (!visited.contains(currentJointState))
					visited.add(currentJointState);
				else
					continue;

				currentStates = jointStateToRobotStates(productMDPs, currentJointState, jvlTosvl, mainLog);

				ArrayList<ArrayList<Integer>> robotStates = new ArrayList<ArrayList<Integer>>();
				ArrayList<ArrayList<Double>> robotStatesProbs = new ArrayList<ArrayList<Double>>();
				String infoString = "";

				double stateActionCost = 0;

				for (int i = 0; i < numRobots; i++) {
					ArrayList<Integer> nextRobotStates = new ArrayList<Integer>();

					ArrayList<Double> nextRobotStatesProbs = new ArrayList<Double>();

					MDStrategy strat = nviStrategies.get(i);
					MDP mdp = productMDPs.get(i);

					int actionChoice = -1;
					if (strat != null)
						actionChoice = strat.getChoiceIndex(currentStates[i]);

					if (actionChoice > -1) {
						stateActionCost += costRewards.get(i).getTransitionReward(currentStates[i], actionChoice);
						actions[i] = strat.getChoiceAction(currentStates[i]);

						infoString += i + ":" + currentStates[i] + mdp.getStatesList().get(currentStates[i]).toString() + "->" + actions[i].toString() + " ";
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
				//put them all together in the array 
				ArrayList<Integer> robotStateNums = new ArrayList<Integer>();
				//				stopAtFirstDeadend = true; 

				boolean allZero = true;
				if (!stopAtFirstDeadend) { //meaning we go all the way till all the robots execute their tasks 

					for (int i = 0; i < robotStates.size(); i++) {
						if (robotStates.get(i).size() != 0)
							allZero = false;
						else {
							robotStates.get(i).add(currentStates[i]);
							robotStatesProbs.get(i).add(1.0);

						}
						robotStateNums.add(robotStates.get(i).size());
					}
				} else {
					allZero = false; //so we're not zeroing everything 
					//but if the first robot we encounter has no successors 
					//then we check if its current state is an accepting state 
					//if its not we stop 
					//and say this is a reallocation 
					for (int i = 0; i < robotStates.size(); i++) {
						if (robotStates.get(i).size() == 0) {
							int robotCurrentState = currentStates[i];
							BitSet currentRobotAcceptingStates = singleAgentNPAcceptingStates.get(i);
							if (currentRobotAcceptingStates != null) {
								if (!currentRobotAcceptingStates.get(robotCurrentState)) {
									allZero = true;
									break;
								}
							}
							//if its null 
							//then we dont care about this robot 
							//cuz its failed 
							if (!allZero) {
								robotStates.get(i).add(currentStates[i]);
								robotStatesProbs.get(i).add(1.0);
							}
						}
						robotStateNums.add(robotStates.get(i).size());
					}
				}
				String ja = createJointAction(actions);
				//				mainLog.println(ja);

				if (!allZero) {
					ArrayList<Entry<State, Double>> successorsWithProbs = new ArrayList<Entry<State, Double>>();
					ArrayList<Object[]> essAndAcc = new ArrayList<Object[]>();

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

						Object[] nextJSPlusEssAccFlags = createJointState(ssNames, daIndices, jvlTosvl, jvlDAMap, productMDPs, null, nextStatesCombo,
								currentJointState);

						State nextJointState = (State) nextJSPlusEssAccFlags[0];
						int numEssNextjs = (int) nextJSPlusEssAccFlags[1];
						boolean isAccNextjs = (boolean) nextJSPlusEssAccFlags[2];
						essAndAcc.add(new Object[] { numEssNextjs, isAccNextjs });
						statesQueues.add(nextJointState);
						statesProbQueue.add(currentJointStateProb * comboProb);

						successorsWithProbs.add(new AbstractMap.SimpleEntry<State, Double>(nextJointState, comboProb));
					}
					mdpCreator.addAction(currentJointState, ja, successorsWithProbs, essAndAcc, stateActionCost);

				} else {
					//lets add these to a queue!!! 
					//add the current joint state to the possible realloc queue
					if (parentState != null) {
						if (currentJointState.compareTo(parentState) == 0)
							continue;
					}
					if (mdpCreator.getAccState(currentJointState)) {
						mainLog.println("Accepting State: " + currentJointState);
						continue;
					}
					//okay so lets do the essential state check again here 
					//basically we take the processedDAList and the stuff we had to convert it to individual robot states 
					ArrayList<Expression> tasksCompleted = tasksCompletedHere(currentJointState, jvlDAMap, jsToRobotState);
					//					mainLog.println(tasksCompleted.size());
					StateExtended se = new StateExtended(currentJointState, currentJointStateProb);
					possibleReallocStates.add(new AbstractMap.SimpleEntry<StateExtended, ArrayList<Expression>>(se, tasksCompleted));
				}
			}
		} catch (PrismException e) {
			mdpCreator.saveMDP(saveplace + "results/" + fnPrefix, filename + "_jp");
			throw e;
		}

		if (parentState == null)
			mdpCreator.setInitialState(jointState);

		//		mainLog.println("Goal Prob:" + mdpCreator.getProbabilityToReachAccStateFromJointMDP(jointState));
		mdpCreator.createRewardStructures();
		mdpCreator.mdp.findDeadlocks(true);
		if (mdpCreator.accStates.cardinality() > 0) {

			ModelCheckerMultipleResult nviSol = computeNestedValIterFailurePrint(null, mdpCreator.mdp, mdpCreator.accStates, new BitSet(),
					mdpCreator.getRewardsInArray(), 0, true, prism, mainLog);
			return resultValues(nviSol, mdpCreator.mdp, fileLog);
		} else {
			return new double[] { 0.0, 0.0, 0.0 };
		}

	}

	public boolean stateIsDeadend(MDP mdp, int s) throws PrismException
	{
		boolean isdeadend = false;
		int numChoices = mdp.getNumChoices(s);
		if (numChoices == 0) {
			isdeadend = true;
		} else if (numChoices == 1) {
			int numTransitions = mdp.getNumTransitions(s, 0);
			if (numTransitions == 1) {
				//is it a self loop ?? 
				Iterator<Entry<Integer, Double>> choiceIter = mdp.getTransitionsIterator(s, 0);

				while (choiceIter.hasNext()) {
					Entry<Integer, Double> nextS = choiceIter.next();
					if (nextS.getKey() == s) {
						isdeadend = true;

					} else {
						if (isdeadend)
							throw new PrismException(
									"We thought we detected a deadend but apparently its a bit mysterious " + mdp.getStatesList().get(s).toString());
					}

				}
			}
		}
		return isdeadend;
	}

	ArrayList<Expression> tasksCompletedHere(State jointState, HashMap<Integer, DAInfo> jvlDAMap, HashMap<Integer, int[]> jsToRobotState)
	{
		ArrayList<Expression> completedTasks = new ArrayList<Expression>();
		for (int jvlIndex : jvlDAMap.keySet()) {
			DAInfo daInfo = jvlDAMap.get(jvlIndex);
			if (!daInfo.isSafeExpr) {
				int jsVal = (int) jointState.varValues[jvlIndex];
				if (daInfo.daAccStates.get(jsVal)) {
					completedTasks.add(daInfo.daExpr);
				}
			}
		}

		return completedTasks;
	}

	int[] jointStateToRobotStates(ArrayList<MDP> productMDPs, State jointState, HashMap<Integer, HashMap<Integer, Integer>> jvlTosvl, PrismLog mainLog)
			throws PrismException
	{
		int numRobots = productMDPs.size();
		int[] robotStates = new int[numRobots];
		State[] robotStatesStates = new State[numRobots];
		for (int i = 0; i < numRobots; i++) {
			if (productMDPs.get(i) != null) {
				robotStatesStates[i] = new State(productMDPs.get(i).getVarList().getNumVars());
			} else
				robotStatesStates[i] = null;
		}
		for (int jvlIndex : jvlTosvl.keySet()) {
			HashMap<Integer, Integer> rm = jvlTosvl.get(jvlIndex);
			Object valToSet = jointState.varValues[jvlIndex];
			for (int r : rm.keySet()) {
				int svlIndex = rm.get(r);
				if (robotStatesStates[r] /*==*/ != null)
					robotStatesStates[r].setValue(svlIndex, valToSet);
				//		throw new PrismException("trying to get a state for a robot that is not participating here " + r);

			}

		}
		for (int r = 0; r < numRobots; r++) {

			State rs = robotStatesStates[r];
			if (rs != null) {
				//			mainLog.println(rs.toString());
				//we need to find the corresponding state 
				MDP mdp = productMDPs.get(r);
				int matchingS = findMatchingStateNum(mdp, rs);
				robotStates[r] = matchingS;
			} else
				robotStates[r] = -1;
			//			mainLog.println(matchingS);

		}
		return robotStates;
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

		new SSIAuctionNestedProduct().run();
	}
}
