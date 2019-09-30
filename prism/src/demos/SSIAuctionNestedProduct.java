package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import acceptance.AcceptanceType;

import java.util.Queue;

import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.MDPSparse;
import explicit.Model;
import explicit.ModelCheckerMultipleResult;
import explicit.StateValues;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationType;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import parser.ast.RewardStruct;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;

public class SSIAuctionNestedProduct
{
	Queue<Entry<State, ArrayList<Expression>>> possibleReallocStates;
	MDPCreator jointPolicyCreator;
	private HashMap<Integer, int[]> jsToRobotState;
	private MDPCreator mdpCreator = null;
	private boolean doingReallocs = false;
	String fnPrefix = "ssi"; 

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

					if (saMDP.addRewardForTaskCompletion(nextS, s)) {
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
			else
				thisExpr = ((ExpressionQuant) exprs.get(daNum)).getExpression();
			DAInfo daInfo = new DAInfo(mainLog, thisExpr, hasReward);
			daInfoList.add(daInfo);
		}

		return daInfoList;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,
			ArrayList<Boolean> minRewards, int probPreference, Prism prism, PrismLog mL) throws PrismException
	{

		ModelCheckerMultipleResult res2 = computeNestedValIterFailurePrint(mdp, target, statesToAvoid, rewards, minRewards, probPreference, null, prism, mL);// computeNestedValIterFailurePrint(mdp, target, statesToAvoid,
		// rewards,minRewards,target,probPreference,null);

		return res2;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,
			ArrayList<Boolean> minRewards, int probPreference, double[] probInitVal, Prism prismC, PrismLog mainLog) throws PrismException
	{

		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(prismC);
		mc.setGenStrat(true);
		// mc.genStrat = true;
		// lets set them all to true
		//		ModelCheckerResult anotherSol = mc.computeUntilProbs(mdp, statesToRemainIn, target, false);
		//		StatesHelper.saveStrategy(anotherSol.strat, target, "", "computeUntilProbsStrat" + mdp.getFirstInitialState(),
		//				true);
		//		StateValues testValues = StateValues.createFromDoubleArray(anotherSol.soln, mdp);
		//		mainLog.println("Compute Until Probs Vals\n " + Arrays.toString(testValues.getDoubleArray()));
		//		if (mdp.getFirstInitialState() != -1)
		//			mainLog.println("Prob in init" + testValues.getDoubleArray()[mdp.getFirstInitialState()]);

		ModelCheckerMultipleResult res2 = mc.computeNestedValIterArray(mdp, target, statesToRemainIn, rewards, null, minRewards, target, probPreference,
				probInitVal);

		ArrayList<double[]> solns = res2.solns;
		double[] solnProb = solns.get(solns.size() - 1);
		StateValues probsProduct = StateValues.createFromDoubleArray(solnProb, mdp);

		// Get final prob result
		if (mdp.getFirstInitialState() != -1) {
			double maxProb = probsProduct.getDoubleArray()[mdp.getFirstInitialState()];

			String resString = "";
			for (int i = 0; i < solns.size() - 1; i++) {
				StateValues costsProduct = StateValues.createFromDoubleArray(res2.solns.get(i), mdp);

				double minCost = costsProduct.getDoubleArray()[mdp.getFirstInitialState()];
				resString += i + ":" + minCost + " ";

			}
			mainLog.println("\nFor p = " + maxProb + ", rewards " + resString);
		}
		return res2;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,
			int probPreference, boolean doMaxTasks, Prism p, PrismLog m) throws PrismException
	{

		ArrayList<Boolean> minMaxRew = new ArrayList<Boolean>();
		int rewinit = 0;
		if (doMaxTasks) {
			minMaxRew.add(false);
			rewinit++;
		}
		for (int rew = rewinit; rew < rewards.size(); rew++)
			minMaxRew.add(true);
		return computeNestedValIterFailurePrint(mdp, target, statesToAvoid, rewards, minMaxRew, probPreference, p, m);
	}

	public Entry<Integer, Double> getSingleAgentBid(MDP agentMDP, ArrayList<Expression> taskSet, Expression agentTasks, ExpressionReward rewardExpr,
			MDPModelChecker mc) throws PrismException
	{

		Expression bidPick = null;
		double bidValue = 100000;
		//		Expression robotTasks = null;
		Expression currentTask = null;
		int indexOfChosenTask = -1;

		for (int exprNum = 0; exprNum < taskSet.size(); exprNum++) {
			currentTask = getInnerExpression(taskSet.get(exprNum));
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

	public Entry<ExpressionReward, Entry<Expression, ArrayList<Expression>>> processProperties(PropertiesFile propertiesFile, PrismLog mainLog,int numGoals)
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
		Expression safetyExpr = ltlExpressions.get(numOp - 1);
		ArrayList<Expression> taskSet = new ArrayList<Expression>();

		for (int exprNum = 0; exprNum < numGoals-1; exprNum++) {

			Expression currentExpr = ltlExpressions.get(exprNum);
			//			Expression currentExprWithSafetyExpr = Expression.And(currentExpr, safetyExpr);
			taskSet.add(currentExpr);
		}
		//		taskSet.add(safetyExpr);
		return new AbstractMap.SimpleEntry<ExpressionReward, Entry<Expression, ArrayList<Expression>>>(rewExpr,
				new SimpleEntry<Expression, ArrayList<Expression>>(safetyExpr, taskSet));
	}

	public Expression getInnerExpression(Expression expr)
	{
		return ((ExpressionQuant) expr).getExpression();
	}

	public ArrayList<ArrayList<Expression>> auctionTasks(ArrayList<Expression> taskSet, int numRobots, ArrayList<MDPSimple> mdps, ExpressionReward rewExpr,
			Expression safetyExpr, ArrayList<MDPModelChecker> mcs) throws PrismException
	{
		ArrayList<Expression> robotsTasks = new ArrayList<Expression>();
		ArrayList<ArrayList<Expression>> robotsTasksBroken = new ArrayList<ArrayList<Expression>>();

		for (int i = 0; i < numRobots; i++) {
			robotsTasks.add(getInnerExpression(safetyExpr));
			robotsTasksBroken.add(new ArrayList<Expression>());
		}
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
			robotsTasksBroken.get(bestBidRobotIndex).add(bestBidExpression);
			if (robotTasks != null) {
				bestBidExpression = Expression.And(robotTasks, getInnerExpression(bestBidExpression));

			}

			robotsTasks.set(bestBidRobotIndex, bestBidExpression);
			taskSet.remove(bestBidIndex);

		}
		for (int i = 0; i < numRobots; i++) {
			robotsTasksBroken.get(i).add(safetyExpr);
		}
		return robotsTasksBroken;
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
			pc.savePolicy(saveplace + "results/"+fnPrefix, filename + "_" + i);
			nviStrategies.add(nviStrategy);
		}
	}

	public PropertiesFile loadFiles(Prism prism, String saveplace, String filename, int numRobots, ArrayList<MDPSimple> mdps, ArrayList<MDPModelChecker> mcs)
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
	protected SingleAgentNestedProductMDP buildSingleAgentNestedProductMDP(String name, Model model, ArrayList<DAInfo> daList, BitSet statesOfInterest,
			Prism prism, MDPModelChecker mc, PrismLog mainLog) throws PrismException
	{
		// return the list of daInfo and the product mdp

		SingleAgentNestedProductMDP res = new SingleAgentNestedProductMDP(mainLog);
		res.setNumMDPVars(model.getVarList().getNumVars());
		res.initializeProductToMDPStateMapping((MDP) model);

		LTLProduct<MDP> product = null;
		MDP productMDP = null;
		LTLModelChecker mcLTL = new LTLModelChecker(prism); // is this okay ?
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };

		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		// all the states are states of interest
		bsInit.set(0, numStates);
		productMDP = (MDP) model;
		res.daList = new ArrayList<DAInfo>();

		for (int daNum = 0; daNum < daList.size(); daNum++) {
			DAInfo daInfo = new DAInfo(daList.get(daNum));
			daInfo.associatedIndexInProduct++; //should go to zero from -1 

			product = daInfo.constructDAandProductModel(mcLTL, mc, allowedAcceptance, productMDP, null, true);
			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);

			//			RewardStruct costStruct = exprRew.getRewardStructByIndexObject(getModulesFile(), getModulesFile().getConstantValues());
			//			mainLog.println("Building cost structure...");
			//			Rewards costsModel = constructRewards(model, costStruct);
			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				res.daList.get(otherDAs).updateStateNumbers(product);
				res.daList.get(otherDAs).associatedIndexInProduct++; //and everyone else also gets shifted once. 

				//				StatesHelper.saveBitSet(res.daList.get(otherDAs).essentialStates, "",
				//						name + "pda_" + daNum + "_" + otherDAs + ".ess", true);
				//				StatesHelper.saveBitSet(res.daList.get(otherDAs).productAcceptingStates, "",
				//						name + "pda_" + daNum + "_" + otherDAs + ".acc", true);
			}
			//			StatesHelper.saveHashMap(res.productStateToMDPState, "",
			//					name + "pda_" + daNum + "_before_productStateToMDPState.txt", true);
			res.updateProductToMDPStateMapping(product);
			//			StatesHelper.saveHashMap(res.productStateToMDPState, "",
			//					name + "pda_" + daNum + "_after_productStateToMDPState.txt", true);
			res.daList.add(daInfo);
		}
		DAInfo daInfo = res.daList.get(res.daList.size() - 1);
		int daNum = res.daList.size() - 1;
		//					StatesHelper.saveDA(daInfo.da, "", name + "da_" + daNum, true);
		//					StatesHelper.saveMDP(productMDP, daInfo.productAcceptingStates, "", name + "pda_" + daNum, true);
		//					StatesHelper.saveMDP(productMDP, daInfo.essentialStates, "", name + "pda_" + daNum + "switchStates", true);
		//					StatesHelper.saveMDPstatra(productMDP, "", name + "pda_" + daNum + "sta_tra", true);

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
		return run(saveplace,fn, numRobots,numGoals, numDoors);
	}

	public double[] run(String saveplace,String fn, int numRobots, int numGoals,int numDoors)
	{
		fnPrefix +="r"+numRobots+"_g"+numGoals+"d"+numDoors; 
		try {
//			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
			String filename = fn;//"g5_r2_t3_d2_fs1";//"g10_r4_t6_d4_fs8";//"g5_r2_t3_d2_fs1";//"g7_r5_t6_d3_fs2";//"g7x3_r2_t3_d0_fs1";//"robot";
			ArrayList<String> ssNames = new ArrayList<String>();
			//			int numRobots = 2;//4; //numRobots

			//			int numDoors = 2;//4;
			for (int i = 0; i < numDoors; i++)
				ssNames.add("door" + i);
			//			ssNames.add("door0");

			// Create a log for PRISM output (hidden or stdout)
			//PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);

			prism.initialise();
			prism.setEngine(Prism.EXPLICIT);

			ArrayList<MDPSimple> mdps = new ArrayList<MDPSimple>();
			ArrayList<MDPModelChecker> mcs = new ArrayList<MDPModelChecker>();

			PropertiesFile propertiesFile = loadFiles(prism, saveplace, filename, numRobots, mdps, mcs);

			//do things to the properties 

			Entry<ExpressionReward, Entry<Expression, ArrayList<Expression>>> processedProperties = processProperties(propertiesFile, mainLog,numGoals);
			ExpressionReward rewExpr = processedProperties.getKey();
			Entry<Expression, ArrayList<Expression>> safetyExprAndList = processedProperties.getValue();
			Expression safetyExpr = safetyExprAndList.getKey();
			ArrayList<Expression> taskSet = safetyExprAndList.getValue();
			ArrayList<Expression> taskSetToEdit = new ArrayList<Expression>();
			taskSetToEdit.addAll(taskSet);

			ArrayList<MDPRewardsSimple> costsModels = new ArrayList<MDPRewardsSimple>();
			getCostsModels(numRobots, rewExpr, mcs, mdps, costsModels);

			ArrayList<ArrayList<Expression>> robotsTasksBroken = auctionTasks(taskSetToEdit, numRobots, mdps, rewExpr, safetyExpr, mcs);
			mainLog.println("\n\nAssigned Tasks");
			mainLog.println(robotsTasksBroken.toString());

			//torepeat
			//print task distribution and get strategy 
			ArrayList<MDStrategy> nviStrategies = new ArrayList<MDStrategy>();
			ArrayList<MDP> productMDPs = new ArrayList<MDP>();
			ArrayList<ArrayList<DAInfo>> finalDAList = new ArrayList<ArrayList<DAInfo>>();
			ArrayList<MDPRewardsSimple> costRewards = new ArrayList<MDPRewardsSimple>();

			getRobotPlansUsingNVINestedProduct(numRobots, robotsTasksBroken, rewExpr, productMDPs, mainLog, mcs, mdps, costRewards, saveplace, filename, prism,
					nviStrategies, finalDAList, costsModels);

			double[] resultvalues = createJointPolicy(costRewards, finalDAList, mainLog, productMDPs, nviStrategies, saveplace, filename, ssNames, prism);
			//torepeat end 
			if (doingReallocs) {
				Queue<ArrayList<Expression>> remainingTasks = new LinkedList<ArrayList<Expression>>();
				Queue<int[]> correspondingMDPInitialStates = new LinkedList<int[]>();
				processReallocations(numRobots, taskSet, remainingTasks, correspondingMDPInitialStates, productMDPs, mdps, mainLog);

				//so now we just repeat for each remainingTasks thing 
				while (!remainingTasks.isEmpty()) {
					ArrayList<Expression> currentTaskSet = remainingTasks.remove();
					if (currentTaskSet.size() > 0) {
						int[] currentMDPInitialStates = correspondingMDPInitialStates.remove();
						//now lets just do this again 
						//one time 
						for (int i = 0; i < mdps.size(); i++) {
							((MDPSimple) mdps.get(i)).clearInitialStates();
							((MDPSimple) mdps.get(i)).addInitialState(currentMDPInitialStates[i]);
						}
						robotsTasksBroken = auctionTasks(currentTaskSet, numRobots, mdps, rewExpr, safetyExpr, mcs);
						mainLog.println("\n\nAssigned Tasks");
						mainLog.println(robotsTasksBroken.toString());

						//lets process these 
						for (int i = 0; i < robotsTasksBroken.size(); i++) {
							if (robotsTasksBroken.get(i).size() == 1) {
								if (robotsTasksBroken.get(i).contains(safetyExpr)) {
									robotsTasksBroken.get(i).remove(0);

								}
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
						//					if (robotsTasksBroken.get(1).size() == 1)
						//						continue;
						getRobotPlansUsingNVINestedProduct(numRobots, robotsTasksBroken, rewExpr, productMDPs, mainLog, mcs, mdps, costRewards, saveplace,
								filename, prism, nviStrategies, finalDAList, costsModels);
						//so now this bit is the bane of my existence really 
						//we have to find a super smart way to create the policy 
						//step 1 we fix a varlist for the joint policy 
						//easily done 
						//step 2 for every reallocation 
						//we've got to find the mapping of the new policy to this reallocation 
						//update it 
						//then go on 
						//it sounds easy samantha 
						//well for the life of me i cant get myself to do it alice 
						//begs the question of liff does it not 
						//indeed liff 

						resultvalues = createJointPolicy(costRewards, finalDAList, mainLog, productMDPs, nviStrategies, saveplace, filename, ssNames, prism);
						//torepeat end 

						processReallocations(numRobots, taskSet, remainingTasks, correspondingMDPInitialStates, productMDPs, mdps, mainLog);
						break;
					}
				}
			}
			return resultvalues;
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	void getCostsModels(int numRobots, ExpressionReward rewExpr, ArrayList<MDPModelChecker> mcs, ArrayList<MDPSimple> mdps,
			ArrayList<MDPRewardsSimple> costsModels) throws PrismException
	{
		for (int rnum = 0; rnum < numRobots; rnum++) {
			RewardStruct costStruct = (rewExpr).getRewardStructByIndexObject(mcs.get(rnum).getModulesFile(),
					mcs.get(rnum).getModulesFile().getConstantValues());
			MDPRewardsSimple costsModel = (MDPRewardsSimple) mcs.get(rnum).constructRewards(mdps.get(rnum), costStruct);
			costsModels.add(costsModel);
		}
	}

	void getRobotPlansUsingNVINestedProduct(int numRobots, ArrayList<ArrayList<Expression>> robotsTasksBroken, ExpressionReward rewExpr,
			ArrayList<MDP> productMDPs, PrismLog mainLog, ArrayList<MDPModelChecker> mcs, ArrayList<MDPSimple> mdps, ArrayList<MDPRewardsSimple> costRewards,
			String saveplace, String filename, Prism prism, ArrayList<MDStrategy> nviStrategies, ArrayList<ArrayList<DAInfo>> finalDAList,
			ArrayList<MDPRewardsSimple> costsModels) throws PrismException
	{
		for (int rnum = 0; rnum < numRobots; rnum++) {
			if (robotsTasksBroken.get(rnum).size() > 0) {
				ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(robotsTasksBroken.get(rnum), mainLog);

				SingleAgentNestedProductMDP res = buildSingleAgentNestedProductMDP("r" + rnum, mdps.get(rnum), daList, null, prism, mcs.get(rnum), mainLog);
				MDPRewardsSimple costsModel = costsModels.get(rnum);
				ArrayList<MDPRewardsSimple> rewards = createMaxExpTaskRewStruct(res, costsModel);

				costRewards.add(rewards.get(1)); //cuz its always 2 for now 

				mainLog.println(res.combinedAcceptingStates.toString());
				mainLog.println(res.combinedEssentialStates.toString());
				mainLog.println(res.combinedStatesToAvoid.toString());
				new MDPCreator().saveMDP(res.finalProduct.getProductModel(), saveplace + "results/"+fnPrefix, filename + "_prod_" + 0, res.combinedAcceptingStates);
				mainLog.println(res.numMDPVars);
				ModelCheckerMultipleResult nviSol = computeNestedValIterFailurePrint(res.finalProduct.getProductModel(), res.combinedAcceptingStates,
						res.combinedStatesToAvoid, rewards, 0, true, prism, mainLog);
				//for each da in daList ;
				//lets see what we have 
				productMDPs.add(res.finalProduct.getProductModel());
				nviStrategies.add(nviSol.strat);
				finalDAList.add(res.daList);
			}

		}
	}

	void processReallocations(int numRobots, ArrayList<Expression> taskSet, Queue<ArrayList<Expression>> remainingTasks,
			Queue<int[]> correspondingMDPInitialStates, ArrayList<MDP> productMDPs, ArrayList<MDPSimple> mdps, PrismLog mainLog) throws PrismException
	{
		while (!possibleReallocStates.isEmpty()) {
			Entry<State, ArrayList<Expression>> reallocStateTaskPair = possibleReallocStates.remove();
			State s = reallocStateTaskPair.getKey();
			ArrayList<Expression> completedTasks = reallocStateTaskPair.getValue();
			ArrayList<Expression> newTaskSet = new ArrayList<Expression>();
			for (int i = 0; i < taskSet.size(); i++) {
				if (!completedTasks.contains(getInnerExpression(taskSet.get(i)))) {
					newTaskSet.add(taskSet.get(i));
				}
			}

			int[] currentStates = jointStatetoRobotStates(numRobots, s, jsToRobotState, productMDPs, mainLog);
			int[] mdpStates = new int[currentStates.length];
			for (int i = 0; i < productMDPs.size(); i++) {
				mdpStates[i] = productStateToMDPState(currentStates[i], productMDPs.get(i), mdps.get(i));

			}

			remainingTasks.add(newTaskSet);
			correspondingMDPInitialStates.add(mdpStates);

		}
	}

	protected double[] resultValues(ModelCheckerMultipleResult res2, MDPSimple mdp)
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
			//			String resString = "";
			for (int i = 0; i < solns.size() - 1; i++) {

				StateValues costsProduct = StateValues.createFromDoubleArray(res2.solns.get(i), mdp);

				double minCost = costsProduct.getDoubleArray()[mdp.getFirstInitialState()];
				//				resString += i + ":" + minCost + " ";
				result[i + 1] = minCost;
			}
			//			mainLog.println("\nFor p = " + maxProb + ", rewards " + resString);

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

	int doJointStateCreationSetup(ArrayList<MDP> productMDPs, int[] currentStates, int numSS, ArrayList<String> ssNames,
			ArrayList<ArrayList<Integer>> daIndices, HashMap<String, ArrayList<Integer>> ssIndices, ArrayList<ArrayList<Integer>> privateIndices)
	{

		int numStateVars = 0;

		for (int i = 0; i < productMDPs.size(); i++) {

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

	ArrayList<HashMap<Integer, DAInfo>> preprocessDALists(ArrayList<ArrayList<DAInfo>> daList)
	{
		ArrayList<HashMap<Integer, DAInfo>> gulonMeinRung = new ArrayList<HashMap<Integer, DAInfo>>();

		for (ArrayList<DAInfo> rdaList : daList) {
			gulonMeinRung.add(preprocessDAList(rdaList));
		}
		return gulonMeinRung;
	}

	HashMap<Integer, DAInfo> preprocessDAList(ArrayList<DAInfo> daList)
	{
		//basically take a dainfo list for a robot
		//and then just give us a hashmap of the same thing with the index as the key 
		HashMap<Integer, DAInfo> daHashMap = new HashMap<Integer, DAInfo>();
		for (DAInfo daInfo : daList) {
			daHashMap.put(daInfo.associatedIndexInProduct, daInfo);
		}
		return daHashMap;

	}

	VarList createJointStateVarList(ArrayList<HashMap<Integer, DAInfo>> daList, int numStateVars,
			//			int numRobots, 
			ArrayList<MDP> productMDPs, int[] currentStates, ArrayList<ArrayList<Integer>> daIndices, HashMap<String, ArrayList<Integer>> ssIndices,
			ArrayList<ArrayList<Integer>> privateIndices, HashMap<Integer, int[]> jsToRobotState, State parentState, HashMap<String, Integer> jsSSIndices)
			throws PrismException
	{
		VarList vl = new VarList();

		//so we do this for each robot really 
		//its easier to do the da first and then the ss then the ps 
		int jsIndex = 0;
		for (int i = 0; i < daIndices.size(); i++) {

			//so we're on the stuff for robot i 
			//so we want the dalist for that robot 
			HashMap<Integer, DAInfo> rDAList = daList.get(i);

			//now lets add these in succession 
			for (int j = 0; j < daIndices.get(i).size(); j++) {
				//so now we want to find the robot in the da list that has the same index as daInd hmmm... trickyyy 
				//so this is a bit of a loop everytime but we could preprocess this so its not 
				//let us 
				//we have fixed it 
				//so now we're chilling 

				int daInd = daIndices.get(i).get(j);
				DAInfo corrDA = rDAList.get(daInd);
				//TODO: do this 
				jsIndex++;

			}

		}
		//ss stuff 

		//		if (ssIndices != null) {
		//			for (String ss : ssIndices.keySet()) {
		//				//to resolve here - race conditions for the shared state 
		//				//so we need an original or reference 
		//
		//				//we have no reference so we just take both
		//				int r = 0;
		//				State rs = robotStateStates.get(r);
		//				int currentJSValue = (int) rs.varValues[ssIndices.get(ss).get(r)];
		//				for (r = 1; r < productMDPs.size(); r++) {
		//					rs = robotStateStates.get(r);
		//					int nextJSValue = (int) rs.varValues[ssIndices.get(ss).get(r)];
		//					if (currentJSValue != nextJSValue) {
		//						if (parentState != null) {
		//							//if they're not the same 
		//							//use reference 
		//							if (nextJSValue != (int) parentState.varValues[jsSSIndices.get(ss)])
		//
		//							{
		//								currentJSValue = nextJSValue;
		//							}
		//						} else {
		//							throw new PrismException("Shared State values aren't the same in the initial state!!");
		//						}
		//					}
		//
		//				}
		//				jointState.setValue(jsIndex, currentJSValue);
		//				if (parentState == null) {
		//					int arr[] = new int[productMDPs.size() + 1];
		//					arr[0] = -1;
		//					ArrayList<Integer> ssInds = ssIndices.get(ss);
		//					for (int i = 0; i < ssInds.size(); i++) {
		//						arr[i + 1] = ssInds.get(i);
		//					}
		//					jsToRobotState.put(jsIndex, arr.clone());
		//					jsSSIndices.put(ss, jsIndex);
		//
		//				}
		//				jsIndex++;
		//			}
		//		}
		//		for (int i = 0; i < privateIndices.size(); i++) {
		//
		//			State rs = robotStateStates.get(i);
		//
		//			//now lets add these in succession 
		//			for (int j = 0; j < privateIndices.get(i).size(); j++) {
		//				int daInd = privateIndices.get(i).get(j);
		//				jointState.setValue(jsIndex, rs.varValues[daInd]);
		//				if (parentState == null) {
		//					int[] arr = new int[2];
		//					arr[0] = i;
		//					arr[1] = daInd;
		//					jsToRobotState.put(jsIndex, arr.clone());
		//				}
		//				jsIndex++;
		//			}
		//
		//		}
		//
		//		//checking if its an accepting state 
		//
		//		if (numAcc == numDAs)
		//			isAcc = true;
		//		if (numEss > 0) {
		//			System.out.println("Essential State: " + jointState.toString());
		//
		//		}
		//		if (isAcc)
		//			System.out.println("Accepting State: " + jointState.toString());
		//		res[0] = jointState;
		//		res[1] = numEss;
		//		res[2] = isAcc;
		//
		//		return res;

		throw new PrismException("Not Implemented");
	}

	Object[] createJointState(ArrayList<HashMap<Integer, DAInfo>> daList, int numStateVars,
			//			int numRobots, 
			ArrayList<MDP> productMDPs, int[] currentStates, ArrayList<ArrayList<Integer>> daIndices, HashMap<String, ArrayList<Integer>> ssIndices,
			ArrayList<ArrayList<Integer>> privateIndices, HashMap<Integer, int[]> jsToRobotState, State parentState, HashMap<String, Integer> jsSSIndices)
			throws PrismException
	{

		Object[] res = new Object[3];
		boolean isEss = false;
		boolean isAcc = false;
		int numAcc = 0;
		int numDAs = 0;
		int numEss = 0;
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

			//so we're on the stuff for robot i 
			//so we want the dalist for that robot 
			HashMap<Integer, DAInfo> rDAList = daList.get(i);
			numDAs += daIndices.get(i).size() - 1;
			State rs = productMDPs.get(i).getStatesList().get(currentStates[i]);
			robotStateStates.add(rs);

			//now lets add these in succession 
			for (int j = 0; j < daIndices.get(i).size(); j++) {
				//so now we want to find the robot in the da list that has the same index as daInd hmmm... trickyyy 
				//so this is a bit of a loop everytime but we could preprocess this so its not 
				//let us 
				//we have fixed it 
				//so now we're chilling 

				int daInd = daIndices.get(i).get(j);
				Object daVal = rs.varValues[daInd];
				if (!rDAList.get(daInd).isSafeExpr) {
					if (rDAList.get(daInd).daAccStates.get((int) daVal)) {

						numAcc++;

						if (parentState != null) {
							if (!rDAList.get(daInd).daAccStates.get((int) parentState.varValues[jsIndex])) {
								if (!isEss) {
									isEss = true;
								}
								numEss++;
							}
						}
					}
				}
				jointState.setValue(jsIndex, daVal);
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

		//checking if its an accepting state 

		if (numAcc == numDAs)
			isAcc = true;
		if (numEss > 0) {
			System.out.println("Essential State: " + jointState.toString());

		}
		if (isAcc)
			System.out.println("Accepting State: " + jointState.toString());
		res[0] = jointState;
		res[1] = numEss;
		res[2] = isAcc;

		return res;

	}

	double[] createJointPolicy(ArrayList<MDPRewardsSimple> costRewards, ArrayList<ArrayList<DAInfo>> daList, PrismLog mainLog, ArrayList<MDP> productMDPs,
			ArrayList<MDStrategy> nviStrategies, String saveplace, String filename, ArrayList<String> ssNames, Prism prism) throws PrismException
	{
		int numRobots = productMDPs.size();

		//things to do here 
		//make the mdp stuff for reallocations too 
		//so that everything is the same 
		//ah but for reallocations the das will have shifted 
		//so we'll need to remember what the das looked like earlier 
		//so I'm going to leave this for tomorrow 
		//today we just focus on stuff without reallocation 

		//saving the acc and ess states 
		BitSet accStates = new BitSet();
		BitSet essStates = new BitSet();

		possibleReallocStates = new LinkedList<Entry<State, ArrayList<Expression>>>();

		ArrayList<ArrayList<Integer>> daIndices = new ArrayList<ArrayList<Integer>>();
		HashMap<String, ArrayList<Integer>> ssIndices = new HashMap<String, ArrayList<Integer>>();
		ArrayList<ArrayList<Integer>> privateIndices = new ArrayList<ArrayList<Integer>>();
		jsToRobotState = new HashMap<Integer, int[]>();
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

		int numStateVars = 0;
		if (doingReallocs) {
			if (mdpCreator == null) //only do this for the first go then 
			{
				mdpCreator = new MDPCreator(mainLog);
				numStateVars = doJointStateCreationSetup(productMDPs, currentStates, numSS, ssNames, daIndices, ssIndices, privateIndices);
			} else {
				//we need a fixed varlist and we need to match that one really 
			}
		} else {
			mdpCreator = new MDPCreator(mainLog);
			numStateVars = doJointStateCreationSetup(productMDPs, currentStates, numSS, ssNames, daIndices, ssIndices, privateIndices);

		}
		ArrayList<HashMap<Integer, DAInfo>> processedDAList = this.preprocessDALists(daList);

		Object[] jsPlusEssAccFlags = createJointState(processedDAList, numStateVars, productMDPs, currentStates, daIndices, ssIndices, privateIndices,
				jsToRobotState, parentState, jsSSIndices);
		State jointState = (State) jsPlusEssAccFlags[0];
		int numEss = (int) jsPlusEssAccFlags[1];
		boolean isAcc = (boolean) jsPlusEssAccFlags[2];
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

				double stateActionCost = 0;

				for (int i = 0; i < numRobots; i++) {
					ArrayList<Integer> nextRobotStates = new ArrayList<Integer>();

					ArrayList<Double> nextRobotStatesProbs = new ArrayList<Double>();

					MDStrategy strat = nviStrategies.get(i);
					MDP mdp = productMDPs.get(i);

					int actionChoice = strat.getChoiceIndex(currentStates[i]);

					//+= cuz sum can change this to not sum 

					if (actionChoice > -1) {
						stateActionCost += costRewards.get(i).getTransitionReward(currentStates[i], actionChoice);
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

						Object[] nextJSPlusEssAccFlags = createJointState(processedDAList, numStateVars, productMDPs, nextStatesCombo, daIndices, ssIndices,
								privateIndices, jsToRobotState, parentState, jsSSIndices);
						State nextJointState = (State) nextJSPlusEssAccFlags[0];
						int numEssNextjs = (int) nextJSPlusEssAccFlags[1];
						boolean isAccNextjs = (boolean) nextJSPlusEssAccFlags[2];
						essAndAcc.add(new Object[] { numEssNextjs, isAccNextjs });
						statesQueues.add(nextJointState);
						//					statesProbQueue.add(comboProb);
						successorsWithProbs.add(new AbstractMap.SimpleEntry<State, Double>(nextJointState, comboProb));
					}
					mdpCreator.addAction(currentJointState, ja, successorsWithProbs, essAndAcc, stateActionCost);

				} else {
					//lets add these to a queue!!! 
					//add the current joint state to the possible realloc queue

					//okay so lets do the essential state check again here 
					//basically we take the processedDAList and the stuff we had to convert it to individual robot states 
					ArrayList<Expression> tasksCompleted = tasksCompletedHere(currentJointState, processedDAList, jsToRobotState);
					//					mainLog.println(tasksCompleted.size());
					possibleReallocStates.add(new AbstractMap.SimpleEntry<State, ArrayList<Expression>>(currentJointState, tasksCompleted));
				}
			}
		} catch (PrismException e) {
			mdpCreator.saveMDP(saveplace + "results/"+fnPrefix, filename + "_jp");
			throw e;
		}
		mdpCreator.saveMDP(saveplace + "results/"+fnPrefix, filename + "_jp");
		mainLog.println(mdpCreator.essStates.toString());
		mainLog.println(mdpCreator.accStates.toString());
		mdpCreator.createRewardStructures();
		mdpCreator.setInitialState(jointState);
		mainLog.println("Goal Prob:" + mdpCreator.getProbabilityToReachAccStateFromJointMDP(jointState));

		//lets just do nvi on this here quickly
		//		ModelCheckerMultipleResult nviSol = computeNestedValIterFailurePrint(res.finalProduct.getProductModel(), res.combinedAcceptingStates,
		//		res.combinedStatesToAvoid, rewards, 0, true, prism, mainLog);
		mdpCreator.mdp.findDeadlocks(true);
		if (mdpCreator.accStates.cardinality() > 0) {
			ModelCheckerMultipleResult nviSol = computeNestedValIterFailurePrint(mdpCreator.mdp, mdpCreator.accStates, new BitSet(),
					mdpCreator.getRewardsInArray(), 0, true, prism, mainLog);
			return resultValues(nviSol, mdpCreator.mdp);
		} else {
			return new double[] { 0.0, 0.0, 0.0 };
		}
		//	}

	}

	ArrayList<Expression> tasksCompletedHere(State jointState, ArrayList<HashMap<Integer, DAInfo>> daList, HashMap<Integer, int[]> jsToRobotState)
	{
		ArrayList<Expression> completedTasks = new ArrayList<Expression>();

		for (int key : jsToRobotState.keySet()) {
			int[] rIndCombo = jsToRobotState.get(key);
			int rnum = rIndCombo[0];
			int indVal = rIndCombo[1];
			//say -1 is for an ss 
			if (rnum > -1) {
				//so jstorobotstate gives us the index 
				//of the index in the robot state 

				if (daList.get(rnum).containsKey(indVal)) {
					DAInfo daInfo = daList.get(rnum).get(indVal);
					if (!daInfo.isSafeExpr) {
						//so if this state corresponds to a da and is in the final state 
						int jsVal = (int) jointState.varValues[key];
						if (daInfo.daAccStates.get(jsVal)) {
							//then we can add it 
							completedTasks.add(daInfo.daExpr);
						}
					}
				}
			}
		}

		return completedTasks;
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

		new SSIAuctionNestedProduct().run();
	}
}
