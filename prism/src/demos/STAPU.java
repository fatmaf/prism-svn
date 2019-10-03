package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import acceptance.AcceptanceType;
import demos.ResultsTiming.varIDs;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.MDPSparse;
import explicit.Model;
import explicit.ModelCheckerMultipleResult;
import explicit.ProbModelChecker;
import explicit.StateValues;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

public class STAPU
{

	static String saveplace_suffix = "/tests/decomp_tests/temp/";
	static String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";

	public STAPU()
	{
		String dir = System.getProperty("user.dir");
		saveplace = dir + saveplace_suffix;
		StatesHelper.setSavePlace(saveplace);
	}

	public static void main(String[] args)
	{

		
		STAPU stapu = new STAPU();
//		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
				String dir= "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSI/";
		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";

		String resString = "";

		numRobots = 2;
		numFS = 0;//5;//1;
		numGoals = 3;//6;//4;
		numDoors = 2;//2;
		fn = "andar_r2_g3_d2_fs0";

//
//		numRobots = 8;
//		numFS = 0;//5;//1;
//		numGoals = 10;//6;//4;
//		numDoors = 4;//2;
//		fn = "andar_r8_g10_d4_fs0";

		int maxRobots = 2; 
		int maxGoals = 3; 
		
		stapu.runGUISimpleTestsOne(dir, fn, maxRobots, numFS, maxGoals, numDoors, false,null,null);


	}

	// long timeout = 100 * 60 * 1000;
	public PrismLog mainLog;
	Prism prismC;
	//	MMDPSimple jointPolicy;

	boolean hasDoor = true;
	ResultsTiming resSaver;

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
			ProbModelChecker mcProb, ModulesFile modulesFile) throws PrismException
	{
		// return the list of daInfo and the product mdp

		SingleAgentNestedProductMDP res = new SingleAgentNestedProductMDP(mainLog);
		res.setNumMDPVars(model.getVarList().getNumVars());
		res.initializeProductToMDPStateMapping((MDP) model);

		LTLProduct<MDP> product = null;
		MDP productMDP = null;
		LTLModelChecker mcLTL = new LTLModelChecker(mcProb); // is this okay ?
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };

		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		// all the states are states of interest
		bsInit.set(0, numStates);
		productMDP = (MDP) model;
		res.daList = new ArrayList<DAInfo>();

		for (int daNum = 0; daNum < daList.size(); daNum++) {
			DAInfo daInfo = new DAInfo(daList.get(daNum));

			product = daInfo.constructDAandProductModel(mcLTL, mcProb, modulesFile, allowedAcceptance, productMDP, null, true);
			daInfo.associatedIndexInProduct++; //should go to zero from -1 

			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);

			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				res.daList.get(otherDAs).updateStateNumbers(product);
				res.daList.get(otherDAs).associatedIndexInProduct++; //and everyone else also gets shifted once. 

				//								StatesHelper.saveBitSet(res.daList.get(otherDAs).essentialStates, "",
				//										name + "pda_" + daNum + "_" + otherDAs + ".ess", true);
				//								StatesHelper.saveBitSet(res.daList.get(otherDAs).productAcceptingStates, "",
				//										name + "pda_" + daNum + "_" + otherDAs + ".acc", true);
			}
			//						StatesHelper.saveHashMap(res.productStateToMDPState, "",
			//								name + "pda_" + daNum + "_before_productStateToMDPState.txt", true);
			res.updateProductToMDPStateMapping(product);
			//						StatesHelper.saveHashMap(res.productStateToMDPState, "",
			//								name + "pda_" + daNum + "_after_productStateToMDPState.txt", true);
			res.daList.add(daInfo);
		}
		DAInfo daInfo = res.daList.get(res.daList.size() - 1);
		int daNum = res.daList.size() - 1;
		//		StatesHelper.saveDA(daInfo.da, "", name + "da_" + daNum, true);
		//		StatesHelper.saveMDP(productMDP, daInfo.productAcceptingStates, "", name + "pda_" + daNum, true);
		//		StatesHelper.saveMDP(productMDP, daInfo.essentialStates, "", name + "pda_" + daNum + "switchStates", true);
		//		StatesHelper.saveMDPstatra(productMDP, "", name + "pda_" + daNum + "sta_tra", true);

		res.setDAListAndFinalProduct(product);
		return res;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,
			ArrayList<Boolean> minRewards, int probPreference) throws PrismException
	{

		ModelCheckerMultipleResult res2 = computeNestedValIterFailurePrint(mdp, target, statesToAvoid, rewards, minRewards, probPreference, null);// computeNestedValIterFailurePrint(mdp, target, statesToAvoid,
		// rewards,minRewards,target,probPreference,null);

		return res2;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,
			ArrayList<Boolean> minRewards, int probPreference, double[] probInitVal) throws PrismException
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
			int probPreference, boolean doMaxTasks) throws PrismException
	{

		ArrayList<Boolean> minMaxRew = new ArrayList<Boolean>();
		int rewinit = 0;
		if (doMaxTasks) {
			minMaxRew.add(false);
		}
		for (int rew = rewinit; rew < rewards.size(); rew++)
			minMaxRew.add(true);
		return computeNestedValIterFailurePrint(mdp, target, statesToAvoid, rewards, minMaxRew, probPreference);
	}

	protected double[] doSTAPULimitGoals(ArrayList<Model> models, ExpressionFunc expr, BitSet statesOfInterest, ProbModelChecker mcProb,
			ArrayList<ModulesFile> modulesFiles, ArrayList<String> shared_vars_list, boolean includefailstatesinswitches, boolean matchSharedVars,
			boolean completeSwitchRing, int numGoals, boolean noReallocs, ArrayList<Integer> goalNumbers) throws PrismException
	{

		resSaver.recordTime("total models loading time", varIDs.totalmodelloadingtime, false);
		resSaver.setLocalStartTime();

		int probPreference = 0;

		// process ltl expressions
		int numRobots = models.size();// getNumRobots(exampleNumber());
		boolean sameModelForAll = false;
		if (numRobots != models.size() && models.size() == 1)
			sameModelForAll = true;
		else
			numRobots = models.size();

		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = new ArrayList<SingleAgentNestedProductMDP>();
		Entry<ArrayList<Expression>,ExpressionReward> ltlExpressionsAndExprRew = getLTLExpressionsLimit(expr, numGoals,goalNumbers);
		ArrayList<Expression> ltlExpressions = ltlExpressionsAndExprRew.getKey(); 
		ExpressionReward rewToAttach = ltlExpressionsAndExprRew.getValue(); 
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions,rewToAttach);

		
		//record num tasks here
		resSaver.recordInits(daList.size(), "Num Tasks", varIDs.numtasks);

		Model model = models.get(0);
		ModulesFile modulesFile = modulesFiles.get(0);
		resSaver.setScopeStartTime();

		for (int i = 0; i < numRobots; i++) {
			if (!sameModelForAll) {

				model = models.get(i);
				modulesFile = modulesFiles.get(i);
			}

			int initState = model.getFirstInitialState();

			if (i != 0 && sameModelForAll) {
				initState = getInitState(i, exampleNumber());
				((MDPSparse) model).clearInitialStates();
				((MDPSparse) model).addInitialState(initState);

			}

			SingleAgentNestedProductMDP nestedProduct = buildSingleAgentNestedProductMDP("" + i, model, daList, statesOfInterest, mcProb, modulesFile);

			singleAgentProductMDPs.add(nestedProduct);
			resSaver.recordTime("Nested Product Time", varIDs.nestedproducttimes, true);
			resSaver.recordValues(nestedProduct.finalProduct.getProductModel().getNumStates(), "Nested Product States", varIDs.nestedproductstates);

		}

		resSaver.recordTime("Total Single Agent Nested Product Time", varIDs.allnestedproductcreationtime, false);
		resSaver.setScopeStartTime();

		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP = new SequentialTeamMDP(this.mainLog, numRobots, matchSharedVars); // buildSequentialTeamMDPTemplate(singleAgentProductMDPs);

		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs, shared_vars_list);

		int firstRobot = 0; // fix this

		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing);

		resSaver.recordTime("Team MDP Time", varIDs.teammdptimeonly, true);

		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++) {
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));

		}

		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>(seqTeamMDP.rewardsWithSwitches);
		ArrayList<Boolean> minRewards = new ArrayList<Boolean>();
		for (int rew = 0; rew < rewards.size(); rew++) {
			minRewards.add(true);
		}
		rewards.add(0, seqTeamMDP.progressionRewards);

		minRewards.add(0, false);

		combinedEssentialStates.or(seqTeamMDP.acceptingStates);

		//		mainLog.println("Team MDP essential states: "+combinedEssentialStates.toString());
		resSaver.recordTime("Team MDP Time (including single agent time)", varIDs.totalteammdpcreationtime, false);
		resSaver.recordValues(seqTeamMDP.teamMDPWithSwitches.getNumStates(), "Team MDP States", varIDs.teammdpstates);
		resSaver.recordValues(seqTeamMDP.teamMDPWithSwitches.getNumTransitions(), "Team MDP Transitions", varIDs.teammdptransitions);

		//		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates, "", "teamMDPWithSwitches", true);
		//		StatesHelper.saveMDPstatra(seqTeamMDP.teamMDPWithSwitches,  "", "teamMDPWithSwitches", true);

		resSaver.setLocalStartTime();
		resSaver.setScopeStartTime();
		ModelCheckerMultipleResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
				seqTeamMDP.statesToAvoid, rewards, minRewards, probPreference);// ,probInitVals);
		resSaver.recordTime("First Solution", varIDs.reallocations, true);

		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();

		mainLog.println("InitState = " + initialState);

		// *************************************************************//
		// testing the new joint policy stuff
		// this is a build as you go along kind of thing
		// so it doesn't really work right now okay
		// cuz soy un perdedor (sp?) i'm a loser baybay:P so why dont you kill me

		resSaver.setScopeStartTime();
		JointPolicyBuilder jointPolicyBuilder = new JointPolicyBuilder(seqTeamMDP.numRobots, seqTeamMDP.agentMDPs.get(0).daList.size(), shared_vars_list,
				seqTeamMDP.teamMDPTemplate.getVarList(), rewards, mainLog);

		jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP, initialState);
		resSaver.recordTime("Joint Policy Creationg Time", varIDs.jointpolicycreation, true);
		// *************************************************************//
		// while failedstatesQ is not empty
		// statextended stateToExplore = failedStatesQ.remove()
		// State stateToExploreState =jointPolicyBuilder.getStateState(stateToExplore)
		// Get initial states from this state
		// update initial states for team mdp
		// update switches
		// solve
		// add to joint policy
		if (!noReallocs) {
			int numPlanning = 1;
			while (jointPolicyBuilder.hasFailedStates()) {

				Entry<State, BitSet> stateToExploreAndBitSet = jointPolicyBuilder.getNextFailedState();
				State stateToExplore = stateToExploreAndBitSet.getKey();
				BitSet statesToAvoid = stateToExploreAndBitSet.getValue();

				if (!jointPolicyBuilder.inStatesExplored(stateToExplore)) {

					// get first failed robot
					numPlanning++;
					resSaver.recordValues(numPlanning, "Realloc", varIDs.numreallocationsincode);
					resSaver.setScopeStartTime();

					int[] robotStates = jointPolicyBuilder.extractIndividualRobotStatesFromJointState(stateToExplore,
							seqTeamMDP.teamMDPWithSwitches.getStatesList(), seqTeamMDP.teamMDPWithSwitches.getVarList());
					firstRobot = jointPolicyBuilder.getFirstFailedRobotFromRobotStates(robotStates, seqTeamMDP.teamMDPWithSwitches);

					seqTeamMDP.setInitialStates(robotStates);
					seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing);

					if (statesToAvoid == null)
						statesToAvoid = seqTeamMDP.statesToAvoid;
					else {

						statesToAvoid.or(seqTeamMDP.statesToAvoid);
					}

					solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates, statesToAvoid, rewards, minRewards,
							probPreference);// ,probInitVals);

					resSaver.recordTime("Solution Time", varIDs.reallocations, true);
					resSaver.setScopeStartTime();
					jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP.teamMDPWithSwitches, stateToExplore);
					resSaver.recordTime("Joint Policy Time", varIDs.jointpolicycreation, true);
				}
			}

			resSaver.recordTime("All Reallocations", varIDs.allreallocationstime, false);

			//		jointPolicyBuilder.saveJointPolicyMDP();
			mainLog.println("All done");
			mainLog.println("NVI done " + numPlanning + " times");
			jointPolicyBuilder.printStatesExploredOrder();
		}
		resSaver.saveJointPolicy(jointPolicyBuilder);
		mainLog.println(jointPolicyBuilder.accStates.toString());
		HashMap<String, Double> values = new HashMap<String, Double>();
		values.put("prob", jointPolicyBuilder.getProbabilityOfSatisfactionFromInitState());
		jointPolicyBuilder.createRewardStructures();
		ArrayList<MDPRewardsSimple> finalRewards = jointPolicyBuilder.getExpTaskAndCostRewards();
		jointPolicyBuilder.jointMDP.findDeadlocks(true);
		ModelCheckerMultipleResult result = computeNestedValIterFailurePrint(jointPolicyBuilder.jointMDP, jointPolicyBuilder.accStates, new BitSet(),
				finalRewards, minRewards, probPreference);
		resSaver.saveValues(values);
		mainLog.println("All done");
		return resultValues(result, jointPolicyBuilder.jointMDP);

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

	protected void doSTAPU(ArrayList<Model> models, ExpressionFunc expr, BitSet statesOfInterest, ProbModelChecker mcProb, ArrayList<ModulesFile> modulesFiles,
			ArrayList<String> shared_vars_list, boolean includefailstatesinswitches, boolean matchSharedVars, boolean completeSwitchRing) throws PrismException
	{

		resSaver.recordTime("total models loading time", varIDs.totalmodelloadingtime, false);
		resSaver.setLocalStartTime();

		int probPreference = 0;

		// process ltl expressions
		int numRobots = models.size();// getNumRobots(exampleNumber());
		boolean sameModelForAll = false;
		if (numRobots != models.size() && models.size() == 1)
			sameModelForAll = true;
		else
			numRobots = models.size();

		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = new ArrayList<SingleAgentNestedProductMDP>();
		ArrayList<Expression> ltlExpressions = getLTLExpressions(expr);
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions,null);

		//record num tasks here
		resSaver.recordInits(daList.size(), "Num Tasks", varIDs.numtasks);

		Model model = models.get(0);
		ModulesFile modulesFile = modulesFiles.get(0);
		resSaver.setScopeStartTime();

		for (int i = 0; i < numRobots; i++) {
			if (!sameModelForAll) {
				model = models.get(i);
				modulesFile = modulesFiles.get(i);
			}

			int initState = model.getFirstInitialState();

			if (i != 0 && sameModelForAll) {
				initState = getInitState(i, exampleNumber());
				((MDPSparse) model).clearInitialStates();
				((MDPSparse) model).addInitialState(initState);

			}

			SingleAgentNestedProductMDP nestedProduct = buildSingleAgentNestedProductMDP("" + i, model, daList, statesOfInterest, mcProb, modulesFile);

			singleAgentProductMDPs.add(nestedProduct);
			resSaver.recordTime("Nested Product Time", varIDs.nestedproducttimes, true);
			resSaver.recordValues(nestedProduct.finalProduct.getProductModel().getNumStates(), "Nested Product States", varIDs.nestedproductstates);

		}

		resSaver.recordTime("Total Single Agent Nested Product Time", varIDs.allnestedproductcreationtime, false);
		resSaver.setScopeStartTime();

		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP = new SequentialTeamMDP(this.mainLog, numRobots, matchSharedVars); // buildSequentialTeamMDPTemplate(singleAgentProductMDPs);

		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs, shared_vars_list);

		int firstRobot = 0; // fix this

		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing);

		resSaver.recordTime("Team MDP Time", varIDs.teammdptimeonly, true);

		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++) {
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));

		}

		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>(seqTeamMDP.rewardsWithSwitches);
		ArrayList<Boolean> minRewards = new ArrayList<Boolean>();
		for (int rew = 0; rew < rewards.size(); rew++) {
			minRewards.add(true);
		}
		rewards.add(0, seqTeamMDP.progressionRewards);

		minRewards.add(0, false);

		combinedEssentialStates.or(seqTeamMDP.acceptingStates);

		resSaver.recordTime("Team MDP Time (including single agent time)", varIDs.totalteammdpcreationtime, false);
		resSaver.recordValues(seqTeamMDP.teamMDPWithSwitches.getNumStates(), "Team MDP States", varIDs.teammdpstates);
		resSaver.recordValues(seqTeamMDP.teamMDPWithSwitches.getNumTransitions(), "Team MDP Transitions", varIDs.teammdptransitions);

		resSaver.setLocalStartTime();
		resSaver.setScopeStartTime();
		ModelCheckerMultipleResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
				seqTeamMDP.statesToAvoid, rewards, minRewards, probPreference);// ,probInitVals);
		resSaver.recordTime("First Solution", varIDs.reallocations, true);

		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		mainLog.println("InitState = " + initialState);

		// *************************************************************//
		// testing the new joint policy stuff
		// this is a build as you go along kind of thing
		// so it doesn't really work right now okay
		// cuz soy un perdedor (sp?) i'm a loser baybay:P so why dont you kill me

		resSaver.setScopeStartTime();
		JointPolicyBuilder jointPolicyBuilder = new JointPolicyBuilder(seqTeamMDP.numRobots, seqTeamMDP.agentMDPs.get(0).daList.size(), shared_vars_list,
				seqTeamMDP.teamMDPTemplate.getVarList(), rewards, mainLog);

		jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP, initialState);
		resSaver.recordTime("Joint Policy Creationg Time", varIDs.jointpolicycreation, true);
		// *************************************************************//
		// while failedstatesQ is not empty
		// statextended stateToExplore = failedStatesQ.remove()
		// State stateToExploreState =jointPolicyBuilder.getStateState(stateToExplore)
		// Get initial states from this state
		// update initial states for team mdp
		// update switches
		// solve
		// add to joint policy
		int numPlanning = 1;
		while (jointPolicyBuilder.hasFailedStates()) {

			Entry<State, BitSet> stateToExploreAndBitSet = jointPolicyBuilder.getNextFailedState();
			State stateToExplore = stateToExploreAndBitSet.getKey();
			BitSet statesToAvoid = stateToExploreAndBitSet.getValue();

			if (!jointPolicyBuilder.inStatesExplored(stateToExplore)) {

				// get first failed robot
				numPlanning++;
				resSaver.recordValues(numPlanning, "Realloc", varIDs.numreallocationsincode);
				resSaver.setScopeStartTime();

				int[] robotStates = jointPolicyBuilder.extractIndividualRobotStatesFromJointState(stateToExplore,
						seqTeamMDP.teamMDPWithSwitches.getStatesList(), seqTeamMDP.teamMDPWithSwitches.getVarList());
				firstRobot = jointPolicyBuilder.getFirstFailedRobotFromRobotStates(robotStates, seqTeamMDP.teamMDPWithSwitches);

				seqTeamMDP.setInitialStates(robotStates);
				seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing);

				if (statesToAvoid == null)
					statesToAvoid = seqTeamMDP.statesToAvoid;
				else {

					statesToAvoid.or(seqTeamMDP.statesToAvoid);
				}

				solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates, statesToAvoid, rewards, minRewards,
						probPreference);// ,probInitVals);

				resSaver.recordTime("Solution Time", varIDs.reallocations, true);
				resSaver.setScopeStartTime();
				jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP.teamMDPWithSwitches, stateToExplore);
				resSaver.recordTime("Joint Policy Time", varIDs.jointpolicycreation, true);
			}
		}
		resSaver.recordTime("All Reallocations", varIDs.allreallocationstime, false);
		resSaver.saveJointPolicy(jointPolicyBuilder);
		//		jointPolicyBuilder.saveJointPolicyMDP();
		mainLog.println("All done");
		mainLog.println("NVI done " + numPlanning + " times");
		jointPolicyBuilder.printStatesExploredOrder();
		mainLog.println("All done");

	}

	private int getInitState(int robotNum, int robotModel)
	{
		int initState = -1;

		if (robotModel == 0)
			initState = 2;// hardcoding this realy
		else if (robotModel == 1) {
			if (robotNum == 1)
				initState = 3;
			if (robotNum == 2)
				initState = 5;
			if (robotNum == 3)
				initState = 13;
		} else if (robotModel == 2) {
			if (robotNum == 0) {
				initState = 2;
			}
			if (robotNum == 1)
				initState = 7;
			if (robotNum == 2)
				initState = 8;
		} else if (robotModel == 3) {
			if (robotNum == 0)
				initState = 6;
			if (robotNum == 1)
				initState = 2;
			if (robotNum == 2)
				initState = 3;
		} else if (robotModel == 4) {
			int initPoses[] = { 4, 8, 21, 28, 27, 17, 22, 20, 24, 26, 1, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 18, 19, 23, 25, 29, 30 };
			// int notHave = 2;

			initState = initPoses[robotNum] + 1;

			// 5 -4
			// 29-8
			// 26-21
			// 24-28
			// 11-27
			// 10-17
			// 12-22
			// 13-21
			// 14-20
			// 18-24
			// 21-26

		} else if (robotModel == 5) {
			if (robotNum == 0)
				initState = 1;
			if (robotNum == 1)
				initState = 4;
			if (robotNum == 2)
				initState = 7;

		} else if (robotModel == 6) {
			switch (robotNum) {
			case 0:
				initState = 1;
				break;
			case 1:
				initState = 4;
				break;
			}
		}
		///////////////////////////////////// DECIDE Robot init states
		///////////////////////////////////// HERE///////////////////////////////////////
		return initState;
	}

	/**
	 * Return a list of expressions
	 */
	protected ArrayList<Expression> getLTLExpressions(ExpressionFunc expr) throws PrismException
	{
		int numOp = expr.getNumOperands();
		String exprString = "";
		ExpressionFunc conjunctionOfExpressions = null;
		ArrayList<Expression> ltlExpressions = new ArrayList<Expression>(numOp);
		for (int exprNum = 0; exprNum < numOp; exprNum++) {
			if (expr.getOperand(exprNum) instanceof ExpressionQuant) {
				ltlExpressions.add((expr.getOperand(exprNum)));
				exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
				if (Expression.isCoSafeLTLSyntactic(expr.getOperand(exprNum))) {
					if (conjunctionOfExpressions == null) {
						conjunctionOfExpressions = new ExpressionFunc(((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString());
					}
				}
			}
		}

		mainLog.println("LTL Mission: " + exprString);
		return ltlExpressions;
	}

	/**
	 * Return a list of expressions
	 */
	protected Entry<ArrayList<Expression>,ExpressionReward> getLTLExpressionsLimit(ExpressionFunc expr, int lim, ArrayList<Integer> goalNumbers) throws PrismException
	{
		ExpressionReward exprRew = null;
		int numOp = expr.getNumOperands();
		//lets get one reward structure 
		ArrayList<Integer> rewardFuncIndices = new ArrayList<Integer>(); 
		String exprString = "";
		ExpressionFunc conjunctionOfExpressions = null;
		ArrayList<Expression> ltlExpressions = new ArrayList<Expression>(numOp);
		for (int exprNum = 0; exprNum < numOp; exprNum++) {
			if (expr.getOperand(exprNum) instanceof ExpressionQuant) {
				ltlExpressions.add((expr.getOperand(exprNum)));
				exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
				if (Expression.isCoSafeLTLSyntactic(expr.getOperand(exprNum))) {
					if (conjunctionOfExpressions == null) {
						conjunctionOfExpressions = new ExpressionFunc(((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString());
					}
				}
			}
			if(expr.getOperand(exprNum) instanceof ExpressionReward)
				rewardFuncIndices.add(exprNum);

		}

		boolean noRewardIndices = true; 
		//limiting the expressions 
		ArrayList<Expression> ltlExpressionsLimited = new ArrayList<Expression>(lim);
		exprString = "";
		if (goalNumbers == null) {
			for (int exprNum = 0; exprNum < numOp; exprNum++) {
				if(noRewardIndices && rewardFuncIndices.contains(exprNum))
					noRewardIndices = false; 
				Expression current = ltlExpressions.get(exprNum);
				if (exprNum < (lim - 1)) {
					exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
					ltlExpressionsLimited.add(current);
				}
				if (exprNum == (numOp - 1)) {
					exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
					ltlExpressionsLimited.add(current);
				}
			}
		} else {
			for (int exprNum : goalNumbers) {
				if(noRewardIndices && rewardFuncIndices.contains(exprNum))
					noRewardIndices = false; 
				Expression current = ltlExpressions.get(exprNum);

				exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
				ltlExpressionsLimited.add(current);

			}
			exprString += ((ExpressionQuant) ltlExpressions.get(numOp - 1)).getExpression().toString();
			ltlExpressionsLimited.add(ltlExpressions.get(numOp - 1));

		}

		if(noRewardIndices)
		{
			//just pick a reward expression
			//any 
			if(rewardFuncIndices.size()>0)
			exprRew =(ExpressionReward) expr.getOperand(rewardFuncIndices.get(0)); 
			else
				throw new PrismException("No reward expression!!!");
		}
		mainLog.println("LTL Mission: " + exprString);
		return new AbstractMap.SimpleEntry<ArrayList<Expression>,ExpressionReward>(ltlExpressionsLimited,exprRew);
	}

	protected ArrayList<DAInfo> initializeDAInfoFromLTLExpressions(ArrayList<Expression> exprs,ExpressionReward rewToAttach)
	{
		//if no one has their own reward 
		//we're going to attach the reward to the first DA that is not a safety condition 
		boolean rewardAttached = false; 
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
			if(rewToAttach !=null && !daInfo.isSafeExpr &&  !rewardAttached)
			{
				daInfo.daExprRew = rewToAttach; 
				rewardAttached = true; 
			}
			daInfoList.add(daInfo);
		}

		return daInfoList;
	}

	public double[] runGUISimpleTestsOne(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors, boolean noReallocations,
			ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers)
	{
		double[] res = null;
		// saving filenames etc
		//		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		//System.getProperty("user.dir");
		String modelLocation = dir;
		StatesHelper.setSavePlace(modelLocation + "results/");
		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_goals_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list = new HashMap<String, Integer>();
		ArrayList<String> examples = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();

		//		int numRobots = 2;
		//		int numFS = 1;
		//		int numGoals = 4;
		//		int numDoors = 2;
		//simpleTests/g5_r3_t3_d0_fs0.png  simpleTests/g5_r3_t3_d0_fs3.png  simpleTests/g5_r3_t3_d2_fs3.png
		String example = fn;//"g5_r2_t3_d2_fs1";//"g5_r3_t3_d0_fs0";//"test_grid_nodoors_nofs";
		String example_id = example;//example + "r" + numRobots;//cumberland_doors; 
		String example_to_run = example;//cumberland_doors; 

		//		numRobots = 4;
		//		numFS = 8;
		//		numGoals = 7;
		//		numDoors = 4;
		//		//simpleTests/g5_r3_t3_d0_fs0.png  simpleTests/g5_r3_t3_d0_fs3.png  simpleTests/g5_r3_t3_d2_fs3.png
		//		example = "g10_r4_t6_d4_fs8";//"g5_r3_t3_d0_fs0";//"test_grid_nodoors_nofs";
		//		example_id = example;//example + "r" + numRobots;//cumberland_doors; 
		//		example_to_run = example;//cumberland_doors; 

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

		for (int i = 0; i < examples.size(); i++) {
			example_to_run = examples.get(i);
			example_id = example_ids.get(i);

			int maxRobots = example_num_robot_list.get(example_id);
			int maxGoals = example_num_goals_list.get(example_id);
			try {

				res = runOneExampleNumRobotsGoals(example_to_run, example_id, example_has_door_list, example_num_door_list, maxRobots, maxGoals,
						example_num_fs_list, modelLocation, true, dir + "results/stapu", noReallocations,robotNumbers,goalNumbers);

			} catch (FileNotFoundException e) {
				e.printStackTrace();

				System.out.println("Error: " + e.getMessage());
				//						System.exit(1);
			} catch (PrismException e) {
				e.printStackTrace();

				System.out.println("Error: " + e.getMessage());
				//						System.exit(1);
			} catch (Exception e) {
				e.printStackTrace();
				System.out.println("Error: " + e.getMessage());
			}

		}
		return res;

	}


	public double[] runOneExampleNumRobotsGoals(String example_name, String example_id, HashMap<String, Boolean> example_has_door_list,

			HashMap<String, Integer> example_num_door_list, int numRobots, int numGoals, HashMap<String, Integer> example_num_fs_list, String modelLocation,
			boolean doorVarNameHas0, String resLoc, boolean noReallocs, ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers)

			throws PrismException, FileNotFoundException
	{
		double[] res = null;

		//Setting up
		String filename = example_name;

		boolean example_has_door = example_has_door_list.get(example_id);

		String filename_suffix = "";

		//these affect the switch transitions and hence the solution 
		boolean includefailstatesinswitches = false;
		boolean matchsharedstatesinswitch = false;
		boolean completeSwitchRing = false;

		ArrayList<String> shared_vars_list = new ArrayList<String>();
		int numdoors = 0;
		if (example_has_door) {
			numdoors = example_num_door_list.get(example_id);
			if (!doorVarNameHas0) {
				if (numdoors == 1)
					shared_vars_list.add("door");
				else {
					for (int i = 0; i < numdoors; i++) {
						shared_vars_list.add("door" + i);

					}
				}
			} else {
				for (int i = 0; i < numdoors; i++) {
					shared_vars_list.add("door" + i);

				}
			}
		}

		ArrayList<String> filenames = new ArrayList<String>();

		int fnNumber = 0;
		for (int i = 0; i < numRobots; i++) {
			if (robotNumbers != null)
				fnNumber = robotNumbers.get(i);
			else
				fnNumber = i;
			filenames.add(filename + fnNumber);
		}
		StatesHelper.setFolder(modelLocation + filename);
		if (shared_vars_list.size() <= 0)
			hasDoor = false;
		else
			hasDoor = true;
		int maxMDPVars = 0;

		ArrayList<Model> models = new ArrayList<Model>();
		ArrayList<PropertiesFile> propFiles = new ArrayList<PropertiesFile>();
		ArrayList<ModulesFile> modulesFiles = new ArrayList<ModulesFile>();

		// Create a log for PRISM output (hidden or stdout)
		// PrismLog mainLog = new PrismDevNullLog();
		PrismLog mainLog = new PrismFileLog("stdout");
		this.mainLog = mainLog;
		StatesHelper.mainLog = mainLog;

		resSaver = new ResultsTiming(mainLog, filename, StatesHelper.getLocation(), false, "stapu");

		//record num robots and doors 
		resSaver.recordInits(numRobots, "Robots", varIDs.numrobots);
		resSaver.recordInits(numdoors, "Doors", varIDs.numdoors);

		int numFS = example_num_fs_list.get(example_id);
		resSaver.recordInits(numFS, "FS", varIDs.failstates);

		// Initialise PRISM engine
		Prism prism = new Prism(mainLog);
		prismC = prism;
		prism.initialise();

		//loading models 
		resSaver.setGlobalStartTime();
		resSaver.setLocalStartTime();

		for (int files = 0; files < filenames.size(); files++) {
			resSaver.setScopeStartTime();
			// Parse and load a PRISM model from a file
			ModulesFile modulesFile = prism.parseModelFile(new File(modelLocation + filenames.get(files) + ".prism"));
			modulesFiles.add(modulesFile);
			prism.loadPRISMModel(modulesFile);

			// Parse and load a properties model for the model
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(modelLocation + filename + filename_suffix + ".prop"));

			// Get PRISM to build the model and then extract it
			prism.setEngine(Prism.EXPLICIT);
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit();
			int numVars = mdp.getVarList().getNumVars();
			if (numVars > maxMDPVars)
				maxMDPVars = numVars;

			models.add(mdp);
			propFiles.add(propertiesFile);
			resSaver.recordTime("model loading time " + files, varIDs.modelloadingtimes, true);
		}

		// Model check the first property from the file using the model checker
		for (int i = 0; i < propFiles.size(); i++) {
			mainLog.println(propFiles.get(i).getPropertyObject(0));
		}
		Expression expr = propFiles.get(0).getProperty(0);

		//		ExecutorService executor = Executors.newSingleThreadExecutor();
		StatesHelper.setNumMDPVars(maxMDPVars);
		//		Runnable task = new Runnable()
		//		{
		//			@Override
		//			public void run()
		//			{
		//				// do your task
		//				try {
		res = doSTAPULimitGoals(models, (ExpressionFunc) expr, null, new ProbModelChecker(prism), modulesFiles, shared_vars_list, includefailstatesinswitches,
				matchsharedstatesinswitch, completeSwitchRing, numGoals, noReallocs, goalNumbers);

		//				} catch (PrismException e) {
		//					// TODO Auto-generated catch block
		//
		//					e.printStackTrace();
		//					//					System.exit(1);
		//				}
		//			}
		//		};
		//
		//		Future<?> future = executor.submit(task);
		//
		//		try {
		//			future.get(resSaver.timeout, TimeUnit.MILLISECONDS);
		//
		//		} catch (InterruptedException e) {
		//			// TODO Auto-generated catch block
		//			e.printStackTrace();
		//			//			System.exit(1);
		//		} catch (ExecutionException e) {
		//			// TODO Auto-generated catch block
		//			e.printStackTrace();
		//			//			System.exit(1);
		//		} catch (TimeoutException e) {
		//			mainLog.println("Timed out - " + TimeUnit.SECONDS.convert(resSaver.timeout, TimeUnit.MILLISECONDS) + " seconds, "
		//					+ TimeUnit.MINUTES.convert(resSaver.timeout, TimeUnit.MILLISECONDS) + " mins");
		//			// TODO Auto-generated catch block
		//			//			if (jointPolicy != null)
		//			//				mainLog.println("States " + jointPolicy.allFailStatesSeen.toString());
		//			e.printStackTrace();
		//			//			System.exit(1);
		//		} // awaits termination

		resSaver.writeResults();
		// Close down PRISM
		prism.closeDown();
		return res;

	}


	public int exampleNumber()
	{
		return 5;
	}

}
