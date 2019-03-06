package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import acceptance.AcceptanceType;
import cern.colt.Arrays;
import demos.ResultsTiming.varIDs;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSparse;
import explicit.Model;
import explicit.ModelCheckerMultipleResult;
import explicit.ModelCheckerResult;
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

public class STAPU {

	static String saveplace_suffix = "/tests/decomp_tests/temp/";
	static String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";

	public static void main(String[] args) {
		String dir = System.getProperty("user.dir");
		saveplace = dir + saveplace_suffix;
		StatesHelper.setSavePlace(saveplace);

		STAPU stapu = new STAPU();
		stapu.run();
	}

	// long timeout = 100 * 60 * 1000;
	public PrismLog mainLog;
	Prism prismC;
	MMDPSimple jointPolicy;

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
	protected SingleAgentNestedProductMDP buildSingleAgentNestedProductMDP(String name, Model model,
			ArrayList<DAInfo> daList, BitSet statesOfInterest, ProbModelChecker mcProb, ModulesFile modulesFile)
			throws PrismException {
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

			product = daInfo.constructDAandProductModel(mcLTL, mcProb, modulesFile, allowedAcceptance, productMDP, null,
					true);
			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);

			StatesHelper.saveDA(daInfo.da, "", name + "da_" + daNum, true);
			StatesHelper.saveMDP(productMDP, daInfo.productAcceptingStates, "", name + "pda_" + daNum, true);
			StatesHelper.saveMDP(productMDP, daInfo.essentialStates, "", name + "pda_" + daNum + "switchStates", true);
			StatesHelper.saveMDPstatra(productMDP, "", name + "pda_" + daNum + "sta_tra", true);

			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				res.daList.get(otherDAs).updateStateNumbers(product);
				StatesHelper.saveBitSet(res.daList.get(otherDAs).essentialStates, "",
						name + "pda_" + daNum + "_" + otherDAs + ".ess", true);
				StatesHelper.saveBitSet(res.daList.get(otherDAs).productAcceptingStates, "",
						name + "pda_" + daNum + "_" + otherDAs + ".acc", true);
			}
			StatesHelper.saveHashMap(res.productStateToMDPState, "",
					name + "pda_" + daNum + "_before_productStateToMDPState.txt", true);
			res.updateProductToMDPStateMapping(product);
			StatesHelper.saveHashMap(res.productStateToMDPState, "",
					name + "pda_" + daNum + "_after_productStateToMDPState.txt", true);
			res.daList.add(daInfo);
		}
		res.setDAListAndFinalProduct(product);
		return res;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid,
			ArrayList<MDPRewardsSimple> rewards, ArrayList<Boolean> minRewards, int probPreference)
			throws PrismException {

		ModelCheckerMultipleResult res2 = computeNestedValIterFailurePrint(mdp, target, statesToAvoid, rewards,
				minRewards, probPreference, null);// computeNestedValIterFailurePrint(mdp, target, statesToAvoid,
		// rewards,minRewards,target,probPreference,null);

		return res2;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid,
			ArrayList<MDPRewardsSimple> rewards, ArrayList<Boolean> minRewards, int probPreference,
			double[] probInitVal) throws PrismException {

		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(prismC);
		mc.setGenStrat(true);
		// mc.genStrat = true;
		// lets set them all to true
		ModelCheckerResult anotherSol = mc.computeUntilProbs(mdp, statesToRemainIn, target, false);
		StatesHelper.saveStrategy(anotherSol.strat, target, "", "computeUntilProbsStrat" + mdp.getFirstInitialState(),
				true);
		StateValues testValues = StateValues.createFromDoubleArray(anotherSol.soln, mdp);
		mainLog.println("Compute Until Probs Vals\n " + Arrays.toString(testValues.getDoubleArray()));
		if (mdp.getFirstInitialState() != -1)
			mainLog.println("Prob in init" + testValues.getDoubleArray()[mdp.getFirstInitialState()]);

		ModelCheckerMultipleResult res2 = mc.computeNestedValIterArray(mdp, target, statesToRemainIn, rewards, null,
				minRewards, target, probPreference, probInitVal);

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

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid,
			ArrayList<MDPRewardsSimple> rewards, int probPreference, boolean doMaxTasks) throws PrismException {

		ArrayList<Boolean> minMaxRew = new ArrayList<Boolean>();
		int rewinit = 0;
		if (doMaxTasks) {
			minMaxRew.add(false);
		}
		for (int rew = rewinit; rew < rewards.size(); rew++)
			minMaxRew.add(true);
		return computeNestedValIterFailurePrint(mdp, target, statesToAvoid, rewards, minMaxRew, probPreference);
	}

	protected void doSTAPU(ArrayList<Model> models, ExpressionFunc expr, BitSet statesOfInterest,
			ProbModelChecker mcProb, ArrayList<ModulesFile> modulesFiles, ArrayList<String> shared_vars_list,
			boolean includefailstatesinswitches, boolean matchSharedVars, boolean completeSwitchRing)
			throws PrismException {


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
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions);

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
			StatesHelper.saveMDP((MDP) model, null, "", "mdp" + i, true);

			int initState = model.getFirstInitialState();
		

			// for (int i = 0; i < numRobots; i++) {
			if (i != 0 && sameModelForAll) {
				initState = getInitState(i, exampleNumber());
				((MDPSparse) model).clearInitialStates();
				((MDPSparse) model).addInitialState(initState);

			}

			SingleAgentNestedProductMDP nestedProduct = buildSingleAgentNestedProductMDP("" + i, model, daList,
					statesOfInterest, mcProb, modulesFile);

			singleAgentProductMDPs.add(nestedProduct);
			resSaver.recordTime("Nested Product Time", varIDs.nestedproducttimes, true);
			resSaver.recordValues(nestedProduct.finalProduct.getProductModel().getNumStates(),
					"Nested Product States",varIDs.nestedproductstates);
			// }

		}

		resSaver.recordTime("Total Single Agent Nested Product Time", varIDs.allnestedproductcreationtime, false);
		resSaver.setScopeStartTime();

		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP = new SequentialTeamMDP(this.mainLog, numRobots, matchSharedVars); // buildSequentialTeamMDPTemplate(singleAgentProductMDPs);

		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs, shared_vars_list);

		int firstRobot = 0; // fix this
		// if (hasDoor)
		// StatesHelper.setMDPVar(seqTeamMDP.teamMDPTemplate.getVarList().getNumVars() -
		// 2); //cuz there is door too
		// else
		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing);
		resSaver.recordTime("Team MDP Time", varIDs.teammdptimeonly,true);

		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++) {
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));
//			mainLog.println("Essential States " + seqTeamMDP.essentialStates.get(i));

		}
//		mainLog.println("Accepting States " + seqTeamMDP.acceptingStates);


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

		
		for (int rew = 0; rew < rewards.size(); rew++) {
			StatesHelper.saveReward(seqTeamMDP.teamMDPWithSwitches, rewards.get(rew), combinedEssentialStates, "",
					"rew" + rew, true);
		}
		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates, "", "teamMDPWithSwitches", true);
		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.statesToAvoid, "", "teamMDPWithSwitchesAvoid",
				true);
		StatesHelper.saveMDPstatra(seqTeamMDP.teamMDPWithSwitches, "", "teamMDPWithSwitches_sta_tra", true);
		StatesHelper.saveBitSet(combinedEssentialStates, "", "teamMDP.ess", true);
		StatesHelper.saveBitSet(seqTeamMDP.acceptingStates, "", "teamMDP.acc", true);


		StatesHelper.saveReward(seqTeamMDP.teamMDPWithSwitches, rewards.get(1), null, "",
				"teamMDPWithSwitches_Rewards" + 1, true);
		
		resSaver.setLocalStartTime();
		resSaver.setScopeStartTime();
		ModelCheckerMultipleResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
				seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, rewards, minRewards, probPreference);// ,probInitVals);
		resSaver.recordTime("First Solution", varIDs.reallocations, true);
		
		StatesHelper.saveStrategy(solution.strat, null, "", "initialStrat", true);
		// solution.strat.exportInducedModel(mainLogRef);
		// solution.strat.exportActions(mainLogRef);
		mainLog.println(seqTeamMDP.acceptingStates.toString());

		// add to joint policy
		// MMDPSimple
		// jointPolicy = new MMDPSimple(seqTeamMDP.numRobots,
		// seqTeamMDP.agentMDPs.get(0).daList.size(), shared_vars_list,
		// seqTeamMDP.teamMDPTemplate.getVarList(), mainLog);
		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		mainLog.println("InitState = " + initialState);

		// *************************************************************//
		// testing the new joint policy stuff
		// this is a build as you go along kind of thing
		// so it doesn't really work right now okay
		// cuz soy un perdedor (sp?) i'm a loser baybay:P so why dont you kill me

		resSaver.setScopeStartTime();
		JointPolicyBuilder jointPolicyBuilder = new JointPolicyBuilder(seqTeamMDP.numRobots,
				seqTeamMDP.agentMDPs.get(0).daList.size(), shared_vars_list, seqTeamMDP.teamMDPTemplate.getVarList(),
				mainLog);

		jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP, initialState);
		resSaver.recordTime("Joint Policy Creationg Time", varIDs.jointpolicycreation,true);
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
				resSaver.recordValues(numPlanning, "Realloc "+numPlanning, varIDs.numreallocationsincode);
				resSaver.setScopeStartTime();
				
				int[] robotStates = jointPolicyBuilder.extractIndividualRobotStatesFromJointState(stateToExplore,
						seqTeamMDP.teamMDPWithSwitches.getStatesList(), seqTeamMDP.teamMDPWithSwitches.getVarList());
				firstRobot = jointPolicyBuilder.getFirstFailedRobotFromRobotStates(robotStates,
						seqTeamMDP.teamMDPWithSwitches);

				seqTeamMDP.setInitialStates(robotStates);
				seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing);

				if (statesToAvoid == null)
					statesToAvoid = seqTeamMDP.statesToAvoid;
				else {

					statesToAvoid.or(seqTeamMDP.statesToAvoid);
				}
				if (stateToExploreAndBitSet.getValue() != null) {
					String stateVal = stateToExplore.toString();
					stateVal = stateVal.replace(",", "_");
					stateVal = stateVal.replace(")", "");
					stateVal = stateVal.replace("(", "");
					StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, statesToAvoid, "", "fs" + stateVal, true);
				}
				if (stateToExplore.toString().contains("0,0,0,1,0,0,5,-1"))
					mainLog.println("pause here - Passenger Side - I brought a lemon to a knife");

				solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
						statesToAvoid, rewards, minRewards, probPreference);// ,probInitVals);
				
				resSaver.recordTime("Solution Time "+numPlanning, varIDs.reallocations, true);
				resSaver.setScopeStartTime();
				jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP.teamMDPWithSwitches,
						stateToExplore);
				resSaver.recordTime( "Joint Policy Time", varIDs.jointpolicycreation,true);
			}
		}
		resSaver.recordTime("All Reallocations", varIDs.allreallocationstime,false);
		
		mainLog.println("All done");
		mainLog.println("NVI done " + numPlanning + " times");
		jointPolicyBuilder.printStatesExploredOrder();
		mainLog.println("All done");

	}


	private int getInitState(int robotNum, int robotModel) {
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
			int initPoses[] = { 4, 8, 21, 28, 27, 17, 22, 20, 24, 26, 1, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
					18, 19, 23, 25, 29, 30 };
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
	protected ArrayList<Expression> getLTLExpressions(ExpressionFunc expr) throws PrismException {
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
						conjunctionOfExpressions = new ExpressionFunc(
								((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString());
					}
				}
			}
		}

		mainLog.println("LTL Mission: " + exprString);
		return ltlExpressions;
	}

	protected ArrayList<DAInfo> initializeDAInfoFromLTLExpressions(ArrayList<Expression> exprs) {
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

	public void run() {
		// saving filenames etc
		String dir = System.getProperty("user.dir");
		String modelLocation = dir + "/tests/decomp_tests/";

		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();

		String cumberland_nodoors = "topo_map_modified_goals"; // the cumberland map with no doors
		example_has_door_list.put(cumberland_nodoors, false);
		example_num_robot_list.put(cumberland_nodoors, 3);

		String cumberland_doors = "topo_map_modified_goals_doors"; // the cumberland map with doors
		example_has_door_list.put(cumberland_doors, true);
		example_num_door_list.put(cumberland_doors, 1);
		example_num_robot_list.put(cumberland_doors, 3);

		String a_door_example = "a_door_example"; // a single door
		example_has_door_list.put(a_door_example, true);
		example_num_door_list.put(a_door_example, 1);
		example_num_robot_list.put(a_door_example, 3);

		String no_door_example = "no_door_example"; // no door
		example_has_door_list.put(no_door_example, false);
		example_num_robot_list.put(no_door_example, 3);

		String a_door_example_actionwithtwogoals = "a_door_example_actionwithtwogoals";
		example_has_door_list.put(a_door_example_actionwithtwogoals, true);
		example_num_door_list.put(a_door_example_actionwithtwogoals, 1);
		example_num_robot_list.put(a_door_example_actionwithtwogoals, 3);

		String different_goals_example = "a_door_example_differenttasks"; // has completely different tasks but
																			// engineered such that there needs to be a
																			// wait before the door is checked
		example_has_door_list.put(different_goals_example, true);
		example_num_door_list.put(different_goals_example, 1);
		example_num_robot_list.put(different_goals_example, 3);

		String different_goals_example_longer = "a_door_example_differenttasks_longer";
		example_has_door_list.put(different_goals_example_longer, true);
		example_num_door_list.put(different_goals_example_longer, 1);
		example_num_robot_list.put(different_goals_example_longer, 3);

		String grid_2_example = "grid_2_topomap_sim";
		example_has_door_list.put(grid_2_example, false);
		example_num_robot_list.put(grid_2_example, 3);

		String grid_3_example = "grid_3_topomap_sim_doors";
		example_has_door_list.put(grid_3_example, true);
		example_num_door_list.put(grid_3_example, 2);
		example_num_robot_list.put(grid_3_example, 3);

		String three_robot_one_door = "three_robot_one_door";
		example_has_door_list.put(three_robot_one_door, true);
		example_num_door_list.put(three_robot_one_door, 1);
		example_num_robot_list.put(three_robot_one_door, 3);

		String two_robot_no_door = "two_robot_no_door";
		example_has_door_list.put(two_robot_no_door, false);
		example_num_door_list.put(two_robot_no_door, 1);
		example_num_robot_list.put(two_robot_no_door, 2);

		String two_robot_no_door_multiple_switches = "two_robot_no_door_ms";
		example_has_door_list.put(two_robot_no_door_multiple_switches, false);
		example_num_robot_list.put(two_robot_no_door_multiple_switches, 3);

		String two_robot_door_multiple_switches = "two_robot_door";
		example_has_door_list.put(two_robot_door_multiple_switches, true);
		example_num_door_list.put(two_robot_door_multiple_switches, 1);
		example_num_robot_list.put(two_robot_door_multiple_switches, 2);

		String tro_example = "tro_example";
		example_has_door_list.put(tro_example, true);
		example_num_door_list.put(tro_example, 1);
		example_num_robot_list.put(tro_example, 2);

		String tro_example_small = "tro_example_small";
		example_has_door_list.put(tro_example_small, true);
		example_num_door_list.put(tro_example_small, 1);
		example_num_robot_list.put(tro_example_small, 2);

		String autogen_example = "temp";
		example_has_door_list.put(autogen_example, true);
		example_num_door_list.put(autogen_example, 2);
		example_num_robot_list.put(autogen_example, 3);

		try {
			runOneExample(
					grid_3_example,
//					three_robot_one_door, 
					example_has_door_list,
					example_num_door_list, 
					example_num_robot_list,
					modelLocation);

		} catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
	}

	public void runOneExample(String example_name, HashMap<String, Boolean> example_has_door_list,
			HashMap<String, Integer> example_num_door_list, 
			HashMap<String, Integer> example_num_robot_list,
			String modelLocation) throws PrismException, FileNotFoundException {

		//Setting up
		String filename = example_name;
		
		boolean example_has_door = example_has_door_list.get(filename);

		String filename_suffix = "";
		
		//these affect the switch transitions and hence the solution 
		boolean includefailstatesinswitches = false;
		boolean matchsharedstatesinswitch = false;
		boolean completeSwitchRing = false;
		
		ArrayList<String> shared_vars_list = new ArrayList<String>();
		int numdoors = 0;
		if (example_has_door) {
			numdoors = example_num_door_list.get(example_name);
			if (numdoors == 1)
				shared_vars_list.add("door");
			else {
				for (int i = 0; i < numdoors; i++) {
					shared_vars_list.add("door" + i);

				}
			}
		}
		
		ArrayList<String> filenames = new ArrayList<String>();

		
		int numRobots = example_num_robot_list.get(example_name);
		for (int i = 0; i < numRobots; i++)
			filenames.add(filename + i);

		StatesHelper.setFolder(modelLocation + filename);
		hasDoor = false;
		int maxMDPVars = 0;

		ArrayList<Model> models = new ArrayList<Model>();
		ArrayList<PropertiesFile> propFiles = new ArrayList<PropertiesFile>();
		ArrayList<ModulesFile> modulesFiles = new ArrayList<ModulesFile>();

		// Create a log for PRISM output (hidden or stdout)
		// PrismLog mainLog = new PrismDevNullLog();
		PrismLog mainLog = new PrismFileLog("stdout");
		this.mainLog = mainLog;
		StatesHelper.mainLog = mainLog;
		resSaver = new ResultsTiming(mainLog, filename);
		
		//record num robots and doors 
		resSaver.recordInits(numRobots, "Robots", varIDs.numrobots);
		resSaver.recordInits(numdoors, "Doors", varIDs.numdoors);

		
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
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile,
					new File(modelLocation + filename + filename_suffix + ".prop"));

			// Get PRISM to build the model and then extract it
			prism.setEngine(Prism.EXPLICIT);
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit();
			int numVars = mdp.getVarList().getNumVars();
			if (numVars > maxMDPVars)
				maxMDPVars = numVars;

			models.add(mdp);
			propFiles.add(propertiesFile);
			resSaver.recordTime("model loading time "+files, varIDs.modelloadingtimes, true);
		}

		// Model check the first property from the file using the model checker
		for (int i = 0; i < propFiles.size(); i++) {
			mainLog.println(propFiles.get(i).getPropertyObject(0));
		}
		Expression expr = propFiles.get(0).getProperty(0);

		ExecutorService executor = Executors.newSingleThreadExecutor();
		StatesHelper.setNumMDPVars(maxMDPVars);
		Runnable task = new Runnable() {
			@Override
			public void run() {
				// do your task
				try {
					doSTAPU(models, (ExpressionFunc) expr, null, new ProbModelChecker(prism), modulesFiles,
							shared_vars_list, includefailstatesinswitches, matchsharedstatesinswitch,
							completeSwitchRing);
				} catch (PrismException e) {
					// TODO Auto-generated catch block

					e.printStackTrace();
				}
			}
		};

		Future<?> future = executor.submit(task);

		try {
			future.get(resSaver.timeout, TimeUnit.MILLISECONDS);

		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (ExecutionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (TimeoutException e) {
			mainLog.println("Timed out - " + (resSaver.timeout / (1000)) + " seconds, "
					+ (resSaver.timeout / (1000 * 60)) + " mins");
			// TODO Auto-generated catch block
			if (jointPolicy != null)
				mainLog.println("States " + jointPolicy.allFailStatesSeen.toString());
			e.printStackTrace();
		} // awaits termination

		resSaver.writeResults();
		// Close down PRISM
		prism.closeDown();

	}
	public int exampleNumber()
	{
		return 5;
	}

}
