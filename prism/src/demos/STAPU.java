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
import cern.colt.Arrays;
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
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

public class STAPU
{

	static String saveplace_suffix = "/tests/decomp_tests/temp/";
	static String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	public boolean noreallocations = false;
	// long timeout = 100 * 60 * 1000;
	public PrismLog mainLog;
	Prism prismC;
	//	MMDPSimple jointPolicy;

	boolean hasDoor = true;
	ResultsTiming resSaver;
	public boolean debugSTAPU = false;
	public boolean doSeqPolicyBuilding = false;
	public long stapuTimeDuration = 0;
	public long stapuFirstSolDuration = 0;
	public long stapuAllReplanningDuration = 0;

	public STAPU()
	{
		String dir = System.getProperty("user.dir");
		saveplace = dir + saveplace_suffix;
		StatesHelper.setSavePlace(saveplace);
	}

	public static void main(String[] args)
	{

		boolean reallocOnFirstRobotDeadend = false;
		boolean excludeRobotInitStates = false;
		STAPU stapu = new STAPU();

		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/";
		dir = dir + "compareSTAPUSSIFS/";

		String resString = "";

		int numRobots = 10;
		int numFS = 31;
		int numGoals = 11;
		int numDoors = 0;
		String fn = "g20x4_r10_g10_fs80fs31_fsgen9_";

		int maxRobots = 2;
		int maxGoals = 3;
		PrismLog fileLog = new PrismDevNullLog();

		ArrayList<Integer> robotNumbers = new ArrayList<Integer>();//generateListOfRandomNumbers(r, numRobots);
		ArrayList<Integer> goalNumbers = new ArrayList<Integer>(); //generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

		robotNumbers.add(0);
		robotNumbers.add(2);

		goalNumbers.add(5);
		goalNumbers.add(2);

		stapu.runGUISimpleTestsOne(dir, fn, maxRobots, numFS, maxGoals, numDoors, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, fileLog, null,
				excludeRobotInitStates);

		fileLog.close();
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

			product = daInfo.constructDAandProductModel(mcLTL, mcProb, modulesFile, allowedAcceptance, productMDP, null, true, (MDP) model);
			daInfo.associatedIndexInProduct++; //should go to zero from -1 

			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);

			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				res.daList.get(otherDAs).updateStateNumbers(product);
				res.daList.get(otherDAs).associatedIndexInProduct++; //and everyone else also gets shifted once. 

			}

			res.updateProductToMDPStateMapping(product);
			res.daList.add(daInfo);
		}
		DAInfo daInfo = res.daList.get(res.daList.size() - 1);
		int daNum = res.daList.size() - 1;

		res.setDAListAndFinalProduct(product);
		return res;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,
			ArrayList<Boolean> minRewards, int probPreference, PrismLog fileLog) throws PrismException
	{

		ModelCheckerMultipleResult res2 = computeNestedValIterFailurePrint(mdp, target, statesToAvoid, rewards, minRewards, probPreference, null, fileLog);// computeNestedValIterFailurePrint(mdp, target, statesToAvoid,

		return res2;
	}

	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target, BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,
			ArrayList<Boolean> minRewards, int probPreference, double[] probInitVal, PrismLog fileLog) throws PrismException
	{

		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		BitSet statesToIgnoreForVI = (BitSet) target.clone();
		statesToIgnoreForVI.or(statesToAvoid);
		MDPModelChecker mc = new MDPModelChecker(prismC);
		mc.setGenStrat(true);

		ModelCheckerMultipleResult res2 = mc.computeNestedValIterArray(mdp, target, statesToRemainIn, rewards, null, minRewards, statesToIgnoreForVI,
				probPreference, probInitVal);

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
			if (fileLog != null)
				fileLog.println("\nFor p = " + maxProb + ", rewards " + resString);
		}
		return res2;
	}

	public String getTimeString(long time)
	{
		String timeString = time + "ms" + "(" + TimeUnit.SECONDS.convert(time, TimeUnit.MILLISECONDS) + "s)";
		return timeString;
	}


	protected double[] doSTAPULimitGoals(ArrayList<Model> models, ExpressionFunc expr, BitSet statesOfInterest, ProbModelChecker mcProb,
			ArrayList<ModulesFile> modulesFiles, ArrayList<String> shared_vars_list, boolean includefailstatesinswitches, boolean matchSharedVars,
			boolean completeSwitchRing, int numGoals, boolean noReallocs, ArrayList<Integer> goalNumbers, boolean reallocateOnSingleAgentDeadend,
			PrismLog fileLog, boolean excludeRobotInitStates) throws PrismException
	{

		////profile
		ModelCheckerMultipleResult result=null;
		ArrayList<MDPRewardsSimple> finalRewards=null;
		long startTime = System.currentTimeMillis();
		long runningTimer = 0;
		long startTimer = 0;
		long stopTimer = 0;
		long runTimer = 0;
		int numPlanning = 0;
		ArrayList<double[]> planningValuesSTAPU = null;
		ArrayList<double[]> planningValuesJP = null;
		if (debugSTAPU) {
			planningValuesSTAPU = new ArrayList<double[]>();
			planningValuesJP = new ArrayList<double[]>();
		}

		int probPreference = 0;

		// process ltl expressions
		int numRobots = models.size();
		boolean sameModelForAll = false;
		if (numRobots != models.size() && models.size() == 1)
			sameModelForAll = true;
		else
			numRobots = models.size();

		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = new ArrayList<SingleAgentNestedProductMDP>();
		Entry<ArrayList<Expression>, ExpressionReward> ltlExpressionsAndExprRew = getLTLExpressionsLimit(expr, numGoals, goalNumbers);
		ArrayList<Expression> ltlExpressions = ltlExpressionsAndExprRew.getKey();
		ExpressionReward rewToAttach = ltlExpressionsAndExprRew.getValue();
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions, rewToAttach);

		Model model = models.get(0);
		ModulesFile modulesFile = modulesFiles.get(0);

		//TODO:your code here 

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		stapuTimeDuration += runTime;

		fileLog.println("Initialization: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(stapuTimeDuration));

		long maxTimeNP = 0;
		for (int i = 0; i < numRobots; i++) {
			////profile
			long startTimex = System.currentTimeMillis();

			if (!sameModelForAll) {

				model = models.get(i);
				modulesFile = modulesFiles.get(i);
			}
			SingleAgentNestedProductMDP nestedProduct = buildSingleAgentNestedProductMDP("" + i, model, daList, statesOfInterest, mcProb, modulesFile);
			singleAgentProductMDPs.add(nestedProduct);
			//TODO:your code here 

			long stopTimex = System.currentTimeMillis();
			long runTimex = stopTimex - startTimex;
			if (runTimex > maxTimeNP)
				maxTimeNP = runTimex;

			fileLog.println("Build Nested Product: " + getTimeString(runTimex));
			fileLog.println("MDP details:\n "+nestedProduct.finalProduct.getProductModel().infoStringTable());
			fileLog.println("XXX,A1,"+System.currentTimeMillis());
		}
		
		fileLog.println("Max Nested Product: " + getTimeString(maxTimeNP));
		stapuTimeDuration += maxTimeNP;
		fileLog.println("Time so far: " + getTimeString(stapuTimeDuration));

		startTime = System.currentTimeMillis();
		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP = new SequentialTeamMDP(this.mainLog, numRobots, matchSharedVars); // buildSequentialTeamMDPTemplate(singleAgentProductMDPs);

		seqTeamMDP.doDebug = this.debugSTAPU;
		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs, shared_vars_list);
		fileLog.println("Team MDP Template:\n "+seqTeamMDP.teamMDPTemplate.infoStringTable());

		int firstRobot = 0; // fix this

		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing, excludeRobotInitStates);

		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++) {
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));

		}
		
		fileLog.println("XXX,B,"+System.currentTimeMillis());
		fileLog.println("Team MDP with Switches:\n "+seqTeamMDP.teamMDPWithSwitches.infoStringTable());
		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>(seqTeamMDP.rewardsWithSwitches);
		ArrayList<Boolean> minRewards = new ArrayList<Boolean>();
		for (int rew = 0; rew < rewards.size(); rew++) {
			minRewards.add(true);
		}
		rewards.add(0, seqTeamMDP.progressionRewards);

		minRewards.add(0, false);

		combinedEssentialStates.or(seqTeamMDP.acceptingStates);

		//		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates, "", "teamMDPWithSwitches", true);
		if (debugSTAPU)
			StatesHelper.saveMDPstatra(seqTeamMDP.teamMDPWithSwitches, "", "teamMDPWithSwitches", true);

		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		stapuTimeDuration += runTime;
		fileLog.println("Team Updation/Building: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(stapuTimeDuration));

		startTime = System.currentTimeMillis();

		ModelCheckerMultipleResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
				seqTeamMDP.statesToAvoid, rewards, minRewards, probPreference, fileLog);// ,probInitVals);

		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		fileLog.println("XXX,C,"+System.currentTimeMillis());
		mainLog.println("InitState = " + initialState);
		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		stapuTimeDuration += runTime;
		fileLog.println("Solution: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(stapuTimeDuration));

		startTime = System.currentTimeMillis();

		// *************************************************************//
		// testing the new joint policy stuff
		// this is a build as you go along kind of thing
		// so it doesn't really work right now okay
		// cuz soy un perdedor (sp?) i'm a loser baybay:P so why dont you kill me

		JointPolicyBuilder jointPolicyBuilder = new JointPolicyBuilder(seqTeamMDP.numRobots, seqTeamMDP.agentMDPs.get(0).daList.size(), shared_vars_list,
				seqTeamMDP.teamMDPTemplate.getVarList(), rewards, mainLog);
		
		if (debugSTAPU) {
			PolicyCreator pc = new PolicyCreator();
			pc.createPolicyWithRewardsStructuresAsLabels(seqTeamMDP.teamMDPWithSwitches.getFirstInitialState(), seqTeamMDP.teamMDPWithSwitches, solution.strat,
					seqTeamMDP.progressionRewards, seqTeamMDP.rewardsWithSwitches.get(0), seqTeamMDP.acceptingStates);
			//			pc.savePolicy("/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSIFS/results/", "seqTeamPolicyRew.dot");
			StatesHelper.saveMDP(pc.mdpCreator.mdp, null, "", "_0_init_seqTeamPolicyRews", true);
		}
		jointPolicyBuilder.doSeq = doSeqPolicyBuilding;
		jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP, initialState, reallocateOnSingleAgentDeadend, 1.0);

		if (debugSTAPU) {
			PolicyCreator pc = new PolicyCreator();

			pc.recreateMDPWithRewardsStructuresAsLabels(seqTeamMDP.teamMDPWithSwitches.getFirstInitialState(), seqTeamMDP.teamMDPWithSwitches,
					seqTeamMDP.progressionRewards, seqTeamMDP.rewardsWithSwitches.get(0), seqTeamMDP.acceptingStates);
			StatesHelper.saveMDP(pc.mdpCreator.mdp, null, "", "_initTeamMDPWithRews", true);
			StatesHelper.saveMDPstatra(pc.mdpCreator.mdp, "", "_initTeamMDPWithRews", true);
			planningValuesSTAPU.add(resultValues(solution, seqTeamMDP.teamMDPWithSwitches));

			jointPolicyBuilder.createRewardStructures();
			 finalRewards = jointPolicyBuilder.getExpTaskAndCostRewards();
			jointPolicyBuilder.jointMDP.findDeadlocks(true);
			result = computeNestedValIterFailurePrint(jointPolicyBuilder.jointMDP, jointPolicyBuilder.accStates, new BitSet(),
					finalRewards, minRewards, probPreference, fileLog);
			planningValuesJP.add(resultValues(result, jointPolicyBuilder.jointMDP));

			pc = new PolicyCreator();
			pc.createPolicyWithRewardsStructuresAsLabels(jointPolicyBuilder.jointMDP.getFirstInitialState(), jointPolicyBuilder.jointMDP, result.strat,
					finalRewards.get(0), finalRewards.get(1), jointPolicyBuilder.accStates);
			StatesHelper.saveMDP(pc.mdpCreator.mdp, null, "", "_0" + "_init_jpRews", true);
			jointPolicyBuilder.createRewardStructures();
			 finalRewards = jointPolicyBuilder.getExpTaskAndCostRewards();
			jointPolicyBuilder.jointMDP.findDeadlocks(true);
			 result = computeNestedValIterFailurePrint(jointPolicyBuilder.jointMDP, jointPolicyBuilder.accStates, new BitSet(),
					finalRewards, minRewards, probPreference, fileLog);
		}

		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		stapuTimeDuration += runTime;
		fileLog.println("Joint Policy Building: " + getTimeString(runTime));
		fileLog.println("Time so far: " + getTimeString(stapuTimeDuration));
		
		fileLog.println("XXX,D,"+System.currentTimeMillis());
		fileLog.println("Policy MC:\n "+jointPolicyBuilder.jointMDP.infoStringTable());
		//		startTime = System.currentTimeMillis();

		//		stopTime = System.currentTimeMillis();
		//		runTime = stopTime - startTime;
		//		stapuTimeDuration += runTime;
		stapuFirstSolDuration = stapuTimeDuration;
		startTime = System.currentTimeMillis();
		if (!noReallocs) {
			
			runningTimer = 0;
			while (jointPolicyBuilder.hasFailedStates()) {
				////profile
				startTimer = System.currentTimeMillis();

				Entry<Entry<State, Double>, BitSet> stateToExploreAndBitSet = jointPolicyBuilder.getNextFailedState();
				Entry<State, Double> stateToExploreProbPair = stateToExploreAndBitSet.getKey();
				State stateToExplore = stateToExploreProbPair.getKey();
				double stateToExploreProb = stateToExploreProbPair.getValue();
				BitSet statesToAvoid = stateToExploreAndBitSet.getValue();

				//TODO:your code here 

				stopTimer = System.currentTimeMillis();
				runTimer = stopTimer - startTimer;
				runningTimer += runTimer;
				fileLog.println("Replanning inits: " + getTimeString(runTimer));
				
				fileLog.println("Time so far: " + getTimeString(stapuTimeDuration + runningTimer));
				if (!jointPolicyBuilder.inStatesExplored(stateToExplore)) {
					startTimer = System.currentTimeMillis();
					// get first failed robot
					numPlanning++;

					int[] robotStates = jointPolicyBuilder.extractIndividualRobotStatesFromJointState(stateToExplore,
							seqTeamMDP.teamMDPWithSwitches.getStatesList(), seqTeamMDP.teamMDPWithSwitches.getVarList());
					firstRobot = jointPolicyBuilder.getFirstFailedRobotFromRobotStates(robotStates, seqTeamMDP.teamMDPWithSwitches);

					seqTeamMDP.setInitialStates(robotStates);
					seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing, excludeRobotInitStates);

					if (statesToAvoid == null)
						statesToAvoid = seqTeamMDP.statesToAvoid;
					else {

						statesToAvoid.or(seqTeamMDP.statesToAvoid);
					}
					stopTimer = System.currentTimeMillis();
					runTimer = stopTimer - startTimer;
					fileLog.println("Replanning " + numPlanning + "" + "\nTeam Updation : " + getTimeString(runTimer));
					runningTimer += runTimer;
					fileLog.println("Time so far: " + getTimeString(stapuTimeDuration + runningTimer));

					startTimer = System.currentTimeMillis();
					fileLog.println("XXX,B,"+System.currentTimeMillis());
					fileLog.println("Team MDP with Switches:\n "+seqTeamMDP.teamMDPWithSwitches.infoStringTable());
					solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates, statesToAvoid, rewards, minRewards,
							probPreference, fileLog);// ,probInitVals);
					fileLog.println("XXX,C,"+System.currentTimeMillis());
					stopTimer = System.currentTimeMillis();
					runTimer = stopTimer - startTimer;
					fileLog.println("Solution : " + getTimeString(runTimer));
					runningTimer += runTimer;
					fileLog.println("Time so far: " + getTimeString(stapuTimeDuration + runningTimer));

					startTimer = System.currentTimeMillis();
					jointPolicyBuilder.buildJointPolicyFromSequentialPolicy(solution.strat, seqTeamMDP.teamMDPWithSwitches, stateToExplore,
							seqTeamMDP.acceptingStates, reallocateOnSingleAgentDeadend, stateToExploreProb);
				
					if (debugSTAPU) {
						PolicyCreator pc = new PolicyCreator();
						pc.createPolicyWithRewardsStructuresAsLabels(seqTeamMDP.teamMDPWithSwitches.getFirstInitialState(), seqTeamMDP.teamMDPWithSwitches,
								solution.strat, seqTeamMDP.progressionRewards, seqTeamMDP.rewardsWithSwitches.get(0), seqTeamMDP.acceptingStates);
						//						pc.savePolicy("/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSIFS/results/", "seqTeamPolicyRew.dot");
						StatesHelper.saveMDP(pc.mdpCreator.mdp, null, "", "_" + numPlanning + "_" + stateToExplore.toString() + "_seqTeamPolicyRews", true);
						planningValuesSTAPU.add(resultValues(solution, seqTeamMDP.teamMDPWithSwitches));
						jointPolicyBuilder.createRewardStructures();
						 finalRewards = jointPolicyBuilder.getExpTaskAndCostRewards();
						jointPolicyBuilder.jointMDP.findDeadlocks(true);
						 result = computeNestedValIterFailurePrint(jointPolicyBuilder.jointMDP, jointPolicyBuilder.accStates,
								new BitSet(), finalRewards, minRewards, probPreference, fileLog);
						planningValuesJP.add(resultValues(result, jointPolicyBuilder.jointMDP));
						pc = new PolicyCreator();
						pc.createPolicyWithRewardsStructuresAsLabels(jointPolicyBuilder.jointMDP.getFirstInitialState(), jointPolicyBuilder.jointMDP,
								result.strat, finalRewards.get(0), finalRewards.get(1), jointPolicyBuilder.accStates);
						StatesHelper.saveMDP(pc.mdpCreator.mdp, null, "", "_" + numPlanning + "_" + stateToExplore.toString() + "_jpRews", true);
						jointPolicyBuilder.createRewardStructures();
						 finalRewards = jointPolicyBuilder.getExpTaskAndCostRewards();
						jointPolicyBuilder.jointMDP.findDeadlocks(true);
						 result = computeNestedValIterFailurePrint(jointPolicyBuilder.jointMDP, jointPolicyBuilder.accStates,
								new BitSet(), finalRewards, minRewards, probPreference, fileLog);
					}
					fileLog.println("XXX,D,"+System.currentTimeMillis());
					fileLog.println("Policy MC:\n "+jointPolicyBuilder.jointMDP.infoStringTable());
					stopTimer = System.currentTimeMillis();
					
					runTimer = stopTimer - startTimer;
					fileLog.println("Joint Policy Building: " + getTimeString(runTimer));
					runningTimer += runTimer;
					fileLog.println("Time so far: " + getTimeString(stapuTimeDuration + runningTimer));

				}
			}

			//		jointPolicyBuilder.saveJointPolicyMDP();
			mainLog.println("All done");
			mainLog.println("NVI done " + numPlanning + " times");
			//			jointPolicyBuilder.printStatesExploredOrder();
		}
		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		stapuAllReplanningDuration = runTime;
		stapuTimeDuration += runTime;
		//		HashMap<String, Double> values = new HashMap<String, Double>();
		//		values.put("prob", jointPolicyBuilder.getProbabilityOfSatisfactionFromInitState());
		startTimer = System.currentTimeMillis();
		startTime = System.currentTimeMillis();
		jointPolicyBuilder.createRewardStructures();
		 finalRewards = jointPolicyBuilder.getExpTaskAndCostRewards();
		jointPolicyBuilder.jointMDP.findDeadlocks(true);

		 result = computeNestedValIterFailurePrint(jointPolicyBuilder.jointMDP, jointPolicyBuilder.accStates, new BitSet(),
				finalRewards, minRewards, probPreference, fileLog);

		if (debugSTAPU) {
			for (int i = 0; i < planningValuesSTAPU.size(); i++) {
				mainLog.println(i + ":" + "P:" + Arrays.toString(planningValuesSTAPU.get(i)) + " C:" + Arrays.toString(planningValuesJP.get(i)));
			}
		}
		mainLog.println("All done");

		double[] results = resultValues(result, jointPolicyBuilder.jointMDP);
		stopTime = System.currentTimeMillis();
		stopTimer = System.currentTimeMillis();
		runTimer = stopTimer - startTimer;
		
		fileLog.println("Final Clean up: " + getTimeString(runTimer));
		runningTimer += runTimer;
		fileLog.println("Time so far: " + getTimeString(stapuTimeDuration ));

		fileLog.println("All Replanning: "+numPlanning+" time "+ getTimeString(runningTimer-runTimer));
		fileLog.println("All Replanning: "+numPlanning+" time "+ getTimeString(this.stapuAllReplanningDuration));
		
		runTime = stopTime - startTime;
	
		stapuTimeDuration += runTime;
		runTimer = stopTimer - startTimer;
		fileLog.println("Time so far without timer: " + getTimeString(stapuTimeDuration));
		fileLog.println("XXX,E,"+System.currentTimeMillis());
		fileLog.println("Policy MC:\n "+jointPolicyBuilder.jointMDP.infoStringTable());
		//end profiling
		return results;

	}

	protected double[] resultValues(ModelCheckerMultipleResult res2, MDPSimple mdp)
	{

		////profile
		long startTime = System.currentTimeMillis();

		//TODO:your code here 

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
		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		System.out.println("\nResults values section: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
		//end profiling

		return result;

	}

	/**
	 * Return a list of expressions
	 */
	protected Entry<ArrayList<Expression>, ExpressionReward> getLTLExpressionsLimit(ExpressionFunc expr, int lim, ArrayList<Integer> goalNumbers)
			throws PrismException
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
			if (expr.getOperand(exprNum) instanceof ExpressionReward)
				rewardFuncIndices.add(exprNum);

		}

		boolean noRewardIndices = true;
		//limiting the expressions 
		ArrayList<Expression> ltlExpressionsLimited = new ArrayList<Expression>(lim);
		exprString = "";
		if (goalNumbers == null) {
			for (int exprNum = 0; exprNum < numOp; exprNum++) {
				if (noRewardIndices && rewardFuncIndices.contains(exprNum))
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
				if (noRewardIndices && rewardFuncIndices.contains(exprNum))
					noRewardIndices = false;
				Expression current = ltlExpressions.get(exprNum);

				exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
				ltlExpressionsLimited.add(current);

			}
			exprString += ((ExpressionQuant) ltlExpressions.get(numOp - 1)).getExpression().toString();
			ltlExpressionsLimited.add(ltlExpressions.get(numOp - 1));

		}

		if (noRewardIndices) {
			//just pick a reward expression
			//any 
			if (rewardFuncIndices.size() > 0)
				exprRew = (ExpressionReward) expr.getOperand(rewardFuncIndices.get(0));
			else
				throw new PrismException("No reward expression!!!");
		}
		mainLog.println("LTL Mission: " + exprString);
		return new AbstractMap.SimpleEntry<ArrayList<Expression>, ExpressionReward>(ltlExpressionsLimited, exprRew);
	}

	protected ArrayList<DAInfo> initializeDAInfoFromLTLExpressions(ArrayList<Expression> exprs, ExpressionReward rewToAttach)
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
			if (rewToAttach != null && !daInfo.isSafeExpr && !rewardAttached) {
				daInfo.daExprRew = rewToAttach;
				rewardAttached = true;
			}
			daInfoList.add(daInfo);
		}

		return daInfoList;
	}

	public double[] runGUISimpleTestsOne(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors, ArrayList<Integer> robotNumbers,
			ArrayList<Integer> goalNumbers, boolean reallocateOnSingleAgentDeadend, PrismLog fileLog, String mainLogfn, boolean excludeRobotInitStates)
	{
		fileLog.println("XXX,-A,"+System.currentTimeMillis());
		double[] res = null;

		String modelLocation = dir;
		StatesHelper.setSavePlace(modelLocation + "results/");
		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_goals_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list = new HashMap<String, Integer>();
		ArrayList<String> examples = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();

		String example = fn;//"g5_r2_t3_d2_fs1";//"g5_r3_t3_d0_fs0";//"test_grid_nodoors_nofs";
		String example_id = example;//example + "r" + numRobots;//cumberland_doors; 
		String example_to_run = example;//cumberland_doors; 

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);
		long startTime = System.currentTimeMillis();
		long endTime;
		for (int i = 0; i < examples.size(); i++) {
			example_to_run = examples.get(i);
			example_id = example_ids.get(i);

			int maxRobots = example_num_robot_list.get(example_id);
			int maxGoals = example_num_goals_list.get(example_id);
			try {
				startTime = System.currentTimeMillis();
				res = runOneExampleNumRobotsGoals(example_to_run, example_id, example_has_door_list, example_num_door_list, maxRobots, maxGoals,
						example_num_fs_list, modelLocation, true, dir + "results/stapu", this.noreallocations, robotNumbers, goalNumbers,
						reallocateOnSingleAgentDeadend, fileLog, mainLogfn, excludeRobotInitStates);
				endTime = System.currentTimeMillis();
				fileLog.println("Finished: " + (endTime - startTime));
			} catch (Exception e) {
				e.printStackTrace();

				System.out.println("Error: " + e.getMessage());
				//						System.exit(1);
				endTime = System.currentTimeMillis();
				fileLog.println("Exception: " + (endTime - startTime));
				fileLog.println(e.getStackTrace().toString());
			}

		}
		fileLog.println("XXX,G,"+System.currentTimeMillis());
		return res;

	}

	public double[] runOneExampleNumRobotsGoals(String example_name, String example_id, HashMap<String, Boolean> example_has_door_list,

			HashMap<String, Integer> example_num_door_list, int numRobots, int numGoals, HashMap<String, Integer> example_num_fs_list, String modelLocation,
			boolean doorVarNameHas0, String resLoc, boolean noReallocs, ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers,
			boolean reallocateOnSingleAgentDeadend, PrismLog fileLog, String mainLogfn, boolean excludeRobotInitStates)

			throws PrismException, FileNotFoundException
	{

		double[] res = null;

		fileLog.println("XXX,A,"+System.currentTimeMillis());
		//Setting up
		//Current Time 
		////profile
		long startTime = System.currentTimeMillis();

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
		if (mainLogfn != null)
			mainLog = new PrismFileLog(mainLogfn);
		this.mainLog = mainLog;
		StatesHelper.mainLog = mainLog;

		// Initialise PRISM engine
		Prism prism = new Prism(mainLog);
		prismC = prism;
		prism.initialise();

		for (int files = 0; files < filenames.size(); files++) {

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

		}

		// Model check the first property from the file using the model checker
		for (int i = 0; i < propFiles.size(); i++) {
			mainLog.println(propFiles.get(i).getPropertyObject(0));
		}
		Expression expr = propFiles.get(0).getProperty(0);

		StatesHelper.setNumMDPVars(maxMDPVars);

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		stapuTimeDuration += runTime;
		res = doSTAPULimitGoals(models, (ExpressionFunc) expr, null, new ProbModelChecker(prism), modulesFiles, shared_vars_list, includefailstatesinswitches,
				matchsharedstatesinswitch, completeSwitchRing, numGoals, noReallocs, goalNumbers, reallocateOnSingleAgentDeadend, fileLog,
				excludeRobotInitStates);

		// Close down PRISM
		prism.closeDown();
		//TODO:your code here 

		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		System.out.println("\nSTAPU Runtime: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
		//end profiling
		fileLog.println("XXX,F,"+System.currentTimeMillis());
		return res;

	}
}
