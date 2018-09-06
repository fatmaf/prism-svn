package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import acceptance.AcceptanceType;
import cern.colt.Arrays;
import demos.MMDPSimple.StateProb;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSparse;
import explicit.Model;
import explicit.ModelCheckerMultipleResult;
import explicit.ModelCheckerPartialSatResult;
import explicit.ModelCheckerResult;
import explicit.ProbModelChecker;
import explicit.StateValues;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MDPRewards;
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

	long timeout = 1000 * 60 * 1000;
	public PrismLog mainLog;
	Prism prismC;
	MMDPSimple jointPolicy;

	boolean hasDoor = true;

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
			StatesHelper.saveMDPstatra(productMDP,  "", name + "pda_" + daNum+"sta_tra", true);
			
			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				res.daList.get(otherDAs).updateStateNumbers(product);
				StatesHelper.saveBitSet(res.daList.get(otherDAs).essentialStates, "", name+"pda_"+daNum+"_"+otherDAs+".ess", true);
				StatesHelper.saveBitSet(res.daList.get(otherDAs).productAcceptingStates, "", name+"pda_"+daNum+"_"+otherDAs+".acc", true);
			}
			res.updateProductToMDPStateMapping(product);
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
		mainLog.println("Prob in init" + testValues.getDoubleArray()[mdp.getFirstInitialState()]);

		ModelCheckerMultipleResult res2 = mc.computeNestedValIterArray(mdp, target, statesToRemainIn, rewards, null,
				minRewards, target, probPreference, probInitVal);

		ArrayList<double[]> solns = res2.solns;
		double[] solnProb = solns.get(solns.size() - 1);
		StateValues probsProduct = StateValues.createFromDoubleArray(solnProb, mdp);

		// Get final prob result

		double maxProb = probsProduct.getDoubleArray()[mdp.getFirstInitialState()];

		String resString = "";
		for (int i = 0; i < solns.size() - 1; i++) {
			StateValues costsProduct = StateValues.createFromDoubleArray(res2.solns.get(i), mdp);

			double minCost = costsProduct.getDoubleArray()[mdp.getFirstInitialState()];
			resString += i + ":" + minCost + " ";

		}
		mainLog.println("\nFor p = " + maxProb + ", rewards " + resString);

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
			boolean includefailstatesinswitches, boolean matchSharedVars,boolean completeSwitchRing) throws PrismException {

		long startTime = System.currentTimeMillis();
		int probPreference = 0;

		// process ltl expressions
		int numRobots = getNumRobots(exampleNumber());
		boolean sameModelForAll = false;
		if (numRobots != models.size() && models.size() == 1)
			sameModelForAll = true;
		else
			numRobots = models.size();

		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = new ArrayList<SingleAgentNestedProductMDP>();
		ArrayList<Expression> ltlExpressions = getLTLExpressions(expr);
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions);
		if (hasTimedOut(startTime, "Initialized DA from LTL Expressions"))
			return;
		Model model = models.get(0);
		ModulesFile modulesFile = modulesFiles.get(0);
		for (int i = 0; i < numRobots; i++) {
			if (!sameModelForAll) {
				model = models.get(i);
				modulesFile = modulesFiles.get(i);
			}
			StatesHelper.saveMDP((MDP) model, null, "", "mdp" + i, true);
			if (hasTimedOut(startTime, "Saved original MDP"))
				return;
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
			if (hasTimedOut(startTime, "Created Nested Product " + i))
				return;

			// }

		}

		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP = new SequentialTeamMDP(this.mainLog, numRobots, matchSharedVars); // buildSequentialTeamMDPTemplate(singleAgentProductMDPs);

		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs, shared_vars_list);

		if (hasTimedOut(startTime, "Created Sequential MDP Template"))
			return;

		int firstRobot = 0; // fix this
		// if (hasDoor)
		// StatesHelper.setMDPVar(seqTeamMDP.teamMDPTemplate.getVarList().getNumVars() -
		// 2); //cuz there is door too
		// else
		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches,completeSwitchRing);

		if (hasTimedOut(startTime, "Created Sequential MDP with Switches"))
			return;

		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++) {
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));
			mainLog.println("Essential States " + seqTeamMDP.essentialStates.get(i));

		}
		mainLog.println("Accepting States " + seqTeamMDP.acceptingStates);

		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates, "", "teamMDPWithSwitches", true);
		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.statesToAvoid, "", "teamMDPWithSwitchesAvoid",
				true);
		StatesHelper.saveMDPstatra(seqTeamMDP.teamMDPWithSwitches, "", "teamMDPWithSwitches_sta_tra", true);
		StatesHelper.saveBitSet(combinedEssentialStates, "", "teamMDP.ess",true);
		StatesHelper.saveBitSet(seqTeamMDP.acceptingStates, "", "teamMDP.acc",true);

		// solve
		// ModelCheckerPartialSatResult solution =
		// computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
		// seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid,
		// seqTeamMDP.rewardsWithSwitches.get(0));
		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>(seqTeamMDP.rewardsWithSwitches);
		ArrayList<Boolean> minRewards = new ArrayList<Boolean>();
		for (int rew = 0; rew < rewards.size(); rew++) {
			minRewards.add(true);
		}
		rewards.add(0, seqTeamMDP.progressionRewards);

		minRewards.add(0, false);

		combinedEssentialStates.or(seqTeamMDP.acceptingStates);
		for (int rew = 0; rew < rewards.size(); rew++) {
			StatesHelper.saveReward(seqTeamMDP.teamMDPWithSwitches, rewards.get(rew), combinedEssentialStates, "",
					"rew" + rew, true);
		}

		// double[] probInitVals = new
		// double[seqTeamMDP.teamMDPWithSwitches.getNumStates()];
		// for(int i = 0; i<probInitVals.length; i++)
		// {
		// if(seqTeamMDP.acceptingStates.get(i) || combinedEssentialStates.get(i))
		// probInitVals[i]=1.0;
		// else
		// probInitVals[i]=0.0;
		// }

		ModelCheckerMultipleResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
				seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, rewards, minRewards, probPreference);// ,probInitVals);

		StatesHelper.saveStrategy(solution.strat, null, "", "initialStrat", true);
		// solution.strat.exportInducedModel(mainLogRef);
		// solution.strat.exportActions(mainLogRef);
		mainLog.println(seqTeamMDP.acceptingStates.toString());
		if (hasTimedOut(startTime, "Solved Initial Sequential MDP"))
			return;

		// add to joint policy
		// MMDPSimple
		jointPolicy = new MMDPSimple(seqTeamMDP.numRobots, seqTeamMDP.agentMDPs.get(0).daList.size(), shared_vars_list,
				seqTeamMDP.teamMDPTemplate.getVarList(), mainLog);
		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		mainLog.println("InitState = " + initialState);

		jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, true,
				!includefailstatesinswitches);

		if (hasTimedOut(startTime, "Added Initial Solution to Joint Policy"))
			return;

		StatesHelper.saveMDP(jointPolicy.mdp, null, "", "jointPolicy", true);
		boolean stopHere = false;// true;
		if (stopHere)
			return;
		Vector<StateProb> orderOfFailStates = new Vector<StateProb>();

		// start the loop
		while (!jointPolicy.stuckStatesQ.isEmpty()) {

			// get the state with the highest probability, given the current joint policy
			StateProb stuckState = jointPolicy.stuckStatesQ.remove();
			initialState = stuckState.getState();
			orderOfFailStates.add(stuckState.copy());

			mainLog.println("Exploring " + stuckState.toString());

			// update the teammdp
			List<State> states = jointPolicy.mdp.getStatesList();// seqTeamMDP.teamMDPWithSwitches.getStatesList();
			State currState = states.get(initialState);
			int rNum = jointPolicy.firstFailedRobot(currState);
			int[] robotStates = jointPolicy.getRobotStatesIndexFromJointState(currState,
					seqTeamMDP.teamMDPWithSwitches.getStatesList());
			seqTeamMDP.setInitialStates(robotStates);

			if (hasTimedOut(startTime, "Set Initial States for Fail State"))
				return;

			// we need to change the switches to the initial states
			// so we will just add switches from all failed robots
			// we really dont care who failed first because
			// the switches will take care of that
			seqTeamMDP.addSwitchesAndSetInitialState(rNum, includefailstatesinswitches,completeSwitchRing);

			if (hasTimedOut(startTime, "Added Switches"))
				return;

			StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates, "",
					"teamMDPWithSwitches" + currState.toString(), true);

			solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
					seqTeamMDP.statesToAvoid, rewards, probPreference, true);
			// solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
			// seqTeamMDP.acceptingStates,
			// seqTeamMDP.statesToAvoid, seqTeamMDP.rewardsWithSwitches.get(0));

			StatesHelper.saveStrategy(solution.strat, null, "", "stratFor" + currState.toString(), true);
			if (hasTimedOut(startTime, "Solved for Fail State"))
				return;

			jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, false,
					!includefailstatesinswitches);

			if (hasTimedOut(startTime, "Added Solution to Joint Policy"))
				return;

		}
		StatesHelper.saveMDP(jointPolicy.mdp, null, "", "finalJointPolicy", true);
		StatesHelper.saveMDPstatra(jointPolicy.mdp, "", "finalJointPolicy", true);
		jointPolicy.saveJointPolicy();
		mainLog.println("Completed STAPU for full policy");
		mainLog.println("DeadEnd States " + jointPolicy.deadendStates.toString());
		mainLog.println("Accepting States " + jointPolicy.allTasksCompletedStates.toString());
		mainLog.println("Information about fail states ");

		for (StateProb fs : orderOfFailStates) {
			double probAnyAcc = jointPolicy.getProbabilityAnyAcceptingState(fs.getState(), 1.0, new BitSet());
			double probAllAcc = jointPolicy.getProbabilityAllPossibleAcceptingStates(fs.getState(), 1.0, new BitSet(),
					new HashMap<Integer, Double>());
			mainLog.println("Explored state " + fs.toString() + " with prob " + probAnyAcc
					+ " of getting to an accepting state from this state \n Prob of acheiving task = "
					+ fs.getProb() * probAllAcc);
		}

		mainLog.println("Probablilty of acheiving task from initial state using entire policy = " + jointPolicy
				.getProbabilityAllPossibleAcceptingStates(0, 1.0, new BitSet(), new HashMap<Integer, Double>()));
		hasTimedOut(startTime, "All Done");

	}

	private int exampleNumber() {
		// SET the EXAMPLE NUMBER HERE
		// 0 = two room three robot not extended , 1 = two room three robot extended, 2
		// = debugging on
		// two_room_three_robot_blowup_reduced, 3 = three_robot_simple
		// 4 = topo_map
		// 5= chain example
		// 6 = vi_example
		return 5;
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
				if(Expression.isCoSafeLTLSyntactic(expr.getOperand(exprNum))) {
				if(conjunctionOfExpressions == null )
				{
					conjunctionOfExpressions =  new ExpressionFunc(((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString());
				}
				}
			}
		}

		mainLog.println("LTL Mission: " + exprString);
		return ltlExpressions;
	}

	private int getNumRobots(int exampleNumber) {
		int toret = -1;
		switch (exampleNumber) {
		// case 0:
		// toret =1;
		// break;
		case 6:
			toret = 2;
			break;
		case 0:
		case 1:
			toret = 4;
			break;
		case 2:
		case 3:
		case 4:
		case 5:
			toret = 3;
			break;

		}
		return toret;
	}

	private boolean hasTimedOut(long startTime, long endTime, String text) {
		long time = endTime - startTime;
		printTime(time, text);
		if (time > timeout) {
			mainLog.println("Timed Out");
			return true;
		} else
			return false;
	}

	private boolean hasTimedOut(long startTime, String text) {
		long endTime = System.currentTimeMillis();
		return hasTimedOut(startTime, endTime, text);

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

	private void printTime(long time, String text) {
		if (text == "")
			text = "Time";
		mainLog.println(text + ": " + time / 1000.0 + " seconds (" + time / (1000.0 * 60) + " mins)");
	}

	public void run() {
		try {
			String dir = System.getProperty("user.dir");
			String modelLocation = dir + "/tests/decomp_tests/";
			String cumberland_nodoors = "topo_map_modified_goals";
			String cumberland_doors = "topo_map_modified_goals_doors";
			String a_door_example = "a_door_example";
			String no_door_example = "no_door_example";
			String a_door_example_actionwithtwogoals = "a_door_example_actionwithtwogoals";
			String different_goals_example="a_door_example_differenttasks"; //has completely different tasks but engineered such that there needs to be a wait before the door is checked 
			String different_goals_example_longer="a_door_example_differenttasks_longer";
			String filename = different_goals_example;
			
			
			String filename_suffix = "";// "_seq"; //seq_simp for two robots
			boolean includefailstatesinswitches = false;
			boolean matchsharedstatesinswitch = true;
			boolean completeSwitchRing = false;//true;
			ArrayList<String> shared_vars_list = new ArrayList<String>();
			shared_vars_list.add("door");

			ArrayList<String> filenames = new ArrayList<String>();
			filenames.add(filename + 0);
			filenames.add(filename + 1);
		
	


//		 	filenames.add(filename + 2);
		
			// filenames.add(filename+3);
			// filenames.add("chain_example_simple_mod");

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
			// Initialise PRISM engine
			Prism prism = new Prism(mainLog);
			prismC = prism;
			prism.initialise();

			for (int files = 0; files < filenames.size(); files++) {
				// Parse and load a PRISM model from a file
				ModulesFile modulesFile = prism
						.parseModelFile(new File(modelLocation + filenames.get(files) + ".prism"));
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
			}
			// Build an MDP model checker
			// MDPModelChecker mc = new MDPModelChecker(prism);

			// mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile,
			// currentModelGenerator);

			// Model check the first property from the file using the model checker
			for (int i = 0; i < propFiles.size(); i++) {
				mainLog.println(propFiles.get(i).getPropertyObject(0));
			}
			Expression expr = propFiles.get(0).getProperty(0);

			// I dont know if this is the best way to do this
			// Doesnt look like this works
			// But yeah, maybe move this bit in the while loop in doSTAPU
			// models.add(mdp);

			ExecutorService executor = Executors.newSingleThreadExecutor();
			StatesHelper.setNumMDPVars(maxMDPVars);
			Runnable task = new Runnable() {
				@Override
				public void run() {
					// do your task
					try {
						doSTAPU(models, (ExpressionFunc) expr, null, new ProbModelChecker(prism), modulesFiles,
								shared_vars_list, includefailstatesinswitches, matchsharedstatesinswitch,completeSwitchRing);
					} catch (PrismException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			};

			Future<?> future = executor.submit(task);

			try {
				future.get(timeout, TimeUnit.MILLISECONDS);

			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (ExecutionException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (TimeoutException e) {
				// TODO Auto-generated catch block
				if (jointPolicy != null)
					mainLog.println("States " + jointPolicy.allFailStatesSeen.toString());
				e.printStackTrace();
			} // awaits termination

			// Close down PRISM
			prism.closeDown();

		} catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
	}

}
