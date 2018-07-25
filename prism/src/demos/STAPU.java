package demos;


import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
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
import explicit.MDPSimple;
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
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;


public class STAPU {
	
	static String saveplace_suffix = "/tests/decomp_tests/IROS_2018_final_submission/res/";
	static String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	long timeout = 1000 *60*1000;
	public PrismLog mainLogRef;
	Prism prismC; 
	MMDPSimple jointPolicy; 
	boolean hasDoor = true;
	boolean noPrintouts = true;
	
	public static void main(String[] args)
	{
		String dir = System.getProperty("user.dir"); 
		saveplace = dir+saveplace_suffix;
		StatesHelper.setSavePlace(saveplace);
		STAPU stapu = new STAPU(); 
		try {
			stapu.runIROS2018();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private boolean hasTimedOut(long startTime, long endTime,String text)
	{
		long time = endTime-startTime; 
		printTime(time,text);
//		if(time > timeout)
//			{System.out.println("Timed Out");
//			return true; }
//		else
			return false; 
	}
	private boolean hasTimedOut(long startTime,String text)
	{
		long endTime = System.currentTimeMillis(); 
		return hasTimedOut(startTime,endTime,text);
		
	}
	private long getTime(long startTime)
	{
		return System.currentTimeMillis()-startTime;
	}

	private void printTime(long time, String text)
	{
		if (text == "")
			text = "Time";
		System.out.println(text+": "+time/1000.0+" seconds ("+time+"s, "+time/(1000.0*60)+" mins) ");
	}
	
	/**
	 * @param model - the mdp model
	 * 
	 * @param exprs - array list of expressions
	 * 
	 * @param statesOfInterest - states to care about - we care about everything so
	 * we don't really need this
	 * 
	 */
	protected SingleAgentNestedProductMDP buildSingleAgentNestedProductMDP(String name,Model model, ArrayList<DAInfo> daList,
			BitSet statesOfInterest,ProbModelChecker mcProb,ModulesFile modulesFile) throws PrismException {
		// return the list of daInfo and the product mdp
 		SingleAgentNestedProductMDP res = new SingleAgentNestedProductMDP(mainLogRef);
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
			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);
			
			if (!noPrintouts) {
			StatesHelper.saveDA(daInfo.da, "", name+"da_"+daNum, true);
			StatesHelper.saveMDP(productMDP, daInfo.productAcceptingStates, "",name+"pda_" + daNum, true);
			StatesHelper.saveMDP(productMDP, daInfo.essentialStates, "",name+"pda_" + daNum+"switchStates", true);
			}
			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				res.daList.get(otherDAs).updateStateNumbers(product);
			}
			res.updateProductToMDPStateMapping(product);
			res.daList.add(daInfo);
		}
		res.setDAListAndFinalProduct(product);
		return res;
	}
	
	protected ModelCheckerResult computeUntilProbs(MDP mdp, BitSet target,
			BitSet statesToAvoid ) throws PrismException {
		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(prismC);
		mc.setGenStrat(true);
//		mc.genStrat = true;
		//lets set them all to true 
		ModelCheckerResult anotherSol = mc.computeUntilProbs(mdp, statesToRemainIn, target, false);
		
		if (!noPrintouts)
		StatesHelper.saveStrategy(anotherSol.strat, target, "", "computeUntilProbsStrat"+mdp.getFirstInitialState(), true);
		
		StateValues testValues = StateValues.createFromDoubleArray(anotherSol.soln, mdp); 
		mainLogRef.println("Compute Until Probs Vals\n "+Arrays.toString(testValues.getDoubleArray())); 
		mainLogRef.println("Prob in init"+testValues.getDoubleArray()[mdp.getFirstInitialState()]);
		double[] solnProb = anotherSol.soln;
		StateValues probsProduct = StateValues.createFromDoubleArray(solnProb, mdp);

		// Get final prob result
		
		double maxProb = probsProduct.getDoubleArray()[mdp.getFirstInitialState()];

		mainLogRef.println("\nFor p = " + maxProb );
		return anotherSol;
	}
	
	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target,
			BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards, ArrayList<Boolean>minRewards,int probPreference, double[] probInitVal) throws PrismException {

		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(prismC);
		mc.setGenStrat(true);
//		mc.genStrat = true;
		//lets set them all to true 
		ModelCheckerResult anotherSol = mc.computeUntilProbs(mdp, statesToRemainIn, target, false);
		
		if(!noPrintouts)
		StatesHelper.saveStrategy(anotherSol.strat, target, "", "computeUntilProbsStrat"+mdp.getFirstInitialState(), true);
		
		StateValues testValues = StateValues.createFromDoubleArray(anotherSol.soln, mdp); 
		mainLogRef.println("Compute Until Probs Vals\n "+Arrays.toString(testValues.getDoubleArray())); 
		mainLogRef.println("Prob in init"+testValues.getDoubleArray()[mdp.getFirstInitialState()]);
		
		ModelCheckerMultipleResult res2 = mc.computeNestedValIterArray(mdp, target, statesToRemainIn,
				rewards,null,minRewards,target,probPreference,probInitVal);

		ArrayList<double[]> solns = res2.solns;
		double[] solnProb = solns.get(solns.size()-1);
		StateValues probsProduct = StateValues.createFromDoubleArray(solnProb, mdp);

		// Get final prob result
		
		double maxProb = probsProduct.getDoubleArray()[mdp.getFirstInitialState()];

		String resString = "";
		for(int i = 0; i<solns.size()-1; i++) {
		StateValues costsProduct = StateValues.createFromDoubleArray(res2.solns.get(i), mdp);

		double minCost = costsProduct.getDoubleArray()[mdp.getFirstInitialState()];
		resString+=i+":"+minCost+" ";
		
		}
		mainLogRef.println("\nFor p = " + maxProb + ", rewards "
				+ resString);
		
				return res2;
	}
	
	
	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target,
			BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards, ArrayList<Boolean>minRewards,int probPreference) throws PrismException {


		ModelCheckerMultipleResult res2 =  computeNestedValIterFailurePrint(mdp, target,
				statesToAvoid,rewards,minRewards,probPreference, null);// computeNestedValIterFailurePrint(mdp, target, statesToAvoid,
				//rewards,minRewards,target,probPreference,null);

		return res2;
	}
	
	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target,
			BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards,int probPreference) throws PrismException {

	
		ArrayList<Boolean> minMaxRew = new ArrayList<Boolean>(); 
		for(int rew = 0; rew<rewards.size(); rew++)
			minMaxRew.add(true);
		return computeNestedValIterFailurePrint(mdp,target,statesToAvoid,rewards,minMaxRew,probPreference);
	}

	protected ModelCheckerPartialSatResult computeNestedValIterFailurePrint(MDP mdp, BitSet target,
			BitSet statesToAvoid, MDPRewards rewards) throws PrismException {

		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(prismC);
		
		ModelCheckerPartialSatResult res2 = mc.computeNestedValIterFailure(mdp, target, statesToRemainIn,
				rewards);

		StateValues probsProduct = StateValues.createFromDoubleArray(res2.solnProb, mdp);

		// Get final prob result
		double maxProb = probsProduct.getDoubleArray()[mdp.getFirstInitialState()];

		StateValues costsProduct = StateValues.createFromDoubleArray(res2.solnCost, mdp);

		double minCost = costsProduct.getDoubleArray()[mdp.getFirstInitialState()];

		mainLogRef.println("\nFor p = " + maxProb + ", the minimum expected cummulative cost to satisfy specification is "
				+ minCost);
		return res2;
	}

	
	
	protected void doSTAPU(ArrayList<Model> models, ExpressionFunc expr, BitSet statesOfInterest,ProbModelChecker mcProb, ArrayList<ModulesFile> modulesFiles) throws PrismException {

		long startTime = System.currentTimeMillis();
		int probPreference = 0; 
		
		// process ltl expressions
		int numRobots = getNumRobots(exampleNumber());
		boolean sameModelForAll = false;
		if(numRobots != models.size() && models.size()==1)
			sameModelForAll = true;
		else
			numRobots = models.size();
		
		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = new ArrayList<SingleAgentNestedProductMDP>();
		ArrayList<Expression> ltlExpressions = getLTLExpressions(expr);
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions);
		if (hasTimedOut(startTime,"Initialized DA from LTL Expressions"))
			return;
		Model model = models.get(0);
		ModulesFile modulesFile = modulesFiles.get(0);
		for(int i = 0; i<numRobots; i++) {
			if(!sameModelForAll)
			{model = models.get(i);
			modulesFile = modulesFiles.get(i);
			}
		if(!noPrintouts)
		StatesHelper.saveMDP((MDP) model, null, "","mdp"+i, true);
		if (hasTimedOut(startTime,"Saved original MDP"))
			return;
		int initState = model.getFirstInitialState();
		
//		for (int i = 0; i < numRobots; i++) {
			if (i != 0 && sameModelForAll) {
				initState = getInitState(i, exampleNumber());
				((MDPSparse) model).clearInitialStates();
				((MDPSparse) model).addInitialState(initState);

			}
		
			SingleAgentNestedProductMDP nestedProduct = buildSingleAgentNestedProductMDP(""+i,model, daList,
					statesOfInterest,mcProb,modulesFile);

			singleAgentProductMDPs.add(nestedProduct);
			if (hasTimedOut(startTime,"Created Nested Product "+i))
				return;

//		}
		
		}
		
		
		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP =  new SequentialTeamMDP(this.mainLogRef,numRobots); //buildSequentialTeamMDPTemplate(singleAgentProductMDPs);
		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs);
		StatesHelper.writeToDataColl(""+getTime(startTime));
		if (hasTimedOut(startTime,"Created Sequential MDP Template"))
			return;
		
		int firstRobot = 0; // fix this
//		if (hasDoor)
//		StatesHelper.setMDPVar(seqTeamMDP.teamMDPTemplate.getVarList().getNumVars() - 2); //cuz there is door too 
//		else
		StatesHelper.setMDPVar(seqTeamMDP.teamMDPTemplate.getVarList().getNumVars() - StatesHelper.numMdpVars); //cuz there is door too 
		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot);
		
		if (hasTimedOut(startTime,"Created Sequential MDP with Switches"))
			return;
		
		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++)
			{combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));
			if (!noPrintouts)
			mainLogRef.println("Essential States "+seqTeamMDP.essentialStates.get(i)); 
			
			}
		if(!noPrintouts)
		mainLogRef.println("Accepting States "+seqTeamMDP.acceptingStates); 
		if(!noPrintouts) {
		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates,"", "teamMDPWithSwitches", true);
		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.statesToAvoid, "","teamMDPWithSwitchesAvoid", true);
		StatesHelper.saveMDPstatra(seqTeamMDP.teamMDPWithSwitches, "", "teamMDPWithSwitches_sta_tra", true);
		}

//		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>(seqTeamMDP.rewardsWithSwitches); 
//		ArrayList<Boolean> minRewards = new ArrayList<Boolean>(); 
//		for(int rew = 0; rew<rewards.size(); rew++)
//		{
//			minRewards.add(true);
//		}
//		rewards.add(0,seqTeamMDP.progressionRewards);
//
//		minRewards.add(0,false);
		
		combinedEssentialStates.or(seqTeamMDP.acceptingStates);
//		for(int rew = 0; rew<rewards.size(); rew++)
//		{
//			StatesHelper.saveReward(seqTeamMDP.teamMDPWithSwitches, rewards.get(rew),combinedEssentialStates ,
//					"", "rew"+rew, true);
//		}
		

		
//		ModelCheckerMultipleResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
//				seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, rewards,minRewards,probPreference);//,probInitVals);
	ModelCheckerResult solution = computeUntilProbs(seqTeamMDP.teamMDPWithSwitches,seqTeamMDP.acceptingStates,seqTeamMDP.statesToAvoid);
	
		if(!noPrintouts)
		StatesHelper.saveStrategy(solution.strat, null, "", "initialStrat", true);

		mainLogRef.println(seqTeamMDP.acceptingStates.toString());
		if (hasTimedOut(startTime,"Solved Initial Sequential MDP"))
			return;
		
		
		// add to joint policy
		//MMDPSimple
		jointPolicy = new MMDPSimple(seqTeamMDP.numRobots,seqTeamMDP.agentMDPs.get(0).daList.size());
		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		mainLogRef.println("InitState = "+initialState);
		
		jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, true);

		if (hasTimedOut(startTime,"Added Initial Solution to Joint Policy"))
			return;
		
		StatesHelper.saveMDP(jointPolicy.mdp, null, "","jointPolicy", true);
		boolean stopHere =false;// true; 
		if (stopHere)
		return;
		Vector<StateProb> orderOfFailStates = new Vector<StateProb>();
		
		//start the loop 
		while (!jointPolicy.stuckStatesQ.isEmpty()) {

			//get the state with the highest probability, given the current joint policy 
			StateProb stuckState = jointPolicy.stuckStatesQ.remove();
			initialState = stuckState.getState();
			orderOfFailStates.add(stuckState.copy());
			mainLogRef.println("Exploring " + stuckState.toString());

			//update the teammdp 
			List<State> states = jointPolicy.mdp.getStatesList();// seqTeamMDP.teamMDPWithSwitches.getStatesList();
			State currState = states.get(initialState);
			int rNum = jointPolicy.firstFailedRobot(currState);
			int[] robotStates = jointPolicy.getRobotStatesIndexFromJointState(currState,
					seqTeamMDP.teamMDPWithSwitches.getStatesList());
			seqTeamMDP.setInitialStates(robotStates);
			
			if (hasTimedOut(startTime,"Set Initial States for Fail State"))
				return;
			
			
			// we need to change the switches to the initial states
			// so we will just add switches from all failed robots 
			// we really dont care who failed first because 
			// the switches will take care of that 
			seqTeamMDP.addSwitchesAndSetInitialState(rNum);
			
			if (hasTimedOut(startTime,"Added Switches"))
				return;
			
			if(!noPrintouts)
			StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates,"",
					"teamMDPWithSwitches" + currState.toString(), true);
			
			solution = computeUntilProbs(seqTeamMDP.teamMDPWithSwitches,seqTeamMDP.acceptingStates,seqTeamMDP.statesToAvoid);
			//computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
			//		seqTeamMDP.statesToAvoid, rewards,probPreference);
			if(!noPrintouts)
			StatesHelper.saveStrategy(solution.strat, null, "", "stratFor"+currState.toString(), true);
			if (hasTimedOut(startTime,"Solved for Fail State"))
				return;
			
			
			jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, false);
			
			if (hasTimedOut(startTime,"Added Solution to Joint Policy"))
				return;
			
		}
		StatesHelper.saveMDP(jointPolicy.mdp, null, "", "finalJointPolicy", true);
		StatesHelper.writeToDataColl(""+getTime(startTime));
		StatesHelper.saveMDPstatra(jointPolicy.mdp, "", "finalJointPolicy", true);
		jointPolicy.saveJointPolicy();
		mainLogRef.println("Completed STAPU for full policy");
		mainLogRef.println("DeadEnd States " + jointPolicy.deadendStates.toString());
		mainLogRef.println("Accepting States " + jointPolicy.allTasksCompletedStates.toString());
		mainLogRef.println("Information about fail states ");

		for (StateProb fs : orderOfFailStates) {
			double prob = jointPolicy.getProbabilityAcceptingStateOnly(fs.getState(), 1.0,new BitSet());
			mainLogRef.println("Explored state " + fs.toString() + " with prob " + prob + "= " + prob * fs.getProb());
		}

		hasTimedOut(startTime,"All Done");
		StatesHelper.writeToDataColl(""+orderOfFailStates.size());
	}
	


	private int exampleNumber() {
		// SET the EXAMPLE NUMBER HERE
		// 0 = two room three robot not extended , 1 = two room three robot extended, 2
		// = debugging on
		// two_room_three_robot_blowup_reduced, 3 = three_robot_simple
		// 4 = topo_map
		// 5= chain example
		// 6 = vi_example
		return 7;
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
		case 7:
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
			int notHave = 2;

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

		}
		else if(robotModel == 6)
		{
			switch (robotNum)
			{
			case 0:
				initState = 1; 
				break; 
			case 1:
				initState = 4; 
				break; 
			}
		}
		else if (robotModel == 7)
		{
			switch(robotNum)
			{
			case 0:
				initState = 21; 
				break; 
			case 1:
				initState = 24; 
				break; 
			case 2:
				initState = 6; 
				break; 
			case 3:
				initState = 15; 
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
		ArrayList<Expression> ltlExpressions = new ArrayList<Expression>(numOp);
		for (int exprNum = 0; exprNum < numOp; exprNum++) {
			if (expr.getOperand(exprNum) instanceof ExpressionQuant) {
				ltlExpressions.add((expr.getOperand(exprNum)));
				exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
			}
		}

		mainLogRef.println("LTL Mission: " + exprString);
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
			DAInfo daInfo = new DAInfo(mainLogRef, thisExpr, hasReward);
			daInfoList.add(daInfo);
		}

		return daInfoList;
	}

	public void runIROS2018() throws FileNotFoundException
	{
		String dir = System.getProperty("user.dir"); 
		String baseLocation= dir+"/tests/decomp_tests/IROS_2018_final_submission/";
		StatesHelper.openDataCollFile(baseLocation+"res.csv");
		try {
			int numfs = 5; 
			int numfiles = 2; 
			int propnum = 0;
			
			
			
			int[] fsvals = {5,10,15,20,25}; 
			int[] rvals = {4,8}; 
			int[] propvals = {0,1,2,3};
			
			for(int er = 0; er<rvals.length; er++)
			{
			numfiles = rvals[er];
			for(int et=0; et<propvals.length; et++)
			{
				propnum = propvals[et]; 
			for(int efs = 0; efs<fsvals.length; efs++)
			{
				numfs = fsvals[efs];
			
			
			StatesHelper.writeToDataColl("\n"+numfiles+"\t"+propnum*2+3+"\t"+numfs);
			String modelsuffix = "fs_"+numfs+"/";
			String filename ="topo_map_failbase_fs"+numfs+"_";//"topo_map_modified_goals";//"two_actions_spec";//"cant_complete_spec";//"two_actions_spec";//"can_complete_spec2pc";//"chain_example_simple_mod";//"alice_in_chains";// "chain_example";//"chain_example_simple_mod";// "vi_example";//"chain_example";
			String filename_suffix = "";//"_seq"; //seq_simp for two robots 
			String propfilename = "topo_map_modified_goals";

			String newsaveplace = baseLocation+modelsuffix+"res/r"+numfiles+"/t"+propnum; 
			new File(newsaveplace).mkdirs();
			StatesHelper.setSavePlace(newsaveplace+"/");
			
			ArrayList<String> filenames = new ArrayList<String>(); 
			for(int i = 0; i<numfiles; i++)
			filenames.add(filename+i); 

			StatesHelper.setFolder(baseLocation+modelsuffix+filename);
			hasDoor = false;
			int maxMDPVars = 0;
			
			ArrayList<Model> models = new ArrayList<Model>(); 
			ArrayList<PropertiesFile> propFiles = new ArrayList<PropertiesFile>();
			ArrayList<ModulesFile> modulesFiles = new ArrayList<ModulesFile>();

			// Create a log for PRISM output (hidden or stdout)
			PrismLog mainLog = new PrismDevNullLog();
	//		PrismLog mainLog = new PrismFileLog("stdout");
			this.mainLogRef = mainLog; 

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);
			prismC = prism;
			prism.initialise();

			for(int files = 0; files<filenames.size(); files++) {
			// Parse and load a PRISM model from a file
			ModulesFile modulesFile = prism.parseModelFile(new File(baseLocation+modelsuffix+filenames.get(files)+".prism"));
			modulesFiles.add(modulesFile);
			prism.loadPRISMModel(modulesFile);

			// Parse and load a properties model for the model
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(baseLocation+propfilename +filename_suffix+".prop"));

			// Get PRISM to build the model and then extract it
			prism.setEngine(Prism.EXPLICIT);
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit(); 
			int numVars = mdp.getVarList().getNumVars();
			if(numVars > maxMDPVars)
				maxMDPVars = numVars;
			
			models.add(mdp);
			propFiles.add(propertiesFile);
			}
			// Build an MDP model checker
			MDPModelChecker mc = new MDPModelChecker(prism);
			
			//mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, currentModelGenerator);
			
			// Model check the first property from the file using the model checker
//			for(int i = 0; i<propFiles.get(0).getNumProperties(); i++) {
//			System.out.println(propFiles.get(0).getPropertyObject(i));
//			}
			Expression expr = propFiles.get(0).getProperty(propnum);			
			

			StatesHelper.setNumMDPVars(maxMDPVars);
		
						doSTAPU(models,(ExpressionFunc) expr,null,new ProbModelChecker(prism),modulesFiles);
			
			// Close down PRISM
			prism.closeDown();
			}
			System.out.println("DONE\n"+numfiles+"\t"+(propnum*2+3)+"\t"+numfs);
			}
			}
			
			
		}
		catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
		StatesHelper.closeDataColl();
	}



	
	public void run()
	{
		try {
			String dir = System.getProperty("user.dir"); 
			String modelLocation= dir+"/tests/decomp_tests/IROS_2018_final_submission/";
			String filename ="topo_map_failbase_fs5_";//"topo_map_modified_goals";//"two_actions_spec";//"cant_complete_spec";//"two_actions_spec";//"can_complete_spec2pc";//"chain_example_simple_mod";//"alice_in_chains";// "chain_example";//"chain_example_simple_mod";// "vi_example";//"chain_example";
			String filename_suffix = "";//"_seq"; //seq_simp for two robots 
			String propfilename = "topo_map_modified_goals";
			ArrayList<String> filenames = new ArrayList<String>(); 
			filenames.add(filename+0); 
			filenames.add(filename+1);
//			filenames.add(filename+2);
//			filenames.add(filename+3);
//			filenames.add("chain_example_simple_mod");
			StatesHelper.setFolder(modelLocation+filename);
			hasDoor = false;
			int maxMDPVars = 0;
			
			ArrayList<Model> models = new ArrayList<Model>(); 
			ArrayList<PropertiesFile> propFiles = new ArrayList<PropertiesFile>();
			ArrayList<ModulesFile> modulesFiles = new ArrayList<ModulesFile>();

			// Create a log for PRISM output (hidden or stdout)
//			PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");
			this.mainLogRef = mainLog; 

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);
			prismC = prism;
			prism.initialise();

			for(int files = 0; files<filenames.size(); files++) {
			// Parse and load a PRISM model from a file
			ModulesFile modulesFile = prism.parseModelFile(new File(modelLocation+filenames.get(files)+".prism"));
			modulesFiles.add(modulesFile);
			prism.loadPRISMModel(modulesFile);

			// Parse and load a properties model for the model
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(modelLocation+propfilename +filename_suffix+".prop"));

			// Get PRISM to build the model and then extract it
			prism.setEngine(Prism.EXPLICIT);
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit(); 
			int numVars = mdp.getVarList().getNumVars();
			if(numVars > maxMDPVars)
				maxMDPVars = numVars;
			
			models.add(mdp);
			propFiles.add(propertiesFile);
			}
			// Build an MDP model checker
			MDPModelChecker mc = new MDPModelChecker(prism);
			
			//mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, currentModelGenerator);
			
			// Model check the first property from the file using the model checker
			for(int i = 0; i<propFiles.size(); i++) {
			System.out.println(propFiles.get(i).getPropertyObject(0));
			}
			Expression expr = propFiles.get(0).getProperty(0);			
			
			// I dont know if this is the best way to do this 
			// Doesnt look like this works 
			// But yeah, maybe move this bit in the while loop in doSTAPU 
//			models.add(mdp);
			ExecutorService executor = Executors.newSingleThreadExecutor();
			StatesHelper.setNumMDPVars(maxMDPVars);
		    Runnable task = new Runnable() {
		        @Override
		        public void run() {
		            //do your task
		        	try {
						doSTAPU(models,(ExpressionFunc) expr,null,new ProbModelChecker(prism),modulesFiles);
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
				if(jointPolicy!=null)
				mainLog.println("States "+jointPolicy.allFailStatesSeen.toString());
				e.printStackTrace();
			} // awaits termination 
			
			
			// Close down PRISM
			prism.closeDown();

			
		}
		catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
	}

}
