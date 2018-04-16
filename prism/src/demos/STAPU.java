package demos;


import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
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
import explicit.DAInfo;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSparse;
import explicit.MMDPSimple;
import explicit.Model;
import explicit.ModelCheckerMultipleResult;
import explicit.ModelCheckerPartialSatResult;
import explicit.ProbModelChecker;
import explicit.SequentialTeamMDP;
import explicit.SingleAgentNestedProductMDP;
import explicit.StateValues;
import explicit.StatesHelper;
import explicit.LTLModelChecker.LTLProduct;
import explicit.MMDPSimple.StateProb;
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
import prism.Result;


public class STAPU {
	
	
	String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	long timeout = 1000 *60*1000;
	public PrismLog mainLogRef;
	Prism prismC; 
	MMDPSimple jointPolicy; 

	
	public static void main(String[] args)
	{
		STAPU stapu = new STAPU(); 
		stapu.run();
	}
	
	private boolean hasTimedOut(long startTime, long endTime,String text)
	{
		long time = endTime-startTime; 
		printTime(time,text);
		if(time > timeout)
			{System.out.println("Timed Out");
			return true; }
		else
			return false; 
	}
	private boolean hasTimedOut(long startTime,String text)
	{
		long endTime = System.currentTimeMillis(); 
		return hasTimedOut(startTime,endTime,text);
		
	}
	

	private void printTime(long time, String text)
	{
		if (text == "")
			text = "Time";
		System.out.println(text+": "+time/1000.0+" seconds");
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
	protected SingleAgentNestedProductMDP buildSingleAgentNestedProductMDP(Model model, ArrayList<DAInfo> daList,
			BitSet statesOfInterest,ProbModelChecker mcProb,ModulesFile modulesFile) throws PrismException {
		// return the list of daInfo and the product mdp
		SingleAgentNestedProductMDP res = new SingleAgentNestedProductMDP(mainLogRef);
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
		for (int daNum = 0; daNum < daList.size(); daNum++) {
			DAInfo daInfo = daList.get(daNum);
			product = daInfo.constructDAandProductModel(mcLTL, mcProb, modulesFile, allowedAcceptance, productMDP, null, true);
			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);
			StatesHelper.saveMDP(productMDP, daInfo.productAcceptingStates, "","pda_" + daNum, true);
			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				daList.get(otherDAs).updateStateNumbers(product);
			}
			res.updateProductToMDPStateMapping(product);
		}
		res.setDAListAndFinalProduct(daList, product);
		return res;
	}
	
	protected ModelCheckerMultipleResult computeNestedValIterFailurePrint(MDP mdp, BitSet target,
			BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards, ArrayList<Boolean>minRewards) throws PrismException {

		BitSet statesToRemainIn = (BitSet) statesToAvoid.clone();
		statesToRemainIn.flip(0, mdp.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(prismC);
//		mc.genStrat = true;
		//lets set them all to true 
		
		
		ModelCheckerMultipleResult res2 = mc.computeNestedValIterArray(mdp, target, statesToRemainIn,
				rewards,minRewards);

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
			BitSet statesToAvoid, ArrayList<MDPRewardsSimple> rewards) throws PrismException {

	
		ArrayList<Boolean> minMaxRew = new ArrayList<Boolean>(); 
		for(int rew = 0; rew<rewards.size(); rew++)
			minMaxRew.add(true);
		return computeNestedValIterFailurePrint(mdp,target,statesToAvoid,rewards,minMaxRew);
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

	protected void doSTAPU(Model model, ExpressionFunc expr, BitSet statesOfInterest,ProbModelChecker mcProb, ModulesFile modulesFile) throws PrismException {

		long startTime = System.currentTimeMillis();
		
		//do we want to use an executorservice ? 
		//lets do this when we separate this stuff from prism 
		//then it would make more sense 
		//for now we'll just check over and over 
		
//		class MyTask implements Runnable
//		{
//		    public void run() { 
//		        // add your code here
//		    }
//		}
//
////		Then we can use ExecutorService like this,
//
//		ExecutorService executor = Executors.newSingleThreadExecutor();
//		executor.invokeAll(Arrays.asList(new MyTask()), 10, TimeUnit.SECONDS); // Timeout of 10 seconds.
//		executor.shutdown();
		
		// process ltl expressions
		int numRobots = getNumRobots(exampleNumber());
		
		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = new ArrayList<SingleAgentNestedProductMDP>();
		ArrayList<Expression> ltlExpressions = getLTLExpressions(expr);
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions);
		if (hasTimedOut(startTime,"Initialized DA from LTL Expressions"))
			return;
		StatesHelper.saveMDP((MDP) model, null, "","mdp", true);
		if (hasTimedOut(startTime,"Saved original MDP"))
			return;
		int initState = model.getFirstInitialState();
		
		for (int i = 0; i < numRobots; i++) {
			if (i != 0) {
				initState = getInitState(i, exampleNumber());
				((MDPSparse) model).clearInitialStates();
				((MDPSparse) model).addInitialState(initState);

			}
			
			SingleAgentNestedProductMDP nestedProduct = buildSingleAgentNestedProductMDP(model, daList,
					statesOfInterest,mcProb,modulesFile);

			singleAgentProductMDPs.add(nestedProduct);
			if (hasTimedOut(startTime,"Created Nested Product "+i))
				return;

		}

		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP =  new SequentialTeamMDP(this.mainLogRef,numRobots); //buildSequentialTeamMDPTemplate(singleAgentProductMDPs);
		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs);

		if (hasTimedOut(startTime,"Created Sequential MDP Template"))
			return;
		
		int firstRobot = 0; // fix this
		StatesHelper.setMDPVar(seqTeamMDP.teamMDPTemplate.getVarList().getNumVars() - 1);
		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot);
		
		if (hasTimedOut(startTime,"Created Sequential MDP with Switches"))
			return;
		
		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++)
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));
		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates,"", "teamMDPWithSwitches", true);
		StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.statesToAvoid, "","teamMDPWithSwitchesAvoid", true);
		
		// solve
//		ModelCheckerPartialSatResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
//				seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, seqTeamMDP.rewardsWithSwitches.get(0));
		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>(seqTeamMDP.rewardsWithSwitches); 
		ArrayList<Boolean> minRewards = new ArrayList<Boolean>(); 
		for(int rew = 0; rew<rewards.size(); rew++)
		{
			minRewards.add(true);
		}
		rewards.add(seqTeamMDP.progressionRewards);

		minRewards.add(false);
		
		ModelCheckerMultipleResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
				seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, rewards);
		
		if (hasTimedOut(startTime,"Solved Initial Sequential MDP"))
			return;
		
		
		// add to joint policy
		//MMDPSimple
		jointPolicy = new MMDPSimple(seqTeamMDP.numRobots);
		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, true);

		if (hasTimedOut(startTime,"Added Initial Solution to Joint Policy"))
			return;
		
		StatesHelper.saveMDP(jointPolicy.mdp, null, "","jointPolicy", true);
		
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
			
			
			StatesHelper.saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates,"",
					"teamMDPWithSwitches" + currState.toString(), true);
			
			solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
					seqTeamMDP.statesToAvoid, rewards);
//			solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
//					seqTeamMDP.statesToAvoid, seqTeamMDP.rewardsWithSwitches.get(0));
			
			
			if (hasTimedOut(startTime,"Solved for Fail State"))
				return;
			
			
			jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, false);
			
			if (hasTimedOut(startTime,"Added Solution to Joint Policy"))
				return;
			
		}

		mainLogRef.println("Completed STAPU for full policy");
		mainLogRef.println("DeadEnd States " + jointPolicy.deadendStates.toString());
		mainLogRef.println("Accepting States " + jointPolicy.allTasksCompletedStates.toString());
		mainLogRef.println("Information about fail states ");

		for (StateProb fs : orderOfFailStates) {
			double prob = jointPolicy.getProbabilityAcceptingStateOnly(fs.getState(), 1.0);
			mainLogRef.println("Explored state " + fs.toString() + " with prob " + prob + "= " + prob * fs.getProb());
		}

		hasTimedOut(startTime,"All Done");
		
	}

	private int exampleNumber() {
		// SET the EXAMPLE NUMBER HERE
		// 0 = two room three robot not extended , 1 = two room three robot extended, 2
		// = debugging on
		// two_room_three_robot_blowup_reduced, 3 = three_robot_simple
		// 4 = topo_map
		// 5= chain example

		return 5;
	}

	private int getNumRobots(int exampleNumber) {
		int toret = -1;
		switch (exampleNumber) {
		// case 0:
		// toret =1;
		// break;
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


	
	public void run()
	{
		try {
			String modelLocation= "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
			String filename = "chain_example";
			// Create a log for PRISM output (hidden or stdout)
//			PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");
			this.mainLogRef = mainLog; 

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);
			prismC = prism;
			prism.initialise();

			// Parse and load a PRISM model from a file
			ModulesFile modulesFile = prism.parseModelFile(new File(modelLocation+filename+".prism"));
			prism.loadPRISMModel(modulesFile);

			// Parse and load a properties model for the model
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(modelLocation+filename+".prop"));

			// Get PRISM to build the model and then extract it
			prism.setEngine(Prism.EXPLICIT);
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit(); 
			
			// Build an MDP model checker
			MDPModelChecker mc = new MDPModelChecker(prism);
			
			//mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, currentModelGenerator);
			
			// Model check the first property from the file using the model checker
			System.out.println(propertiesFile.getPropertyObject(0));
			Expression expr = propertiesFile.getProperty(0);
			ProbModelChecker mcProb = new ProbModelChecker(prism);
			
			
			// I dont know if this is the best way to do this 
			// Doesnt look like this works 
			// But yeah, maybe move this bit in the while loop in doSTAPU 

			ExecutorService executor = Executors.newSingleThreadExecutor();

		    Runnable task = new Runnable() {
		        @Override
		        public void run() {
		            //do your task
		        	try {
						doSTAPU(mdp,(ExpressionFunc) expr,null,new ProbModelChecker(prism),modulesFile);
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
