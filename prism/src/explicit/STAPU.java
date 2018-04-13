/**
 * 
 */
package explicit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Vector;

import acceptance.AcceptanceType;
import explicit.LTLModelChecker.LTLProduct;
import explicit.MMDPSimple.StateProb;
import explicit.rewards.MDPRewards;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

/**
 * @author fatma
 *
 */
public class STAPU extends ProbModelChecker {

	String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	long timeout = 1000 * 3 * 60;
	public PrismLog mainLogRef;

	public STAPU(MDPModelChecker parent) throws PrismException {
		super(parent);
		this.modulesFile = parent.modulesFile;
		this.propertiesFile = parent.propertiesFile;
		this.constantValues = parent.constantValues;
		mainLogRef = mainLog;

		// mc.setModulesFileAndPropertiesFile(currentModelInfo, propertiesFile,
		// currentModelGenerator);

	}
	private boolean hasTimedOut(long startTime, long endTime)
	{
		long time = endTime-startTime; 
		printTime(time);
		if(time > timeout)
			{System.out.println("Timed Out");
			return true; }
		else
			return false; 
	}
	private boolean hasTimedOut(long startTime)
	{
		long endTime = System.currentTimeMillis(); 
		return hasTimedOut(startTime,endTime);
		
	}
	
	private void printTime(long time)
	{
		System.out.println("Time: "+time/1000.0+" seconds");
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
			BitSet statesOfInterest) throws PrismException {
		// return the list of daInfo and the product mdp
		SingleAgentNestedProductMDP res = new SingleAgentNestedProductMDP(this);
		res.initializeProductToMDPStateMapping((MDP) model);
		LTLProduct<MDP> product = null;
		MDP productMDP = null;
		LTLModelChecker mcLTL = new LTLModelChecker(this); // is this okay ?
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };

		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		// all the states are states of interest
		bsInit.set(0, numStates);
		productMDP = (MDP) model;
		for (int daNum = 0; daNum < daList.size(); daNum++) {
			DAInfo daInfo = daList.get(daNum);
			product = daInfo.constructDAandProductModel(mcLTL, this, allowedAcceptance, productMDP, null, true);
			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);
			saveMDP(productMDP, daInfo.productAcceptingStates, "pda_" + daNum, true);
			// update state numbers
			for (int otherDAs = 0; otherDAs < daNum; otherDAs++) {
				daList.get(otherDAs).updateStateNumbers(product);
			}
			res.updateProductToMDPStateMapping(product);
		}
		res.setDAListAndFinalProduct(daList, product);
		return res;
	}

	@Override
	protected StateValues checkExpressionFunc(Model model, ExpressionFunc expr, BitSet statesOfInterest)
			throws PrismException {
		switch (expr.getNameCode()) {
		case ExpressionFunc.STAPU: {
			mainLog.println("Calling doSTAPU");

			return doSTAPU(model, expr, statesOfInterest); // return function name;

		}
		default:
			return super.checkExpressionFunc(model, expr, statesOfInterest);
		}
	}

	protected ModelCheckerPartialSatResult computeNestedValIterFailurePrint(MDP sumprod, BitSet accStatesF,
			BitSet badStates, MDPRewards rewards) throws PrismException {

		BitSet statesToRemainIn = (BitSet) badStates.clone();
		statesToRemainIn.flip(0, sumprod.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(this);
		mc.genStrat = true;
		ModelCheckerPartialSatResult res2 = mc.computeNestedValIterFailure(sumprod, accStatesF, statesToRemainIn,
				rewards);

		StateValues probsProduct = StateValues.createFromDoubleArray(res2.solnProb, sumprod);

		// Get final prob result
		double maxProb = probsProduct.getDoubleArray()[sumprod.getFirstInitialState()];

		StateValues costsProduct = StateValues.createFromDoubleArray(res2.solnCost, sumprod);

		double minCost = costsProduct.getDoubleArray()[sumprod.getFirstInitialState()];

		mainLog.println("\nFor p = " + maxProb + ", the minimum expected cummulative cost to satisfy specification is "
				+ minCost);
		return res2;
	}

	protected StateValues doSTAPU(Model model, ExpressionFunc expr, BitSet statesOfInterest) throws PrismException {

		StateValues res = null;
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
		if (hasTimedOut(startTime))
			return res;
		saveMDP((MDP) model, null, "mdp", true);
		if (hasTimedOut(startTime))
			return res;
		int initState = model.getFirstInitialState();
		
		for (int i = 0; i < numRobots; i++) {
			if (i != 0) {
				initState = getInitState(i, exampleNumber());
				((MDPSparse) model).clearInitialStates();
				((MDPSparse) model).addInitialState(initState);

			}

			SingleAgentNestedProductMDP nestedProduct = buildSingleAgentNestedProductMDP(model, daList,
					statesOfInterest);

			singleAgentProductMDPs.add(nestedProduct);
			if (hasTimedOut(startTime))
				return res;

		}

		// create team automaton from a set of MDP DA stuff
		SequentialTeamMDP seqTeamMDP =  new SequentialTeamMDP(this,numRobots); //buildSequentialTeamMDPTemplate(singleAgentProductMDPs);
		seqTeamMDP = seqTeamMDP.buildSequentialTeamMDPTemplate(singleAgentProductMDPs);

		if (hasTimedOut(startTime))
			return res;
		
		int firstRobot = 0; // fix this
		StatesHelper.setMDPVar(seqTeamMDP.teamMDPTemplate.getVarList().getNumVars() - 1);
		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot);
		
		if (hasTimedOut(startTime))
			return res;
		
		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++)
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));
		saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates, "teamMDPWithSwitches", true);
		saveMDP(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.statesToAvoid, "teamMDPWithSwitchesAvoid", true);
		
		// solve
		ModelCheckerPartialSatResult solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
				seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, seqTeamMDP.rewardsWithSwitches.get(0));
		
		if (hasTimedOut(startTime))
			return res;
		
		
		// add to joint policy
		MMDPSimple jointPolicy = new MMDPSimple(seqTeamMDP.numRobots);
		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, true);

		if (hasTimedOut(startTime))
			return res;
		
		saveMDP(jointPolicy.mdp, null, "jointPolicy", true);
		
		Vector<StateProb> orderOfFailStates = new Vector<StateProb>();
		
		//start the loop 
		while (!jointPolicy.stuckStatesQ.isEmpty()) {

			//get the state with the highest probability, given the current joint policy 
			StateProb stuckState = jointPolicy.stuckStatesQ.remove();
			initialState = stuckState.getState();
			orderOfFailStates.add(stuckState.copy());
			mainLog.println("Exploring " + stuckState.toString());

			//update the teammdp 
			List<State> states = jointPolicy.mdp.getStatesList();// seqTeamMDP.teamMDPWithSwitches.getStatesList();
			State currState = states.get(initialState);
			int rNum = jointPolicy.firstFailedRobot(currState);
			int[] robotStates = jointPolicy.getRobotStatesIndexFromJointState(currState,
					seqTeamMDP.teamMDPWithSwitches.getStatesList());
			seqTeamMDP.setInitialStates(robotStates);
			
			if (hasTimedOut(startTime))
				return res;
			
			
			// we need to change the switches to the initial states
			// so we will just add switches from all failed robots 
			// we really dont care who failed first because 
			// the switches will take care of that 
			seqTeamMDP.addSwitchesAndSetInitialState(rNum);
			
			if (hasTimedOut(startTime))
				return res;
			
			
			saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates,
					"teamMDPWithSwitches" + currState.toString(), true);
			
			solution = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.acceptingStates,
					seqTeamMDP.statesToAvoid, seqTeamMDP.rewardsWithSwitches.get(0));
			
			
			if (hasTimedOut(startTime))
				return res;
			
			
			jointPolicy.addSeqPolicyToJointPolicy(seqTeamMDP, solution.strat, initialState, false);
			
			if (hasTimedOut(startTime))
				return res;
			
		}

		mainLog.println("Completed STAPU for full policy");
		mainLog.println("DeadEnd States " + jointPolicy.deadendStates.toString());
		mainLog.println("Accepting States " + jointPolicy.allTasksCompletedStates.toString());
		mainLog.println("Information about fail states ");

		for (StateProb fs : orderOfFailStates) {
			double prob = jointPolicy.getProbabilityAcceptingStateOnly(fs.getState(), 1.0);
			mainLog.println("Explored state " + fs.toString() + " with prob " + prob + "= " + prob * fs.getProb());
		}

		hasTimedOut(startTime);
		
		return res;
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
			DAInfo daInfo = new DAInfo(this, thisExpr, hasReward);
			daInfoList.add(daInfo);
		}

		return daInfoList;
	}

	/**
	 * @param mdp The mdp
	 * 
	 * @param statesToMark a bitset for states you'd like to highlight in the mdp
	 * 
	 * @param name filename
	 * 
	 * @param saveinsaveplace save in predefined save location (set to true) if
	 * false saves in same location as adversary
	 */
	protected void saveMDP(MDP mdp, BitSet statesToMark, String name, boolean saveinsaveplace) {
		String temp = exportAdvFilename;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = saveplace;
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location + name + ".dot");
		mdp.exportToDotFile(out, statesToMark, true);
		out.close();

	}
}
