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

	boolean run_tests = false;
	String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	// System.getProperty("user.dir") + "/tests/decomp_tests/temp/";
	boolean printHighlights = false;
	boolean printDetails = false;
	private boolean saveIntermediates = false;
	boolean printTestRelated = true;
	ArrayList<ArrayList<Entry<String, Double>>> testResults;
	int arrNum = 0;
	long timeout = 1000 * 3 * 60;
	long time = 0;
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

	protected SequentialTeamMDP buildSequentialTeamMDPTemplate(ArrayList<SingleAgentNestedProductMDP> agentMDPs)
			throws PrismException {

		SequentialTeamMDP seqTeamMDP = new SequentialTeamMDP(this, agentMDPs.size());
		seqTeamMDP.agentMDPs = agentMDPs;
		seqTeamMDP.agentMDPsToSeqTeamMDPStateMapping = new ArrayList<int[]>(agentMDPs.size());

		int numRobots = agentMDPs.size();
		int numTeamStates = 0; // do I have an extra state somewhere ??
		// TODO: something seems to be off with the numbering of states and number of
		// states!
		MDPSimple teamMDP = null;
		// I dont know how many rewards there are
		// so lets just set like this unique rewards model thing ? yes okay
		// its still an arraylist though
		// woot
		ArrayList<MDPRewardsSimple> teamRewardsList = null;
		// number of states in team = sum of number of states in each mdp
		for (int r = 0; r < numRobots; r++) {
			numTeamStates += agentMDPs.get(r).finalProduct.getProductModel().getNumStates();
		}

		VarList teamMDPVarList = null;

		// the variable list - r,da0,da1...mdp
		// so we can just take the varlist for r1 and do the "needful" rukhsana :P

		MDP productMDP = agentMDPs.get(0).finalProduct.getProductModel();
		if (productMDP.getVarList() != null) {
			teamMDPVarList = (VarList) (productMDP.getVarList()).clone();

			String daVar = "r";// "_da"; // TODO: come back to fix this
			// while (teamMDPVarList.getIndex(daVar) != -1) {
			// daVar = "_" + daVar;
			// }
			// NB: if DA only has one state, we add an extra dummy state
			DeclarationInt daint = new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(numRobots - 1, 1)));
			Declaration decl = new Declaration(daVar, daint);
			teamMDPVarList.addVar(0, decl, 1, productMDP.getConstantValues());
			// //lets rename all the other ones
			// //so from 1 all the way to the end minus s
			// //we want to do stuff like dan, dan-1, dan-2
			// int numVar = teamMDPVarList.getNumVars();
			// for(int i = 1; i< numVar; i++)
			// {
			// decl = teamMDPVarList.getDeclaration(i);
			// decl.setName("da"+(numVar-(i-1)));
			// teamMDPVarList.addVar(i, decl, teamMDPVarList.getModule(i), constantValues);
			// }

		}

		teamMDP = new MDPSimple();
		teamRewardsList = new ArrayList<MDPRewardsSimple>();

		teamMDP.setVarList(teamMDPVarList);

		ArrayList<State> teamMDPStatesList = null, robotNumList = null;

		// again making assumption that what works for r1 works for all of them
		if (productMDP.getStatesList() != null) {
			teamMDPStatesList = new ArrayList<State>();
			robotNumList = new ArrayList<State>(numRobots);

			for (int i = 0; i < numRobots; i++) {
				robotNumList.add(new State(1).setValue(0, i));
			}
		}

		HashMap<Integer, Integer> rewardNumToCorrespondingDA = new HashMap<Integer, Integer>();
		// setting the rewards
		// basically we'll have the same number of rewards as in the mdp for any of the
		// robots
		for (int rew = 0; rew < agentMDPs.get(0).daList.size(); rew++) {
			if (agentMDPs.get(0).daList.get(rew).costsModel != null) {
				teamRewardsList.add(new MDPRewardsSimple(numTeamStates));
				rewardNumToCorrespondingDA.put(teamRewardsList.size() - 1, rew);
			}
		}

		ArrayList<int[]> maps = new ArrayList<int[]>();
		BitSet acceptingStates = new BitSet(numTeamStates); // for the team mdp they are acc for r1 || acc for r2 || acc
															// for r3 ...
		BitSet statesToAvoid = new BitSet(numTeamStates); // for the team mdp they are bad for r1 || bad for r2 || bad
															// for r3

		ArrayList<BitSet> initialStates = new ArrayList<BitSet>(); // for the team mdp they are different for each robot
		ArrayList<BitSet> switchStates = new ArrayList<BitSet>(); // for the team mdp they are different for each robot

		int numStates = 0;
		int numChoices;

		for (int r = 0; r < agentMDPs.size(); r++) {

			// check for self
			if (numStates * (r) != teamMDP.getNumStates())
				mainLog.println("Something is wrong here cuz the number of states isnt what you expected");
			SingleAgentNestedProductMDP singleAgentNestedMDP = agentMDPs.get(r);
			BitSet essentialStates = new BitSet(numTeamStates);
			BitSet agentInitialStates = singleAgentNestedMDP.getInitialStates();
			BitSet agentInitialStatesInTeam = new BitSet(numTeamStates);
			MDP agentMDP = agentMDPs.get(r).finalProduct.getProductModel();
			numStates = agentMDP.getNumStates();
			int[] map = new int[numStates];
			Arrays.fill(map, StatesHelper.BADVALUE);
			int indexInTeamState;
			for (int s = 0; s < numStates; s++) {

				// so we will not repeat any states cuz well we cant at all cuz we're adding
				// them as we find them
				// at least here

				indexInTeamState = map[s];
				if (indexInTeamState == StatesHelper.BADVALUE) {
					teamMDP.addState();
					if (teamMDPStatesList != null) {
						teamMDPStatesList.add(new State(robotNumList.get(r), agentMDP.getStatesList().get(s)));
					}
					indexInTeamState = teamMDP.getNumStates() - 1;
					// map it
					map[s] = indexInTeamState; // do I need this ??
				}
				// set the states
				if (singleAgentNestedMDP.combinedAcceptingStates.get(s)) {
					acceptingStates.set(indexInTeamState);
				}
				if (singleAgentNestedMDP.combinedStatesToAvoid.get(s)) {
					statesToAvoid.set(indexInTeamState);
				}

				if (singleAgentNestedMDP.combinedEssentialStates.get(s)) {
					essentialStates.set(indexInTeamState);
				}
				if (agentInitialStates.get(s)) {
					agentInitialStatesInTeam.set(indexInTeamState);
				}

				numChoices = agentMDP.getNumChoices(s);

				Iterator<Map.Entry<Integer, Double>> iter;

				for (int j = 0; j < numChoices; j++) {
					iter = agentMDP.getTransitionsIterator(s, j);
					Distribution distr = new Distribution();
					while (iter.hasNext()) {
						Entry<Integer, Double> nextStatePair = iter.next();
						int nextState = nextStatePair.getKey();
						double nextStateProb = nextStatePair.getValue();
						int indexInTeamNextState = map[nextState];
						if (indexInTeamNextState == StatesHelper.BADVALUE) {
							indexInTeamNextState = teamMDP.getNumStates();

							teamMDP.addState();
							if (teamMDPStatesList != null) {
								teamMDPStatesList
										.add(new State(robotNumList.get(r), agentMDP.getStatesList().get(nextState)));
							}

							map[nextState] = indexInTeamNextState;

						}
						distr.add(indexInTeamNextState, nextStateProb);

					}
					Object action = agentMDP.getAction(s, j);
					teamMDP.addActionLabelledChoice(indexInTeamState, distr,action );
					for (int rew = 0; rew < teamRewardsList.size(); rew++) {
						int daNum = rewardNumToCorrespondingDA.get(rew);
						MDPRewardsSimple rewardStruct = singleAgentNestedMDP.daList.get(daNum).costsModel;
						double rewardHere = rewardStruct
								.getTransitionReward(singleAgentNestedMDP.productStateToMDPState.get(s), j);
						int transitionNum = teamMDP.getNumChoices(indexInTeamState) - 1;
						teamRewardsList.get(rew).addToTransitionReward(indexInTeamState, transitionNum, rewardHere);
					}

				}
				for (int rew = 0; rew < teamRewardsList.size(); rew++) {
					int daNum = rewardNumToCorrespondingDA.get(rew);
					double rewardHere = singleAgentNestedMDP.daList.get(daNum).costsModel
							.getStateReward(singleAgentNestedMDP.productStateToMDPState.get(s));
					teamRewardsList.get(rew).addToStateReward(indexInTeamState, rewardHere);
				}

			}

			seqTeamMDP.essentialStates.add((BitSet) essentialStates.clone());
			seqTeamMDP.initialStates.add((BitSet) agentInitialStatesInTeam.clone());
			seqTeamMDP.agentMDPsToSeqTeamMDPStateMapping.add(map.clone());

		}
		teamMDP.setStatesList(teamMDPStatesList);
		seqTeamMDP.acceptingStates = acceptingStates;
		seqTeamMDP.statesToAvoid = statesToAvoid;
		seqTeamMDP.teamMDPTemplate = teamMDP;
		seqTeamMDP.teamRewardsTemplate = teamRewardsList;

		saveMDP(teamMDP, acceptingStates, "teamMDPTemplate", true);

		teamMDP.findDeadlocks(true); // TODO: do we do this here ? does it matter
		// just return this
		// add switch states after
		return seqTeamMDP;

	}

	/*
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

		BitSet statesToRemainIn =(BitSet) badStates.clone(); 
		statesToRemainIn.flip(0, sumprod.getNumStates());
		MDPModelChecker mc = new MDPModelChecker(this);
		mc.genStrat = true;
		ModelCheckerPartialSatResult res2 = mc.computeNestedValIterFailure(sumprod, accStatesF, statesToRemainIn, rewards);

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

		// process ltl expressions
		int numRobots = getNumRobots(exampleNumber());
		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = new ArrayList<SingleAgentNestedProductMDP>();
		ArrayList<Expression> ltlExpressions = getLTLExpressions(expr);
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions);
		saveMDP((MDP)model,null,"mdp",true);
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

		}
		

		// create team automaton from a set of MDP DA stuff

		SequentialTeamMDP seqTeamMDP = buildSequentialTeamMDPTemplate(singleAgentProductMDPs);
		int firstRobot = 0; // fix this
		StatesHelper.setMDPVar(seqTeamMDP.teamMDPTemplate.getVarList().getNumVars() - 1);
		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot);
		BitSet combinedEssentialStates = new BitSet();
		for (int i = 0; i < seqTeamMDP.essentialStates.size(); i++)
			combinedEssentialStates.or(seqTeamMDP.essentialStates.get(i));
		saveMDP(seqTeamMDP.teamMDPWithSwitches, combinedEssentialStates, "teamMDPWithSwitches", true);
		saveMDP(seqTeamMDP.teamMDPWithSwitches, seqTeamMDP.statesToAvoid, "teamMDPWithSwitchesAvoid", true);
		// seqTeamMDP.addSwitchTransitions(firstRobot);

		// solve
		ModelCheckerPartialSatResult res = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
				seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, seqTeamMDP.rewardsWithSwitches.get(0));
		// add to joint policy

		MMDPSimple jointPolicy = new MMDPSimple(seqTeamMDP.numRobots);
		// jointPolicy.stuckStatesQ.add(seqTeamMDP.teamMDPWithSwitches.getFirstInitialState());
		int initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		jointPolicy.unfoldPolicyForState(seqTeamMDP, res.strat, initialState, true);
		saveMDP(jointPolicy.mdp, null, "jointPolicy", true);
		Vector<StateProb> orderOfFailStates = new Vector<StateProb>();
		while (!jointPolicy.stuckStatesQ.isEmpty()) {
			 StateProb stuckState = jointPolicy.stuckStatesQ.remove();
			initialState = stuckState.getState();
			orderOfFailStates.add(stuckState.copy());
			mainLog.println("Exploring "+stuckState.toString());
			List<State> states = jointPolicy.mdp.getStatesList();// seqTeamMDP.teamMDPWithSwitches.getStatesList();
			State currState = states.get(initialState);
			int rNum = jointPolicy.firstFailedRobot(currState);
			int[] robotStates = jointPolicy.getRobotStatesIndexFromJointState(currState,
					seqTeamMDP.teamMDPWithSwitches.getStatesList());
			int robotStateId = robotStates[rNum];
			int nextRobot = (rNum + 1) % jointPolicy.nRobots;
			int nextRobotstate = robotStates[nextRobot];
			//
			seqTeamMDP.setInitialStates(robotStates);	
			//we need to change the switches to the initial states 
			seqTeamMDP.addSwitchesAndSetInitialState(rNum);
			saveMDP(seqTeamMDP.teamMDPWithSwitches,combinedEssentialStates,"teamMDPWithSwitches"+currState.toString(),true);
			res = computeNestedValIterFailurePrint(seqTeamMDP.teamMDPWithSwitches,
					seqTeamMDP.acceptingStates, seqTeamMDP.statesToAvoid, seqTeamMDP.rewardsWithSwitches.get(0));
			 jointPolicy.unfoldPolicyForState(seqTeamMDP, res.strat, initialState,false);
		}
	//	int setbit = jointPolicy.allTasksCompletedStates.nextSetBit(0);
//		int fsetbit = jointPolicy.allFailStatesSeen.nextSetBit(0);
//		 initState = 0;
//		while(initState!=-1) {
		for(int fs = 0; fs<orderOfFailStates.size(); fs++) {
			StateProb failState = orderOfFailStates.get(fs); 
			initState = failState.getState(); 
			double probTo = jointPolicy.getProbability(0, initState, 1.0);
			if(probTo == failState.getProb())
			{
				mainLog.println("probs equal");
			}
		 double prob = jointPolicy.getProbabilityAcceptingStateOnly(initState, 1.0);
		 prob = prob*probTo;
		 mainLog.println(failState.toString()+" Prob "+prob);
		// setbit = jointPolicy.allTasksCompletedStates.nextSetBit(setbit+1);
//		 initState = fsetbit; 
//		 fsetbit = jointPolicy.allFailStatesSeen.nextSetBit(fsetbit+1);
		}
	

		return null;
	}

	private int exampleNumber() {
		// SET the EXAMPLE NUMBER HERE
		// 0 = two room three robot not extended , 1 = two room three robot extended, 2 = debugging on
		// two_room_three_robot_blowup_reduced, 3 = three_robot_simple
// 4 = topo_map
		// 5= chain example 
		
		return 3;
	}
	private int getNumRobots(int exampleNumber)
	{
		int toret = -1;
		switch(exampleNumber) {
//		case 0:
//			toret =1; 
//			break; 
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

		}	else if (robotModel == 5)
		{
			if(robotNum == 0)
				initState = 1; 
			if(robotNum == 1)
				initState = 4; 
			if(robotNum == 2)
				initState = 7; 
			
		}
		///////////////////////////////////// DECIDE Robot init states
		///////////////////////////////////// HERE///////////////////////////////////////
		return initState;
	}

	/*
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

	/*
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
