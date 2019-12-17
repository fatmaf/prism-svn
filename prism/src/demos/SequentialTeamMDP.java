package demos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import explicit.Distribution;
import explicit.MDP;
import explicit.MDPSimple;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.Expression;
import prism.PrismException;
import prism.PrismLog;

public class SequentialTeamMDP
{
	/**
	 * 
	 */
	// private final STAPU stapu;
	public ArrayList<SingleAgentNestedProductMDP> agentMDPs;
	public ArrayList<int[]> agentMDPsToSeqTeamMDPStateMapping;
	public BitSet acceptingStates;
	public BitSet statesToAvoid;
	public Vector<BitSet> essentialStates;
	public Vector<BitSet> initialStates;
	public Vector<BitSet> failStates; // this is a hack at this point just to get things going really
	public MDPSimple teamMDPTemplate;
	public ArrayList<MDPRewardsSimple> teamRewardsTemplate;
	public MDPSimple teamMDPWithSwitches;
	public ArrayList<MDPRewardsSimple> rewardsWithSwitches;
	public MDPRewardsSimple progressionRewards;
	private Vector<Integer> sharedVarIndices;
	boolean switchesMatchSharedStatesToo;
	PrismLog mainLog;
	// basically its just the essential states that have progressionRewad

	public int numRobots;
	private ArrayList<String> sharedVarList;

	public SequentialTeamMDP(PrismLog mainLogRef, int nRobots, boolean useSharedStatesInSwitches)
	{
		// this.stapu = stapu;
		essentialStates = new Vector<BitSet>();
		initialStates = new Vector<BitSet>();
		numRobots = nRobots;
		mainLog = mainLogRef;
		switchesMatchSharedStatesToo = useSharedStatesInSwitches;
		this.sharedVarIndices = new Vector<Integer>();
	}

	public void addSwitchesAndSetInitialState(int firstRobot, boolean includefailstatesinswitches, boolean completeRing,boolean excludeRobotInitStates) throws PrismException
	{
		int[] robotStates = new int[numRobots];
		for (int i = 0; i < numRobots; i++) {
			// these are not the real states but just states that reflect the mdp state
			// this of course is not nice at all
			// it is a silly thing to do and should be fixed
			robotStates[i] = initialStates.get(i).nextSetBit(0);
		}
		addSwitchesAndSetInitialState(firstRobot, robotStates, includefailstatesinswitches, completeRing,excludeRobotInitStates);
	}

	public void addSwitchesAndSetInitialState(int firstRobot, int[] robotStates, boolean includefailstates, boolean completeRing,boolean excludeRobotInitStates) throws PrismException
	{
		// set initial state as those from the first robot
		// you still need to check if the robot has failed
		initializeTeamMDPFromTemplate();
		BitSet initialStatesFirstRobot = initialStates.get(firstRobot);
		// get the state from the coresponding model
		int state = agentMDPs.get(firstRobot).finalProduct.getProductModel().getFirstInitialState();
		state = agentMDPsToSeqTeamMDPStateMapping.get(firstRobot)[state];
		if (initialStatesFirstRobot.get(state))
			teamMDPWithSwitches.addInitialState(state);
		boolean[] isFailedState = new boolean[numRobots];
		for (int i = 0; i < numRobots; i++) {
			// check if there are any deadend states
			//replaced failstate check with deadend states 
			isFailedState[i] = StatesHelper.stateIsDeadend(teamMDPTemplate, robotStates[i]);
			//StatesHelper.isFailState(teamMDPTemplate.getStatesList().get(robotStates[i]));
			// also if you have an initial state that is an accepting state
			// you need to figure out what to do with it
			// because it could be the first robot - which is fine cuz thats the initial
			// state
			// but if its not you need to get rid of all states where that thing is not an
			// accepting state
			// what i mean to say is suppose v1 is a goal, and r2 is on v1 for the first
			// time ever
			// you need to know this stuff and account for it
			// technically check labels and stuff ??? I dont know I have to think about this
			// TODO: INITIAL STATE STUFF FOR LATER see above
		}

		addSwitchTransitions(firstRobot, isFailedState, includefailstates, completeRing,excludeRobotInitStates);
	}

	public int addSwitchTransitions(int firstRobot, boolean[] hasFailed, boolean includefailstates, boolean completeRing, boolean excludeRobotInitStates)
			throws PrismException
	{

		int totalSwitches = 0;
		boolean addSwitches = true; // just going to use this to make sure that the first robot doesnt get a switch
		for (int r = 0; r < numRobots; r++) {
			int fromRobot = r;
			int toRobot = (r + 1) % numRobots;
			// dont add switches to the first robot
			if (!completeRing)
				addSwitches = (toRobot != firstRobot);
			if (addSwitches) {

				BitSet fromRobotEssentialStates = (BitSet) essentialStates.get(fromRobot).clone();
				if (hasFailed[fromRobot])
					fromRobotEssentialStates = initialStates.get(fromRobot);
				// adding an else so we have switch states from intial states too
				else {
					if (!excludeRobotInitStates) {
						//					if(fromRobot != firstRobot)
						//					fromRobotEssentialStates.or(initialStates.get(fromRobot));}
						//					else
						fromRobotEssentialStates.or(initialStates.get(fromRobot));
					}
					if (includefailstates) {
						fromRobotEssentialStates.or(failStates.get(fromRobot));
					}
				}

				BitSet toRobotInitialStates = initialStates.get(toRobot);
				totalSwitches += addSwitchTransitionsBetweenRobots(toRobot, fromRobot, fromRobotEssentialStates, toRobotInitialStates);
			}
		}
		teamMDPWithSwitches.findDeadlocks(true);
		return totalSwitches;
	}

	private int addSwitchTransitionsBetweenRobots(int toRobot, int fromRobot, BitSet fromRobotEssentialStates, BitSet toRobotInitialStates)
	{
		// from the essential states of from robot
		// to the initial states of to robot
		int totalSwitches = 0;
		double switchProb = 1.0;

		int fromRobotState = fromRobotEssentialStates.nextSetBit(0);
		int toRobotState;// = toRobotInitialStates.nextSetBit(0);

		while (fromRobotState != -1) {
			State fromRobotStateVar = teamMDPTemplate.getStatesList().get(fromRobotState);
			toRobotState = toRobotInitialStates.nextSetBit(0);
			while (toRobotState != -1) {

				State toRobotStateVar = teamMDPTemplate.getStatesList().get(toRobotState);
				// add a link if the automaton progress is the same
				boolean stateVarsMatch = StatesHelper.statesHaveTheSameAutomataProgress(fromRobotStateVar, toRobotStateVar);
				if (this.switchesMatchSharedStatesToo) {
					stateVarsMatch = stateVarsMatch && StatesHelper.areEqual(fromRobotStateVar, toRobotStateVar, sharedVarIndices);
				}
				//				if(fromRobotState==6720)
				//					mainLog.println("check here");
				if (stateVarsMatch) {
					Distribution distr = new Distribution();
					distr.add(toRobotState, switchProb);
					teamMDPWithSwitches.addActionLabelledChoice(fromRobotState, distr, "switch_" + fromRobot + "-" + toRobot);
					// TODO check if we need a reward for a switch
					for (int i = 0; i < rewardsWithSwitches.size(); i++) {
						int numChoice = teamMDPWithSwitches.getNumChoices(fromRobotState) - 1;
						rewardsWithSwitches.get(i).addToTransitionReward(fromRobotState, numChoice, 0.0);

					}
					totalSwitches++;
					// making the assumption that since the to and from states are actually states
					// from the mdp
					// where we need too match progress
					// this can only happen once per state
					// hence this break
					// TODO: verify this
					break;
				}
				toRobotState = toRobotInitialStates.nextSetBit(toRobotState + 1);
			}
			fromRobotState = fromRobotEssentialStates.nextSetBit(fromRobotState + 1);
		}
		// mainLog.println(totalSwitches == fromRobotEssentialStates.cardinality());
		if (!(totalSwitches == fromRobotEssentialStates.cardinality()))
			mainLog.println("Number of switches doesnt match expected number");
		return totalSwitches;
	}

	public SequentialTeamMDP buildSequentialTeamMDPTemplate(ArrayList<SingleAgentNestedProductMDP> agentMDPs, ArrayList<String> sharedVarList)
			throws PrismException
	{

		// SequentialTeamMDP seqTeamMDP = new SequentialTeamMDP(this, agentMDPs.size());
		this.agentMDPs = agentMDPs;
		this.agentMDPsToSeqTeamMDPStateMapping = new ArrayList<int[]>(agentMDPs.size());
		this.failStates = new Vector<BitSet>();
		this.sharedVarList = sharedVarList;
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
			teamMDPVarList = new VarList();
			String daVar = "r";
			DeclarationInt daint;// = new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(numRobots - 1, 1)));
			Declaration decl;// = new Declaration(daVar, daint);

			int prodMDPNumVars = productMDP.getVarList().getNumVars();
			int daCount = 0;
			for (int i = 0; i < prodMDPNumVars; i++) {
				daVar = productMDP.getVarList().getName(i);
				decl = productMDP.getVarList().getDeclaration(i);
				if (daVar.contains("da")) {
					decl.setName("da" + daCount);
					daCount++;
				}

				teamMDPVarList.addVar(decl, productMDP.getVarList().getModule(i), productMDP.getConstantValues());
			}

			//String
			daVar = "r";// "_da"; // TODO: come back to fix this

			// NB: if DA only has one state, we add an extra dummy state
			daint = new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(numRobots - 1, 1)));
			decl = new Declaration(daVar, daint);
			teamMDPVarList.addVar(0, decl, 1, productMDP.getConstantValues());
			// //lets rename all the other ones
			// //so from 1 all the way to the end minus s
			// //we want to do stuff like dan, dan-1, dan-2

		}

		teamMDP = new MDPSimple();
		teamRewardsList = new ArrayList<MDPRewardsSimple>();
		this.progressionRewards = new MDPRewardsSimple(numTeamStates);

		teamMDP.setVarList(teamMDPVarList);

		ArrayList<State> teamMDPStatesList = null, robotNumList = null;
		// setting the shared var list
		for (int i = 0; i < sharedVarList.size(); i++) {
			int index = teamMDPVarList.getIndex(sharedVarList.get(i));
			if (index != -1) {
				this.sharedVarIndices.add(index);
			}
		}

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

		// ArrayList<int[]> maps = new ArrayList<int[]>();
		BitSet acceptingStates = new BitSet(numTeamStates); // for the team mdp they are acc for r1 || acc for r2 || acc
															// for r3 ...
		BitSet statesToAvoid = new BitSet(numTeamStates); // for the team mdp they are bad for r1 || bad for r2 || bad
															// for r3

		int numStates = 0;
		int numChoices;
		double expectedProgRewardValue = 1.0;
		double progRewardFixedValue = 1.0;

		StatesHelper.setMDPVar(teamMDPVarList.getNumVars() - StatesHelper.numMdpVars); // cuz there is door too

		for (int r = 0; r < agentMDPs.size(); r++) {

			SingleAgentNestedProductMDP singleAgentNestedMDP = agentMDPs.get(r);
			// BitSet allAcceptingStates = singleAgentNestedMDP.getAllAcceptingStates();
			BitSet essentialStates = new BitSet(numTeamStates);
			BitSet agentInitialStates = singleAgentNestedMDP.getInitialStates(this.switchesMatchSharedStatesToo, sharedVarList);
			BitSet agentInitialStatesInTeam = new BitSet(numTeamStates);
			BitSet agentFailStates = new BitSet();

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
				// mark if fail state
				if (StatesHelper.isFailState(teamMDPStatesList.get(indexInTeamState))) {
					agentFailStates.set(indexInTeamState);
				}
				// set the states
				if (singleAgentNestedMDP.combinedAcceptingStates.get(s)) {
					acceptingStates.set(indexInTeamState);
					// singleAgentNestedMDP.daList.get(0).productAcceptingStates.get(s);
					// this.progressionRewards.addToStateReward(indexInTeamState, 1.0);
				}
				if (singleAgentNestedMDP.combinedStatesToAvoid.get(s)) {
					statesToAvoid.set(indexInTeamState);
//					if(teamMDPStatesList.get(indexInTeamState).toString().contains("1,0,1,1,"))
//					{
//						
//							mainLog.println("Error "+indexInTeamState);
//						
//					}
//					if(teamMDPStatesList.get(indexInTeamState).toString().contains("0,0,1,1,"))
//					{
//						
//							mainLog.println("Error "+indexInTeamState);
//						
//					}
				}

				if (singleAgentNestedMDP.combinedEssentialStates.get(s)) {
					essentialStates.set(indexInTeamState);
					// this.progressionRewards.addToStateReward(indexInTeamState, 1.0);
				}
				if (agentInitialStates.get(s)) {
					agentInitialStatesInTeam.set(indexInTeamState);
				}

				numChoices = agentMDP.getNumChoices(s);

				Iterator<Map.Entry<Integer, Double>> iter;

				for (int j = 0; j < numChoices; j++) {
					boolean addProgReward = false;
					//				int ssCount = 0;
					iter = agentMDP.getTransitionsIterator(s, j);
					Distribution distr = new Distribution();
					while (iter.hasNext()) {
						//						ssCount++; 
						Entry<Integer, Double> nextStatePair = iter.next();
						int nextState = nextStatePair.getKey();
						double nextStateProb = nextStatePair.getValue();
						int indexInTeamNextState = map[nextState];
						if (indexInTeamNextState == StatesHelper.BADVALUE) {
							indexInTeamNextState = teamMDP.getNumStates();

							teamMDP.addState();
							if (teamMDPStatesList != null) {
								teamMDPStatesList.add(new State(robotNumList.get(r), agentMDP.getStatesList().get(nextState)));
							}

							map[nextState] = indexInTeamNextState;

						}

//						if(teamMDPStatesList.get(indexInTeamNextState).toString().contains("1,0,1,1,65"))
//						{
//							mainLog.println("debug");
//						}
						boolean[] essAcc = singleAgentNestedMDP.addRewardForTaskCompletion(nextState, s);
//						if(teamMDPStatesList.get(indexInTeamNextState).toString().contains("1,0,1,1,"))
//						{
//							if(essAcc[1]==false)
//							{
//								mainLog.println("Error "+indexInTeamNextState);
//							}
//						}
//						if(teamMDPStatesList.get(indexInTeamNextState).toString().contains("0,0,1,1,"))
//						{
//							if(essAcc[1]==false)
//							{
//								mainLog.println("Error "+indexInTeamNextState);
//							}
//						}
						if (essAcc[0]) {
							//somethings off with essential states stuff 
							//this is a better way to do stuff anyway so just putting it here 
							
							essentialStates.set(indexInTeamNextState);
							if(essAcc[1])
							{
								acceptingStates.nextSetBit(indexInTeamNextState);
							}
							//end update 
							
							if (addProgReward) {
								expectedProgRewardValue += nextStateProb * progRewardFixedValue;
							} else {
								addProgReward = true;
								expectedProgRewardValue = nextStateProb * progRewardFixedValue;
							}
						}

						distr.add(indexInTeamNextState, nextStateProb);
					}
					Object action = agentMDP.getAction(s, j);
					teamMDP.addActionLabelledChoice(indexInTeamState, distr, action);
					//					boolean isDeadend = StatesHelper.stateIsDeadend(agentMDP, s);

					int transitionNum = teamMDP.getNumChoices(indexInTeamState) - 1;
					for (int rew = 0; rew < teamRewardsList.size(); rew++) {
						int daNum = rewardNumToCorrespondingDA.get(rew);
						MDPRewardsSimple rewardStruct = singleAgentNestedMDP.daList.get(daNum).costsModel;

						int singleAgentState = singleAgentNestedMDP.productStateToMDPState.get(s);
						double rewardHere = rewardStruct.getTransitionReward(singleAgentState, j);

						//if its a deadend state set the reward to 0 

						//						if(isDeadend)
						//							rewardHere = 0.0;

						teamRewardsList.get(rew).addToTransitionReward(indexInTeamState, transitionNum, rewardHere);

					}
					if (addProgReward) {
						this.progressionRewards.addToTransitionReward(indexInTeamState, transitionNum, expectedProgRewardValue);
					}

				}
				for (int rew = 0; rew < teamRewardsList.size(); rew++) {
					int daNum = rewardNumToCorrespondingDA.get(rew);
					double rewardHere = singleAgentNestedMDP.daList.get(daNum).costsModel.getStateReward(singleAgentNestedMDP.productStateToMDPState.get(s));
					teamRewardsList.get(rew).addToStateReward(indexInTeamState, rewardHere);
				}

			}

			this.essentialStates.add((BitSet) essentialStates.clone());
			this.initialStates.add((BitSet) agentInitialStatesInTeam.clone());
			this.agentMDPsToSeqTeamMDPStateMapping.add(map.clone());
			this.failStates.add((BitSet) agentFailStates.clone());

		}
		teamMDP.setStatesList(teamMDPStatesList);
		this.acceptingStates = acceptingStates;
		this.statesToAvoid = statesToAvoid;
		this.teamMDPTemplate = teamMDP;
		this.teamRewardsTemplate = teamRewardsList;

//		StatesHelper.saveMDP(teamMDP, acceptingStates, "", "teamMDPTemplate", true);

		teamMDP.findDeadlocks(true); // TODO: do we do this here ? does it matter
		// just return this
		// add switch states after
		return this;

	}

	private BitSet convertAgentMDPStateToTeamState(BitSet agentStates, int r)
	{
		BitSet teamStates = new BitSet();

		int setBit = agentStates.nextSetBit(0);
		while (setBit != -1) {
			int teamBit = agentMDPsToSeqTeamMDPStateMapping.get(r)[setBit];
			// interesting caveat bith (in urdu with the thay)
			if (!statesToAvoid.get(teamBit)) {
				teamStates.set(teamBit);
				State s1 = teamMDPTemplate.getStatesList().get(teamBit);
				s1.toString();
			}
			setBit = agentStates.nextSetBit(setBit + 1);
		}
		return teamStates;

	}

	public int getNestedProductStateFromTeamState(int state, int rNum)
	{
		int nestedProductState = StatesHelper.BADVALUE;
		int[] mapping = agentMDPsToSeqTeamMDPStateMapping.get(rNum);
		for (int agentState = 0; agentState < mapping.length; agentState++) {
			if (mapping[agentState] == state) {
				nestedProductState = agentState;
				break;
			}
		}
		return nestedProductState;
	}

	public void initializeTeamMDPFromTemplate()
	{
		teamMDPWithSwitches = new MDPSimple(teamMDPTemplate);
		teamMDPWithSwitches.setStatesList(teamMDPTemplate.getStatesList());
		rewardsWithSwitches = new ArrayList<MDPRewardsSimple>(teamRewardsTemplate);

		// remember MDPs are not so easy to copy !!!
		// so we need to fix this

	}

	public void setInitialStates(int[] robotStates)
	{
		initialStates = new Vector<BitSet>();
		for (int r = 0; r < robotStates.length; r++) {
			SingleAgentNestedProductMDP mdp = agentMDPs.get(r);
			int nestedProductState = getNestedProductStateFromTeamState(robotStates[r], r);
			BitSet initialStatesForRobot = mdp.getAndSetInitialStates(nestedProductState, false, this.switchesMatchSharedStatesToo, this.sharedVarList);
			BitSet initialStatesForRobotInTeam = (BitSet) convertAgentMDPStateToTeamState(initialStatesForRobot, r).clone();
			// if (initialStates.size() > r)
			// initialStates.set(r, initialStatesForRobot);
			// else
			initialStates.add(initialStatesForRobotInTeam);
		}
	}

}