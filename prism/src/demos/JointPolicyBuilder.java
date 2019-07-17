package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import acceptance.AcceptanceReach;

import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import java.lang.Double;

import explicit.Distribution;
import explicit.MDPSimple;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import prism.PrismException;
import prism.PrismLangException;
import prism.PrismLog;
import strat.MDStrategyArray;
import strat.Strategy.Choice;

/*
 * An attempt to clean up code 
 * and make this more doable 
 * A new class for creating and storing the joint policy
 * so maybe this isnt the smartest name
 */
public class JointPolicyBuilder
{

	/*
	 * Storing state realted information parent state, robot number, probability etc
	 * just for ease of use
	 */
	public class StateExtended implements Comparable<StateExtended>
	{
		protected int parentState = -1;
		protected int parentStateRobot = -1;
		protected int childState = -1;
		protected int childStateRobot = -1;
		protected double parentToChildTransitionProbability = -1;
		protected String actionInChildState = null;
		protected int choice = -1;
		public BitSet statesToAvoid = null;

		public StateExtended()
		{
			// dummy
		}

		public StateExtended(int ps, int psr, int cs, int csr, double prob, String a)
		{
			parentState = ps;
			parentStateRobot = psr;
			childState = cs;
			childStateRobot = csr;
			parentToChildTransitionProbability = prob;
			actionInChildState = a;
		}

		public StateExtended(int s, double prob, String a)
		{
			childState = s;
			parentToChildTransitionProbability = prob;
			actionInChildState = a;
		}

		public StateExtended(StateExtended other)
		{
			this.parentState = other.parentState;
			this.parentStateRobot = other.parentStateRobot;
			this.childState = other.childState;
			this.childStateRobot = other.childStateRobot;
			this.parentToChildTransitionProbability = other.parentToChildTransitionProbability;
			this.actionInChildState = other.actionInChildState;
		}

		public StateExtended(int initialState, double d)
		{
			childState = initialState;
			parentToChildTransitionProbability = d;

		}

		@Override
		public int compareTo(StateExtended other)
		{
			double comp = this.parentToChildTransitionProbability - other.parentToChildTransitionProbability;
			int res = 0;
			if (comp > 0)
				res = -1;
			else {
				if (comp < 0) {
					res = 1;
				}
			}
			return res;
		}

		@Override
		public String toString()
		{
			String strtoret = "[";

			if (parentState != -1)
				strtoret += "ps=" + parentState;

			if (parentStateRobot != -1)
				strtoret += ", psRob=" + parentStateRobot;

			if (childState != -1)
				strtoret += ", cs=" + childState;

			if (childStateRobot != -1)
				strtoret += ", csRob=" + childStateRobot;

			if (parentToChildTransitionProbability > 0)
				strtoret += ", ps->csProb=" + parentToChildTransitionProbability;

			if (actionInChildState != null)
				strtoret += ", a=" + actionInChildState;
			strtoret += "]";

			return strtoret; //"[ps=" + parentState + ", psRob=" + parentStateRobot + ", cs="
			//+ childState + ", csRob=" + childStateRobot + ", ps->csProb="
			//+ parentToChildTransitionProbability + ", a=" + actionInChildState + "]";
		}

	}
	// elements
	// store the joint policy - mdp
	// things to help us remember stuff for the mdp
	// map - indicies and states
	// shared variable indices list

	int numRobots;
	int numTasks;
	int numSharedStates;
	ArrayList<String> sharedStatesNamesList;
	ArrayList<String> isolatedStatesNamesList;

	protected MDPSimple jointMDP;
	MDPRewardsSimple progressionRewards = null;
	ArrayList<MDPRewardsSimple> otherRewards = null;
	HashMap<State, Double> progressionRewardsHashMap = null;
	ArrayList<HashMap<State, Double>> otherRewardsHashMap = null;

	// helper bits
	PriorityQueue<StateExtended> failedStatesQueue = null;

	HashMap<String, Integer> varListMapping;
	ArrayList<Integer> daFinalStates = null;
	ArrayList<Integer> daInitialStates = null;
	ArrayList<String> daTags = new ArrayList<String>();
	HashMap<String, Integer> sharedVarsInitialStates;
	HashMap<State, Integer> statesMap = null;
	PrismLog mainLog;

	// sanity checking
	ArrayList<Entry<State, Double>> statesExploredOrder = null;
	double currentStateProbability = 1.0;

	BitSet accStates = new BitSet();

	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList, VarList seqTeamMDPVarList, PrismLog log)
	{
		statesMap = new HashMap<State, Integer>();
		ArrayList<String> isolatedStatesList = new ArrayList<String>();
		for (int i = 0; i < seqTeamMDPVarList.getNumVars(); i++) {
			String name = seqTeamMDPVarList.getName(i);
			if ((!sharedStatesList.contains(name)) && (!name.contains("da")) && (name != "r")) {
				isolatedStatesList.add(name);
			}
		}
		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, seqTeamMDPVarList, log);
		HashMap<State, Double> progressionRewardsHashMap = new HashMap<State, Double>();
		ArrayList<HashMap<State, Double>> otherRewardsHashMap = new ArrayList<HashMap<State, Double>>();
	}

	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList, ArrayList<String> isolatedStatesList, VarList seqTeamMDPVarList,
			PrismLog log)
	{
		statesMap = new HashMap<State, Integer>();
		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, seqTeamMDPVarList, log);
	}

	private VarList createVarList(VarList seqTeamMDPVarList)
	{
		VarList varlist = new VarList();
		varListMapping = new HashMap<String, Integer>();

		// end of the list - isolated robot states in order
		// we're making the assumption that all isolated states can be lumped into one
		String decname;
		try {

			for (int i = numRobots - 1; i >= 0; i--) {
				decname = "r" + i;
				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);

			}

			//keep the order of the shared states the same as the order in the seqteam mdp 
			//so just go over all the states in the varlist 

			for (int i = seqTeamMDPVarList.getNumVars() - 1; i >= 0; i--) {
				String name = seqTeamMDPVarList.getName(i);
				if (sharedStatesNamesList.contains(name)) {
					varlist.addVar(0, new Declaration(name, new DeclarationIntUnbounded()), 1, null);
				}
			}
			//			for (int i = 0; i < numSharedStates; i++) {
			//				decname = sharedStatesNamesList.get(i);
			//				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
			//			}
			//			for (int i = 0; i < numTasks; i++) {
			//				decname = "da" + i;
			//				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
			//			}
			for (int i = seqTeamMDPVarList.getNumVars() - 1; i >= 0; i--) {
				String name = seqTeamMDPVarList.getName(i);
				if (name.contains("da")) {
					varlist.addVar(0, new Declaration(name, new DeclarationIntUnbounded()), 1, null);

				}
			}

			for (int i = 0; i < varlist.getNumVars(); i++) {
				varListMapping.put(varlist.getName(i), i);
			}

		} catch (PrismLangException e) {
			e.printStackTrace();
		}

		return varlist;

	}

	private void initialize(int nrobots, int ntasks, ArrayList<String> sharedStatesList, ArrayList<String> isolatedStatesList, VarList seqTeamMDPVarList,
			PrismLog log)
	{
		numRobots = nrobots;
		numTasks = ntasks;
		numSharedStates = sharedStatesList.size();
		sharedStatesNamesList = sharedStatesList;
		isolatedStatesNamesList = isolatedStatesList;
		mainLog = log;

		jointMDP = new MDPSimple();
		jointMDP.setVarList(createVarList(seqTeamMDPVarList));
		jointMDP.setStatesList(new ArrayList<State>());
		this.failedStatesQueue = new PriorityQueue<StateExtended>();
		this.statesExploredOrder = new ArrayList<Entry<State, Double>>();

	}

	public BitSet getDAaccStatesForRobot(int da_num, int r, SequentialTeamMDP seqTeamMDP)
	{
		if (!seqTeamMDP.agentMDPs.get(r).daList.get(da_num).isSafeExpr)
			return ((AcceptanceReach) seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getAcceptance()).getGoalStates();
		else {
			// TODO : maybe there are multiple initial states or something and so we need to
			// do something like not the final state in our check
			BitSet temp = new BitSet();
			temp.set(seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getStartState());
			return temp;
		}
	}

	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, SequentialTeamMDP seqTeamMDP, int initialStateInSeqTeamMDP) throws PrismException
	{
		if (daInitialStates == null) {
			daInitialStates = new ArrayList<Integer>();
			daFinalStates = new ArrayList<Integer>();

			// match these to what we have in the varlist
			// TODO: hardcoding this
			// we know that its always the opposite - if this changes all hell breaks lose
			for (int i = seqTeamMDP.agentMDPs.get(0).daList.size() - 1; i >= 0; i--) {
				daInitialStates.add(seqTeamMDP.agentMDPs.get(0).daList.get(i).da.getStartState());
				if (!seqTeamMDP.agentMDPs.get(0).daList.get(i).isSafeExpr)
					daFinalStates.add(getDAaccStatesForRobot(i, 0, seqTeamMDP).nextSetBit(0));
				else
					daFinalStates.add(seqTeamMDP.agentMDPs.get(0).daList.get(i).da.getStartState());

			}

		}
		State currentState = seqTeamMDP.teamMDPWithSwitches.getStatesList().get(initialStateInSeqTeamMDP);
		int firstRobotNumber = StatesHelper.getRobotNumberFromSeqTeamMDPState(currentState);
		int[] currentRobotStates = new int[numRobots];
		currentRobotStates[firstRobotNumber] = initialStateInSeqTeamMDP;
		for (int i = 0; i < numRobots; i++) {
			if (i != firstRobotNumber) {
				currentRobotStates[i] = seqTeamMDP.initialStates.get(i).nextSetBit(0);
			}
		}
		// create a new joint state but use only the automata states of the first robot
		State currentJointState = createJointStateConsideringFirstRobotOnly(currentRobotStates, seqTeamMDP.teamMDPWithSwitches.getStatesList(),
				firstRobotNumber, seqTeamMDP.teamMDPWithSwitches.getVarList(), null);
		if (sharedVarsInitialStates == null)
			sharedVarsInitialStates = new HashMap<String, Integer>();
		for (int i = 0; i < this.sharedStatesNamesList.size(); i++) {
			int mapping = varListMapping.get(this.sharedStatesNamesList.get(i));
			if (seqTeamMDP.teamMDPWithSwitches.getVarList().exists(sharedStatesNamesList.get(i)))
				sharedVarsInitialStates.put(sharedStatesNamesList.get(i), (int) currentJointState.varValues[mapping]);
		}
		buildJointPolicyFromSequentialPolicy(strat, seqTeamMDP.teamMDPWithSwitches, currentJointState);

	}

	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, MDPSimple mdp, int initialJointState) throws PrismException
	{
		State currentJointState = this.getStateFromInd(initialJointState);
		if (sharedVarsInitialStates == null)
			sharedVarsInitialStates = new HashMap<String, Integer>();
		for (int i = 0; i < this.sharedStatesNamesList.size(); i++) {
			int mapping = varListMapping.get(this.sharedStatesNamesList.get(i));
			sharedVarsInitialStates.put(sharedStatesNamesList.get(i), (int) currentJointState.varValues[mapping]);
		}
		buildJointPolicyFromSequentialPolicy(strat, mdp, currentJointState);

	}

	private double getStateProb(int s, MDPSimple mdp, int depth, int maxDepth)
	{
		if (accStates.get(s))
			return 1.0;

		int numChoices = mdp.getNumChoices(s);
		if (numChoices == 0)
			return 0.0;
		if (depth >= maxDepth) //cuz we couldnt find the goal till then 
			return 0.0;
		double prob = 0;
		for (int i = 0; i < numChoices; i++) {
			Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, i);
			while (tranIter.hasNext()) {
				Entry<Integer, Double> stateProb = tranIter.next();
				int ss = stateProb.getKey();
				double p = stateProb.getValue();
				prob += p * getStateProb(ss, mdp, depth + 1, maxDepth);

			}
		}
		return prob;
	}

	private double getProbabilityToReachAccStateFromJointMDP(State js)
	{
		int s = statesMap.get(js);
		

		MDPSimple mdp = jointMDP;
		double prob = getStateProb(s, mdp, 0, mdp.getNumStates());

		return prob;
		//		int s = statesMap.get(js);
		//		double prob = 1.0;
		//		Stack<Integer> toVisit = new Stack<Integer>();
		//		Stack<Integer> visited = new Stack<Integer>();
		//		toVisit.push(s);
		//		MDPSimple mdp = jointMDP;
		//		while (!toVisit.isEmpty()) {
		//			//get the action for this state 
		//			//all the choices really 
		//			s = toVisit.pop();
		//
		//			if (!visited.contains(s)) {
		//				mainLog.println(s);
		//				visited.push(s);
		//				int numChoices = mdp.getNumChoices(s);
		//				for (int i = 0; i < numChoices; i++) {
		//					//get all the tran iters 
		//
		//					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, i);
		//					while (tranIter.hasNext()) {
		//						Entry<Integer, Double> stateProb = tranIter.next();
		//						int ss = stateProb.getKey();
		//						double p = stateProb.getValue();
		//						if (this.accStates.get(ss))
		//							mainLog.println("Accepting State found");
		//						toVisit.push(ss);
		//
		//					}
		//
		//				}
		//			}
		//		}
		//		return prob;
	}

	private void extractPolicyTreeAsDotFile(MDStrategyArray strat, MDPSimple mdp, int initialState)
	{

		MDPSimple policyTree = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		int[] stateLabels = new int[mdp.getNumStates()];
		Arrays.fill(stateLabels, -1);
		Queue<Integer> stateQ = new LinkedList<Integer>();
		stateQ.add(initialState);
		int state, ps, choice;
		Object action = null;
		BitSet discovered = new BitSet();
		while (!stateQ.isEmpty()) {
			state = stateQ.remove();
			discovered.set(state);
			if (stateLabels[state] == -1) {
				stateLabels[state] = policyTree.addState();
				statesList.add(mdp.getStatesList().get(state));
			}
			ps = stateLabels[state];
			action = strat.getChoiceAction(state);
			choice = strat.getChoiceIndex(state);
			if (choice > -1) {
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(state, choice);
				Distribution distr = new Distribution();
				while (tranIter.hasNext()) {
					Entry<Integer, Double> csp = tranIter.next();
					int childstate = csp.getKey();
					double stateProb = csp.getValue();
					if (stateLabels[childstate] == -1) {
						stateLabels[childstate] = policyTree.addState();
						statesList.add(mdp.getStatesList().get(childstate));
					}
					int cs = stateLabels[childstate];
					distr.add(cs, stateProb);
					if (!discovered.get(childstate))
						stateQ.add(childstate);
				}
				policyTree.addActionLabelledChoice(ps, distr, action);

			} else {
				//well we dont have a choice so we just skip this 

			}

		}
		policyTree.setStatesList(statesList);
		StatesHelper.saveMDP(policyTree, null, "", "policyTree" + initialState, true);
	}

	// the real thing
	// input: \pi_seq, s_J, mdp
	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, MDPSimple mdp, State initialJointState) throws PrismException
	{
		State currentJointState = initialJointState;
		if (!inStatesExplored(currentJointState)) {
			statesExploredOrder.add(new AbstractMap.SimpleEntry<State, Double>(currentJointState, this.currentStateProbability));
			Queue<Entry<State, Double>> jointStateQueue = new LinkedList<Entry<State, Double>>();

			jointStateQueue.add(new AbstractMap.SimpleEntry<State, Double>(currentJointState, 1.0));
			BitSet jointStatesDiscovered = new BitSet();
			boolean followingPath = true;// false;
			boolean doreset = false;//true;// false;
			boolean skipAutomataStatesInTA = true;
			ArrayList<ArrayList<StateExtended>> statesDiscovered = null;
			Entry<State, Double> currentJointStateProbPair = null;
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValuesBeforeProcessing = null;
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues = null;
			int[] modifiedRobotStatesInSeqTeamMDP = null;
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> stateValuesBeforeTaskAllocationBeforeProcessing = null;
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> stateValuesBeforeTaskAllocation = null;
			Entry<boolean[], Integer> numFailedPlusFlagsInitialState = numFailedInJointState(initialJointState);
			int numFailedInInitialState = numFailedPlusFlagsInitialState.getValue();
			boolean[] failedInInitialState = numFailedPlusFlagsInitialState.getKey();
			Queue<HashMap<Integer, ArrayList<Entry<Integer, Integer>>>> stateValuesBeforeTaskAllocationBeforeProcessingQ = new LinkedList<HashMap<Integer, ArrayList<Entry<Integer, Integer>>>>();
			Queue<HashMap<Integer, ArrayList<Entry<Integer, Integer>>>> mostProbableTaskAllocationStateValuesBeforeProcessingQ = new LinkedList<HashMap<Integer, ArrayList<Entry<Integer, Integer>>>>();
			Queue<ArrayList<ArrayList<StateExtended>>> statesDiscoveredQ = new LinkedList<ArrayList<ArrayList<StateExtended>>>();
			Queue<HashMap<Integer, ArrayList<Entry<Integer, Entry<Integer, Integer>>>>> sharedStateChangesQ = new LinkedList<HashMap<Integer, ArrayList<Entry<Integer, Entry<Integer, Integer>>>>>();
			HashMap<Integer, ArrayList<Entry<Integer, Entry<Integer, Integer>>>> sharedStateChanges;
			mostProbableTaskAllocationStateValuesBeforeProcessingQ.add(null);
			stateValuesBeforeTaskAllocationBeforeProcessingQ.add(null);
			sharedStateChangesQ.add(null);
			statesDiscoveredQ.add(null);

			if (jointStateQueue != null) {

				while (!jointStateQueue.isEmpty()) {
					mostProbableTaskAllocationStateValuesBeforeProcessing = mostProbableTaskAllocationStateValuesBeforeProcessingQ.remove();
					stateValuesBeforeTaskAllocationBeforeProcessing = stateValuesBeforeTaskAllocationBeforeProcessingQ.remove();
					currentJointStateProbPair = jointStateQueue.remove();
					statesDiscovered = statesDiscoveredQ.remove();
					sharedStateChanges = sharedStateChangesQ.remove();
					currentJointState = currentJointStateProbPair.getKey();
					boolean isAcc = this.isAcceptingState(currentJointState);

					int stateIndex = findStateIndex(currentJointState);
					boolean discovered = false;
					boolean stateReset = false;

					if (stateIndex != StatesHelper.BADVALUE) {
						if (jointStatesDiscovered.get(stateIndex)) {
							discovered = true;

						}
					}
					if (!discovered) {
						Entry<boolean[], Integer> numFailedPlusFlagsInState = numFailedInJointState(currentJointState);
						int numFailedInState = numFailedPlusFlagsInState.getValue();
						boolean[] failedInState = numFailedPlusFlagsInState.getKey();

						double probVar = currentJointStateProbPair.getValue();

						if (numFailedInState > numFailedInInitialState) {
							// the assumption is we have seen fail states before
							if (stateIndex != StatesHelper.BADVALUE) {
								probVar = probVar * this.currentStateProbability;
								if (numFailedInState != numRobots) {
									if (doreset) {
										// here we need to check if states are to be reset
										// basically for the robots that have failed, if they were allocated any tasks
										// where we are in an intermediate state
										// if this happens we need to add a "reset" action from the current state
										// to the new state we create purely to help us figure out what happened
										Entry<State, String> updatedJointStateString = resetTasksForFailedRobot(failedInInitialState, failedInState,
												currentJointState, mostProbableTaskAllocationStateValuesBeforeProcessing);
										State updatedJointState = updatedJointStateString.getKey();
										if (updatedJointState != null) {
											{
												addResetTransitionToJointPolicyMDP(currentJointState, updatedJointState, jointMDP,
														updatedJointStateString.getValue());
												stateReset = true;
											}
											stateIndex = findStateIndex(updatedJointState);

										}
									}
									if (!inStatesExplored(currentJointState)) {

										// if anyone other than the failed robot is assigned a seq task //i.e we did not
										// do a reset
										BitSet statesToAvoidDueToSeqTask = null;
										if (doreset) {
											if (!stateReset) {
												// think of a better function name atleast yaar
												// what is this
												statesToAvoidDueToSeqTask = getStatesToAvoidDueToSeqTask(currentJointState,
														mostProbableTaskAllocationStateValuesBeforeProcessing, mdp.getStatesList());
												if (statesToAvoidDueToSeqTask == null || statesToAvoidDueToSeqTask.isEmpty())
													mainLog.println("kabhi kabhi aisa hota hai janab");
												else
													mainLog.println("fazool zid karna");
											}
										}
										// get all states for all robots which don't have the exact seq task value
										StateExtended failState = new StateExtended(stateIndex, probVar);
										if (statesToAvoidDueToSeqTask != null && !statesToAvoidDueToSeqTask.isEmpty())
											failState.statesToAvoid = (BitSet) statesToAvoidDueToSeqTask.clone();
										this.failedStatesQueue.add(failState);

									}
									continue;
								}
							}

						}

						mainLog.println(currentJointState.toString());
						int[] robotStatesInSeqTeamMDP = extractIndividualRobotStatesFromJointState(currentJointState, mdp.getStatesList(), mdp.getVarList());

						// revamping all of this
						// we get the task allocation following a path to ensure that no tasks are
						// repeated
						// when we check the new states, we also do this following a path thing
						// when we get the action, we get the action assuming everyone else has done
						// what they needed to do
						// so when we create the succ joint state we also assume this.
						// this means restructuring all of the code below

						followingPath = true;
						if (statesDiscovered == null) {
							//							extractPolicyTreeAsDotFile(strat, mdp, robotStatesInSeqTeamMDP[0]);
							statesDiscovered = getTaskAllocationForAllRobots(strat, mdp, robotStatesInSeqTeamMDP,
									followingPath/* follow a path cuz you dont know anything */, false /*don't use all robot states*/);
							mostProbableTaskAllocationStateValuesBeforeProcessing = getStateIndexValuesForTaskAllocationForAllRobots(statesDiscovered,
									mdp.getStatesList(), mdp.getVarList(), followingPath);
							mostProbableTaskAllocationStateValues = this.processStateIndexValuesHolistic(mostProbableTaskAllocationStateValuesBeforeProcessing);

							modifiedRobotStatesInSeqTeamMDP = modifyRobotStatesToReflectExpectedTaskCompletionHolistic(robotStatesInSeqTeamMDP,
									mostProbableTaskAllocationStateValues, mdp.getStatesList());

							stateValuesBeforeTaskAllocationBeforeProcessing = getStateIndexValuesBeforeTaskAllocationForAllRobots(statesDiscovered,
									mdp.getStatesList(), mdp.getVarList(), followingPath);
							stateValuesBeforeTaskAllocation = this.processStateIndexValuesHolistic(stateValuesBeforeTaskAllocationBeforeProcessing);
						} else {
							//update current states to reflect seq task allocation
							//check if we need a new TA
							if (!skipAutomataStatesInTA) {
								modifiedRobotStatesInSeqTeamMDP = this.modifyRobotStatesToReflectExpectedTaskCompletionSeq(statesDiscovered,
										robotStatesInSeqTeamMDP, mostProbableTaskAllocationStateValuesBeforeProcessing,
										stateValuesBeforeTaskAllocationBeforeProcessing, sharedStateChanges, mdp.getStatesList(), 0);
							} else {
								modifiedRobotStatesInSeqTeamMDP = robotStatesInSeqTeamMDP.clone();
							}
							//checking for bad state 
							for (int bs = 0; bs < modifiedRobotStatesInSeqTeamMDP.length; bs++) {
								if (modifiedRobotStatesInSeqTeamMDP[bs] == StatesHelper.BADVALUE) {
									mainLog.println("Error");
								}
							}
							if (getNewTaskAllocation(statesDiscovered, modifiedRobotStatesInSeqTeamMDP, followingPath /* we have to follow the path */,
									skipAutomataStatesInTA, mdp.getStatesList(), mdp.getVarList())) {
								//								extractPolicyTreeAsDotFile(strat, mdp,  modifiedRobotStatesInSeqTeamMDP[0]);
								statesDiscovered = getTaskAllocationForAllRobots(strat, mdp, /*robotStatesInSeqTeamMDP*/modifiedRobotStatesInSeqTeamMDP,
										followingPath/* follow a path cuz you dont know anything */, true/*use all robot states*/);
								mostProbableTaskAllocationStateValuesBeforeProcessing = getStateIndexValuesForTaskAllocationForAllRobots(statesDiscovered,
										mdp.getStatesList(), mdp.getVarList(), followingPath);

							}

							mostProbableTaskAllocationStateValues = this.processStateIndexValuesHolistic(mostProbableTaskAllocationStateValuesBeforeProcessing);

							modifiedRobotStatesInSeqTeamMDP = modifyRobotStatesToReflectExpectedTaskCompletionHolistic(robotStatesInSeqTeamMDP,
									mostProbableTaskAllocationStateValues, mdp.getStatesList());

							stateValuesBeforeTaskAllocationBeforeProcessing = getStateIndexValuesBeforeTaskAllocationForAllRobots(statesDiscovered,
									mdp.getStatesList(), mdp.getVarList(), followingPath);
							stateValuesBeforeTaskAllocation = this.processStateIndexValuesHolistic(stateValuesBeforeTaskAllocationBeforeProcessing);

						}

						boolean usingModifiedStates = false;
						Entry<String, ArrayList<Entry<int[], Double>>> actionAndCombinations = getActionAndSuccStatesAllRobots(strat,
								modifiedRobotStatesInSeqTeamMDP, robotStatesInSeqTeamMDP, mdp, usingModifiedStates);

						String action = actionAndCombinations.getKey();
						ArrayList<State> succStatesQueue = new ArrayList<State>();
						ArrayList<Double> succStatesProbQueue = new ArrayList<Double>();
						for (Entry<int[], Double> combination : actionAndCombinations.getValue()) {

							int[] modifiedRobotSuccStatesInSeqTeamMDP = combination.getKey();
							int[] newSuccStatesForJointState = modifiedRobotSuccStatesInSeqTeamMDP;
							if (usingModifiedStates) {
								newSuccStatesForJointState = modifyRobotStatesToUndoExpectedTaskCompletionHolistic(modifiedRobotSuccStatesInSeqTeamMDP,
										stateValuesBeforeTaskAllocation, mdp.getStatesList());
							}
							sharedStateChanges = new HashMap<Integer, ArrayList<Entry<Integer, Entry<Integer, Integer>>>>();
							State succJointState = createJointState(newSuccStatesForJointState, mdp.getStatesList(), mdp.getVarList(), currentJointState,
									sharedStateChanges);

							succStatesQueue.add(succJointState);
							succStatesProbQueue.add(combination.getValue());
							jointStateQueue.add(new AbstractMap.SimpleEntry<State, Double>(succJointState, combination.getValue()));
							statesDiscoveredQ.add(statesDiscovered);
							mostProbableTaskAllocationStateValuesBeforeProcessingQ.add(mostProbableTaskAllocationStateValuesBeforeProcessing);
							stateValuesBeforeTaskAllocationBeforeProcessingQ.add(stateValuesBeforeTaskAllocationBeforeProcessing);
							sharedStateChangesQ.add(sharedStateChanges);
							// add to mdp
							//							if (succJointState.toString().contains("(0,0,0,1,-1,1,18)"))
							//								mainLog.println("Debug here");
						}
						mainLog.println(action.toString());
						this.addTranstionToMDP(jointMDP, currentJointState, succStatesQueue, succStatesProbQueue, action, 1.0);
						jointStatesDiscovered.set(statesMap.get(currentJointState));

						//						saveMDP(jointMDP, "new");
					}
					if (isAcc) {
						accStates.set(statesMap.get(currentJointState));
					}
				}
			}
			//			saveMDP(jointMDP, "new");
		}

	}

	private int[] modifyRobotStatesToReflectExpectedTaskCompletionSeq(ArrayList<ArrayList<StateExtended>> statesDiscovered, int[] robotStatesInSeqTeamMDP,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValuesBeforeProcessing,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> stateValuesBeforeTaskAllocationBeforeProcessing,
			HashMap<Integer, ArrayList<Entry<Integer, Entry<Integer, Integer>>>> sharedStateChanges, List<State> statesList, int firstRobot)
			throws PrismException
	{
		int[] newStates = (int[]) robotStatesInSeqTeamMDP.clone();
		// For each state
		// get the states values
		// modify them
		// move on
		int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(statesList.get(robotStatesInSeqTeamMDP[0]));

		HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues = this.processStateIndexValuesSeq(statesDiscovered,
				mostProbableTaskAllocationStateValuesBeforeProcessing, statesList);
		ArrayList<Entry<Integer, Integer>> indicesToChange = null;

		//for each robot, add everything from the previous robot
		//undo everything from future ones 
		//what do we undo it to ? 
		//we need the undo list. 
		for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
			robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(statesList.get(robotStatesInSeqTeamMDP[i]));
			Object[] currentState = (Object[]) statesList.get(robotStatesInSeqTeamMDP[i]).varValues.clone();

			//undo tasks by all future robots 
			int nextRobot = (robotNum + 1) % (numRobots);
			while (nextRobot != firstRobot) {
				if (stateValuesBeforeTaskAllocationBeforeProcessing.containsKey(nextRobot)) {
					indicesToChange = stateValuesBeforeTaskAllocationBeforeProcessing.get(nextRobot);
					for (int j = 0; j < indicesToChange.size(); j++) {

						currentState[indicesToChange.get(j).getKey()] = indicesToChange.get(j).getValue();
					}
				}
				//if the shared state is changed by a robot later we have to undo it here
				if (sharedStateChanges.containsKey(nextRobot)) {
					ArrayList<Entry<Integer, Entry<Integer, Integer>>> sharedStatesToChange = sharedStateChanges.get(nextRobot);
					for (int j = 0; j < sharedStatesToChange.size(); j++) {
						currentState[sharedStatesToChange.get(j).getKey() + 1] = sharedStatesToChange.get(j).getValue().getValue();
					}
				}
				nextRobot = (nextRobot + 1) % (numRobots);
			}

			if (robotNum != firstRobot) {

				int prevRobot = (robotNum - 1) % (this.numRobots); // ring thing

				// what do we do if there is no task allocation for this robot ?
				// I think its best to add an empty array thing to changed thing - this won't
				// materialize right now though
				// so need to do this for later

				if (mostProbableTaskAllocationStateValues.containsKey(prevRobot)) {
					indicesToChange = mostProbableTaskAllocationStateValues.get(prevRobot);
				}
				// otherwise just use the one from before because those things should be done
				// so now we change this
				if (indicesToChange != null) {
					for (int j = 0; j < indicesToChange.size(); j++) {

						currentState[indicesToChange.get(j).getKey()] = indicesToChange.get(j).getValue();
					}
				}

				newStates[robotNum] = StatesHelper.getExactlyTheSameState(currentState, statesList);
			} else {
				newStates[robotNum] = StatesHelper.getExactlyTheSameState(currentState, statesList);//robotStatesInSeqTeamMDP[i];
			}
			if (newStates[robotNum] == StatesHelper.BADVALUE)
				throw new PrismException("Bad state when getting new state " + currentState);
		}

		return newStates;
	}

	private int[] modifyRobotStatesToUndoExpectedTaskCompletionSeq(int[] robotStatesInSeqTeamMDP,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues, List<State> statesList, int firstRobot)
	{
		int[] newStates = (int[]) robotStatesInSeqTeamMDP.clone();
		// For each state
		// get the states values
		// modify them
		// move on
		ArrayList<Entry<Integer, Integer>> indicesToChange = null;
		for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
			int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(statesList.get(robotStatesInSeqTeamMDP[i]));
			if (robotNum != firstRobot) {
				int prevRobot = (robotNum - 1) % (this.numRobots); // ring thing

				Object[] currentState = (Object[]) statesList.get(robotStatesInSeqTeamMDP[i]).varValues.clone();
				if (mostProbableTaskAllocationStateValues.containsKey(prevRobot))
					indicesToChange = mostProbableTaskAllocationStateValues.get(prevRobot);
				if (indicesToChange != null) {
					// so now we change this
					for (int j = 0; j < indicesToChange.size(); j++) {

						currentState[indicesToChange.get(j).getKey()] = indicesToChange.get(j).getValue();
					}
				}
				newStates[robotNum] = StatesHelper.getExactlyTheSameState(currentState, statesList);
			} else {
				newStates[robotNum] = robotStatesInSeqTeamMDP[i];
			}

		}
		return newStates;
	}

	private void addResetTransitionToJointPolicyMDP(State currentJointState, State updatedJointState, MDPSimple mdp, String string)
	{
		int parentIndex = addStateToMDP(currentJointState, mdp);
		Object action = "reset" + string;
		int index;
		Distribution distr = new Distribution();

		index = addStateToMDP(updatedJointState, mdp);
		double normalizedProb = 1.0;
		distr.add(index, normalizedProb);

		mdp.addActionLabelledChoice(parentIndex, distr, action);

	}

	private BitSet getStatesToAvoidDueToSeqTask(State currentJointState,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValuesBeforeProcessing, List<State> teamMDPStateList)
	{
		BitSet toret = null;

		// so basically we need
		// the state thing of the seq task, if there is any

		// step 1
		// determine which robot has the seq task
		// determine the state of the seq task
		// we could do this in the resetTasksForFailedRobot function or use its thing
		HashMap<Integer, ArrayList<Entry<Integer, Integer>>> robotsWithSeqTasksAndTheirValues = new HashMap<Integer, ArrayList<Entry<Integer, Integer>>>();

		for (int i = 0; i < numRobots; i++) {
			if (mostProbableTaskAllocationStateValuesBeforeProcessing.containsKey(i)) {
				ArrayList<Entry<Integer, Integer>> seqTasksAndValues = new ArrayList<Entry<Integer, Integer>>();
				ArrayList<Entry<Integer, Integer>> tasks = mostProbableTaskAllocationStateValuesBeforeProcessing.get(i);
				for (Entry<Integer, Integer> e : tasks) {
					if (e.getKey() - 1 < numTasks) {
						int valueInJointState = (int) currentJointState.varValues[e.getKey() - 1];
						if (!(valueInJointState == daInitialStates.get(e.getKey() - 1) || valueInJointState == daFinalStates.get(e.getKey() - 1))) {
							// so we know this robot has a seq task
							// we need the task index , the robot number, the task value
							seqTasksAndValues.add(new AbstractMap.SimpleEntry<Integer, Integer>(e.getKey(), valueInJointState)); // e.getkey
																																	// is
																																	// the
																																	// value
																																	// in
																																	// the
																																	// team
																																	// mdp
																																	// e.getkey-1 is the value in the joint mdp thing

						}
					}
					if (seqTasksAndValues.size() > 0) {
						robotsWithSeqTasksAndTheirValues.put(i, seqTasksAndValues);

					}

				}
			}
		}

		// step 2
		// so now we know which robots have which seq task
		// for each robot that has a seq task, we get all the other states for all the
		// other robots that do not have the same value in the team mdp
		BitSet statesToDoStuffWith = new BitSet();

		for (int i = 0; i < numRobots; i++) {
			if (robotsWithSeqTasksAndTheirValues.containsKey(i)) {
				ArrayList<Entry<Integer, Integer>> seqTasksAndValues = robotsWithSeqTasksAndTheirValues.get(i);
				// now for all the other robots
				for (int k = 0; k < teamMDPStateList.size(); k++) {
					State mdpState = teamMDPStateList.get(k);

					// if this state belongs to any robot other than i
					// and it does not have the same values , add to list
					if (StatesHelper.getRobotNumberFromSeqTeamMDPState(mdpState) != i) {
						for (Entry<Integer, Integer> t : seqTasksAndValues) {
							if ((int) mdpState.varValues[t.getKey()] != t.getValue()) {
								statesToDoStuffWith.set(k);
							}
						}
					}
				}

			}

		}
		//		mainLog.println(statesToDoStuffWith.toString());

		return statesToDoStuffWith; //these are the states we want to avoid. 
	}

	private Entry<State, String> resetTasksForFailedRobot(boolean[] failedInInitialState, boolean[] failedInState,

			State currentJointState, HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValuesBeforeProcessing)
	{
		State updatedJointState = null;
		String resetRs = "";
		// if the robot has failed - one that did not fail in the initial state
		// then check if the tasks it was allocated are in their final/initial states
		// if not , we need to reset them to their initial states
		boolean hasFailed = false;
		for (int i = 0; i < numRobots; i++) {
			hasFailed = !failedInInitialState[i] && failedInState[i];
			if (hasFailed) {
				if (mostProbableTaskAllocationStateValuesBeforeProcessing.containsKey(i)) {
					ArrayList<Entry<Integer, Integer>> tasks = mostProbableTaskAllocationStateValuesBeforeProcessing.get(i);
					for (Entry<Integer, Integer> e : tasks) {
						// hardcoding this
						// but if the value is not a da value we don't care
						// the hard coding way to do this is to say if the index is greater than the
						// number of tasks
						// don't do it
						// the nicer cleaner way is to use varmapping
						if (e.getKey() - 1 < numTasks) {
							int valueInJointState = (int) currentJointState.varValues[e.getKey() - 1];
							if (!(valueInJointState == daInitialStates.get(e.getKey() - 1) || valueInJointState == daFinalStates.get(e.getKey() - 1))) {

								// then change it to its initial state
								if (updatedJointState == null)
									updatedJointState = new State(currentJointState);
								updatedJointState.setValue(e.getKey() - 1, daInitialStates.get(e.getKey() - 1));
								resetRs += i + " ";

							}
						}
					}
				}
			}
		}
		if (updatedJointState != null) {
			if (updatedJointState.compareTo(currentJointState) == 0)
				updatedJointState = null;
		}
		return new AbstractMap.SimpleEntry<State, String>(updatedJointState, resetRs);
	}

	public boolean inStatesExplored(State currentJointState)
	{
		boolean alreadyExplored = false;

		for (int i = 0; i < this.statesExploredOrder.size(); i++) {
			State aJointState = statesExploredOrder.get(i).getKey();
			if (aJointState.compareTo(currentJointState) == 0) {
				alreadyExplored = true;
				break;
			}
		}

		return alreadyExplored;
	}

	public void saveMDP(MDPSimple mdp, String name)
	{

		StatesHelper.saveMDP(mdp, null, "", name + "jointPolicy", true);
		StatesHelper.saveMDPstatra(mdp, "", name + "jointPolicy", true);

	}

	public void saveJointPolicyMDP()
	{
		saveMDP(this.jointMDP, "final");
	}

	public int findStateIndex(State s)
	{
		int indexInt = StatesHelper.BADVALUE;
		Object index = statesMap.get(s);
		if (index != null) {
			indexInt = (int) index;
		}
		return indexInt;
	}

	private int addStateToMDP(State s, MDPSimple mdp)
	{
		int index = findStateIndex(s);
		if (index == StatesHelper.BADVALUE) {
			mdp.getStatesList().add(s);
			index = mdp.getNumStates();
			statesMap.put(s, index);
			mdp.addState();

		}

		return index;
	}

	private void addTranstionToMDP(MDPSimple mdp, State parentStates, ArrayList<State> states, ArrayList<Double> probs, String action, double norm)
	{
		int parentIndex = addStateToMDP(parentStates, mdp);
		int index;
		Distribution distr = new Distribution();

		for (int succ = 0; succ < states.size(); succ++) {
			index = addStateToMDP(states.get(succ), mdp);
			double normalizedProb = probs.get(succ) / norm;
			distr.add(index, normalizedProb);

		}

		mdp.addActionLabelledChoice(parentIndex, distr, action);

	}

	private State getStateFromInd(int stateInd)
	{
		return this.jointMDP.getStatesList().get(stateInd);
	}

	private Entry<boolean[], Integer> numFailedInJointState(State jointState)
	{
		Entry<boolean[], Integer> numFailedPlusFlags;
		boolean[] failedFlags = new boolean[numRobots];
		Arrays.fill(failedFlags, false);
		int numFailed = 0;

		for (int i = 0; i < this.numRobots; i++) {
			if ((int) jointState.varValues[this.varListMapping.get("r" + i)] == StatesHelper.failState) {
				numFailed++;
				failedFlags[i] = true;
			}
		}
		numFailedPlusFlags = new AbstractMap.SimpleEntry<boolean[], Integer>(failedFlags, numFailed);
		return numFailedPlusFlags;
	}

	private boolean hasFailed(int initialStates, MDPSimple mdp)
	{
		State stateVal = (mdp.getStatesList().get(initialStates));
		for (int i = 0; i < this.isolatedStatesNamesList.size(); i++) {
			int stateIndex = mdp.getVarList().getIndex(this.isolatedStatesNamesList.get(i));
			if ((int) stateVal.varValues[stateIndex] == StatesHelper.failState)
				return true;
		}
		return false;
	}

	private boolean childStateInStateExtended(StateExtended state, int cs, boolean skipAutomataStatesInTA, List<State> statesList, VarList varlist)
	{
		if (!skipAutomataStatesInTA)
			return state.childState == cs;
		else {
			//match robot num 
			//match everything else 
			State stateState = statesList.get(state.childState);
			State csState = statesList.get(cs);
			int stateR = StatesHelper.getRobotNumberFromSeqTeamMDPState(stateState);
			int csR = StatesHelper.getRobotNumberFromSeqTeamMDPState(csState);
			if (stateR == csR) {
				//same robot 
				//now we match the mdp states 
				//which ones are the mdp states ? 
				//do we have this information ? I think we do 
				//this is the end 
				Object[] stateMDPStatesSS = StatesHelper.getSharedStatesFromState(stateState, varlist, this.sharedStatesNamesList);
				Object[] csMDPStatesSS = StatesHelper.getSharedStatesFromState(csState, varlist, sharedStatesNamesList);
				Object[] stateMDPStateIS = StatesHelper.getMDPStateFromState(stateState, varlist, isolatedStatesNamesList);
				Object[] csMDPStateIS = StatesHelper.getMDPStateFromState(csState, varlist, isolatedStatesNamesList);
				boolean sameSS = true;
				if (stateMDPStatesSS != null) {
					for (int i = 0; i < stateMDPStatesSS.length; i++) {
						if (stateMDPStatesSS[i] != csMDPStatesSS[i]) {
							sameSS = false;
							break;
						}
					}
				}
				if (sameSS) {
					for (int i = 0; i < stateMDPStateIS.length; i++) {

						if (stateMDPStateIS[i] != csMDPStateIS[i]) {
							sameSS = false;
							break;
						}
					}
				}
				return sameSS;
			}
			return false;
		}
	}

	private boolean childStateInStateExtendedArray(ArrayList<StateExtended> states, int cs, boolean skipAutomataStatesInTA, List<State> statesList,
			VarList varlist)
	{
		boolean toret = false;
		for (int i = 0; i < states.size(); i++) {
			if (childStateInStateExtended(states.get(i), cs, skipAutomataStatesInTA, statesList, varlist)) {
				toret = true;
				break;
			}
		}
		return toret;
	}

	// check if the states are in the states we found earlier
	// right now if any state is not in the ones we found earlier, we redo task
	// allocation
	// a smarter way would be to redo task allocation only for those states (hence
	// robots) that are not on the path
	// TODO: what I said above
	private boolean getNewTaskAllocation(ArrayList<ArrayList<StateExtended>> statesDiscovered, int[] robotStatesInSeqTeamMDP, boolean followingPath,
			boolean skipAutomataStatesInTA, List<State> statesList, VarList varlist)
	{
		// so basically check if the robotStatesInSeqTeamMDP are here
		if (statesDiscovered != null) {
			if (!followingPath) {
				boolean newTA = false;
				for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
					if (!childStateInStateExtendedArray(statesDiscovered.get(i), robotStatesInSeqTeamMDP[i], skipAutomataStatesInTA, statesList, varlist)) {
						newTA = true;
						break;
					}
				}
				return newTA;
			} else {
				boolean newTA = false;
				for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {

					if (!childStateInStateExtendedArray(statesDiscovered.get(0), robotStatesInSeqTeamMDP[i], skipAutomataStatesInTA, statesList, varlist)) {
						newTA = true;
						break;
					}
				}
				return newTA;
			}
		}
		// mainLog.println("getNewTaskAllocation() not implemented");
		return true;
	}

	private int[] modifyRobotStatesToReflectExpectedTaskCompletionHolistic(int[] robotStatesInSeqTeamMDP,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues, List<State> statesList) throws PrismException
	{
		int[] newStates = (int[]) robotStatesInSeqTeamMDP.clone();
		// For each state
		// get the states values
		// modify them
		// move on
		ArrayList<Entry<Integer, Integer>> indicesToChange = null;
		for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
			int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(statesList.get(robotStatesInSeqTeamMDP[i]));

			Object[] currentState = (Object[]) statesList.get(robotStatesInSeqTeamMDP[i]).varValues.clone();

			if (mostProbableTaskAllocationStateValues.containsKey(robotNum)) {
				indicesToChange = mostProbableTaskAllocationStateValues.get(robotNum);
			}
			// otherwise just use the one from before because those things should be done
			// so now we change this
			if (indicesToChange != null) {
				for (int j = 0; j < indicesToChange.size(); j++) {

					currentState[indicesToChange.get(j).getKey()] = indicesToChange.get(j).getValue();
				}
			}
			newStates[robotNum] = StatesHelper.getExactlyTheSameState(currentState, statesList);
			if (newStates[robotNum] == StatesHelper.BADVALUE)
				throw new PrismException("Bad state when getting new state " + currentState);
		}

		return newStates;
	}

	private int[] modifyRobotStatesToUndoExpectedTaskCompletionHolistic(int[] robotStatesInSeqTeamMDP,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues, List<State> statesList)
	{
		int[] newStates = (int[]) robotStatesInSeqTeamMDP.clone();
		// For each state
		// get the states values
		// modify them
		// move on
		ArrayList<Entry<Integer, Integer>> indicesToChange = null;
		for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
			int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(statesList.get(robotStatesInSeqTeamMDP[i]));

			Object[] currentState = (Object[]) statesList.get(robotStatesInSeqTeamMDP[i]).varValues.clone();
			if (mostProbableTaskAllocationStateValues.containsKey(robotNum))
				indicesToChange = mostProbableTaskAllocationStateValues.get(robotNum);
			if (indicesToChange != null) {
				// so now we change this
				for (int j = 0; j < indicesToChange.size(); j++) {

					currentState[indicesToChange.get(j).getKey()] = indicesToChange.get(j).getValue();
				}
			}
			newStates[robotNum] = StatesHelper.getExactlyTheSameState(currentState, statesList);
		}
		return newStates;
	}

	protected HashMap<Integer, ArrayList<Entry<Integer, Integer>>> processStateIndexValuesSeq(ArrayList<ArrayList<StateExtended>> statesDiscovered,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> changedStatesForAllRobots, List<State> statesList)
	{
		StateExtended firstState = statesDiscovered.get(0).get(0);
		State firstStateState = statesList.get(firstState.childState);
		int firstRobot = StatesHelper.getRobotNumberFromSeqTeamMDPState(firstStateState);
		// for each robot the indices that need to be changed are from all of the
		// previous robots
		int r = (firstRobot + 1) % numRobots;
		HashMap<Integer, ArrayList<Entry<Integer, Integer>>> concatenatedChangedStatesForAllRobots = new HashMap<Integer, ArrayList<Entry<Integer, Integer>>>();
		if (changedStatesForAllRobots.containsKey(firstRobot))
			concatenatedChangedStatesForAllRobots.put(firstRobot, changedStatesForAllRobots.get(firstRobot));
		while (r != firstRobot) {
			int prevRobot = (r - 1) % numRobots;
			ArrayList<Entry<Integer, Integer>> updatedChangedStates = new ArrayList<Entry<Integer, Integer>>();
			if (concatenatedChangedStatesForAllRobots.containsKey(prevRobot))
				updatedChangedStates.addAll(concatenatedChangedStatesForAllRobots.get(prevRobot));
			if (changedStatesForAllRobots.containsKey(r))
				updatedChangedStates.addAll(changedStatesForAllRobots.get(r));
			// else
			// {
			// mainLog.println("error here");
			// }

			concatenatedChangedStatesForAllRobots.put(r, updatedChangedStates);
			r = (r + 1) % numRobots;
		}
		return concatenatedChangedStatesForAllRobots;
	}

	protected HashMap<Integer, ArrayList<Entry<Integer, Integer>>> processStateIndexValuesHolistic(
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> changedStatesForAllRobots)
	{

		HashMap<Integer, ArrayList<Entry<Integer, Integer>>> concatenatedChangedStatesForAllRobots = new HashMap<Integer, ArrayList<Entry<Integer, Integer>>>();

		// for each robot the indices that need to be changed are from all the other
		// robots
		for (int robotToConsider = 0; robotToConsider < numRobots; robotToConsider++) {
			ArrayList<Entry<Integer, Integer>> updatedChangedStates = new ArrayList<Entry<Integer, Integer>>();
			for (int allOtherRobots = 0; allOtherRobots < numRobots; allOtherRobots++) {
				if (allOtherRobots != robotToConsider) {
					if (changedStatesForAllRobots.containsKey(allOtherRobots)) {
						updatedChangedStates.addAll(changedStatesForAllRobots.get(allOtherRobots));
					}
				}
			}
			concatenatedChangedStatesForAllRobots.put(robotToConsider, updatedChangedStates);
		}

		return concatenatedChangedStatesForAllRobots;
	}

	protected HashMap<Integer, ArrayList<Entry<Integer, Integer>>> getStateIndexValuesForTaskAllocationForAllRobots(
			ArrayList<ArrayList<StateExtended>> statesDiscovered, List<State> statesList, VarList varList, boolean followingPath)
	{

		StateExtended firstState = statesDiscovered.get(0).get(0);
		State firstStateState = statesList.get(firstState.childState);

		HashMap<Integer, ArrayList<Entry<Integer, Integer>>> changedStatesForAllRobots = new HashMap<Integer, ArrayList<Entry<Integer, Integer>>>();
		if (!followingPath) {
			for (int i = 0; i < statesDiscovered.size(); i++) {
				if (statesDiscovered.get(i).size() > 0) {
					// for each robot
					// get the last state
					// these are the values for the next robot
					StateExtended lastState = statesDiscovered.get(i).get(statesDiscovered.get(i).size() - 1);
					firstState = statesDiscovered.get(i).get(0);
					State lastStateState = statesList.get(lastState.childState);
					firstStateState = statesList.get(firstState.childState);
					int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(firstStateState);
					int[] changedStates = StatesHelper.XORIntegers(firstStateState, lastStateState);
					ArrayList<Entry<Integer, Integer>> changedStateIndices = new ArrayList<Entry<Integer, Integer>>();
					for (int j = 0; j < changedStates.length; j++) {
						if (changedStates[j] == 1 && !this.sharedStatesNamesList.contains(varList.getName(j))
								&& !this.isolatedStatesNamesList.contains(varList.getName(j))) {
							changedStateIndices.add(new AbstractMap.SimpleEntry<Integer, Integer>(j, (int) lastStateState.varValues[j]));
						}
					}
					changedStatesForAllRobots.put(robotNum, changedStateIndices);
				}

			}

		} else {

			for (int i = 0; i < statesDiscovered.get(1).size(); i++) {
				StateExtended nextState = statesDiscovered.get(1).get(i);
				State lastStateState = statesList.get(nextState.childState);

				int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(lastStateState);
				int[] changedStates = StatesHelper.XORIntegers(firstStateState, lastStateState);
				ArrayList<Entry<Integer, Integer>> changedStateIndices = new ArrayList<Entry<Integer, Integer>>();
				for (int j = 0; j < changedStates.length; j++) {
					if ((varList.getName(j) != "r") && changedStates[j] == 1 && !this.sharedStatesNamesList.contains(varList.getName(j))
							&& !this.isolatedStatesNamesList.contains(varList.getName(j))) {
						changedStateIndices.add(new AbstractMap.SimpleEntry<Integer, Integer>(j, (int) lastStateState.varValues[j]));
					}
				}
				changedStatesForAllRobots.put(robotNum, changedStateIndices);
				firstState = nextState;
				firstStateState = lastStateState;
			}
		}

		return changedStatesForAllRobots;
	}

	protected HashMap<Integer, ArrayList<Entry<Integer, Integer>>> getStateIndexValuesBeforeTaskAllocationForAllRobots(
			ArrayList<ArrayList<StateExtended>> statesDiscovered, List<State> statesList, VarList varList, boolean followingPath)
	{
		StateExtended firstState = statesDiscovered.get(0).get(0);
		State firstStateState = statesList.get(firstState.childState);

		HashMap<Integer, ArrayList<Entry<Integer, Integer>>> changedStatesForAllRobots = new HashMap<Integer, ArrayList<Entry<Integer, Integer>>>();
		if (!followingPath) {
			for (int i = 0; i < statesDiscovered.size(); i++) {
				if (statesDiscovered.get(i).size() > 0) {
					// for each robot
					// get the last state
					// these are the values for the next robot
					StateExtended lastState = statesDiscovered.get(i).get(statesDiscovered.get(i).size() - 1);
					firstState = statesDiscovered.get(i).get(0);
					State lastStateState = statesList.get(lastState.childState);
					firstStateState = statesList.get(firstState.childState);
					int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(firstStateState);
					int[] changedStates = StatesHelper.XORIntegers(firstStateState, lastStateState);
					ArrayList<Entry<Integer, Integer>> changedStateIndices = new ArrayList<Entry<Integer, Integer>>();
					for (int j = 0; j < changedStates.length; j++) {
						if (changedStates[j] == 1 && !this.sharedStatesNamesList.contains(varList.getName(j))
								&& !this.isolatedStatesNamesList.contains(varList.getName(j))) {
							changedStateIndices.add(new AbstractMap.SimpleEntry<Integer, Integer>(j, (int) firstStateState.varValues[j]));
						}
					}
					changedStatesForAllRobots.put(robotNum, changedStateIndices);

				}

			}
		} else {
			firstState = statesDiscovered.get(0).get(0);
			firstStateState = statesList.get(firstState.childState);
			for (int i = 0; i < statesDiscovered.get(1).size(); i++) {
				StateExtended nextState = statesDiscovered.get(1).get(i);
				State lastStateState = statesList.get(nextState.childState);

				int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(lastStateState);
				int[] changedStates = StatesHelper.XORIntegers(firstStateState, lastStateState);
				ArrayList<Entry<Integer, Integer>> changedStateIndices = new ArrayList<Entry<Integer, Integer>>();
				for (int j = 0; j < changedStates.length; j++) {
					if ((varList.getName(j) != "r") && changedStates[j] == 1 && !this.sharedStatesNamesList.contains(varList.getName(j))
							&& !this.isolatedStatesNamesList.contains(varList.getName(j))) {
						changedStateIndices.add(new AbstractMap.SimpleEntry<Integer, Integer>(j, (int) firstStateState.varValues[j]));
					}
				}
				changedStatesForAllRobots.put(robotNum, changedStateIndices);
				firstState = nextState;
				firstStateState = lastStateState;
			}

		}

		return changedStatesForAllRobots;

	}

	protected ArrayList<StateExtended> getTaskAllocationForRobot(MDStrategyArray strat, MDPSimple mdp, int initialState)
	{
		// we do a BEST cost search
		// we need to keep all the states we've seen
		ArrayList<StateExtended> statesDiscovered = new ArrayList<StateExtended>();
		PriorityQueue<StateExtended> statesToExploreQ = new PriorityQueue<StateExtended>();
		StateExtended currentState = new StateExtended(initialState, 1.0);
		statesToExploreQ.add(currentState);
		while (!statesToExploreQ.isEmpty()) {
			currentState = statesToExploreQ.remove();
			if (!statesDiscovered.contains(currentState)) {
				statesDiscovered.add(currentState);
				Entry<Object, Integer> actionChoice = getActionChoice(strat, currentState.childState);
				if (actionChoice.getKey() != null) {
					currentState.actionInChildState = actionChoice.getKey().toString();
					if (currentState.actionInChildState != "*" && !currentState.actionInChildState.contains("switch")) {
						Iterator<Entry<Integer, Double>> tranIter = getTranIter(actionChoice.getValue(), currentState.childState, mdp, true, null);
						ArrayList<Entry<Integer, Double>> succStates = tranIterToArrayList(tranIter);

						for (int i = 0; i < succStates.size(); i++) {
							StateExtended succState = new StateExtended(succStates.get(i).getKey(), succStates.get(i).getValue());
							succState.parentState = currentState.childState;

							statesToExploreQ.add(succState);
						}
					} else {
						// if (currentState.action.contains("switch"))
						//						mainLog.println(statesDiscovered.toString());
						break;
					}
				}
			}

		}
		return statesDiscovered;

	}

	int getFirstFailedRobotFromRobotStates(int[] initialStates, MDPSimple mdp)
	{
		int firstRobot = 0;
		// the first robot that hasnt failed
		for (int i = 0; i < this.numRobots; i++) {
			if (!hasFailed(initialStates[i], mdp)) {
				firstRobot = i;
				break;
			}
		}
		return firstRobot;
	}

	protected ArrayList<ArrayList<StateExtended>> getTaskAllocationForAllRobots(MDStrategyArray strat, MDPSimple mdp, int[] initialStates,
			boolean followingPath, boolean useAllRobotStates)
	{
		ArrayList<ArrayList<StateExtended>> statesDiscovered = null;
		if (!followingPath) {
			statesDiscovered = new ArrayList<ArrayList<StateExtended>>();
			for (int i = 0; i < this.numRobots; i++)
				statesDiscovered.add(getTaskAllocationForRobot(strat, mdp, initialStates[i]));
		} else {

			int firstRobot = 0;
			// the first robot that hasnt failed
			for (int i = 0; i < this.numRobots; i++) {
				if (!hasFailed(initialStates[i], mdp)) {
					firstRobot = i;
					break;
				}
			}
			statesDiscovered = getTaskAllocationForAllRobotsFollowingPath(strat, mdp, initialStates, firstRobot, useAllRobotStates);
		}
		return statesDiscovered;

	}

	protected ArrayList<ArrayList<StateExtended>> getTaskAllocationForAllRobotsFollowingPath(MDStrategyArray strat, MDPSimple mdp, int[] initialstates,
			int firstRobot, boolean useAllStates)
	{

		// we do a BEST cost search
		// we need to keep all the states we've seen
		int initialstate = initialstates[firstRobot];
		ArrayList<ArrayList<StateExtended>> statesDiscoveredPlusSwitches = new ArrayList<ArrayList<StateExtended>>();
		ArrayList<StateExtended> endStates = new ArrayList<StateExtended>();
		ArrayList<StateExtended> statesDiscovered = new ArrayList<StateExtended>();
		PriorityQueue<StateExtended> statesToExploreQ = new PriorityQueue<StateExtended>();
		StateExtended currentState = new StateExtended(initialstate, 1.0);
		statesToExploreQ.add(currentState);

		while (!statesToExploreQ.isEmpty()) {
			int succStateToUseID = -1;
			currentState = statesToExploreQ.remove();
			if (!statesDiscovered.contains(currentState)) {
				statesDiscovered.add(currentState);
				Entry<Object, Integer> actionChoice = getActionChoice(strat, currentState.childState);
				if (actionChoice.getKey() != null) {
					currentState.actionInChildState = actionChoice.getKey().toString();
					if (currentState.actionInChildState != "*") {
						if (currentState.actionInChildState.contains("switch"))
							endStates.add(currentState);
						Iterator<Entry<Integer, Double>> tranIter = getTranIter(actionChoice.getValue(), currentState.childState, mdp, true, null);
						ArrayList<Entry<Integer, Double>> succStates = tranIterToArrayList(tranIter);

						//if use all states 
						//then we check if any of the successors is a state on our current path 
						//if so we explore that successor and not the others 
						if (useAllStates) {
							//for all the successors 
							for (int i = 0; i < succStates.size(); i++) {
								for (int r = 0; r < numRobots; r++) {
									if (initialstates[r] == succStates.get(i).getKey()) {
										succStateToUseID = i;
										break;
									}

								}
								if (succStateToUseID != -1)
									break;
							}
						}
						if (useAllStates && succStateToUseID != -1) {
							StateExtended succState = new StateExtended(succStates.get(succStateToUseID).getKey(), succStates.get(succStateToUseID).getValue());
							succState.parentState = currentState.childState;

							statesToExploreQ.add(succState);
						} else {
							for (int i = 0; i < succStates.size(); i++) {
								StateExtended succState = new StateExtended(succStates.get(i).getKey(), succStates.get(i).getValue());
								succState.parentState = currentState.childState;

								statesToExploreQ.add(succState);
							}
						}
					} else {
						// if (currentState.action.contains("switch"))
						endStates.add(currentState);
						//						mainLog.println(statesDiscovered.toString());
						break;
					}
				}
			}

		}
		statesDiscoveredPlusSwitches.add(statesDiscovered);
		statesDiscoveredPlusSwitches.add(endStates);
		return statesDiscoveredPlusSwitches;// statesDiscovered;

	}

	protected Entry<Object, Integer> getActionChoice(MDStrategyArray strat, int state)
	{
		Entry<Object, Integer> actionChoice;
		Object action = null;
		int choice = -1;

		action = strat.getChoiceAction(state);
		choice = strat.getChoiceIndex(state);

		actionChoice = new AbstractMap.SimpleEntry<Object, Integer>(action, choice);
		return actionChoice;
	}

	protected HashMap<Integer, Entry<Object, Integer>> getActionChoiceAllRobots(MDStrategyArray strat, int[] states)
	{
		HashMap<Integer, Entry<Object, Integer>> actionChoices = new HashMap<Integer, Entry<Object, Integer>>();
		for (int i = 0; i < states.length; i++) {

			Entry<Object, Integer> actionChoice = getActionChoice(strat, states[i]);

			actionChoices.put(states[i], actionChoice);
		}
		return actionChoices;
	}

	protected double getProgressionReward(MDStrategyArray strat, int[] states, SequentialTeamMDP teamMDP)
	{
		double progRew = 0;
		//rewards are sums 

		for (int i = 0; i < states.length; i++) {

			Entry<Object, Integer> actionChoice = getActionChoice(strat, states[i]);
			progRew += teamMDP.progressionRewards.getTransitionReward(states[i], actionChoice.getValue());

		}
		return progRew;
	}

	protected ArrayList<Double> getOtherRewards(MDStrategyArray strat, int[] states, SequentialTeamMDP teamMDP)
	{
		ArrayList<Double> allRews = new ArrayList<Double>();
		for (int rew = 0; rew < teamMDP.rewardsWithSwitches.size(); rew++)
			allRews.add(0.0);
		for (int i = 0; i < states.length; i++) {

			Entry<Object, Integer> actionChoice = getActionChoice(strat, states[i]);
			for (int rew = 0; rew < teamMDP.rewardsWithSwitches.size(); rew++) {
				double currentRew = allRews.get(rew);
				double rewForState = teamMDP.rewardsWithSwitches.get(rew).getTransitionReward(states[i], actionChoice.getValue());
				currentRew += rewForState;
				allRews.set(rew, currentRew);
			}

		}
		return allRews;
	}

	protected Entry<String, ArrayList<Entry<int[], Double>>> getActionAndSuccStatesAllRobots(MDStrategyArray strat, int[] modifiedStates, int[] states,
			MDPSimple mdp, boolean usingModifiedState) throws PrismException
	{
		Entry<String, ArrayList<Entry<int[], Double>>> toret = null;
		HashMap<Integer, Entry<Object, Integer>> actionChoices = getActionChoiceAllRobots(strat, modifiedStates);
		HashMap<Integer, ArrayList<Entry<Integer, Double>>> succArrs = new HashMap<Integer, ArrayList<Entry<Integer, Double>>>();
		// for each action choice we need to get the successorstates
		String jointAction = "";
		int[] numSuccs = new int[numRobots];

		for (int i = 0; i < states.length; i++) {
			boolean forceNull = false;
			String currentAction = "";
			//			this.extractPolicyTreeAsDotFile(strat, mdp, modifiedStates[i]);
			if (actionChoices.get(modifiedStates[i]).getKey() != null)
				currentAction = actionChoices.get(modifiedStates[i]).getKey().toString();
			if (currentAction.contains("switch"))
				forceNull = true;
			int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(mdp.getStatesList().get(states[i]));
			Iterator<Entry<Integer, Double>> tranIter = getTranIter(actionChoices.get(modifiedStates[i]).getValue(), states[i], mdp, usingModifiedState,
					currentAction);
			ArrayList<Entry<Integer, Double>> tranIterList = tranIterToArrayList(tranIter);
			if (forceNull)
				tranIterList = null;
			if (tranIterList == null) {
				// then the list is just that state there aren't any more actions
				tranIterList = new ArrayList<Entry<Integer, Double>>();
				tranIterList.add(new AbstractMap.SimpleEntry<Integer, Double>(states[i], 1.0));
			}
			succArrs.put(rnum, tranIterList);
			numSuccs[i] = tranIterList.size();
			jointAction = jointAction + "r" + rnum + "_" + currentAction;
			if (i != modifiedStates.length - 1)
				jointAction = jointAction + "_";
		}
		// for each robot and each succArrs we have to make combinations
		// ooo that old code
		// i want tea or water or something
		// its okay - everything is okay - damn it
		ArrayList<int[]> combinations = new ArrayList<int[]>();
		generateCombinations(numSuccs.clone(), numSuccs.clone(), combinations);
		ArrayList<Entry<int[], Double>> succStateProbList = new ArrayList<Entry<int[], Double>>();
		double sumSuccProb = 0.0;
		double combProb = 1.0;
		for (int[] combination : combinations) {
			int[] currentSuccStates = new int[numRobots];
			combProb = 1.0;
			for (int i = 0; i < combination.length; i++) {
				currentSuccStates[i] = succArrs.get(i).get(combination[i] - 1).getKey();
				combProb = combProb * succArrs.get(i).get(combination[i] - 1).getValue();
			}
			sumSuccProb += combProb;
			// create new joint state
			// State succJointState=
			// createJointState(currentSuccStates,mdp.getStatesList(),false,0,null);
			succStateProbList.add(new AbstractMap.SimpleEntry<int[], Double>(currentSuccStates.clone(), combProb));
		}
		// normalize here
		for (int i = 0; i < succStateProbList.size(); i++) {
			combProb = succStateProbList.get(i).getValue();
			succStateProbList.get(i).setValue(combProb / sumSuccProb);
		}
		toret = new AbstractMap.SimpleEntry<String, ArrayList<Entry<int[], Double>>>(jointAction, succStateProbList);
		return toret;

	}

	private void generateCombinations(int counter[], int original[], ArrayList<int[]> res) throws PrismException
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

	protected Iterator<Entry<Integer, Double>> getTranIter(int choice, int state, MDPSimple mdp, boolean usingModifiedState, String currentAction)
	{
		if (usingModifiedState) {
			if (choice > -1)
				return mdp.getTransitionsIterator(state, choice);
			else
				return null;
		} else {
			if (currentAction != null) {
				int numChoices = mdp.getNumChoices(state);
				for (int i = 0; i < numChoices; i++) {
					Object action = mdp.getAction(state, i);
					if (action != null) {
						if (action.toString().equals(currentAction)) {
							return mdp.getTransitionsIterator(state, i);
						}
					}
				}
			}
			return null;
		}

	}

	protected ArrayList<Entry<Integer, Double>> tranIterToArrayList(Iterator<Entry<Integer, Double>> tranIter)
	{
		if (tranIter != null) {
			ArrayList<Entry<Integer, Double>> succStatesList = new ArrayList<Entry<Integer, Double>>();

			while (tranIter.hasNext()) {

				succStatesList.add(tranIter.next());
			}
			return succStatesList;
		} else
			return null;

	}

	protected State createJointState(int[] states, List<State> statesList, VarList varlist, State parentState,
			HashMap<Integer, ArrayList<Entry<Integer, Entry<Integer, Integer>>>> sharedStateChanges)
	{

		State stateToRet = new State(parentState);

		// so now we find out what changed between this state and the parent state
		for (int i = 0; i < states.length; i++) {
			ArrayList<Entry<Integer, Entry<Integer, Integer>>> sharedStatesChangedByRobot = new ArrayList<Entry<Integer, Entry<Integer, Integer>>>();
			State robotState = statesList.get(states[i]);
			int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(robotState);

			Object[] ss = StatesHelper.getSharedStatesFromState(robotState, varlist, this.sharedStatesNamesList);
			Object[] das = StatesHelper.getDAStatesFromState(robotState, varlist, this.numTasks);

			// if the da state is different form the parent state copy it
			//			for (int j = 0; j < das.length; j++) {
			//				if (parentState.varValues[j] != das[j] && das[j] != this.daInitialStates.get(j))
			//					stateToRet.setValue(j, das[j]);
			//			}
			int jj = 0;
			for (int j = 0; j < varlist.getNumVars(); j++) {

				String name = varlist.getName(j);
				if (name.contains("da")) {
					int jsIndex = varListMapping.get(name);
					if ((int) parentState.varValues[jsIndex] != (int) das[jj] && (int) das[jj] != this.daInitialStates.get(jj))
						stateToRet.setValue(jsIndex, das[jj]);
					jj++;
				}
			}
			if (ss != null) {
				for (int j = 0; j < ss.length; j++) {
					int jsIndex = this.jointMDP.getVarList().getIndex(this.sharedStatesNamesList.get(j));
					if (ss[j] != null) {
						if ((int) parentState.varValues[jsIndex] != (int) ss[j]) {
							//stateToRet.setValue(j + das.length, ss[j]);
							stateToRet.setValue(jsIndex, ss[j]);
							sharedStatesChangedByRobot.add(new AbstractMap.SimpleEntry<Integer, Entry<Integer, Integer>>(j + das.length,
									new AbstractMap.SimpleEntry<Integer, Integer>((int) ss[j], (int) parentState.varValues[j + das.length])));
						}
					}
				}
			}
			if (sharedStatesChangedByRobot.size() > 0) {
				sharedStateChanges.put(rnum, sharedStatesChangedByRobot);
			}

			Object[] rs = StatesHelper.getMDPStateFromState(robotState, varlist, isolatedStatesNamesList);
			for (int j = 0; j < rs.length; j++) {
				// TODO: We need to figure out how this works for multiple states for robots!!!
				stateToRet.setValue(varListMapping.get("r" + rnum) + j, rs[j]);
			}

		}

		return stateToRet;
	}

	protected State createJointState(int[] states, List<State> statesList, VarList varlist, HashMap<Integer, ArrayList<Entry<Integer, Integer>>> taskAllocation)
	{

		// for each state
		State stateToRet = new State(this.jointMDP.getVarList().getNumVars());

		// ArrayList<State> robotStates = new ArrayList<State>();
		for (int i = 0; i < states.length; i++) {
			State robotState = statesList.get(states[i]);

			int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(robotState);
			// get the robot number
			// get the ss and da

			Object[] ss = StatesHelper.getSharedStatesFromState(robotState, varlist, this.sharedStatesNamesList);
			Object[] das = StatesHelper.getDAStatesFromState(robotState, varlist, this.numTasks);

			for (int j = 0; j < das.length; j++) {
				if (stateToRet.varValues[j] == null)
					stateToRet.setValue(j, das[j]);
				else {
					if (taskAllocation.containsKey(rnum)) {
						for (Entry<Integer, Integer> ta : taskAllocation.get(rnum)) {
							if ((int) ta.getKey() == j + 1) {
								stateToRet.setValue(j, das[j]);
							}
						}
					}
				}

			}

			if (ss != null) {
				for (int j = 0; j < ss.length; j++) {
					if (stateToRet.varValues[j + das.length] == null)
						stateToRet.setValue(j + das.length, ss[j]);
					else {
						if (((int) stateToRet.varValues[j + das.length]) != (int) ss[j]
								&& (int) ss[j] != sharedVarsInitialStates.get(this.sharedStatesNamesList.get(j))) {
							//							stateToRet.setValue(j + das.length, ss[j]);
							stateToRet.setValue(this.jointMDP.getVarList().getIndex(this.sharedStatesNamesList.get(j)), ss[j]);
						}
					}
				}
			}

			Object[] rs = StatesHelper.getMDPStateFromState(robotState, varlist, isolatedStatesNamesList);
			for (int j = 0; j < rs.length; j++) {
				// TODO: We need to figure out how this works for multiple states for robots!!!
				stateToRet.setValue(varListMapping.get("r" + rnum) + j, rs[j]);
			}

		}

		return stateToRet;

	}

	protected State createJointStateConsideringFirstRobotOnly(int[] states, List<State> statesList, int firstRobot, VarList varlist,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> stateValuesBeforeTaskAllocation)
	{
		// if (considerFirstRobotOnly) {
		State stateToRet = new State(this.jointMDP.getVarList().getNumVars());

		// ArrayList<State> robotStates = new ArrayList<State>();
		for (int i = 0; i < states.length; i++) {
			State robotState = statesList.get(states[i]);
			// robotStates.add(robotState);
			int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(robotState);
			if (rnum == firstRobot) {
				// keep this robot's da bits and ss bits
				Object[] ss = StatesHelper.getSharedStatesFromState(robotState, varlist, this.sharedStatesNamesList);
				Object[] das = StatesHelper.getDAStatesFromState(robotState, varlist, this.numTasks);
				int jj = 0;
				for (int j = 0; j < varlist.getNumVars(); j++) {

					String name = varlist.getName(j);
					if (name.contains("da")) {
						stateToRet.setValue(varListMapping.get(name), das[jj]);
						jj++;
					}
				}
				//				for (int j = 0; j < das.length; j++) {
				//					
				//					stateToRet.setValue(j, das[j]);
				//
				//				}
				if (ss != null) {
					//get the joint mdp varlist name 
					//					if (ss != null) {
					//						for (int j = 0; j < ss.length; j++)
					//							if (ss[j] != null)
					//								newState[teamdpvarlist.getIndex(this.sharedStatesNamesList.get(j))] = ss[j];
					for (int j = 0; j < ss.length; j++) {
						stateToRet.setValue(this.jointMDP.getVarList().getIndex(this.sharedStatesNamesList.get(j)), ss[j]);
					}
				}
			}
			Object[] rs = StatesHelper.getMDPStateFromState(robotState, varlist, isolatedStatesNamesList);
			for (int j = 0; j < rs.length; j++) {
				// TODO: We need to figure out how this works for multiple states for robots!!!
				stateToRet.setValue(varListMapping.get("r" + rnum) + j, rs[j]);
			}
		}
		return stateToRet;

	}

	boolean isAcceptingState(State jointState)
	{
		Object[] das = StatesHelper.getDAStatesFromState(jointState, this.jointMDP.getVarList(), this.numTasks);
		//TODO: come back here
		//so the assumption is the that the order is thate same 
		boolean isAcc = true;
		for (int i = 0; i < das.length; i++) {
			if ((int) das[i] != (int) this.daFinalStates.get(i)) {
				isAcc = false;
				break;
			}
		}
		return isAcc;

	}

	// FIXME: a lot of hardcoding here which is not required at all
	// so i'm just being lazy af
	protected int[] extractIndividualRobotStatesFromJointState(State jointState, List<State> teamMDPStatesList, VarList teamdpvarlist) throws PrismException
	{

		Object[] ss = StatesHelper.getSharedStatesFromState(jointState, this.jointMDP.getVarList(), this.sharedStatesNamesList);
		Object[] das = StatesHelper.getDAStatesFromState(jointState, this.jointMDP.getVarList(), this.numTasks);

		int numVarsInTeamMDP = teamdpvarlist.getNumVars();
		int[] statesToRet = new int[this.numRobots];
		Arrays.fill(statesToRet, StatesHelper.BADVALUE);
		VarList varlist = this.jointMDP.getVarList();
		// create a joint state for each robot
		for (int i = 0; i < this.numRobots; i++) {
			Object[] newState = new Object[numVarsInTeamMDP];
			newState[teamdpvarlist.getIndex("r")] = i;
			int jj = 0;
			for (int j = 0; j < varlist.getNumVars(); j++) {

				String name = varlist.getName(j);
				if (name.contains("da")) {
					newState[teamdpvarlist.getIndex(name)] = das[jj];
					//					stateToRet.setValue(teamdpvarlist.getIndex(name), das[jj]); 
					jj++;
				}
			}
			//			for (int j = 0; j < das.length; j++)
			//				newState[teamdpvarlist.getIndex("da" + j)] = das[j];
			// so this is a bit problematic
			// and I dont want to cheat ish
			// so I'll be safe
			// honestly I'm quite dizzy
			// and this is a bit of a dizzier , so what I'm going to do is cheat and fix
			// this later
			// okay ? okay
			// dizzier - a thing that makes me dizzy

			// assuming the shared states are in the same order

			if (ss != null) {
				for (int j = 0; j < ss.length; j++)
					if (ss[j] != null)
						newState[teamdpvarlist.getIndex(this.sharedStatesNamesList.get(j))] = ss[j];
			}
			for (int j = 0; j < isolatedStatesNamesList.size(); j++)
				newState[teamdpvarlist.getIndex(this.isolatedStatesNamesList.get(j))] = StatesHelper.getIndexValueFromState(jointState,
						varListMapping.get("r" + i));
			// i just want to say that my code has been better
			//			if (Arrays.toString(newState).contains("[0, 0, 0, 0, 1, 1, -1]"))
			//				mainLog.println("Debug here");
			int sameState = StatesHelper.getExactlyTheSameState(newState, teamMDPStatesList);
			mainLog.println(Arrays.toString(newState));
			if (sameState == StatesHelper.BADVALUE) {
				mainLog.println("Cant find state index for " + Arrays.toString(newState) + " in teamMDP");
				throw new PrismException("Cant find state index for " + Arrays.toString(newState) + " in teamMDP");
			}
			statesToRet[i] = sameState;
		}
		return statesToRet;
	}

	public boolean hasFailedStates()
	{
		return !failedStatesQueue.isEmpty();
	}

	public Entry<State, BitSet> getNextFailedState()
	{
		StateExtended state = failedStatesQueue.remove();
		this.currentStateProbability = state.parentToChildTransitionProbability;
		State failstate = jointMDP.getStatesList().get(state.childState);
		BitSet failstateBitSetToAvoid = null;
		if (state.statesToAvoid != null)
			failstateBitSetToAvoid = (BitSet) state.statesToAvoid.clone();
		return new AbstractMap.SimpleEntry<State, BitSet>(failstate, failstateBitSetToAvoid);
	}

	public void printStatesExploredOrder()
	{

		for (int i = 0; i < statesExploredOrder.size(); i++) {
			double prob= getProbabilityToReachAccStateFromJointMDP(statesExploredOrder.get(i).getKey());
			this.mainLog.println(i + 1 + ":" + statesExploredOrder.get(i).toString() + " - "+ prob);
			
		}

	}
	
	public double getProbabilityOfSatisfactionFromInitState()
	{
		return getProbabilityToReachAccStateFromJointMDP(statesExploredOrder.get(0).getKey());
	}

}
