package demos;

import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
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

/*
 * An attempt to clean up code 
 * and make this more doable 
 * A new class for creating and storing the joint policy
 * so maybe this isnt the smartest name
 */
public class JointPolicyBuilder
{

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
	ArrayList<MDPRewardsSimple> seqTeamMDPRewards = null;
	ArrayList<MDPRewardsSimple> otherRewards = null;
	HashMap<Entry<Integer, Integer>, Double> progressionRewardsHashMap = null;
	HashMap<Entry<Integer, Integer>, ArrayList<Double>> otherRewardsHashMap = null;

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
	BitSet essStates = new BitSet();
	ArrayList<State> essStatesList = new ArrayList<State>();
	boolean doSeq = false;
	public boolean reallocateOnLastRobotFailure=false;

	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList, VarList seqTeamMDPVarList, ArrayList<MDPRewardsSimple> rewards,
			PrismLog log)
	{

		ArrayList<String> isolatedStatesList = new ArrayList<String>();
		for (int i = 0; i < seqTeamMDPVarList.getNumVars(); i++) {
			String name = seqTeamMDPVarList.getName(i);
			if ((!sharedStatesList.contains(name)) && (!name.contains("da")) && (name != "r")) {
				isolatedStatesList.add(name);
			}
		}
		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, seqTeamMDPVarList, rewards, log);

	}

	//	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList, ArrayList<String> isolatedStatesList, VarList seqTeamMDPVarList,
	//			ArrayList<MDPRewardsSimple> rewards, PrismLog log)
	//	{
	//
	//		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, seqTeamMDPVarList, rewards, log);
	//	}

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
			ArrayList<MDPRewardsSimple> rewards, PrismLog log)
	{
		statesMap = new HashMap<State, Integer>();
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

		progressionRewardsHashMap = new HashMap<Entry<Integer, Integer>, Double>();
		otherRewardsHashMap = new HashMap<Entry<Integer, Integer>, ArrayList<Double>>();

		if (rewards.size() > 1) {
			if (this.seqTeamMDPRewards == null)
				seqTeamMDPRewards = new ArrayList<MDPRewardsSimple>();
			//so the assumption that we dont care about progression rewards 
			for (int i = 1; i < rewards.size(); i++) {
				seqTeamMDPRewards.add(rewards.get(i));
			}

		}

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

	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, SequentialTeamMDP seqTeamMDP, int initialStateInSeqTeamMDP,
			boolean reallocateOnSingleAgentDeadend, double initStateProb) throws PrismException
	{
		//				PolicyCreator pc = new PolicyCreator();
		//				pc.createPolicy(seqTeamMDP.teamMDPWithSwitches, strat);
		//				pc.savePolicy("/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSIFS/results/", "seqTeamPolicy.dot");

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
		buildJointPolicyFromSequentialPolicy(strat, seqTeamMDP.teamMDPWithSwitches, currentJointState, seqTeamMDP.acceptingStates,
				reallocateOnSingleAgentDeadend, initStateProb);
		jointMDP.addInitialState(statesMap.get(currentJointState));
	}



	public double getStateProb(int s, MDPSimple mdp, int depth, int maxDepth)
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
		this.mainLog.println("Probability of satisfaction from state " + js.toString() + ": " + prob);
		return prob;

	}

	// the real thing
	// input: \pi_seq, s_J, mdp
	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, MDPSimple mdp, State initialJointState, BitSet seqMDPAccStates,
			boolean reallocateOnSingleAgentDeadend, double initStateProb) throws PrismException
	{

		State currentJointState = initialJointState;
		int[] initialRobotStatesInSeqTeamMDP = extractIndividualRobotStatesFromJointState(currentJointState, mdp.getStatesList(), mdp.getVarList());

		if (!inStatesExplored(currentJointState)) {
			statesExploredOrder.add(new AbstractMap.SimpleEntry<State, Double>(currentJointState, this.currentStateProbability));
			Queue<Entry<State, Double>> jointStateQueue = new LinkedList<Entry<State, Double>>();

			jointStateQueue.add(new AbstractMap.SimpleEntry<State, Double>(currentJointState, initStateProb));
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
					double currentJointStateProb = currentJointStateProbPair.getValue();

					boolean isAcc = this.isAcceptingState(currentJointState);

					int stateIndex = findStateIndex(currentJointState);
					boolean discovered = false;

					if (stateIndex != StatesHelper.BADVALUE) {
						if (jointStatesDiscovered.get(stateIndex)) {
							discovered = true;

						}
					}
					if (!discovered) {

						int[] robotStatesInSeqTeamMDP = extractIndividualRobotStatesFromJointState(currentJointState, mdp.getStatesList(), mdp.getVarList());

						if (isAcc) {
							accStates.set(statesMap.get(currentJointState));
							continue;
						}
						if (reallocateOnLastRobotFailure) {
							if (currentJointState.compareTo(initialJointState) != 0) {

								//only checking if the last robot has failed 
								int lastRobotIndex = robotStatesInSeqTeamMDP.length - 1;
								if (!failedInInitialState[lastRobotIndex]) {
									boolean robotIsDeadend = StatesHelper.stateIsDeadend(mdp, robotStatesInSeqTeamMDP[lastRobotIndex]);
									if (robotIsDeadend) {
										double probVar = currentJointStateProbPair.getValue();
										StateExtended failState = new StateExtended(stateIndex, probVar);
										this.failedStatesQueue.add(failState);
										continue;
									}
								}
							}
						}
						if (reallocateOnSingleAgentDeadend) {
							if (currentJointState.compareTo(initialJointState) != 0) {
								boolean reallocate = false;
								boolean allDeadend = true;
								//so in this state has a robot failed 
								for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
									boolean robotIsDeadend = StatesHelper.stateIsDeadend(mdp, robotStatesInSeqTeamMDP[i]);
									boolean reallocateHere = robotIsDeadend;
									if (robotStatesInSeqTeamMDP[i] == initialRobotStatesInSeqTeamMDP[i])
										reallocateHere = false;
									else {
										//if its a different da state but still the same robot 
										if (failedInInitialState[i])
											reallocateHere = false;
									}

									allDeadend = allDeadend & robotIsDeadend;
									//TODO: fix this - change the state matching to be independent of DA states so we don't just get new states and say thats a reallocation

									if (!reallocate)
										reallocate = reallocateHere;

								}
								if (reallocate) {
									if (!allDeadend) {
										double probVar = currentJointStateProbPair.getValue();
										StateExtended failState = new StateExtended(stateIndex, probVar);
										this.failedStatesQueue.add(failState);

									}
									continue;
								}
							}
						}
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
							stateValuesBeforeTaskAllocationBeforeProcessing = getStateIndexValuesBeforeTaskAllocationForAllRobots(statesDiscovered,
									mdp.getStatesList(), mdp.getVarList(), followingPath);
							if (!doSeq) {
								mostProbableTaskAllocationStateValues = this
										.processStateIndexValuesHolistic(mostProbableTaskAllocationStateValuesBeforeProcessing);

								modifiedRobotStatesInSeqTeamMDP = modifyRobotStatesToReflectExpectedTaskCompletionHolistic(robotStatesInSeqTeamMDP,
										mostProbableTaskAllocationStateValues, mdp.getStatesList());
							} else {
								mostProbableTaskAllocationStateValues = this.processStateIndexValuesSeq(statesDiscovered,
										mostProbableTaskAllocationStateValuesBeforeProcessing, mdp.getStatesList());

								modifiedRobotStatesInSeqTeamMDP = this.modifyRobotStatesToReflectExpectedTaskCompletionSeq(statesDiscovered,
										robotStatesInSeqTeamMDP, mostProbableTaskAllocationStateValuesBeforeProcessing,
										stateValuesBeforeTaskAllocationBeforeProcessing, sharedStateChanges, mdp.getStatesList(), 0);
							}

							if (!doSeq) {
								stateValuesBeforeTaskAllocation = this.processStateIndexValuesHolistic(stateValuesBeforeTaskAllocationBeforeProcessing);
							} else {
								stateValuesBeforeTaskAllocation = this.processStateIndexValuesSeq(statesDiscovered,
										stateValuesBeforeTaskAllocationBeforeProcessing, mdp.getStatesList());
							}
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
									mainLog.println("Error bad state");
								}
							}
							if (getNewTaskAllocation(statesDiscovered, modifiedRobotStatesInSeqTeamMDP, followingPath /* we have to follow the path */,
									skipAutomataStatesInTA, mdp.getStatesList(), mdp.getVarList(), seqMDPAccStates)) {
								//								extractPolicyTreeAsDotFile(strat, mdp,  modifiedRobotStatesInSeqTeamMDP[0]);
								statesDiscovered = getTaskAllocationForAllRobots(strat, mdp, /*robotStatesInSeqTeamMDP*/modifiedRobotStatesInSeqTeamMDP,
										followingPath/* follow a path cuz you dont know anything */, true/*use all robot states*/);
								mostProbableTaskAllocationStateValuesBeforeProcessing = getStateIndexValuesForTaskAllocationForAllRobots(statesDiscovered,
										mdp.getStatesList(), mdp.getVarList(), followingPath);

							}

							if (!doSeq) {
								mostProbableTaskAllocationStateValues = this
										.processStateIndexValuesHolistic(mostProbableTaskAllocationStateValuesBeforeProcessing);

								modifiedRobotStatesInSeqTeamMDP = modifyRobotStatesToReflectExpectedTaskCompletionHolistic(robotStatesInSeqTeamMDP,
										mostProbableTaskAllocationStateValues, mdp.getStatesList());
							} else {
								mostProbableTaskAllocationStateValues = this.processStateIndexValuesSeq(statesDiscovered,
										mostProbableTaskAllocationStateValuesBeforeProcessing, mdp.getStatesList());

								modifiedRobotStatesInSeqTeamMDP = this.modifyRobotStatesToReflectExpectedTaskCompletionSeq(statesDiscovered,
										robotStatesInSeqTeamMDP, mostProbableTaskAllocationStateValuesBeforeProcessing,
										stateValuesBeforeTaskAllocationBeforeProcessing, sharedStateChanges, mdp.getStatesList(), 0);
							}

							stateValuesBeforeTaskAllocationBeforeProcessing = getStateIndexValuesBeforeTaskAllocationForAllRobots(statesDiscovered,
									mdp.getStatesList(), mdp.getVarList(), followingPath);
							if (!doSeq) {
								stateValuesBeforeTaskAllocation = this.processStateIndexValuesHolistic(stateValuesBeforeTaskAllocationBeforeProcessing);
							} else {
								stateValuesBeforeTaskAllocation = this.processStateIndexValuesSeq(statesDiscovered,
										stateValuesBeforeTaskAllocationBeforeProcessing, mdp.getStatesList());
							}
						}

						boolean usingModifiedStates = true;//false;
//						if(currentJointState.toString().contains("(0,0,0,0,0,0,1,22,2,14,-1)"))
//							mainLog.println("debug here");
//						mainLog.println(currentJointState.toString());
						Entry<Entry<String, ArrayList<Double>>, ArrayList<Entry<int[], Double>>> actionAndCombinations = getActionAndSuccStatesAllRobots(strat,
								modifiedRobotStatesInSeqTeamMDP, robotStatesInSeqTeamMDP, mdp, usingModifiedStates);

						Entry<String, ArrayList<Double>> actionStringAndRews = actionAndCombinations.getKey();
						String action = actionStringAndRews.getKey();
						ArrayList<Double> summedStateActionRewards = actionStringAndRews.getValue();
						ArrayList<State> succStatesQueue = new ArrayList<State>();
						ArrayList<Double> succStatesProbQueue = new ArrayList<Double>();
						double taskProgressionReward = 0;
						for (Entry<int[], Double> combination : actionAndCombinations.getValue()) {

							int[] modifiedRobotSuccStatesInSeqTeamMDP = combination.getKey();
							int[] newSuccStatesForJointState = modifiedRobotSuccStatesInSeqTeamMDP;
							if (usingModifiedStates) {
								if (!doSeq) {
									newSuccStatesForJointState = modifyRobotStatesToUndoExpectedTaskCompletionHolistic(modifiedRobotSuccStatesInSeqTeamMDP,
											stateValuesBeforeTaskAllocation, mdp.getStatesList());
								} else {
									newSuccStatesForJointState = modifyRobotStatesToUndoExpectedTaskCompletionSeq(modifiedRobotSuccStatesInSeqTeamMDP,
											stateValuesBeforeTaskAllocation, mdp.getStatesList(), 0);
								}

							}

							sharedStateChanges = new HashMap<Integer, ArrayList<Entry<Integer, Entry<Integer, Integer>>>>();
							State succJointState = createJointState(newSuccStatesForJointState, mdp.getStatesList(), mdp.getVarList(), currentJointState,
									sharedStateChanges);

							succStatesQueue.add(succJointState);
							succStatesProbQueue.add(combination.getValue());
							jointStateQueue.add(new AbstractMap.SimpleEntry<State, Double>(succJointState, combination.getValue() * currentJointStateProb));
							statesDiscoveredQ.add(statesDiscovered);
							mostProbableTaskAllocationStateValuesBeforeProcessingQ.add(mostProbableTaskAllocationStateValuesBeforeProcessing);
							stateValuesBeforeTaskAllocationBeforeProcessingQ.add(stateValuesBeforeTaskAllocationBeforeProcessing);
							sharedStateChangesQ.add(sharedStateChanges);
							// add to mdp
							int numEss = isEssentialState(currentJointState, succJointState, mdp.getVarList());
							if (numEss > 0) {
								essStatesList.add(succJointState);
								taskProgressionReward += ((double) numEss) * combination.getValue();
							}
						}


						int actionIndex = addTranstionToMDP(jointMDP, currentJointState, succStatesQueue, succStatesProbQueue, action, 1.0);
						int currentJSIndex = statesMap.get(currentJointState);
						jointStatesDiscovered.set(currentJSIndex);
						SimpleEntry<Integer, Integer> saPair = new AbstractMap.SimpleEntry<Integer, Integer>(currentJSIndex, actionIndex);
						if (taskProgressionReward > 0) {
							this.progressionRewardsHashMap.put(saPair, taskProgressionReward);
						}

						for (int ri = 0; ri < summedStateActionRewards.size(); ri++) {
							if (summedStateActionRewards.get(ri) == 0) {
								if (!action.toString().contains("*") && !action.toString().contains("switch"))
									System.out.println("Why are the state rewards 0??" + action.toString());
							}
						}
						if (this.otherRewardsHashMap.containsKey(saPair)) {

//							System.out.print("Your fears have been realised - the rewards are being replaced");

						} else
							this.otherRewardsHashMap.put(saPair, summedStateActionRewards);


					} else {
						if (!isAcc) {
							// get all states for all robots which don't have the exact seq task value
							//						StateExtended failState = new StateExtended(stateIndex, probVar);
							//						if (statesToAvoidDueToSeqTask != null && !statesToAvoidDueToSeqTask.isEmpty())
							//							failState.statesToAvoid = (BitSet) statesToAvoidDueToSeqTask.clone();
							//						this.failedStatesQueue.add(failState);
							//
							//so we've seen this state before right ? 
							//okay so lets go ahead and add this state to our list 
							int[] robotStatesInSeqTeamMDP = extractIndividualRobotStatesFromJointState(currentJointState, mdp.getStatesList(),
									mdp.getVarList());

							if (currentJointState.compareTo(initialJointState) != 0) {

								boolean allDeadends = true;
								//so in this state has a robot failed 
								for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
									allDeadends &= StatesHelper.stateIsDeadend(mdp, robotStatesInSeqTeamMDP[i]);
									if (!allDeadends)
										break;
								}
								if (!allDeadends) {
									double probVar = currentJointStateProbPair.getValue();
									StateExtended failState = new StateExtended(stateIndex, probVar);
									this.failedStatesQueue.add(failState);
								}
							}
						} else {
							//							if (isAcc) {
							accStates.set(statesMap.get(currentJointState));
							//							}
						}

					}
				}
			}

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
				if (sharedStateChanges != null) {
					//if the shared state is changed by a robot later we have to undo it here
					if (sharedStateChanges.containsKey(nextRobot)) {
						ArrayList<Entry<Integer, Entry<Integer, Integer>>> sharedStatesToChange = sharedStateChanges.get(nextRobot);
						for (int j = 0; j < sharedStatesToChange.size(); j++) {
							currentState[sharedStatesToChange.get(j).getKey() + 1] = sharedStatesToChange.get(j).getValue().getValue();
						}
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

	private int addTranstionToMDP(MDPSimple mdp, State parentStates, ArrayList<State> states, ArrayList<Double> probs, String action, double norm)
	{
		int parentIndex = addStateToMDP(parentStates, mdp);
		int index;
		Distribution distr = new Distribution();

		for (int succ = 0; succ < states.size(); succ++) {
			index = addStateToMDP(states.get(succ), mdp);
			double normalizedProb = probs.get(succ) / norm;
			distr.add(index, normalizedProb);

		}

		int actionIndex = mdp.addActionLabelledChoice(parentIndex, distr, action);
		return actionIndex;
	}

	//	private State getStateFromInd(int stateInd)
	//	{
	//		return this.jointMDP.getStatesList().get(stateInd);
	//	}

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
			boolean skipAutomataStatesInTA, List<State> statesList, VarList varlist, BitSet seqMDPAccStates)
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
				int maxR = robotStatesInSeqTeamMDP.length;
				//we need to check which robots to skip if any 
				for (int i = 0; i < statesDiscovered.get(0).size(); i++) {
					StateExtended cs = statesDiscovered.get(0).get(i);
					if (seqMDPAccStates.get(cs.childState)) {

						State stateState = statesList.get(cs.childState);

						int stateR = StatesHelper.getRobotNumberFromSeqTeamMDPState(stateState);
						maxR = stateR + 1;
						break;
					}
				}
				boolean newTA = false;
				for (int i = 0; i < maxR; i++) {

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

	private int[] modifyRobotStatesToReflectExpectedTaskCompletionSeqNew(int[] robotStatesInSeqTeamMDP,
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
			// but even if you've failed, there's just a switch isn't there ? 
			// okay but I won't change this 
			// though I don't know how this works in detail 

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

	ArrayList<Double> getRobotRewards(int state, int choice)
	{
		double rew = 0;
		ArrayList<Double> costRewards = new ArrayList<Double>();
		//rewards 
		if (seqTeamMDPRewards == null)
			mainLog.println("seqTeamMDPRewards are null!!! I think I get why!! ");
		for (int i = 0; i < seqTeamMDPRewards.size(); i++) {
			//TODO: dont ignore state rewards 
			//ignoring state rewards!!!!!!
			rew = 0;
			if (choice > -1) {
				rew = seqTeamMDPRewards.get(i).getTransitionReward(state, choice);

			}
			costRewards.add(rew);
		}
		return costRewards;
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

	//commented this cuz I got another function that works just fine 
	//	protected double getProgressionReward(MDStrategyArray strat, int[] states, SequentialTeamMDP teamMDP)
	//	{
	//		double progRew = 0;
	//		//rewards are sums 
	//
	//		for (int i = 0; i < states.length; i++) {
	//
	//			Entry<Object, Integer> actionChoice = getActionChoice(strat, states[i]);
	//			progRew += teamMDP.progressionRewards.getTransitionReward(states[i], actionChoice.getValue());
	//
	//		}
	//		return progRew;
	//	}
	//
	//	protected ArrayList<Double> getOtherRewards(MDStrategyArray strat, int[] states, SequentialTeamMDP teamMDP)
	//	{
	//		ArrayList<Double> allRews = new ArrayList<Double>();
	//		for (int rew = 0; rew < teamMDP.rewardsWithSwitches.size(); rew++)
	//			allRews.add(0.0);
	//		for (int i = 0; i < states.length; i++) {
	//
	//			Entry<Object, Integer> actionChoice = getActionChoice(strat, states[i]);
	//			for (int rew = 0; rew < teamMDP.rewardsWithSwitches.size(); rew++) {
	//				double currentRew = allRews.get(rew);
	//				double rewForState = teamMDP.rewardsWithSwitches.get(rew).getTransitionReward(states[i], actionChoice.getValue());
	//				currentRew += rewForState;
	//				allRews.set(rew, currentRew);
	//			}
	//
	//		}
	//		return allRews;
	//	}

	ArrayList<Double> addTwoArrayListElementWise(ArrayList<Double> a1, ArrayList<Double> a2) throws PrismException
	{
		int s1 = a1.size();
		int s2 = a2.size();
		if (s1 == 0 && s2 == 0)
			return a1;
		else if (s1 == 0 && s2 > 0) {
			return a2;
		} else if (s1 > 0 && s2 == 0) {
			return a1;
		} else {
			if (s1 == s2) {
				for (int i = 0; i < s1; i++) {
					double s = a1.get(i) + a2.get(i);
					a1.set(i, s);
				}
				return a1;
			} else
				throw new PrismException("Unequal Arrays");
		}
	}

	ArrayList<Double> getAllRobotRewards(HashMap<Integer, Entry<Object, Integer>> actionChoices) throws PrismException
	{
		ArrayList<Double> summedRewards = new ArrayList<Double>();
		//for each entry in the action choice thing 
		for (int state : actionChoices.keySet()) {
			//get the robots reward 
			Entry<Object, Integer> actionChoice = actionChoices.get(state);
			int choice = actionChoice.getValue();
			ArrayList<Double> robotRewards = getRobotRewards(state, choice);
			summedRewards = addTwoArrayListElementWise(summedRewards, robotRewards);
		}

		return summedRewards;

	}

	protected Entry<Entry<String, ArrayList<Double>>, ArrayList<Entry<int[], Double>>> getActionAndSuccStatesAllRobots(MDStrategyArray strat,
			int[] modifiedStates, int[] states, MDPSimple mdp, boolean usingModifiedState) throws PrismException
	{
		Entry<Entry<String, ArrayList<Double>>, ArrayList<Entry<int[], Double>>> toret = null;
		HashMap<Integer, Entry<Object, Integer>> actionChoices = getActionChoiceAllRobots(strat, modifiedStates);
		HashMap<Integer, ArrayList<Entry<Integer, Double>>> succArrs = new HashMap<Integer, ArrayList<Entry<Integer, Double>>>();
		// for each action choice we need to get the successorstates
		ArrayList<Double> summedRews = getAllRobotRewards(actionChoices);
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
			Iterator<Entry<Integer, Double>> tranIter =null;
			if(usingModifiedState)
				tranIter= getTranIter(actionChoices.get(modifiedStates[i]).getValue(), modifiedStates[i], mdp, usingModifiedState,
					currentAction);
			else
				tranIter= getTranIter(actionChoices.get(modifiedStates[i]).getValue(), states[i], mdp, usingModifiedState,
						currentAction);
			ArrayList<Entry<Integer, Double>> tranIterList = tranIterToArrayList(tranIter);
			if (forceNull)
				tranIterList = null;
			if (tranIterList == null) {
				// then the list is just that state there aren't any more actions
				tranIterList = new ArrayList<Entry<Integer, Double>>();
				if(usingModifiedState)
				tranIterList.add(new AbstractMap.SimpleEntry<Integer, Double>(modifiedStates[i], 1.0));
				else
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
		Entry<String, ArrayList<Double>> actionStringAndRews = new AbstractMap.SimpleEntry<String, ArrayList<Double>>(jointAction, summedRews);

		toret = new AbstractMap.SimpleEntry<Entry<String, ArrayList<Double>>, ArrayList<Entry<int[], Double>>>(actionStringAndRews, succStateProbList);
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

	int isEssentialState(State ps, State cs, VarList varlist)
	{
		int essState = 0;

		int jj = 0;
		for (int j = 0; j < varlist.getNumVars(); j++) {
			String name = varlist.getName(j);
			if (name.contains("da")) {

				int jsIndex = varListMapping.get(name);
				if ((int) ps.varValues[jsIndex] != (int) cs.varValues[jsIndex]) {
					if ((int) cs.varValues[jsIndex] == daFinalStates.get(jj)) {
						if (daFinalStates.get(jj) != daInitialStates.get(jj)) {
							essState++;
						}
					}
				}
				jj++;
			}

		}

		return essState;
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
					if ((int) parentState.varValues[jsIndex] != (int) das[jj] && (int) das[jj] != this.daInitialStates.get(jj)) {
						stateToRet.setValue(jsIndex, das[jj]);
					}
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

	//	protected State createJointState(int[] states, List<State> statesList, VarList varlist, HashMap<Integer, ArrayList<Entry<Integer, Integer>>> taskAllocation)
	//	{
	//
	//		// for each state
	//		State stateToRet = new State(this.jointMDP.getVarList().getNumVars());
	//
	//		// ArrayList<State> robotStates = new ArrayList<State>();
	//		for (int i = 0; i < states.length; i++) {
	//			State robotState = statesList.get(states[i]);
	//
	//			int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(robotState);
	//			// get the robot number
	//			// get the ss and da
	//
	//			Object[] ss = StatesHelper.getSharedStatesFromState(robotState, varlist, this.sharedStatesNamesList);
	//			Object[] das = StatesHelper.getDAStatesFromState(robotState, varlist, this.numTasks);
	//
	//			for (int j = 0; j < das.length; j++) {
	//				if (stateToRet.varValues[j] == null)
	//					stateToRet.setValue(j, das[j]);
	//				else {
	//					if (taskAllocation.containsKey(rnum)) {
	//						for (Entry<Integer, Integer> ta : taskAllocation.get(rnum)) {
	//							if ((int) ta.getKey() == j + 1) {
	//								stateToRet.setValue(j, das[j]);
	//							}
	//						}
	//					}
	//				}
	//
	//			}
	//
	//			if (ss != null) {
	//				for (int j = 0; j < ss.length; j++) {
	//					if (stateToRet.varValues[j + das.length] == null)
	//						stateToRet.setValue(j + das.length, ss[j]);
	//					else {
	//						if (((int) stateToRet.varValues[j + das.length]) != (int) ss[j]
	//								&& (int) ss[j] != sharedVarsInitialStates.get(this.sharedStatesNamesList.get(j))) {
	//							//							stateToRet.setValue(j + das.length, ss[j]);
	//							stateToRet.setValue(this.jointMDP.getVarList().getIndex(this.sharedStatesNamesList.get(j)), ss[j]);
	//						}
	//					}
	//				}
	//			}
	//
	//			Object[] rs = StatesHelper.getMDPStateFromState(robotState, varlist, isolatedStatesNamesList);
	//			for (int j = 0; j < rs.length; j++) {
	//				// TODO: We need to figure out how this works for multiple states for robots!!!
	//				stateToRet.setValue(varListMapping.get("r" + rnum) + j, rs[j]);
	//			}
	//
	//		}
	//
	//		return stateToRet;
	//
	//	}

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
			//			mainLog.println(Arrays.toString(newState));
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

	public Entry<Entry<State, Double>, BitSet> getNextFailedState()
	{
		StateExtended state = failedStatesQueue.remove();
		this.currentStateProbability = state.parentToChildTransitionProbability;
		State failstate = jointMDP.getStatesList().get(state.childState);
		BitSet failstateBitSetToAvoid = null;
		if (state.statesToAvoid != null)
			failstateBitSetToAvoid = (BitSet) state.statesToAvoid.clone();
		Entry<State, Double> failStateProbPair = new AbstractMap.SimpleEntry<State, Double>(failstate, state.parentToChildTransitionProbability);
		return new AbstractMap.SimpleEntry<Entry<State, Double>, BitSet>(failStateProbPair, failstateBitSetToAvoid);
	}

	public void printStatesExploredOrder()
	{

		for (int i = 0; i < statesExploredOrder.size(); i++) {
			double prob = getProbabilityToReachAccStateFromJointMDP(statesExploredOrder.get(i).getKey());
			this.mainLog.println(i + 1 + ":" + statesExploredOrder.get(i).toString() + " - " + prob);

		}

	}

	public double getProbabilityOfSatisfactionFromInitState()
	{
		return getProbabilityToReachAccStateFromJointMDP(statesExploredOrder.get(0).getKey());
	}

	public void createRewardStructures()
	{
		MDPSimple mdp = jointMDP;

		//the assumption is we're all done 
		//so we can just add stuff 
		if (otherRewardsHashMap != null & progressionRewardsHashMap != null) {
			progressionRewards = new MDPRewardsSimple(mdp.getNumStates());
			otherRewards = new ArrayList<MDPRewardsSimple>();

			for (Entry<Integer, Integer> saPair : progressionRewardsHashMap.keySet()) {
				progressionRewards.addToTransitionReward(saPair.getKey(), saPair.getValue(), progressionRewardsHashMap.get(saPair));
			}
			for (Entry<Integer, Integer> saPair : otherRewardsHashMap.keySet()) {
				for (int i = 0; i < otherRewardsHashMap.get(saPair).size(); i++) {
					if (otherRewards.size() < (i + 1)) {
						otherRewards.add(new MDPRewardsSimple(mdp.getNumStates()));
					}
					otherRewards.get(i).addToTransitionReward(saPair.getKey(), saPair.getValue(), otherRewardsHashMap.get(saPair).get(i));

				}
			}
		}
	}

	public ArrayList<MDPRewardsSimple> getExpTaskAndCostRewards()
	{
		ArrayList<MDPRewardsSimple> rewstoret = new ArrayList<MDPRewardsSimple>();
		rewstoret.add(progressionRewards);
		rewstoret.add(otherRewards.get(0));
		return rewstoret;
	}
}
