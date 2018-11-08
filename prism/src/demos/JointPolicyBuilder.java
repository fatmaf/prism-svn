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
import java.lang.Double;

import explicit.Distribution;
import explicit.MDPSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import prism.PrismLangException;
import prism.PrismLog;
import strat.MDStrategyArray;

/*
 * An attempt to clean up code 
 * and make this more doable 
 * A new class for creating and storing the joint policy
 * so maybe this isnt the smartest name
 */
public class JointPolicyBuilder {

	/*
	 * Storing state realted information parent state, robot number, probability etc
	 * just for ease of use
	 */
	public class StateExtended implements Comparable<StateExtended> {
		protected int parentState = -1;
		protected int parentStateRobot = -1;
		protected int childState = -1;
		protected int childStateRobot = -1;
		protected double parentToChildTransitionProbability = -1;
		protected String actionInChildState = null;
		protected int choice = -1;

		public StateExtended() {
			// dummy
		}

		public StateExtended(int ps, int psr, int cs, int csr, double prob, String a) {
			parentState = ps;
			parentStateRobot = psr;
			childState = cs;
			childStateRobot = csr;
			parentToChildTransitionProbability = prob;
			actionInChildState = a;
		}

		public StateExtended(int s, double prob, String a) {
			childState = s;
			parentToChildTransitionProbability = prob;
			actionInChildState = a;
		}

		public StateExtended(StateExtended other) {
			this.parentState = other.parentState;
			this.parentStateRobot = other.parentStateRobot;
			this.childState = other.childState;
			this.childStateRobot = other.childStateRobot;
			this.parentToChildTransitionProbability = other.parentToChildTransitionProbability;
			this.actionInChildState = other.actionInChildState;
		}

		public StateExtended(int initialState, double d) {
			childState = initialState;
			parentToChildTransitionProbability = d;

		}

		@Override
		public int compareTo(StateExtended other) {
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
		public String toString() {
			return "StateInfo [parentState=" + parentState + ", parentStateRobot=" + parentStateRobot + ", childState="
					+ childState + ", childStateRobot=" + childStateRobot + ", parentToChildTransitionProbability="
					+ parentToChildTransitionProbability + ", action=" + actionInChildState + "]";
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
	// helper bits
	PriorityQueue<StateExtended> failedStatesQueue = null;
	HashMap<String, Integer> varListMapping;
	ArrayList<Integer> daFinalStates = null;
	ArrayList<Integer> daInitialStates = null;
	HashMap<String, Integer> sharedVarsInitialStates;
	HashMap<State, Integer> statesMap = null;
	PrismLog mainLog;

	// sanity checking
	ArrayList<Entry<State, Double>> statesExploredOrder = null;
	double currentStateProbability = 1.0;

	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList, VarList seqTeamMDPVarList,
			PrismLog log) {
		statesMap = new HashMap<State, Integer>();
		ArrayList<String> isolatedStatesList = new ArrayList<String>();
		for (int i = 0; i < seqTeamMDPVarList.getNumVars(); i++) {
			String name = seqTeamMDPVarList.getName(i);
			if ((!sharedStatesList.contains(name)) && (!name.contains("da")) && (name != "r")) {
				isolatedStatesList.add(name);
			}
		}
		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, log);
	}

	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList,
			ArrayList<String> isolatedStatesList, PrismLog log) {
		statesMap = new HashMap<State, Integer>();
		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, log);
	}

	private VarList createVarList() {
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

			for (int i = 0; i < numSharedStates; i++) {
				decname = sharedStatesNamesList.get(i);
				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
			}
			for (int i = 0; i < numTasks; i++) {
				decname = "da" + i;
				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
			}
			for (int i = 0; i < varlist.getNumVars(); i++) {
				varListMapping.put(varlist.getName(i), i);
			}

		} catch (PrismLangException e) {
			e.printStackTrace();
		}

		return varlist;

	}

	private void initialize(int nrobots, int ntasks, ArrayList<String> sharedStatesList,
			ArrayList<String> isolatedStatesList, PrismLog log) {
		numRobots = nrobots;
		numTasks = ntasks;
		numSharedStates = sharedStatesList.size();
		sharedStatesNamesList = sharedStatesList;
		isolatedStatesNamesList = isolatedStatesList;
		mainLog = log;

		jointMDP = new MDPSimple();
		jointMDP.setVarList(createVarList());
		jointMDP.setStatesList(new ArrayList<State>());
		this.failedStatesQueue = new PriorityQueue<StateExtended>();
		this.statesExploredOrder = new ArrayList<Entry<State, Double>>();

	}

	public BitSet getDAaccStatesForRobot(int da_num, int r, SequentialTeamMDP seqTeamMDP) {
		if (!seqTeamMDP.agentMDPs.get(r).daList.get(da_num).isSafeExpr)
			return ((AcceptanceReach) seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getAcceptance())
					.getGoalStates();
		else {
			// TODO : maybe there are multiple initial states or something and so we need to
			// do something like not the final state in our check
			BitSet temp = new BitSet();
			temp.set(seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getStartState());
			return temp;
		}
	}

	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, SequentialTeamMDP seqTeamMDP,
			int initialStateInSeqTeamMDP) {
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
		State currentJointState = createJointState(currentRobotStates, seqTeamMDP.teamMDPWithSwitches.getStatesList(),
				true, firstRobotNumber, seqTeamMDP.teamMDPWithSwitches.getVarList(), null);
		if (sharedVarsInitialStates == null)
			sharedVarsInitialStates = new HashMap<String, Integer>();
		for (int i = 0; i < this.sharedStatesNamesList.size(); i++) {
			int mapping = varListMapping.get(this.sharedStatesNamesList.get(i));
			if (seqTeamMDP.teamMDPWithSwitches.getVarList().exists(sharedStatesNamesList.get(i)))
				sharedVarsInitialStates.put(sharedStatesNamesList.get(i), (int) currentJointState.varValues[mapping]);
		}
		buildJointPolicyFromSequentialPolicy(strat, seqTeamMDP.teamMDPWithSwitches, currentJointState);

	}

	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, MDPSimple mdp, int initialJointState) {
		State currentJointState = this.getStateFromInd(initialJointState);
		if (sharedVarsInitialStates == null)
			sharedVarsInitialStates = new HashMap<String, Integer>();
		for (int i = 0; i < this.sharedStatesNamesList.size(); i++) {
			int mapping = varListMapping.get(this.sharedStatesNamesList.get(i));
			sharedVarsInitialStates.put(sharedStatesNamesList.get(i), (int) currentJointState.varValues[mapping]);
		}
		buildJointPolicyFromSequentialPolicy(strat, mdp, currentJointState);

	}

	// the real thing
	// input: \pi_seq, s_J, mdp
	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, MDPSimple mdp, State initialJointState) {
		State currentJointState = initialJointState;
		if (!inStatesExplored(currentJointState)) {
			statesExploredOrder
					.add(new AbstractMap.SimpleEntry<State, Double>(currentJointState, this.currentStateProbability));
			Queue<Entry<State, Double>> jointStateQueue = new LinkedList<Entry<State, Double>>();

			jointStateQueue.add(new AbstractMap.SimpleEntry<State, Double>(currentJointState, 1.0));
			BitSet jointStatesDiscovered = new BitSet();
			boolean followingPath = false;// true;// false;
			boolean doreset = false;
			ArrayList<ArrayList<StateExtended>> statesDiscovered = null;
			Entry<State, Double> currentJointStateProbPair = null;
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValuesBeforeProcessing = null;
			Entry<boolean[], Integer> numFailedPlusFlagsInitialState = numFailedInJointState(initialJointState);
			int numFailedInInitialState = numFailedPlusFlagsInitialState.getValue();
			boolean[] failedInInitialState = numFailedPlusFlagsInitialState.getKey();

			if (jointStateQueue != null) {

				while (!jointStateQueue.isEmpty()) {

					currentJointStateProbPair = jointStateQueue.remove();
					currentJointState = currentJointStateProbPair.getKey();
					int stateIndex = findStateIndex(currentJointState);
					boolean discovered = false;

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
										State updatedJointState = resetTasksForFailedRobot(failedInInitialState,
												failedInState, currentJointState,
												mostProbableTaskAllocationStateValuesBeforeProcessing);
										if (updatedJointState != null) {
											addResetTransitionToJointPolicyMDP(currentJointState, updatedJointState,
													jointMDP);
											stateIndex = findStateIndex(updatedJointState);

										}
									}
									if (!inStatesExplored(currentJointState)) {
										this.failedStatesQueue.add(new StateExtended(stateIndex, probVar));
									}
									continue;
								}
							}

						}
						// if failure
						// continue;
						// else
						// do below

						int[] robotStatesInSeqTeamMDP = extractIndividualRobotStatesFromJointState(currentJointState,
								mdp.getStatesList(), mdp.getVarList());

						// once we have the initial robot states
						// we need to get the task allocation
						// from this particular start state to the one we end up in till we get to a
						// switch
						if (getNewTaskAllocation(statesDiscovered, robotStatesInSeqTeamMDP, followingPath)) // get new
																											// task
																											// allocation
							statesDiscovered = getTaskAllocationForAllRobots(strat, mdp, robotStatesInSeqTeamMDP,
									followingPath);

						mostProbableTaskAllocationStateValuesBeforeProcessing = getStateIndexValuesForTaskAllocationForAllRobots(
								statesDiscovered, mdp.getStatesList(), mdp.getVarList(), followingPath);
						HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues = this
								.processStateIndexValues(statesDiscovered,
										mostProbableTaskAllocationStateValuesBeforeProcessing, mdp.getStatesList());

						HashMap<Integer, ArrayList<Entry<Integer, Integer>>> stateValuesBeforeTaskAllocationBeforeProcessing = getStateIndexValuesBeforeTaskAllocationForAllRobots(
								statesDiscovered, mdp.getStatesList(), mdp.getVarList(), followingPath);

						HashMap<Integer, ArrayList<Entry<Integer, Integer>>> stateValuesBeforeTaskAllocation = this
								.processStateIndexValues(statesDiscovered,
										stateValuesBeforeTaskAllocationBeforeProcessing, mdp.getStatesList());

						int[] modifiedRobotStatesInSeqTeamMDP = modifyRobotStatesToReflectExpectedTaskCompletion(
								robotStatesInSeqTeamMDP, mostProbableTaskAllocationStateValues, mdp.getStatesList(), 0); // FIXME:
																															// hardcoding
																															// first
																															// robot
																															// fix
																															// this
																															// please

						Entry<String, ArrayList<Entry<int[], Double>>> actionAndCombinations = getActionAndSuccStatesAllRobots(
								strat, modifiedRobotStatesInSeqTeamMDP, mdp);

						String action = actionAndCombinations.getKey();
						ArrayList<State> succStatesQueue = new ArrayList<State>();
						ArrayList<Double> succStatesProbQueue = new ArrayList<Double>();
						for (Entry<int[], Double> combination : actionAndCombinations.getValue()) {
							// {
							int[] modifiedRobotSuccStatesInSeqTeamMDP = combination.getKey();

							int[] newSuccStatesForJointState = modifyRobotStatesToUndoExpectedTaskCompletion(
									modifiedRobotSuccStatesInSeqTeamMDP, stateValuesBeforeTaskAllocation,
									mdp.getStatesList(), 0);
							State succJointState = createJointState(newSuccStatesForJointState, mdp.getStatesList(),
									false, 0, mdp.getVarList(), stateValuesBeforeTaskAllocation);
							succStatesQueue.add(succJointState);
							succStatesProbQueue.add(combination.getValue());
							jointStateQueue.add(
									new AbstractMap.SimpleEntry<State, Double>(succJointState, combination.getValue()));
							// }
							// add to mdp

						}
						this.addTranstionToMDP(jointMDP, currentJointState, succStatesQueue, succStatesProbQueue,
								action, 1.0);
						jointStatesDiscovered.set(statesMap.get(currentJointState));
						saveMDP(jointMDP, "new");
					}
				}
			}
			saveMDP(jointMDP, "new");
		}

	}

	private void addResetTransitionToJointPolicyMDP(State currentJointState, State updatedJointState, MDPSimple mdp) {
		int parentIndex = addStateToMDP(currentJointState, mdp);
		Object action = "reset";
		int index;
		Distribution distr = new Distribution();

		index = addStateToMDP(updatedJointState, mdp);
		double normalizedProb = 1.0;
		distr.add(index, normalizedProb);

		mdp.addActionLabelledChoice(parentIndex, distr, action);

	}

	private State resetTasksForFailedRobot(boolean[] failedInInitialState, boolean[] failedInState,
			State currentJointState,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValuesBeforeProcessing) {
		State updatedJointState = null;
		// if the robot has failed - one that did not fail in the initial state
		// then check if the tasks it was allocated are in their final/initial states
		// if not , we need to reset them to their initial states
		boolean hasFailed = false;
		for (int i = 0; i < numRobots; i++) {
			hasFailed = !failedInInitialState[i] && failedInState[i];
			if (hasFailed) {
				if (mostProbableTaskAllocationStateValuesBeforeProcessing.containsKey(i)) {
					ArrayList<Entry<Integer, Integer>> tasks = mostProbableTaskAllocationStateValuesBeforeProcessing
							.get(i);
					for (Entry<Integer, Integer> e : tasks) {
						// hardcoding this
						// but if the value is not a da value we don't care
						// the hard coding way to do this is to say if the index is greater than the
						// number of tasks
						// don't do it
						// the nicer cleaner way is to use varmapping
						if (e.getKey() < numTasks) {
							int valueInJointState = (int) currentJointState.varValues[e.getKey()];
							if (!(valueInJointState == daInitialStates.get(e.getKey())
									|| valueInJointState == daFinalStates.get(e.getKey()))) {

								// then change it to its initial state
								if (updatedJointState == null)
									updatedJointState = new State(currentJointState);
								updatedJointState.setValue(e.getKey(), daInitialStates.get(e.getKey()));

							}
						}
					}
				}
			}
		}
		return updatedJointState;
	}

	public boolean inStatesExplored(State currentJointState) {
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

	public void saveMDP(MDPSimple mdp, String name) {
		// String saveplace =
		// "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
		//
		// PrismLog log = new PrismFileLog(saveplace + "jointPolicy.dot");
		// mdp.exportToDotFile(log, null, true);
		// log.close();
		StatesHelper.saveMDP(mdp, null, "", name + "jointPolicy", true);
		StatesHelper.saveMDPstatra(mdp, "", name + "jointPolicy", true);
		// StatesHelper.saveHashMap(probsVector, "", "jointPolicyProbs.lab", true);
		// StatesHelper.saveBitSet(allTasksCompletedStates, "", "jointPolicy.acc",
		// true);
		// StatesHelper.saveBitSet(this.allFailStatesSeen, "", "jointPolicy.failstates",
		// true);
		// StatesHelper.saveBitSet(this.deadendStates, "", "jointPolicy.deadends",
		// true);

	}

	public int findStateIndex(State s) {
		int indexInt = StatesHelper.BADVALUE;
		Object index = statesMap.get(s);
		if (index != null) {
			indexInt = (int) index;
		}
		return indexInt;
	}

	private int addStateToMDP(State s, MDPSimple mdp) {
		int index = findStateIndex(s);
		if (index == StatesHelper.BADVALUE) {
			mdp.getStatesList().add(s);
			index = mdp.getNumStates();
			statesMap.put(s, index);
			mdp.addState();

		}
		// // for the first state only
		// // initialize the probs vector
		// if (mdp.getNumStates() == 1) {
		// probsVector.put(index, 1.0);
		// }
		return index;
	}

	private void addTranstionToMDP(MDPSimple mdp, State parentStates, ArrayList<State> states, ArrayList<Double> probs,
			String action, double norm) {
		int parentIndex = addStateToMDP(parentStates, mdp);
		int index;
		Distribution distr = new Distribution();

		for (int succ = 0; succ < states.size(); succ++) {
			index = addStateToMDP(states.get(succ), mdp);
			double normalizedProb = probs.get(succ) / norm;
			distr.add(index, normalizedProb);
			// if (!probsVector.containsKey(index)) {
			// // this can happen so we're kind of ignoring it
			// // the scenario where two different paths end up at the same node
			// // we're just going to stick to the previous probs
			// double probHere = normalizedProb * probsVector.get(parentIndex);
			// probsVector.put(index, probHere);
			// }

		}
		// int actionNo = mdp.getNumChoices(parentIndex);
		mdp.addActionLabelledChoice(parentIndex, distr, action);

	}

	private State getStateFromInd(int stateInd) {
		return this.jointMDP.getStatesList().get(stateInd);
	}

	private Entry<boolean[], Integer> numFailedInJointState(State jointState) {
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

	private boolean hasFailed(int initialStates, MDPSimple mdp) {
		State stateVal = (mdp.getStatesList().get(initialStates));
		for (int i = 0; i < this.isolatedStatesNamesList.size(); i++) {
			int stateIndex = mdp.getVarList().getIndex(this.isolatedStatesNamesList.get(i));
			if ((int) stateVal.varValues[stateIndex] == StatesHelper.failState)
				return true;
		}
		return false;
	}

	private boolean childStateInStateExtended(StateExtended state, int cs) {
		return state.childState == cs;
	}

	private boolean childStateInStateExtendedArray(ArrayList<StateExtended> states, int cs) {
		boolean toret = false;
		for (int i = 0; i < states.size(); i++) {
			if (childStateInStateExtended(states.get(i), cs)) {
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
	private boolean getNewTaskAllocation(ArrayList<ArrayList<StateExtended>> statesDiscovered,
			int[] robotStatesInSeqTeamMDP, boolean followingPath) {
		// so basically check if the robotStatesInSeqTeamMDP are here
		if (statesDiscovered != null) {
			if (!followingPath) {
				boolean newTA = false;
				for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
					if (!childStateInStateExtendedArray(statesDiscovered.get(i), robotStatesInSeqTeamMDP[i])) {
						newTA = true;
						break;
					}
				}
				return newTA;
			} else {
				boolean newTA = false;
				for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
					if (!childStateInStateExtendedArray(statesDiscovered.get(0), robotStatesInSeqTeamMDP[i])) {
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

	private int[] modifyRobotStatesToReflectExpectedTaskCompletion(int[] robotStatesInSeqTeamMDP,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues,
			List<State> statesList, int firstRobot) {
		int[] newStates = (int[]) robotStatesInSeqTeamMDP.clone();
		// For each state
		// get the states values
		// modify them
		// move on
		ArrayList<Entry<Integer, Integer>> indicesToChange = null;
		for (int i = 0; i < robotStatesInSeqTeamMDP.length; i++) {
			int robotNum = StatesHelper.getRobotNumberFromSeqTeamMDPState(statesList.get(robotStatesInSeqTeamMDP[i]));
			if (robotNum != firstRobot) {
				Object[] currentState = (Object[]) statesList.get(robotStatesInSeqTeamMDP[i]).varValues.clone();

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
				newStates[robotNum] = robotStatesInSeqTeamMDP[i];
			}

		}

		return newStates;
	}

	private int[] modifyRobotStatesToUndoExpectedTaskCompletion(int[] robotStatesInSeqTeamMDP,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> mostProbableTaskAllocationStateValues,
			List<State> statesList, int firstRobot) {
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

	protected HashMap<Integer, ArrayList<Entry<Integer, Integer>>> processStateIndexValues(
			ArrayList<ArrayList<StateExtended>> statesDiscovered,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> changedStatesForAllRobots, List<State> statesList) {
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

	protected HashMap<Integer, ArrayList<Entry<Integer, Integer>>> getStateIndexValuesForTaskAllocationForAllRobots(
			ArrayList<ArrayList<StateExtended>> statesDiscovered, List<State> statesList, VarList varList,
			boolean followingPath) {

		StateExtended firstState = statesDiscovered.get(0).get(0);
		State firstStateState = statesList.get(firstState.childState);
		int firstRobot = StatesHelper.getRobotNumberFromSeqTeamMDPState(firstStateState);

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
							changedStateIndices.add(new AbstractMap.SimpleEntry<Integer, Integer>(j,
									(int) lastStateState.varValues[j]));
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
					if ((varList.getName(j) != "r") && changedStates[j] == 1
							&& !this.sharedStatesNamesList.contains(varList.getName(j))
							&& !this.isolatedStatesNamesList.contains(varList.getName(j))) {
						changedStateIndices.add(
								new AbstractMap.SimpleEntry<Integer, Integer>(j, (int) lastStateState.varValues[j]));
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
			ArrayList<ArrayList<StateExtended>> statesDiscovered, List<State> statesList, VarList varList,
			boolean followingPath) {
		StateExtended firstState = statesDiscovered.get(0).get(0);
		State firstStateState = statesList.get(firstState.childState);
		int firstRobot = StatesHelper.getRobotNumberFromSeqTeamMDPState(firstStateState);

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
							changedStateIndices.add(new AbstractMap.SimpleEntry<Integer, Integer>(j,
									(int) firstStateState.varValues[j]));
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
					if ((varList.getName(j) != "r") && changedStates[j] == 1
							&& !this.sharedStatesNamesList.contains(varList.getName(j))
							&& !this.isolatedStatesNamesList.contains(varList.getName(j))) {
						changedStateIndices.add(
								new AbstractMap.SimpleEntry<Integer, Integer>(j, (int) firstStateState.varValues[j]));
					}
				}
				changedStatesForAllRobots.put(robotNum, changedStateIndices);
				firstState = nextState;
				firstStateState = lastStateState;
			}

		}

		return changedStatesForAllRobots;

	}

	protected ArrayList<StateExtended> getTaskAllocationForRobot(MDStrategyArray strat, MDPSimple mdp,
			int initialState) {
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
						Iterator<Entry<Integer, Double>> tranIter = getTranIter(actionChoice.getValue(),
								currentState.childState, mdp);
						ArrayList<Entry<Integer, Double>> succStates = tranIterToArrayList(tranIter);

						for (int i = 0; i < succStates.size(); i++) {
							StateExtended succState = new StateExtended(succStates.get(i).getKey(),
									succStates.get(i).getValue());
							succState.parentState = currentState.childState;

							statesToExploreQ.add(succState);
						}
					} else {
						// if (currentState.action.contains("switch"))
						mainLog.println(statesDiscovered.toString());
						break;
					}
				}
			}

		}
		return statesDiscovered;

	}

	int getFirstFailedRobotFromRobotStates(int[] initialStates, MDPSimple mdp) {
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

	protected ArrayList<ArrayList<StateExtended>> getTaskAllocationForAllRobots(MDStrategyArray strat, MDPSimple mdp,
			int[] initialStates, boolean followingPath) {
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
			statesDiscovered = getTaskAllocationForAllRobotsFollowingPath(strat, mdp, initialStates[firstRobot]);
		}
		return statesDiscovered;

	}

	protected ArrayList<ArrayList<StateExtended>> getTaskAllocationForAllRobotsFollowingPath(MDStrategyArray strat,
			MDPSimple mdp, int initialstate) {

		// we do a BEST cost search
		// we need to keep all the states we've seen
		ArrayList<ArrayList<StateExtended>> statesDiscoveredPlusSwitches = new ArrayList<ArrayList<StateExtended>>();
		ArrayList<StateExtended> endStates = new ArrayList<StateExtended>();
		ArrayList<StateExtended> statesDiscovered = new ArrayList<StateExtended>();
		PriorityQueue<StateExtended> statesToExploreQ = new PriorityQueue<StateExtended>();
		StateExtended currentState = new StateExtended(initialstate, 1.0);
		statesToExploreQ.add(currentState);
		while (!statesToExploreQ.isEmpty()) {
			currentState = statesToExploreQ.remove();
			if (!statesDiscovered.contains(currentState)) {
				statesDiscovered.add(currentState);
				Entry<Object, Integer> actionChoice = getActionChoice(strat, currentState.childState);
				if (actionChoice.getKey() != null) {
					currentState.actionInChildState = actionChoice.getKey().toString();
					if (currentState.actionInChildState != "*") {
						if (currentState.actionInChildState.contains("switch"))
							endStates.add(currentState);
						Iterator<Entry<Integer, Double>> tranIter = getTranIter(actionChoice.getValue(),
								currentState.childState, mdp);
						ArrayList<Entry<Integer, Double>> succStates = tranIterToArrayList(tranIter);

						for (int i = 0; i < succStates.size(); i++) {
							StateExtended succState = new StateExtended(succStates.get(i).getKey(),
									succStates.get(i).getValue());
							succState.parentState = currentState.childState;

							statesToExploreQ.add(succState);
						}
					} else {
						// if (currentState.action.contains("switch"))
						endStates.add(currentState);
						mainLog.println(statesDiscovered.toString());
						break;
					}
				}
			}

		}
		statesDiscoveredPlusSwitches.add(statesDiscovered);
		statesDiscoveredPlusSwitches.add(endStates);
		return statesDiscoveredPlusSwitches;// statesDiscovered;

	}

	protected Entry<Object, Integer> getActionChoice(MDStrategyArray strat, int state) {
		Entry<Object, Integer> actionChoice;
		Object action = null;
		int choice = -1;

		action = strat.getChoiceAction(state);
		choice = strat.getChoiceIndex(state);

		actionChoice = new AbstractMap.SimpleEntry<Object, Integer>(action, choice);
		return actionChoice;
	}

	protected HashMap<Integer, Entry<Object, Integer>> getActionChoiceAllRobots(MDStrategyArray strat, int[] states) {
		HashMap<Integer, Entry<Object, Integer>> actionChoices = new HashMap<Integer, Entry<Object, Integer>>();
		for (int i = 0; i < states.length; i++) {

			Entry<Object, Integer> actionChoice = getActionChoice(strat, states[i]);

			actionChoices.put(states[i], actionChoice);
		}
		return actionChoices;
	}

	protected Entry<String, ArrayList<Entry<int[], Double>>> getActionAndSuccStatesAllRobots(MDStrategyArray strat,
			int[] states, MDPSimple mdp) {
		Entry<String, ArrayList<Entry<int[], Double>>> toret = null;
		HashMap<Integer, Entry<Object, Integer>> actionChoices = getActionChoiceAllRobots(strat, states);
		HashMap<Integer, ArrayList<Entry<Integer, Double>>> succArrs = new HashMap<Integer, ArrayList<Entry<Integer, Double>>>();
		// for each action choice we need to get the successorstates
		String jointAction = "";
		int[] numSuccs = new int[numRobots];
		for (int i = 0; i < states.length; i++) {
			boolean forceNull = false;
			String currentAction = actionChoices.get(states[i]).getKey().toString();
			if (currentAction.contains("switch"))
				forceNull = true;
			int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(mdp.getStatesList().get(states[i]));
			Iterator<Entry<Integer, Double>> tranIter = getTranIter(actionChoices.get(states[i]).getValue(), states[i],
					mdp);
			ArrayList<Entry<Integer, Double>> tranIterList = tranIterToArrayList(tranIter);
			if (forceNull)
				tranIterList = null;
			if (tranIterList == null) {
				// then the list is just that state there aren't any more actions
				tranIterList = new ArrayList<Entry<Integer, Double>>();
				tranIterList.add(new AbstractMap.SimpleEntry(states[i], 1.0));
			}
			succArrs.put(rnum, tranIterList);
			numSuccs[i] = tranIterList.size();
			jointAction = jointAction + "r" + rnum + "_" + currentAction;
			if (i != states.length - 1)
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
			succStateProbList.add(new AbstractMap.SimpleEntry(currentSuccStates.clone(), combProb));
		}
		// normalize here
		for (int i = 0; i < succStateProbList.size(); i++) {
			combProb = succStateProbList.get(i).getValue();
			succStateProbList.get(i).setValue(combProb / sumSuccProb);
		}
		toret = new AbstractMap.SimpleEntry(jointAction, succStateProbList);
		return toret;

	}

	private void generateCombinations(int counter[], int original[], ArrayList<int[]> res) {
		generateCombinations(counter, original, res, original.length - 1);

	}

	private void generateCombinations(int counter[], int original[], ArrayList<int[]> res, int dec_pos) {
		int end_check = 1;
		if (dec_pos == 0) {
			while (counter[dec_pos] != end_check) {
				res.add(counter.clone());
				counter[dec_pos]--;
			}
			res.add(counter.clone());
		} else {
			// res.add(counter.clone());
			generateCombinations(counter, original, res, dec_pos - 1);
			if (counter[dec_pos] != end_check) {
				counter[dec_pos]--;
				for (int i = 0; i < dec_pos; i++) {
					counter[i] = original[i];
				}
				generateCombinations(counter, original, res, dec_pos - 1);
			}
		}

	}

	protected Iterator<Entry<Integer, Double>> getTranIter(int choice, int state, MDPSimple mdp) {
		if (choice > -1)
			return mdp.getTransitionsIterator(state, choice);
		else
			return null;

	}

	protected ArrayList<Entry<Integer, Double>> tranIterToArrayList(Iterator<Entry<Integer, Double>> tranIter) {
		if (tranIter != null) {
			ArrayList<Entry<Integer, Double>> succStatesList = new ArrayList<Entry<Integer, Double>>();

			while (tranIter.hasNext()) {

				succStatesList.add(tranIter.next());
			}
			return succStatesList;
		} else
			return null;

	}

	protected State createJointState(int[] states, List<State> statesList, boolean considerFirstRobotOnly,
			int firstRobot, VarList varlist,
			HashMap<Integer, ArrayList<Entry<Integer, Integer>>> stateValuesBeforeTaskAllocation) {
		if (considerFirstRobotOnly) {
			State stateToRet = new State(this.jointMDP.getVarList().getNumVars());

			// ArrayList<State> robotStates = new ArrayList<State>();
			for (int i = 0; i < states.length; i++) {
				State robotState = statesList.get(states[i]);
				// robotStates.add(robotState);
				int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(robotState);
				if (rnum == firstRobot) {
					// keep this robot's da bits and ss bits
					Object[] ss = StatesHelper.getSharedStatesFromState(robotState, varlist,
							this.sharedStatesNamesList);
					Object[] das = StatesHelper.getDAStatesFromState(robotState, varlist, this.numTasks);
					for (int j = 0; j < das.length; j++) {
						stateToRet.setValue(j, das[j]);

					}
					if (ss != null) {
						for (int j = 0; j < ss.length; j++) {
							stateToRet.setValue(j + das.length, ss[j]);
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

		} else {
			// since we've undone all the changes
			// we just combine the states
			State stateToRet = new State(this.jointMDP.getVarList().getNumVars());
			// start from the first robot
			// xor then modify
			int lastRobot = numRobots - (firstRobot + 1); // check if this is the right thing
			State robotState = statesList.get(states[firstRobot]);
			ArrayList<Object[]> robotStateElements = StatesHelper.getDAandSharedStatesFromState(robotState, varlist,
					sharedStatesNamesList, numTasks);
			Object[] das = robotStateElements.get(0);
			Object[] ss = robotStateElements.get(1);

			for (int j = 0; j < das.length; j++) {
				stateToRet.setValue(j, das[j]);

			}
			if (ss != null) {
				for (int j = 0; j < ss.length; j++) {
					if (ss[j] != null)
						stateToRet.setValue(j + das.length, ss[j]);
				}
			}
			Object[] rs = StatesHelper.getMDPStateFromState(robotState, varlist, isolatedStatesNamesList);
			for (int j = 0; j < rs.length; j++) {
				stateToRet.setValue(varListMapping.get("r" + firstRobot) + j, rs[j]);
			}
			int robotCount = firstRobot;
			do {

				int nextRobot = (robotCount + 1) % numRobots;
				State nextRobotState = statesList.get(states[nextRobot]);
				ArrayList<Object[]> nextRobotStateElements = StatesHelper.getDAandSharedStatesFromState(nextRobotState,
						varlist, sharedStatesNamesList, numTasks);
				Object[] nextRobotDAs = nextRobotStateElements.get(0);
				Object[] nextRobotSS = nextRobotStateElements.get(1);
				for (int j = 0; j < das.length; j++) {
					// this says only change this value if there has been any progress
					// for sequential states progress is not limited to the initial state
					// so there has to be a better way to say this.
					// only change this value if it is different from the other robots
					// and it is not an initial state and it is not the same as the previous value
					// of this robot
					if ((int) stateToRet.varValues[j] != (int) nextRobotDAs[j]
							&& (int) nextRobotDAs[j] != daInitialStates.get(j)) {
						if (stateValuesBeforeTaskAllocation != null) {
							if (stateValuesBeforeTaskAllocation.containsKey(nextRobot)) {
								boolean containsJ = false;
								ArrayList<Entry<Integer, Integer>> stateValsForNextRobot = stateValuesBeforeTaskAllocation
										.get(nextRobot);
								for (int k = 0; k < stateValsForNextRobot.size(); k++) {
									if ((int) stateValsForNextRobot.get(k).getKey() == (j + 1)) {
										containsJ = true;
										if (stateValsForNextRobot.get(k).getValue() != nextRobotDAs[j]) {
											stateToRet.setValue(j, nextRobotDAs[j]);

										}
										break;
									}

								}
								// if (!containsJ)
								// stateToRet.setValue(j, nextRobotDAs[j]);
							}
							// else
							// stateToRet.setValue(j, nextRobotDAs[j]);
						} else
							stateToRet.setValue(j, nextRobotDAs[j]);
					}
				}
				if (ss != null) {
					for (int j = 0; j < ss.length; j++) {
						if (ss[j] != null) {

							if (((int) stateToRet.varValues[j + das.length]) != (int) nextRobotSS[j]
									&& (int) nextRobotSS[j] != sharedVarsInitialStates
											.get(this.sharedStatesNamesList.get(j))) {
								stateToRet.setValue(j + das.length, nextRobotSS[j]);
							}
						}
					}
				}
				// we compare the da and shared states
				rs = StatesHelper.getMDPStateFromState(nextRobotState, varlist, isolatedStatesNamesList);
				for (int j = 0; j < rs.length; j++) {
					stateToRet.setValue(varListMapping.get("r" + ((robotCount + 1) % numRobots)) + j, rs[j]);
				}
				robotCount = (robotCount + 1) % numRobots;

			} while (robotCount != lastRobot);
			return stateToRet;
		}
		// return null;

	}

	// FIXME: a lot of hardcoding here which is not required at all
	// so i'm just being lazy af
	protected int[] extractIndividualRobotStatesFromJointState(State jointState, List<State> teamMDPStatesList,
			VarList teamdpvarlist) {
		Object[] ss = StatesHelper.getSharedStatesFromState(jointState, this.jointMDP.getVarList(),
				this.sharedStatesNamesList);
		Object[] das = StatesHelper.getDAStatesFromState(jointState, this.jointMDP.getVarList(), this.numTasks);

		int numVarsInTeamMDP = teamdpvarlist.getNumVars();
		int[] statesToRet = new int[this.numRobots];
		Arrays.fill(statesToRet, StatesHelper.BADVALUE);

		// create a joint state for each robot
		for (int i = 0; i < this.numRobots; i++) {
			Object[] newState = new Object[numVarsInTeamMDP];
			newState[teamdpvarlist.getIndex("r")] = i;

			for (int j = 0; j < das.length; j++)
				newState[teamdpvarlist.getIndex("da" + j)] = das[j];
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
				newState[teamdpvarlist.getIndex(this.isolatedStatesNamesList.get(j))] = StatesHelper
						.getIndexValueFromState(jointState, varListMapping.get("r" + i));
			// i just want to say that my code has been better
			int sameState = StatesHelper.getExactlyTheSameState(newState, teamMDPStatesList);
			statesToRet[i] = sameState;
		}
		return statesToRet;
	}

	public boolean hasFailedStates() {
		return !failedStatesQueue.isEmpty();
	}

	public State getNextFailedState() {
		StateExtended state = failedStatesQueue.remove();
		this.currentStateProbability = state.parentToChildTransitionProbability;
		State toret = jointMDP.getStatesList().get(state.childState);
		return toret;
	}

	public void printStatesExploredOrder() {

		for (int i = 0; i < statesExploredOrder.size(); i++) {
			this.mainLog.println(i + 1 + ":" + statesExploredOrder.get(i).toString());
		}

	}

}
