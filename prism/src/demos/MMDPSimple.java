package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import acceptance.AcceptanceReach;

import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import java.util.Vector;

import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import prism.PrismException;
import prism.PrismLangException;
import prism.PrismLog;
import strat.MDStrategyArray;
import strat.Strategy;
import explicit.Distribution;
import explicit.MDPSimple;

//import explicit.stateInfoFromStatesList;

/*
 * A class that stores the joint policy for multiple agents 
 */
public class MMDPSimple {

	public class StateProb implements Comparable<StateProb> {
		private double prob;
		private int state;
		private String action;

		public StateProb(int state, double prob) {
			this.prob = prob;

			this.state = state;
			this.action = null;
		}

		public StateProb(int state, double prob, String action) {
			this.prob = prob;

			this.state = state;
			this.action = action;
		}

		public StateProb(StateProb other) {
			this.prob = other.prob;
			this.state = other.state;
			this.action = other.action;
		}

		@Override
		public int compareTo(StateProb other) {
			double comp = this.prob - other.prob;
			int res = 0;
			if (comp > 0)
				res = -1;
			else {
				if (comp < 0)
					res = 1;
			}
			return res;
		}

		public StateProb copy() {
			return new StateProb(this);
		}

		public double getProb() {
			return this.prob;
		}

		public int getState() {
			return this.state;
		}

		public String getAction() {
			return this.action;
		}

		@Override
		public String toString() {
			if (action == null)
				return "[s=" + state + ", p=" + prob + "]";
			else
				return "[s=" + state + ", p=" + prob + ", a=" + action + "]";
		}

	}

	public MDPSimple mdp;
	// ArrayList<Map.Entry<State, Integer>>
	HashMap<State, Integer> statesMap;
	HashMap<Integer, Double> probsVector;
	int nRobots;
	public PriorityQueue<StateProb> stuckStatesQ;
	public BitSet deadendStates;
	public BitSet allTasksCompletedStates;
	public BitSet allFailStatesSeen;
	BitSet initStates;
	ArrayList<String> sharedVarsList;
	ArrayList<String> notSharedVarsList; // just to make life easier
	int numDA;
	VarList teamMDPVarlist;
	PrismLog mainLog;

	/**
	 * creates a joint policy MDP
	 * 
	 * @numrobots the number of robots
	 * 
	 */
	public MMDPSimple(int numrobots, int numDAs, ArrayList<String> sharedVarsList, VarList seqTeamMDPVarlist,
			PrismLog ml) {
		mainLog = ml;
		stuckStatesQ = new PriorityQueue<StateProb>();
		mdp = new MDPSimple();
		this.sharedVarsList = sharedVarsList;
		notSharedVarsList = new ArrayList<String>(); // this is for now
		for (int i = 0; i < seqTeamMDPVarlist.getNumVars(); i++) {
			String name = seqTeamMDPVarlist.getName(i);
			if (!sharedVarsList.contains(name) && (!name.contains("da")) && (name != "r"))
				notSharedVarsList.add(name);
		}

		int numSharedVars = this.sharedVarsList.size();
		int numSeqTeamMDPVars = seqTeamMDPVarlist.getNumVars();
		this.teamMDPVarlist = seqTeamMDPVarlist;

		VarList varlist = new VarList();

		try {
			for (int i = numrobots - 1; i >= 0; i--) {

				varlist.addVar(0, new Declaration("r" + i, new DeclarationIntUnbounded()), 1, null);

			}
			for (int i = 0; i < numSharedVars; i++) {
				varlist.addVar(0, new Declaration(sharedVarsList.get(i), new DeclarationIntUnbounded()), 1, null);
			}
			// varlist.addVar(0, new Declaration("da", new DeclarationIntUnbounded()), 1,
			// null);
			for (int i = 0; i < numDAs; i++) {
				varlist.addVar(0, new Declaration("da" + i, new DeclarationIntUnbounded()), 1, null);
			}
		} catch (PrismLangException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		mdp.setVarList(varlist);
		mdp.setStatesList(new ArrayList<State>());
		statesMap = new HashMap<State, Integer>();
		nRobots = numrobots;
		deadendStates = new BitSet();
		allTasksCompletedStates = new BitSet();
		allFailStatesSeen = new BitSet();
		setNumDA(numDAs);
		probsVector = new HashMap<Integer, Double>();

	}

	public class switchStateInfo {
		public int pstate;
		public int crnum;
		public int cstate;
		public double prob;
		private int prnum;

		public switchStateInfo(int ps, int r, int s, double p) {
			pstate = ps;
			crnum = r;
			cstate = s;
			prob = p;
		}

		public switchStateInfo(int ps, int r, int s, double p, int prnum) {
			pstate = ps;
			crnum = r;
			cstate = s;
			prob = p;
			this.prnum = prnum;
		}

		@Override
		public String toString() {
			return " [ps:" + pstate + ", r:" + crnum + ", s:" + cstate + ", " + prob + "]";
		}

		public void setParentRobotNum(int parentRobot) {
			prnum = parentRobot;
		}

		public int getParentRobotNum() {
			return prnum;
		}

	}

	// https://stackoverflow.com/questions/8119366/sorting-hashmap-by-values
	public LinkedHashMap<String, Double> sortHashMapByValues(HashMap<String, Double> passedMap, boolean highestFirst) {
		List<String> mapKeys = new ArrayList<>(passedMap.keySet());
		List<Double> mapValues = new ArrayList<>(passedMap.values());
		if (!highestFirst) {
			Collections.sort(mapValues);
			Collections.sort(mapKeys);
		} else {
			Collections.sort(mapValues, Collections.reverseOrder());
			Collections.sort(mapKeys, Collections.reverseOrder());

		}
		LinkedHashMap<String, Double> sortedMap = new LinkedHashMap<>();

		Iterator<Double> valueIt = mapValues.iterator();
		while (valueIt.hasNext()) {
			Double val = valueIt.next();
			Iterator<String> keyIt = mapKeys.iterator();

			while (keyIt.hasNext()) {
				String key = keyIt.next();
				Double comp1 = passedMap.get(key);
				Double comp2 = val;

				if (comp1.equals(comp2)) {
					keyIt.remove();
					sortedMap.put(key, val);
					break;
				}
			}
		}
		return sortedMap;
	}

	/***
	 * createJointPolicyFromSequentialSolution a work in progress creates a joint
	 * policy from the sequential solution
	 * 
	 * @param strat
	 *            - the sequential solution
	 * @param initState
	 *            - the initial state of the seqTeamMDP
	 * @param seqTeamMDP
	 *            - the MDP whose solution is in strat
	 * @param initialStateIsNotJointState
	 *            - if it is not , we have to create one
	 */
	public void createJointPolicyFromSequentialSolution(MDStrategyArray strat, int initialStateInSeqTeamMDP,
			SequentialTeamMDP seqTeamMDP, boolean initialStateIsNotJointState) {
		
		List<State> states = null;
		int[] currentRobotStates = null;
		State currentState = null;
		int firstRobotNumber = -1;
		if (initialStateIsNotJointState) {
			states = seqTeamMDP.teamMDPWithSwitches.getStatesList();
			currentState = states.get(initialStateInSeqTeamMDP);
			firstRobotNumber = StatesHelper.getRobotNumberFromSeqTeamMDPState(currentState);
			currentRobotStates = new int[nRobots];
			currentRobotStates[firstRobotNumber] = initialStateInSeqTeamMDP;

			for (int i = 0; i < nRobots; i++) {
				if (i != firstRobotNumber) {
					currentRobotStates[i] = seqTeamMDP.initialStates.get(i).nextSetBit(0);
				}
			}
		}
		// do {
		ArrayList<StateProb> mostProbablePath = findMostProbablePath(strat, initialStateInSeqTeamMDP, seqTeamMDP);
		// get the state that switches to a new robot
		ArrayList<switchStateInfo> switchInfo = getSwitchStateInfoFromPath(mostProbablePath,
				seqTeamMDP.teamMDPWithSwitches.getStatesList());
		int lastState = mostProbablePath.get(mostProbablePath.size() - 1).getState();
		// from the switch info get the task allocations
		ArrayList<ArrayList<Integer>> taskAllocation = findTaskAllocation(switchInfo, initialStateInSeqTeamMDP,
				lastState, seqTeamMDP.teamMDPWithSwitches.getStatesList());

//		for (int ar = 0; ar < taskAllocation.size(); ar++)
//			mainLog.println(ar + ":" + taskAllocation.get(ar).toString());

		// Arrays.toString(taskAllocation));
		HashMap<Integer, State> robotTaskStates = new HashMap<Integer, State>();
		// List<State> robotTaskStates = new ArrayList<State>();
		// we have to create new robot states
		int rnum = firstRobotNumber;
		int switchNumber = 0;
		do {
			if (switchNumber < switchInfo.size()) {
				robotTaskStates.put(rnum, states.get(switchInfo.get(switchNumber).pstate));
			} else
				robotTaskStates.put(rnum, states.get(lastState));
			switchNumber++;

			rnum = (rnum + 1) % nRobots;
		} while (rnum != firstRobotNumber);

		Queue<int[]> robotStatesList = new LinkedList<int[]>();
		Queue<State> jointStatesList = new LinkedList<State>();
		robotStatesList.add(currentRobotStates.clone());
		while (!robotStatesList.isEmpty()) {
			currentRobotStates = robotStatesList.remove();

			// TODO: marker this is where we check for failure of the states
			// if we know that the state has failed we just add it and do nothing else.

			// testing the querying bit
			// create new states
			List<Integer> updatedCurrentRobotStates = new ArrayList<Integer>();

			for (int i = 0; i < nRobots; i++) {

				State currentRobotStateState = states.get(currentRobotStates[i]);
				int newStateNumber;
				if (i != firstRobotNumber) {
					State taskAllocatedState = robotTaskStates.get((i - 1) % nRobots);
					// combined state
					Object[] newState = StatesHelper.getMergedState(currentRobotStateState, taskAllocatedState, 1,
							1 + numDA); // hardcoding this
					newStateNumber = StatesHelper.getExactlyTheSameState(newState, states);

				} else {
					newStateNumber = currentRobotStates[i];// initialStateInSeqTeamMDP;
				}
				updatedCurrentRobotStates.add(newStateNumber);

			}

			ArrayList<ArrayList<Entry<Integer, Double>>> robotStatesIteratorList = new ArrayList<ArrayList<Entry<Integer, Double>>>();
			int[] numSuccessors = new int[nRobots];
//			mainLog.println(updatedCurrentRobotStates.toString());
			String jointAction = "";
			Object action;
			for (int i = 0; i < nRobots; i++) {


				
				action = getActionFromStrat(strat, updatedCurrentRobotStates.get(i));
				ArrayList<Entry<Integer, Double>> nextStates = findNextState(updatedCurrentRobotStates.get(i), action,
						seqTeamMDP.teamMDPWithSwitches);
				numSuccessors[i] = nextStates.size();
				if (numSuccessors[i] == 0) {
					numSuccessors[i] = 1;
					nextStates = new ArrayList<Entry<Integer, Double>>();
					nextStates.add(new AbstractMap.SimpleEntry<Integer, Double>(updatedCurrentRobotStates.get(i), 1.0));
				}
				robotStatesIteratorList.add(nextStates);
				jointAction = jointAction + "_r" + i + "_" + action.toString();

			}
//			mainLog.println(jointAction);

			// i got this
			// everything is alright
			// now we have to make combinations

			ArrayList<int[]> combinations = new ArrayList<int[]>();
			generateCombinations(numSuccessors.clone(), numSuccessors.clone(), combinations, numSuccessors.length - 1);

			int[] combinationStates = new int[nRobots];
			ArrayList<int[]> allCombinationStates = new ArrayList<int[]>();
			ArrayList<Double> probs = new ArrayList<Double>();
			double sumFinalProb = 0;
			for (int[] combination : combinations) {
				double finalProb = 1.0;
				// lets make a combination
				for (int r = 0; r < combination.length; r++) {
					Entry<Integer, Double> stateProb = (robotStatesIteratorList.get(r)).get(combination[r] - 1);
					combinationStates[r] = stateProb.getKey();
					double prob = stateProb.getValue();
					finalProb = prob * finalProb;
				}
				allCombinationStates.add(combinationStates.clone());
				robotStatesList.add(combinationStates.clone());
				probs.add(finalProb);
				sumFinalProb = sumFinalProb + finalProb;
			}
			// yay done with step one
			// now we recycle
			// skipping the joint state stuff
			// cuz something is wrong with me

			// }while(true);//final joint state does not have all tasks completed or we
			// haven't gotten to a state thats like the last switch ?
			// or joint action is empty or all * you know
			// I really dont know how long to do this for
			// lets make a queue
		}
	}

	public Object getActionFromStrat(Strategy strat, int state) {
		strat.initialise(state);
		return strat.getChoiceAction();
	}

	public ArrayList<Entry<Integer, Double>> findNextState(int currentState, Object action, MDPSimple mdp) {
		ArrayList<Entry<Integer, Double>> nextStates = new ArrayList<Entry<Integer, Double>>();
		Iterator<Entry<Integer, Double>> stateIter = null;
		int numChoices = mdp.getNumChoices(currentState);
		int actionChoice = -1;
		for (int i = 0; i < numChoices; i++) {
			if (mdp.getAction(currentState, i).equals(action)) {
				actionChoice = i;
				break;
			}
		}
		if (actionChoice != -1) {
			stateIter = mdp.getTransitionsIterator(currentState, actionChoice);

			while (stateIter.hasNext()) {
				// Entry<Integer, Double> stateProb = stateIter.next();
				nextStates.add(stateIter.next());
			}
		}
		return nextStates;

	}

	public ArrayList<ArrayList<Integer>> findTaskAllocation(ArrayList<switchStateInfo> switchInfo, int initialState,
			int lastState, List<State> statesList) {
		ArrayList<ArrayList<Integer>> taskAllocation = new ArrayList<ArrayList<Integer>>();

		int taskAllocationArray[] = new int[numDA];
		Arrays.fill(taskAllocationArray, StatesHelper.BADVALUE);
		int prevRobotState = initialState;
		int thisRobotState;
		int rnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(statesList.get(initialState));
		for (int i = 0; i < switchInfo.size() + 1; i++) {
			if (i < switchInfo.size())
				thisRobotState = switchInfo.get(i).cstate;
			else
				thisRobotState = lastState;
			int[] integerXORmask = StatesHelper.XORIntegers(statesList.get(prevRobotState).varValues,
					statesList.get(thisRobotState).varValues);
			for (int j = 0; j < numDA; j++) {
				if (integerXORmask[j + 1] == 1) {
					if (taskAllocationArray[j] != StatesHelper.BADVALUE)
						mainLog.println("Overwriting previous task allocation, issue here");
					taskAllocationArray[j] = rnum;

					while (taskAllocation.size() <= rnum) {
						taskAllocation.add(new ArrayList<Integer>());
					}
					taskAllocation.get(rnum).add(j);

				}
			}

			prevRobotState = thisRobotState;
			if (i < switchInfo.size())
				rnum = switchInfo.get(i).crnum;
		}

		return taskAllocation;

	}

	public ArrayList<switchStateInfo> getSwitchStateInfoFromPath(ArrayList<StateProb> path, List<State> stateList) {
		String actionToLookFor = "switch";
		ArrayList<switchStateInfo> switchStateInfoList = new ArrayList<switchStateInfo>();

		for (int i = 0; i < path.size(); i++) {
			if (path.get(i).getAction().contains(actionToLookFor)) {
				int pstate = path.get(i).getState();
				int cstate = path.get(i + 1).getState();
				double prob = path.get(i).getProb();
				int crnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(stateList.get(cstate));
				int prnum = StatesHelper.getRobotNumberFromSeqTeamMDPState(stateList.get(pstate));
				switchStateInfoList.add(new switchStateInfo(pstate, crnum, cstate, prob, prnum));

			}
		}
		return switchStateInfoList;

	}

	public ArrayList<StateProb> findMostProbablePath(Strategy strat, int initialStateForStrategy,
			SequentialTeamMDP seqTeamMDP) {
		ArrayList<StateProb> path = new ArrayList<StateProb>();
		int currentState = initialStateForStrategy;
		int nextState = -1;
		Object action;

		strat.initialise(currentState);
		action = strat.getChoiceAction();
		path.add(new StateProb(currentState, 1.0, action.toString()));
		while (action != null && action.toString() != "*") {
			int tranIterNum = -1;
			int numActionChoices = seqTeamMDP.teamMDPWithSwitches.getNumChoices(currentState);
			for (int choice = 0; choice < numActionChoices; choice++) {
				if (seqTeamMDP.teamMDPWithSwitches.getAction(currentState, choice) == action) {
					tranIterNum = choice;
					break;
				}
			}
			if (tranIterNum != -1) {
				double maxProb = 0;
				Iterator<Entry<Integer, Double>> tranIter = seqTeamMDP.teamMDPWithSwitches
						.getTransitionsIterator(currentState, tranIterNum);
				while (tranIter.hasNext()) {
					Entry<Integer, Double> stateProbPair = tranIter.next();
					if (stateProbPair.getValue() > maxProb) {
						nextState = stateProbPair.getKey();
					}

				}

				currentState = nextState;
				strat.initialise(currentState);
				action = strat.getChoiceAction();
				path.add(new StateProb(currentState, maxProb, action.toString()));
			}

			else {
				mainLog.println("No such action - " + action.toString());
				break;
			}

		}
		return path;
	}

	public ArrayList<Integer> breakSeqPolicyAndAddtoPolicy(SequentialTeamMDP seqTeamMDP, Strategy strat,
			int initialState, int nextSuccState, int[] allRobotInitStates, boolean synchronizationpoints,
			boolean simulateOnly) {
		boolean debugStuff = true;

		mainLog.println("Breaking Seq Policy to MDPs, initialState in teamMDP: " + initialState);
		// if(initialState == 1104)
		// mainLog.println("Lets start debugging here");
		// attempting to label all the paths so that I can break them down
		HashMap<String, Vector<BitSet>> allPathStates = new HashMap<String, Vector<BitSet>>(); // each bitset represents
		HashMap<Integer, Vector<BitSet>> allStateNumbers = new HashMap<Integer, Vector<BitSet>>(); // labeling states

		// a different path
		HashMap<String, Double> allPathProbs = new HashMap<String, Double>();
		ArrayList<String> acceptingPaths = new ArrayList<String>();
		ArrayList<String> failingPaths = new ArrayList<String>();
		ArrayList<String> allOtherPaths = new ArrayList<String>();
		MDPSimple tempMDP = null;
		int[] tempMDPMap = null;
		BitSet tempAccStates = null;
		boolean actionFound = false;
		MDPSimple sumprod = seqTeamMDP.teamMDPWithSwitches;
		int numRobots = seqTeamMDP.agentMDPs.size();
		// n MDP stuff
		MDPSimple mdps[] = new MDPSimple[numRobots];
		int mdpMaps[][] = new int[numRobots][sumprod.getNumStates()];
		BitSet accStates[] = new BitSet[numRobots];
		// <state,child_state_robot,child_state>
		ArrayList<ArrayList<switchStateInfo>> robotLastStateInitStatePairs = new ArrayList<ArrayList<switchStateInfo>>();
		double probsArr[] = new double[sumprod.getNumStates()];
		Vector<BitSet> pathStates = new Vector<BitSet>();
		for (int i = 0; i < numRobots; i++) {
			robotLastStateInitStatePairs.add(new ArrayList<switchStateInfo>());
			pathStates.add(new BitSet());
			mdps[i] = new MDPSimple();
			mdps[i].setVarList((VarList) sumprod.getVarList().clone());
			Arrays.fill(mdpMaps[i], StatesHelper.BADVALUE);
			mdps[i].setStatesList(new ArrayList<State>());
			accStates[i] = new BitSet();
		}
		allPathStates.put("0", pathStates);
		allPathProbs.put("0", 1.0);
		if (debugStuff) {
			tempMDP = new MDPSimple();
			tempMDPMap = new int[sumprod.getNumStates()];
			Arrays.fill(tempMDPMap, StatesHelper.BADVALUE);
			tempAccStates = new BitSet();
			tempMDP.setVarList((VarList) sumprod.getVarList().clone());
			tempMDP.setStatesList(new ArrayList<State>());
		}
		// n MDP stuff done
		ArrayList<Integer> statesDiscovered = new ArrayList<Integer>();
		int statesForPolicy = 0;

		int choices = -1;

		Object action = null;

		// this shouldnt matter
		List<Integer> discoveredStates = new LinkedList<Integer>();
		// on second thought this does not matter (the thing below) because we might
		// visit the same state again but it may be a child of another state
		// and so the initial state of the robot would have changed (though maybe no new
		// task would be achieved)
		// BitSet addedToStuckQ = new BitSet(tAutomaton.numStates); //for each policy
		// unfolding we don't need to add the same state over and over again to the
		// stuck stack

		Stack<Integer> statesStack = new Stack<Integer>(); // to store all the states we need to explore, so dfs
															// to the goal , well its not dfs but yeah
		Stack<Integer> stateNumStack = new Stack<Integer>();

		int currState = initialState;
		int firstRobot = StatesHelper.getRobotNumberFromSeqTeamMDPState(sumprod.getStatesList().get(currState));
		int currStateNum = 0;
		Stack<String> pathStack = new Stack<String>();
		statesStack.push(currState);
		pathStack.push("0");
		probsArr[currState] = 1.0;
		String pathnum = "0";
		stateNumStack.push(0);
		// TODO: what do you do when the policy has been computed before and we're just
		// here again ?
		// Do we care about this here or later ?

		// String prevTextToAdd = "";
		// int timecount = 0;
		while (!statesStack.isEmpty()) // the main bit - go to the next state, get the action go to the next state and
										// so on
		{

			// TODO: addstatetopolmdp(currState.state)
			// TODO: distribution
			currState = statesStack.pop();
			currStateNum = stateNumStack.pop();
			pathnum = pathStack.pop();
			strat.initialise(currState);
			action = strat.getChoiceAction();
			actionFound = false;
			// boolean linkSwitch = false;
			int currentRobot = StatesHelper.getRobotNumberFromSeqTeamMDPState(sumprod.getStatesList().get(currState));
			if (!allStateNumbers.containsKey(currStateNum)) {
				Vector<BitSet> tempv = new Vector<BitSet>();
				for (int r = 0; r < nRobots; r++)
					tempv.add(new BitSet());
				allStateNumbers.put(currStateNum, tempv);
			}

			statesForPolicy++;
			// just making sure we have enough elements in the array

			if (!discoveredStates.contains(currState)
					// !discovered.get(currState.state)
					& !seqTeamMDP.acceptingStates.get(currState)) // if it is not an accepting state and not
																	// discovered explore
			{

				// mainLog.println(currState.toString());
				// discovered.set(currState.state);
				discoveredStates.add(currState);
				choices = sumprod.getNumChoices(currState);

				for (int c = 0; c < choices; c++) {
					if (action.equals(sumprod.getAction(currState, c))) {
						// TODO: the prodDistr stuff
						ArrayList<Integer> states = new ArrayList<Integer>();
						ArrayList<Double> probs = new ArrayList<Double>();
						// TODO: adding to the MDP policy
						Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(currState, c);
						while (iter.hasNext()) {
							Entry<Integer, Double> stateProbPair = iter.next();
							// get which robot
							int nextState = stateProbPair.getKey();
							double nextStateProb = stateProbPair.getValue();

							statesStack.push(nextState);
							states.add(nextState);
							probs.add(nextStateProb);
						}

						accStates[currentRobot] = StatesHelper.addLinkInMDP(mdps[currentRobot], mdpMaps[currentRobot],
								sumprod.getStatesList(), states, probs, currState, action, "r" + currentRobot,
								accStates[currentRobot], seqTeamMDP.acceptingStates);

						allPathStates.get(pathnum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
						allStateNumbers.get(currStateNum).get(currentRobot).set(mdpMaps[currentRobot][currState]);

						boolean remove_old_path = false;
						// adding probs
						for (int ns = 0; ns < states.size(); ns++) {
							int nextState = states.get(ns);
							double probsNext = probs.get(ns);
							probsNext = probsNext * probsArr[currState];
							probsArr[nextState] = probsNext;
							String tempPathnum = pathnum;
							if (states.size() > 1) {
								tempPathnum = pathnum + ns;
								remove_old_path = true;
							}
							pathStack.push(tempPathnum);

							if (!allPathStates.containsKey(tempPathnum))

							{
								Vector<BitSet> tempv = new Vector<BitSet>();
								for (int i = 0; i < numRobots; i++) {
									tempv.add((BitSet) allPathStates.get(pathnum).get(i).clone());
								}

								allPathStates.put(tempPathnum, tempv);

							}

							double tempProb = allPathProbs.get(pathnum) * probs.get(ns);
							allPathProbs.put(tempPathnum, tempProb);
							if (!action.toString().contains("switch")) {
								stateNumStack.push(currStateNum + 1);
							} else {
								int nextStateRobot = StatesHelper
										.getRobotNumberFromSeqTeamMDPState(sumprod.getStatesList().get(nextState));
								int nextRobotStackNum = allPathStates.get(pathnum).get(nextStateRobot).cardinality();
								// if ( nextRobotStackNum > 0)
								stateNumStack.push(nextRobotStackNum);
								if (nextRobotStackNum > 0) {
									// linkSwitch = true;
									// link this state to the last state we have on this path for this robot
									// tricky stuff
									// accStates[currentRobot] = StatesHelper.addLinkInMDP(mdps[currentRobot],
									// mdpMaps[currentRobot],
									// sumprod.getStatesList(), states, probs, currState, action, "r" +
									// currentRobot,
									// accStates[currentRobot], seqTeamMDP.acceptingStates);
									BitSet possibleLastStates = (BitSet) allStateNumbers
											.get(allPathStates.get(pathnum).get(nextStateRobot).cardinality() - 1)
											.get(nextStateRobot).clone();
									possibleLastStates.and(allPathStates.get(pathnum).get(nextStateRobot));

									int lastStateOnPath = possibleLastStates.nextSetBit(0);
									// we have to get this states successor actually

									// accStates[nextStateRobot]=StatesHelper.addLinkInMDP(mdps[nextStateRobot],
									// mdpMaps[nextStateRobot],
									// sumprod.getStatesList(), states, probs, lastStateOnPath, action,
									// "r"+nextStateRobot, accStates[nextStateRobot],
									// seqTeamMDP.acceptingStates,true);

								}
							}
						}
						if (remove_old_path) {
							allPathStates.remove(pathnum);
							allPathProbs.remove(pathnum);
						}
						// need to fix this really
						// there can be multiple initial states because we have multiple switch actions
						// we also want to track the state with the higher probability - so this is a
						// bit of a problem right now
						if (action.toString().contains("switch")) {
							// cycle through the states and get initial states
							for (int nextState : states) {
								int rindex = StatesHelper
										.getRobotNumberFromSeqTeamMDPState(sumprod.getStatesList().get(nextState));
								mdps[rindex].addInitialState(StatesHelper.addStateMDP(mdps[rindex], mdpMaps[rindex],
										sumprod.getStatesList(), nextState));
								switchStateInfo ssi = new switchStateInfo(mdpMaps[currentRobot][currState], rindex,
										mdpMaps[rindex][nextState], probsArr[nextState]);
								robotLastStateInitStatePairs.get(currentRobot).add(ssi);
							}
						}
						if (mdps[currentRobot].getNumInitialStates() == 0)
							mdps[currentRobot].addInitialState(0);
						if (debugStuff) {
							tempAccStates = StatesHelper.addLinkInMDP(tempMDP, tempMDPMap, sumprod.getStatesList(),
									states, probs, currState, action, "", tempAccStates, seqTeamMDP.acceptingStates);

						}
						actionFound = true;
						// addLinkInPolicyMDP(states,probs, currState.state, action);
						break;
						// don't go over all choices - actions are unique
					}
				}
				if (!actionFound) {

					if (StatesHelper.isFailState(sumprod.getStatesList().get(currState))) {
						// huh ?
						failingPaths.add(pathnum);
					} else {
						allOtherPaths.add(pathnum);
					}
					if (mdpMaps[currentRobot][currState] != StatesHelper.BADVALUE) {
						allPathStates.get(pathnum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
						allStateNumbers.get(currStateNum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
					}
				}

			} else {
				if (mdpMaps[currentRobot][currState] != StatesHelper.BADVALUE) {
					allPathStates.get(pathnum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
					allStateNumbers.get(currStateNum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
				}
				if (seqTeamMDP.acceptingStates.get(currState))
					acceptingPaths.add(pathnum);
				else {
					if (StatesHelper.isFailState(sumprod.getStatesList().get(currState))) {
						// huh ?
						failingPaths.add(pathnum);
					} else
						allOtherPaths.add(pathnum);
				}
			}

		}
		// once we're done add the current policy to the big policy
		// and also check if the policy has at least init values for all robots
		// if(allRobotInitStates != null)
		// checkCurrentPolicy(allRobotInitStates,rts);
		if (debugStuff) {
			String stratName = "tempStrat" + initialState;
			if (simulateOnly)
				stratName = "tempStratsim" + initialState;
			StatesHelper.saveMDP(tempMDP, tempAccStates, "", stratName, true);
			StatesHelper.checkMDPs(mdps, mdpMaps, sumprod.getStatesList(), allRobotInitStates, initialState, accStates,
					seqTeamMDP.acceptingStates);
		}
		// constructJointPolicyFromMDPsAndAddToCurrentPolicy(mdps,mdpMaps,firstRobot,accStates,seqTeamMDP,indexOfRobotThatFailedLast);
		// constructJointPolicyFromMDPsAndAddToCurrentPolicy_new(mdps, mdpMaps,
		// firstRobot, accStates, seqTeamMDP);

		if (!simulateOnly) {
			// printing stuff
//			for (String key : allPathStates.keySet()) {
//				mainLog.println(key + ":" + allPathStates.get(key) + " - " + allPathProbs.get(key));
//			}
//			for (int i = 0; i < robotLastStateInitStatePairs.size(); i++) {
//				for (int j = 0; j < robotLastStateInitStatePairs.get(i).size(); j++) {
//					mainLog.println(i + "->" + robotLastStateInitStatePairs.get(i).get(j));
//				}
//			}

			String firstPath = getMostProbablePath(allPathProbs, acceptingPaths, allOtherPaths, failingPaths);
			// check initial allStateNumbers
			for (int r = 0; r < nRobots; r++) {
				if (allStateNumbers.get(0).get(r).cardinality() == 0) {
					if (allPathStates.get(firstPath).get(r).nextSetBit(0) != -1)
						allStateNumbers.get(0).get(r).set(allPathStates.get(firstPath).get(r).nextSetBit(0));
					else
						allStateNumbers.get(0).get(r).set(mdps[r].getFirstInitialState());
				}
			}
			convertMDPsToJointPolicy(mdps, mdpMaps, firstRobot, accStates, seqTeamMDP, synchronizationpoints,
					allPathStates, firstPath, strat, allStateNumbers, allPathProbs, acceptingPaths, allOtherPaths);
		}
		statesDiscovered.add(statesForPolicy);

		return statesDiscovered;

	}

	private String getMostProbablePath(HashMap<String, Double> allPathProbs, ArrayList<String> acceptingPaths,
			ArrayList<String> allOtherPaths, ArrayList<String> failingPaths) {
		String firstPath = null;
		LinkedHashMap<String, Double> sortedAllPathProbs = sortHashMapByValues(allPathProbs, true);
//		mainLog.println("Sorted Paths:" + sortedAllPathProbs.toString());
		if (acceptingPaths.size() > 0) {
//			mainLog.println("Accepting Paths: " + acceptingPaths.toString());
			firstPath = findFirstStringFromSortedListInList(sortedAllPathProbs, acceptingPaths);
		} else {
			if (allOtherPaths.size() > 0) {
//				mainLog.println("Other Paths: " + allOtherPaths.toString());
				firstPath = findFirstStringFromSortedListInList(sortedAllPathProbs, allOtherPaths);
			} else {
				if (failingPaths.size() > 0) {
//					mainLog.println("Failing Paths: " + failingPaths.toString());
					firstPath = findFirstStringFromSortedListInList(sortedAllPathProbs, failingPaths);
				}
			}
		}
		return firstPath;

	}

	private String findFirstStringFromSortedListInList(LinkedHashMap<String, Double> sortedList,
			ArrayList<String> list) {
		String toret = null;

		// go over each element in the sortedList and see if its in the list
		for (String key : sortedList.keySet()) {
			if (list.contains(key)) {
				toret = key;
				break;
			}

		}

		return toret;
	}

	public ArrayList<Integer> addSeqPolicyToJointPolicy(SequentialTeamMDP seqTeamMDP, Strategy strat, int initialState,
			boolean noJointState, boolean syncpoints) throws PrismException {

		int nextRobotstate = StatesHelper.BADVALUE;

		int rNum = 0;
		int[] robotStates = null;
		int nextRobot = 0;
		List<State> states;
		State currState;// = states.get(initialState);
		if (!noJointState) {
			states = mdp.getStatesList();
			currState = states.get(initialState);
			states = seqTeamMDP.teamMDPTemplate.getStatesList();
			rNum = firstFailedRobot(currState);
			robotStates = getRobotStatesIndexFromJointState(currState, states);
			int robotStateId = robotStates[rNum];

		} else {
			states = seqTeamMDP.teamMDPWithSwitches.getStatesList();
			currState = states.get(initialState);
			// lets get some information from the model itself
			// if we dont have initial states
			rNum = StatesHelper
					.getRobotNumberFromSeqTeamMDPState(seqTeamMDP.teamMDPWithSwitches.getStatesList().get(initialState));
			robotStates = new int[nRobots];
			for (int i = 0; i < nRobots; i++) {
				if (i == rNum) {
					// we dont want mdp states
					robotStates[i] = initialState;
				} else {
					// this is just in case there are no actions for the other robot
					// so we know where it is in the mdp
					int anyinitstate = seqTeamMDP.initialStates.get(i).nextSetBit(0);
					robotStates[i] = anyinitstate;

				}
			}

		}
		nextRobot = (rNum + 1) % nRobots;
		nextRobotstate = robotStates[nextRobot];

		//
		// seqTeamMDP.setInitialStates(robotStates);
		// seqTeamMDP.addSwitchesAndSetInitialState(rNum);
		initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		State s1 = seqTeamMDP.teamMDPTemplate.getStatesList().get(initialState);
		mainLog.println("Adding policy for " + s1.toString());
		return breakSeqPolicyAndAddtoPolicy(seqTeamMDP, strat, initialState, nextRobotstate, robotStates, syncpoints,
				false);

	}

	private int addStateToMDP(State s) {
		int index = findStateIndex(s);
		if (index == StatesHelper.BADVALUE) {
			mdp.getStatesList().add(s);
			index = mdp.getNumStates();
			statesMap.put(s, index);
			mdp.addState();

		}
		// for the first state only
		// initialize the probs vector
		if (mdp.getNumStates() == 1) {
			probsVector.put(index, 1.0);
		}
		return index;
	}

	private void addTransitionToMDPResetTask(State parentState, State resetState, double prob) {
		ArrayList<State> states = new ArrayList<State>();
		states.add(resetState);
		ArrayList<Double> probs = new ArrayList<Double>();
		probs.add(prob);
		addTranstionToMDP(parentState, states, probs, "reset", 1.0);
	}

	private void addTranstionToMDP(State parentStates, ArrayList<State> states, ArrayList<Double> probs, String action,
			double norm) {
		int parentIndex = addStateToMDP(parentStates);
		int index;
		Distribution distr = new Distribution();

		for (int succ = 0; succ < states.size(); succ++) {
			index = addStateToMDP(states.get(succ));
			double normalizedProb = probs.get(succ) / norm;
			distr.add(index, normalizedProb);
			if (!probsVector.containsKey(index)) {
				// this can happen so we're kind of ignoring it
				// the scenario where two different paths end up at the same node
				// we're just going to stick to the previous probs
				double probHere = normalizedProb * probsVector.get(parentIndex);
				probsVector.put(index, probHere);
			}

		}
		// int actionNo = mdp.getNumChoices(parentIndex);
		mdp.addActionLabelledChoice(parentIndex, distr, action);

	}

	// private boolean allDone(int[] currentSteps, int[] maxSteps) {
	// boolean toret = true;
	// for (int i = 0; i < currentSteps.length; i++) {
	// if (currentSteps[i] < maxSteps[i] - 1) {
	// toret = false;
	// break;
	// }
	// }
	// return toret;
	// }

	public boolean checkAndAddJointFailStateToJointPolicy(int stateNumInMDP, State currentJointState,
			int numFailedRobots, int numFailedRobotsInInitState, SequentialTeamMDP seqTeamMDP,
			int[] robotDAassociations) {
		boolean isJointFailState = false;
		double stateProb = 0;
		// get number of failed robots
		numFailedRobots = hasFailState(currentJointState);

		// now check if its a state where robots have failed
		if (numFailedRobots > numFailedRobotsInInitState) {
			isJointFailState = true;
			// if everyone has failed, theres nothing to do
			if (numFailedRobots == nRobots) {
				allFailStatesSeen.set(stateNumInMDP);
				deadendStates.set(stateNumInMDP);
			} else {
				if (!allFailStatesSeen.get(stateNumInMDP)) {
					stateProb = getProbability(0, stateNumInMDP, 1.0, new BitSet());
					State newJointState = resetTasksInIntermediateStates(currentJointState, seqTeamMDP,
							robotDAassociations);
					// do we need to reset any tasks, if so we need a new failstate
					if (newJointState != null) {
						addTransitionToMDPResetTask(currentJointState, newJointState, 1.0);
						stateNumInMDP = statesMap.get(newJointState);

					}
					if (!allFailStatesSeen.get(stateNumInMDP)) {
						stuckStatesQ.add(new StateProb(stateNumInMDP, stateProb));
						allFailStatesSeen.set(stateNumInMDP);

					} else {
						deadendStates.set(stateNumInMDP);
					}
				} else {
					deadendStates.set(stateNumInMDP);
				}
			}

		}
		return isJointFailState;
	}

//	private boolean checkCombinationCounter(int arr[], int start, int end) {
//		int endcondition = 1;
//		boolean allDone = true;
//		for (int i = start; i < end; i++) {
//			if (arr[i] != endcondition) {
//				allDone = false;
//				break;
//			}
//		}
//		return allDone;
//	}

	public boolean checkSynchronizationPointAddToQ(int stateNumInMDP, State currentJointState, int numFailedRobots,
			int numFailedRobotsInInitState, SequentialTeamMDP seqTeamMDP, int[] robotDAassociations) {
		boolean isSynchronizationPoint = checkAndAddJointFailStateToJointPolicy(stateNumInMDP, currentJointState,
				numFailedRobots, numFailedRobotsInInitState, seqTeamMDP, robotDAassociations);
		return isSynchronizationPoint;
	}

	public int convertDAIndToStateInd(int daNum, int numda) {
		return ((numda - 1) - daNum);
	}

	public void testMDPsToJointPolicy(MDPSimple[] pols, int[][] mdpMaps, SequentialTeamMDP seqTeamMDP) {
		Object DAInitStates[] = getDAStartStates(seqTeamMDP); // all das dont start at 0, so we need these if we want to
		// reset stuff
		BitSet DAGoalStates[] = getDAGoalStates(seqTeamMDP); // we need these to check if we've got accepting states
		int robotDAassociations[] = new int[numDA]; // to track which robot affects which da
		Arrays.fill(robotDAassociations, -1); // initializing

	}

	// dear joint policy, you are the joy in my life - thank you
	// someday you wont be but thats how it is - thank you for helping drown out the
	// noise
	// stay safe bruv :P
	// you and of course an absolutely distracting playlist - things to be grateful
	// for
	public void convertMDPsToJointPolicy(MDPSimple[] pols, int[][] mdpMaps, int firstRobot, BitSet[] acceptingStates,
			SequentialTeamMDP seqTeamMDP, boolean synchronizationpoints, HashMap<String, Vector<BitSet>> paths,
			String firstPath, Strategy strat, HashMap<Integer, Vector<BitSet>> allStateNumbers,
			HashMap<String, Double> allPathProbs, ArrayList<String> acceptingPaths, ArrayList<String> allOtherPaths) {

		mainLog.println("Seq to Joint");
		String initialPathString = firstPath;
		boolean doPaths = ((firstPath != null) && (paths != null));
		Object DAInitStates[] = getDAStartStates(seqTeamMDP); // all das dont start at 0, so we need these if we want to
																// reset stuff
		BitSet DAGoalStates[] = getDAGoalStates(seqTeamMDP); // we need these to check if we've got accepting states
		int robotDAassociations[] = new int[numDA]; // to track which robot affects which da
		Arrays.fill(robotDAassociations, -1); // initializing

		int initialStates[] = new int[nRobots];
		int lastStates[] = new int[nRobots];

		int nextStates[] = new int[nRobots];
		int currentStates[] = new int[nRobots];

		Queue<State> statesToExploreQ = new LinkedList<State>();
		Queue<int[]> correspondingRobotMDPStatesQ = new LinkedList<int[]>();

		BitSet discoveredStates = new BitSet();

		ArrayList<Double> nextStateProbs = new ArrayList<Double>();
		ArrayList<State> nextJointStates = new ArrayList<State>();
		ArrayList<int[]> combinations = new ArrayList<int[]>();
		double sumProb = 0;

		int numFailedRobots = 0;
		int numFailedRobotsInInitState = StatesHelper.BADVALUE;

		double stateProb;
		int stateNumInMDP;
		boolean isAcceptingJointState;

		int[] numSuccessorStates = new int[nRobots];
		ArrayList<ArrayList<Entry<Integer, Double>>> successorStates = new ArrayList<ArrayList<Entry<Integer, Double>>>(
				nRobots);

		String jointAction = "";
		int numChoices = 0;
		Object action = null;
		for (int r = 0; r < nRobots; r++) {

			StatesHelper.saveMDP(pols[r], null, "", "stratmdp" + r, true);
		}
		HashMap<String, int[]> pathInitialStates = new HashMap<String, int[]>();
		HashMap<String, int[]> pathLastStates = new HashMap<String, int[]>();
		// lets do this for all paths
		if (doPaths) {
			for (String key : paths.keySet()) {
				pathInitialStates.put(key, new int[nRobots]);
				pathLastStates.put(key, new int[nRobots]);
				// update initial states to reflect DA progress
				updateInitialStates(pols, mdpMaps, firstRobot, acceptingStates,
						seqTeamMDP.teamMDPWithSwitches.getStatesList(), seqTeamMDP.acceptingStates, paths.get(key));

				// get all the initial states and last states
				for (int r = 0; r < nRobots; r++) {

					// StatesHelper.saveMDP(pols[r], null, "", "startmdp"+r, true);

					// pick the initial states according to paths
					if (!doPaths) {
						initialStates[r] = pols[r].getFirstInitialState();
						lastStates[r] = getLastState(pols[r], acceptingStates[r], null);
					} else {
						BitSet pathForRobot = (BitSet) paths.get(key).get(r);
						BitSet tempPathHolder = (BitSet) allStateNumbers.get(0).get(r).clone();
						tempPathHolder.and(pathForRobot);
						// use all state numbers
						int temp = tempPathHolder.nextSetBit(0);
						if (temp == -1)
							temp = paths.get(key).get(r).nextSetBit(0);
						pathInitialStates.get(key)[r] = temp;

						tempPathHolder = (BitSet) allStateNumbers.get(pathForRobot.cardinality() - 1).get(r).clone();
						tempPathHolder.and(pathForRobot);
						temp = tempPathHolder.nextSetBit(0);
						if (temp == -1) {
							// get the last set bit on this path
							temp = pathForRobot.previousSetBit(pathForRobot.size());
							// temp = paths.get(key).get(r).nextSetBit(0);
						}
						pathLastStates.get(key)[r] = temp;// getLastState(pols[r], acceptingStates[r],
															// paths.get(key).get(r));
					}
				}
//				mainLog.println(key);
//				mainLog.println(paths.get(key));
//				mainLog.println(Arrays.toString(pathInitialStates.get(key)));
//				mainLog.println(Arrays.toString(pathLastStates.get(key)));
			}
			initialStates = pathInitialStates.get(firstPath);
			lastStates = pathLastStates.get(firstPath);
			if (initialStates[nRobots - 1] == -1) {
				for (int r = 0; r < nRobots; r++) {
					initialStates[r] = pols[r].getFirstInitialState();
					lastStates[r] = getLastState(pols[r], acceptingStates[r], null);
				}
			}
		}

		for (int r = 0; r < nRobots; r++) {

			StatesHelper.saveMDP(pols[r], null, "", "stratmdpfixed" + r, true);
		}
		Object[] teamProgress = getTeamDAProgress(pols, firstRobot, initialStates, lastStates, DAInitStates,
				robotDAassociations);
		Object[] sharedStates = getTeamSharedStatesProgress(pols, firstRobot, initialStates, lastStates, initialStates);
		// use this to check if we need to trigger a synchronization point thing
		// but waaa
		// mainLog.println(sharedStates.toString());
		currentStates = initialStates.clone();
		State currentJointState = createJointState(teamProgress, sharedStates, currentStates, pols);
		// createJointState(teamProgress, currentStates, pols);
		statesToExploreQ.add(currentJointState);
		correspondingRobotMDPStatesQ.add(currentStates.clone());

		numFailedRobots = hasFailState(currentJointState);
		if (numFailedRobotsInInitState == StatesHelper.BADVALUE) {
			// set it for the initial state
			// we'll use this to determine new fail states
			numFailedRobotsInInitState = numFailedRobots;
		}
		int stcount = 0;

		Queue<Integer> stcountstack = new LinkedList<Integer>();
		Queue<String> statePathStack = new LinkedList<String>();
		statePathStack.add(firstPath);
		stcountstack.add(stcount);
		int[] robotStates;
		int[] switchCurrentStates = new int[nRobots];
		while (!statesToExploreQ.isEmpty()) {
			// remove states from Q
			currentJointState = statesToExploreQ.remove();
			//
			// if (currentJointState.toString().equals("(0,0,1,0,1,-1,4,-1,9)"))
			// mainLog.println("At (0,0,1,0,1,-1,4,-1,9), wrong switch here");

			robotStates = getBrokenRobotStatesIndexFromJointState(currentJointState, pols);

			stcount = stcountstack.remove();
			firstPath = statePathStack.remove();
			currentStates = correspondingRobotMDPStatesQ.remove();
			boolean switchPath = false;
			String switchPathName = null;
			int switchPathRobot = -1;
			int robotNotOnPath = -1;

			// check if currentStates are on the path

			int maxRobotsOnPath = 0;
			if (doPaths) {
				ArrayList<String> actualRobotStatePaths = new ArrayList<String>();
				ArrayList<String> currentRobotStatePaths = new ArrayList<String>();
				HashMap<String, Integer> firstRobotToSwitchPathArr = new HashMap<String, Integer>();
				HashMap<String, Integer> numRobotsOnPath = new HashMap<String, Integer>();
				int r = firstRobot;
				robotNotOnPath = -1;
				do {
					if (!paths.get(firstPath).get(r).get(currentStates[r])) {
						switchPath = true;
						robotNotOnPath = r;
						break;
					}
					r = (r + 1) % nRobots;
				} while (r != firstRobot);
				// so we know this path isnt the one
				// now we need to find the next path
				// our first priority is a path that has all these states on it
				if (switchPath) {

					// for all paths,
					// we know which robot is not on the path
					// first lets filter all the paths where some of the robots are on the path
					for (String key : allPathProbs.keySet()) {
						int robotsOnPath = 0;
						r = firstRobot;
						do {
							if (paths.get(key).get(r).get(currentStates[r])) {
								robotsOnPath++;
							} else {
								break; // so if we're not on the path starting from the first robot, then there's no
										// point
							}
							r = (r + 1) % nRobots;
						} while (r != firstRobot && r != (robotNotOnPath + 1) % nRobots);
						if (robotsOnPath > 0) {
							currentRobotStatePaths.add(key);

							if (numRobotsOnPath.containsKey(key))
								numRobotsOnPath.put(key, numRobotsOnPath.get(key) + robotsOnPath);
							else
								numRobotsOnPath.put(key, robotsOnPath);
						}
					}

					// from those find ones where maybe the next expected state is on that path
					for (int pathnum = 0; pathnum < currentRobotStatePaths.size(); pathnum++) {
						r = (robotNotOnPath + 1) % nRobots;
						int robotsOnPath = 0;
						String pathName = currentRobotStatePaths.get(pathnum);
						do {

							if (robotStates[r] >= 0 && paths.get(pathName).get(r).get(robotStates[r])) {
								robotsOnPath++;
							}

							r = (r + 1) % nRobots;
						} while (r != firstRobot);

						if (robotsOnPath > 0) {
							actualRobotStatePaths.add(pathName);
							if (numRobotsOnPath.containsKey(pathName))
								numRobotsOnPath.put(pathName, numRobotsOnPath.get(pathName) + robotsOnPath);
							else
								numRobotsOnPath.put(pathName, robotsOnPath);
						}
					}

					boolean aresame = true;
					if (switchPath) // reverify are the initial states the same ? if so its not a switchPath thing
					{
						if (actualRobotStatePaths.size() > 0) {
							if (actualRobotStatePaths.size() > 1) {
								// then like check if there's an accepting one
								for (int acp = 0; acp < acceptingPaths.size(); acp++) {
									if (actualRobotStatePaths.contains(acceptingPaths.get(acp))) {
										switchPathName = acceptingPaths.get(acp);
										break;
									}
								}
								if (!acceptingPaths.contains(switchPathName)) {
									for (int acp = 0; acp < allOtherPaths.size(); acp++) {
										if (actualRobotStatePaths.contains(allOtherPaths.get(acp))) {
											switchPathName = allOtherPaths.get(acp);
											break;
										}
									}
									if (!allOtherPaths.contains(switchPathName)) {
										switchPathName = actualRobotStatePaths.get(0);

									}
								}

							} else
								switchPathName = actualRobotStatePaths.get(0);
						} else {
							int mostRobots = 0;
							for (int acp = 0; acp < acceptingPaths.size(); acp++) {
								if (currentRobotStatePaths.contains(acceptingPaths.get(acp))) {
									switchPathName = acceptingPaths.get(acp);
									break;
								}
							}
							if (!acceptingPaths.contains(switchPathName)) {
								for (int acp = 0; acp < allOtherPaths.size(); acp++) {
									if (currentRobotStatePaths.contains(allOtherPaths.get(acp))) {
										switchPathName = allOtherPaths.get(acp);
										break;
									}
								}
								if (!allOtherPaths.contains(switchPathName)) {
									// get the highest num one
									for (int pathnum = 0; pathnum < currentRobotStatePaths.size(); pathnum++) {
										String pathName = currentRobotStatePaths.get(pathnum);
										{
											if (mostRobots < numRobotsOnPath.get(pathName)) {
												mostRobots = numRobotsOnPath.get(pathName);
												switchPathName = pathName;
											}
										}
									}
								}

							}

							// switchPathName = currentRobotStatePaths.get(0);
						}

						// for(int r = 0; r<nRobots; r++)
						r = firstRobot;
						do {
							if (pathInitialStates.get(firstPath)[r] != pathInitialStates.get(switchPathName)[r]) {
								aresame = false;
								break;
							}
							r = (r + 1) % nRobots;
						} while (r != firstRobot);
					}
					switchPath = !aresame; // only switch if initial states are not the same
					if (switchPath && maxRobotsOnPath == nRobots) {
						// we dont need to switch anything,
						// we just need to change the value of first path
						firstPath = switchPathName;
						switchPath = false;
					}
//					if (switchPath) {
//						mainLog.println("chosen path");
//						mainLog.println(paths.get(switchPathName));
//					}
				}
			}
			// find state in joint policy mdp
			stateNumInMDP = addStateToMDP(currentJointState);

			// dont explore if already done
			if (discoveredStates.get(stateNumInMDP)) {
				continue;
			}
			// if not go on :P
			discoveredStates.set(stateNumInMDP);

			// now check if its an accepting state
			// if so we don't need to explore this state
			isAcceptingJointState = isAcceptingState(currentJointState, DAGoalStates);
			if (isAcceptingJointState) {
				allTasksCompletedStates.set(stateNumInMDP);
				continue;
			}

			if (synchronizationpoints) {
				// check if its a fail state
				if (checkSynchronizationPointAddToQ(stateNumInMDP, currentJointState, numFailedRobots,
						numFailedRobotsInInitState, seqTeamMDP, robotDAassociations)) {
					continue;
				}
			}

			Arrays.fill(numSuccessorStates, 0);
			successorStates.clear();

			// if its not a fail state, we need to check if we have to switch paths
			// switchPath = false;
			if (switchPath) {

//				mainLog.println("Need to switch path now - magic please :P");

				// TODO: put shit here that works
				// just shift everyone to that path
				// issue - at this point I magically dont care about the tasks accomplished on
				// the previous policy
				// FIXME: so fix that ^
				// int r ;
				for (int r = 0; r < nRobots; r++) {
					successorStates.add(new ArrayList<Entry<Integer, Double>>());
				}
				jointAction = "switchPath";
				int r = robotNotOnPath;

				do {

					// come back here
					ArrayList<Entry<Integer, Double>> successorStatesTemp = successorStates.get(r);
					// get corresponding state from switch path
					// on this path get the stcount state for each robot
					BitSet stateIsolated = (BitSet) paths.get(switchPathName).get(r).clone();
					stateIsolated.and(allStateNumbers.get(stcount).get(r));
//					mainLog.println(stateIsolated.toString());
					numSuccessorStates[r] = stateIsolated.cardinality();
					// so now lets put all the states in the bitset stateIsolated in the list
					// o but how do we the associated probabilities ?
					// o but we cant :P so we'll just say 1
					// also the cardinality of this bitset should be 1 because its a path we've
					// chosen
					if (numSuccessorStates[r] != 1)
						mainLog.println("Multiple states on this path, we havent thought of what to do here so yeah");
					int nxs = stateIsolated.nextSetBit(0);
					while (nxs != -1) {
						successorStatesTemp.add(new AbstractMap.SimpleEntry<Integer, Double>(nxs, 1.0));
						// set current state to nxs
						switchCurrentStates[r] = nxs;
						nxs = stateIsolated.nextSetBit(nxs + 1);

					}
					if (numSuccessorStates[r] == 0) {
						numSuccessorStates[r] = 1;
						successorStatesTemp.add(new AbstractMap.SimpleEntry<Integer, Double>(currentStates[r], 1.0));
						switchCurrentStates[r] = currentStates[r];
					}

					// successorStatesTemp.add(new
					// AbstractMap.SimpleEntry<Integer,Double>(currentStates[r],1.0));

					r = (r + 1) % nRobots;
				} while (r != firstRobot);

				for (r = 0; r < nRobots; r++) {
					if (successorStates.get(r).size() == 0) {
						numSuccessorStates[r] = 1;
						ArrayList<Entry<Integer, Double>> successorStatesTemp = successorStates.get(r);
						successorStatesTemp.add(new AbstractMap.SimpleEntry<Integer, Double>(currentStates[r], 1.0));
						switchCurrentStates[r] = currentStates[r];
					}
				}
				firstPath = switchPathName;

			} else {

				jointAction = "";
				for (int r = 0; r < nRobots; r++) {
					boolean alldone = false;
					while (!alldone) {
						numChoices = pols[r].getNumChoices(currentStates[r]);
						if (numChoices > 0) {
							action = pols[r].getAction(currentStates[r], numChoices - 1);
							if (action != null) {
								if (action.toString().contains("switch")) {
									// TODO: Fatma FIX THIS - MULTIPLE SWITCHES FOR ONE ROBOT ON ONE PATH
									// THIS DOES NOT WORK THIS WAY
									// WE NEED TO DEAL WITH MULTIPLE SWITCHES TO AND FROM THE SAME ROBOT
									// FIXME: THE MULTIPLE SWITCHES THING FOR ONE ROBOT
									// check if the next state belongs to the same robot
									// so we're going to be a bit meh and just check if the thing after the _ is the
									// same as this robot number

									// we're switching to another robot
									// meaning we're either at the end of this path
									// or we might come back to this same robot later

									// are we at the end of this path ?
									BitSet thisRobotsPath = (BitSet) paths.get(firstPath).get(r).clone();
									if (thisRobotsPath.cardinality() > (stcount + 1)) {
										// then we have to come back to this path
										thisRobotsPath.and(allStateNumbers.get(stcount + 1).get(r));

										// change the current state to th
									}

									// String actionString = action.toString();
									// int underscorePosition = actionString.lastIndexOf('-');
									// String nextRobotNumberString = actionString.substring(underscorePosition + 1,
									// actionString.length());
									// int nextRobotNumber = (int) (Double.parseDouble(nextRobotNumberString));
									// if (!(nextRobotNumber == r)) {
									// numChoices = 0;
									// alldone = true;
									// } else {
									// // we've got to get the next state
									// // so the next state in our path at the next number ?
									// BitSet stateIsolated = (BitSet) paths.get(firstPath).get(r).clone();
									// stateIsolated.and(allStateNumbers.get(stcount + 1).get(r));
									// mainLog.println(stateIsolated.toString());
									// // set this as our current state
									// // and get its choices
									// currentStates[r] = stateIsolated.nextSetBit(0);
									// numChoices = 0;

									/***********
									 * If you want things to run do not uncomment the lines below, unless you've got
									 * full code for this part
									 ******/
									numChoices = 0;
									alldone = true;
									/*****/
									// }
								}
							}
							if (numChoices == 1) {
								numSuccessorStates[r] = pols[r].getNumTransitions(currentStates[r], numChoices - 1);
								jointAction += pols[r].getAction(currentStates[r], numChoices - 1);
								ArrayList<Entry<Integer, Double>> succStates = getSuccessorStatesFromMDP(pols[r],
										currentStates[r], numChoices - 1);
								successorStates.add(succStates);
								alldone = true;

							} else {
								if (numChoices > 1) {
									mainLog.println(
											"More than one choice for state? We talked about ignoring this. see notes");
									alldone = true;
								}
							}
						} else
							alldone = true;

						if (numChoices == 0 && alldone) {
							numSuccessorStates[r] = 1;
							ArrayList<Entry<Integer, Double>> succStates = new ArrayList<Entry<Integer, Double>>();
							succStates.add(new AbstractMap.SimpleEntry<Integer, Double>(currentStates[r], 1.0));
							successorStates.add(succStates);
							alldone = true;
						}
					}
				} // done going over the policy for each robot
			}
			nextStateProbs.clear(); // new ArrayList<Double>();
			nextJointStates.clear(); // new ArrayList<State>();
			combinations.clear();
			sumProb = 0;
			// Set<String> intersection = null;
			generateCombinations(numSuccessorStates.clone(), numSuccessorStates.clone(), combinations,
					numSuccessorStates.length);
			// Vector<Set<String>> switchingPathsForRobots = new Vector<Set<String>>();
			// boolean succSwitchPath = false;
			for (int[] combination : combinations) {
				// succSwitchPath = false;
				stateProb = 1.0;
				for (int r = 0; r < nRobots; r++) {
					nextStates[r] = successorStates.get(r).get(combination[r] - 1).getKey();
					stateProb = stateProb * successorStates.get(r).get(combination[r] - 1).getValue();
					// if (doPaths) {
					// if (!paths.get(firstPath).get(r).get(nextStates[r])) {
					// // succSwitchPath = true;
					// // mainLog.println("This state is not on the same path we're following");
					// for (String key : paths.keySet()) {
					// // find the path with this state
					// if (paths.get(key).get(r).get(nextStates[r])) {
					// // if(switchingPathsForRobots.size() <=r)
					// // {
					// // while(switchingPathsForRobots.size()<=r)
					// // switchingPathsForRobots.add(new HashSet<String>());
					// // }
					// // switchingPathsForRobots.get(r).add(key);
					// //// mainLog.println("This state is on path " + key);
					// //// mainLog.println(paths.get(key));
					// }
					// }
					// }
					//
					// }
				}

				initialStates = pathInitialStates.get(firstPath);
				lastStates = pathLastStates.get(firstPath);
				teamProgress = getTeamDAProgress(pols, firstRobot, nextStates, lastStates, DAInitStates,
						robotDAassociations);

				// FIXME: this has to do something you know :P
				sharedStates = getTeamSharedStatesProgress(pols, firstRobot, nextStates, lastStates, initialStates);

				State nextJointState = createJointState(teamProgress, sharedStates, nextStates, pols);
				// createJointState(teamProgress, nextStates, pols);
				nextStateProbs.add(stateProb);
				sumProb += stateProb;
				nextJointStates.add(nextJointState);
				statesToExploreQ.add(nextJointState);
				correspondingRobotMDPStatesQ.add(nextStates.clone());
				stcountstack.add(stcount + 1);
				statePathStack.add(firstPath);

			}
			if (switchPath && nextJointStates.size() == 1 && currentJointState.equals(nextJointStates.get(0))) {
				mainLog.println("skipping the switch path bit but switching the path");
				// but unsetting it as a discovered state cuz we need to go here again
				discoveredStates.set(stateNumInMDP, false);
			} else {
				if (!firstPath.equals(initialPathString))
					jointAction = "sp" + jointAction;
				addTranstionToMDP(currentJointState, nextJointStates, nextStateProbs, jointAction, sumProb);

				// just to test stuff
				if (jointAction.toString().contains("switchPath")) {
					// what are the current robot states
					int[] robotStatesInTeamMDP = getRobotStatesIndexFromJointState(currentJointState,
							seqTeamMDP.teamMDPWithSwitches.getStatesList());
					// lets see if we can find policies for any of these
					for (int r = 0; r < nRobots; r++) {
						int currRobotState = robotStatesInTeamMDP[r];
						int nextRobotState = robotStatesInTeamMDP[(r + 1) % nRobots];
						// if(currRobotState != -2)
						// breakSeqPolicyAndAddtoPolicy(seqTeamMDP,strat,currRobotState,nextRobotState,
						// robotStatesInTeamMDP,true,true);
					}

				}

			}
		}
		saveJointPolicy();
	}

	public State createJointState(Object[] DFAProgress, int[] currentRobotStates, MDPSimple[] pols) {
		State currentJointState = new State(DFAProgress.length + currentRobotStates.length);
		int offset = 0;
		for (int i = 0; i < DFAProgress.length; i++) {
			currentJointState.setValue(i + offset, DFAProgress[i]);
		}
		offset = DFAProgress.length;
		for (int i = 0; i < currentRobotStates.length; i++) {
			int temp = StatesHelper.getMDPStateFromState(pols[i].getStatesList().get(currentRobotStates[i]));
			currentJointState.setValue(i + offset, temp);
		}
		return currentJointState;
	}

	public State createJointState(Object[] DFAProgress, Object[] sharedState, int[] currentRobotStates,
			MDPSimple[] pols) {
		if (sharedState != null) {
			State currentJointState = new State(DFAProgress.length + sharedState.length + currentRobotStates.length);
			int offset = 0;
			for (int i = 0; i < DFAProgress.length; i++) {
				currentJointState.setValue(i + offset, DFAProgress[i]);
			}
			offset = DFAProgress.length;
			for (int i = 0; i < sharedState.length; i++) {
				currentJointState.setValue(i + offset, sharedState[i]);
			}
			offset += sharedState.length;
			for (int i = 0; i < currentRobotStates.length; i++) {
				Object[] temp = StatesHelper.getMDPStateFromState(pols[i].getStatesList().get(currentRobotStates[i]),
						this.teamMDPVarlist, this.notSharedVarsList);
				for (int j = 0; j < temp.length; j++) {
					currentJointState.setValue(offset, temp[j]);

				}
				offset = offset + (temp.length);
			}
			return currentJointState;
		} else {
			State currentJointState = new State(DFAProgress.length + currentRobotStates.length);
			int offset = 0;
			for (int i = 0; i < DFAProgress.length; i++) {
				currentJointState.setValue(i + offset, DFAProgress[i]);
			}
			offset = DFAProgress.length;
			for (int i = 0; i < currentRobotStates.length; i++) {
				Object[] temp = StatesHelper.getMDPStateFromState(pols[i].getStatesList().get(currentRobotStates[i]),
						this.teamMDPVarlist, this.notSharedVarsList);
				for (int j = 0; j < temp.length; j++) {
					currentJointState.setValue(offset, temp[j]);

				}
				offset = offset + (temp.length);
			}
			return currentJointState;
		}
	}

	public Object[] createSingleRobotState(State jointState, int rNum) {
		// pick the robot num
		Object[] mdpStates = jointState.varValues;
		// int dfaStart = 0;
		int dfaEnd = mdpStates.length - nRobots;
		int robotIndex = dfaEnd + rNum;
		Object[] singleRobotState = new Object[dfaEnd + 2];
		singleRobotState[0] = rNum;
		for (int i = 1; i < dfaEnd + 1; i++) {
			singleRobotState[i] = mdpStates[i - 1];
		}
		singleRobotState[dfaEnd + 1] = mdpStates[robotIndex];
		return singleRobotState;
	}

	public Object[] createSingleRobotStateIncludeSharedState(State jointState, int rNum) {
		// pick the robot num
		Object[] mdpStates = jointState.varValues;
		int sharedStateStart = this.numDA;// this.numDA;
		// int dfaStart = 0;
		int dfaEnd = this.numDA;// mdpStates.length - nRobots;
		int robotIndex = dfaEnd + sharedVarsList.size() + rNum;
		Object[] singleRobotState = new Object[dfaEnd + 2 + this.sharedVarsList.size()];
		singleRobotState[0] = rNum;
		for (int i = 1; i < dfaEnd + 1; i++) {
			singleRobotState[i] = mdpStates[i - 1];
		}
		// match everything else

		for (int i = 0; i < this.sharedVarsList.size(); i++) { // singleRobotState[i+(dfaEnd+1)]=mdpStates[dfaEnd+i];
			int index = this.teamMDPVarlist.getIndex(sharedVarsList.get(i));
			int otherIndex = this.mdp.getVarList().getIndex(sharedVarsList.get(i));
			if (index != -1) {
				singleRobotState[index] = mdpStates[otherIndex];
			}
		}
		for (int i = 0; i < this.notSharedVarsList.size(); i++) { // singleRobotState[i+(dfaEnd+1)]=mdpStates[dfaEnd+i];
			int index = this.teamMDPVarlist.getIndex(notSharedVarsList.get(i));
			// int otherIndex = this.mdp.getVarList().getIndex(notSharedVarsList.get(i));
			if (index != -1) {
				singleRobotState[index] = mdpStates[robotIndex];
			}
		}

		// singleRobotState[dfaEnd +this.sharedVarsList.size()+ 1] =
		// mdpStates[robotIndex];
		return singleRobotState;
	}

	public int findStateIndex(State s) {
		int indexInt = StatesHelper.BADVALUE;
		Object index = statesMap.get(s);
		if (index != null) {
			indexInt = (int) index;
		}
		return indexInt;
	}

	public int firstFailedRobot(Object[] mdpStates, int start, int end) {
		int failedRobot = StatesHelper.BADVALUE; // no one
		for (int i = start; i < end; i++) {
			if ((int) mdpStates[i] == StatesHelper.failState) {
				failedRobot = i - start;
				break;
			}
		}
		return failedRobot;
	}

	public int firstFailedRobot(State jointState) { // TODO fix this for when the number of states for each robot
													// vary/are more than 1
		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
		int end = mdpStates.length;
		return firstFailedRobot(mdpStates, start, end);

	}

	// private void generateCombinations(int counter[], int original[],
	// ArrayList<int[]> res) {
	// // mainLog.println(Arrays.toString(counter));
	// res.add(counter.clone());
	// if (!checkCombinationCounter(counter, 0, counter.length)) {
	// boolean do0 = true;
	// for (int i = counter.length - 1; i >= 0; i--) {
	// if (checkCombinationCounter(counter, 0, i + 1)) {
	// counter[i + 1]--;
	// counter[i] = original[i];
	// do0 = false;
	// break;
	// }
	// }
	// if (do0) {
	// counter[0]--;
	// }
	// generateCombinations(counter, original, res);
	//
	// }
	// }
	//

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

	public BitSet[] getDAGoalStates(SequentialTeamMDP seqTeamMDP) {
		BitSet[] daStartStates = new BitSet[numDA];
		for (int i = 0; i < numDA; i++) {
			daStartStates[i] = getDAaccStatesForRobot(convertDAIndToStateInd(i, numDA), 0, seqTeamMDP);
		}
		return daStartStates;
	}

	public int getDAStartStateForRobot(int da_num, int r, SequentialTeamMDP seqTeamMDP) {
		return seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getStartState();
	}

	public Object[] getDAStartStates(SequentialTeamMDP seqTeamMDP) {
		Object[] daStartStates = new Object[numDA];
		for (int i = 0; i < numDA; i++) {
			daStartStates[i] = getDAStartStateForRobot(convertDAIndToStateInd(i, numDA), 0, seqTeamMDP);
		}
		return daStartStates;
	}

	// TODO: this can be more efficient
	public int getLastState(MDPSimple pol, BitSet acceptingStates, BitSet path) {
		int state = getLastState(pol, acceptingStates, 0, path);
		for (int kindOfState = 1; (state == StatesHelper.BADVALUE) && (kindOfState < 3); kindOfState++) {
			state = getLastState(pol, acceptingStates, kindOfState, path);
		}
		// StatesHelper.saveMDP(pol, null, "", "polx", true);
		if (state == StatesHelper.BADVALUE) {
			// if there is no accepting state,
			// no switch state
			// no fail state
			// then we can just do a dfs and get the last last state
			int kindOfState = 3;
			state = getLastState(pol, acceptingStates, kindOfState, path);
		}
		return state;
	}

	// kindOfState - 0 - accepting state
	// - 2 - fail state
	// - 1 - switch
	// - 3 - just the last state
	public int getLastState(MDPSimple pol, BitSet acceptingStates, int kindOfState, BitSet path) {

		int state = -1; // TODO: CHANGE THIS TO SOMETHING FOR MULTIPLE
		if (path != null) {
			state = path.nextSetBit(0);
			while (!pol.isInitialState(state) & state != -1) {
				state = path.nextSetBit(state + 1);

			}
			if (state == -1) {
				mainLog.println("ERROR ERROR ERROR - the path has no state that is an initial state");

			}
		}
		if (state == -1)
			state = pol.getFirstInitialState();
		// INITIAL STATES
		int lastState = StatesHelper.BADVALUE;
		LinkedList<Integer> statesQ = new LinkedList<Integer>();
		BitSet discovered = new BitSet();

		statesQ.add(state);
		while (!statesQ.isEmpty()) {
			state = statesQ.remove();
			if (!discovered.get(state)) {
				discovered.set(state);
				if (kindOfState == 0) {
					if (acceptingStates.get(state))
						lastState = state;
				}
				if (kindOfState == 2) {
					if (StatesHelper.isFailState(pol.getStatesList().get(state))) // TODO: fix this here , because this
																					// has changed
						lastState = state;
				}
				int choices = pol.getNumChoices(state);
				for (int i = 0; i < choices; i++) {
					if (kindOfState == 1) {
						if (pol.getAction(state, i) != null) {
							if (pol.getAction(state, i).toString().contains("switch"))
								lastState = state;
						}
					}
					Iterator<Entry<Integer, Double>> tIter = pol.getTransitionsIterator(state, i);
					while (tIter.hasNext()) {
						Entry<Integer, Double> child = tIter.next();
						int childState = child.getKey();
						statesQ.add(childState);
					}
				}
			}
		}
		if (kindOfState == 3) {
			// just get the last state in the dfs
			lastState = state;
		}

		return lastState; // so basically this is the last state we tried to explore
	}

	public int getNumDA() {
		return numDA;
	}

	public double getProbability(int startState, int stopState, double probHere, BitSet discovered) {
		int choice = 0;
		double probSum = 0;

		if (discovered.get(stopState)) { // once we've found the state , we dont care so everything returns 0
			probHere = 0;
			return probHere;
		}
		if (startState != stopState) {
			if (!discovered.get(startState)) {

				int numChoice = mdp.getNumChoices(startState);
				discovered.set(startState);
				if (numChoice == 1) {
					// Object action = mdp.getAction(startState, choice);
					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(startState, choice);
					while (tranIter.hasNext()) {
						Entry<Integer, Double> stateProbPair = tranIter.next();
						int nextState = stateProbPair.getKey();
						double nextStateProb = stateProbPair.getValue();
						// stop at the first failstate

						probSum += getProbability(nextState, stopState, nextStateProb * probHere, discovered);
					}
					probHere = probSum;
				} else
					probHere = 0; // because we dont care if you can get to any other state

			}
		} else {
			discovered.set(startState);
		}

		return probHere;
	}

	public double getProbabilityAllPossibleAcceptingStates(int startState, double probHere, BitSet discoveredStates,
			HashMap<Integer, Double> probMap) {
		int choice = 0;
		double probSum = 0;

		if (!allTasksCompletedStates.get(startState)) {

			int numChoice = mdp.getNumChoices(startState);
			if (!discoveredStates.get(startState)) {
				discoveredStates.set(startState);
				if (numChoice == 1) {
					Object action = mdp.getAction(startState, choice);
					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(startState, choice);
					while (tranIter.hasNext()) {
						Entry<Integer, Double> stateProbPair = tranIter.next();
						int nextState = stateProbPair.getKey();
						double nextStateProb = stateProbPair.getValue();
						// don't explore if the child state is the same as the parent state
						if (nextState != startState)
							probSum += getProbabilityAllPossibleAcceptingStates(nextState, nextStateProb * probHere,
									discoveredStates, probMap);
						else {
							mainLog.println("State " + nextState
									+ " in joint policy leads to itself, something is wrong here?");
						}
					}
					probHere = probSum;
				} else
					probHere = 0; // no chance of getting to an accepting state
			} else {
				double tprob = probMap.get(startState);
				if (tprob != 0) {
					discoveredStates.set(startState, false);
					probHere = getProbabilityAllPossibleAcceptingStates(startState, probHere, discoveredStates,
							probMap);
				} else
					probHere = tprob;
			}
			// else
			// {
			// //its a self loop so we will just stop here
			// probHere = 0;
			// }
			// probHere = probSum;
		}
		if (!probMap.containsKey(startState))
			probMap.put(startState, probHere);

		return probHere;
	}

	public double getProbabilityAnyAcceptingState(int startState, double probHere, BitSet discoveredStates) {
		int choice = 0;
		double probSum = 0;

		BitSet temp = (BitSet) discoveredStates.clone();
		temp.and(allTasksCompletedStates);
		if (temp.cardinality() > 0) // if we've reached any accepting state, just end
		{
			probHere = 0;
			return probHere;
		}
		if (!allTasksCompletedStates.get(startState)) {

			int numChoice = mdp.getNumChoices(startState);
			if (!discoveredStates.get(startState)) {
				discoveredStates.set(startState);
				if (numChoice == 1) {
					// Object action = mdp.getAction(startState, choice);
					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(startState, choice);
					while (tranIter.hasNext()) {
						Entry<Integer, Double> stateProbPair = tranIter.next();
						int nextState = stateProbPair.getKey();
						double nextStateProb = stateProbPair.getValue();
						probSum += getProbabilityAnyAcceptingState(nextState, nextStateProb * probHere,
								discoveredStates);
					}
					probHere = probSum;
				} else
					probHere = 0; // no chance of getting to an accepting state
			}
			// else
			// {
			// //its a self loop so we will just stop here
			// probHere = 0;
			// }
		} else {
			discoveredStates.set(startState);
		}
		return probHere;
	}

	public Object[] getRobotDAProgress(State previousRobotsState, State currentRobotsState, Object[] daInitStates,
			int[] robotDAassoc, int rnum) {

		Object[] previousRobotsDAState = StatesHelper.getDAStatesFromState(previousRobotsState);
		Object[] currentRobotsDAState = StatesHelper.getDAStatesFromState(currentRobotsState);
		int[] integerXORmask = StatesHelper.XORIntegers(previousRobotsDAState, currentRobotsDAState);
		for (int i = 0; i < integerXORmask.length; i++) {
			if (integerXORmask[i] == 1)
				robotDAassoc[i] = rnum; // should just be 0 or 1 and each task can be assinged to a single robot
		}
		Object[] updatedState = StatesHelper.multiplyWithMask(integerXORmask, currentRobotsDAState, daInitStates);

		return updatedState;

	}

	private Object[] getRobotSharedStateProgress(State previousRobotsState, State currentRobotsState,
			State initialState) {

		Object[] previousRobotsSharedState = StatesHelper.getSharedStatesFromState(previousRobotsState, teamMDPVarlist,
				sharedVarsList);
		Object[] currentRobotsSharedState = StatesHelper.getSharedStatesFromState(currentRobotsState, teamMDPVarlist,
				sharedVarsList);
		Object[] initialSharedState = StatesHelper.getSharedStatesFromState(initialState, teamMDPVarlist,
				sharedVarsList);

		int[] integerXORmask = StatesHelper.XORIntegers(previousRobotsSharedState, currentRobotsSharedState);
		Object[] updatedState = StatesHelper.multiplyWithMask(integerXORmask, currentRobotsSharedState,
				initialSharedState);

		// TODO Auto-generated method stub
		return updatedState;
	}

	public int getRobotState(State jointState, int rnum) {

		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
		// int end = mdpStates.length;

		return (int) mdpStates[rnum + start];
	}

	public int getRobotStateIndexFromJointState(State currState, int rNum, List<State> states) {
		Object[] robotState = /* createSingleRobotState */createSingleRobotStateIncludeSharedState(currState, rNum);
		int robotStateId = StatesHelper.getExactlyTheSameState(robotState, states);
		if (robotStateId == StatesHelper.BADVALUE)
			mainLog.println("Bad value for " + rNum + ":" + currState.toString() + " " + Arrays.toString(robotState));
		return robotStateId;
	}

	public int[] getRobotStates(State jointState) {// FIXME: this is not okay!!!

		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
		int end = mdpStates.length;
		int[] robotStates = new int[nRobots];
		for (int i = start; i < end; i++) {
			robotStates[i - start] = (int) mdpStates[i];
		}
		return robotStates;
	}

	public int[] getRobotStatesIndexFromJointState(State currState, List<State> states) {
		int[] toRet = new int[nRobots];
		for (int rNum = 0; rNum < nRobots; rNum++) {
			toRet[rNum] = getRobotStateIndexFromJointState(currState, rNum, states);
		}
		return toRet;
	}

	public int[] getBrokenRobotStatesIndexFromJointState(State currState, MDPSimple[] pols) {
		int[] toRet = new int[nRobots];
		for (int rNum = 0; rNum < nRobots; rNum++) {
			toRet[rNum] = getRobotStateIndexFromJointState(currState, rNum, pols[rNum].getStatesList());
		}
		return toRet;
	}

	public ArrayList<Entry<Integer, Double>> getSuccessorStatesFromMDP(MDPSimple pol, int state, int choice) {
		ArrayList<Entry<Integer, Double>> successorStates = new ArrayList<Entry<Integer, Double>>();
		Iterator<Entry<Integer, Double>> tIter = pol.getTransitionsIterator(state, choice);
		while (tIter.hasNext()) {
			Entry<Integer, Double> currIter = tIter.next();
			successorStates.add(currIter);
		}

		return successorStates;
	}

	public Object[] getTeamDAProgress(int firstRobot, State[] lastStates, State[] currentStates, Object[] daInitStates,
			int robotDAassoc[]) {
		// DA States
		// start from the first robot
		// ArrayList<Object[]> robotsProgress = new ArrayList<Object[]>();

		Object[] teamProgress = null;
		int currentRobot = firstRobot;
		int prevRobot = firstRobot;
		do {

			Object[] updatedState = getRobotDAProgress(lastStates[prevRobot], currentStates[currentRobot], daInitStates,
					robotDAassoc, currentRobot);
			if (currentRobot == firstRobot)
				teamProgress = StatesHelper.ORIntegers(updatedState,
						StatesHelper.getDAStatesFromState(currentStates[firstRobot]), daInitStates);
			// robotsProgress.add(updatedState);
			prevRobot = currentRobot;
			currentRobot = (currentRobot + 1) % this.nRobots;
			teamProgress = StatesHelper.ORIntegers(teamProgress, updatedState, daInitStates);
		} while (currentRobot != firstRobot);

		return teamProgress;
	}

	public Object[] getTeamDAProgress(MDPSimple[] pols, int firstRobot, int[] currentStates, int[] lastStates,
			Object[] initStates, int robotDAassoc[]) {

		State[] lastStatesStates = new State[currentStates.length];
		State[] currentStatesStates = new State[currentStates.length];

		for (int i = 0; i < nRobots; i++) {
			lastStatesStates[i] = pols[i].getStatesList().get(lastStates[i]);
			currentStatesStates[i] = pols[i].getStatesList().get(currentStates[i]);

		}
		Object[] teamProgress = getTeamDAProgress(firstRobot, lastStatesStates, currentStatesStates, initStates,
				robotDAassoc);
		return teamProgress;
	}

	public Object[] getTeamSharedStatesProgress(MDPSimple pols[], int firstRobot, int[] currentStates, int[] lastStates,
			int[] initStates) {
		State[] lastStatesStates = new State[lastStates.length];
		State[] currentStatesStates = new State[currentStates.length];
		State[] initStatesStates = new State[initStates.length];
		for (int i = 0; i < nRobots; i++) {
			lastStatesStates[i] = pols[i].getStatesList().get(lastStates[i]);
			currentStatesStates[i] = pols[i].getStatesList().get(currentStates[i]);
			initStatesStates[i] = pols[i].getStatesList().get(initStates[i]);

		}
		Object[] sharedProgress = getTeamSharedStatesProgress(firstRobot, lastStatesStates, currentStatesStates,
				initStatesStates);
		return sharedProgress;
	}

	private Object[] getTeamSharedStatesProgress(int firstRobot, State[] lastStatesStates, State[] currentStatesStates,
			State[] initStatesStates) {

		Object[] teamProgress = null;
		int currentRobot = firstRobot;
		int prevRobot = firstRobot;
		Object[] initState = StatesHelper.getSharedStatesFromState(initStatesStates[firstRobot], this.teamMDPVarlist,
				this.sharedVarsList);

		if (initState != null) {
			Object[] firsRobotState = StatesHelper.getSharedStatesFromState(currentStatesStates[firstRobot],
					this.teamMDPVarlist, this.sharedVarsList);
			teamProgress = firsRobotState;
			prevRobot = currentRobot;
			currentRobot = (currentRobot + 1) % this.nRobots;
			while (currentRobot != firstRobot) {

				Object[] updatedState = getRobotSharedStateProgress(lastStatesStates[prevRobot],
						currentStatesStates[currentRobot], initStatesStates[firstRobot]);
				prevRobot = currentRobot;
				currentRobot = (currentRobot + 1) % this.nRobots;
				teamProgress = StatesHelper.ORIntegers(teamProgress, updatedState, initState);
			}
		}
		return teamProgress;
	}

	public int hasFailState(int[] mdpStates) {
		int sum = 0;
		for (int i = 0; i < mdpStates.length; i++) {
			if (mdpStates[i] == StatesHelper.failState)
				sum++;
		}
		return sum;
	}

	public int hasFailState(Object[] mdpStates, int start, int end) { // TODO: fix this for multiple states
		int sum = 0;
		for (int i = start; i < end; i++) {
			if ((int) mdpStates[i] == StatesHelper.failState)
				sum++;
		}
		return sum;
	}

	public int hasFailState(State jointState) {
		Object[] mdpStates = jointState.varValues;
		int start = numDA + this.sharedVarsList.size();// mdpStates.length - nRobots;
		int end = mdpStates.length;
		return hasFailState(mdpStates, start, end);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#hashCode()
	 */
	@Override
	public int hashCode() {
		// TODO Auto-generated method stub
		return super.hashCode();
	}

	// private void incrementSteps(int[] currentSteps, int[] maxSteps) {
	// int inc = 1;
	// for (int i = 0; i < currentSteps.length; i++) {
	// if (currentSteps[i] + inc < maxSteps[i]) {
	// currentSteps[i] += inc;
	// }
	// }
	// }

	public boolean isAcceptingState(Object[] varvalues, int start, int end, BitSet[] acceptingStates) {
		boolean acceptingState = true;
		if (end <= start) {
			acceptingState = false;
		}
		for (int i = start; i < end; i++) {
			if (!acceptingStates[i].get((int) varvalues[i])) {
				acceptingState = false;
				break;
			}
		}
		return acceptingState;
	}

	public boolean isAcceptingState(State jointState, BitSet[] acceptingStates) {
		int daStart = 0;
		int daEnd = numDA;// jointState.varValues.length - nRobots;
		return isAcceptingState(jointState.varValues, daStart, daEnd, acceptingStates);
	}

	private State resetTasksInIntermediateStates(State jointState, SequentialTeamMDP seqTeamMDP, int[] robotDAassoc) {

		// for each DA element
		Object[] startStatesArr = getDAStartStates(seqTeamMDP);
		BitSet[] accStatesArr = getDAGoalStates(seqTeamMDP);
		// boolean changed = false;
		State newJointState = null;
		for (int i = 0; i < numDA; i++) {
			int sval = StatesHelper.getIndexValueFromState(jointState, i);
			if (sval != (int) startStatesArr[i] && !accStatesArr[i].get(sval)) {
				if (robotDAassoc[i] != -1) {
					if (getRobotState(jointState, robotDAassoc[i]) == StatesHelper.failState) {
						mainLog.println("Resetting State for DA " + i + " robot " + robotDAassoc[i]);
						if (newJointState == null)
							newJointState = new State(jointState);
						// reset it
						newJointState.setValue(i, startStatesArr[i]);
						// changed = true;
					}

				}
			}
		}

		return newJointState;

	}

	public void saveJointPolicy() {
		// String saveplace =
		// "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
		//
		// PrismLog log = new PrismFileLog(saveplace + "jointPolicy.dot");
		// mdp.exportToDotFile(log, null, true);
		// log.close();
		StatesHelper.saveMDP(mdp, null, "", "jointPolicy", true);
		StatesHelper.saveMDPstatra(mdp, "", "jointPolicy", true);
		StatesHelper.saveHashMap(probsVector, "", "jointPolicyProbs.lab", true);
		StatesHelper.saveBitSet(allTasksCompletedStates, "", "jointPolicy.acc", true);
		StatesHelper.saveBitSet(this.allFailStatesSeen, "", "jointPolicy.failstates", true);
		StatesHelper.saveBitSet(this.deadendStates, "", "jointPolicy.deadends", true);

	}

	public void setNumDA(int n) {
		numDA = n;
	}

	public void testQ() {
		PriorityQueue<StateProb> testQ = new PriorityQueue<StateProb>();
		double qvals[] = { 0.5, 0.1, 0.2, 1.4, 0.02 };
		for (int i = 0; i < qvals.length; i++) {
			testQ.add(new StateProb(i, qvals[i]));
			mainLog.println("Added " + testQ.peek().toString());
		}
		while (!testQ.isEmpty()) {
			StateProb meh = testQ.remove();
			mainLog.println("Removed " + meh.toString());
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return super.toString();
	}

	public void updateInitialStates(MDPSimple[] pols, int[][] mdpMaps, int firstRobot, BitSet[] acceptingStates,
			List<State> teamMDPStates, BitSet teamAccStates, Vector<BitSet> path) {

		// note to self - being lazy pays no one
		// do it right the first time!

		// basically sometimes a robot might not be used at all
		// so what we want to be able to do is like update it's possible state
		// how can we figure this out - if we don't have a policy for this robot
		// that means it has just one state and no action
		// boolean doneOnce = false;
		boolean doPath = (path != null);
		int currentRobotState;
		boolean fixInitialState;
		for (int i = 0; i < nRobots; i++) {
			if (doPath) {
				fixInitialState = (path.get(i).cardinality() == 0);
			} else
				fixInitialState = ((pols[i].getNumStates() == 1)
						&& (pols[i].getNumChoices(pols[i].getFirstInitialState()) == 0) && (i != firstRobot));

			if (fixInitialState) {

				currentRobotState = pols[i].getFirstInitialState();

				// get the last state of the i-1 thing
				// unless you're at 0 in which case you need to go to nRobots-1
				int prevRobot = i - 1;
				if (prevRobot == -1) {
					prevRobot = nRobots - 1;
				}
				// if (!doneOnce)
				// doneOnce = true;
				// else {
				// mainLog.println("Something Unexpected is happening");
				// }
				// now I get the last state
				int prevRobotLastState = getLastState(pols[prevRobot], acceptingStates[prevRobot], path.get(prevRobot)); // FIXME:
																															// CHANGE
																															// THIS
																															// ASAP
																															// FATMA

				// now make a new state from the last state of the previous
				// robot and this state
				// more maslay!!! like how do I know this is the real state
				// what if we have more than one last state ??
				// which one do we choose ? how ?
				// we choose the accepting state
				// because thats the only reason this robot does not have a thing
				// or well it failed in all cases in which case we choose the fail state
				// I have fixed these maslay - we do a priority thing where we care about
				// the last accepting state and if we find no accepting state the last switch
				// and if we find no switch we care about the last fail state
				// so we're really chilling
				// so now that we have this what do we do ?
				// we make that state
				// have to make sure this new state is part of the team automaton btw

				int newState = StatesHelper.getMergedStateRobotMDP(
						pols[prevRobot].getStatesList().get(prevRobotLastState),
						pols[i].getStatesList().get(currentRobotState), teamMDPStates);
				StatesHelper.addStateMDP(pols[i], mdpMaps[i], teamMDPStates, StatesHelper.BADVALUE, newState,
						acceptingStates[i], teamAccStates);
				pols[i].clearInitialStates();
				pols[i].addInitialState(mdpMaps[i][newState]);
				path.get(i).set(mdpMaps[i][newState]);

			}
		}
	}

}
