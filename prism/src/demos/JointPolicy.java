package demos;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import explicit.Distribution;
import explicit.MDPSimple;
import parser.State;
import prism.PrismException;
import prism.PrismLog;

public class JointPolicy {
	int numRobots;
	int numTasks;
	int numSharedStates;
	ArrayList<String> sharedStatesNamesList;
	ArrayList<String> isolatedStatesNamesList;
	PrismLog mainLog;

	protected MDPSimple jointMDP;
	protected HashMap<State, HashMap<Object, Integer>> stateActionVisits;
	protected HashMap<State, HashMap<Object, Double>> stateActionQvalues;
	protected HashMap<State, Integer> stateIndices;
	protected HashMap<State, HashMap<Object, Integer>> stateActionIndices;
	protected BitSet visited;

	protected BitSet accStates;

	public void setAsAccState(State state) {
		int stateIndex = getStateIndex(state);
		accStates.set(stateIndex);
	}

	public JointPolicy(PrismLog log, int numRobots, int numTasks, int numSharedStates,
			ArrayList<String> sharedStatesNamesList, ArrayList<String> isolatedStatesNamesList) {
		mainLog = log;
		visited = new BitSet();
		this.sharedStatesNamesList = sharedStatesNamesList;
		this.isolatedStatesNamesList = isolatedStatesNamesList;
		this.numRobots = numRobots;
		this.numTasks = numTasks;
		this.numSharedStates = numSharedStates;
		jointMDP = new MDPSimple();
		stateActionVisits = new HashMap<State, HashMap<Object, Integer>>();
		stateActionQvalues = new HashMap<State, HashMap<Object, Double>>();
		stateIndices = new HashMap<State, Integer>();
		stateActionIndices = new HashMap<State, HashMap<Object, Integer>>();
		jointMDP.setStatesList(new ArrayList<State>());
		accStates = new BitSet();
	}

	int addState(State state) {
		int stateInd = getStateIndex(state);
		if (stateIndices.containsKey(state)) {
			stateInd = stateIndices.get(state);
		} else {
			stateInd = jointMDP.getNumStates();
			stateIndices.put(state, stateInd);
			jointMDP.addState();
			jointMDP.getStatesList().add(state);
		}
		return stateInd;
	}

	int getStateIndex(State state) {
		int stateInd = -1;
		if (stateIndices.containsKey(state)) {
			stateInd = stateIndices.get(state);
		}
		return stateInd;
	}

	int getActionIndex(State state, Object action) {
		int actionInd = -1;
		if (stateActionIndices.containsKey(state)) {
			if (stateActionIndices.get(state).containsKey(action)) {
				actionInd = stateActionIndices.get(state).get(action);
			}
		}
		return actionInd;
	}

	int addAction(State state, Object action, State succState, double prob) {
		int actionInd = getActionIndex(state, action);
		int stateInd = -1;
		Distribution distr;
		if (actionInd == -1) {
			stateInd = addState(state);
			if (stateInd != -1) {
				int numActions = this.jointMDP.getNumChoices(stateInd);
				if (stateActionIndices.containsKey(state)) {

					stateActionIndices.get(state).put(action, numActions);
				} else {
					stateActionIndices.put(state, new HashMap<Object, Integer>());
					stateActionIndices.get(state).put(action, numActions);
				}

			}
			distr = new Distribution();
			int succStateIndex = addState(succState);
			distr.add(succStateIndex, prob);
			jointMDP.addActionLabelledChoice(stateInd, distr, action);

		} else {
			// TODO check if this works
			// because it might not

			stateInd = getStateIndex(state);
			int succStateIndex = addState(succState);
			// only add this new state if it is not there already
			distr = jointMDP.getChoice(stateInd, actionInd);
			if (!distr.contains(succStateIndex))
				jointMDP.getChoice(stateInd, actionInd).add(succStateIndex, prob);
		}
		return actionInd;

	}

	int addAction(State state, Object action, List<State> succStates, List<Double> probs) {
		int actionInd = getActionIndex(state, action);
		int stateInd = -1;
		Distribution distr;
		if (actionInd == -1) {
			stateInd = addState(state);
			if (stateInd != -1) {
				int numActions = this.jointMDP.getNumChoices(stateInd);
				if (stateActionIndices.containsKey(state)) {

					stateActionIndices.get(state).put(action, numActions);
				} else {
					stateActionIndices.put(state, new HashMap<Object, Integer>());
					stateActionIndices.get(state).put(action, numActions);
				}

			}
			distr = new Distribution();
			for (int i = 0; i < succStates.size(); i++) {
				int succStateIndex = addState(succStates.get(i));
				distr.add(succStateIndex, probs.get(i));
			}
			jointMDP.addActionLabelledChoice(stateInd, distr, action);

		} else {
			// TODO check if this works
			// because it might not

			stateInd = getStateIndex(state);
			for (int i = 0; i < succStates.size(); i++) {
				int succStateIndex = addState(succStates.get(i));
				// only add this new state if it is not there already
				distr = jointMDP.getChoice(stateInd, actionInd);
				if (!distr.contains(succStateIndex))
					jointMDP.getChoice(stateInd, actionInd).add(succStateIndex, probs.get(i));
			}
		}
		return actionInd;

	}

	int getNumVisits(State state, Object action) {
		int visits = 0;
		if (stateActionVisits.containsKey(state)) {
			if (stateActionVisits.get(state).containsKey(action)) {
				visits = stateActionVisits.get(state).get(action);

			}

		}
		return visits;
	}

	int getNumVisits(State state) {
		int visits = 0;
		if (stateActionVisits.containsKey(state)) {
			for (Object act : stateActionVisits.get(state).keySet()) {
				visits += stateActionVisits.get(state).get(act);
			}
		}
		return visits;
	}

	int increaseVisits(State state, Object action) {
		int visits = 0;
		if (stateActionVisits.containsKey(state)) {
			if (stateActionVisits.get(state).containsKey(action)) {
				visits = stateActionVisits.get(state).get(action);

			}

		} else {
			stateActionVisits.put(state, new HashMap<Object, Integer>());
		}
		stateActionVisits.get(state).put(action, visits + 1);
		return (visits + 1);

	}

	void initialiseVisits(State state, Object action) throws PrismException {
		int visits = 0;
		if (!stateActionVisits.containsKey(state)) {
			stateActionVisits.put(state, new HashMap<Object, Integer>());
		}
		if (!stateActionVisits.get(state).containsKey(action)) {

			stateActionVisits.get(state).put(action, visits);
		}
		int stateInd = getStateIndex(state);
		if (stateInd == -1)
			throw new PrismException("State not added before initialising visits");
		else
			visited.set(stateInd);
	}

	boolean stateVisited(State state) {
		boolean visit = false;
		int stateInd = getStateIndex(state);
		if (stateInd != -1) {
			visit = visited.get(stateInd);
		}
		return visit;
	}

	double getQvalue(State state, Object action) {
		double qvalue = 0;
		if (stateActionQvalues.containsKey(state)) {
			if (stateActionQvalues.get(state).containsKey(action)) {
				qvalue = stateActionQvalues.get(state).get(action);
			} else {
				stateActionQvalues.get(state).put(action, qvalue);
			}
		} else {
			stateActionQvalues.put(state, new HashMap<Object, Double>());
			stateActionQvalues.get(state).put(action, qvalue);
		}
		return qvalue;
	}

	void updateQValue(State state, Object action, double qvalue) {

		if (!stateActionQvalues.containsKey(state)) {
			stateActionQvalues.put(state, new HashMap<Object, Double>());

		}
		stateActionQvalues.get(state).put(action, qvalue);

	}

	protected void generateCombinations(int counter[], int original[], ArrayList<int[]> res) throws PrismException {
		int numP = generateCombinations(counter, 0, original.length - 1, original, 0, res);
		int estimatedC = getNumberOfCombinations(original);
		if (res.size() != estimatedC) {
			mainLog.println(
					"ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
			throw new PrismException(
					"ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
		}
	}

	int getNumberOfCombinations(int[] arr) {
		int num = 1;
		for (int i = 0; i < arr.length; i++)
			num *= arr[i];
		return num;
	}

	int generateCombinations(int[] arr, int start, int end, int[] orig, int numC, ArrayList<int[]> res) {
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

	public Object getBestAction(State state, boolean minimizeCost) {
		int numActions = 0;
		int numActionsZeroQ = 0;
		Object bestAction = null;
		double cost = 0;
		double minCost = Double.MAX_VALUE;
		if (!minimizeCost)
			minCost = 0;
		boolean costCheck;
		double exploration_bias = 0.5;
		for (Object act : stateActionVisits.get(state).keySet()) {
			numActions++;
			// adding 1 so that I dont get undefined
			double lnStateVisits = Math.log(getNumVisits(state) + 1);
			double stateActionVisits = Math.log(getNumVisits(state, act) + 1);
			double exploration_term = lnStateVisits / stateActionVisits;
			if (Double.isNaN(exploration_term)) {
				exploration_term = 0;
			}
			double sqrt = Math.sqrt(exploration_term);
			cost = getQvalue(state, act) - exploration_bias * sqrt;
			if (cost == 0) {
				numActionsZeroQ++;
			}
			if (minimizeCost)
				costCheck = cost < minCost;
			else
				costCheck = cost > minCost;

			if (costCheck) {
				minCost = cost;
				bestAction = act;
			}
		}
		if (numActionsZeroQ == numActions) {
			// then pick something at random
			Random rand = new Random();
			int choice = rand.nextInt(numActions);
			bestAction = (stateActionVisits.get(state).keySet().toArray())[choice];
		}

		return bestAction;

	}

	HashMap<State, Object> getBestPolicySoFar(boolean minimiseCosts) {
		HashMap<State, Object> bestPolicySoFar = new HashMap<State, Object>();
		for (State s : stateActionQvalues.keySet()) {
			Object action = getBestAction(s, minimiseCosts);
			bestPolicySoFar.put(s, action);
		}
		return bestPolicySoFar;
	}

	public void printStateDetails(State state) {
		// print everything about this state
		String visits = "v:" + getNumVisits(state);
		String actionvisits = "";
		String qvalues = "";
		for (Object a : stateActionVisits.get(state).keySet()) {
			actionvisits += " ";
			actionvisits += a.toString() + ":" + getNumVisits(state, a);
			qvalues += a.toString() + ":" + getQvalue(state, a);

		}
		mainLog.println("State Info\n" + state);
		mainLog.println("Visits - " + visits + actionvisits);
		mainLog.println("Q Values - " + qvalues);

	}
}
