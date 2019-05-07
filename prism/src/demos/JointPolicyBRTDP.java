package demos;

import java.io.FileWriter;
import java.io.IOException;
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
import java.util.Queue;
import java.util.Random;
import java.util.Set;

import explicit.Distribution;
import explicit.MDPSimple;
import parser.State;
import prism.PrismException;
import prism.PrismLog;

public class JointPolicyBRTDP
{
	int numRobots;
	int numTasks;
	int numSharedStates;
	ArrayList<String> sharedStatesNamesList;
	ArrayList<String> isolatedStatesNamesList;
	PrismLog mainLog;

	protected MDPSimple jointMDP;

	protected HashMap<State, Double> stateValuesProbUpperBound;
	protected HashMap<State, Double> stateValuesProbLowerBound;

	protected HashMap<State, Double> stateValuesCostUpperBound;
	protected HashMap<State, Double> stateValuesCostLowerBound;

	protected HashMap<State, Integer> stateIndices;
	protected HashMap<State, HashMap<Object, Integer>> stateActionIndices;
	protected BitSet visited;
	protected HashMap<State, HashMap<Object, ArrayList<Integer>>> robotStateActionIndices;
	protected HashMap<State, HashMap<Object, Double>> stateActionCosts;
	protected BitSet accStates;
	protected BitSet leafStates;
	double explorationBias = 10;
	private double defaultProbLowerQ = 0.0;
	private double defaultProbUpperQ = 1.0;
	private double defaultCostLowerQ = 0.0;
	List<Double> dadists;

	public void setAsAccState(State state)
	{
		int stateIndex = getStateIndex(state);
		if (stateIndex != -1)
			accStates.set(stateIndex);
	}

	public void setAsLeafState(State state)
	{
		int stateIndex = getStateIndex(state);
		if (stateIndex != -1) {
			if (!leafStates.get(stateIndex))
				leafStates.set(stateIndex);
		}
	}

	public void setAsLeafState(ArrayList<State> states)
	{
		for (State s : states)
			setAsLeafState(s);
	}

	public void removeFromLeafState(State state)
	{
		int stateIndex = getStateIndex(state);
		if (stateIndex != -1) {
			if (leafStates.get(stateIndex))
				leafStates.set(stateIndex, false);
		}
	}

	public void removeFromLeafState(ArrayList<State> states)
	{
		for (State s : states)
			removeFromLeafState(s);
	}

	public JointPolicyBRTDP(PrismLog log, int numRobots, int numTasks, int numSharedStates, ArrayList<String> sharedStatesNamesList,
			ArrayList<String> isolatedStatesNamesList, List<Double> da_dists)
	{

		mainLog = log;
		robotStateActionIndices = new HashMap<State, HashMap<Object, ArrayList<Integer>>>();
		visited = new BitSet();
		this.sharedStatesNamesList = sharedStatesNamesList;
		this.isolatedStatesNamesList = isolatedStatesNamesList;
		this.numRobots = numRobots;
		this.numTasks = numTasks;
		this.numSharedStates = numSharedStates;
		jointMDP = new MDPSimple();
		stateIndices = new HashMap<State, Integer>();
		stateActionIndices = new HashMap<State, HashMap<Object, Integer>>();
		jointMDP.setStatesList(new ArrayList<State>());
		accStates = new BitSet();
		leafStates = new BitSet();
		this.stateValuesProbUpperBound = new HashMap<State, Double>();
		this.stateValuesProbLowerBound = new HashMap<State, Double>();
		stateValuesCostUpperBound = new HashMap<State, Double>();
		stateValuesCostLowerBound = new HashMap<State, Double>();
		dadists = da_dists;
		stateActionCosts = new HashMap<State, HashMap<Object, Double>>();
	}

	int addState(State state)
	{
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

	int getStateIndex(State state)
	{
		int stateInd = -1;
		if (stateIndices.containsKey(state)) {
			stateInd = stateIndices.get(state);
		}
		return stateInd;
	}

	int getActionIndex(State state, Object action)
	{
		int actionInd = -1;
		if (stateActionIndices.containsKey(state)) {
			if (stateActionIndices.get(state).containsKey(action)) {
				actionInd = stateActionIndices.get(state).get(action);
			}
		}
		return actionInd;
	}

	public boolean addRobotStateActionIndices(State jointstate, Object jointAction, ArrayList<Object> actionList, int[] actionsIndices)
	{
		boolean added = false;
		if (!robotStateActionIndices.containsKey(jointstate)) {
			robotStateActionIndices.put(jointstate, new HashMap<Object, ArrayList<Integer>>());
		}
		if (!robotStateActionIndices.get(jointstate).containsKey(jointAction)) {
			// now we make the arraylist
			ArrayList<Integer> actionIndices = new ArrayList<Integer>();
			for (int i = 0; i < numRobots; i++) {
				int actionIndex = (actionsIndices[i]);
				actionIndices.add(actionIndex);
			}
			robotStateActionIndices.get(jointstate).put(jointAction, actionIndices);
			added = true;

		}
		return added;

	}

	public boolean addRobotStateActionIndices(State jointstate, Object jointAction, ArrayList<Object> actionList,
			HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState)
	{
		boolean added = false;
		if (!robotStateActionIndices.containsKey(jointstate)) {
			robotStateActionIndices.put(jointstate, new HashMap<Object, ArrayList<Integer>>());
		}
		if (!robotStateActionIndices.get(jointstate).containsKey(jointAction)) {
			// now we make the arraylist
			ArrayList<Integer> actionIndices = new ArrayList<Integer>();
			for (int i = 0; i < numRobots; i++) {
				int actionIndex = allPossibleActionsForState.get(i).get(actionList.get(i));
				actionIndices.add(actionIndex);
			}
			robotStateActionIndices.get(jointstate).put(jointAction, actionIndices);
			added = true;

		}
		return added;

	}

	boolean isGoal(State state)
	{
		int stateIndex = this.getStateIndex(state);

		// get the da state
		State dastate = state.substate(this.numRobots, this.numRobots + 1);
		dastate = (State) dastate.varValues[0];
		int dastateint = (int) dastate.varValues[0];
		double dist = this.dadists.get(dastateint);
		if (dist == 0) {

			accStates.set(stateIndex);
			return true;
		}
		return false;
	}

	int addAction(State state, Object action, State succState, double prob)
	{
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

	void addActionReward(State state, Object action, double reward) throws PrismException
	{

		if (stateActionCosts.containsKey(state)) {
			if (!stateActionCosts.get(state).containsKey(action))
				stateActionCosts.get(state).put(action, reward);
			else
				throw new PrismException("Trying to add same state action for reward!!");

		} else {
			HashMap<Object, Double> actionReward = new HashMap<Object, Double>();
			actionReward.put(action, reward);
			stateActionCosts.put(state, actionReward);
		}
	}

	int addAction(State state, Object action, List<State> succStates, List<Double> probs)
	{
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
			// we're exploring all successors so we should not have to do this.
			// this makes no sense to me.
			// why does this even happen ???
			stateInd = getStateIndex(state);
			mainLog.println("Action " + action.toString());
			mainLog.println("State " + state.toString());
			distr = jointMDP.getChoice(stateInd, actionInd);
			Iterator<Entry<Integer, Double>> dist_list = distr.iterator();
			while (dist_list.hasNext()) {
				Entry<Integer, Double> sa = dist_list.next();
				State sas = jointMDP.getStatesList().get(sa.getKey());
				mainLog.println("succ: " + sas.toString() + ", " + sa.getValue());
			}
			for (int i = 0; i < succStates.size(); i++) {
				int succStateIndex = addState(succStates.get(i));
				// only add this new state if it is not there already

				if (!distr.contains(succStateIndex))
					jointMDP.getChoice(stateInd, actionInd).add(succStateIndex, probs.get(i));
			}
		}
		return actionInd;

	}

	protected State simulateAction(State state, Object action)
	{
		int state_index = -1;
		if (stateActionIndices.containsKey(state))
			state_index = stateIndices.get(state);
		if (state_index != -1) {
			if (stateActionIndices.get(state).containsKey(action)) {
				int action_choice = stateActionIndices.get(state).get(action);
				int succ_state_index = -1;
				double prob = (new Random()).nextDouble();
				double cum_prob = 0;
				Iterator<Entry<Integer, Double>> iter = jointMDP.getTransitionsIterator(state_index, action_choice);
				while (iter.hasNext()) {
					Entry<Integer, Double> entry = iter.next();
					cum_prob += entry.getValue();
					if (cum_prob >= prob) {
						succ_state_index = entry.getKey();
						break;
					}

				}

				if (succ_state_index != -1) {
					return jointMDP.getStatesList().get(succ_state_index);
				}
			} else
				return null;
		}

		return null;

	}

	protected HashMap<State, Double> getSuccessors(State state, Object action)
	{
		int state_index = -1;

		if (stateActionIndices.containsKey(state))

			state_index = stateIndices.get(state);

		if (state_index != -1) {
			if (stateActionIndices.get(state).containsKey(action)) {
				HashMap<State, Double> succStates = new HashMap<State, Double>();
				int action_choice = stateActionIndices.get(state).get(action);
				int succ_state_index = -1;
				double prob = 0.0;
				Iterator<Entry<Integer, Double>> iter = jointMDP.getTransitionsIterator(state_index, action_choice);
				while (iter.hasNext()) {
					Entry<Integer, Double> entry = iter.next();
					succ_state_index = entry.getKey();
					prob = entry.getValue();

					if (succ_state_index != -1) {
						State succ_state = jointMDP.getStatesList().get(succ_state_index);
						succStates.put(succ_state, prob);
					}
				}
				return succStates;
			} else
				return null;
		}

		return null;

	}

	//	protected boolean toExpand(State state, boolean defaultPolicyNotRandom)
	//	{
	//		boolean expand = false;
	//		if (defaultPolicyNotRandom) {
	//			if (getNumVisits(state) > 0)
	//				return false;
	//			else
	//				return true;
	//		} else {
	//			if (stateActionVisits.containsKey(state)) {
	//				for (Object act : stateActionVisits.get(state).keySet()) {
	//					if (stateActionVisits.get(state).get(act) == 0) {
	//						expand = true;
	//						break;
	//					}
	//				}
	//			} else {
	//				expand = true;
	//			}
	//			return expand;
	//		}
	//	}
	//
	//	int getNumVisits(State state, Object action)
	//	{
	//		int visits = 0;
	//		if (stateActionVisits.containsKey(state)) {
	//			if (stateActionVisits.get(state).containsKey(action)) {
	//				visits = stateActionVisits.get(state).get(action);
	//
	//			}
	//
	//		}
	//		return visits;
	//	}
	//
	//	int getNumVisits(State state)
	//	{
	//		int visits = 0;
	//		if (stateActionVisits.containsKey(state)) {
	//			for (Object act : stateActionVisits.get(state).keySet()) {
	//				visits += stateActionVisits.get(state).get(act);
	//			}
	//		}
	//		return visits;
	//	}
	//
	//	int increaseVisits(State state, Object action)
	//	{
	//		int visits = 0;
	//		if (stateActionVisits.containsKey(state)) {
	//			if (stateActionVisits.get(state).containsKey(action)) {
	//				visits = stateActionVisits.get(state).get(action);
	//
	//			}
	//
	//		} else {
	//			stateActionVisits.put(state, new HashMap<Object, Integer>());
	//		}
	//		stateActionVisits.get(state).put(action, visits + 1);
	//		return (visits + 1);
	//
	//	}
	//
	//	void initialiseVisits(State state, Object action) throws PrismException
	//	{
	//		int visits = 0;
	//		if (!stateActionVisits.containsKey(state)) {
	//			stateActionVisits.put(state, new HashMap<Object, Integer>());
	//		}
	//		if (!stateActionVisits.get(state).containsKey(action)) {
	//
	//			stateActionVisits.get(state).put(action, visits);
	//		}
	//		int stateInd = getStateIndex(state);
	//		if (stateInd == -1)
	//			throw new PrismException("State not added before initialising visits");
	//		else
	//			visited.set(stateInd);
	//	}
	//
	//	void initialiseVisitsNoVisit(State state, Object action) throws PrismException
	//	{
	//		int visits = 0;
	//		if (!stateActionVisits.containsKey(state)) {
	//			stateActionVisits.put(state, new HashMap<Object, Integer>());
	//		}
	//		if (!stateActionVisits.get(state).containsKey(action)) {
	//
	//			stateActionVisits.get(state).put(action, visits);
	//		}
	//
	//	}
	//
	//	boolean stateVisited(State state)
	//	{
	//		boolean visit = false;
	//		int stateInd = getStateIndex(state);
	//		if (stateInd != -1) {
	//			visit = visited.get(stateInd);
	//		}
	//
	//		return visit;
	//	}

	//	double getQvalue(State state, Object action, boolean upperBound)
	//	{
	//		if (upperBound)
	//			return getQvalueUpper(state, action);
	//		else
	//			return getQvalueLower(state, action);
	//	}
	//
	//	double getQvalueUpper(State state, Object action)
	//	{
	//
	//		//prob only so max prob = 1
	//		double qvalue = this.defaultUpperQ;
	//		if (stateActionQupper.containsKey(state)) {
	//			if (stateActionQupper.get(state).containsKey(action)) {
	//				qvalue = stateActionQupper.get(state).get(action);
	//			} else {
	//				stateActionQupper.get(state).put(action, qvalue);
	//			}
	//		} else {
	//			stateActionQupper.put(state, new HashMap<Object, Double>());
	//			stateActionQupper.get(state).put(action, qvalue);
	//		}
	//		return qvalue;
	//	}
	//
	//	double getQvalueLower(State state, Object action)
	//	{
	//		//prob only so min prob = 0 
	//		double qvalue = this.defaultLowerQ;
	//		if (stateActionQlower.containsKey(state)) {
	//			if (stateActionQlower.get(state).containsKey(action)) {
	//				qvalue = stateActionQlower.get(state).get(action);
	//			} else {
	//				stateActionQlower.get(state).put(action, qvalue);
	//			}
	//		} else {
	//			stateActionQlower.put(state, new HashMap<Object, Double>());
	//			stateActionQlower.get(state).put(action, qvalue);
	//		}
	//		return qvalue;
	//	}
	//
	//	double getStateValue(State state, boolean minCost, boolean upperBound) throws PrismException
	//	{
	//		return (getStateValueAndAction(state, minCost, upperBound)).getKey();
	//	}
	//
	//	Entry<Double, Object> getStateValueAndAction(State state, boolean minCost, boolean upperBound) throws PrismException
	//	{
	//		HashMap<State, HashMap<Object, Double>> stateActionQ = this.stateActionQupper;
	//		if (!upperBound)
	//			stateActionQ = this.stateActionQlower;
	//		double valtoret = this.defaultUpperQ;
	//		if (!minCost)
	//			valtoret = this.defaultLowerQ;
	//		return getStateValueAndAction(state, minCost, stateActionQ, valtoret);
	//	}
	//
	//	Entry<Double, Object> getStateValueAndAction(State state, boolean minCost, HashMap<State, HashMap<Object, Double>> stateActionQ, double initval)
	//			throws PrismException
	//	{
	//
	//		Entry<Double, Object> toret = null;// new AbstractMap.SimpleEntry<Double,Object>(0.0,null);
	//		if (stateActionQ.containsKey(state)) {
	//			//find the minimum 
	//
	//			double valtoret = initval;
	//
	//			Object action = null;
	//			for (Object act : stateActionQ.get(state).keySet()) {
	//				double qval = getQvalue(state, action, true);
	//				if (action == null) {
	//					action = act;
	//					valtoret = qval;
	//				} else {
	//					boolean check;
	//					if (minCost)
	//						check = qval < valtoret;
	//					else
	//						check = qval > valtoret;
	//					if (check) {
	//
	//						valtoret = qval;
	//						action = act;
	//					}
	//				}
	//			}
	//			if (action == null)
	//				throw new PrismException("No action for state in " + " q value: " + state.toString());
	//
	//			toret = new AbstractMap.SimpleEntry<Double, Object>(valtoret, action);
	//
	//		} else {
	//			//has no actions for this state !!
	//			throw new PrismException("No state in q value: " + state.toString());
	//		}
	//		return toret;
	//	}
	//
	//	void updateQValueUpper(State state, Object action, double qvalue)
	//	{
	//		if (!stateActionQupper.containsKey(state)) {
	//			stateActionQupper.put(state, new HashMap<Object, Double>());
	//
	//		}
	//		stateActionQupper.get(state).put(action, qvalue);
	//	}
	//
	//	void updateQValueLower(State state, Object action, double qvalue)
	//	{
	//		if (!stateActionQlower.containsKey(state)) {
	//			stateActionQlower.put(state, new HashMap<Object, Double>());
	//
	//		}
	//		stateActionQlower.get(state).put(action, qvalue);
	//	}
	//
	//	public void updateQValue(State state, Object action, double qvalue, boolean upperBound)
	//	{
	//		if (upperBound)
	//			updateQValueUpper(state, action, qvalue);
	//		else
	//			updateQValueLower(state, action, qvalue);
	//	}
	//
	//	public double getQValueUpperLowerDiff(State state, Object action) throws PrismException
	//	{
	//		double qvalUpper = getQvalue(state, action, true);
	//		double qvalLower = getQvalue(state, action, false);
	//		double diff = (qvalUpper - qvalLower);
	//		if (diff < 0) {
	//			//waaa its negative hain ? 
	//			throw new PrismException("Difference between upper and lower bounds is negative, can not haapen " + state.toString() + " , " + action.toString());
	//		}
	//		return diff;
	//	}
	//
	//	public double getValueUpperLowerDiff(State state,boolean minCost) throws PrismException
	//	{
	//		double valUpper = this.getStateValue(state, minCost, true);
	//		double valLower = this.getStateValue(state, minCost, false);
	//		double diff= (valUpper-valLower); 
	//		if(diff < 0)
	//		{
	//			//waaa its negative hain ? 
	//			throw new PrismException("Difference between upper and lower bounds is negative, can not haapen "+state.toString());
	//		}
	//		return diff; 
	//	}

	public double getProbQValue(State state, Object action, double cost, boolean upperbound, boolean isgoal, boolean printVal) throws PrismException
	{
		HashMap<State, Double> succStates = this.getSuccessors(state, action);
		boolean self_loop = false;
		double sum = 0;
		if (succStates != null) {
			for (State s : succStates.keySet()) {
				double prob = succStates.get(s);
				if (prob == 1.0) {
					if (s.equals(state))
						self_loop = true;
				}
				sum += prob * getProbValue(s, upperbound, isGoal(s), self_loop, printVal);

			}
		} else {
			//hmmm we have a problem here maybe 
			throw new PrismException("State: " + state.toString() + " action: " + action.toString() + " has no successors!!");
		}
		//for probability it cant be cost + sum
		//can it be ? 
		//TODO: double check this 

		//return cost + sum;
		return sum; //cuz its probabilities so we dont need the cost 
	}

	public double getCostQValue(State state, Object action, double cost, boolean upperbound, boolean printVal) throws PrismException
	{
		HashMap<State, Double> succStates = this.getSuccessors(state, action);
		boolean self_loop = false; 
		double sum = 0;
		if (succStates != null) {
			for (State s : succStates.keySet()) {
				double prob = succStates.get(s);
				if (prob == 1.0) {
					if (s.equals(state))
						self_loop = true;
				}
				sum += prob * getCostValue(s, upperbound, isGoal(s), printVal);

			}
		} else {
			//hmmm we have a problem here maybe 
			throw new PrismException("State: " + state.toString() + " action: " + action.toString() + " has no successors!!");
		}
		//if this is a self_loop action then the cost should be disregarded 
		//this is a bit of a hack isnt it 
		//think about transient loops ??? 
		//lets come back to this 
		//TODO: Check EC stuff for this
		if(self_loop)
			cost = 0.0; 
		return cost + sum;

	}

	public double getCostValue(State state, boolean upperbound, boolean isgoal, boolean printVal)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesCostUpperBound;
		else
			stateV = this.stateValuesCostLowerBound;
		double valToRet = 0;
		//costs are an upper bound 
		if (!stateV.containsKey(state)) {
			if (upperbound) {
				valToRet = MRuctPaper.getInitCostBound(state);
				//because we're doing maxprob mincost 
				//I dont have to initialise anything it can just be 0
				//this is a bit of a dilemma - how do we do this cost stuff 

			} else {
				valToRet = this.defaultCostLowerQ;
			}
			stateV.put(state, valToRet);
		}
		valToRet = stateV.get(state);
		if (printVal) {
			String printText = state.toString() + "c:";
			if (upperbound)
				printText += "u";
			else
				printText += "l";

			printText += valToRet;
			mainLog.println(printText);
		}
		return valToRet;
	}

	public void setCostValue(State state, boolean upperbound, double cost)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesCostUpperBound;
		else
			stateV = this.stateValuesCostLowerBound;

		stateV.put(state, cost);

	}

	public double getProbValue(State state, boolean upperbound, boolean isgoal, boolean selfLoop, boolean printVal)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesProbUpperBound;
		else
			stateV = this.stateValuesProbLowerBound;
		double valToRet = -1;
		if (selfLoop) {
			valToRet = this.defaultProbLowerQ;
			if (!stateV.containsKey(state))
				stateV.put(state, valToRet);
			else {
				if (stateV.get(state) != valToRet)
					stateV.put(state, valToRet);
			}
		} else {
			if (stateV.containsKey(state)) {
				valToRet = stateV.get(state);
			}

			else {
				if (isgoal) {
					valToRet = this.defaultProbUpperQ;

				} else {

					if (upperbound) {

						valToRet = this.defaultProbUpperQ;
					} else {

						valToRet = MRuctPaper.getInitProbBound(state);
						//one robot performs worse than 2 so lower bound//this.defaultProbLowerQ;
					}

				}
				stateV.put(state, valToRet);
			}
		}
		//
		if (printVal) {
			String printText = state.toString() + "p:";
			if (upperbound)
				printText += "u";
			else
				printText += "l";

			printText += valToRet;
			mainLog.println(printText);
		}
		return valToRet;
	}

	public void setProbCostValue(State state, double probvalue, double costvalue, boolean upperbound)
	{
		setProbValue(state, probvalue, upperbound);
		setCostValue(state, upperbound, costvalue);
	}

	public void setProbValue(State state, double value, boolean upperbound)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesProbUpperBound;
		else
			stateV = this.stateValuesProbLowerBound;
		stateV.put(state, value);

	}

	double getProbValueIgnoreSelfLoop(State state, boolean upperbound, boolean isgoal, boolean printVal)
	{
		//setting the selfloop variable to false because we dont care 
		return getProbValue(state, upperbound, isgoal, false, printVal);
	}

	public double getProbValueDiff(State state, boolean isgoal, boolean printVal) throws PrismException //get the difference between the upper and lower bounds 
	{
		//boolean selfloop = false because we dont care
		double diff = getProbValueIgnoreSelfLoop(state, true, isgoal, printVal) - getProbValueIgnoreSelfLoop(state, false, isgoal, printVal);
		if (diff < 0)
			throw new PrismException("State diff negative!!! " + state.toString());
		return diff;
	}

	public Entry<Double, Double> getMinMaxValue(State state, double probCost, boolean minCost, boolean upperBound, boolean isgoal, boolean printVal)
			throws PrismException
	{
		//so basically we can get all the actions here 
		if (this.robotStateActionIndices.containsKey(state)) {
			ArrayList<Object> actions = new ArrayList<Object>();
			actions.addAll(robotStateActionIndices.get(state).keySet());

			return getMinMaxValue(state, actions, probCost, minCost, upperBound, isgoal, printVal);
		} else {
			throw new PrismException("state not in list!!" + state.toString());
		}
	}

	public Entry<Double, Double> getMinMaxValue(State state, double probCost, double cost, boolean minCost, boolean upperBound, boolean isgoal,
			boolean printVal) throws PrismException
	{
		//so basically we can get all the actions here 
		if (this.robotStateActionIndices.containsKey(state)) {
			ArrayList<Object> actions = new ArrayList<Object>();
			actions.addAll(robotStateActionIndices.get(state).keySet());
			return getMinMaxValue(state, actions, probCost, cost, minCost, upperBound, isgoal, printVal);
		} else {
			throw new PrismException("state not in list!!" + state.toString());
		}
	}

	double getStateActionCost(State state, Object act) throws PrismException
	{
		if (this.stateActionCosts.containsKey(state)) {
			if (this.stateActionCosts.get(state).containsKey(act)) {
				return this.stateActionCosts.get(state).get(act);
			}
		}
		throw new PrismException("No costs preset for " + state.toString() + " " + act.toString() + " Error!!");
	}

	public Entry<Double, Double> getMinMaxValue(State state, ArrayList<Object> actions, double probcost, boolean minCost, boolean upperBound, boolean isgoal,
			boolean printVal) throws PrismException
	{

		boolean setcost = true;
		double probVal = 0;
		double costVal = 0;
		//duplication doesnt matter here cuz its just a value 

		for (int i = 0; i < actions.size(); i++) {
			
			Object act = actions.get(i);
			//			double cost = costs.get(i);
			double cost = getStateActionCost(state, act);
			double probqval = getProbQValue(state, act, probcost, upperBound, isgoal, printVal);
			double costqval = getCostQValue(state, act, cost, upperBound, printVal);
			if (printVal)
			{	
				mainLog.println(actions.get(i).toString()+":Q["+probqval+","+costqval+"]");
			
			}
			if (setcost) {
				probVal = probqval;
				costVal = costqval;
				setcost = false;
			} else {
				boolean check;
				if (minCost)
					check = probVal > probqval;
				else
					check = probVal < probqval;
				if (check) {
					probVal = probqval;
					costVal = costqval;
				} else {
					if (probVal == probqval) {
						if (costVal > costqval) //choose the lowest cost here ???? //hmmm 
						{
							costVal = costqval;
						}
					}
				}

			}
		}
		return new AbstractMap.SimpleEntry<Double, Double>(probVal, costVal);

	}

	public Entry<Double, Double> getMinMaxValue(State state, ArrayList<Object> actions, double probcost, double cost, boolean minCost, boolean upperBound,
			boolean isgoal, boolean printVal) throws PrismException
	{

		boolean setcost = true;
		double probVal = 0;
		double costVal = 0;
		//duplication doesnt matter here cuz its just a value 

		for (int i = 0; i < actions.size(); i++) {
			Object act = actions.get(i);
			//			double cost = costs.get(i);
			double probqval = getProbQValue(state, act, probcost, upperBound, isgoal, printVal);
			double costqval = getCostQValue(state, act, cost, upperBound, printVal);
			if(printVal)
				mainLog.println(actions.get(i).toString()+":Q["+probqval+","+costqval+"]");
			if (setcost) {
				probVal = probqval;
				costVal = costqval;
				setcost = false;
			} else {
				boolean check;
				if (minCost)
					check = probVal > probqval;
				else
					check = probVal < probqval;
				if (check) {
					probVal = probqval;
					costVal = costqval;
				} else {
					if (probVal == probqval) {
						if (costVal > costqval) {
							costVal = costqval;
						}
					}
				}

			}
		}
		return new AbstractMap.SimpleEntry<Double, Double>(probVal, costVal);

	}

	public Object getMinMaxAction(State state, double probcost, boolean minCost, boolean upperBound, boolean printVal) throws PrismException

	{
		//so basically we can get all the actions here 
		if (this.robotStateActionIndices.containsKey(state)) {
			ArrayList<Object> actions = new ArrayList<Object>();
			actions.addAll(robotStateActionIndices.get(state).keySet());
			return getMinMaxAction(state, actions, probcost, minCost, upperBound, printVal);
		} else {
			throw new PrismException("state not in list!!" + state.toString());
		}
	}

	public Object getMinMaxAction(State state, ArrayList<Object> actions, double probcost, boolean minCost, boolean upperBound, boolean printVal)
			throws PrismException
	{

		double probVal = 0;
		double costVal = 0;
		Object actionRet = null;
		ArrayList<Object> duplicateRetVals = new ArrayList<Object>();
		ArrayList<Double> duplicateRetValsCost = new ArrayList<Double>();

		for (int i = 0; i < actions.size(); i++) {
			Object act = actions.get(i);
			
			//			double cost = costs.get(i);
			double cost = getStateActionCost(state, act);
			double probqval = getProbQValue(state, act, probcost, minCost, upperBound, printVal);
			double costqval = getCostQValue(state, act, cost, upperBound, printVal);
			if (printVal)
				mainLog.println(actions.get(i).toString()+":Q["+probqval+","+costqval+"]");
			if (actionRet == null) {
				probVal = probqval;
				actionRet = act;
				costVal = costqval;
				duplicateRetVals.add(act);
				duplicateRetValsCost.add(costqval);

			} else {
				boolean check;
				if (probVal == probqval) {

					//just minimise this cost 
					if (costVal > costqval) {
						probVal = probqval;
						actionRet = act;
						costVal = costqval;
					} else if (costVal == costqval) {
						duplicateRetVals.add(act);
						duplicateRetValsCost.add(costqval);
					}
				} else {
					if (minCost)
						check = probVal > probqval;
					else
						check = probVal < probqval;
					if (check) {
						probVal = probqval;
						costVal = costqval;
						actionRet = act;
					}
				}

			}
		}
		//		if (duplicateRetVals.size() > 1) {
		//			//randomly choose an action here 
		//			int actionChoice = new Random().nextInt(duplicateRetVals.size());
		//			actionRet = duplicateRetVals.get(actionChoice);
		//		}
		return actionRet;

	}

	public Object getMinMaxAction(State state, ArrayList<Object> actions, double probcost, double cost, boolean minCost, boolean upperBound, boolean printVal)
			throws PrismException
	{

		double probVal = 0;
		double costVal = 0;
		Object actionRet = null;
		ArrayList<Object> duplicateRetVals = new ArrayList<Object>();
		ArrayList<Double> duplicateRetValsCost = new ArrayList<Double>();

		for (int i = 0; i < actions.size(); i++) {
			Object act = actions.get(i);
			//			double cost = costs.get(i);
			double probqval = getProbQValue(state, act, probcost, minCost, upperBound, printVal);
			double costqval = getCostQValue(state, act, cost, upperBound, printVal);
			if (actionRet == null) {
				probVal = probqval;
				actionRet = act;
				costVal = costqval;
				duplicateRetVals.add(act);
				duplicateRetValsCost.add(costqval);

			} else {
				boolean check;
				if (probVal == probqval) {

					//just minimise this cost 
					if (costVal > costqval) {
						probVal = probqval;
						actionRet = act;
						costVal = costqval;
					} else if (costVal == costqval) {
						duplicateRetVals.add(act);
						duplicateRetValsCost.add(costqval);
					}
				} else {
					if (minCost)
						check = probVal > probqval;
					else
						check = probVal < probqval;
					if (check) {
						probVal = probqval;
						costVal = costqval;
						actionRet = act;
					}
				}

			}
		}
		//		if (duplicateRetVals.size() > 1) {
		//			//randomly choose an action here 
		//			int actionChoice = new Random().nextInt(duplicateRetVals.size());
		//			actionRet = duplicateRetVals.get(actionChoice);
		//		}
		return actionRet;

	}

	protected void generateCombinations(ArrayList<Integer> numActionsPerRobot, ArrayList<int[]> res) throws PrismException
	{
		int[] counter = new int[numActionsPerRobot.size()];
		for (int i = 0; i < counter.length; i++)
			counter[i] = numActionsPerRobot.get(i);
		int[] original = counter.clone();
		int numP = generateCombinations(counter, 0, original.length - 1, original, 0, res);
		int estimatedC = getNumberOfCombinations(original);
		if (res.size() != estimatedC) {
			mainLog.println("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
			throw new PrismException("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
		}
	}

	protected void generateCombinations(int counter[], int original[], ArrayList<int[]> res) throws PrismException
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

	public State getStateFromIndex(int ind)
	{
		State storet = null;
		for (State s : stateIndices.keySet()) {
			if (stateIndices.get(s) == ind) {
				storet = s;
				break;
			}

		}
		return storet;
	}

	public void exportAllStatesValues(String filename) throws IOException
	{
		FileWriter fileWriter = new FileWriter(filename);
		//just go over all the states we have and print their values 
		fileWriter.write("\n");
		for (State s : stateValuesProbUpperBound.keySet()) {
			fileWriter.write(getStateValues(s) + "\n");
		}
		fileWriter.close();
	}

	public void printAllStatesValues()
	{
		//just go over all the states we have and print their values 
		for (State s : stateValuesProbUpperBound.keySet()) {
			printStateValues(s);
		}
	}

	public void printStateValues(State state)
	{
		String toprint = getStateValues(state);
		mainLog.println(toprint);

	}

	public String getStateValues(State state)
	{
		boolean isgoal = isGoal(state);

		String toprint = state.toString() + "[P u:" + getProbValueIgnoreSelfLoop(state, true, isgoal, false) + ", l:"
				+ getProbValueIgnoreSelfLoop(state, false, isgoal, false) + "]";
		toprint += "[C u:" + getCostValue(state, true, isgoal, false) + ", l:" + getCostValue(state, false, isgoal, false) + "]";
		return toprint;

	}
	//
	//	public Object getDefaultPolicyAction(State state)
	//	{
	//		int numActions = stateActionVisits.get(state).keySet().size();
	//		Random rand = new Random();
	//		int choice = rand.nextInt(numActions);
	//		Object bestAction = (stateActionVisits.get(state).keySet().toArray())[choice];
	//		return bestAction;
	//	}

	//	public Object getBestAction(State state, boolean minimizeCost) throws PrismException
	//	{
	//		int numActions = 0;
	//		int numActionsZeroQ = 0;
	//		Object bestAction = null;
	//		boolean minCostInitialized = false;
	//		ArrayList<Object> numActionsZeroQInd = new ArrayList<Object>();
	//		double cost = 0;
	//		double minCost = Double.MAX_VALUE;
	//		if (!minimizeCost)
	//			minCost = Double.MIN_VALUE;
	//		double defaultCost = Double.MAX_VALUE;
	//		if (minimizeCost)
	//			defaultCost = Double.MIN_VALUE;
	//		boolean costCheck;
	//		double exploration_bias = explorationBias;
	//		if (stateActionVisits.containsKey(state)) {
	//			mainLog.println("Vists/Values " + state.toString());
	//			mainLog.println(stateActionVisits.get(state).toString());
	//			mainLog.println(stateActionQupper.get(state).toString());
	//			//let the minCost be the cost of the first action 
	//
	//			for (Object act : stateActionVisits.get(state).keySet()) {
	//				if (!minCostInitialized) {
	//					minCost = getQvalue(state, act, minimizeCost);
	//					bestAction = act;
	//					minCostInitialized = true;
	//				}
	//				numActions++;
	//				// adding 1 so that I dont get undefined
	//				double lnStateVisits = Math.log(getNumVisits(state));
	//				double stateActionVisits = Math.log(getNumVisits(state, act));
	//				double exploration_term = lnStateVisits / stateActionVisits;
	//				if (stateActionVisits == 0)
	//					exploration_term = defaultCost;
	//				if (Double.isNaN(exploration_term)) {
	//					exploration_term = defaultCost;
	//				}
	//				double sqrt = Math.sqrt(exploration_term);
	//				cost = getQvalue(state, act, minimizeCost) - exploration_bias * sqrt;
	//				if (cost == defaultCost) {
	//					numActionsZeroQ++;
	//					numActionsZeroQInd.add(act);
	//				}
	//				if (minimizeCost)
	//					costCheck = cost < minCost;
	//				else
	//					costCheck = cost > minCost;
	//
	//				if (costCheck) {
	//					minCost = cost;
	//					bestAction = act;
	//				}
	//			}
	//			if (numActionsZeroQ == numActions) {
	//				// then pick something at random
	//				Random rand = new Random();
	//				int choice = rand.nextInt(numActions);
	//				bestAction = (stateActionVisits.get(state).keySet().toArray())[choice];
	//			} else if (numActionsZeroQ > 0 && minCost == defaultCost) {
	//				//choose for the random ones 
	//				Random rand = new Random();
	//				int choice = rand.nextInt(numActionsZeroQInd.size());
	//				bestAction = numActionsZeroQInd.get(choice);
	//			}
	//		}
	//		//			else
	//		//			throw new PrismException("state not found");
	//
	//		if (bestAction != null)
	//			mainLog.println(state.toString() + " **" + bestAction.toString());
	//		return bestAction;
	//
	//	}

	public MDPSimple extractPolicyTreeAsDotFile(MDPSimple mdp, int initialState, boolean minimiseCosts)
	{

		MDPSimple policyTree = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		int[] stateLabels = new int[mdp.getNumStates()];
		Arrays.fill(stateLabels, -1);
		Queue<Integer> stateQ = new LinkedList<Integer>();
		stateQ.add(initialState);
		int state, ps, choice;
		Object action = null;
		BitSet visited = new BitSet();

		while (!stateQ.isEmpty()) {
			state = stateQ.remove();
			if (!visited.get(state)) {
				visited.set(state);
				if (stateLabels[state] == -1) {
					stateLabels[state] = policyTree.addState();
					statesList.add(mdp.getStatesList().get(state));
				}
				ps = stateLabels[state];
				State statestate = getStateFromIndex(state);
				try {
					action = this.getMinMaxAction(statestate, 0.0, false, true, false);//getBestAction(statestate, minimiseCosts);
				} catch (PrismException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				if (action != null) {
					if (stateActionIndices.containsKey(statestate)) {
						if (stateActionIndices.get(statestate).containsKey(action)) {
							choice = stateActionIndices.get(statestate).get(action);
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
									stateQ.add(childstate);
								}
								policyTree.addActionLabelledChoice(ps, distr, action.toString() + getStateValues(statesList.get(ps)));
							}

						}
					}
				}

			}
		}
		policyTree.setStatesList(statesList);

		return policyTree;
	}

	//	HashMap<State, Object> getBestPolicySoFar(State initialState, boolean minimiseCosts)
	//	{
	//
	//		HashMap<State, Object> bestPolicySoFar = new HashMap<State, Object>();
	//		for (State s : stateActionQupper.keySet()) {
	//			Object action = null;
	//			try {
	//				action = getBestAction(s, minimiseCosts);
	//			} catch (PrismException e) {
	//				// TODO Auto-generated catch block
	//				e.printStackTrace();
	//			}
	//			bestPolicySoFar.put(s, action);
	//		}
	//
	//		return bestPolicySoFar;
	//	}
	//
	//	public void printStateDetails(State state, boolean minimiseCost)
	//	{
	//		// print everything about this state
	//		String visits = "v:" + getNumVisits(state);
	//		String actionvisits = "";
	//		String qvalues = "";
	//		for (Object a : stateActionVisits.get(state).keySet()) {
	//			actionvisits += " ";
	//			actionvisits += a.toString() + ":" + getNumVisits(state, a);
	//			qvalues += a.toString() + ":" + getQvalue(state, a, minimiseCost);
	//
	//		}
	//		mainLog.println("State Info\n" + state);
	//		mainLog.println("Visits - " + visits + actionvisits);
	//		mainLog.println("Q Values - " + qvalues);
	//
	//	}
}
