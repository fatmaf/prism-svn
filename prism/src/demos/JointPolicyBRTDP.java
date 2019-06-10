package demos;

import java.io.FileWriter;
import java.io.IOException;
import java.util.AbstractMap;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;
import explicit.Distribution;
import explicit.MDP;
import explicit.MDPSimple;
import explicit.rewards.MDPRewardsSimple;
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

	protected HashMap<State, Double> stateValuesProgUpperBound;
	protected HashMap<State, Double> stateValuesProgLowerBound;

	protected HashMap<State, Double> stateValuesCostUpperBound;
	protected HashMap<State, Double> stateValuesCostLowerBound;

	protected HashMap<State, Double> oldStateValuesProbUpperBound;
	protected HashMap<State, Double> oldStateValuesProbLowerBound;

	protected HashMap<State, Double> oldStateValuesCostUpperBound;
	protected HashMap<State, Double> oldStateValuesCostLowerBound;

	protected HashMap<State, Integer> stateIndices;
	protected HashMap<State, HashMap<Object, Integer>> stateActionIndices;
	protected BitSet visited;
	protected HashMap<State, HashMap<Object, ArrayList<Integer>>> robotStateActionIndices;
	protected HashMap<State, HashMap<Object, Double>> stateActionCosts;
	protected HashMap<State, Boolean> stateSolved;
	protected BitSet accStates;
	protected BitSet notAccStates;
	protected BitSet leafStates;
	double explorationBias = 10;
	private double defaultProbLowerQ = 0.0;
	private double defaultProbUpperQ = 1.0;
	private double defaultCostLowerQ = 0.0;
	List<Double> dadists;
	MDPRewardsSimple progRewards = null;
	MDPRewardsSimple costRewards = null;
	BitSet progStates = null;
	List<State> deadends = null;

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
		notAccStates = new BitSet();

		stateValuesProbUpperBound = new HashMap<State, Double>();
		stateValuesProbLowerBound = new HashMap<State, Double>();
		stateValuesProgUpperBound = new HashMap<State, Double>();
		stateValuesProgLowerBound = new HashMap<State, Double>();
		stateValuesCostUpperBound = new HashMap<State, Double>();
		stateValuesCostLowerBound = new HashMap<State, Double>();

		dadists = da_dists;
		stateActionCosts = new HashMap<State, HashMap<Object, Double>>();
		stateSolved = new HashMap<State, Boolean>();
	}

	public void resetStateSolved()
	{
		stateSolved = new HashMap<State, Boolean>();
	}

	void addToLeafStates(State s)
	{
		//get state index 
		int sIndex = getStateIndex(s);
		if (sIndex != -1) {
			if (!leafStates.get(sIndex))
				leafStates.set(sIndex);
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
				if (!hasSuccessors(succStates.get(i)))

					addToLeafStates(succStates.get(i));
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

	void addActionReward(State state, Object action, double reward) throws PrismException
	{

		if (stateActionCosts.containsKey(state)) {
			if (!stateActionCosts.get(state).containsKey(action)) {

				stateActionCosts.get(state).put(action, reward);
			} else
				throw new PrismException("Trying to add same state action for reward!!");

		} else {
			HashMap<Object, Double> actionReward = new HashMap<Object, Double>();
			actionReward.put(action, reward);
			stateActionCosts.put(state, actionReward);
		}
	}

	public void clearRewards()
	{
		this.costRewards = null;
		this.progRewards = null;
	}

	public void addCostReward(double reward, State state, Object act)
	{
		if (this.costRewards == null) {
			this.costRewards = new MDPRewardsSimple(jointMDP.getNumStates());

		}
		addToRewardStructure(costRewards, reward, state, act);
	}

	public boolean addRobotStateActionIndices(State jointstate, Object jointAction, ArrayList<Object> actionList,
			HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState, State tstate)
	{
		boolean added = false;
		if (!robotStateActionIndices.containsKey(jointstate)) {
			robotStateActionIndices.put(jointstate, new HashMap<Object, ArrayList<Integer>>());
		}
		if (!robotStateActionIndices.get(jointstate).containsKey(jointAction)) {
			// now we make the arraylist
			ArrayList<Integer> actionIndices = new ArrayList<Integer>();
			for (int i = 0; i < numRobots; i++) {
				double t = 0.0;
				if(tstate !=null)
				{
					t = (double)tstate.varValues[i];
				}
				
				int actionIndex = -1; 
				if(t== 0.0)
					actionIndex = allPossibleActionsForState.get(i).get(actionList.get(i));
				actionIndices.add(actionIndex);
			}
			robotStateActionIndices.get(jointstate).put(jointAction, actionIndices);
			added = true;

		}
		return added;

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

	public void addToRewardStructure(MDPRewardsSimple rewStruct, double reward, State state, Object action)
	{
		//get state index in our mdp 
		//get action index 
		int stateIndex = getStateIndex(state);
		int actionIndex = getActionIndex(state, action);
		if (stateIndex != -1) {
			if (actionIndex != -1) {
				//				mainLog.println("reward for "+state.toString()+"-"+stateIndex+","+action.toString()+"-"+actionIndex+",r:"+reward);
				rewStruct.setTransitionReward(stateIndex, actionIndex, reward);
			}
		}
	}

	public void createCostRewardStructure()
	{
		double rew;
		for (State s : stateActionCosts.keySet()) {
			for (Object a : stateActionCosts.get(s).keySet()) {
				rew = stateActionCosts.get(s).get(a);
				addCostReward(rew, s, a);
			}
		}
	}

	//	
	//	/**
	//	 * Calculate the progression reward from the automaton distance metric
	//	 * @param distsToAcc List of automaton distances to goal
	//	 * @return the product mdp progression reward
	//	 */	
	//	public MDPRewards liftProgressionFromAutomaton(List<Double> distsToAcc)
	//	{
	//		MDP productMDP = (MDP)productModel;
	//		int numStates = productMDP.getNumStates();
	//		MDPRewardsSimple rewSimple = new MDPRewardsSimple(numStates);
	//		double currentStateDistance;//		Iterator<Entry<Integer, Double>> transitions;
	//		Entry<Integer, Double> transition;//		double rewardValue;//		int nextState;
	//		
	//		for (int productState = 0; productState < numStates; productState++) {
	//			currentStateDistance = distsToAcc.get(getAutomatonState(productState));
	//			int numChoices = productMDP.getNumChoices(productState);
	//			for (int i = 0; i < numChoices; i++) {
	//				transitions = productMDP.getTransitionsIterator(productState, i); //				rewardValue=0.0;
	//				while(transitions.hasNext()) {
	//					transition = transitions.next();
	//					nextState = transition.getKey();
	//					rewardValue = rewardValue + transition.getValue()*
	//	Math.max(currentStateDistance - distsToAcc.get(getAutomatonState(nextState)), 0.0);
	//				}
	//				rewSimple.setTransitionReward(productState, i, rewardValue);
	//			}
	//		}				
	//		return rewSimple;
	//	}
	/**
	 * this is literally a copy of Bruno's liftProgressionFromAutomaton in MDPModelChecker 
	 */
	public void createProgressionRewards()
	{
		int numStates = jointMDP.getNumStates();
		progRewards = new MDPRewardsSimple(numStates);
		double currentStateDistance;
		Iterator<Entry<Integer, Double>> trans;
		Entry<Integer, Double> tran;
		double rew;
		int ns;
		for (int s = 0; s < numStates; s++) {
			int currentDAState = this.getDAState(jointMDP.getStatesList().get(s));
			currentStateDistance = this.dadists.get(currentDAState);
			int numChoices = jointMDP.getNumChoices(s);

			for (int i = 0; i < numChoices; i++) {
				trans = jointMDP.getTransitionsIterator(s, i);
				rew = 0;
				while (trans.hasNext()) {
					tran = trans.next();
					ns = tran.getKey();
					int nsda = this.getDAState(jointMDP.getStatesList().get(ns));
					rew += tran.getValue() * (Math.max(currentStateDistance - dadists.get(nsda), 0.0));
				}
				progRewards.setTransitionReward(s, i, rew);

			}

		}
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

	//	public boolean addRobotStateActionIndices(State jointstate, Object jointAction, ArrayList<Object> actionList, int[] actionsIndices)
	//	{
	//		boolean added = false;
	//		if (!robotStateActionIndices.containsKey(jointstate)) {
	//			robotStateActionIndices.put(jointstate, new HashMap<Object, ArrayList<Integer>>());
	//		}
	//		if (!robotStateActionIndices.get(jointstate).containsKey(jointAction)) {
	//			// now we make the arraylist
	//			ArrayList<Integer> actionIndices = new ArrayList<Integer>();
	//			for (int i = 0; i < numRobots; i++) {
	//				int actionIndex = (actionsIndices[i]);
	//				actionIndices.add(actionIndex);
	//			}
	//			robotStateActionIndices.get(jointstate).put(jointAction, actionIndices);
	//			added = true;
	//
	//		}
	//		return added;
	//
	//	}

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

	//	int addAction(State state, Object action, State succState, double prob)
	//	{
	//		int actionInd = getActionIndex(state, action);
	//		int stateInd = -1;
	//		Distribution distr;
	//		if (actionInd == -1) {
	//			stateInd = addState(state);
	//			if (stateInd != -1) {
	//				int numActions = this.jointMDP.getNumChoices(stateInd);
	//				if (stateActionIndices.containsKey(state)) {
	//
	//					stateActionIndices.get(state).put(action, numActions);
	//				} else {
	//					stateActionIndices.put(state, new HashMap<Object, Integer>());
	//					stateActionIndices.get(state).put(action, numActions);
	//				}
	//
	//			}
	//			distr = new Distribution();
	//			int succStateIndex = addState(succState);
	//			distr.add(succStateIndex, prob);
	//			jointMDP.addActionLabelledChoice(stateInd, distr, action);
	//
	//		} else {
	//			// TODO check if this works
	//			// because it might not
	//
	//			stateInd = getStateIndex(state);
	//			int succStateIndex = addState(succState);
	//			// only add this new state if it is not there already
	//			distr = jointMDP.getChoice(stateInd, actionInd);
	//			if (!distr.contains(succStateIndex))
	//				jointMDP.getChoice(stateInd, actionInd).add(succStateIndex, prob);
	//		}
	//		return actionInd;
	//
	//	}

	public double getCostQValue(State state, Object action, boolean upperbound, boolean printVal) throws PrismException
	{
		if (isGoal(state))
			return 0.0;
		if (MRuctPaper.isDeadend(state))
			return 0.0;
		double cost = getStateActionCost(state, action);
		HashMap<State, Double> succStates = this.getSuccessors(state, action);
		//		boolean self_loop = false;
		double sum = 0;
		if (succStates != null) {
			for (State s : succStates.keySet()) {
				double prob = succStates.get(s);
				//				if (prob == 1.0) {
				//					if (s.equals(state))
				//						self_loop = true;
				//				}
				sum += prob * getCostValue(s, upperbound, printVal);

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
		//		if (self_loop)
		//			cost = 0.0;
		return cost + sum;

	}

	public double getCostValue(State state, boolean upperbound, boolean printVal)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesCostUpperBound;
		else
			stateV = this.stateValuesCostLowerBound;
		double valToRet = 0;
		//costs are an upper bound 
		if (!stateV.containsKey(state)) {
			boolean deadend = MRuctPaper.isDeadend(state);
			if (deadend) {
				valToRet = this.defaultCostLowerQ;
			} else {
				boolean isgoal = isGoal(state);
				if (isgoal)
					valToRet = 0;
				else {
					if (upperbound) {
						valToRet = MRuctPaper.getInitCostBound(state);
						//because we're doing maxprob mincost 
						//I dont have to initialise anything it can just be 0
						//this is a bit of a dilemma - how do we do this cost stuff 

					} else {
						valToRet = this.defaultCostLowerQ;
					}
				}
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

	public double getProgValue(State state, boolean upperbound, boolean printVal)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesProgUpperBound;
		else
			stateV = this.stateValuesProgLowerBound;
		double valToRet = 0;
		//costs are an upper bound 
		if (!stateV.containsKey(state)) {

			boolean isgoal = isGoal(state);
			if (isgoal) {
				valToRet = 0;//this.defaultCostLowerQ;
			} else {
				boolean deadend = MRuctPaper.isDeadend(state);
				if (deadend)
					valToRet = 0;
				else {
					if (upperbound) {

						valToRet = 1;//MRuctPaper.getInitProgBound(state,!upperbound);

					} else {
						valToRet = MRuctPaper.getInitProgBound(state,!upperbound);//assuming everyone can get to the goal //this.dadists.get(getDAState(state));
					}
				}
			}
			stateV.put(state, valToRet);
		}
		valToRet = stateV.get(state);
		if (printVal) {
			String printText = state.toString() + "prog:";
			if (upperbound)
				printText += "u";
			else
				printText += "l";

			printText += valToRet;
			mainLog.println(printText);
		}
		return valToRet;
	}

	public double getProgQValue(State state, Object action, boolean upperbound, boolean printVal) throws PrismException
	{
		if (isGoal(state))
			return 0.0;

		if (MRuctPaper.isDeadend(state))
			return 0.0;

		double currentStateDistance = dadists.get(getDAState(state));
		double stateActionReward = 0;
		//		tran.getValue() * (Math.max(currentStateDistance - dadists.get(nsda), 0.0));
		HashMap<State, Double> succStates = this.getSuccessors(state, action);
		double sum = 0;
		if (succStates != null) {
			for (State s : succStates.keySet()) {
				double prob = succStates.get(s);
				sum += prob * getProgValue(s, upperbound, printVal);
				//(Math.max(currentStateDistance - getProgValue(s, upperbound, printVal), 0.0));
				double succStateDist = dadists.get(getDAState(s));
				stateActionReward += prob * (Math.max(currentStateDistance - succStateDist, 0.0));
			}
		} else {
			//hmmm we have a problem here maybe 
			throw new PrismException("State: " + state.toString() + " action: " + action.toString() + " has no successors!!");
		}

		return stateActionReward + sum;

	}

	//	protected State simulateAction(State state, Object action)
	//	{
	//		int state_index = -1;
	//		if (stateActionIndices.containsKey(state))
	//			state_index = stateIndices.get(state);
	//		if (state_index != -1) {
	//			if (stateActionIndices.get(state).containsKey(action)) {
	//				int action_choice = stateActionIndices.get(state).get(action);
	//				int succ_state_index = -1;
	//				double prob = (new Random()).nextDouble();
	//				double cum_prob = 0;
	//				Iterator<Entry<Integer, Double>> iter = jointMDP.getTransitionsIterator(state_index, action_choice);
	//				while (iter.hasNext()) {
	//					Entry<Integer, Double> entry = iter.next();
	//					cum_prob += entry.getValue();
	//					if (cum_prob >= prob) {
	//						succ_state_index = entry.getKey();
	//						break;
	//					}
	//
	//				}
	//
	//				if (succ_state_index != -1) {
	//					return jointMDP.getStatesList().get(succ_state_index);
	//				}
	//			} else
	//				return null;
	//		}
	//
	//		return null;
	//
	//	}

	int getDAState(State state)
	{
		// get the da state
		State dastate;
		if (this.sharedStatesNamesList == null)
			dastate = state.substate(this.numRobots, this.numRobots + 1);
		else
			dastate = state.substate(this.numRobots + 1, this.numRobots + 2);
		dastate = (State) dastate.varValues[0];
		int dastateint = (int) dastate.varValues[0];
		return dastateint;
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

	//	public Object getMinMaxAction(State state, ArrayList<Object> actions, double probcost, boolean minCost, boolean upperBound, boolean printVal)
	//			throws PrismException
	//	{
	//
	//		double probVal = 0;
	//		double costVal = 0;
	//		Object actionRet = null;
	//		ArrayList<Object> duplicateRetVals = new ArrayList<Object>();
	//		ArrayList<Double> duplicateRetValsCost = new ArrayList<Double>();
	//
	//		for (int i = 0; i < actions.size(); i++) {
	//			Object act = actions.get(i);
	//
	//			//			double cost = costs.get(i);
	//			double cost = getStateActionCost(state, act);
	//			double probqval = getProbQValue(state, act, probcost, minCost, upperBound, printVal);
	//			double costqval = getCostQValue(state, act, cost, upperBound, printVal);
	//			if (printVal)
	//				mainLog.println(actions.get(i).toString() + ":Q[" + probqval + "," + costqval + "]");
	//			if (actionRet == null) {
	//				probVal = probqval;
	//				actionRet = act;
	//				costVal = costqval;
	//				duplicateRetVals.add(act);
	//				duplicateRetValsCost.add(costqval);
	//
	//			} else {
	//				boolean check;
	//				if (probVal == probqval) {
	//
	//					//just minimise this cost 
	//					if (costVal > costqval) {
	//						probVal = probqval;
	//						actionRet = act;
	//						costVal = costqval;
	//					} else if (costVal == costqval) {
	//						duplicateRetVals.add(act);
	//						duplicateRetValsCost.add(costqval);
	//					}
	//				} else {
	//					if (minCost)
	//						check = probVal > probqval;
	//					else
	//						check = probVal < probqval;
	//					if (check) {
	//						probVal = probqval;
	//						costVal = costqval;
	//						actionRet = act;
	//					}
	//				}
	//
	//			}
	//		}
	//		//		if (duplicateRetVals.size() > 1) {
	//		//			//randomly choose an action here 
	//		//			int actionChoice = new Random().nextInt(duplicateRetVals.size());
	//		//			actionRet = duplicateRetVals.get(actionChoice);
	//		//		}
	//		return actionRet;
	//
	//	}
	//	//
	//	public Object getMinMaxAction(State state, ArrayList<Object> actions, double probcost, double cost, boolean minCost, boolean upperBound, boolean printVal)
	//			throws PrismException
	//	{
	//
	//		double probVal = 0;
	//		double costVal = 0;
	//		Object actionRet = null;
	//		ArrayList<Object> duplicateRetVals = new ArrayList<Object>();
	//		ArrayList<Double> duplicateRetValsCost = new ArrayList<Double>();
	//
	//		for (int i = 0; i < actions.size(); i++) {
	//			Object act = actions.get(i);
	//			//			double cost = costs.get(i);
	//			double probqval = getProbQValue(state, act, probcost, minCost, upperBound, printVal);
	//			double costqval = getCostQValue(state, act, cost, upperBound, printVal);
	//			if (actionRet == null) {
	//				probVal = probqval;
	//				actionRet = act;
	//				costVal = costqval;
	//				duplicateRetVals.add(act);
	//				duplicateRetValsCost.add(costqval);
	//
	//			} else {
	//				boolean check;
	//				if (probVal == probqval) {
	//
	//					//just minimise this cost 
	//					if (costVal > costqval) {
	//						probVal = probqval;
	//						actionRet = act;
	//						costVal = costqval;
	//					} else if (costVal == costqval) {
	//						duplicateRetVals.add(act);
	//						duplicateRetValsCost.add(costqval);
	//					}
	//				} else {
	//					if (minCost)
	//						check = probVal > probqval;
	//					else
	//						check = probVal < probqval;
	//					if (check) {
	//						probVal = probqval;
	//						costVal = costqval;
	//						actionRet = act;
	//					}
	//				}
	//
	//			}
	//		}
	//		//		if (duplicateRetVals.size() > 1) {
	//		//			//randomly choose an action here 
	//		//			int actionChoice = new Random().nextInt(duplicateRetVals.size());
	//		//			actionRet = duplicateRetVals.get(actionChoice);
	//		//		}
	//		return actionRet;
	//
	//	}

	//	public Object getMinMaxAction(State state, double probcost, boolean minCost, boolean upperBound, boolean printVal) throws PrismException
	//
	//	{
	//		//so basically we can get all the actions here 
	//		if (this.robotStateActionIndices.containsKey(state)) {
	//			ArrayList<Object> actions = new ArrayList<Object>();
	//			actions.addAll(robotStateActionIndices.get(state).keySet());
	//			return getMinMaxAction(state, actions, probcost, minCost, upperBound, printVal);
	//		} else {
	//			throw new PrismException("state not in list!!" + state.toString());
	//		}
	//	}

	public Entry<Object, ArrayList<Double>> getMinMaxValue(State state, ArrayList<Object> actions, boolean minCost, boolean upperBound, boolean isgoal,
			boolean printVal) throws PrismException
	{
		boolean aha = false;
		if (state.toString().contains("(-1),(-1)"))
			aha = true;
		Entry<Object, ArrayList<Double>> toret;
		if (isGoal(state)) {
			ArrayList<Double> temp = new ArrayList<Double>();
			temp.add(1.0);
			temp.add(0.0);
			temp.add(0.0);
			toret = new AbstractMap.SimpleEntry<Object, ArrayList<Double>>("*", temp);
		} else {
			if (MRuctPaper.isDeadend(state)) {
				ArrayList<Double> temp = new ArrayList<Double>();
				temp.add(0.0);
				temp.add(0.0);
				temp.add(0.0);
				toret = new AbstractMap.SimpleEntry<Object, ArrayList<Double>>("*", temp);
			} else {
				boolean setFirstCost = true;
				double bestProbYet = getProbValue(state, upperBound, printVal);
				double bestCostYet = getCostValue(state, upperBound, printVal);
				double bestProgYet = getProgValue(state, upperBound, printVal);
				Object bestActYet = null;

				for (int i = 0; i < actions.size(); i++) {

					Object act = actions.get(i);
					//			double cost = costs.get(i);
					//				double cost = getStateActionCost(state, act);
					double probqval = getProbQValue(state, act, upperBound, printVal);
					double costqval = getCostQValue(state, act, upperBound, printVal);
					double progqval = getProgQValue(state, act, upperBound, printVal);

					if (printVal) {
						mainLog.println(actions.get(i).toString() + ":Q[" + probqval + "," + progqval + "," + costqval + "]");

					}
					if (setFirstCost) {
						bestProbYet = probqval;
						bestCostYet = costqval;
						bestProgYet = progqval;
						setFirstCost = false;
						bestActYet = act;
					} else {
						boolean check;
						if (minCost)
							check = bestProbYet > probqval;
						else
							check = bestProbYet < probqval;
						if (check) {
							bestProbYet = probqval;
							bestCostYet = costqval;
							bestProgYet = progqval;
							bestActYet = act;
						} else {
							//this if is redundant but hey
							if (bestProbYet == probqval) {
								if (bestProgYet < progqval) {
									bestProgYet = progqval;
									bestCostYet = costqval;
									bestActYet = act;
								} else {
									if (bestProgYet == progqval) {
										if (bestCostYet > costqval) {
											bestCostYet = costqval;
											bestActYet = act;
										}
									}
								}
								//							if (bestCostYet > costqval) //choose the lowest cost here ???? //hmmm 
								//							{
								//								bestCostYet = costqval;
								//								bestProgYet = progqval;
								//								bestActYet = act;
								//							}
							}
						}

					}
				}
				ArrayList<Double> valuesToRet = new ArrayList<Double>();
				valuesToRet.add(bestProbYet);
				valuesToRet.add(bestProgYet);
				valuesToRet.add(bestCostYet);
				toret = new AbstractMap.SimpleEntry<Object, ArrayList<Double>>(bestActYet, valuesToRet);
			}
		}
		return toret;

	}

	//	public Entry<Double, Double> getMinMaxValue(State state, ArrayList<Object> actions, 
	//			double probcost, double cost, boolean minCost, boolean upperBound,
	//			boolean isgoal, boolean printVal) throws PrismException
	//	{
	//
	//		boolean setcost = true;
	//		double probVal = 0;
	//		double costVal = 0;
	//		//duplication doesnt matter here cuz its just a value 
	//
	//		for (int i = 0; i < actions.size(); i++) {
	//			Object act = actions.get(i);
	//			//			double cost = costs.get(i);
	//			double probqval = getProbQValue(state, act, probcost, upperBound, isgoal, printVal);
	//			double costqval = getCostQValue(state, act, cost, upperBound, printVal);
	//			if (printVal)
	//				mainLog.println(actions.get(i).toString() + ":Q[" + probqval + "," + costqval + "]");
	//			if (setcost) {
	//				probVal = probqval;
	//				costVal = costqval;
	//				setcost = false;
	//			} else {
	//				boolean check;
	//				if (minCost)
	//					check = probVal > probqval;
	//				else
	//					check = probVal < probqval;
	//				if (check) {
	//					probVal = probqval;
	//					costVal = costqval;
	//				} else {
	//					if (probVal == probqval) {
	//						if (costVal > costqval) {
	//							costVal = costqval;
	//						}
	//					}
	//				}
	//
	//			}
	//		}
	//		return new AbstractMap.SimpleEntry<Double, Double>(probVal, costVal);
	//
	//	}
	//
	public Entry<Object, ArrayList<Double>> getMinMaxValue(State state, boolean minCost, boolean upperBound, boolean printVal) throws PrismException
	{
		//so basically we can get all the actions here 
		if (this.robotStateActionIndices.containsKey(state)) {
			ArrayList<Object> actions = new ArrayList<Object>();
			actions.addAll(robotStateActionIndices.get(state).keySet());

			return getMinMaxValue(state, actions, minCost, upperBound, isGoal(state), printVal);
		} else {

			//			throw new PrismException("state not in list!!" + state.toString());
			mainLog.println("State has no associated actions " + state.toString());
			return null;
		}
	}

	//	public Entry<Double, Double> getMinMaxValue(State state, double probCost, double cost, boolean minCost, boolean upperBound, boolean isgoal,
	//			boolean printVal) throws PrismException
	//	{
	//		//so basically we can get all the actions here 
	//		if (this.robotStateActionIndices.containsKey(state)) {
	//			ArrayList<Object> actions = new ArrayList<Object>();
	//			actions.addAll(robotStateActionIndices.get(state).keySet());
	//			return getMinMaxValue(state, actions, probCost, cost, minCost, upperBound, isgoal, printVal);
	//		} else {
	//			throw new PrismException("state not in list!!" + state.toString());
	//		}
	//	}

	public double getProbQValue(State state, Object action, boolean upperbound, boolean printVal) throws PrismException
	{
		HashMap<State, Double> succStates = this.getSuccessors(state, action);
		//		boolean self_loop = false;
		double sum = 0;
		if (succStates != null) {
			for (State s : succStates.keySet()) {
				double prob = succStates.get(s);
				//				if (prob == 1.0) {
				//					if (s.equals(state))
				//						self_loop = true;
				//				}
				sum += prob * getProbValue(s, upperbound, /* isGoal(s),  self_loop,*/ printVal);

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

	public double getProbValue(State state, boolean upperbound, /* boolean selfLoop, */boolean printVal)
	{
		boolean alert = false;
		String stateString = "(1),(1),(1)";
		if (state.toString().contains(stateString))
			alert = true;
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesProbUpperBound;
		else
			stateV = this.stateValuesProbLowerBound;
		double valToRet = -1;
		//		if (selfLoop) {
		//			valToRet = this.defaultProbLowerQ;
		//			if (!stateV.containsKey(state))
		//				stateV.put(state, valToRet);
		//			else {
		//				if (stateV.get(state) != valToRet)
		//					stateV.put(state, valToRet);
		//			}
		//		} else {
		if (stateV.containsKey(state)) {
			valToRet = stateV.get(state);
		}

		else {
			boolean isgoal = isGoal(state);
			if (isgoal) {
				valToRet = this.defaultProbUpperQ;

			} else {
				boolean deadend = MRuctPaper.isDeadend(state);
				if (deadend) {
					valToRet = this.defaultProbLowerQ;
				} else {

					if (upperbound) {

						valToRet = this.defaultProbUpperQ;
					} else {

						valToRet = MRuctPaper.getInitProbBound(state);
						//one robot performs worse than 2 so lower bound//this.defaultProbLowerQ;
					}

				}

			}
			stateV.put(state, valToRet);
		}
		//		}

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

	public double getProbValueDiff(State state, boolean isgoal, boolean printVal) throws PrismException //get the difference between the upper and lower bounds 
	{
		//boolean selfloop = false because we dont care
		double diff = getProbValue(state, true, printVal) - getProbValue(state, false, printVal);
		if (diff < 0)
			throw new PrismException("State diff negative!!! " + state.toString());
		return diff;
	}

	//	double getProbValueIgnoreSelfLoop(State state, boolean upperbound, boolean isgoal, boolean printVal)
	//	{
	//		//setting the selfloop variable to false because we dont care 
	//		return getProbValue(state, upperbound, isgoal, false, printVal);
	//	}

	double getStateActionCost(State state, Object act) throws PrismException
	{
		if (this.stateActionCosts.containsKey(state)) {
			if (this.stateActionCosts.get(state).containsKey(act)) {
				return this.stateActionCosts.get(state).get(act);
			}
		}
		throw new PrismException("No costs preset for " + state.toString() + " " + act.toString() + " Error!!");
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

	int getStateIndex(State state)
	{
		int stateInd = -1;
		if (stateIndices.containsKey(state)) {
			stateInd = stateIndices.get(state);
		}
		return stateInd;
	}

	public String getStateValues(State state)
	{
		boolean isgoal = isGoal(state);

		String toprint = state.toString() + "P:" + getProbValue(state, true, false) + "," + getProbValue(state, false, false) + "|";
		toprint += "R:" + getProgValue(state, true, false) + "," + getProgValue(state, false, false) + "|";
		toprint += "C:" + getCostValue(state, true, false) + "," + getCostValue(state, false, false);
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

	protected boolean hasSuccessors(State state)
	{
		boolean hasSuccs = false;
		int state_index = -1;

		if (stateActionIndices.containsKey(state))

			state_index = stateIndices.get(state);

		if (state_index != -1) {
			if (stateActionIndices.get(state).keySet().size() > 0) {
				hasSuccs = true;
			}
		}
		return hasSuccs;
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

	boolean isGoal(State state)
	{
		int stateIndex = this.getStateIndex(state);
		if (stateIndex != -1) {
			if (accStates.get(stateIndex)) {
				return true;
			} else {
				if (notAccStates.get(stateIndex)) {
					return false;
				} else {
					int dastateint = getDAState(state);
					double dist = this.dadists.get(dastateint);
					if (dist == 0) {

						accStates.set(stateIndex);
						return true;
					} else {
						notAccStates.set(stateIndex);
						return false;
					}
				}
			}
		} else {
			int dastateint = getDAState(state);
			double dist = this.dadists.get(dastateint);
			if (dist == 0) {

				//				accStates.set(stateIndex);
				return true;
			} else {
				//				notAccStates.set(stateIndex);
				return false;
			}
		}
	}

	boolean isSolved(State state)
	{
		boolean solved = false;
		if (this.stateSolved.containsKey(state))
			solved = stateSolved.get(state);
		return solved;
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

	/**
	 * again a literal copy of Bruno's function 
	 * @param product
	 * @param progRewards
	 * @param prodCosts
	 * @return
	 */
	public void progressionTrim()
	{
		MDP productModel = jointMDP;
		int numStates = productModel.getNumStates();
		List<HashSet<Integer>> predList = new ArrayList<HashSet<Integer>>(numStates);
		Deque<Integer> queue = new ArrayDeque<Integer>();
		progStates = new BitSet(numStates);
		long time;

		time = System.currentTimeMillis();

		//init predList and queue
		for (int i = 0; i < numStates; i++) {
			predList.add(new HashSet<Integer>());
			for (int j = 0; j < productModel.getNumChoices(i); j++) {
				if (progRewards.getTransitionReward(i, j) > 0.0) {
					queue.add(i);
					progStates.set(i);
					break;
				}
			}
		}

		Iterator<Integer> successorsIt;
		HashSet<Integer> statePreds;
		//set predList
		for (int i = 0; i < numStates; i++) {
			successorsIt = productModel.getSuccessorsIterator(i);
			while (successorsIt.hasNext()) {
				statePreds = predList.get(successorsIt.next());
				statePreds.add(i);
			}
		}

		//set 
		int currentState;
		while (!queue.isEmpty()) {
			currentState = queue.poll();
			for (int pred : predList.get(currentState)) {
				if (!progStates.get(pred)) {
					queue.add(pred);
					progStates.set(pred);
				}
			}
		}

		time = System.currentTimeMillis() - time;
		mainLog.println("Time for cost trimming: " + time / 1000.0 + " seconds.");

	}

	public void setCostValue(State state, boolean upperbound, double cost)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesCostUpperBound;
		else
			stateV = this.stateValuesCostLowerBound;

		//just going to check 
		if (stateV.containsKey(state)) {
			double oldVal = stateV.get(state);

			if (upperbound) {
				if (oldVal < cost)
					mainLog.println("Error!!! Updating State Cost" + state.toString() + "- upper bound: " + oldVal + " to " + cost + "!!");
			} else {
				if (oldVal > cost)
					mainLog.println("Error!!! Updating State Cost" + state.toString() + "- lower bound: " + oldVal + " to " + cost + "!!");
			}
		}
		stateV.put(state, cost);

	}

	public void setProgValue(State state, boolean upperbound, double cost)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesProgUpperBound;
		else
			stateV = this.stateValuesProgLowerBound;
		//just going to check 
		if (stateV.containsKey(state)) {
			double oldVal = stateV.get(state);

			if (upperbound) {
				if (oldVal < cost)
					mainLog.println("Error!!! Updating State Prog" + state.toString() + "- upper bound: " + oldVal + " to " + cost + "!!");
			} else {
				if (oldVal > cost)
					mainLog.println("Error!!! Updating State Prog" + state.toString() + "- lower bound: " + oldVal + " to " + cost + "!!");
			}
		}
		stateV.put(state, cost);

	}

	public void setProbRewValues(State state, double probvalue, double progvalue, double costvalue, boolean upperbound)
	{
		setProbValue(state, probvalue, upperbound);
		setProgValue(state, upperbound, progvalue);
		setCostValue(state, upperbound, costvalue);
	}

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

	public void setProbValue(State state, double value, boolean upperbound)
	{
		HashMap<State, Double> stateV = null;
		if (upperbound)
			stateV = this.stateValuesProbUpperBound;
		else
			stateV = this.stateValuesProbLowerBound;

		//just going to check 
		if (stateV.containsKey(state)) {
			double oldVal = stateV.get(state);

			if (upperbound) {
				if (oldVal < value)
					mainLog.println("Error!!! Updating State Prog" + state.toString() + "- upper bound: " + oldVal + " to " + value + "!!");
			} else {
				if (oldVal > value)
					mainLog.println("Error!!! Updating State Prob" + state.toString() + "- lower bound: " + oldVal + " to " + value + "!!");
			}
		}

		stateV.put(state, value);

	}

	public void updateProbCostValues(HashMap<State, HashMap<String, Double>> resStuff)
	{
		String[] stringPrefixes = { "l", "u" };

		boolean[] bounds = { false, true };
		for (State s : stateValuesProbUpperBound.keySet()) {
			HashMap<String, Double> valueUpdates = resStuff.get(s);

			for (int i = 0; i < stringPrefixes.length; i++) {
				String stringPrefix = stringPrefixes[i];

				String pString = stringPrefix + "p";
				String progString = stringPrefix + "prog";
				String costString = stringPrefix + "c";

				double probValue = valueUpdates.get(pString);
				double progValue = valueUpdates.get(progString);
				double costValue = valueUpdates.get(costString);
				setProbRewValues(s, probValue, progValue, costValue, bounds[i]);
			}
		}

	}

	public void saveOldStateValues()
	{
		oldStateValuesProbUpperBound = new HashMap<State, Double>();
		oldStateValuesProbLowerBound = new HashMap<State, Double>();

		oldStateValuesCostUpperBound = new HashMap<State, Double>();
		oldStateValuesCostLowerBound = new HashMap<State, Double>();

		oldStateValuesProbUpperBound.putAll(stateValuesProbUpperBound);
		oldStateValuesProbLowerBound.putAll(stateValuesProbLowerBound);
		oldStateValuesCostUpperBound.putAll(stateValuesCostUpperBound);
		oldStateValuesCostLowerBound.putAll(stateValuesCostLowerBound);

	}

	public boolean doStateValuesDiff(double epsilon)
	{
		if (oldStateValuesProbUpperBound != null) {
			boolean nodiff = true;
			HashMap<State, Double> oldOne = oldStateValuesProbUpperBound;
			HashMap<State, Double> currOne = stateValuesProbUpperBound;

			for (State s : stateValuesProbUpperBound.keySet()) {
				//				String toprint = state.toString() + "[P u:" + getProbValueIgnoreSelfLoop(state, true, isgoal, false) + ", l:"
				//						+ getProbValueIgnoreSelfLoop(state, false, isgoal, false) + "]";
				//				toprint += "[C u:" + getCostValue(state, true, isgoal, false) + ", l:" + getCostValue(state, false, isgoal, false) + "]";

				double res = 1.0; //let there be a difference if we haven't explored this state. 
				if (currOne.containsKey(s) && oldOne.containsKey(s))
					res = Math.abs(currOne.get(s) - oldOne.get(s));
				if (res > epsilon) {
					mainLog.println("Values not the same " + s.toString() + "- res");
					nodiff = false;
					break;
				}
			}
			return nodiff;
		}
		return false;
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
