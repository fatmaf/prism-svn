package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Stack;

import explicit.MDP;
import explicit.MDPSimple;
import explicit.MDPSparse;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import strat.MDStrategy;
import strat.Strategy;

public class PolicyCreator
{

	protected MDPCreator mdpCreator;

	public HashMap<Integer, Integer> policyStateToOriginalMDPMap = null;

	public PolicyCreator()
	{
		mdpCreator = new MDPCreator();
		policyStateToOriginalMDPMap = new HashMap<Integer, Integer>();
	}

	public MDPSimple createPolicy(MDP productMdp, Strategy strat) throws PrismException
	{
		int initialState = productMdp.getFirstInitialState();
		//		if(productMdp instanceof MDPSimple)
		return createPolicy(initialState, /*(MDPSimple)*/ productMdp, strat);
		//		if(productMdp instanceof MDPSparse)
		//			return createPolicy(initialState, (MDPSparse) productMdp, strat);
	}

	int findActionIndex(MDP mdp, int s, Object a)
	{
		int numChoices = mdp.getNumChoices(s);
		int actionIndex = -1;
		if(a == null)
			return actionIndex;
		for (int i = 0; i < numChoices; i++) {
			Object action = mdp.getAction(s, i);
			//			System.out.println(action.toString());
			
			if (action != null) {
				if (action.toString().contentEquals(a.toString())) {
					actionIndex = i;
					break;
				}
			}
		}
		return actionIndex;
	}

	public MDPSimple createPolicy(int initialState, MDP mdp, Strategy strat)
	{
		Stack<Integer> toVisit = new Stack<Integer>();
		BitSet visited = new BitSet();
		toVisit.add(initialState);
		int s;
		while (!toVisit.isEmpty()) {
			s = toVisit.pop();
			visited.set(s);
			State sState = mdp.getStatesList().get(s);

			strat.initialise(s);
			//			strat.initialise(s);
			Object action = strat.getChoiceAction();
			int actionIndex = findActionIndex(mdp, s, action);

			if (actionIndex > -1) {
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, actionIndex);
				ArrayList<Entry<State, Double>> successors = new ArrayList<Entry<State, Double>>();
				ArrayList<Integer> mdpSuccInts = new ArrayList<Integer>();
				while (tranIter.hasNext()) {
					Entry<Integer, Double> stateProbPair = tranIter.next();
					int succ = stateProbPair.getKey();
					mdpSuccInts.add(succ);
					State succState = mdp.getStatesList().get(stateProbPair.getKey());
					double prob = stateProbPair.getValue();
					successors.add(new AbstractMap.SimpleEntry<State, Double>(succState, prob));
					if (!toVisit.contains(succ) && !visited.get(succ)) {
						toVisit.add(succ);
					}
				}
				mdpCreator.addAction(sState, action, successors);
				addStateToMap(s, sState);

				addSuccessorStatesToMap(successors, mdpSuccInts);

			}
		}
		return mdpCreator.mdp;
	}

	public void addStateToMap(int s, State sState)
	{
		if (!policyStateToOriginalMDPMap.containsKey(s)) {
			int si = mdpCreator.getStateIndex(sState);
			policyStateToOriginalMDPMap.put(si, s);
		}
	}

	public void addSuccessorStatesToMap(ArrayList<Entry<State, Double>> successors, ArrayList<Integer> mdpsucints)
	{
		for (int i = 0; i < mdpsucints.size(); i++) {
			int s = mdpsucints.get(i);
			State sState = successors.get(i).getKey();
			addStateToMap(s, sState);
		}
	}

	public MDPSimple createPolicyWithRewardsStructures(int initialState, MDP mdp, Strategy strat, MDPRewardsSimple progRews, MDPRewardsSimple costs,
			BitSet accStates)
	{
		boolean hasRewards = (progRews != null) & (costs != null);
		Stack<Integer> toVisit = new Stack<Integer>();
		BitSet visited = new BitSet();
		toVisit.add(initialState);
		int s;
		while (!toVisit.isEmpty()) {
			s = toVisit.pop();

			visited.set(s);
			State sState = mdp.getStatesList().get(s);

			if (accStates != null) {
				if (accStates.get(s))
					this.mdpCreator.setAccState(sState);
			}
			strat.initialise(s);
			//			strat.initialise(s);
			Object action = strat.getChoiceAction();
			int actionIndex = findActionIndex(mdp, s, action);

			double progRew = 0;

			double cost = 0;
			if (actionIndex > -1) {
				if (hasRewards) {
					progRew = progRews.getTransitionReward(s, actionIndex);
					cost = costs.getTransitionReward(s, actionIndex);
				}

				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, actionIndex);
				ArrayList<Entry<State, Double>> successors = new ArrayList<Entry<State, Double>>();
				ArrayList<Integer> mdpSuccInts = new ArrayList<Integer>();
				while (tranIter.hasNext()) {
					Entry<Integer, Double> stateProbPair = tranIter.next();
					int succ = stateProbPair.getKey();
					mdpSuccInts.add(succ);
					State succState = mdp.getStatesList().get(stateProbPair.getKey());
					double prob = stateProbPair.getValue();
					successors.add(new AbstractMap.SimpleEntry<State, Double>(succState, prob));
					if (!toVisit.contains(succ) && !visited.get(succ)) {
						toVisit.add(succ);
					}
				}
				mdpCreator.addAction(sState, action, successors, progRew, cost);
				addStateToMap(s, sState);

				addSuccessorStatesToMap(successors, mdpSuccInts);
			}
		}
		return mdpCreator.mdp;
	}

	public MDPSimple createPolicyWithRewardsStructuresAsLabelsAllStates(MDP mdp, Strategy strat, MDPRewardsSimple progRews, MDPRewardsSimple costs,
			BitSet accStates)
	{
		MDPSimple toret = null;
		for (int i = 0; i < mdp.getNumStates(); i++) {
			toret = createPolicyWithRewardsStructuresAsLabels(i, mdp, strat, progRews, costs, accStates);
		}
		return toret;
	}

	public MDPSimple createPolicyWithRewardsStructuresAsLabels(int initialState, MDP mdp, Strategy strat, MDPRewardsSimple progRews, MDPRewardsSimple costs,
			BitSet accStates)
	{
		boolean hasRewards = (progRews != null) & (costs != null);
		Stack<Integer> toVisit = new Stack<Integer>();
		BitSet visited = new BitSet();
		toVisit.add(initialState);
		int s;
		while (!toVisit.isEmpty()) {
			s = toVisit.pop();

			visited.set(s);
			State sState = mdp.getStatesList().get(s);

			if (accStates != null) {
				if (accStates.get(s))
					this.mdpCreator.setAccState(sState);
			}
			strat.initialise(s);
			//			strat.initialise(s);
			Object action = strat.getChoiceAction();
			int actionIndex = findActionIndex(mdp, s, action);

			double progRew = 0;

			double cost = 0;
			if (actionIndex > -1) {
				if (hasRewards) {
					progRew = progRews.getTransitionReward(s, actionIndex);
					cost = costs.getTransitionReward(s, actionIndex);
				}
				ArrayList<Integer> mdpSuccInts = new ArrayList<Integer>();
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, actionIndex);
				ArrayList<Entry<State, Double>> successors = new ArrayList<Entry<State, Double>>();
				while (tranIter.hasNext()) {
					Entry<Integer, Double> stateProbPair = tranIter.next();
					int succ = stateProbPair.getKey();
					mdpSuccInts.add(succ);
					State succState = mdp.getStatesList().get(stateProbPair.getKey());
					double prob = stateProbPair.getValue();
					successors.add(new AbstractMap.SimpleEntry<State, Double>(succState, prob));
					if (!toVisit.contains(succ) && !visited.get(succ)) {
						toVisit.add(succ);
					}
				}
				mdpCreator.addActionAndLabelRews(sState, action, successors, progRew, cost);
				addStateToMap(s, sState);

				addSuccessorStatesToMap(successors, mdpSuccInts);
			}
		}
		return mdpCreator.mdp;
	}

	public MDPSimple recreateMDPWithRewardsStructuresAsLabels(int initialState, MDP mdp, MDPRewardsSimple progRews, MDPRewardsSimple costs, BitSet accStates)
	{
		boolean hasRewards = (progRews != null) & (costs != null);
		Stack<Integer> toVisit = new Stack<Integer>();
		BitSet visited = new BitSet();
		toVisit.add(initialState);
		int s;
		while (!toVisit.isEmpty()) {
			s = toVisit.pop();

			visited.set(s);
			State sState = mdp.getStatesList().get(s);

			if (accStates != null) {
				if (accStates.get(s))
					this.mdpCreator.setAccState(sState);
			}
			int numChoices = mdp.getNumChoices(s);
			for (int c = 0; c < numChoices; c++) {
				//			strat.initialise(s);
				Object action = mdp.getAction(s, c);
				int actionIndex = c;

				double progRew = 0;

				double cost = 0;
				if (actionIndex > -1) {
					if (hasRewards) {
						progRew = progRews.getTransitionReward(s, actionIndex);
						cost = costs.getTransitionReward(s, actionIndex);
					}
					ArrayList<Integer> mdpSuccInts = new ArrayList<Integer>();
					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, actionIndex);
					ArrayList<Entry<State, Double>> successors = new ArrayList<Entry<State, Double>>();
					while (tranIter.hasNext()) {
						Entry<Integer, Double> stateProbPair = tranIter.next();
						int succ = stateProbPair.getKey();
						mdpSuccInts.add(succ);
						State succState = mdp.getStatesList().get(stateProbPair.getKey());
						double prob = stateProbPair.getValue();
						successors.add(new AbstractMap.SimpleEntry<State, Double>(succState, prob));
						if (!toVisit.contains(succ) && !visited.get(succ)) {
							toVisit.add(succ);
						}
					}
					mdpCreator.addActionAndLabelRews(sState, action, successors, progRew, cost);
					addStateToMap(s, sState);

					addSuccessorStatesToMap(successors, mdpSuccInts);
				}
			}
		}
		this.mdpCreator.mdp.setVarList(mdp.getVarList());
		return mdpCreator.mdp;
	}

	void savePolicy(String saveLocation, String name)
	{
		mdpCreator.saveMDP(saveLocation, name);

	}
}
