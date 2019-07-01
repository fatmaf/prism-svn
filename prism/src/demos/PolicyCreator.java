package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Stack;

import explicit.MDP;
import explicit.MDPSimple;
import parser.State;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import strat.MDStrategy;
import strat.Strategy;

public class PolicyCreator
{

	MDPCreator mdpCreator;

	public PolicyCreator()
	{
		mdpCreator = new MDPCreator();
	}

	public MDPSimple createPolicy(DecisionNode rootNode, ActionSelection actSel, boolean upperbound) throws PrismException
	{
		Stack<THTSNode> toVisit = new Stack<THTSNode>();
		Stack<THTSNode> visited = new Stack<THTSNode>();
		THTSNode currNode = rootNode;
		DecisionNode currDecNode = rootNode;
		ChanceNode currChanceNode = null;
		boolean onChanceNode = false;
		toVisit.add(currNode);
		while (!toVisit.isEmpty()) {
			currNode = toVisit.pop();
			visited.add(currNode);
			if (currNode instanceof DecisionNode) {
				currDecNode = (DecisionNode) currNode;
				onChanceNode = false;
			} else {
				currChanceNode = (ChanceNode) currNode;
				onChanceNode = true;
			}

			if (onChanceNode) {
				//we have the action and the state 
				//now we just get the successors and add all of them 
				//but we also need to save these so we can add them to the MDP 
				ArrayList<DecisionNode> successorDecNodes = currChanceNode.getChildren();
				ArrayList<Entry<State, Double>> successors = new ArrayList<Entry<State, Double>>();
				for (DecisionNode succDecNode : successorDecNodes) {
					successors.add(new AbstractMap.SimpleEntry<State, Double>(succDecNode.getState(), succDecNode.getTranProb(currChanceNode)));
					if (!toVisit.contains(succDecNode) && !visited.contains(succDecNode)) {
						toVisit.add(succDecNode);
					}
				}
				mdpCreator.addAction(currChanceNode.getState(), currChanceNode.getAction(), successors);

			} else {
				Object action = actSel.selectActionBound(currDecNode, upperbound);
				currChanceNode = currDecNode.getChild(action);
				if (currChanceNode != null) {
					if (!toVisit.contains(currChanceNode) && !visited.contains(currChanceNode)) {
						toVisit.add(currChanceNode);
					}
				}
			}
		}
		return mdpCreator.mdp;
	}

	public MDPSimple createPolicy(MDP productMdp, Strategy strat) throws PrismException
	{
		int initialState = productMdp.getFirstInitialState();
		return createPolicy(initialState, (MDPSimple) productMdp, strat);
	}

	int findActionIndex(MDPSimple mdp, int s, Object a)
	{
		int numChoices = mdp.getNumChoices(s);
		int actionIndex = -1;
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

	private MDPSimple createPolicy(int initialState, MDPSimple mdp, Strategy strat)
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
				while (tranIter.hasNext()) {
					Entry<Integer, Double> stateProbPair = tranIter.next();
					int succ = stateProbPair.getKey();
					State succState = mdp.getStatesList().get(stateProbPair.getKey());
					double prob = stateProbPair.getValue();
					successors.add(new AbstractMap.SimpleEntry<State, Double>(succState, prob));
					if (!toVisit.contains(succ) && !visited.get(succ)) {
						toVisit.add(succ);
					}
				}
				mdpCreator.addAction(sState, action, successors);
			}
		}
		return mdpCreator.mdp;
	}

	public MDPSimple createPolicy(MDPSimple mdp, ActionSelection actionSelection) throws PrismException
	{
		int initialState = mdp.getFirstInitialState();
		return createPolicy(initialState, mdp, actionSelection);
	}

	public MDPSimple createPolicy(int initialState, MDPSimple mdp, ActionSelection actionSelection) throws PrismException
	{

		Stack<Integer> toVisit = new Stack<Integer>();
		BitSet visited = new BitSet();
		toVisit.add(initialState);
		int s;
		while (!toVisit.isEmpty()) {
			s = toVisit.pop();
			visited.set(s);
			State sState = mdp.getStatesList().get(s);

			int actionIndex = actionSelection.selectAction(s);
			Object action = mdp.getAction(s, actionIndex);

			Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, actionIndex);
			ArrayList<Entry<State, Double>> successors = new ArrayList<Entry<State, Double>>();
			while (tranIter.hasNext()) {
				Entry<Integer, Double> stateProbPair = tranIter.next();
				int succ = stateProbPair.getKey();
				State succState = mdp.getStatesList().get(stateProbPair.getKey());
				double prob = stateProbPair.getValue();
				successors.add(new AbstractMap.SimpleEntry<State, Double>(succState, prob));
				if (!toVisit.contains(succ) && !visited.get(succ)) {
					toVisit.add(succ);
				}
			}
			mdpCreator.addAction(sState, action, successors);

		}
		return mdpCreator.mdp;
	}

	void savePolicy(String saveLocation, String name)
	{
		mdpCreator.saveMDP(saveLocation, name);

	}
}
