package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Stack;

import explicit.MDP;
import explicit.MDPSimple;
import explicit.MDPSparse;
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

	private MDPSimple createPolicy(int initialState, MDP mdp, Strategy strat)
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




	void savePolicy(String saveLocation, String name)
	{
		mdpCreator.saveMDP(saveLocation, name);

	}
}
