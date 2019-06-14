package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Stack;

import explicit.MDPSimple;
import parser.State;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

public class PolicyCreator
{

	MDPCreator mdpCreator;

	public PolicyCreator()
	{
		mdpCreator = new MDPCreator();
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
		mdpCreator.saveMDP(saveLocation,name);
		
	}
}
