package demos;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;

import explicit.MDP;
import parser.State;
import strat.Strategy;

//uses policy creator to make a path 
//so you can kind of mix a policy with like a predefined path 

public class PathCreator
{

	PolicyCreator pc = new PolicyCreator();

	public void createPathPolicy(int s, MDP m, Strategy strat, String savePath, String saveName)
	{

		pc.createPolicy(s, m, strat);
		State ps = m.getStatesList().get(s);
		pc.mdpCreator.setInitialState(ps);
		pc.savePolicy(savePath, saveName);
	}

	public void creatPath(int s, int a, MDP m, Strategy strat, String savePath, String saveName)
	{
		//from the mdp get the states at this action index 
		Iterator<Entry<Integer, Double>> tranIter = m.getTransitionsIterator(s, a);
		ArrayList<Entry<State, Double>> succPairs = new ArrayList<Entry<State, Double>>();
		while (tranIter.hasNext()) {

			Entry<Integer, Double> stateProb = tranIter.next();
			int state = stateProb.getKey();
			double prob = stateProb.getValue();
			State sState = m.getStatesList().get(state);

			succPairs.add(new AbstractMap.SimpleEntry<State, Double>(sState, prob));
			pc.createPolicy(state, m, strat);
		}
		State ps = m.getStatesList().get(s);
		Object action = m.getAction(s, a);
		pc.mdpCreator.addAction(ps, action, succPairs);
		pc.mdpCreator.setInitialState(ps);
		pc.savePolicy(savePath, saveName);
	}

	public void creatPath(ArrayList<Integer> states, ArrayList<Integer> actions, MDP m, Strategy strat, String savePath, String saveName)
	{

		//from the mdp get the states at this action index 
		//first create the path from the saPairs 
		//basically we just assume this is what happened 
		//but we can start with the first sa and choose to expand on the next sa 
		int s;
		int a;
		boolean initStateSet = false;
		for (int i = 0; i < states.size(); i++) {
			s = states.get(i);
			a = actions.get(i);
			Iterator<Entry<Integer, Double>> tranIter = m.getTransitionsIterator(s, a);
			ArrayList<Entry<State, Double>> succPairs = new ArrayList<Entry<State, Double>>();
			while (tranIter.hasNext()) {

				Entry<Integer, Double> stateProb = tranIter.next();
				int state = stateProb.getKey();
				double prob = stateProb.getValue();
				State sState = m.getStatesList().get(state);

				succPairs.add(new AbstractMap.SimpleEntry<State, Double>(sState, prob));
				if (!states.contains(state)) {
					pc.createPolicy(state, m, strat);
				}
			}
			State ps = m.getStatesList().get(s);
			Object action = m.getAction(s, a);
			pc.mdpCreator.addAction(ps, action, succPairs);

			if (!initStateSet) {
				pc.mdpCreator.setInitialState(ps);
				initStateSet = true;
			}
		}

		pc.savePolicy(savePath, saveName);
	}
}
