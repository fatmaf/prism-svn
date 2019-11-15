package demos;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;
import java.util.Queue;

import explicit.MDP;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import prism.PrismException;
import strat.Strategy;

//uses policy creator to make a path 
//so you can kind of mix a policy with like a predefined path 

public class XAIPathCreator
{

	PolicyCreator pc = new PolicyCreator();

	public Object getStateAction(State s)
	{
		Object action =null;
		int sI = pc.mdpCreator.getStateIndexNoAddition(s);
		if(sI !=-1)
		 action = pc.mdpCreator.mdp.getAction(sI, 0);
		return action;
	}

	public void createPathPolicy(int s, MDP m, Strategy strat, String savePath, String saveName, MDPRewardsSimple progRews, MDPRewardsSimple costs,
			BitSet accStates) throws PrismException
	{
		setPCVarList(m);
		boolean hasRewards = (progRews != null) & (costs != null);
		if (hasRewards)
			pc.createPolicyWithRewardsStructures(s, m, strat, progRews, costs, accStates);
		else
			pc.createPolicy(s, m, strat);
		State ps = m.getStatesList().get(s);
		pc.mdpCreator.setInitialState(ps);
		pc.mdpCreator.mdp.findDeadlocks(true);
		pc.savePolicy(savePath, saveName);
	}

	public void creatPath(int s, int a, MDP m, Strategy strat, String savePath, String saveName, MDPRewardsSimple progRews, MDPRewardsSimple costs,
			BitSet accStates) throws PrismException
	{
		setPCVarList(m);
		boolean hasRewards = (progRews != null) & (costs != null);
		//from the mdp get the states at this action index 
		Iterator<Entry<Integer, Double>> tranIter = m.getTransitionsIterator(s, a);
		ArrayList<Entry<State, Double>> succPairs = new ArrayList<Entry<State, Double>>();
		while (tranIter.hasNext()) {

			Entry<Integer, Double> stateProb = tranIter.next();
			int state = stateProb.getKey();
			double prob = stateProb.getValue();
			State sState = m.getStatesList().get(state);

			succPairs.add(new AbstractMap.SimpleEntry<State, Double>(sState, prob));
			pc.createPolicyWithRewardsStructures(state, m, strat, progRews, costs, accStates);
		}
		State ps = m.getStatesList().get(s);
		Object action = m.getAction(s, a);
		if (hasRewards) {
			double progRew = progRews.getTransitionReward(s, a);
			double cost = costs.getTransitionReward(s, a);
			pc.mdpCreator.addAction(ps, action, succPairs, progRew, cost);
		} else
			pc.mdpCreator.addAction(ps, action, succPairs);
		pc.mdpCreator.setInitialState(ps);
		pc.mdpCreator.mdp.findDeadlocks(true);
		pc.savePolicy(savePath, saveName);
	}

	public void creatPathFlex(ArrayList<Integer> states, ArrayList<Integer> actions, MDP m, Strategy strat, String savePath, String saveName,
			MDPRewardsSimple progCosts, MDPRewardsSimple costs, BitSet accStates) throws PrismException
	{
		setPCVarList(m);
		boolean hasRewards = (progCosts != null) & (costs != null);
		//from the mdp get the states at this action index 
		//first create the path from the saPairs 
		//basically we just assume this is what happened 
		//but we can start with the first sa and choose to expand on the next sa 
		int s;
		int a;
		boolean initStateSet = false;
		int initState = m.getFirstInitialState();
		Queue<Integer> stateQ = new LinkedList<Integer>();
		stateQ.add(initState);

		//so basically if the state that we've gotten is the initial state we can go ahead 
		//otherwise we wait till we get to the state that matches the first state 
		//then just keep matching. 
		//if we never do then we're in trouble 
		BitSet statesSeen = new BitSet();
		while (!stateQ.isEmpty()) {
			s = stateQ.remove();

			if (statesSeen.get(s))
				continue;
			else
				statesSeen.set(s);
			if (states.contains(s)) {
				int sIndex = states.indexOf(s);
				a = actions.get(sIndex);
			} else {
				strat.initialise(s);
				Object action = strat.getChoiceAction();
				a = pc.findActionIndex(m, s, action);
			}
			State ps = m.getStatesList().get(s);
			if(accStates.get(s))
				pc.mdpCreator.setAccState(ps);
			
			if (a != -1) {
				ArrayList<Integer> mdpSuccInts = new ArrayList<Integer>();
				Iterator<Entry<Integer, Double>> tranIter = m.getTransitionsIterator(s, a);
				ArrayList<Entry<State, Double>> succPairs = new ArrayList<Entry<State, Double>>();
				while (tranIter.hasNext()) {
					Entry<Integer, Double> stateProb = tranIter.next();
					int state = stateProb.getKey();
					double prob = stateProb.getValue();
					State sState = m.getStatesList().get(state);
					mdpSuccInts.add(state);
					succPairs.add(new AbstractMap.SimpleEntry<State, Double>(sState, prob));
					stateQ.add(state);
				}
				
				Object action = m.getAction(s, a);
				if (hasRewards) {
					double progRew = progCosts.getTransitionReward(s, a);
					double cost = costs.getTransitionReward(s, a);
					pc.mdpCreator.addAction(ps, action, succPairs, progRew, cost);
				} else
					pc.mdpCreator.addAction(ps, action, succPairs);
				
				pc.addStateToMap(s, ps);

				pc.addSuccessorStatesToMap(succPairs, mdpSuccInts);
			}
			if (!initStateSet) {
				pc.mdpCreator.setInitialState(ps);
				initStateSet = true;
			}

		}

		pc.mdpCreator.mdp.findDeadlocks(true);
		pc.savePolicy(savePath, saveName);
	}

	public void creatPath(ArrayList<Integer> states, ArrayList<Integer> actions, MDP m, Strategy strat, String savePath, String saveName,
			MDPRewardsSimple progCosts, MDPRewardsSimple costs, BitSet accStates) throws PrismException
	{
		setPCVarList(m);
		boolean hasRewards = (progCosts != null) & (costs != null);
		//from the mdp get the states at this action index 
		//first create the path from the saPairs 
		//basically we just assume this is what happened 
		//but we can start with the first sa and choose to expand on the next sa 
		int s;
		int a;
		boolean initStateSet = false;
		//so basically if the state that we've gotten is the initial state we can go ahead 
		//otherwise we wait till we get to the state that matches the first state 
		//then just keep matching. 
		//if we never do then we're in trouble 
		for (int i = 0; i < states.size(); i++) {
			s = states.get(i);
			a = actions.get(i);
			ArrayList<Integer> mdpSuccInts = new ArrayList<Integer>();
			Iterator<Entry<Integer, Double>> tranIter = m.getTransitionsIterator(s, a);
			ArrayList<Entry<State, Double>> succPairs = new ArrayList<Entry<State, Double>>();
			while (tranIter.hasNext()) {

				Entry<Integer, Double> stateProb = tranIter.next();
				int state = stateProb.getKey();
				double prob = stateProb.getValue();
				State sState = m.getStatesList().get(state);
				mdpSuccInts.add(state);
				succPairs.add(new AbstractMap.SimpleEntry<State, Double>(sState, prob));
				if (!states.contains(state)) {
					pc.createPolicyWithRewardsStructures(state, m, strat, progCosts, costs, accStates);
				}
			}
			State ps = m.getStatesList().get(s);
			Object action = m.getAction(s, a);
			if (hasRewards) {
				double progRew = progCosts.getTransitionReward(s, a);
				double cost = costs.getTransitionReward(s, a);
				pc.mdpCreator.addAction(ps, action, succPairs, progRew, cost);
			} else
				pc.mdpCreator.addAction(ps, action, succPairs);
			pc.addStateToMap(s, ps);

			pc.addSuccessorStatesToMap(succPairs, mdpSuccInts);
			if (!initStateSet) {
				pc.mdpCreator.setInitialState(ps);
				initStateSet = true;
			}
		}
		pc.mdpCreator.mdp.findDeadlocks(true);
		pc.savePolicy(savePath, saveName);
	}

	public void setPCVarList(MDP mdp)
	{
		if (pc.mdpCreator.mdp.getVarList() == null)
			pc.mdpCreator.mdp.setVarList((VarList) mdp.getVarList().clone());
	}
}
