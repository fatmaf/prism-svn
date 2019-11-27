/**
 * 
 */
package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;

import explicit.Distribution;
import explicit.MDP;
import explicit.MDPSimple;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

/**
 * @author fatma
 * a class that can be used to create an MDP 
 * why because there's plenty of book keeping to do 
 * and it can be annoying 
 * 
 * it can also be used to mask some MDP things that I use regularly 
 * cuz mein thak gai hun ye kar kar k 
 * so reusable code 
 * I will probably add reward structures to this too 
 * cuz like bus yaar 
 * 
 * hey old me this was a really great move 
 * i'm proud of you =D 
 */
public class MDPCreator
{

	boolean printMessages;
	MDPSimple mdp;
	HashMap<State, Integer> stateIndices;
	BitSet accStates;
	BitSet essStates;
	PrismLog mainLog;
	MDPRewardsSimple expectedTaskCompletionRewards = null;
	MDPRewardsSimple stateActionCostRewards = null;

	HashMap<Entry<Integer, Integer>, Double> stateActionTaskCompletionRewards = null;
	HashMap<Entry<Integer, Integer>, Double> stateActionCosts = null;

	public void createRewardStructures()
	{
		//the assumption is we're all done 
		//so we can just add stuff 
		if (stateActionCosts != null & stateActionTaskCompletionRewards != null) {
			expectedTaskCompletionRewards = new MDPRewardsSimple(mdp.getNumStates());
			stateActionCostRewards = new MDPRewardsSimple(mdp.getNumStates());

			for (Entry<Integer, Integer> saPair : stateActionTaskCompletionRewards.keySet()) {
				expectedTaskCompletionRewards.addToTransitionReward(saPair.getKey(), saPair.getValue(), stateActionTaskCompletionRewards.get(saPair));
			}
			for (Entry<Integer, Integer> saPair : stateActionCosts.keySet()) {
				stateActionCostRewards.addToTransitionReward(saPair.getKey(), saPair.getValue(), stateActionCosts.get(saPair));
			}
		}
	}

	public ArrayList<MDPRewardsSimple> getRewardsInArray()
	{
		ArrayList<MDPRewardsSimple> rewards = new ArrayList<MDPRewardsSimple>();
		rewards.add(expectedTaskCompletionRewards);
		rewards.add(stateActionCostRewards);
		return rewards;
	}

	public double getStateProb(int s, MDPSimple mdp, int depth, int maxDepth)
	{
		if (accStates.get(s))
			return 1.0;

		int numChoices = mdp.getNumChoices(s);
		if (numChoices == 0)
			return 0.0;
		if (depth >= maxDepth) //cuz we couldnt find the goal till then //possibly a loop and thats not cool 
			return 0.0;
		double prob = 0;
		for (int i = 0; i < numChoices; i++) {
			Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, i);
			while (tranIter.hasNext()) {
				Entry<Integer, Double> stateProb = tranIter.next();
				int ss = stateProb.getKey();
				double p = stateProb.getValue();
				prob += p * getStateProb(ss, mdp, depth + 1, maxDepth);

			}
		}
		return prob;
	}

	double getProbabilityToReachAccStateFromJointMDP(State js)
	{
		int s = getStateIndex(js);

		MDPSimple mdp = this.mdp;
		double prob = getStateProb(s, mdp, 0, mdp.getNumStates());
		this.mainLog.println("Probability of satisfaction from state " + js.toString() + ": " + prob);
		return prob;

	}

	public MDPCreator(PrismLog mainLog)
	{
		printMessages = false;
		mdp = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		mdp.setStatesList(statesList);
		stateIndices = new HashMap<State, Integer>();
		this.mainLog = mainLog;
		accStates = new BitSet();
		essStates = new BitSet();
	}

	public MDPCreator()
	{
		printMessages = false;
		mdp = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		mdp.setStatesList(statesList);
		stateIndices = new HashMap<State, Integer>();
		this.mainLog = null;
		accStates = new BitSet();
		essStates = new BitSet();
	}

	public void setAccState(State js)
	{
		int s = getStateIndex(js);
		accStates.set(s);
	}

	public void setAccState(int s)
	{

		accStates.set(s);
	}
	
	public boolean getAccState(State js)
	{
		int s = getStateIndex(js); 
		return accStates.get(s);
	}

	public void setEssState(State js)
	{
		int s = getStateIndex(js);
		essStates.set(s);
	}

	public void setEssState(int s)
	{

		essStates.set(s);
	}

	public void setPrintMessagesOn()
	{
		this.printMessages = true;
	}

	public void setPrintMessagesOff()
	{
		this.printMessages = false;
	}

	public boolean addState(State s)
	{
		boolean added = false;
		if (!stateIndices.containsKey(s)) {
			int stateIndex = mdp.getNumStates();
			mdp.addState();
			stateIndices.put(s, stateIndex);
			mdp.getStatesList().add(s);
			added = true;

		}
		return added;
	}

	int getStateIndex(State s)
	{
		if (!stateIndices.containsKey(s)) {
			boolean added = addState(s);
			if (added) {
				if (printMessages) {
					mainLog.println("State added");
				}
			}
		}
		return stateIndices.get(s);
	}
	int getStateIndexNoAddition(State s)
	{
		if (!stateIndices.containsKey(s)) {
			return -1;
		}
		return stateIndices.get(s);
	}
	int getActionIndex(int stateIndex, Object a)
	{
		int actionIndex = -1;
		if (stateIndex != -1) {
			int choices = mdp.getNumChoices(stateIndex);
			for (int c = 0; c < choices; c++) {
				Object act = mdp.getAction(stateIndex, c);
				if (act.toString().contentEquals(a.toString())) {
					actionIndex = c;
					break;
				}
			}
		}
		return actionIndex;
	}

	public int addAction(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs, ArrayList<Object[]> essAcc, double cost)
	{
		int actionIndex = addAction(s, a, successorsWithProbs);
		//if there are any essential states we get a reward 
		double rew = 0;
		for (int i = 0; i < successorsWithProbs.size(); i++) {
			State ss = successorsWithProbs.get(i).getKey();
			double prob = successorsWithProbs.get(i).getValue();

			int ess = (int) essAcc.get(i)[0];
			boolean acc = (boolean) essAcc.get(i)[1];
			if (ess > 0) {
				this.setEssState(ss);
				rew += prob * ((double) ess);
			}
			if (acc)
				this.setAccState(ss);

		}
		int stateIndex = getStateIndex(s);
		if (rew > 0) {
			if (stateActionTaskCompletionRewards == null) {
				stateActionTaskCompletionRewards = new HashMap<Entry<Integer, Integer>, Double>();

			}

			stateActionTaskCompletionRewards.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), rew);
		}
		if (cost > 0) {
			if (this.stateActionCosts == null)
				stateActionCosts = new HashMap<Entry<Integer, Integer>, Double>();
			stateActionCosts.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), cost);
		}

		return actionIndex;
	}

	public int addAction(int s, Object a, ArrayList<Entry<Integer, Double>> successorsWithProbs, ArrayList<Integer> numDAaccs, ArrayList<Boolean> isAcc,
			double cost)
	{
		int actionIndex = addAction(s, a, successorsWithProbs);
		//if there are any essential states we get a reward 
		double rew = 0;
		for (int i = 0; i < successorsWithProbs.size(); i++) {
			int ss = successorsWithProbs.get(i).getKey();
			double prob = successorsWithProbs.get(i).getValue();
			int ess = numDAaccs.get(i);
			boolean isAccState = isAcc.get(i);

			if (ess > 0) {
				this.setEssState(ss);
				rew += prob * ((double) ess);
			}
			if (isAccState)
				this.setAccState(ss);

		}
		int stateIndex = s;
		if (rew > 0) {
			if (stateActionTaskCompletionRewards == null) {
				stateActionTaskCompletionRewards = new HashMap<Entry<Integer, Integer>, Double>();

			}

			stateActionTaskCompletionRewards.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), rew);
		}
		if (cost > 0) {
			if (this.stateActionCosts == null)
				stateActionCosts = new HashMap<Entry<Integer, Integer>, Double>();
			stateActionCosts.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), cost);
		}

		return actionIndex;
	}

	public int addAction(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs)
	{

		//add an action to a state
		//does not check if the same action is added twice!!! 
		//it shouldnt be you know 
		Distribution distr = new Distribution();
		int stateIndex = getStateIndex(s);
		int actionIndex = mdp.getNumChoices(stateIndex);
		for (int i = 0; i < successorsWithProbs.size(); i++) {
			Entry<State, Double> stateProbPair = successorsWithProbs.get(i);
			State succState = stateProbPair.getKey();
			double prob = stateProbPair.getValue();
			int succStateIndex = getStateIndex(succState);
			distr.add(succStateIndex, prob);
		}
		mdp.addActionLabelledChoice(stateIndex, distr, a);
		return actionIndex;
	}
	public int addAction(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs,double progRew, double cost)
	{

		//add an action to a state
		//does not check if the same action is added twice!!! 
		//it shouldnt be you know 
		Distribution distr = new Distribution();
		int stateIndex = getStateIndex(s);
		int actionIndex = mdp.getNumChoices(stateIndex);
		for (int i = 0; i < successorsWithProbs.size(); i++) {
			Entry<State, Double> stateProbPair = successorsWithProbs.get(i);
			State succState = stateProbPair.getKey();
			double prob = stateProbPair.getValue();
			int succStateIndex = getStateIndex(succState);
			distr.add(succStateIndex, prob);
		}
		mdp.addActionLabelledChoice(stateIndex, distr, a);
		
		
		if (progRew > 0) {
			if (stateActionTaskCompletionRewards == null) {
				stateActionTaskCompletionRewards = new HashMap<Entry<Integer, Integer>, Double>();

			}

			stateActionTaskCompletionRewards.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), progRew);
		}
		if (cost > 0) {
			if (this.stateActionCosts == null)
				stateActionCosts = new HashMap<Entry<Integer, Integer>, Double>();
			stateActionCosts.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), cost);
		}
		
		
		
		return actionIndex;
	}
	
	public int addActionAndLabelRews(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs,double progRew, double cost)
	{

		//add an action to a state
		//does not check if the same action is added twice!!! 
		//it shouldnt be you know 
		Distribution distr = new Distribution();
		int stateIndex = getStateIndex(s);
		int actionIndex = mdp.getNumChoices(stateIndex);
		for (int i = 0; i < successorsWithProbs.size(); i++) {
			Entry<State, Double> stateProbPair = successorsWithProbs.get(i);
			State succState = stateProbPair.getKey();
			double prob = stateProbPair.getValue();
			int succStateIndex = getStateIndex(succState);
			distr.add(succStateIndex, prob);
		}
		mdp.addActionLabelledChoice(stateIndex, distr, a.toString()+"_t:"+progRew+"_c:"+cost);
		
		
		if (progRew > 0) {
			if (stateActionTaskCompletionRewards == null) {
				stateActionTaskCompletionRewards = new HashMap<Entry<Integer, Integer>, Double>();

			}

			stateActionTaskCompletionRewards.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), progRew);
		}
		if (cost > 0) {
			if (this.stateActionCosts == null)
				stateActionCosts = new HashMap<Entry<Integer, Integer>, Double>();
			stateActionCosts.put(new AbstractMap.SimpleEntry<Integer, Integer>(stateIndex, actionIndex), cost);
		}
		
		
		
		return actionIndex;
	}
	public int addAction(int s, Object a, ArrayList<Entry<Integer, Double>> successorsWithProbs)
	{

		//add an action to a state
		//does not check if the same action is added twice!!! 
		//it shouldnt be you know 
		Distribution distr = new Distribution();
		int stateIndex = s;//getStateIndex(s);
		int actionIndex = mdp.getNumChoices(stateIndex);
		for (int i = 0; i < successorsWithProbs.size(); i++) {
			Entry<Integer, Double> stateProbPair = successorsWithProbs.get(i);
			Integer succState = stateProbPair.getKey();
			double prob = stateProbPair.getValue();
			int succStateIndex = succState;//getStateIndex(succState);
			distr.add(succStateIndex, prob);
		}
		mdp.addActionLabelledChoice(stateIndex, distr, a);
		return actionIndex;
	}

	//adds an action or parts of an action 
	public int addActionPartial(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs)
	{

		boolean added = false;
		int stateIndex = getStateIndex(s);
		int actionIndex = getActionIndex(stateIndex, a);
		if (actionIndex == -1) {

			actionIndex = addAction(s, a, successorsWithProbs);
			added = actionIndex != -1;

		} else {
			//this action exists 
			//so we're just going to add to it 
			Distribution distr = mdp.getChoice(stateIndex, actionIndex);
			if (printMessages) {
				mainLog.println("Action " + a.toString() + " exists, adding to it");

				Iterator<Entry<Integer, Double>> distList = distr.iterator();
				String succStatesString = "Existing states: ";
				while (distList.hasNext()) {
					Entry<Integer, Double> stateProbPair = distList.next();
					State sas = mdp.getStatesList().get(stateProbPair.getKey());
					succStatesString += " " + sas.toString() + ":" + stateProbPair.toString();
				}
				mainLog.println(succStatesString);
			}
			for (int i = 0; i < successorsWithProbs.size(); i++) {
				Entry<State, Double> stateProbPair = successorsWithProbs.get(i);
				State state = stateProbPair.getKey();
				int succStateIndex = getStateIndex(state);
				double prob = stateProbPair.getValue();
				//if we dont have a transition to this state 
				//add it 
				if (distr.contains(succStateIndex)) {
					if (printMessages)
						mainLog.println(state.toString() + " already exists!! Not adding it.");
				} else {
					mdp.getChoice(stateIndex, actionIndex).add(succStateIndex, prob);
					if (!added)
						added = true;
				}

			}

		}
		return actionIndex;
	}

	public void setInitialState(State s)
	{
		int sIndex = getStateIndex(s);
		mdp.addInitialState(sIndex);
	}

	public List<State> getInitialStates()
	{
		List<State> initStates = new ArrayList<State>();
		Iterable<Integer> iterable = mdp.getInitialStates();
		Iterator<Integer> iter = iterable.iterator();
		while (iter.hasNext()) {
			int s = iter.next();
			State ss = mdp.getStatesList().get(s);
			initStates.add(ss);
		}
		return initStates;
	}

	public State getFirstInitialState()
	{
		int s = mdp.getFirstInitialState();
		return mdp.getStatesList().get(s);
	}

	public void saveMDP(MDP mdp, String saveLoc, String name, BitSet statesToMark)
	{
		String fn = saveLoc + name + ".dot";
		System.out.println("Saving MDP to "+fn);
		System.out.println(mdp.infoStringTable());
		PrismLog out = new PrismFileLog(fn);
		mdp.exportToDotFile(out, statesToMark, true);
		out.close();
	}
	public void saveMDPstatra(MDP mdp, String saveLoc, String name, BitSet statesToMark)
	{
		String fn = saveLoc + name + ".sta";
		System.out.println("Saving MDP states to "+fn);
		
		PrismLog out = new PrismFileLog(fn);
		try {
			mdp.exportStates(Prism.EXPORT_PLAIN, mdp.getVarList(), out);
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
//		mdp.exportToDotFile(out, statesToMark, true);
		out.close();
		fn = saveLoc + name + ".tra";
		System.out.println("Saving MDP transitions to "+fn);
		out = new PrismFileLog(fn);
		mdp.exportToPrismExplicitTra(out);
		out.close();
	}

	public void saveMDP(String saveLoc, String name)
	{
		saveMDP(mdp, saveLoc, name, null);

	}
	public void setVarList(VarList vl)
	{
		mdp.setVarList((VarList)vl.clone());
	}
}
