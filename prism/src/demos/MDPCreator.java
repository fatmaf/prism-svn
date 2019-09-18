/**
 * 
 */
package demos;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;

import explicit.Distribution;
import explicit.MDP;
import explicit.MDPSimple;
import parser.State;
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
		int s =  getStateIndex(js);
		

		MDPSimple mdp = this.mdp;
		double prob = getStateProb(s, mdp, 0, mdp.getNumStates());
		this.mainLog.println("Probability of satisfaction from state "+js.toString()+": "+prob);
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

	public void setEssState(State js)
	{
		int s = getStateIndex(js);
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

	public boolean addAction(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs, ArrayList<boolean[]> essAcc)
	{
		boolean res = addAction(s, a, successorsWithProbs);
		for (int i = 0; i < successorsWithProbs.size(); i++) {
			State ss = successorsWithProbs.get(i).getKey();
			boolean ess = essAcc.get(i)[0];
			boolean acc = essAcc.get(i)[1];
			if (ess)
				this.setEssState(ss);
			if (acc)
				this.setAccState(ss);
		}
		return res;
	}

	public boolean addAction(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs)
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
		return true;
	}

	//adds an action or parts of an action 
	public boolean addActionPartial(State s, Object a, ArrayList<Entry<State, Double>> successorsWithProbs)
	{

		boolean added = false;
		int stateIndex = getStateIndex(s);
		int actionIndex = getActionIndex(stateIndex, a);
		if (actionIndex == -1) {
			added = addAction(s, a, successorsWithProbs);
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
		return added;
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
		PrismLog out = new PrismFileLog(fn);
		mdp.exportToDotFile(out, statesToMark, true);
		out.close();
	}

	public void saveMDP(String saveLoc, String name)
	{
		saveMDP(mdp, saveLoc, name, null);

	}
}
