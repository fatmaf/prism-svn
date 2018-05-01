package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismLog;
import strat.Strategy;
import explicit.Distribution;
import explicit.MDPSimple;

//import explicit.stateInfoFromStatesList;

/*
 * A class that stores the joint policy for multiple agents 
 */
public class MMDPSimple {

	public class StateProb implements Comparable<StateProb>
	{
		private double prob; 
		private int state; 
		
		public StateProb(int state, double prob)
		{
			this.prob = prob; 
			
			this.state = state; 
		}
		
		public StateProb (StateProb other)
		{
			this.prob = other.prob; 
			this.state = other.state;
		}
		public StateProb copy()
		{
			return new StateProb(this);
		}
		public int compareTo(StateProb other)
		{
			double comp = this.prob-other.prob; 
			int res = 0; 
			if(comp > 0 )
				res = -1; 
			else 
			{	if(comp < 0)
					res = 1; 
			}
			return res; 
		}
		
		public double getProb() {return this.prob; }
		public int getState() {return this.state; }

		@Override
		public String toString() {
			return "[s=" + state + ", p=" + prob + "]";
		}
		 
	}

	public MDPSimple mdp;
	// ArrayList<Map.Entry<State, Integer>>
	HashMap<State, Integer> statesMap;
	int nRobots;
	public PriorityQueue<StateProb> stuckStatesQ; 
	public BitSet deadendStates; 
	public BitSet allTasksCompletedStates; 
	public BitSet allFailStatesSeen;
	
	
	public boolean stuckStatesQContainsState(int state)
	{
		for (StateProb stateprob: stuckStatesQ)
		{
			if (stateprob.getState() == state)
				return true; 
		}
		return false; 
		
	}
	
	public void testQ() {
		PriorityQueue<StateProb> testQ = new PriorityQueue<StateProb>(); 
		double qvals[] = {0.5,0.1,0.2,1.4, 0.02};
		for(int i = 0; i<qvals.length; i++)
		{
			testQ.add(new StateProb(i,qvals[i])); 
			System.out.println("Added "+testQ.peek().toString());
		}
		while(!testQ.isEmpty())
		{
			StateProb meh = testQ.remove(); 
			System.out.println("Removed "+ meh.toString());
		}
	}

	/**
	 * creates a joint policy MDP
	 * 
	 * @numrobots the number of robots
	 * 
	 */
	public MMDPSimple(int numrobots) {
		stuckStatesQ = new PriorityQueue<StateProb>();
		mdp = new MDPSimple();
		VarList varlist = new VarList();
		try {
			for (int i = numrobots - 1; i >= 0; i--) {

				varlist.addVar(0, new Declaration("r" + i, new DeclarationIntUnbounded()), 1, null);

			}
			varlist.addVar(0, new Declaration("t", new DeclarationIntUnbounded()), 1, null);
		} catch (PrismLangException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		mdp.setVarList(varlist);
		mdp.setStatesList(new ArrayList<State>());
		statesMap = new HashMap<State, Integer>();
		nRobots = numrobots;
		deadendStates = new BitSet(); 
		allTasksCompletedStates = new BitSet(); 
		allFailStatesSeen = new BitSet();

	}

	public void constructJointPolicyFromMDPsAndAddToCurrentPolicy(MDPSimple[] pols, int[][] mdpMaps, int firstRobot, BitSet[] acceptingStates,
			SequentialTeamMDP seqTeamMDP) {
		int initialStates[] = new int[nRobots];
		int lastStates[] = new int[nRobots];
		// int switchStates[] = new int[nRobots];
		int maxSteps[] = new int[nRobots];
		int currentSteps[] = new int[nRobots];
		int nextStates[] = new int[nRobots];
		int currentStates[] = new int[nRobots];
		int mdpStates[] = new int[nRobots];
		String actions[] = new String[nRobots];
		Queue<State> statesQ = new LinkedList<State>();
		Queue<int[]> polStatesQ = new LinkedList<int[]>();
		BitSet discovered = new BitSet(); 
		ArrayList<Double> probabilities = null;
		ArrayList<State> nextJointStates = null;
		double sumProb = 0;
		updateInitialStates(pols, mdpMaps, firstRobot, acceptingStates,
				seqTeamMDP.teamMDPWithSwitches.getStatesList(),
				seqTeamMDP.acceptingStates
				);

		// for the first step get the states for all the robots
		for (int r = 0; r < nRobots; r++) {
			// for each robot get initial state
			initialStates[r] = pols[r].getFirstInitialState(); // TODO: CHANGE THIS TO SOMETHING FOR MULTIPLE
																// INITIAL STATES
			mdpStates[r] = StatesHelper.getMDPStateFromState(pols[r].getStatesList().get(initialStates[r]));
			
			lastStates[r] = getLastState(pols[r], acceptingStates[r]);// findStateWithSwitch(pols[r]);
			// get the DFA for the first state - its basically the changes from the initial
			// state
			// now lets get the state such that we keep the DFA progress for the initial
			// state

		}
		Object[] teamProgress = getTeamProgressAtCurrentStates(pols, mdpMaps, firstRobot, acceptingStates,
				initialStates, lastStates);
		currentStates = initialStates.clone();
		State currentJointState = createJointState(teamProgress, mdpStates);
		statesQ.add(currentJointState);
		polStatesQ.add(currentStates.clone());
		double prob;
		while (!polStatesQ.isEmpty()) {
			currentStates = polStatesQ.remove();

			currentJointState = statesQ.remove();
			int stateNumInMDP = StatesHelper.BADVALUE; 
			if(statesMap.get(currentJointState)!=null) {
				stateNumInMDP = statesMap.get(currentJointState);
			if(stateNumInMDP != StatesHelper.BADVALUE)
			{
				if(discovered.get(stateNumInMDP))
					{
					continue;
					
					}
			}
			}
			int hasfailed = hasFailState(currentJointState);

			
			// is this an accepting state ?
			boolean hasAcceptingState = false;
			//check if this is an accepting state 
		
			int[] robotStates =getRobotStatesIndexFromJointState(currentJointState,seqTeamMDP.teamMDPWithSwitches.getStatesList());
			
			// now get the successor states
			int[] numSuccessorStates = new int[nRobots];
			ArrayList<ArrayList<Entry<Integer, Double>>> successorStates = new ArrayList<ArrayList<Entry<Integer, Double>>>(
					nRobots);
			int sumNumChoices = 0;
			String action = "";
			for (int r = 0; r < nRobots; r++) {

				if(seqTeamMDP.acceptingStates.get(robotStates[r]))
					hasAcceptingState = true; 

				int numChoices = pols[r].getNumChoices(currentStates[r]);
				if (numChoices > 0) {
					Object actionhere = pols[r].getAction(currentStates[r], 0);
					if (actionhere != null) {
						// is there a better way to do this ???
						if (actionhere.toString().contains("switch")) {
							numChoices = 0;
						}
					}
				}
				sumNumChoices += numChoices;
				if (numChoices == 1) {
					numSuccessorStates[r] = pols[r].getNumTransitions(currentStates[r], 0);
					action += pols[r].getAction(currentStates[r], 0);
					// hardcoding the 0 because we know there is only one choice always.
					ArrayList<Entry<Integer, Double>> succStates = getSuccessorStatesFromMDP(pols[r], currentStates[r],
							0);
					successorStates.add(succStates);

				} else {
					// has more than 1 ?
					if (numChoices == 0) {
						numSuccessorStates[r] = 1;
						ArrayList<Entry<Integer, Double>> succStates = new ArrayList<Entry<Integer, Double>>();
						succStates.add(new AbstractMap.SimpleEntry(currentStates[r], 1.0));
						successorStates.add(succStates);

						// link back to the same state really - this can be useful later too
						// right ???
					} else {
						System.out.println("KUCH HOGIYA HAI YAAR");
					}
				}
			}
			// if all the choices are 0 we've reached a terminal state and it should be a
			// fail state
			// we also need to avoid cases where a state may have stuff in the joint policy
			// too
			// so if its a state that has an action in the joint policy that means we dont
			// care
			if (sumNumChoices == 0) {
				if (hasfailed > 0 && hasfailed != nRobots) {
					 stateNumInMDP = statesMap.get(currentJointState);
					if (mdp.getNumChoices(stateNumInMDP) == 0) {
						if (!hasAcceptingState) {
							if (!stuckStatesQContainsState(stateNumInMDP))
								{
								if(!allFailStatesSeen.get(stateNumInMDP)) {
									
									double failprob = getProbability(0,stateNumInMDP,1.0,new BitSet());
								stuckStatesQ.add(new StateProb(stateNumInMDP,failprob));
								allFailStatesSeen.set(stateNumInMDP);
								}
								else 
									deadendStates.set(stateNumInMDP);
								}
							
						}
						else
						{
							allTasksCompletedStates.set(stateNumInMDP);	
						}
					}
				}
				else
				{
					 stateNumInMDP = statesMap.get(currentJointState);
					//all robots failed 
					if(hasfailed == nRobots)
					{
						deadendStates.set(stateNumInMDP);
					}
					//an accepting state ? 
					if(hasAcceptingState)
					{
						allTasksCompletedStates.set(stateNumInMDP);
					}
				}
				continue;
			}
			ArrayList<int[]> combinations = new ArrayList<int[]>();
			probabilities = new ArrayList<Double>();
			nextJointStates = new ArrayList<State>();
			sumProb = 0;
			generateCombinations(numSuccessorStates.clone(), numSuccessorStates.clone(), combinations);
			for (int[] combination : combinations) {
				prob = 1.0;
				for (int r = 0; r < nRobots; r++) {
					nextStates[r] = successorStates.get(r).get(combination[r] - 1).getKey();
					prob = prob * successorStates.get(r).get(combination[r] - 1).getValue();
					mdpStates[r] = StatesHelper.getMDPStateFromState(pols[r].getStatesList().get(nextStates[r]));

				}

				teamProgress = getTeamProgressAtCurrentStates(pols, mdpMaps, firstRobot, acceptingStates, nextStates,
						lastStates);
				State nextJointState = createJointState(teamProgress, mdpStates);
				probabilities.add(prob);
				sumProb += prob;
				nextJointStates.add(nextJointState);
				statesQ.add(nextJointState);
				polStatesQ.add(nextStates.clone());

			}
			addTranstionToMDP(currentJointState, nextJointStates, probabilities, action, sumProb);
			stateNumInMDP = statesMap.get(currentJointState);
			discovered.set(stateNumInMDP);

		}
		saveJointPolicy();

	}
	
	private int addStateToMDP(State s) {
		int index = findStateIndex(s);
		if (index == StatesHelper.BADVALUE) {
			mdp.getStatesList().add(s);
			index = mdp.getNumStates();
			statesMap.put(s, index);
			mdp.addState();

		}
		return index;
	}

	private void addTranstionToMDP(State parentStates, ArrayList<State> states, ArrayList<Double> probs, String action,
			double norm) {
		int parentIndex = addStateToMDP(parentStates);
		int index;
		Distribution distr = new Distribution();

		for (int succ = 0; succ < states.size(); succ++) {
			index = addStateToMDP(states.get(succ));
			distr.add(index, probs.get(succ) / norm);

		}
		int actionNo = mdp.getNumChoices(parentIndex);
		mdp.addActionLabelledChoice(parentIndex, distr, action);

	}

	private boolean allDone(int[] currentSteps, int[] maxSteps) {
	boolean toret = true;
	for (int i = 0; i < currentSteps.length; i++) {
		if (currentSteps[i] < maxSteps[i] - 1) {
			toret = false;
			break;
		}
	}
	return toret;
}

	private boolean checkCombinationCounter(int arr[], int start, int end) {
		int endcondition = 1;
		boolean allDone = true;
		for (int i = start; i < end; i++) {
			if (arr[i] != endcondition) {
				allDone = false;
				break;
			}
		}
		return allDone;
	}

	
	public State createJointState(Object[] DFAProgress, int[] currentRobotStates) {
		State currentJointState = new State(DFAProgress.length + currentRobotStates.length);
		int offset = 0;
		for (int i = 0; i < DFAProgress.length; i++) {
			currentJointState.setValue(i + offset, DFAProgress[i]);
		}
		offset = DFAProgress.length;
		for (int i = 0; i < currentRobotStates.length; i++) {
			currentJointState.setValue(i + offset, currentRobotStates[i]);
		}
		return currentJointState;
	}

	public Object[] createSingleRobotState(State jointState, int rNum) {
		// pick the robot num
		Object[] mdpStates = jointState.varValues;
		int dfaStart = 0;
		int dfaEnd = mdpStates.length - nRobots;
		int robotIndex = dfaEnd + rNum;
		Object[] singleRobotState = new Object[dfaEnd + 2];
		singleRobotState[0] = rNum;
		for (int i = 1; i < dfaEnd + 1; i++) {
			singleRobotState[i] = mdpStates[i - 1];
		}
		singleRobotState[dfaEnd + 1] = mdpStates[robotIndex];
		return singleRobotState;
	}
	

	public int findStateIndex(State s) {
		int indexInt = StatesHelper.BADVALUE;
		Object index = statesMap.get(s);
		if (index != null) {
			indexInt = (int) index;
		}
		return indexInt;
	}

	
	public int findStateWithSwitch(MDPSimple pol) {
		int switchState = StatesHelper.BADVALUE;
		int state = pol.getFirstInitialState(); // TODO: CHANGE THIS TO SOMETHING FOR MULTIPLE
												// INITIAL STATES
		LinkedList<Integer> statesQ = new LinkedList<Integer>();
		statesQ.add(state);
		while (!statesQ.isEmpty()) {
			state = statesQ.remove();
			int choices = pol.getNumChoices(state);
			for (int i = 0; i < choices; i++) {
				if (pol.getAction(state, i).toString().contains("switch")) {
					switchState = state;
					break;
				} else {
					Iterator<Entry<Integer, Double>> tIter = pol.getTransitionsIterator(state, i);
					while (tIter.hasNext()) {
						Entry<Integer, Double> child = tIter.next();
						int childState = child.getKey();
						statesQ.add(childState);
					}
				}

			}
		}
		return switchState;

	}

	public int firstFailedRobot(Object[] mdpStates, int start, int end) {
		int failedRobot =  StatesHelper.BADVALUE; // no one
		for (int i = start; i < end; i++) {
			if ((int) mdpStates[i] == StatesHelper.failState) {
				failedRobot = i - start;
				break;
			}
		}
		return failedRobot;
	}

	public int firstFailedRobot(State jointState) {
		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
		int end = mdpStates.length;
		return firstFailedRobot(mdpStates, start, end);

	}

	private void generateCombinations(int counter[], int original[], ArrayList<int[]> res) {
		// System.out.println(Arrays.toString(counter));
		res.add(counter.clone());
		if (!checkCombinationCounter(counter, 0, counter.length)) {
			boolean do0 = true;
			for (int i = counter.length - 1; i >= 0; i--) {
				if (checkCombinationCounter(counter, 0, i + 1)) {
					counter[i + 1]--;
					counter[i] = original[i];
					do0 = false;
					break;
				}
			}
			if (do0) {
				counter[0]--;
			}
			generateCombinations(counter, original, res);

		}
	}

	public int getLastState(MDPSimple pol) {
		int state = pol.getFirstInitialState(); // TODO: CHANGE THIS TO SOMETHING FOR MULTIPLE
		// INITIAL STATES
		LinkedList<Integer> statesQ = new LinkedList<Integer>();

		statesQ.add(state);
		while (!statesQ.isEmpty()) {
			state = statesQ.remove();
			int choices = pol.getNumChoices(state);
			for (int i = 0; i < choices; i++) {
				Iterator<Entry<Integer, Double>> tIter = pol.getTransitionsIterator(state, i);
				while (tIter.hasNext()) {
					Entry<Integer, Double> child = tIter.next();
					int childState = child.getKey();
					statesQ.add(childState);
				}
			}

		}
		return state; // so basically this is the last state we tried to explore
	}

	//TODO: this can be more efficient 
	public int getLastState(MDPSimple pol, BitSet acceptingStates) {
		int state = getLastState(pol, acceptingStates, 0);
		for (int kindOfState = 1; (state ==  StatesHelper.BADVALUE) && (kindOfState < 3); kindOfState++) {
			state = getLastState(pol, acceptingStates, kindOfState);
		}
//		StatesHelper.saveMDP(pol, null, "", "polx", true);
		if(state == StatesHelper.BADVALUE)
		{
			//if there is no accepting state, 
			//no switch state 
			//no fail state 
			//then we can just do a dfs and get the last last state 
			int kindOfState = 3; 
			state = getLastState(pol, acceptingStates, kindOfState);
		}
		return state;
	}

	// kindOfState - 0 - accepting state
	// - 2 - fail state
	// - 1 - switch
	// - 3 - just the last state 
	public int getLastState(MDPSimple pol, BitSet acceptingStates, int kindOfState) {
		
		int state = pol.getFirstInitialState(); // TODO: CHANGE THIS TO SOMETHING FOR MULTIPLE
		// INITIAL STATES
		int lastState =  StatesHelper.BADVALUE;
		LinkedList<Integer> statesQ = new LinkedList<Integer>();
		BitSet discovered = new BitSet(); 

		statesQ.add(state);
		while (!statesQ.isEmpty()) {
			state = statesQ.remove();
			if(!discovered.get(state)) {
			discovered.set(state);
			if (kindOfState == 0) {
				if (acceptingStates.get(state))
					lastState = state;
			}
			if (kindOfState == 2) {
				if (StatesHelper.isFailState(pol.getStatesList().get(state)))
					lastState = state;
			}
			int choices = pol.getNumChoices(state);
			for (int i = 0; i < choices; i++) {
				if (kindOfState == 1) {
					if (pol.getAction(state, i) != null) {
						if (pol.getAction(state, i).toString().contains("switch"))
							lastState = state;
					}
				}
				Iterator<Entry<Integer, Double>> tIter = pol.getTransitionsIterator(state, i);
				while (tIter.hasNext()) {
					Entry<Integer, Double> child = tIter.next();
					int childState = child.getKey();
					statesQ.add(childState);
				}
			}
			}
		}
		if(kindOfState == 3)
		{
			//just get the last state in the dfs 
			lastState = state; 
		}
		
		return lastState; // so basically this is the last state we tried to explore
	}

	public ArrayList<Integer> getNextStates(MDPSimple pol, int currentState) {
		ArrayList<Integer> nextstates = null;
		int choices = pol.getNumChoices(currentState);
		if (choices > 0)
			nextstates = new ArrayList<Integer>();
		for (int i = 0; i < choices; i++) {

			Iterator<Entry<Integer, Double>> tIter = pol.getTransitionsIterator(currentState, i);
			while (tIter.hasNext()) {
				Entry<Integer, Double> child = tIter.next();
				int childState = child.getKey();
				nextstates.add(childState);
			}
		}
		return nextstates;

	}


	public int[] getRobotStates(State jointState) {

		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
		int end = mdpStates.length;
		int[] robotStates = new int[nRobots];
		for (int i = start; i < end; i++) {
			robotStates[i - start] = (int) mdpStates[i];
		}
		return robotStates;
	}




	public ArrayList<Entry<Integer, Double>> getSuccessorStatesFromMDP(MDPSimple pol, int state, int choice) {
		ArrayList<Entry<Integer, Double>> successorStates = new ArrayList<Entry<Integer, Double>>();
		Iterator<Entry<Integer, Double>> tIter = pol.getTransitionsIterator(state, choice);
		while (tIter.hasNext()) {
			Entry<Integer, Double> currIter = tIter.next();
			successorStates.add(currIter);
		}

		return successorStates;
	}

	public Object[] getTeamProgressAtCurrentStates(MDPSimple[] pols, int[][] mdpMaps, int firstRobot,
			BitSet[] acceptingStates, int[] currentStates, int[] lastStates) {

		// Ref State ???
		// We need to make one
		// TODO: NEED TO CHANGE THIS I AM HARDCODING IT FOR NOW

		// I need two things - the current progress of the the thing which starts from
		// the first robot
		// this is our base, then we need the progress of the next robot excluding
		// anything done by the first
		Object[] refState = StatesHelper.createRefStateObject();
		Object[] teamProgress = new Object[refState.length - 2]; // hard coding this too
		// initialize team Progress
		int firstRobotState = currentStates[firstRobot];
		State firstRobotStateObj = pols[firstRobot].getStatesList().get(firstRobotState);
		for (int i = 0; i < teamProgress.length; i++) {
			teamProgress[i] = firstRobotStateObj.varValues[i + 1];
		}
		for (int i = 0; i < nRobots; i++) {
			if (i != firstRobot) {
				int currentRobotState = currentStates[i];

				// get the last state of the i-1 thing
				// unless you're at 0 in which case you need to go to nRobots-1
				int prevRobot = i - 1;
				if (prevRobot == -1) {
					prevRobot = nRobots - 1;
				}

				// now I get the last state
				int prevRobotLastState = lastStates[prevRobot];

				State currentRobotStateObj = pols[i].getStatesList().get(currentRobotState);
				State prevRobotStateObj = pols[prevRobot].getStatesList().get(prevRobotLastState);

				Object[] robotProgress = StatesHelper.XORStates(prevRobotStateObj,
						currentRobotStateObj, refState);
//				System.out.println(Arrays.toString(robotProgress));
				teamProgress = ORObjectArrays(teamProgress, robotProgress);
//				System.out.println(Arrays.toString(teamProgress));
				// policyVar.addStateMDP(pols[i],mdpMaps[i],newState,acceptingStates[i]);
				// pols[i].clearInitialStates();
				// pols[i].addInitialState(mdpMaps[i][newState]);

			}
		}
		return teamProgress;
	}

	public int hasFailState(int[] mdpStates) {
		int sum = 0;
		for (int i = 0; i < mdpStates.length; i++) {
			if (mdpStates[i] == StatesHelper.failState)
				sum++;
		}
		return sum;
	}

	public int hasFailState(Object[] mdpStates, int start, int end) {
		int sum = 0;
		for (int i = start; i < end; i++) {
			if ((int) mdpStates[i] == StatesHelper.failState)
				sum++;
		}
		return sum;
	}

	public int hasFailState(State jointState) {
		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
		int end = mdpStates.length;
		return hasFailState(mdpStates, start, end);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#hashCode()
	 */
	@Override
	public int hashCode() {
		// TODO Auto-generated method stub
		return super.hashCode();
	}
	
	private void incrementSteps(int[] currentSteps, int[] maxSteps) {
		int inc = 1;
		for (int i = 0; i < currentSteps.length; i++) {
			if (currentSteps[i] + inc < maxSteps[i]) {
				currentSteps[i] += inc;
			}
		}
	}
	
	public Object[] ORObjectArrays(Object[] a1, Object[] a2) {
		for (int i = 0; i < a1.length; i++) {
			if ((int) a1[i] < (int) a2[i])
				a1[i] = a2[i];
		}
		return a1;
	}

		public void saveJointPolicy() {
			String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";

			PrismLog log = new PrismFileLog(saveplace + "jointPolicy.dot");
			mdp.exportToDotFile(log, null, true);
			log.close();

		}

	public boolean stateInStatesList(State s) {
		if (mdp.getStatesList().contains(s))
			return true;
		else
			return false;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		// TODO Auto-generated method stub
		return super.toString();
	}

	public ArrayList<Integer> addSeqPolicyToJointPolicy(SequentialTeamMDP seqTeamMDP,
			Strategy strat, int initialState, boolean noJointState) throws PrismException
	{

		int nextRobotstate =  StatesHelper.BADVALUE;

		int rNum = 0; 
		int[] robotStates = null; 
		int nextRobot = 0 ;
		List<State> states;
		State currState;// = states.get(initialState);
		if(!noJointState)
		{
			states = mdp.getStatesList(); 
			currState = states.get(initialState);
			states = seqTeamMDP.teamMDPTemplate.getStatesList();
			rNum = firstFailedRobot(currState);
		 robotStates = getRobotStatesIndexFromJointState(currState,states);
		int robotStateId = robotStates[rNum];
	
		}
		else
		{
			states = seqTeamMDP.teamMDPWithSwitches.getStatesList();
			currState = states.get(initialState);
			//lets get some information from the model itself 
			//if we dont have initial states 
			rNum = StatesHelper.getRobotNumberFromState(seqTeamMDP.teamMDPWithSwitches.getStatesList().get(initialState)); 
			robotStates = new int[nRobots]; 
			for(int i = 0; i<nRobots; i++)
			{
				if(i == rNum)
				{
					//we dont want mdp states 
					robotStates[i] = initialState;
				}
				else
				{
					int anyinitstate = seqTeamMDP.initialStates.get(i).nextSetBit(0); 
					robotStates[i] = anyinitstate;
					
				}
			}
		 
		}
		 nextRobot = (rNum+1)% nRobots;
			nextRobotstate = robotStates[nextRobot] ;
		
//
//		seqTeamMDP.setInitialStates(robotStates);
//		seqTeamMDP.addSwitchesAndSetInitialState(rNum);
		initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		State s1  = seqTeamMDP.teamMDPTemplate.getStatesList().get(initialState);
		System.out.println(s1.toString());
		return addSeqPolicy(seqTeamMDP,strat,initialState,nextRobotstate,robotStates);
		
		
		
	}

	public ArrayList<Integer> addSeqPolicy(SequentialTeamMDP seqTeamMDP,
			Strategy strat, int initialState, 
			int nextSuccState, int[] allRobotInitStates)
	{
		boolean debugStuff = true; 
		MDPSimple tempMDP = null;
		int[] tempMDPMap = null;
		BitSet tempAccStates = null; 
		MDPSimple sumprod = seqTeamMDP.teamMDPWithSwitches;
		int numRobots = seqTeamMDP.agentMDPs.size();
		//n MDP stuff 
		MDPSimple mdps[]= new MDPSimple[numRobots];
		int mdpMaps[][] = new int[numRobots][sumprod.getNumStates()];
		BitSet accStates[] = new BitSet[numRobots];
		for(int i=0; i<numRobots; i++)
		{	mdps[i]= new MDPSimple();
			mdps[i].setVarList((VarList)sumprod.getVarList().clone());
			Arrays.fill(mdpMaps[i],  StatesHelper.BADVALUE);
			mdps[i].setStatesList(new ArrayList<State>());	
			accStates[i] = new BitSet();
		}
		
		if (debugStuff)
		 {
			tempMDP = new MDPSimple();
			tempMDPMap = new int[sumprod.getNumStates()];
			Arrays.fill(tempMDPMap, StatesHelper.BADVALUE);
			tempAccStates = new BitSet();
			tempMDP.setVarList((VarList)sumprod.getVarList().clone());
			tempMDP.setStatesList(new ArrayList<State>());
		 }
		//n MDP stuff done
		ArrayList<Integer> statesDiscovered = new ArrayList<Integer>();
		int statesForPolicy = 0;

		int choices = -1;

		Object action = null;

		// this shouldnt matter
		List<Integer> discoveredStates = new LinkedList<Integer>();
		// on second thought this does not matter (the thing below) because we might
		// visit the same state again but it may be a child of another state
		// and so the inital state of the robot would have changed (though maybe no new
		// task would be acheived)
		// BitSet addedToStuckQ = new BitSet(tAutomaton.numStates); //for each policy
		// unfolding we don't need to add the same state over and over again to the
		// stuck stack

		Stack<Integer> statesStack = new Stack<Integer>(); // to store all the states we need to explore, so dfs
																	// to the goal , well its not dfs but yeah
		int currState = initialState;
		int firstRobot = StatesHelper.getRobotNumberFromState(sumprod.getStatesList().get(currState));
		statesStack.push(currState);
		// TODO: what do you do when the policy has been computed before and we're just
		// here again ?
		// Do we care about this here or later ?
	
		String prevTextToAdd = "";
		int timecount =0;
		while (!statesStack.isEmpty()) // the main bit - go to the next state, get the action go to the next state and
										// so on
		{
			// TODO: addstatetopolmdp(currState.state)
			// TODO: distribution
			currState = statesStack.pop();


			strat.initialise(currState);
			action = strat.getChoiceAction();
			
			statesForPolicy++;
			// just making sure we have enough elements in the array
	
			if (!discoveredStates.contains(currState)
					// !discovered.get(currState.state)
					& !seqTeamMDP.acceptingStates.get(currState)) // if it is not an accepting state and not
																		// discovered explore
			{
	
				// mainLog.println(currState.toString());
				// discovered.set(currState.state);
				discoveredStates.add(currState);
				choices = sumprod.getNumChoices(currState);
				for (int c = 0; c < choices; c++) {
					if (action.equals(sumprod.getAction(currState, c))) {
						// TODO: the prodDistr stuff
						ArrayList<Integer> states = new ArrayList<Integer>();
						ArrayList<Double> probs = new ArrayList<Double>();
						// TODO: adding to the MDP policy
						Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(currState, c);
						while (iter.hasNext()) {
							Entry<Integer, Double> stateProbPair = iter.next();
							// get which robot
							int nextState = stateProbPair.getKey(); 
							double nextStateProb = stateProbPair.getValue();
						
							statesStack.push(nextState);
							states.add(nextState);
							probs.add(nextStateProb);
						}
						int currentRobot = StatesHelper.getRobotNumberFromState(sumprod.getStatesList().get(currState));
						accStates[currentRobot] = StatesHelper.addLinkInMDP(mdps[currentRobot],mdpMaps[currentRobot],
								sumprod.getStatesList(),
								states, probs, currState, action,
								"r"+currentRobot,accStates[currentRobot],seqTeamMDP.acceptingStates);
						if(mdps[currentRobot].getNumInitialStates()==0)
							mdps[currentRobot].addInitialState(0);
						if(debugStuff)
						{
							tempAccStates= StatesHelper.addLinkInMDP(tempMDP, tempMDPMap, sumprod.getStatesList(), states, 
									probs, currState, action, "", tempAccStates, seqTeamMDP.acceptingStates);
						}
						// addLinkInPolicyMDP(states,probs, currState.state, action);
						break;
						// don't go over all choices - actions are unique
					}
				}

			}

		}
		// once we're done add the current policy to the big policy
		// and also check if the policy has at least init values for all robots
		// if(allRobotInitStates != null)
		// checkCurrentPolicy(allRobotInitStates,rts);
		StatesHelper.saveMDP(tempMDP, tempAccStates, "", "tempStrat"+initialState, true);
		StatesHelper.checkMDPs(mdps,mdpMaps,sumprod.getStatesList(),allRobotInitStates,
				initialState,accStates,seqTeamMDP.acceptingStates);
		constructJointPolicyFromMDPsAndAddToCurrentPolicy(mdps,mdpMaps,firstRobot,accStates,seqTeamMDP);
		statesDiscovered.add(statesForPolicy);
		
		return statesDiscovered;

	}

	public void updateInitialStates(MDPSimple[] pols, int[][] mdpMaps, 
			int firstRobot, BitSet[] acceptingStates,
			List<State> teamMDPStates,BitSet teamAccStates) {
		
		//note to self - being lazy pays no one 
		//do it right the first time! 
		
		// basically sometimes a robot might not be used at all
		// so what we want to be able to do is like update it's possible state
		// how can we figure this out - if we don't have a policy for this robot
		// that means it has just one state and no action
//		boolean doneOnce = false;
		for (int i = 0; i < nRobots; i++) {
			if ((pols[i].getNumStates() == 1) && (pols[i].getNumChoices(pols[i].getFirstInitialState()) == 0)
					&& (i != firstRobot)) {
				int currentRobotState = pols[i].getFirstInitialState();
				// get the last state of the i-1 thing
				// unless you're at 0 in which case you need to go to nRobots-1
				int prevRobot = i - 1;
				if (prevRobot == -1) {
					prevRobot = nRobots - 1;
				}
//				if (!doneOnce)
//					doneOnce = true;
//				else {
//					System.out.println("Something Unexpected is happening");
//				}
				// now I get the last state
				int prevRobotLastState = getLastState(pols[prevRobot], acceptingStates[prevRobot]);

				// now make a new state from the last state of the previous
				// robot and this state
				// more maslay!!! like how do I know this is the real state
				// what if we have more than one last state ??
				// which one do we choose ? how ?
				// we choose the accepting state
				// because thats the only reason this robot does not have a thing
				// or well it failed in all cases in which case we choose the fail state
				// I have fixed these maslay - we do a priority thing where we care about
				// the last accepting state and if we find no accepting state the last switch
				// and if we find no switch we care about the last fail state
				// so we're really chilling
				// so now that we have this what do we do ?
				// we make that state
				// have to make sure this new state is part of the team automaton btw
				

				int newState = StatesHelper.getMergedStateRobotMDP(
						pols[prevRobot].getStatesList().get(prevRobotLastState),
						pols[i].getStatesList().get(currentRobotState),teamMDPStates);
				StatesHelper.addStateMDP(pols[i], mdpMaps[i], teamMDPStates,  StatesHelper.BADVALUE,
						newState, acceptingStates[i],teamAccStates);
				pols[i].clearInitialStates();
				pols[i].addInitialState(mdpMaps[i][newState]);

			}
		}
	}

	
	
	public double getProbabilityAcceptingStateOnly(int startState, double probHere,BitSet discoveredStates)
	{
		int choice = 0; 
		double probSum = 0; 
		if(!allTasksCompletedStates.get(startState))
		{

			int numChoice = mdp.getNumChoices(startState); 
			if(!discoveredStates.get(startState)) {
			discoveredStates.set(startState);
			if(numChoice == 1)
			{
				Object action = mdp.getAction(startState, choice); 
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(startState, choice); 
				while(tranIter.hasNext())
				{
					Entry<Integer, Double> stateProbPair = tranIter.next(); 
					int nextState = stateProbPair.getKey(); 
					double nextStateProb = stateProbPair.getValue(); 
					probSum+=getProbabilityAcceptingStateOnly(nextState,nextStateProb*probHere,discoveredStates);
				}
				probHere = probSum; 
			}
			else
				probHere = 0; //no chance of getting to an accepting state
		}
//			else
//			{
//				//its a self loop so we will just stop here 
//				probHere = 0; 
//			}
		}
		return probHere;
	}

	
	public double getProbability(int startState, int stopState,double probHere,BitSet discovered)
	{
		int choice = 0; 
		double probSum = 0; 
		
		if(startState != stopState)
		{
			if(!discovered.get(startState)) {
			int numChoice = mdp.getNumChoices(startState); 
			discovered.set(startState);
			if(numChoice == 1)
			{
				Object action = mdp.getAction(startState, choice); 
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(startState, choice); 
				while(tranIter.hasNext())
				{
					Entry<Integer, Double> stateProbPair = tranIter.next(); 
					int nextState = stateProbPair.getKey(); 
					double nextStateProb = stateProbPair.getValue(); 
					//stop at the first failstate 
					
					probSum+=getProbability(nextState,stopState,nextStateProb*probHere,discovered);
				}
				probHere = probSum; 
			}
			else
				probHere = 0; //because we dont care if you can get to any other state
		}
		}
		return probHere;
	}
	
	public int[] getRobotStatesIndexFromJointState (State currState,List<State> states) {
		int[] toRet = new int[nRobots]; 
		for(int rNum = 0; rNum < nRobots; rNum++) {
	toRet[rNum] = getRobotStateIndexFromJointState(currState,rNum,states);
		}
	return toRet; 
	}
	public int getRobotStateIndexFromJointState (State currState, int rNum,List<State> states) {
		Object[] robotState = createSingleRobotState(currState, rNum); 
		int robotStateId =StatesHelper.getExactlyTheSameState(robotState,states); 
		return robotStateId; 
	}

	
}



