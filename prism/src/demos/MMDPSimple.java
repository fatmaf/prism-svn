package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import acceptance.AcceptanceReach;

import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Stack;
import java.util.Vector;

import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import prism.PrismException;
import prism.PrismLangException;
import strat.Strategy;
import explicit.Distribution;
import explicit.MDPSimple;

//import explicit.stateInfoFromStatesList;

/*
 * A class that stores the joint policy for multiple agents 
 */
public class MMDPSimple {

	public class StateProb implements Comparable<StateProb> {
		private double prob;
		private int state;

		public StateProb(int state, double prob) {
			this.prob = prob;

			this.state = state;
		}

		public StateProb(StateProb other) {
			this.prob = other.prob;
			this.state = other.state;
		}

		@Override
		public int compareTo(StateProb other) {
			double comp = this.prob - other.prob;
			int res = 0;
			if (comp > 0)
				res = -1;
			else {
				if (comp < 0)
					res = 1;
			}
			return res;
		}

		public StateProb copy() {
			return new StateProb(this);
		}

		public double getProb() {
			return this.prob;
		}

		public int getState() {
			return this.state;
		}

		@Override
		public String toString() {
			return "[s=" + state + ", p=" + prob + "]";
		}

	}

	public MDPSimple mdp;
	// ArrayList<Map.Entry<State, Integer>>
	HashMap<State, Integer> statesMap;
	HashMap<Integer, Double> probsVector;
	int nRobots;
	public PriorityQueue<StateProb> stuckStatesQ;
	public BitSet deadendStates;
	public BitSet allTasksCompletedStates;
	public BitSet allFailStatesSeen;
	BitSet initStates;
	ArrayList<String> sharedVarsList; 
	ArrayList<String> notSharedVarsList; //just to make life easier  
	int numDA;
	VarList teamMDPVarlist; 

	/**
	 * creates a joint policy MDP
	 * 
	 * @numrobots the number of robots
	 * 
	 */
	public MMDPSimple(int numrobots, int numDAs,ArrayList<String> sharedVarsList,VarList seqTeamMDPVarlist) {
		stuckStatesQ = new PriorityQueue<StateProb>();
		mdp = new MDPSimple();
		this.sharedVarsList = sharedVarsList;
		notSharedVarsList = new ArrayList<String>(); //this is for now 
		for (int i =0; i<seqTeamMDPVarlist.getNumVars(); i++)
		{
			String name = seqTeamMDPVarlist.getName(i); 
			if (!sharedVarsList.contains(name) && (!name.contains("da")) && (name != "r"))
				notSharedVarsList.add(name);
		}
		
		int numSharedVars = this.sharedVarsList.size(); 
		int numSeqTeamMDPVars = seqTeamMDPVarlist.getNumVars(); 
		this.teamMDPVarlist = seqTeamMDPVarlist; 
		
		VarList varlist = new VarList();
		
		
		try {
			for (int i = numrobots - 1; i >= 0; i--) {

				varlist.addVar(0, new Declaration("r" + i, new DeclarationIntUnbounded()), 1, null);

			}
			for(int i = 0; i<numSharedVars; i++)
			{
				varlist.addVar(0, new Declaration(sharedVarsList.get(i), new DeclarationIntUnbounded()), 1, null);
			}
			// varlist.addVar(0, new Declaration("da", new DeclarationIntUnbounded()), 1,
			// null);
			for (int i = 0; i < numDAs; i++) {
				varlist.addVar(0, new Declaration("da" + i, new DeclarationIntUnbounded()), 1, null);
			}
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
		setNumDA(numDAs);
		probsVector = new HashMap<Integer, Double>();

	}

	public class switchStateInfo{
		public int pstate; 
		public int crnum; 
		public int cstate; 
		public double prob; 
		public switchStateInfo(int ps,int r, int s, double p)
		{
			pstate = ps;
			crnum = r; 
			cstate = s; 
			prob = p; 
		}
		@Override
		public String toString() {
			return " [ps:" + pstate + ", r:" + crnum + ", s:" + cstate + ", " + prob + "]";
		}
		
	}
	public ArrayList<Integer> breakSeqPolicyAndAddtoPolicy(SequentialTeamMDP seqTeamMDP, Strategy strat, int initialState,
			int nextSuccState, int[] allRobotInitStates,boolean synchronizationpoints) {
		boolean debugStuff = true;
	
		//attempting to label all the paths so that I can break them down 
		HashMap<String,Vector<BitSet>> allPathStates= new HashMap<String,Vector<BitSet>>(); //each bitset represents a different path 
		HashMap<String,Double> allPathProbs = new HashMap<String,Double>();
		ArrayList<String> acceptingPaths = new ArrayList<String>();
		ArrayList<String> failingPaths = new ArrayList<String>();
		ArrayList<String> allOtherPaths = new ArrayList<String>();
		MDPSimple tempMDP = null;
		int[] tempMDPMap = null;
		BitSet tempAccStates = null;
		boolean actionFound = false;
		MDPSimple sumprod = seqTeamMDP.teamMDPWithSwitches;
		int numRobots = seqTeamMDP.agentMDPs.size();
		// n MDP stuff
		MDPSimple mdps[] = new MDPSimple[numRobots];
		int mdpMaps[][] = new int[numRobots][sumprod.getNumStates()];
		BitSet accStates[] = new BitSet[numRobots];
		//<state,child_state_robot,child_state>
		ArrayList<ArrayList<switchStateInfo>> robotLastStateInitStatePairs = 
				new ArrayList<ArrayList<switchStateInfo>>(); 
		double probsArr[] = new double[sumprod.getNumStates()];
		 Vector <BitSet>pathStates = new Vector<BitSet>(); 
		for (int i = 0; i < numRobots; i++) {
			robotLastStateInitStatePairs.add(new ArrayList<switchStateInfo>());
			pathStates.add(new BitSet());
			mdps[i] = new MDPSimple();
			mdps[i].setVarList((VarList) sumprod.getVarList().clone());
			Arrays.fill(mdpMaps[i], StatesHelper.BADVALUE);
			mdps[i].setStatesList(new ArrayList<State>());
			accStates[i] = new BitSet();
		}
		allPathStates.put("0",pathStates);
		allPathProbs.put("0", 1.0);
		if (debugStuff) {
			tempMDP = new MDPSimple();
			tempMDPMap = new int[sumprod.getNumStates()];
			Arrays.fill(tempMDPMap, StatesHelper.BADVALUE);
			tempAccStates = new BitSet();
			tempMDP.setVarList((VarList) sumprod.getVarList().clone());
			tempMDP.setStatesList(new ArrayList<State>());
		}
		// n MDP stuff done
		ArrayList<Integer> statesDiscovered = new ArrayList<Integer>();
		int statesForPolicy = 0;

		int choices = -1;

		Object action = null;

		// this shouldnt matter
		List<Integer> discoveredStates = new LinkedList<Integer>();
		// on second thought this does not matter (the thing below) because we might
		// visit the same state again but it may be a child of another state
		// and so the initial state of the robot would have changed (though maybe no new
		// task would be achieved)
		// BitSet addedToStuckQ = new BitSet(tAutomaton.numStates); //for each policy
		// unfolding we don't need to add the same state over and over again to the
		// stuck stack

		Stack<Integer> statesStack = new Stack<Integer>(); // to store all the states we need to explore, so dfs
															// to the goal , well its not dfs but yeah
		int currState = initialState;
		int firstRobot = StatesHelper.getRobotNumberFromState(sumprod.getStatesList().get(currState));
		
	   
	    Stack<String> pathStack = new Stack<String>();
		statesStack.push(currState);
		pathStack.push("0");
		probsArr[currState]=1.0;
		String pathnum = "0";
	
		// TODO: what do you do when the policy has been computed before and we're just
		// here again ?
		// Do we care about this here or later ?

//		String prevTextToAdd = "";
//		int timecount = 0;
		while (!statesStack.isEmpty()) // the main bit - go to the next state, get the action go to the next state and
										// so on
		{
			
			// TODO: addstatetopolmdp(currState.state)
			// TODO: distribution
			currState = statesStack.pop();
			pathnum = pathStack.pop();
			strat.initialise(currState);
			action = strat.getChoiceAction();
			actionFound = false;
		
			int currentRobot = StatesHelper.getRobotNumberFromState(sumprod.getStatesList().get(currState));
			
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
						
						
						accStates[currentRobot] = StatesHelper.addLinkInMDP(mdps[currentRobot], mdpMaps[currentRobot],
								sumprod.getStatesList(), states, probs, currState, action, "r" + currentRobot,
								accStates[currentRobot], seqTeamMDP.acceptingStates);
						
						allPathStates.get(pathnum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
						
						boolean remove_old_path = false;
						//adding probs 
						for(int ns = 0; ns<states.size(); ns++)
						{
							int nextState = states.get(ns); 
							double probsNext = probs.get(ns); 
							probsNext = probsNext * probsArr[currState]; 
							probsArr[nextState]=probsNext;
							String tempPathnum = pathnum; 
							if (states.size()>1)
							{	tempPathnum =pathnum+ns;
								remove_old_path = true;
							}
							pathStack.push(tempPathnum);
							
							if(!allPathStates.containsKey(tempPathnum))
						
							{
								Vector<BitSet> tempv = new Vector<BitSet>();
								for (int i = 0; i < numRobots; i++) {
									tempv.add((BitSet)allPathStates.get(pathnum).get(i).clone());
								}
								
								allPathStates.put(tempPathnum,tempv); 
							
							}
							
							double tempProb = allPathProbs.get(pathnum)*probs.get(ns);
								allPathProbs.put(tempPathnum,tempProb);
							
							
						}
						if(remove_old_path)
						{
							allPathStates.remove(pathnum);
							allPathProbs.remove(pathnum);
						}
						//need to fix this really 
						//there can be multiple initial states because we have multiple switch actions 
						//we also want to track the state with the higher probability - so this is a bit of a problem right now
						if (action.toString().contains("switch"))
						{
							//cycle through the states and get initial states 
							for (int nextState: states)
							{
								int rindex = StatesHelper.getRobotNumberFromState(sumprod.getStatesList().get(nextState)); 
								mdps[rindex].addInitialState(
								     StatesHelper.addStateMDP(mdps[rindex], mdpMaps[rindex],
										sumprod.getStatesList(), nextState));
								switchStateInfo ssi = new switchStateInfo(mdpMaps[currentRobot][currState],rindex,
										mdpMaps[rindex][nextState],probsArr[nextState]);
								robotLastStateInitStatePairs.get(currentRobot).add(ssi);
							}
						}
						if (mdps[currentRobot].getNumInitialStates() == 0)
							mdps[currentRobot].addInitialState(0);
						if (debugStuff) {
							tempAccStates = StatesHelper.addLinkInMDP(tempMDP, tempMDPMap, sumprod.getStatesList(),
									states, probs, currState, action, "", tempAccStates, seqTeamMDP.acceptingStates);
							
						}
						actionFound = true;
						// addLinkInPolicyMDP(states,probs, currState.state, action);
						break;
						// don't go over all choices - actions are unique
					}
				}
				if (!actionFound) {
					allPathStates.get(pathnum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
					if (StatesHelper.isFailState(sumprod.getStatesList().get(currState))) {
						//huh ? 
						failingPaths.add(pathnum);
					}
					else
					{
						allOtherPaths.add(pathnum);
					}
				}

			}
			else
			{
				allPathStates.get(pathnum).get(currentRobot).set(mdpMaps[currentRobot][currState]);
				if(seqTeamMDP.acceptingStates.get(currState))
				acceptingPaths.add(pathnum);
				else {
					if (StatesHelper.isFailState(sumprod.getStatesList().get(currState))) {
						//huh ? 
						failingPaths.add(pathnum);
					}
					else
					allOtherPaths.add(pathnum);
			}}

		}
		// once we're done add the current policy to the big policy
		// and also check if the policy has at least init values for all robots
		// if(allRobotInitStates != null)
		// checkCurrentPolicy(allRobotInitStates,rts);
		if (debugStuff)
			StatesHelper.saveMDP(tempMDP, tempAccStates, "", "tempStrat" + initialState, true);
		StatesHelper.checkMDPs(mdps, mdpMaps, sumprod.getStatesList(), allRobotInitStates, initialState, accStates,
				seqTeamMDP.acceptingStates);
		// constructJointPolicyFromMDPsAndAddToCurrentPolicy(mdps,mdpMaps,firstRobot,accStates,seqTeamMDP,indexOfRobotThatFailedLast);
//		constructJointPolicyFromMDPsAndAddToCurrentPolicy_new(mdps, mdpMaps, firstRobot, accStates, seqTeamMDP);
		
		//printing stuff 
		for(String key: allPathStates.keySet())
		{
			System.out.println(key+":"+ allPathStates.get(key)+" - "+allPathProbs.get(key));
		}
		for(int i = 0; i<robotLastStateInitStatePairs.size();i++)
		{
			for(int j = 0; j< robotLastStateInitStatePairs.get(i).size(); j++)
			{
				System.out.println(i+"->"+robotLastStateInitStatePairs.get(i).get(j));
			}
		}
		if(acceptingPaths.size()>0)
		System.out.println("Accepting Paths: "+acceptingPaths.toString());
		else
			System.out.println("Other Paths: "+allOtherPaths.toString());
		convertMDPsToJointPolicy(mdps, mdpMaps, firstRobot, accStates, seqTeamMDP,synchronizationpoints);
		statesDiscovered.add(statesForPolicy);
		
		
		
		return statesDiscovered;

	}

	public ArrayList<Integer> addSeqPolicyToJointPolicy(SequentialTeamMDP seqTeamMDP, Strategy strat,  int initialState,
			boolean noJointState,boolean syncpoints) throws PrismException {

		int nextRobotstate = StatesHelper.BADVALUE;

		int rNum = 0;
		int[] robotStates = null;
		int nextRobot = 0;
		List<State> states;
		State currState;// = states.get(initialState);
		if (!noJointState) {
			states = mdp.getStatesList();
			currState = states.get(initialState);
			states = seqTeamMDP.teamMDPTemplate.getStatesList();
			rNum = firstFailedRobot(currState);
			robotStates = getRobotStatesIndexFromJointState(currState, states);
			int robotStateId = robotStates[rNum];

		} else {
			states = seqTeamMDP.teamMDPWithSwitches.getStatesList();
			currState = states.get(initialState);
			// lets get some information from the model itself
			// if we dont have initial states
			rNum = StatesHelper
					.getRobotNumberFromState(seqTeamMDP.teamMDPWithSwitches.getStatesList().get(initialState));
			robotStates = new int[nRobots];
			for (int i = 0; i < nRobots; i++) {
				if (i == rNum) {
					// we dont want mdp states
					robotStates[i] = initialState;
				} else {
					// this is just in case there are no actions for the other robot
					// so we know where it is in the mdp
					int anyinitstate = seqTeamMDP.initialStates.get(i).nextSetBit(0);
					robotStates[i] = anyinitstate;

				}
			}

		}
		nextRobot = (rNum + 1) % nRobots;
		nextRobotstate = robotStates[nextRobot];

		//
		// seqTeamMDP.setInitialStates(robotStates);
		// seqTeamMDP.addSwitchesAndSetInitialState(rNum);
		initialState = seqTeamMDP.teamMDPWithSwitches.getFirstInitialState();
		State s1 = seqTeamMDP.teamMDPTemplate.getStatesList().get(initialState);
		System.out.println("Adding policy for "+ s1.toString());
		return breakSeqPolicyAndAddtoPolicy(seqTeamMDP, strat, initialState, nextRobotstate, robotStates,syncpoints);

	}

	private int addStateToMDP(State s) {
		int index = findStateIndex(s);
		if (index == StatesHelper.BADVALUE) {
			mdp.getStatesList().add(s);
			index = mdp.getNumStates();
			statesMap.put(s, index);
			mdp.addState();

		}
		// for the first state only
		// initialize the probs vector
		if (mdp.getNumStates() == 1) {
			probsVector.put(index, 1.0);
		}
		return index;
	}

	private void addTransitionToMDPResetTask(State parentState, State resetState, double prob) {
		ArrayList<State> states = new ArrayList<State>();
		states.add(resetState);
		ArrayList<Double> probs = new ArrayList<Double>();
		probs.add(prob);
		addTranstionToMDP(parentState, states, probs, "reset", 1.0);
	}

	private void addTranstionToMDP(State parentStates, ArrayList<State> states, ArrayList<Double> probs, String action,
			double norm) {
		int parentIndex = addStateToMDP(parentStates);
		int index;
		Distribution distr = new Distribution();

		for (int succ = 0; succ < states.size(); succ++) {
			index = addStateToMDP(states.get(succ));
			double normalizedProb = probs.get(succ) / norm;
			distr.add(index, normalizedProb);
			if (!probsVector.containsKey(index)) {
				// this can happen so we're kind of ignoring it
				// the scenario where two different paths end up at the same node
				// we're just going to stick to the previous probs
				double probHere = normalizedProb * probsVector.get(parentIndex);
				probsVector.put(index, probHere);
			}

		}
//		int actionNo = mdp.getNumChoices(parentIndex);
		mdp.addActionLabelledChoice(parentIndex, distr, action);

	}

//	private boolean allDone(int[] currentSteps, int[] maxSteps) {
//		boolean toret = true;
//		for (int i = 0; i < currentSteps.length; i++) {
//			if (currentSteps[i] < maxSteps[i] - 1) {
//				toret = false;
//				break;
//			}
//		}
//		return toret;
//	}

	public boolean checkAndAddJointFailStateToJointPolicy(int stateNumInMDP, State currentJointState,
			int numFailedRobots, int numFailedRobotsInInitState, SequentialTeamMDP seqTeamMDP,
			int[] robotDAassociations) {
		boolean isJointFailState = false;
		double stateProb = 0;
		// get number of failed robots
		numFailedRobots = hasFailState(currentJointState);


		// now check if its a state where robots have failed
		if (numFailedRobots > numFailedRobotsInInitState) {
			isJointFailState = true;
			// if everyone has failed, theres nothing to do
			if (numFailedRobots == nRobots) {
				allFailStatesSeen.set(stateNumInMDP);
				deadendStates.set(stateNumInMDP);
			} else {
				if (!allFailStatesSeen.get(stateNumInMDP)) {
					stateProb = getProbability(0, stateNumInMDP, 1.0, new BitSet());
					State newJointState = resetTasksInIntermediateStates(currentJointState, seqTeamMDP,
							robotDAassociations);
					// do we need to reset any tasks, if so we need a new failstate
					if (newJointState != null) {
						addTransitionToMDPResetTask(currentJointState, newJointState, 1.0);
						stateNumInMDP = statesMap.get(newJointState);

					}
					if (!allFailStatesSeen.get(stateNumInMDP)) {
						stuckStatesQ.add(new StateProb(stateNumInMDP, stateProb));
						allFailStatesSeen.set(stateNumInMDP);

					} else {
						deadendStates.set(stateNumInMDP);
					}
				}
				else {
					deadendStates.set(stateNumInMDP);
				}
			}

		}
		return isJointFailState;
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

	public boolean checkSynchronizationPointAddToQ(int stateNumInMDP, State currentJointState, int numFailedRobots,
			int numFailedRobotsInInitState, SequentialTeamMDP seqTeamMDP, int[] robotDAassociations) {
		boolean isSynchronizationPoint = checkAndAddJointFailStateToJointPolicy(stateNumInMDP, currentJointState,
				numFailedRobots, numFailedRobotsInInitState, seqTeamMDP, robotDAassociations);
		return isSynchronizationPoint;
	}


	public int convertDAIndToStateInd(int daNum, int numda) {
		return ((numda - 1) - daNum);
	}

	public void testMDPsToJointPolicy(MDPSimple[] pols, int[][] mdpMaps, SequentialTeamMDP seqTeamMDP) {
		Object DAInitStates[] = getDAStartStates(seqTeamMDP); // all das dont start at 0, so we need these if we want to
		// reset stuff
		BitSet DAGoalStates[] = getDAGoalStates(seqTeamMDP); // we need these to check if we've got accepting states
		int robotDAassociations[] = new int[numDA]; // to track which robot affects which da
		Arrays.fill(robotDAassociations, -1); // initializing

	}
	//dear joint policy, you are the joy in my life - thank you 
	//someday you wont be but thats how it is - thank you for helping drown out the noise 
	//stay safe bruv :P 
	//you and of course an absolutely distracting playlist - things to be grateful for 
	public void convertMDPsToJointPolicy(MDPSimple[] pols, int[][] mdpMaps, int firstRobot, BitSet[] acceptingStates,
			SequentialTeamMDP seqTeamMDP,boolean synchronizationpoints) {
		Object DAInitStates[] = getDAStartStates(seqTeamMDP); // all das dont start at 0, so we need these if we want to
																// reset stuff
		BitSet DAGoalStates[] = getDAGoalStates(seqTeamMDP); // we need these to check if we've got accepting states
		int robotDAassociations[] = new int[numDA]; // to track which robot affects which da
		Arrays.fill(robotDAassociations, -1); // initializing

		int initialStates[] = new int[nRobots];
		int lastStates[] = new int[nRobots];

		int nextStates[] = new int[nRobots];
		int currentStates[] = new int[nRobots];

		Queue<State> statesToExploreQ = new LinkedList<State>();
		Queue<int[]> correspondingRobotMDPStatesQ = new LinkedList<int[]>();

		BitSet discoveredStates = new BitSet();

		ArrayList<Double> nextStateProbs = new ArrayList<Double>();
		ArrayList<State> nextJointStates = new ArrayList<State>();
		ArrayList<int[]> combinations = new ArrayList<int[]>();
		double sumProb = 0;

		int numFailedRobots = 0;
		int numFailedRobotsInInitState = StatesHelper.BADVALUE;

		double stateProb;
		int stateNumInMDP;
		boolean isAcceptingJointState;

		int[] numSuccessorStates = new int[nRobots];
		ArrayList<ArrayList<Entry<Integer, Double>>> successorStates = new ArrayList<ArrayList<Entry<Integer, Double>>>(
				nRobots);

		String jointAction = "";
		int numChoices = 0;
		Object action = null;
		for (int r = 0; r < nRobots; r++) {

			StatesHelper.saveMDP(pols[r], null, "", "stratmdp"+r, true);}
		// update initial states to reflect DA progress
		updateInitialStates(pols, mdpMaps, firstRobot, acceptingStates, seqTeamMDP.teamMDPWithSwitches.getStatesList(),
				seqTeamMDP.acceptingStates);
		for (int r = 0; r < nRobots; r++) {

			StatesHelper.saveMDP(pols[r], null, "", "stratmdpfixed"+r, true);}
		// get all the initial states and last states
		for (int r = 0; r < nRobots; r++) {

			//StatesHelper.saveMDP(pols[r], null, "", "startmdp"+r, true);
			initialStates[r] = pols[r].getFirstInitialState();
			lastStates[r] = getLastState(pols[r], acceptingStates[r]);
			
		}

		Object[] teamProgress = getTeamDAProgress(pols, firstRobot, initialStates, lastStates, DAInitStates,
				robotDAassociations);
		Object[] sharedStates =  getTeamSharedStatesProgress(pols,firstRobot,initialStates,lastStates,initialStates);
		//use this to check if we need to trigger a synchronization point thing 
		//but waaa
//		System.out.println(sharedStates.toString());
		currentStates = initialStates.clone();
		State currentJointState = createJointState(teamProgress,sharedStates,currentStates,pols);
		//createJointState(teamProgress, currentStates, pols);
		statesToExploreQ.add(currentJointState);
		correspondingRobotMDPStatesQ.add(currentStates.clone());

		numFailedRobots = hasFailState(currentJointState);
		if (numFailedRobotsInInitState == StatesHelper.BADVALUE) {
			// set it for the initial state
			// we'll use this to determine new fail states
			numFailedRobotsInInitState = numFailedRobots;
		}

		
		while (!statesToExploreQ.isEmpty()) {
			// remove states from Q
			currentJointState = statesToExploreQ.remove();
			currentStates = correspondingRobotMDPStatesQ.remove();

			// find state in joint policy mdp
			stateNumInMDP = addStateToMDP(currentJointState);

			// dont explore if already done
			if (discoveredStates.get(stateNumInMDP)) {
				continue;
			}
			// if not go on :P
			discoveredStates.set(stateNumInMDP);

			// now check if its an accepting state
			// if so we don't need to explore this state
			isAcceptingJointState = isAcceptingState(currentJointState, DAGoalStates);
			if (isAcceptingJointState) {
				allTasksCompletedStates.set(stateNumInMDP);
				continue;
			}
			
			if(synchronizationpoints) {
			// check if its a fail state
			if (checkSynchronizationPointAddToQ(stateNumInMDP, currentJointState, numFailedRobots,
					numFailedRobotsInInitState, seqTeamMDP, robotDAassociations)) {
				continue;
			}
			}

			Arrays.fill(numSuccessorStates, 0);
			successorStates.clear();

			jointAction = "";
			for (int r = 0; r < nRobots; r++) {
				numChoices = pols[r].getNumChoices(currentStates[r]);
				if (numChoices > 0) {
					action = pols[r].getAction(currentStates[r], numChoices - 1);
					if (action != null) {
						if (action.toString().contains("switch")) {
							numChoices = 0;
						}
					}
					if (numChoices == 1) {
						numSuccessorStates[r] = pols[r].getNumTransitions(currentStates[r], numChoices - 1);
						jointAction += pols[r].getAction(currentStates[r], numChoices - 1);
						ArrayList<Entry<Integer, Double>> succStates = getSuccessorStatesFromMDP(pols[r],
								currentStates[r], numChoices - 1);
						successorStates.add(succStates);

					}
					else {
						if(numChoices > 1)
							System.out.println(
									"More than one choice for state? We talked about ignoring this. see notes");
					}}
					
						if (numChoices == 0) {
							numSuccessorStates[r] = 1;
							ArrayList<Entry<Integer, Double>> succStates = new ArrayList<Entry<Integer, Double>>();
							succStates.add(new AbstractMap.SimpleEntry<Integer,Double>(currentStates[r], 1.0));
							successorStates.add(succStates);
						} 
						
				
			} //done going over the policy for each robot 
			
	
			 nextStateProbs.clear(); //new ArrayList<Double>();
			nextJointStates.clear(); // new ArrayList<State>();
			combinations.clear();
			sumProb = 0;
			generateCombinations(numSuccessorStates.clone(), numSuccessorStates.clone(), combinations);
			for (int[] combination : combinations) {
				stateProb = 1.0;
				for (int r = 0; r < nRobots; r++) {
					nextStates[r] = successorStates.get(r).get(combination[r] - 1).getKey();
					stateProb =stateProb* successorStates.get(r).get(combination[r] - 1).getValue();
				

				}

				teamProgress = getTeamDAProgress(pols, firstRobot, nextStates, lastStates, DAInitStates, robotDAassociations);
				
				//FIXME: this has to do something you know :P 
				sharedStates =  getTeamSharedStatesProgress(pols,firstRobot,nextStates,lastStates,initialStates); 
				
				State nextJointState = createJointState(teamProgress,sharedStates,nextStates,pols);
				//createJointState(teamProgress, nextStates, pols);
				nextStateProbs.add(stateProb);
				sumProb +=stateProb;
				nextJointStates.add(nextJointState);
				statesToExploreQ.add(nextJointState);
				correspondingRobotMDPStatesQ.add(nextStates.clone());
		
			}
			addTranstionToMDP(currentJointState, nextJointStates, nextStateProbs, jointAction, sumProb);

		}
		saveJointPolicy();
	}

	public State createJointState(Object[] DFAProgress, int[] currentRobotStates, MDPSimple[] pols) {
		State currentJointState = new State(DFAProgress.length + currentRobotStates.length);
		int offset = 0;
		for (int i = 0; i < DFAProgress.length; i++) {
			currentJointState.setValue(i + offset, DFAProgress[i]);
		}
		offset = DFAProgress.length;
		for (int i = 0; i < currentRobotStates.length; i++) {
			int temp = StatesHelper
					.getMDPStateFromState(pols[i].getStatesList().get(currentRobotStates[i]));
			currentJointState.setValue(i + offset, temp);
		}
		return currentJointState;
	}
	
	public State createJointState(Object[] DFAProgress, Object[] sharedState,int[] currentRobotStates, MDPSimple[] pols) {
		if (sharedState != null) {
		State currentJointState = new State(DFAProgress.length +sharedState.length+currentRobotStates.length);
		int offset = 0;
		for (int i = 0; i < DFAProgress.length; i++) {
			currentJointState.setValue(i + offset, DFAProgress[i]);
		}
		offset = DFAProgress.length;
		for(int i = 0; i<sharedState.length; i++) {
			currentJointState.setValue(i+offset, sharedState[i]);
		}
		offset +=sharedState.length;
		for (int i = 0; i < currentRobotStates.length; i++) {
			Object[] temp = StatesHelper
					.getMDPStateFromState(pols[i].getStatesList().get(currentRobotStates[i]),this.teamMDPVarlist, this.notSharedVarsList);
			for(int j = 0; j<temp.length; j++)
			{ currentJointState.setValue( offset,temp[j]);
			
			}
			offset = offset + (temp.length);
		}
		return currentJointState;
		}
		else
		{
			State currentJointState = new State(DFAProgress.length +currentRobotStates.length);
			int offset = 0;
			for (int i = 0; i < DFAProgress.length; i++) {
				currentJointState.setValue(i + offset, DFAProgress[i]);
			}
			offset = DFAProgress.length;
			for (int i = 0; i < currentRobotStates.length; i++) {
				Object[] temp = StatesHelper
						.getMDPStateFromState(pols[i].getStatesList().get(currentRobotStates[i]),this.teamMDPVarlist, this.notSharedVarsList);
				for(int j = 0; j<temp.length; j++)
				{ currentJointState.setValue( offset,temp[j]);
				
				}
				offset = offset + (temp.length);
			}
			return currentJointState;
		}
	}
	public Object[] createSingleRobotState(State jointState, int rNum) {
		// pick the robot num
		Object[] mdpStates = jointState.varValues;
//		int dfaStart = 0;
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
	public Object[] createSingleRobotStateIncludeSharedState(State jointState, int rNum) {
		// pick the robot num
		Object[] mdpStates = jointState.varValues;
		int sharedStateStart = this.numDA;//this.numDA;
//		int dfaStart = 0;
		int dfaEnd = this.numDA;//mdpStates.length - nRobots;
		int robotIndex = dfaEnd+sharedVarsList.size()+ rNum;
		Object[] singleRobotState = new Object[dfaEnd + 2+this.sharedVarsList.size()];
		singleRobotState[0] = rNum;
		for (int i = 1; i < dfaEnd + 1; i++) {
			singleRobotState[i] = mdpStates[i - 1];
		}
		//match everything else 
		
		for(int i = 0; i< this.sharedVarsList.size(); i++)
		{	//singleRobotState[i+(dfaEnd+1)]=mdpStates[dfaEnd+i]; 
			int index = this.teamMDPVarlist.getIndex(sharedVarsList.get(i));
			int otherIndex = this.mdp.getVarList().getIndex(sharedVarsList.get(i));
			if (index != -1)
			{
				singleRobotState[index] = mdpStates[otherIndex];
			}
		}
		for(int i = 0; i< this.notSharedVarsList.size(); i++)
		{	//singleRobotState[i+(dfaEnd+1)]=mdpStates[dfaEnd+i]; 
			int index = this.teamMDPVarlist.getIndex(notSharedVarsList.get(i));
//			int otherIndex = this.mdp.getVarList().getIndex(notSharedVarsList.get(i));
			if (index != -1)
			{
				singleRobotState[index] = mdpStates[robotIndex];
			}
		}
		
		//singleRobotState[dfaEnd +this.sharedVarsList.size()+ 1] = mdpStates[robotIndex];
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


	public int firstFailedRobot(Object[] mdpStates, int start, int end) {
		int failedRobot = StatesHelper.BADVALUE; // no one
		for (int i = start; i < end; i++) {
			if ((int) mdpStates[i] == StatesHelper.failState) {
				failedRobot = i - start;
				break;
			}
		}
		return failedRobot;
	}

	public int firstFailedRobot(State jointState) {   //TODO fix this for when the number of states for each robot vary/are more than 1
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

	public BitSet getDAaccStatesForRobot(int da_num, int r, SequentialTeamMDP seqTeamMDP) {
		if (!seqTeamMDP.agentMDPs.get(r).daList.get(da_num).isSafeExpr)
			return ((AcceptanceReach) seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getAcceptance())
					.getGoalStates();
		else {
			// TODO : maybe there are multiple initial states or something and so we need to
			// do something like not the final state in our check
			BitSet temp = new BitSet();
			temp.set(seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getStartState());
			return temp;
		}
	}

	public BitSet[] getDAGoalStates(SequentialTeamMDP seqTeamMDP) {
		BitSet[] daStartStates = new BitSet[numDA];
		for (int i = 0; i < numDA; i++) {
			daStartStates[i] = getDAaccStatesForRobot(convertDAIndToStateInd(i, numDA), 0, seqTeamMDP);
		}
		return daStartStates;
	}

	public int getDAStartStateForRobot(int da_num, int r, SequentialTeamMDP seqTeamMDP) {
		return seqTeamMDP.agentMDPs.get(r).daList.get(da_num).da.getStartState();
	}

	public Object[] getDAStartStates(SequentialTeamMDP seqTeamMDP) {
		Object[] daStartStates = new Object[numDA];
		for (int i = 0; i < numDA; i++) {
			daStartStates[i] = getDAStartStateForRobot(convertDAIndToStateInd(i, numDA), 0, seqTeamMDP);
		}
		return daStartStates;
	}



	// TODO: this can be more efficient
	public int getLastState(MDPSimple pol, BitSet acceptingStates) {
		int state = getLastState(pol, acceptingStates, 0);
		for (int kindOfState = 1; (state == StatesHelper.BADVALUE) && (kindOfState < 3); kindOfState++) {
			state = getLastState(pol, acceptingStates, kindOfState);
		}
		// StatesHelper.saveMDP(pol, null, "", "polx", true);
		if (state == StatesHelper.BADVALUE) {
			// if there is no accepting state,
			// no switch state
			// no fail state
			// then we can just do a dfs and get the last last state
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
		int lastState = StatesHelper.BADVALUE;
		LinkedList<Integer> statesQ = new LinkedList<Integer>();
		BitSet discovered = new BitSet();

		statesQ.add(state);
		while (!statesQ.isEmpty()) {
			state = statesQ.remove();
			if (!discovered.get(state)) {
				discovered.set(state);
				if (kindOfState == 0) {
					if (acceptingStates.get(state))
						lastState = state;
				}
				if (kindOfState == 2) {
					if (StatesHelper.isFailState(pol.getStatesList().get(state))) //TODO: fix this here , because this has changed
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
		if (kindOfState == 3) {
			// just get the last state in the dfs
			lastState = state;
		}

		return lastState; // so basically this is the last state we tried to explore
	}



	public int getNumDA() {
		return numDA;
	}

	public double getProbability(int startState, int stopState, double probHere, BitSet discovered) {
		int choice = 0;
		double probSum = 0;

		if (discovered.get(stopState)) { //once we've found the state , we dont care so everything returns 0
			probHere = 0; 
			return probHere;
		}
		if (startState != stopState) {
			if (!discovered.get(startState)) {
		
				int numChoice = mdp.getNumChoices(startState);
				discovered.set(startState);
				if (numChoice == 1) {
//					Object action = mdp.getAction(startState, choice);
					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(startState, choice);
					while (tranIter.hasNext()) {
						Entry<Integer, Double> stateProbPair = tranIter.next();
						int nextState = stateProbPair.getKey();
						double nextStateProb = stateProbPair.getValue();
						// stop at the first failstate

						probSum += getProbability(nextState, stopState, nextStateProb * probHere, discovered);
					}
					probHere = probSum;
				} else
					probHere = 0; // because we dont care if you can get to any other state
		
		}
		}
		else 
		{
			discovered.set(startState);
		}
		
		return probHere;
	}
	public double getProbabilityAllPossibleAcceptingStates(int startState, double probHere,BitSet discoveredStates,HashMap<Integer,Double> probMap)
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
					//don't explore if the child state is the same as the parent state 
					if(nextState != startState)
					probSum+=getProbabilityAllPossibleAcceptingStates(nextState,nextStateProb*probHere,discoveredStates,probMap);
					else
					{
						System.out.println("State "+nextState+" in joint policy leads to itself, something is wrong here?");
					}
				}
				probHere = probSum; 
			}
			else
				probHere = 0; //no chance of getting to an accepting state
		}
			else
			{
				double tprob = probMap.get(startState);
				if (tprob!=0)
				{
					discoveredStates.set(startState,false);
					probHere=getProbabilityAllPossibleAcceptingStates(startState,probHere,discoveredStates,probMap);
				}
				else
					probHere = tprob;
			}
//			else
//			{
//				//its a self loop so we will just stop here 
//				probHere = 0; 
//			}
		//	probHere = probSum;
		}
		if(!probMap.containsKey(startState))
			probMap.put(startState, probHere);

		return probHere;
	}

	public double getProbabilityAnyAcceptingState(int startState, double probHere, BitSet discoveredStates) {
		int choice = 0;
		double probSum = 0;
		
		BitSet temp = (BitSet)discoveredStates.clone(); 
		temp.and(allTasksCompletedStates);
		if(temp.cardinality() > 0) //if we've reached any accepting state, just end 
		{
			probHere = 0; 
			return probHere; 
		}
		if (!allTasksCompletedStates.get(startState)) {

			int numChoice = mdp.getNumChoices(startState);
			if (!discoveredStates.get(startState)) {
				discoveredStates.set(startState);
				if (numChoice == 1) {
//					Object action = mdp.getAction(startState, choice);
					Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(startState, choice);
					while (tranIter.hasNext()) {
						Entry<Integer, Double> stateProbPair = tranIter.next();
						int nextState = stateProbPair.getKey();
						double nextStateProb = stateProbPair.getValue();
						probSum += getProbabilityAnyAcceptingState(nextState, nextStateProb * probHere,
								discoveredStates);
					}
					probHere = probSum;
				} else
					probHere = 0; // no chance of getting to an accepting state
			}
			// else
			// {
			// //its a self loop so we will just stop here
			// probHere = 0;
			// }
		}
		else
		{
			discoveredStates.set(startState);
		}
		return probHere;
	}

	public Object[] getRobotDAProgress(State previousRobotsState, State currentRobotsState, Object[] daInitStates,
			int[] robotDAassoc, int rnum) {

		Object[] previousRobotsDAState = StatesHelper.getDAStatesFromState(previousRobotsState);
		Object[] currentRobotsDAState = StatesHelper.getDAStatesFromState(currentRobotsState);
		int[] integerXORmask = StatesHelper.XORIntegers(previousRobotsDAState, currentRobotsDAState);
		for (int i = 0; i < integerXORmask.length; i++) {
			if (integerXORmask[i] == 1)
				robotDAassoc[i] = rnum; // should just be 0 or 1 and each task can be assinged to a single robot 
		}
		Object[] updatedState = StatesHelper.multiplyWithMask(integerXORmask, currentRobotsDAState, daInitStates);

		return updatedState;

	}
	private Object[] getRobotSharedStateProgress(State previousRobotsState, State currentRobotsState, State initialState) {
		
		Object[] previousRobotsSharedState = StatesHelper.getSharedStatesFromState(previousRobotsState, teamMDPVarlist, sharedVarsList); 
		Object[] currentRobotsSharedState = StatesHelper.getSharedStatesFromState(currentRobotsState, teamMDPVarlist, sharedVarsList); 
		Object[] initialSharedState = StatesHelper.getSharedStatesFromState(initialState, teamMDPVarlist, sharedVarsList); 

		int[] integerXORmask = StatesHelper.XORIntegers(previousRobotsSharedState, currentRobotsSharedState);
		Object[] updatedState = StatesHelper.multiplyWithMask(integerXORmask, currentRobotsSharedState, initialSharedState); 
		
		
		// TODO Auto-generated method stub
		return updatedState;
	}

	public int getRobotState(State jointState, int rnum) {

		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
//		int end = mdpStates.length;

		return (int) mdpStates[rnum + start];
	}

	public int getRobotStateIndexFromJointState(State currState, int rNum, List<State> states) {
		Object[] robotState = /*createSingleRobotState*/createSingleRobotStateIncludeSharedState(currState, rNum);
		int robotStateId = StatesHelper.getExactlyTheSameState(robotState, states);
		if (robotStateId == StatesHelper.BADVALUE)
			System.out
					.println("Bad value for " + rNum + ":" + currState.toString() + " " + Arrays.toString(robotState));
		return robotStateId;
	}

	public int[] getRobotStates(State jointState) {//FIXME: this is not okay!!! 

		Object[] mdpStates = jointState.varValues;
		int start = mdpStates.length - nRobots;
		int end = mdpStates.length;
		int[] robotStates = new int[nRobots];
		for (int i = start; i < end; i++) {
			robotStates[i - start] = (int) mdpStates[i];
		}
		return robotStates;
	}

	public int[] getRobotStatesIndexFromJointState(State currState, List<State> states) {
		int[] toRet = new int[nRobots];
		for (int rNum = 0; rNum < nRobots; rNum++) {
			toRet[rNum] = getRobotStateIndexFromJointState(currState, rNum, states);
		}
		return toRet;
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


	public Object[] getTeamDAProgress(int firstRobot, State[] lastStates, State[] currentStates, Object[] daInitStates,
			int robotDAassoc[]) {
		// DA States
		// start from the first robot
//		ArrayList<Object[]> robotsProgress = new ArrayList<Object[]>();

		Object[] teamProgress = null;
		int currentRobot = firstRobot;
		int prevRobot = firstRobot;
		do {

			Object[] updatedState = getRobotDAProgress(lastStates[prevRobot], currentStates[currentRobot], daInitStates,
					robotDAassoc, currentRobot);
			if (currentRobot == firstRobot)
				teamProgress = StatesHelper.ORIntegers(updatedState,
						StatesHelper.getDAStatesFromState(currentStates[firstRobot]), daInitStates);
//			robotsProgress.add(updatedState);
			prevRobot = currentRobot;
			currentRobot = (currentRobot + 1) % this.nRobots;
			teamProgress = StatesHelper.ORIntegers(teamProgress, updatedState, daInitStates);
		} while (currentRobot != firstRobot);

		return teamProgress;
	}

	public Object[] getTeamDAProgress(MDPSimple[] pols, int firstRobot, int[] currentStates, int[] lastStates,
			Object[] initStates, int robotDAassoc[]) {

		State[] lastStatesStates = new State[currentStates.length];
		State[] currentStatesStates = new State[currentStates.length];

		for (int i = 0; i < nRobots; i++) {
			lastStatesStates[i] = pols[i].getStatesList().get(lastStates[i]);
			currentStatesStates[i] = pols[i].getStatesList().get(currentStates[i]);

		}
		Object[] teamProgress = getTeamDAProgress(firstRobot, lastStatesStates, currentStatesStates, initStates,
				robotDAassoc);
		return teamProgress;
	}

	public Object[] getTeamSharedStatesProgress(MDPSimple pols[], int firstRobot, int[] currentStates, int[] lastStates, int[] initStates)
	{
		State[] lastStatesStates = new State[lastStates.length]; 
		State[] currentStatesStates = new State[currentStates.length]; 
		State[] initStatesStates = new State[initStates.length];
		for (int i = 0; i < nRobots; i++) {
			lastStatesStates[i] = pols[i].getStatesList().get(lastStates[i]);
			currentStatesStates[i] = pols[i].getStatesList().get(currentStates[i]);
			initStatesStates[i]=pols[i].getStatesList().get(initStates[i]);

		}
		Object[] sharedProgress = getTeamSharedStatesProgress(firstRobot,lastStatesStates,currentStatesStates,initStatesStates);
		return sharedProgress; 
	}

	private Object[] getTeamSharedStatesProgress(int firstRobot, State[] lastStatesStates, State[] currentStatesStates,
			State[] initStatesStates) {
		
		Object[] teamProgress = null; 
		int currentRobot = firstRobot; 
		int prevRobot = firstRobot; 
		Object[] initState = StatesHelper.getSharedStatesFromState(initStatesStates[firstRobot],
				this.teamMDPVarlist, this.sharedVarsList);
		if (initState != null) {
		do {
			Object[] updatedState = getRobotSharedStateProgress(lastStatesStates[prevRobot],
					currentStatesStates[currentRobot],initStatesStates[firstRobot]);
			if (currentRobot == firstRobot)
				teamProgress = StatesHelper.ORIntegers(updatedState,initState,initState);
			prevRobot = currentRobot;
			currentRobot = (currentRobot + 1) % this.nRobots;
			teamProgress = StatesHelper.ORIntegers(teamProgress, updatedState, initState);
		}while(currentRobot != firstRobot);
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

	public int hasFailState(Object[] mdpStates, int start, int end) { //TODO: fix this for multiple states
		int sum = 0;
		for (int i = start; i < end; i++) {
			if ((int) mdpStates[i] == StatesHelper.failState)
				sum++;
		}
		return sum;
	}

	public int hasFailState(State jointState) {
		Object[] mdpStates = jointState.varValues;
		int start = numDA+this.sharedVarsList.size();//mdpStates.length - nRobots;
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

//	private void incrementSteps(int[] currentSteps, int[] maxSteps) {
//		int inc = 1;
//		for (int i = 0; i < currentSteps.length; i++) {
//			if (currentSteps[i] + inc < maxSteps[i]) {
//				currentSteps[i] += inc;
//			}
//		}
//	}

	public boolean isAcceptingState(Object[] varvalues, int start, int end, BitSet[] acceptingStates) {
		boolean acceptingState = true;
		if (end <= start) {
			acceptingState = false;
		}
		for (int i = start; i < end; i++) {
			if (!acceptingStates[i].get((int) varvalues[i])) {
				acceptingState = false;
				break;
			}
		}
		return acceptingState;
	}

	public boolean isAcceptingState(State jointState, BitSet[] acceptingStates) {
		int daStart = 0;
		int daEnd = numDA;//jointState.varValues.length - nRobots;
		return isAcceptingState(jointState.varValues, daStart, daEnd, acceptingStates);
	}



	private State resetTasksInIntermediateStates(State jointState, SequentialTeamMDP seqTeamMDP, int[] robotDAassoc) {

		// for each DA element
		Object[] startStatesArr = getDAStartStates(seqTeamMDP);
		BitSet[] accStatesArr = getDAGoalStates(seqTeamMDP);
//		boolean changed = false;
		State newJointState = null;
		for (int i = 0; i < numDA; i++) {
			int sval = StatesHelper.getIndexValueFromState(jointState, i);
			if (sval != (int) startStatesArr[i] && !accStatesArr[i].get(sval)) {
				if (robotDAassoc[i] != -1) {
					if (getRobotState(jointState, robotDAassoc[i]) == StatesHelper.failState) {
						System.out.println("Resetting State for DA " + i + " robot " + robotDAassoc[i]);
						if (newJointState == null)
							newJointState = new State(jointState);
						// reset it
						newJointState.setValue(i, startStatesArr[i]);
//						changed = true;
					}

				}
			}
		}

		return newJointState;

	}

	public void saveJointPolicy() {
		// String saveplace =
		// "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
		//
		// PrismLog log = new PrismFileLog(saveplace + "jointPolicy.dot");
		// mdp.exportToDotFile(log, null, true);
		// log.close();
		StatesHelper.saveMDP(mdp, null, "", "jointPolicy", true);
		StatesHelper.saveMDPstatra(mdp, "", "jointPolicy", true);
		StatesHelper.saveHashMap(probsVector, "", "jointPolicyProbs.lab", true);
		StatesHelper.saveBitSet(allTasksCompletedStates, "", "jointPolicy.acc", true);
		StatesHelper.saveBitSet(this.allFailStatesSeen, "", "jointPolicy.failstates", true);
		StatesHelper.saveBitSet(this.deadendStates, "", "jointPolicy.deadends", true);

	}

	public void setNumDA(int n) {
		numDA = n;
	}



	public void testQ() {
		PriorityQueue<StateProb> testQ = new PriorityQueue<StateProb>();
		double qvals[] = { 0.5, 0.1, 0.2, 1.4, 0.02 };
		for (int i = 0; i < qvals.length; i++) {
			testQ.add(new StateProb(i, qvals[i]));
			System.out.println("Added " + testQ.peek().toString());
		}
		while (!testQ.isEmpty()) {
			StateProb meh = testQ.remove();
			System.out.println("Removed " + meh.toString());
		}
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

	public void updateInitialStates(MDPSimple[] pols, int[][] mdpMaps, int firstRobot, BitSet[] acceptingStates,
			List<State> teamMDPStates, BitSet teamAccStates) {

		// note to self - being lazy pays no one
		// do it right the first time!

		// basically sometimes a robot might not be used at all
		// so what we want to be able to do is like update it's possible state
		// how can we figure this out - if we don't have a policy for this robot
		// that means it has just one state and no action
		// boolean doneOnce = false;
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
				// if (!doneOnce)
				// doneOnce = true;
				// else {
				// System.out.println("Something Unexpected is happening");
				// }
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
						pols[i].getStatesList().get(currentRobotState), teamMDPStates);
				StatesHelper.addStateMDP(pols[i], mdpMaps[i], teamMDPStates, StatesHelper.BADVALUE, newState,
						acceptingStates[i], teamAccStates);
				pols[i].clearInitialStates();
				pols[i].addInitialState(mdpMaps[i][newState]);

			}
		}
	}

}
