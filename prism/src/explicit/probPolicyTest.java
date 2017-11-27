package explicit;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Stack;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import common.IterableBitSet;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MCRewards;
import explicit.rewards.MCRewardsFromMDPRewards;
import explicit.rewards.MDPRewards;
import explicit.rewards.MDPRewardsSimple;
import explicit.rewards.Rewards;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.DeclarationIntUnbounded;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.RewardStruct;
import parser.type.TypeInt;
import prism.Prism;
import prism.PrismComponent;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.PrismUtils;
import strat.MDStrategy;
import strat.MDStrategyArray;
import strat.Strategy;

//class to store robot state  
class policyState
{
	public int rnum;
	public int time;
	public int state;
	public int pstate;
	public Object action;
	public int associatedStartState; 
	public double probFromParent; 

	public policyState()
	{
		rnum = -1;
		time = -1;
		state = -1;
		pstate = -1;
		associatedStartState = -1;
		action = null;
		probFromParent = 0; //not possible 
		
	}

	/**
	 * @param r robot 
	 * @param t time 
	 * @param s state
	 */
	public policyState(int r, int t, int s)
	{
		rnum = r;
		time = t;
		state = s;
		pstate = -1;
		associatedStartState=-1;
		action = null;
		probFromParent = 0; 
	}

	/**
	 * @param r robot 
	 * @param t time 
	 * @param s state
	 * @param p parent state
	 */
	public policyState(int r, int t, int s, int p)
	{
		rnum = r;
		time = t;
		state = s;
		pstate = p;
		associatedStartState = -1;
		action = null;
		probFromParent = 0; 
	}

		/**
	 * @param r robot 
	 * @param t time 
	 * @param s state
	 * @param p parent state
	 * @param a action 
	 */
	public policyState(int r, int t, int s, int p, Object a, int as)
	{
		rnum = r;
		time = t;
		state = s;
		pstate = p;
		action = a;
		associatedStartState = as; 
		probFromParent = 0; 
	}

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString()
	{
		String toret ="[r:" + rnum + ", t:" + time + ", s:" + state + ", ps:" + pstate; 
		if (action != null)
			toret = toret+", a:" + action ;
		if (associatedStartState != -1)
			toret = toret + ", ss:"+associatedStartState; 
		toret = toret + "]";
		return toret; 
	}

	public void setAction(Object action2)
	{
		// TODO Auto-generated method stub
		action = action2;

	}
	public void setProb(double p)
	{
		probFromParent = p; 
	}
	public double getProb()
	{
		return probFromParent;
	}

};


class robotStateCombination{
	int state; 
	int combinationStates[]; 
	public robotStateCombination(int numrobots)
	{
		state = -1;
		combinationStates = new int[numrobots]; 
	}
	public robotStateCombination(int state_, int numrobots)
	{
		state = state_;
		combinationStates = new int[numrobots]; 
	}
	public void setRobotState(int stateRobot, int numrobot)
	{
		combinationStates[numrobot] = stateRobot;
	}
}


//a class I can use to manipulate the states list because otherwise it gets so messy 
class stateInfoFromStatesList
{
	List<State> states;
	int numVar;
	int robotState;
	int mdpState;
	int failState=9; 

	public stateInfoFromStatesList(MDPSimple sumprod)
	{
		List<State> states_ = sumprod.statesList;
		int numVar_ = sumprod.varList.getNumVars();
		states = states_;
		numVar = numVar_;
		robotState = 0;
		mdpState = numVar - 1;
	}
	private Object[] getNewState(State s1,List<Entry<Integer, Integer>> changedIndices) {
		Object x1v[] = s1.varValues;
		Object x1vp[] = x1v.clone();
		for (int v = 0; v < changedIndices.size(); v++) {
			int ind = changedIndices.get(v).getKey();
			int val = changedIndices.get(v).getValue();
			x1vp[ind] = val;
		}
		return x1vp;

	}
	
	public int[] getStatesFromCombinations(ArrayList<ArrayList<Entry<Integer, Integer>>> combinations, int cs) {
		int numcomb = combinations.size();
		int newstates[] = new int[numcomb];
		for (int comb = 0; comb < numcomb; comb++) {
			Object[] newstate = getNewState(states.get(cs), combinations.get(comb));
			newstates[comb] = getExactlyTheSameState(newstate);
		}
		return newstates;
	}

	public BitSet markIndicesInBitSetFromArrayList(ArrayList<Entry<Integer, Integer>> changedIndicesAndValues)
	{
		BitSet res = new BitSet(numVar);
		if(changedIndicesAndValues != null) {
		for (int i = 0; i < changedIndicesAndValues.size(); i++) {
			res.set(changedIndicesAndValues.get(i).getKey());
		}
		}
		return res;
	}

	public ArrayList<Entry<Integer, Integer>> compareDAStatesOnly(int s1, int s2, boolean returnValuesForS2)
	{
		ArrayList<ArrayList<Entry<Integer, Integer>>> res = compareDAStatesOnly(states.get(s1), states.get(s2));
		if (res != null) {
			if (returnValuesForS2)
				return (res.get(1));
			else
				return (res.get(0));
		} else
			return null;
	}

	public ArrayList<ArrayList<Entry<Integer, Integer>>> compareDAStatesOnly(int s1, int s2)
	{
		return compareDAStatesOnly(states.get(s1), states.get(s2));
	}

	private ArrayList<ArrayList<Entry<Integer, Integer>>> compareDAStatesOnly(State s1, State s2)
	{
		return compareDAStatesOnly(s1.varValues, s2.varValues);

	}

	private ArrayList<ArrayList<Entry<Integer, Integer>>> compareDAStatesOnly(Object[] s1, Object[] s2)
	{
		ArrayList<ArrayList<Entry<Integer, Integer>>> res = null;
		for (int v = robotState; v < mdpState; v++) //because I know this 
		{
			if ((int) s1[v] != (int) s2[v]) {
				if (res == null) {
					res = new ArrayList<ArrayList<Entry<Integer, Integer>>>();

					//I know there are only two states so hardcoding this 
					res.add(new ArrayList<Entry<Integer, Integer>>());
					res.add(new ArrayList<Entry<Integer, Integer>>());
				}

				res.get(0).add(new AbstractMap.SimpleEntry(v, (int) s1[v]));
				res.get(1).add(new AbstractMap.SimpleEntry(v, (int) s2[v]));
			}
		}
		return res;
	}

	//gets the value of the index of a state 
	public int getIndValFromState(int state, int ind)
	{
		State stateObj = states.get(state);
		Object[] stateArr = stateObj.varValues;
		int res = -1;
		if (ind < stateArr.length) {
			res = (int) stateArr[ind];
		}
		return res;
	}

	public int getRobotNum(int state)
	{
		//int robotState = 0; 
		return getIndValFromState(state, robotState);
	}
	
	public boolean isFailState(int state)
	{
		if (getIndValFromState(state,mdpState) == failState)
			return true; 
		else 
			return false; 
	}

	public BitSet getStatesWithSameRobotMDPState(int state)
	{
		BitSet res = new BitSet(states.size());
		State s1 = states.get(state);
		for (int s = 0; s < states.size(); s++) {
			if (robotMDPStatesAreTheSame(state, s)) {
				res.set(s);
			}
		}
		return res;
	}

	public boolean robotMDPStatesAreTheSame(int s1, int s2)
	{
		return robotMDPStatesAreTheSame(states.get(s1), states.get(s2));
	}

	private boolean robotMDPStatesAreTheSame(State s1, State s2)
	{

		return robotMDPStatesAreTheSame(s1.varValues, s2.varValues);
	}

	private boolean robotMDPStatesAreTheSame(Object[] s1, Object[] s2)
	{
		boolean res = false;
		//			for(int v = 0; v<numVar; v++) //this is very general and may cause issues later 
		//i can optimise this and I should v=robotState+1, v<mdpState
		//		for(int v=robotState+1; v<mdpState; v++)	
		//		{
		//				if(v!=robotState && v!=mdpState)
		//				{
		if (s1[robotState] == s2[robotState] && s1[mdpState] == s2[mdpState]) {
			res = true;

		}
		//				}
		//			}
		return res;
	}

	//merge states s1 and s2 such that the robot num and mdp bit are the same as s2 but 
	//the rest are the same as s1
	public int getMergedStateRobotMDP(int s1, int s2)
	{
		int statesToKeeps2[] = { robotState, mdpState };
		return getMergedState(states.get(s1), states.get(s2), statesToKeeps2);
	}

	public int getMergedState(State s1, State s2, int statesToKeeps2[])
	{
		int res = -1;
		Object s1v[] = s1.varValues.clone();
		Object s2v[] = s2.varValues;
		for (int i = 0; i < statesToKeeps2.length; i++) {
			s1v[statesToKeeps2[i]] = s2v[statesToKeeps2[i]];
		}
		res = getExactlyTheSameState(s1v);
		return res;
	}

	public int getExactlyTheSameState(Object s1v[])
	{
		int res = -1;
		for (int s = 0; s < states.size(); s++) {
			if (statesAreEqual(s1v, states.get(s))) {
				res = s;
				break;
			}
		}
		return res;
	}

	public boolean statesAreEqual(Object s1v[], State s2)
	{
		boolean res = true;
		Object s2v[] = s2.varValues;
		for (int v = 0; v < numVar; v++) {
			if ((int) s1v[v] != (int) s2v[v]) {
				res = false;
				break;
			}
		}
		return res;
	}

	public boolean statesAreEqual(State s1, State s2)
	{
		return statesAreEqual(s1.varValues, s2);
	}
};

class teamAutomatonWithoutSwitches
{
	public int numRobots; //the number of robots does not change so its okay to store it
	public BitSet finalAccStates; //the accepting states dont change so its okay to store them here 
	public int numStates; //the number of states in the initial MDP so no one has to remember it 
	public stateInfoFromStatesList allStates;
	public BitSet statesToAvoid;
	public MDPSimple teamAutomaton;
	public BitSet[] switchstatesRobots;

	public teamAutomatonWithoutSwitches(int numrobots, BitSet accstatesf, BitSet statestoavoid, MDPSimple teamAutomatonNoSwitches, BitSet[] switchStates)
	{
		numRobots = numrobots;
		finalAccStates = (BitSet) accstatesf.clone();
		statesToAvoid = (BitSet) statestoavoid.clone();
		numStates = teamAutomatonNoSwitches.numStates; //they should be the same with or without switches
		allStates = new stateInfoFromStatesList(teamAutomatonNoSwitches); //should be the same with or without switches
		teamAutomaton = copyMDPWithVarList(teamAutomatonNoSwitches); //so it doesnt have to be passed 
		switchstatesRobots = new BitSet[numrobots];
		for (int i = 0; i < switchStates.length; i++) {
			switchstatesRobots[i] = (BitSet) switchStates[i].clone();
		}

	}

	public MDPSimple copyMDPWithVarList(MDPSimple mdpToCopy)
	{
		MDPSimple res = new MDPSimple(mdpToCopy);
		res.setVarList((VarList) mdpToCopy.varList.clone());
		res.setStatesList(mdpToCopy.getStatesList());
		res.clearInitialStates();
		return res;
	}
	
	public MDPSimple copyMDPWithVarList(MDPSimple mdpToCopy,int[] initStates)
	{
		MDPSimple res = new MDPSimple(mdpToCopy);
		res.setVarList((VarList) mdpToCopy.varList.clone());
		res.setStatesList(mdpToCopy.getStatesList());
		res.clearInitialStates();
		for(int i =0; i< initStates.length; i++)
		{
			res.addInitialState(initStates[i]);
		}
		return res;
	}
	
	public MDPSimple getNewTeamAutomatonWithSwitches()
	{
		return copyMDPWithVarList(teamAutomaton);
	}
	public MDPSimple getNewTeamAutomatonWithSwitches(int[] initStates)
	{
		return copyMDPWithVarList(teamAutomaton,initStates);
	}
	public MDPSimple getNewTeamAutomatonWithSwitches(int initState)
	{
		int initStates[] = {initState};
		return copyMDPWithVarList(teamAutomaton,initStates);
	}


};
//policy class 
public class probPolicyTest
{
	String saveplace = System.getProperty("user.dir") + "/tests/decomp_tests/temp/";
	String switchInFailureAction = "switch_er";
	String simpleSwitchAction = "switch";

	public Queue<policyState> stuckStatesQueue;
	public ArrayList<Map.Entry<policyState, ArrayList<ArrayList<ArrayList<policyState>>>>> allPoliciesList; //includes all the policies, the same information as policy mdp but just easier to parse (with time) 
	public ArrayList<ArrayList<ArrayList<policyState>>> currentPolicy; //the policy that is currently being used / explored
	private int currentPolicyState; 
	public BitSet[] policyComputedFor; //store the states for which policy has been computed for each robot 
	public MDPSimple policyMDP; //the mdp that stores the policy 
	private int policyStatesMap[]; //contains the mapping from the original team automaton to this policy mdp  	
	private int stateNotAddedToPolicyVal; // the value of the map array if the state hasnt been added

	//stuff related to the mdp bit, maybe store this somewhere else but its helpful here too 
	//	public int numRobots; 	//the number of robots does not change so its okay to store it
	//	public BitSet finalAccStates; 		//the accepting states dont change so its okay to store them here 
	//	public int numStates; //the number of states in the initial MDP so no one has to remember it 
	//	public stateInfoFromStatesList allStates; 
	//	public BitSet statesToAvoid; 
	//	public MDPSimple teamAutomatonNoSwitches; 
	//	public BitSet[] switchstatesRobots; 

	public teamAutomatonWithoutSwitches tAutomaton;
	PrismLog mainLog;
	private boolean printPolicy;
	private boolean printHighlights;

	private boolean entrylistContains(ArrayList<Entry<Integer, Integer>> arayylist, int key)
	{
		boolean res = false;
		for (int i = 0; i < arayylist.size(); i++) {
			if (arayylist.get(i).getKey() == key) {
				res = true;
				break;
			}
		}
		return res;
	}
	public int[] getStatesFromCombinations(ArrayList<ArrayList<Entry<Integer, Integer>>> combinations, int cs) 
	{
		return tAutomaton.allStates.getStatesFromCombinations(combinations, cs);
	}

	public ArrayList<ArrayList<Entry<policyState, ArrayList<Entry<Integer, Integer>>>>> getChangedIndicesAcrossPolicyAtTime(policyState currRobotState)
	{
		return getChangedIndicesAcrossPolicyAtTime(currentPolicy, currRobotState);
	}

	public ArrayList<ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>>> getChangedIndicesAcrossPolicyAtTime(ArrayList<ArrayList<ArrayList<policyState>>> policy,
			policyState currRobotState)
	{
		int currTime = currRobotState.time;
		int currRobotNum = currRobotState.rnum;
		policyState currState;
		ArrayList<ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>>> allChangedStates = new ArrayList<ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>>>();
		
		//for each robot, get the policy at currTime 
		for (int r = 0; r < tAutomaton.numRobots; r++) {
			BitSet changedStates = new BitSet(tAutomaton.allStates.numVar);
			policyState stateAtBeginning = (policy.get(r)).get(0).get(0); //state at the beginning of the policy
			int currTimeRobot = currTime;
			if (currTimeRobot >= (policy.get(r).size())) //if there is no state for robot r at this time 
				currTimeRobot = policy.get(r).size() - 1; //then get the one at the last time step 

			ArrayList<policyState> currStates = (policy.get(r)).get(currTimeRobot);
			ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>> changedIndicesForStates = new ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>>();
			//for each state at this time get the changed values from this state and the state at the beginning 
			for (int s = 0; s < currStates.size(); s++) {
				currState = currStates.get(s);
				ArrayList<Entry<Integer, Integer>> changedIndicesForS2 = tAutomaton.allStates.compareDAStatesOnly(stateAtBeginning.state, currState.state,
						true);
				changedStates.or(tAutomaton.allStates.markIndicesInBitSetFromArrayList(changedIndicesForS2)); //so just saving all the states that have changed so far 
				if (changedIndicesForS2 == null)
					changedIndicesForS2 = new ArrayList<Entry<Integer, Integer>>();
				changedIndicesForStates.add(new AbstractMap.SimpleEntry(currState, changedIndicesForS2));
			}
			//great now we want to make sure we have values for each changed da element for each robot state we've compared 
			//so we want to remember the da states that were changed and their values even if they didnt change for this particular state but changed for some other state 
			//honestly at this point i have no idea why i'm doing this so i'm just going to go with it 
			//its driving me crazy 
			int totalChangedIndices = changedStates.cardinality(); //get the number of changed states 

			for (int ci = 0; ci < changedIndicesForStates.size(); ci++) {
				//for each changed thing 
				Entry<policyState, ArrayList<Entry<Integer, Integer>>> changedIndicesForStateEntry = changedIndicesForStates.get(ci);
				ArrayList<Entry<Integer, Integer>> changedIndicesForState = changedIndicesForStateEntry.getValue();
				int changedIndicesForStateSize = 0; 
				if(changedIndicesForState != null)
					changedIndicesForStateSize = changedIndicesForState.size();
				else
					changedIndicesForState = new ArrayList<Entry<Integer,Integer>>();
				
				if (totalChangedIndices != changedIndicesForStateSize) {
					//okay so just get the ones that you dont have values for and then add values for those 
					int setIndex = changedStates.nextSetBit(0);
					while (setIndex != -1) {
						if (!entrylistContains(changedIndicesForState, setIndex)) {
							//get the value at this point 
							currState = currStates.get(ci);
							int daval = tAutomaton.allStates.getIndValFromState(currState.state, setIndex);
							changedIndicesForState.add(new AbstractMap.SimpleEntry(setIndex, daval));
							
						}
						setIndex = changedStates.nextSetBit(setIndex+1);
					}
				}
			}
			allChangedStates.add(changedIndicesForStates);

		}
		return allChangedStates;
	}

	public stateInfoFromStatesList getAllStates()
	{
		return tAutomaton.allStates;
	}

	private void initialisePolicyComputedFor()
	{
		policyComputedFor = new BitSet[tAutomaton.numRobots];
		for (int i = 0; i < tAutomaton.numRobots; i++) {
			policyComputedFor[i] = new BitSet(tAutomaton.numStates);
		}
	}

	private void initialisePolicyMDP()
	{
		policyMDP = new MDPSimple();
		policyMDP.setVarList((VarList)tAutomaton.teamAutomaton.varList.clone());
		policyMDP.statesList = new ArrayList<State>();
		policyStatesMap = new int[tAutomaton.numStates];
		Arrays.fill(policyStatesMap, stateNotAddedToPolicyVal);
	}
	public void printPolicyMDPStates()
	{
		mainLog.println("Printing Policy MDP Indices");
		BitSet statesInPolicyMDP = new BitSet(policyStatesMap.length);
		//this is just for me 
		for(int i =0; i<policyStatesMap.length; i++)
		{
			if(policyStatesMap[i]!=stateNotAddedToPolicyVal)
			{
				mainLog.print(i+":"+policyStatesMap[i]+", ");
				statesInPolicyMDP.set(i);
			}
		}
		
		mainLog.println("\n"+statesInPolicyMDP.toString());
	}
	private void addStateToPolicyMDP(int state)
	{
		if (policyStatesMap[state]==stateNotAddedToPolicyVal) {
	//add to states list 		
			policyStatesMap[state] = policyMDP.numStates; 
			policyMDP.statesList.add(tAutomaton.allStates.states.get(state)); //should work 
			policyMDP.addState();}
	}
	public void addLinkInPolicyMDP(int[] states, double[] probs, int parentState, Object action,boolean normalize)
	{
		addStateToPolicyMDP(parentState);
		Distribution distr = new Distribution(); 
		String actionText = "childStates";
		double normalizer = 1.0; 
		if(normalize)
		{
			normalizer = 0; 
			for(int s = 0; s<probs.length; s++)
			normalizer+= probs[s];
		}
		if (normalizer != 0) {
		for(int s=0; s<states.length; s++)
		{
			addStateToPolicyMDP(states[s]); 
			distr.add(policyStatesMap[states[s]],probs[s]/normalizer );
		}
		if(action == null)
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, actionText);
		else 
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, action);
	}
	}

	public void addLinkInPolicyMDP(int state, double prob, int parentState, Object action)
	{
		ArrayList<Integer> states = new ArrayList<Integer>(); 
		states.add(state);
		ArrayList<Double> probs = new ArrayList<Double>(); 
	probs.add(prob);
		

		addLinkInPolicyMDP(states,probs,parentState,action);
	}
	public void addLinkInPolicyMDP(ArrayList<Integer> states, ArrayList<Double> probs, int parentState, Object action)
	{
		addStateToPolicyMDP(parentState);
		Distribution distr = new Distribution(); 
		String actionText = "childStates";
		for(int s=0; s<states.size(); s++)
		{
			addStateToPolicyMDP(states.get(s)); 
			distr.add(policyStatesMap[states.get(s)],probs.get(s) );
		}
		if(action == null)
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, actionText);
		else 
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, action);
	}
	private void initialiseCurrentPolicy()
	{
		currentPolicy = new ArrayList();
		for (int i = 0; i < tAutomaton.numRobots; i++) {
			currentPolicy.add(new ArrayList());
		}
	}

	private void initialiseAllPolicies()
	{
		allPoliciesList = new ArrayList();
	}

	public probPolicyTest(int numrobots, BitSet accstatesf, BitSet statestoavoid, MDPSimple teamAutomatonNoSwitches, BitSet[] switchStates, PrismLog log)
	{

		stateNotAddedToPolicyVal = -1;
		//		numRobots = numrobots; 
		//		finalAccStates = (BitSet)accstatesf.clone();
		//		statesToAvoid = (BitSet)statestoavoid.clone(); 
		//		numStates = teamAutomatonNoSwitches.numStates;  //they should be the same with or without switches
		//		allStates = new stateInfoFromStatesList(teamAutomatonNoSwitches); 	//should be the same with or without switches
		mainLog = log;
		printPolicy = false;
		printHighlights = true; 
		//		teamAutomatonNoSwitches = copyMDPWithVarList(teamAutomatonNoSwitches);	//so it doesnt have to be passed 
		tAutomaton = new teamAutomatonWithoutSwitches(numrobots, accstatesf, statestoavoid, teamAutomatonNoSwitches, switchStates);
		//set everything else to null we'll deal with it later 
		stuckStatesQueue = null;
		allPoliciesList = null;
		policyMDP = null;
		currentPolicy = null;
		policyComputedFor = null;

	}

	public void unfoldPolicy(MDPSimple sumprod, Strategy strat, int timecount, int parent_state)
	{
		unfoldPolicy(sumprod,strat,timecount,parent_state,-1,null);
		
	}

	
	public void unfoldPolicy(MDPSimple sumprod, Strategy strat, int timecount, int parent_state, int succState, int[] allRobotInitStates)
	{
		//is this the first time 
		boolean firstUnfolding = false;
		if (stuckStatesQueue == null) {
			//initialise everything 
			initialisePolicyMDP();
			//initialiseCurrentPolicy();
			initialiseAllPolicies();
			initialisePolicyComputedFor();
			stuckStatesQueue = new LinkedList<policyState>();
			firstUnfolding = true;
		}
		// just testing 
		//policyState testState = new policyState(0,0,6,parent_state); 
		//unfoldPolicyForState(sumprod,strat,testState);
		Iterator<Integer> initialStatesIter = sumprod.getInitialStates().iterator();
		while (initialStatesIter.hasNext()) {
			int state = initialStatesIter.next();
			int rnum = tAutomaton.allStates.getRobotNum(state);
			policyState testState = new policyState(rnum, timecount, state, parent_state); //so the timecount is always 0 
			unfoldPolicyForState(sumprod, strat, testState,succState, allRobotInitStates);

		}

	}

	public void unfoldPolicyForState(MDPSimple sumprod, Strategy strat, policyState rts, int nextSuccState, int[] allRobotInitStates)
	{
		if (printHighlights)
		mainLog.println("Unfolding Policy for State "+rts.toString());
		int state = rts.state; //current state 
		int choices = -1;
		int timecount = rts.time;
		int oldtimecount = timecount;
		Object action = null;

		BitSet discovered = new BitSet(tAutomaton.numStates); //just so we don't visit states over and over even though this shouldnt matter
		Stack<policyState> statesStack = new Stack(); //to store all the states we need to explore, so dfs to the goal , well its not dfs but yeah
		policyState currState = rts;
		currState.associatedStartState = rts.state;
		statesStack.push(currState);
		//TODO: what do you do when the policy has been computed before and we're just here again ? 
		//Do we care about this here or later ? 
		policyComputedFor[rts.rnum].set(rts.state); //say that we've computed the policy for this state now //we dont know if we'll need to add a link later
		//forget any of the other policies 
		initialiseCurrentPolicy();
		currentPolicyState = rts.state;

		while (!statesStack.isEmpty()) //the main bit - go to the next state, get the action go to the next state and so on
		{
			//TODO: addstatetopolmdp(currState.state) 
			//TODO: distribution 
			currState = statesStack.pop();
			timecount = currState.time;
			
			strat.initialise(currState.state);
			action = strat.getChoiceAction();
			currState.setAction(action);
			
			//just making sure we have enough elements in the array 
			while (currentPolicy.get(currState.rnum).size() < (timecount + 1)) {//+1 because we're adding successors 
				currentPolicy.get(currState.rnum).add(new ArrayList<policyState>());
			}
			if (!discovered.get(currState.state) & !tAutomaton.finalAccStates.get(currState.state)) //if it is not an accepting state and not discovered explore
			{
				//add it if it is a stuck state 
				if (action == null || "*".equals(action)) {
					//if this state has no next actions and it is not a final state then we need to add it to the stuck states Queue 
					//you cant add this state if the policy has already been computed for this state 
					//only add to stuck states if all robots have not failed 
					if(!allRobotsHaveFailed(allRobotInitStates)) {
					if(policyComputedFor[currState.rnum].get(currState.state))	//if you've said the policy is already computed 
					{
						//then you can add its successor states [basically get the next robot and add that state (like adding a switch)
						//i've already computed these 
						//TODO: come back here 
						//add the nextSuccState to the stuck states stack 
						if (nextSuccState != -1)
						{
							policyState nextState = new policyState(tAutomaton.allStates.getRobotNum(nextSuccState),
									currState.time,nextSuccState,currState.state,null,currState.state); 
							stuckStatesQueue.add(nextState);
							//add the entire policy for this state to to the big policy 
							if(allRobotInitStates!=null)
								createPolicyFromRobotInitStates(allRobotInitStates, currState);
							//and now like dont add it later or do anything so like just quit
							continue; 
							//TODO: this should only be the case for policies that have been expanded and are being expanded for the first time.
							
						}
						else
							mainLog.println("This state is interesting "+currState.toString());
						
					}
					else
					stuckStatesQueue.add(currState);
					//TODO: fix this later because we actually add its successors 
					//continue; 
					//we only add repeat states for different policies
					//once we've discovered a state we do not add it to the stuck state (cuz we added it once before) 
					//does this need fixing ? 
					//if the state and associated state are the same that means we're trying to compute it again 
					//so we should add successors or something 
					//TODO: fix this 
					}
					else 
					{
						String allRobotString ="["; 
						for(int i = 0; i<allRobotInitStates.length; i++)
							allRobotString+= allRobotInitStates[i]+ " "; 
						allRobotString+="]";
						if (printHighlights)
						mainLog.println("All robots have failed here "+allRobotString+" "+currState.toString());
					}
				}
				//					mainLog.println(currState.toString());
				discovered.set(currState.state);
				choices = sumprod.getNumChoices(currState.state);
				for (int c = 0; c < choices; c++) {
					if (action.equals(sumprod.getAction(currState.state, c))) {
						//TODO: the prodDistr stuff 
						ArrayList<Integer> states = new ArrayList<Integer>(); 
						ArrayList<Double> probs = new ArrayList<Double>();
						//TODO: adding to the MDP policy 
						Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(currState.state, c);
						while (iter.hasNext()) {
							Entry<Integer, Double> stateProbPair = iter.next();
							//get which robot 
							policyState succState;
							if (action.toString().contains(switchInFailureAction))
								succState = new policyState(tAutomaton.allStates.getRobotNum(stateProbPair.getKey()), currState.time, stateProbPair.getKey(),
										currState.state,null,rts.state); //just go back one time step
								
							else if (action.toString().contains(simpleSwitchAction))
								succState = new policyState(tAutomaton.allStates.getRobotNum(stateProbPair.getKey()), 0, stateProbPair.getKey(),
										currState.state,null,rts.state); //TODO: fix hardcoding this because I know its 0
							else
								succState = new policyState(tAutomaton.allStates.getRobotNum(stateProbPair.getKey()), currState.time + 1,
										stateProbPair.getKey(), currState.state,null,rts.state);
							succState.setProb(stateProbPair.getValue());
							statesStack.push(succState);
							states.add(succState.state); 
							probs.add(succState.getProb());
						}
						addLinkInPolicyMDP(states,probs, currState.state, action);
					}
				}

			}
			//				else if (finalAccStates.get(currState.state))
			//				{
			//					mainLog.println(currState.toString()+"!");
			//				}
			if (printPolicy)
			mainLog.println(currState.toString());
			addtoCurrentPolicy(currState);

		}
		//once we're done add the current policy to the big policy 
		//and also check if the policy has at least init values for all robots 
		if(allRobotInitStates != null)
			checkCurrentPolicy(allRobotInitStates,rts);
		addCurrentPolicyToAllPolicies(rts);

	}
	public void savePolicyMDPToFile(String location, String name) throws PrismException
	{
		policyMDP.exportToDotFile(location+name+".dot");
		policyMDP.exportToPrismExplicitTra(location+name+".tra");
	}

	private void checkCurrentPolicy(int[] robotInitStates,policyState state)
	{
		//just checking to make sure the current policy has states for each robot 
		//at least init states 
		for(int r = 0; r<robotInitStates.length; r++)
		{
			if(currentPolicy.get(r).size() == 0) //if the size of the policy for this robot is 0 we need to add an init state 
			{
				policyState currState = createRobotState(robotInitStates[r],state); 
				addtoCurrentPolicy(currState);
			}
		}
	}
	private boolean allRobotsHaveFailed(int[] robotInitStates)
	{
		boolean res = false; 
		if(robotInitStates != null) {
		int numrobots = robotInitStates.length; 
		int numFailed = 0; 
		for(int i=0; i<numrobots; i++)
		{
			if(tAutomaton.allStates.isFailState(robotInitStates[i]))
			{
				numFailed++;
			}
		}
		res=(numFailed == numrobots);
		}
		return res; 
	}
	private void createPolicyFromRobotInitStates(int[] robotInitStates, policyState state)
	{
		//so basically add this policy to the current state for this state 
		//we kind of know that this will only happen for when the currentState is null 
		//there is no way to check this actually but meh //so TODO: galiyan - muskurahat meri bolay kya socho naaaa :P 
		for(int r = 0; r<robotInitStates.length; r++)
		{
			policyState currState = createRobotState(robotInitStates[r],state); 
			addtoCurrentPolicy(currState);
		}
	}
	private policyState createPolicyStateFromNumState(int numState)
	{
		policyState res = new policyState(); 
		res.state = numState; 
		res.rnum = tAutomaton.allStates.getRobotNum(numState);
		return res; 
		
	}
	private policyState createRobotState(int numState, policyState state)
	{
		policyState currState = createPolicyStateFromNumState(numState); 
		currState.pstate = state.state; 
		currState.associatedStartState = state.state; 
		currState.time = 0; //TODO: what is this ??? fix this later do we even care ? 
		return currState;
	}
	private void addCurrentPolicyToAllPolicies(policyState startState)
	{
		SimpleEntry currentEntry = new AbstractMap.SimpleEntry<>(startState, deepCopyPolicy(currentPolicy));
		allPoliciesList.add(currentEntry);
	}
	//if policy for state found in all polices change it, otherwise let it be the same
	public void setCurrentPolicyToPolicyFromAllPolicies(int state)
	{
		if(currentPolicyState != state) {
		ArrayList<ArrayList<ArrayList<policyState>>> res = getPolicyFromAllPoliciesForState(state); 
		if(res != null)
			currentPolicy = res; 
		}
	}

	private ArrayList<ArrayList<ArrayList<policyState>>> getPolicyFromAllPoliciesForState(policyState rts)
	{
		return getPolicyFromAllPoliciesForState(rts.state);

	}
	
	private ArrayList<ArrayList<ArrayList<policyState>>> getPolicyFromAllPoliciesForState(int state)
	{
		ArrayList<ArrayList<ArrayList<policyState>>> res = null;
		for (int i = 0; i < allPoliciesList.size(); i++) {
			Entry<policyState, ArrayList<ArrayList<ArrayList<policyState>>>> currentState = allPoliciesList.get(i);
			if (currentState.getKey().state == state) {
				res = deepCopyPolicy(currentState.getValue());
			}
		}
		return res;

	}	

	private ArrayList<ArrayList<ArrayList<policyState>>> deepCopyPolicy(ArrayList<ArrayList<ArrayList<policyState>>> tocopy)
	{
		ArrayList<ArrayList<ArrayList<policyState>>> res = new ArrayList();
		for (int i = 0; i < tocopy.size(); i++) {
			ArrayList<ArrayList<policyState>> toadd = new ArrayList<ArrayList<policyState>>();

			for (int j = 0; j < tocopy.get(i).size(); j++) {
				ArrayList<policyState> toadd2 = new ArrayList<policyState>();
				for (int k = 0; k < tocopy.get(i).get(j).size(); k++) {
					toadd2.add(tocopy.get(i).get(j).get(k));
				}
				toadd.add(toadd2);
			}
			res.add(toadd);
		}
		return res;
	}

	private void addtoCurrentPolicy(policyState currState)
	{
		// TODO Auto-generated method stub
		while (currentPolicy.get(currState.rnum).size() < (currState.time + 1)) {
			currentPolicy.get(currState.rnum).add(new ArrayList<policyState>());
		}
		currentPolicy.get(currState.rnum).get(currState.time).add(currState);
	}
	

};
