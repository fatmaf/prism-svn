package explicit;

import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
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
import explicit.stateInfoFromStatesList;
import explicit.rewards.MDPRewardsSimple;

//class to store robot state  
class policyState implements Comparable<policyState>
{
	int BADVALUE = -2;
	
	public int rnum;
	public int time;
	public int state;
	public int pstate;
	public Object action;
	public int associatedStartState; 
	public double probFromParent; 
	public int associatedPolicyID; //add the id of the associated policy 
	public double cumulativeProb; //the super lazy not great way to do this 

	@Override
	public int compareTo(policyState ps)
	{
		int toret = 0;
		if(this.cumulativeProb==ps.cumulativeProb)
			toret =0; 
		else if (this.cumulativeProb < ps.cumulativeProb)
			toret = 1; 
		else
			toret = -1;
				
		return toret; 
	}
	public policyState()
	{
		rnum = BADVALUE;
		time = BADVALUE;
		state = BADVALUE;
		pstate = BADVALUE;
		associatedStartState = BADVALUE;
		action = null;
		probFromParent = 1; //so there are no errors really but this isnt smart 
		associatedPolicyID=BADVALUE;
		cumulativeProb = BADVALUE;
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
		pstate = BADVALUE;
		associatedStartState=BADVALUE;
		action = null;
		probFromParent = 1; 
		associatedPolicyID=BADVALUE;
		cumulativeProb = BADVALUE;
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
		associatedStartState = BADVALUE;
		action = null;
		probFromParent = 1; 
		associatedPolicyID=BADVALUE;
		cumulativeProb = BADVALUE;
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
		probFromParent = 1; 
		associatedPolicyID=BADVALUE;
		cumulativeProb = BADVALUE;
	}
	public policyState(policyState currState) {
			// TODO Auto-generated constructor stub
		rnum = currState.rnum;
		time = currState.time;
		state = currState.state;
		pstate = currState.pstate;
		action = currState.action;
		associatedStartState = currState.associatedStartState; 
		probFromParent = currState.probFromParent; 
		associatedPolicyID=currState.associatedPolicyID;
		cumulativeProb = currState.cumulativeProb;
		}

	public String toStringSmall()
	{
		String toret ="[t:" + time + ", s:" + state;
		if (action != null)
			toret = toret+", a:" + action ;
		if (probFromParent != 0)
			toret=toret+" ->"+probFromParent;
		if (cumulativeProb != BADVALUE)
			toret=toret+"("+cumulativeProb+") ";
		if (associatedPolicyID !=BADVALUE)
			toret = toret+"="+associatedPolicyID;
		toret = toret + "]";
		return toret; 
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
		if (associatedStartState != BADVALUE)
			toret = toret + ", ss:"+associatedStartState; 
		if (probFromParent != 0)
			toret=toret+" ->"+probFromParent;
		if (cumulativeProb != BADVALUE)
			toret=toret+"("+cumulativeProb+") ";
		if (associatedPolicyID !=BADVALUE)
			toret = toret+"="+associatedPolicyID;
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



	/* (non-Javadoc)
	 * @see java.lang.Object#equals(java.lang.Object)
	 */
	@Override
	public boolean equals(Object obj)
	{
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		policyState other = (policyState) obj;
		if (associatedStartState != other.associatedStartState)
			return false;
		if (pstate != other.pstate)
			return false;
		if (state != other.state)
			return false;
		if (time != other.time)
			return false;
		return true;
	}

	public boolean equalsNoTime(Object obj)
	{
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		policyState other = (policyState) obj;
		if (associatedStartState != other.associatedStartState)
			return false;
		if (pstate != other.pstate)
			return false;
		if (state != other.state)
			return false;
		return true;
	}
	
	public boolean sameState(Object obj)
	{
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		policyState other = (policyState) obj;
	
		if (state != other.state)
			return false;
	
		return true;
	}


	
};

//class CumulativeProbCompare implements Comparator<policyState>
//{
// @Override
//public int compare(policyState m1, policyState m2)
// {
//     if (m1.cumulativeProb < m2.cumulativeProb) return -1;
//     if (m1.cumulativeProb > m2.cumulativeProb) return 1;
//     else return 0;
// }
//}





class teamAutomatonWithoutSwitches
{
	int BADVALUE = -2;
	public int numRobots; //the number of robots does not change so its okay to store it
	public BitSet finalAccStates; //the accepting states dont change so its okay to store them here 
	public int numStates; //the number of states in the initial MDP so no one has to remember it 
	public stateInfoFromStatesList allStates;
	public BitSet statesToAvoid;
	public MDPSimple teamAutomaton;
	public BitSet[] switchstatesRobots;
	public MDPRewardsSimple teamAutomatonRewards; 

	public teamAutomatonWithoutSwitches(int numrobots, BitSet accstatesf, BitSet statestoavoid, MDPSimple teamAutomatonNoSwitches, BitSet[] switchStates,MDPRewardsSimple rewards)
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
		
		teamAutomatonRewards = null; 
		if (rewards!=null)
			teamAutomatonRewards= new MDPRewardsSimple(rewards);

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
class tempState{
	int BADVALUE = -2;
	int[] states; 


	
	public tempState(int ...a)
	{
		
	
		if(a.length>0) {
		states = new int[a.length]; 

		for(int i=0; i<a.length; i++)
		{
			states[i]=a[i];
		}
		}
		
	}
	

	@Override
	public String toString() {
		return "state=" + Arrays.toString(states) + "]";
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + Arrays.hashCode(states);
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		tempState other = (tempState) obj;
		if (!Arrays.equals(states, other.states))
			return false;
		return true;
	}
	
	
};
//policy class 
public class probPolicyTest
{
	
	int BADVALUE = -2;
	String switchInFailureAction = "switch_er";
	String simpleSwitchAction = "switch";

	public PriorityQueue<policyState> stuckStatesQueue;
	public ArrayList<Map.Entry<policyState, ArrayList<ArrayList<ArrayList<policyState>>>>> allPoliciesList; //includes all the policies, the same information as policy mdp but just easier to parse (with time) 
	public ArrayList<ArrayList<ArrayList<policyState>>> currentPolicy; //the policy that is currently being used / explored
	private int currentPolicyState; 
	public BitSet[] policyComputedFor; //store the states for which policy has been computed for each robot
	public ArrayList<policyState> policyComputedForList;
	public ArrayList<int[]> policyComputedForListAllRobotStates;
	public MDPSimple policyMDP; //the mdp that stores the policy 
	private int policyStatesMap[]; //contains the mapping from the original team automaton to this policy mdp  	
	private int stateNotAddedToPolicyVal; // the value of the map array if the state hasnt been added
	private int policyCounter; //to count how many times policies have been expanded and use as an id for them 
	public MDPSimple combinedPolicyMDP;
	private ArrayList<Map.Entry<tempState,Integer>> combinedPolicyStatesMap; //contains the mapping from the original team automaton to this policy mdp  	
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
	private ArrayList<int[]> policyComputedForListAllRobots;
 

	
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

	private ArrayList<Entry<Integer, Integer>> combineTwoArrayLists(ArrayList<Entry<Integer, Integer>> one, ArrayList<Entry<Integer, Integer>> two)
	{
		if (one != null || two!= null)
			{
			if (one == null) 
				one = new ArrayList<Entry<Integer, Integer>>();
			if(two != null) {
		for (Entry<Integer, Integer> x : two){
			   if (!one.contains(x))
			      one.add(x);
			}}
			}
		return one;
	}
	public ArrayList<ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>>> getChangedIndicesAcrossPolicyAtTime(ArrayList<ArrayList<ArrayList<policyState>>> policy,
			policyState currRobotState)
	{
		int currTime = currRobotState.time;
		int currRobotNum = currRobotState.rnum;
		
		policyState currState;
		policyState nextRobotBeginning = null;
		ArrayList<ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>>> allChangedStates = new ArrayList<ArrayList<Entry<policyState,ArrayList<Entry<Integer, Integer>>>>>();
		
		//for each robot, get the policy at currTime 
		for (int r = 0; r < tAutomaton.numRobots; r++) {
			BitSet changedStates = new BitSet(tAutomaton.allStates.numVar);
			policyState stateAtBeginning = (policy.get(r)).get(0).get(0); //state at the beginning of the policy
			if(r < currRobotNum) //could this condition be a problem ? think about this later please 
				//what i'm trying to do is make sure that changes from previous bits are reflected. However it is important to (omg you're done with this however stuff okay) 
				//remember that if we're going round, then those changes need to be preserved for robots > currRobotNum too. 
			{
				nextRobotBeginning = (policy.get(r+1).get(0)).get(0);
			}
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
				
//				//compare with previous state. 
				if(r < currRobotNum) {
				ArrayList<Entry<Integer, Integer>> changedIndicesBetweenRAndCurrRobotState = tAutomaton.allStates.compareDAStatesOnly(currState.state,
						nextRobotBeginning.state,false);
				changedIndicesForS2=combineTwoArrayLists(changedIndicesForS2,changedIndicesBetweenRAndCurrRobotState);
				}
				//but we also want to remember that the changes for currRobotNum override all the others 
				//something we could fix elsewhere ? 
				//TODO: above
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
		policyComputedForList = new ArrayList<policyState>();
		policyComputedForListAllRobots = new ArrayList<int[]>();
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
	private void initialiseCombinedPolicyMDP() throws PrismLangException
	{
		combinedPolicyMDP = new MDPSimple();
		VarList combVarlist = new VarList(); 
	//	newVarList.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
		for(int i = 0; i<tAutomaton.numRobots; i++)
		{
			combVarlist.addVar(0, new Declaration("r"+i,new DeclarationIntUnbounded()), 1, null);
		}
		combinedPolicyMDP.setVarList((VarList)combVarlist.clone());
		
		combinedPolicyMDP.statesList = new ArrayList<State>();
		int maxStates = 17;
		combinedPolicyMDP.statesList = new ArrayList<State>();
		combinedPolicyStatesMap = new ArrayList<Map.Entry<tempState,Integer>>();
		
		
	}
	private int findInCombinedPolicyStatesMap(tempState temp)
	{
		int toret = BADVALUE; 
		for(int i = 0; i<combinedPolicyStatesMap.size(); i++)
		{
			if(combinedPolicyStatesMap.get(i).getKey().equals(temp))
			{
				toret = i;
			}
		}
		return toret; 
	}
	private boolean checkCombinedPolicyStatesMap(int[] states,int value)
	{
		tempState temp = new tempState(states); 
		int index = findInCombinedPolicyStatesMap(temp);
		if (index != BADVALUE)
		{
			if(combinedPolicyStatesMap.get(index).getValue()== stateNotAddedToPolicyVal)
			{combinedPolicyStatesMap.get(index).setValue(value);
			return false;
				
			}
		}
		else
		{
			combinedPolicyStatesMap.add(new AbstractMap.SimpleEntry<tempState,Integer>(temp,value)); 
			index = combinedPolicyStatesMap.size()-1; 

			return false;
		}
	
		return true;
	}
	private int[] getRobotStatesFromCombStates(int[] states)
	{
		for (int i =0; i<states.length; i++)
		{
			if(states[i]==tAutomaton.allStates.failState)
			{
				states[i]=0;
			}
			states[i] = tAutomaton.allStates.getMDPState(states[i]);
		}
		return states;
	}
	private int addStateToCombinedPolicyMDP(int[] states)
	{
		states = getRobotStatesFromCombStates(states);
		if (!checkCombinedPolicyStatesMap(states,combinedPolicyMDP.numStates)) {
			
			State toadd = new State(tAutomaton.numRobots); 
			for(int i = 0; i<tAutomaton.numRobots; i++)
			{
				toadd.setValue(i, states[i]);
			}
			combinedPolicyMDP.statesList.add(toadd);
			combinedPolicyMDP.addState();}
		tempState temp = new tempState(states); 
		int index = findInCombinedPolicyStatesMap(temp);
		return index;
		
	}
	public void addLinkInCombinedPolicyMDP(ArrayList<int[]> states, ArrayList<Double> probs, int[] parentState, Object action,String additionalText)
	{
	
		int indexPS = addStateToCombinedPolicyMDP(parentState);
		Distribution distr = new Distribution(); 
		String actionText = Arrays.toString(parentState);
				
				
		
		for(int s=0; s<states.size(); s++)
		{
			
			int index = addStateToCombinedPolicyMDP(states.get(s)); 
			distr.add(combinedPolicyStatesMap.get(index).getValue(),probs.get(s) );
			actionText+="->"+Arrays.toString(states.get(s));
		}
		//just to check if I have multiple branches because I'm going over the same stuff again 
		int numChoices = combinedPolicyMDP.getNumChoices(combinedPolicyStatesMap.get(indexPS).getValue());
		if(action == null)
			combinedPolicyMDP.addActionLabelledChoice(combinedPolicyStatesMap.get(indexPS).getValue(), distr, additionalText+"_"+actionText+"."+numChoices);
		else 
			combinedPolicyMDP.addActionLabelledChoice(combinedPolicyStatesMap.get(indexPS).getValue(), distr, additionalText+"_"+action.toString()+"."+numChoices);
	
	}
	public void addAllPoliciesListToCombinedPolicyMDP()
	{
		 for(int polNo = 0; polNo<allPoliciesList.size(); polNo++)
		 {
			 Entry<policyState, ArrayList<ArrayList<ArrayList<policyState>>>> currentPolPair = allPoliciesList.get(polNo); 
			 ArrayList<ArrayList<ArrayList<policyState>>> currentPol = currentPolPair.getValue();
			 //size of arr = num of robots 
			 //inner thing is time but we want time across robots
			 int numSucc = 2;
			 int t=1;
			 int succt = 0;
			 int ps[]= new int[tAutomaton.numRobots];
			 String psaction = ""; 
			 String succaction = "";
			 //int ss[]= new int[tAutomaton.numRobots];
			 ArrayList<int[]> successors = new ArrayList<int[]>();
			 ArrayList<Double> probs = new ArrayList<Double>();
			 //hardcoding this 
			 for(int i =0; i<numSucc; i++ )
			 { successors.add(new int[tAutomaton.numRobots]);
			 	probs.add(1.0);
			 }
			 int maxT = 0; 
			 for(int r = 0; r<tAutomaton.numRobots; r++)
			 { if (maxT <currentPol.get(r).size())
				 maxT = currentPol.get(r).size();
			 if(currentPol.get(r).size() > 0) {
				 if(currentPol.get(r).get(0).size() > 0) {
			 	ps[r] = currentPol.get(r).get(0).get(0).state; 
			 	if(currentPol.get(r).get(0).get(0).action != null)
				 psaction+= "\n"+currentPol.get(r).get(0).get(0).action.toString();
				 }
			 }}
			 while(t<maxT)
			 {
				 
				 
				 for(int r=0; r<tAutomaton.numRobots; r++)
				 {
					 if(currentPol.get(r).size()>t) {
					 int size = currentPol.get(r).get(t).size();
					 for(int ns = 0; ns<size; ns++)
					 {
						 successors.get(ns)[r]=currentPol.get(r).get(t).get(ns).state;
						 probs.set(ns, probs.get(ns)*currentPol.get(r).get(t).get(ns).getProb());
						 //probs.add(ns, probs.get(ns)*currentPol.get(r).get(t).get(ns).getProb());
						 if (currentPol.get(r).get(t).get(ns).action != null)
						 succaction+="\n"+currentPol.get(r).get(t).get(ns).action.toString();
					 }
					 if (size < numSucc && size!= 0)
					 {
						 successors.get(1)[r]= currentPol.get(r).get(t).get(size-1).state;
					 }
					 else if (size == 0)
					 {
						 for(int ns = 0; ns<numSucc; ns++)
						 {
							 successors.get(ns)[r]=ps[r];
						 }
					 }
				 }
					 else
					 {
						 for(int ns = 0; ns<numSucc; ns++)
						 {
							 successors.get(ns)[r]=ps[r];
						 }
					 }
					 
				 }
				 
				
				t++;
				addLinkInCombinedPolicyMDP(successors, probs, ps,psaction,"");
				ps = successors.get(0).clone(); 
				psaction = succaction;
				succaction = "";
			 }
		 }
	}
	public void printAllPoliciesList(PrismLog printLog)
	{
		 for(int polNo = 0; polNo<allPoliciesList.size(); polNo++)
		 {
		printPolicyNo(polNo,printLog);
		 }
	}
	public void printPolicyNo(int polNo,PrismLog printLog)
	{
	
			 Entry<policyState, ArrayList<ArrayList<ArrayList<policyState>>>> currentPolPair = allPoliciesList.get(polNo); 
			 ArrayList<ArrayList<ArrayList<policyState>>> currentPol = currentPolPair.getValue();
			 printPolicy(currentPol,printLog);
			
		 
	}
	public void printCurrentPolicy(PrismLog printLog)
	{
		printPolicy(currentPolicy,printLog);
	}
	public void printPolicy(ArrayList<ArrayList<ArrayList<policyState>>> currentPol,PrismLog printLog)
	{
		
		 //size of arr = num of robots 
		 //inner thing is time but we want time across robots
		 int numSucc = 2;
		 int t=1;
		 int succt = 0;
		 int ps[]= new int[tAutomaton.numRobots];
		 String psaction = ""; 
		 String succaction = "";
		 //int ss[]= new int[tAutomaton.numRobots];
		 ArrayList<int[]> successors = new ArrayList<int[]>();
		 ArrayList<Double> probs = new ArrayList<Double>();
		 //hardcoding this 
		 for(int i =0; i<numSucc; i++ )
		 { successors.add(new int[tAutomaton.numRobots]);
		 	probs.add(1.0);
		 }
		 int maxT = 0; 
		 String toprint = "";
		 for(int r = 0; r<tAutomaton.numRobots; r++)
		 { if (maxT <currentPol.get(r).size())
			 maxT = currentPol.get(r).size();
		 if(currentPol.get(r).size() > 0) {
			 if(currentPol.get(r).get(0).size() > 0) {
		 	ps[r] = currentPol.get(r).get(0).get(0).state; 
		 	toprint+=r+":"+ currentPol.get(r).get(0).get(0).toStringSmall()+",";
		 
			 }
		 }}
		 printLog.println(toprint);
		 
		 while(t<maxT)
		 {
			
			 toprint="";
			 for(int r=0; r<tAutomaton.numRobots; r++)
			 {
				 if(currentPol.get(r).size()>t) {
				 int size = currentPol.get(r).get(t).size();
				 for(int ns = 0; ns<size; ns++)
				 {
					 successors.get(ns)[r]=currentPol.get(r).get(t).get(ns).state;
						toprint+=r+":"+ currentPol.get(r).get(t).get(ns).toStringSmall()+",";
					 probs.set(ns, probs.get(ns)*currentPol.get(r).get(t).get(ns).getProb());
					 //probs.add(ns, probs.get(ns)*currentPol.get(r).get(t).get(ns).getProb());
					
				 }
				 if (size < numSucc && size!= 0)
				 {
					 successors.get(1)[r]= currentPol.get(r).get(t).get(size-1).state;
				 }
				 else if (size == 0)
				 {
					 for(int ns = 0; ns<numSucc; ns++)
					 {
						 successors.get(ns)[r]=ps[r];
					 }
				 }
			 }
				 else
				 {
					 for(int ns = 0; ns<numSucc; ns++)
					 {
						 successors.get(ns)[r]=ps[r];
					 }
				 }
				 
			 }
			 
			
			t++;
			printLog.println(toprint);
			//addLinkInCombinedPolicyMDP(successors, probs, ps,psaction,"");
			 }
		 printLog.println();
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
				mainLog.print(i+":"+policyStatesMap[i]+"\n");
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
	
	public void addLinkInPolicyMDP(int[] states, double[] probs, int parentState, Object action,boolean normalize,String additionalText)
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
			probs[s]=(probs[s]/normalizer);
		}
		//just to check if I have multiple branches because I'm going over the same stuff again 
		int numChoices = policyMDP.getNumChoices(policyStatesMap[parentState]);
		if(action == null)
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, additionalText+"_"+actionText+"."+numChoices);
		else 
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, additionalText+"_"+action.toString()+"."+numChoices);
	}
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
		int numChoices = policyMDP.getNumChoices(policyStatesMap[parentState]);
		if(action == null)
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, actionText+"."+numChoices);
		else 
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, action.toString()+"."+numChoices);
	}

//	addLinkInPolicyMDP(states, probs, parentState,  action, normalize,"");
	}
	public void addLinkInPolicyMDP(int state, double prob, int parentState, Object action)
	{
		ArrayList<Integer> states = new ArrayList<Integer>(); 
		states.add(state);
		ArrayList<Double> probs = new ArrayList<Double>(); 
	probs.add(prob);
		

		addLinkInPolicyMDP(states,probs,parentState,action);
	}
	public void addLinkInPolicyMDP(int state, double prob, int parentState, Object action, String additionalText)
	{
		addStateToPolicyMDP(parentState);
		Distribution distr = new Distribution(); 
		String actionText = "childStates";
			addStateToPolicyMDP(state); 
			distr.add(policyStatesMap[state],prob );
		int numChoices = policyMDP.getNumChoices(policyStatesMap[parentState]);
		if(action == null)
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, additionalText+"_"+actionText+"."+numChoices);
		else 
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, additionalText+"_"+action.toString()+".");
		//^adding +numChoices to the above line caused problems, I have no idea why, probably need to ask dave, same state same stuff ? 
		//its evening outside and the tree is gliterring =D 

//		ArrayList<Integer> states = new ArrayList<Integer>(); 
//		states.add(state);
//		ArrayList<Double> probs = new ArrayList<Double>(); 
//	probs.add(prob);
//		
//
//		addLinkInPolicyMDP(states,probs,parentState,action,additionalText);
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
		int numChoices = policyMDP.getNumChoices(policyStatesMap[parentState]);
		if(action == null)
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, actionText+"."+numChoices);
		else 
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, action.toString()+"."+numChoices);

	}
	public void addLinkInPolicyMDP(ArrayList<Integer> states, ArrayList<Double> probs, int parentState, Object action,String additionalText)
	{
		addStateToPolicyMDP(parentState);
		Distribution distr = new Distribution(); 
		String actionText = "childStates";
		for(int s=0; s<states.size(); s++)
		{
			addStateToPolicyMDP(states.get(s)); 
			distr.add(policyStatesMap[states.get(s)],probs.get(s) );
		}
		int numChoices = policyMDP.getNumChoices(policyStatesMap[parentState]);
		if(action == null)
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, additionalText+"_"+actionText+"."+numChoices);
		else 
			policyMDP.addActionLabelledChoice(policyStatesMap[parentState], distr, additionalText+"_"+action.toString()+"."+numChoices);
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

	public probPolicyTest(int numrobots, BitSet accstatesf, BitSet statestoavoid, MDPSimple teamAutomatonNoSwitches, BitSet[] switchStates, PrismLog log, MDPRewardsSimple rewards)
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
		tAutomaton = new teamAutomatonWithoutSwitches(numrobots, accstatesf, statestoavoid, teamAutomatonNoSwitches, switchStates,rewards);
		//set everything else to null we'll deal with it later 
		stuckStatesQueue = null;
		allPoliciesList = null;
		policyMDP = null;
		currentPolicy = null;
		policyComputedFor = null;
		policyComputedForList =null;
		policyComputedForListAllRobots = null;

	}

	public void unfoldPolicy(MDPSimple sumprod, Strategy strat, int timecount, policyState currState) throws PrismException
	{
		unfoldPolicy(sumprod,strat,timecount,currState,-1,null);
		
	}

	
	public void unfoldPolicy(MDPSimple sumprod, Strategy strat, int timecount, policyState currState, int succState, int[] allRobotInitStates) throws PrismException
	{
		int parent_state = currState.pstate;
		//is this the first time 
		boolean firstUnfolding = false;
		if (stuckStatesQueue == null) {
			//initialise everything 
			initialisePolicyMDP();
			//initialiseCurrentPolicy();
			initialiseAllPolicies();
			initialisePolicyComputedFor();
			stuckStatesQueue = new PriorityQueue<policyState>();
			firstUnfolding = true;
			policyCounter = -1;
		}
		// just testing 
		//policyState testState = new policyState(0,0,6,parent_state); 
		//unfoldPolicyForState(sumprod,strat,testState);
		Iterator<Integer> initialStatesIter = sumprod.getInitialStates().iterator();
		while (initialStatesIter.hasNext()) {
			int state = initialStatesIter.next();
			int rnum = tAutomaton.allStates.getRobotNum(state);
			policyState testState = new policyState(currState);//new policyState(rnum, timecount, state, parent_state); //so the timecount is always 0 
			testState.state = state;
			testState.rnum = rnum;
			
			unfoldPolicyForState(sumprod, strat, testState,succState, allRobotInitStates);

		}

	}

	private boolean arrayListContainsArray(ArrayList<int[]> arrlist, int[] arr)
	{
		boolean res = false; 
		for(int i = 0; i<arrlist.size(); i++)
		{
			int[] currArr = arrlist.get(i);
			boolean arraysAreTheSame = true;
			for(int j = 0; j<arr.length; j++)
			{
				if (currArr[j]!=arr[j])
				{	arraysAreTheSame = false;
				break;
				}
			}
			if(arraysAreTheSame)
				{res = true; break;}
		}
		return res;
	}
	public void unfoldPolicyForState(MDPSimple sumprod, Strategy strat, policyState rts, int nextSuccState, int[] allRobotInitStates) throws PrismException
	{
		if (printHighlights)
		mainLog.println("Unfolding Policy for State "+rts.toString());
		int state = rts.state; //current state 
		int choices = -1;
		int timecount = rts.time;
		int oldtimecount = timecount;
		Object action = null;

		BitSet discovered = new BitSet(tAutomaton.numStates); //just so we don't visit states over and over even though this shouldnt matter
		List<policyState> discoveredStates = new LinkedList<policyState>();
		//on second thought this does not matter (the thing below) because we might visit the same state again but it may be a child of another state 
		//and so the inital state of the robot would have changed (though maybe no new task would be acheived)
//		BitSet addedToStuckQ = new BitSet(tAutomaton.numStates); //for each policy unfolding we don't need to add the same state over and over again to the stuck stack
		
		Stack<policyState> statesStack = new Stack<policyState>(); //to store all the states we need to explore, so dfs to the goal , well its not dfs but yeah
		policyState currState = rts;
		currState.associatedStartState = rts.state;
		statesStack.push(currState);
		//TODO: what do you do when the policy has been computed before and we're just here again ? 
		//Do we care about this here or later ? 
		policyComputedForList.add(rts);
		policyComputedFor[rts.rnum].set(rts.state); //say that we've computed the policy for this state now //we dont know if we'll need to add a link later
		policyComputedForListAllRobots.add(allRobotInitStates.clone());
		//forget any of the other policies 
		initialiseCurrentPolicy();
		currentPolicyState = rts.state;
		String prevTextToAdd = "";
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
			while (currentPolicy.get(currState.rnum).size() < ((timecount-rts.time) + 1)) {//+1 because we're adding successors 
				currentPolicy.get(currState.rnum).add(new ArrayList<policyState>());
			}
			if (
					!discoveredStates.contains(currState)
					//!discovered.get(currState.state)
					& !tAutomaton.finalAccStates.get(currState.state)) //if it is not an accepting state and not discovered explore
			{
				//add it if it is a stuck state 
				if (action == null || "*".equals(action)) {
					//if this state has no next actions and it is not a final state then we need to add it to the stuck states Queue 
					//you cant add this state if the policy has already been computed for this state 
					//only add to stuck states if all robots have not failed 
			

					if(!allRobotsHaveFailed(allRobotInitStates)) {	
						boolean addToStuckStates = true;
//						if(policyComputedForList.contains(currState))
//						{
//							mainLog.println("computing policy for a state thats already done");
//						}
//						if(arrayListContainsArray(policyComputedForListAllRobots,allRobotInitStates))
//						{
//							mainLog.println("computing policy for a state thats already done for all robots");
//						}
						
//					if(
//							policyComputedFor[currState.rnum].get(currState.state)
//							)	//if you've said the policy is already computed 
						if(policyComputedForList.contains(currState) && arrayListContainsArray(policyComputedForListAllRobots,allRobotInitStates))
					{
						//then you can add its successor states [basically get the next robot and add that state (like adding a switch)
						//i've already computed these 
						//TODO: come back here 
						//add the nextSuccState to the stuck states stack 
						if (nextSuccState != -1)
						{
							policyState nextState = new policyState(tAutomaton.allStates.getRobotNum(nextSuccState),
									currState.time,nextSuccState,currState.state,null,currState.state); 
							//add a link from currState to nextState in the policy MDP 
							//add the entire policy for this state to to the big policy 
							//and update the initstate of the robot in question (so there are no repeats) 
							if(allRobotInitStates!=null) {
								allRobotInitStates[currState.rnum] = currState.state;
								createPolicyFromRobotInitStates(allRobotInitStates, currState);
								}
							nextState.associatedPolicyID = allPoliciesList.size(); //this works on the assumption that the current policy is empty ^
							nextState.cumulativeProb=currState.cumulativeProb;
							String textToAdd = currState.state+"_"+currState.pstate+"_"+rts.associatedPolicyID;//"abc";//currState.state+"_"+currState.associatedStartState; 
							

							addLinkInPolicyMDP(nextState.state,1.0,currState.state,"switch_er_"+currState.rnum+"_"+nextState.rnum,textToAdd);
							//^ caused the edge lines to disappear in the policy mdp, I'm not entirely sure why, so I removed the number in this function 

							stuckStatesQueue.add(nextState);
							
							
							continue; 
						}
							//TODO: this should only be the case for policies that have been expanded and are being expanded for the first time.
							
						
						else
							{mainLog.println("This state is interesting "+currState.toString());
							throw new PrismException("Something unexpected happened while unrolling the policy for "+rts.toString()+" at "+currState.toString());
							}
					}
					else
					{	currState.associatedPolicyID = allPoliciesList.size();//policyCounter+1; 
						stuckStatesQueue.add(currState);
					
					}
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
				//discovered.set(currState.state);
				discoveredStates.add(currState);
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
								succState = new policyState(tAutomaton.allStates.getRobotNum(stateProbPair.getKey()), rts.time, stateProbPair.getKey(),
										currState.state,null,rts.state); //same as the initial states time
							else
								succState = new policyState(tAutomaton.allStates.getRobotNum(stateProbPair.getKey()), currState.time + 1,
										stateProbPair.getKey(), currState.state,null,rts.state);
							succState.setProb(stateProbPair.getValue());
							succState.cumulativeProb=currState.cumulativeProb*succState.probFromParent;
							statesStack.push(succState);
							states.add(succState.state); 
							probs.add(succState.getProb());
						}
						addLinkInPolicyMDP(states,probs, currState.state, action,currState.state+"_"+rts.state+"_"+rts.associatedStartState);
//						addLinkInPolicyMDP(states,probs, currState.state, action);
						break; 
						//don't go over all choices - actions are unique
					}
				}

			}
			//				else if (finalAccStates.get(currState.state))
			//				{
			//					mainLog.println(currState.toString()+"!");
			//				}
			if (printPolicy)
			mainLog.println(currState.toString());
			addtoCurrentPolicy(currState,rts.time);

		}
		//once we're done add the current policy to the big policy 
		//and also check if the policy has at least init values for all robots 
//		if(allRobotInitStates != null)
//			checkCurrentPolicy(allRobotInitStates,rts);
		checkCurrentPolicyHasPolicyForAllRobotsAndFix(allRobotInitStates,rts);
		addCurrentPolicyToAllPolicies(rts);

	}
	
	public void savePolicyMDPToFile(String location, String name) throws PrismException
	{
		policyMDP.exportToDotFile(location+name+".dot");
		PrismLog out = new PrismFileLog(location+name+"_sta.dot");
		policyMDP.exportToDotFile(out, null, true);
		out.close();
		policyMDP.exportToPrismExplicitTra(location+name+".tra");
		
		initialiseCombinedPolicyMDP();
		printAllPoliciesList(mainLog);
		out = new PrismFileLog(location+name+"_policies.txt");
		printAllPoliciesList(out);
		out.close();
		out = new PrismFileLog(location+name+"_policies_dump.txt");
		out.println(allPoliciesList.toString());
		out.close();
		addAllPoliciesListToCombinedPolicyMDP();
		name = "comb"+name;
		combinedPolicyMDP.exportToDotFile(location+name+".dot");
		//PrismLog 
		out = new PrismFileLog(location+name+"_sta.dot");
		combinedPolicyMDP.exportToDotFile(out, null,true);
		out.close();
		combinedPolicyMDP.exportToPrismExplicitTra(location+name+".tra");
		
	}
	
	public void savePolicyMDPToFile(MDPSimple policyMDP,String location, String name) throws PrismException
	{
		policyMDP.exportToDotFile(location+name+".dot");
		PrismLog out = new PrismFileLog(location+name+"_sta.dot", true);
		policyMDP.exportToDotFile(out, null, true);
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
	private boolean checkCurrentPolicyHasPolicyForAllRobotsAndFix(int[] robotInitStates,policyState state)
	{
		boolean res = true;
		for(int r=0; r<tAutomaton.numRobots; r++)
		{
			if(currentPolicy.get(r).size()==0)
			{
				if(robotInitStates == null) {
				res = false; 
				break; 
				}
				else
				{
					policyState currState = createRobotState(robotInitStates[r],state); 
					addtoCurrentPolicy(currState,state.time); //make this more robust cuz i know the offset 
				}
			}
		}
		if(!res)
			mainLog.println("ERROR HERE!!!");
		return res; 
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
	public boolean allRobotsHaveFailed(int[] robotInitStates,policyState currRobot)
	{
		robotInitStates[currRobot.rnum]=currRobot.state; 
		return allRobotsHaveFailed(robotInitStates);
	}
	private void createPolicyFromRobotInitStates(int[] robotInitStates, policyState state)
	{
		//so basically add this policy to the current state for this state 
		//we kind of know that this will only happen for when the currentState is null 
		//there is no way to check this actually but meh //so TODO: galiyan - muskurahat meri bolay kya socho naaaa :P 
		for(int r = 0; r<robotInitStates.length; r++)
		{
			policyState currState = createRobotState(robotInitStates[r],state); 
			addtoCurrentPolicy(currState,state.time);
		}
	}
	private policyState createPolicyStateFromNumState(int numState)
	{
		policyState res = new policyState(); 
		res.state = numState; 
		res.rnum = tAutomaton.allStates.getRobotNum(numState);
		return res; 
		
	}
	public int getRobotStateFromMDPState(int state,int robotNum)
	{
		int res = BADVALUE; 
		res = tAutomaton.allStates.getRobotStateFromMDPState(state, robotNum);
		return res; 
	}
	private policyState createRobotState(int numState, policyState state)
	{
		policyState currState = createPolicyStateFromNumState(numState); 
		currState.pstate = state.state; 
		currState.associatedStartState = state.state; 
		currState.time = state.time; 
		currState.probFromParent = state.probFromParent; 
		currState.cumulativeProb = state.cumulativeProb;
		return currState;
	}
	private void addCurrentPolicyToAllPolicies(policyState startState)
	{
		SimpleEntry currentEntry = new AbstractMap.SimpleEntry<>(startState, deepCopyPolicy(currentPolicy));
		allPoliciesList.add(currentEntry);
		policyCounter = allPoliciesList.size()-1;
	//	policyCounter++;
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
	//if policy for state found in all polices change it, otherwise let it be the same
	public void setCurrentPolicyToPolicyFromAllPoliciesPolicyID(int polID)
	{
		if(policyCounter!= polID) {
		ArrayList<ArrayList<ArrayList<policyState>>> res = getPolicyFromAllPoliciesIDForState(polID); 
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
//		for (int i = 0; i < allPoliciesList.size(); i++) {
		for(int i = allPoliciesList.size()-1; i>=0; i--) {
			Entry<policyState, ArrayList<ArrayList<ArrayList<policyState>>>> currentState = allPoliciesList.get(i);
			if (currentState.getKey().state == state) {	//possible errors because of this matching - cuz you dont care about the pstate or associatedstate and this is important 
				res = deepCopyPolicy(currentState.getValue()); 
				break; // this does not get the latest ? do I care about this ? this will change things so maybe reverse the order 
			}
		}
		return res;

	}	
	
	private ArrayList<ArrayList<ArrayList<policyState>>> getPolicyFromAllPoliciesIDForState(int polID)
	{
		ArrayList<ArrayList<ArrayList<policyState>>> res = null;
			Entry<policyState, ArrayList<ArrayList<ArrayList<policyState>>>> currentState = allPoliciesList.get(polID);
//			if (currentState.getKey().state == state) {	//possible errors because of this matching - cuz you dont care about the pstate or associatedstate and this is important 
				res = deepCopyPolicy(currentState.getValue()); 
//				break; // this does not get the latest ? do I care about this ? this will change things so maybe reverse the order 
//			}
	
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
		while (currentPolicy.get(currState.rnum).size() < ((currState.time) + 1)) {
			currentPolicy.get(currState.rnum).add(new ArrayList<policyState>());
		}
		currentPolicy.get(currState.rnum).get(currState.time).add(currState);
	}
	private void addtoCurrentPolicy(policyState currState, int timeoffset)
	{
		int newtime = currState.time - timeoffset; 
		// TODO Auto-generated method stub
		while (currentPolicy.get(currState.rnum).size() < ((newtime) + 1)) {
			currentPolicy.get(currState.rnum).add(new ArrayList<policyState>());
		}
		currentPolicy.get(currState.rnum).get(newtime).add(currState);
	}
	public static void main(String[] args){
	 PriorityQueue<policyState> tq = new PriorityQueue<policyState>(); 
	 for(int i = 2; i<5; i++) {
	 policyState ps = new policyState(); 
	 ps.cumulativeProb=(1.0)/(double)i; 
	 tq.add(ps); 
	 }
	 for(int i = 10; i>=5; i--)
	 {
		 policyState ps = new policyState(); 
		 ps.cumulativeProb=(1.0)/(double)i; 
		 tq.add(ps); 
	 }
	 while(!tq.isEmpty())
	 {
		 policyState ps = tq.remove(); 
		 System.out.println(ps);
	 }
	 
	}
};
