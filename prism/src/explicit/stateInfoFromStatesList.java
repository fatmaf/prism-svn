/**
 * 
 */
package explicit;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import parser.State;

/**
 * @author fatma
 *
 */
public class stateInfoFromStatesList//a class I can use to manipulate the states list because otherwise it gets so messy 

{
int BADVALUE = -2;
	List<State> states;
	int numVar;
	int robotState;
	int mdpState;
	int failState=-1;//16;//11;//7;//11;//16;//9; 

	public stateInfoFromStatesList(MDPSimple sumprod)
	{
		List<State> states_ = sumprod.statesList;
		int numVar_ = sumprod.varList.getNumVars();
		states = states_;
		numVar = numVar_;
		robotState = 0;
		mdpState = numVar - 1;
	}
	public stateInfoFromStatesList(List<State> statesList)
	{
		List<State> states_ = statesList;
		int numVar_ =0;
		if (states_.size()>0)
		{
			State ts = states_.get(0);
			numVar_ = ts.varValues.length;
		}

		states = states_;
		numVar = numVar_;
		robotState = 0;
		mdpState = numVar - 1;
	}
	public int[] getMDPStateMappingFromStates(int[] prevStatesMapping)
	{
		int[] mappedStates = new int[states.size()]; 
		Arrays.fill(mappedStates, BADVALUE);
		for (int s=0; s<states.size(); s++)
		{
			mappedStates[s]=getMDPState(s);
		}
		
		//reverse the mapping to make it easy for us 
		HashMap<Integer,Integer> reverseMap = null;
		if (prevStatesMapping !=null)
		{
			reverseMap = new HashMap<Integer,Integer>();
			for(int s=0; s<prevStatesMapping.length; s++)
			{
				reverseMap.put(prevStatesMapping[s], s);
			}
			for(int s=0; s<mappedStates.length; s++)
			{
				mappedStates[s]=reverseMap.get(mappedStates[s]);
			}
		}

		
		return mappedStates;
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
	private Object[] getNewStatePreservingRobotMDPOnlyAndChangedIndices(State s1,List<Entry<Integer, Integer>> changedIndices,boolean setAllOthersToZero) {
		Object x1v[] = s1.varValues;
		Object x1vp[] = x1v.clone();
		if (setAllOthersToZero){
		for(int v=robotState+1; v<mdpState; v++)
		{
			x1vp[v] = 0; 
		}
		}
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
			Object[] newstate = getNewState(states.get(cs), combinations.get(comb));//getNewStatePreservingRobotMDPOnlyAndChangedIndices(states.get(cs),combinations.get(comb),true);
			//getNewState(states.get(cs), combinations.get(comb));
			
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
		for (int v = robotState+1; v < mdpState; v++) //because I know this 
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
		int res = BADVALUE;
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
	public int getMDPState(int state)
	{
		//int robotState = 0; 
		return getIndValFromState(state, mdpState);
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
		int res = BADVALUE;
		Object s1v[] = s1.varValues.clone();
		Object s2v[] = s2.varValues;
		for (int i = 0; i < statesToKeeps2.length; i++) {
			s1v[statesToKeeps2[i]] = s2v[statesToKeeps2[i]];
		}
		res = getExactlyTheSameState(s1v);
		return res;
	}
	//get a matching robot state 
	private int getRobotStateFromMDPState(State s1, int robotNum)
	{
		int res = BADVALUE;
		Object s1v[] = s1.varValues.clone();
		s1v[robotState] = robotNum; 
		res = getExactlyTheSameState(s1v);
		return res;
	}
	//get a matching robot state 
	public int getRobotStateFromMDPState(int s1, int robotNum)
	{
		return getRobotStateFromMDPState(states.get(s1),robotNum);
	}

	public int getExactlyTheSameState(Object s1v[])
	{
		int res = BADVALUE;
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