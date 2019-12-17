package demos;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;

import demos.XAIStateInformation.ValueLabel;
import parser.State;

public class XAIStateInformation
{
	public enum ValueLabel {
		cost, probability, progression, frequency
	}

	State state;
	int stateIndexInPath;
	int stateIndexInMDP;
	Object chosenAction;
	HashMap<Object, Integer> allActionIndicesInMDP;
	HashMap<Object, HashMap<ValueLabel, Double>> actionValues;
	HashMap<Object, HashMap<ValueLabel, Double>> actionValuesDifference;
	ArrayList<XAIStateInformation> parents = null; 

	/**
	 * @return the state
	 */
	public State getState()
	{
		return state;
	}

	/**
	 * @param state the state to set
	 */
	public void setState(State state)
	{
		this.state = state;
	}

	/**
	 * @return the stateIndexInPath
	 */
	public int getStateIndexInPath()
	{
		return stateIndexInPath;
	}

	/**
	 * @param stateIndexInPath the stateIndexInPath to set
	 */
	public void setStateIndexInPath(int stateIndexInPath)
	{
		this.stateIndexInPath = stateIndexInPath;
	}

	/**
	 * @return the stateIndexInMDP
	 */
	public int getStateIndexInMDP()
	{
		return stateIndexInMDP;
	}

	/**
	 * @param stateIndexInMDP the stateIndexInMDP to set
	 */
	public void setStateIndexInMDP(int stateIndexInMDP)
	{
		this.stateIndexInMDP = stateIndexInMDP;
	}

	/**
	 * @return the chosenAction
	 */
	public Object getChosenAction()
	{
		return chosenAction;
	}

	/**
	 * @param chosenAction the chosenAction to set
	 */
	public void setChosenAction(Object chosenAction)
	{
		this.chosenAction = chosenAction;
	}

	public void addValuesForState(Object action, HashMap<ValueLabel, Double> labels)
	{
		actionValues.put(action, labels);
	}

	XAIStateInformation(State s, int sI, Object a)
	{
		initialise();
		stateIndexInPath = sI;
		chosenAction = a;
		state = s;

	}

	void initialise()
	{
		state = null;
		stateIndexInPath = -1;
		chosenAction = null;
		allActionIndicesInMDP = new HashMap<Object, Integer>();
		actionValues = new HashMap<Object, HashMap<ValueLabel, Double>>();
		actionValuesDifference = new HashMap<Object, HashMap<ValueLabel, Double>>();
	}

	public boolean isEqual(int sI, Object a)
	{
		return (stateIndexInPath == sI && a.toString().contentEquals(chosenAction.toString()));
	}

	public boolean isEqual(State s, Object a)
	{
		if(a !=null && chosenAction !=null)
		return (s.compareTo(state) == 0 && a.toString().contentEquals(chosenAction.toString()));
		else
		{
			return ((chosenAction == a) && s.compareTo(state) == 0 );
		}
			
	}

	public boolean isSameState(State s)
	{
		return (s.compareTo(state) == 0);
	}

	public boolean isEqual(int sI, int aI)
	{
		return (sI == stateIndexInPath && allActionIndicesInMDP.get(chosenAction) == aI);
	}

	@Override
	public String toString()
	{
		String strToRet="[state=" + state + ", stateIndexInPath=" + stateIndexInPath + ", chosenAction=" + chosenAction + ", actionValues="
				+ actionValues + ", actionValuesDifference=" + actionValuesDifference ; 
		if(parents!=null)
		{
			String pars = "ps:[";
			for(int i = 0; i<parents.size(); i++)
				pars+=" "+ parents.get(i).toString();
			pars+="]";
			strToRet+=pars;
		}
		strToRet+= "]";
		return strToRet;
	}

	public void addParent(XAIStateInformation p)
	{
		if (parents == null)
		{
			parents = new ArrayList<XAIStateInformation>();
		}
		parents.add(p);
	}
}


