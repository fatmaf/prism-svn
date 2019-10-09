package demos;

import java.util.BitSet;

import parser.State;

/*
 * Storing state related information parent state, robot number, probability etc
 * just for ease of use
 */
public class StateExtended implements Comparable<StateExtended>
{
	protected int parentState = -1;
	protected int parentStateRobot = -1;
	protected int childState = -1;
	protected int childStateRobot = -1;
	protected double parentToChildTransitionProbability = -1;
	protected String actionInChildState = null;
	protected int choice = -1;
	public BitSet statesToAvoid = null;
	protected State childStateState = null;

	public StateExtended()
	{
		// dummy
	}

	public StateExtended(int ps, int psr, int cs, int csr, double prob, String a)
	{
		parentState = ps;
		parentStateRobot = psr;
		childState = cs;
		childStateRobot = csr;
		parentToChildTransitionProbability = prob;
		actionInChildState = a;
	}

	public StateExtended(int s, double prob, String a)
	{
		childState = s;
		parentToChildTransitionProbability = prob;
		actionInChildState = a;
	}

	public StateExtended(StateExtended other)
	{
		this.parentState = other.parentState;
		this.parentStateRobot = other.parentStateRobot;
		this.childState = other.childState;
		this.childStateRobot = other.childStateRobot;
		this.parentToChildTransitionProbability = other.parentToChildTransitionProbability;
		this.actionInChildState = other.actionInChildState;
	}

	public StateExtended(int initialState, double d)
	{
		childState = initialState;
		parentToChildTransitionProbability = d;

	}
	public StateExtended(State s, double d)
	{
		childStateState = s;
		parentToChildTransitionProbability = d;

	}
	public State getChildStateState()
	{
		return childStateState; 
	}
	@Override
	public int compareTo(StateExtended other)
	{
		double comp = this.parentToChildTransitionProbability - other.parentToChildTransitionProbability;
		int res = 0;
		if (comp > 0)
			res = -1;
		else {
			if (comp < 0) {
				res = 1;
			}
		}
		return res;
	}

	@Override
	public String toString()
	{
		String strtoret = "[";

		if (parentState != -1)
			strtoret += "ps=" + parentState;

		if (parentStateRobot != -1)
			strtoret += ", psRob=" + parentStateRobot;

		if (childState != -1)
			strtoret += ", cs=" + childState;

		if (childStateRobot != -1)
			strtoret += ", csRob=" + childStateRobot;

		if (parentToChildTransitionProbability > 0)
			strtoret += ", ps->csProb=" + parentToChildTransitionProbability;

		if (actionInChildState != null)
			strtoret += ", a=" + actionInChildState;
		strtoret += "]";

		return strtoret; 
	}

}