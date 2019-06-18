package demos;

import java.util.ArrayList;

import parser.State;

public interface OutcomeSelection
{
	public State selectOutcome(State s, Object a); 
	public ArrayList<DecisionNode> selectOutcome(ChanceNode c);
}
