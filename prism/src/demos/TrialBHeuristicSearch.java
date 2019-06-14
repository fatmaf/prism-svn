package demos;

import java.util.ArrayList;

import parser.State;
import prism.PrismException;

public class TrialBHeuristicSearch
{
	
	MultiAgentProductModelGenerator maProdModGen; 
	
	//literally going to follow what the thts paper describes 
	//so yeah 
	public TrialBHeuristicSearch(MultiAgentProductModelGenerator mapmg)
	{
		maProdModGen = mapmg;  
	}
	public void doTHTS()
	{
		THTSNode n0 = getRootNode(); 
	}
	THTSNode getRootNode() throws PrismException
	{
		//just get the inital states 
		ArrayList<State> initStates = maProdModGen.createInitialStateFromRobotInitStates();
		//now just use the first initial state 
		State s = initStates.get(0); 
		return createNodeFromState(s);	
		
	}
	THTSNode createNodeFromState(State s)
	{
		DecisionNode n = new DecisionNode(s); 
	}
	
}
