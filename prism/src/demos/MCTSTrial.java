package demos;

import java.util.ArrayList;

import parser.State;
import prism.PrismException;

public class MCTSTrial implements HeuristicFunction
{

	MultiAgentProductModelGenerator maProdModGen; 
	Bounds prob; 
	Bounds prog; 
	ArrayList<Bounds> costs; 
	
	public MCTSTrial(MultiAgentProductModelGenerator jpmg)
	{
		maProdModGen = jpmg; 
	}

	@Override
	public void calculateBounds(State s) throws PrismException
	{
		// TODO Auto-generated method stub
		//do the trial here please 
		//then just use these 

	}

	@Override
	public Bounds getProbabilityBounds()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Bounds getProgressionBounds()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public ArrayList<Bounds> getRewardBounds()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void calculateBounds(State s, Object a, ArrayList<DecisionNode> dn) throws PrismException
	{
		// TODO Auto-generated method stub
		
	}

}
