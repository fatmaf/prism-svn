package demos;

import java.util.ArrayList;

import parser.State;
import prism.PrismException;

public interface HeuristicFunction
{


	
	public void calculateBounds(State s) throws PrismException; 
	public void calculateBounds(State s, Object a,ArrayList<DecisionNode> dn) throws PrismException;
	public Bounds getProbabilityBounds(); 
	public Bounds getProgressionBounds(); 
	public ArrayList<Bounds> getRewardBounds(); 
	
}
