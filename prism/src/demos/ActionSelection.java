package demos;

import parser.State;
import prism.PrismException;

public interface ActionSelection
{
	public Object selectAction(State s) throws PrismException;  
	public int selectAction(int s) throws PrismException; 
	public Object selectActionBound (DecisionNode d, boolean upperBound) throws PrismException;
	
}
