package demos;

import java.util.List;
import java.util.Random;

import explicit.MDPSimple;
import parser.State;
import prism.PrismException;

/*
 * Randomly selections actions from a given MDP 
 */
public class RandomActionSelection implements ActionSelection
{
	MDPSimple mdp; 
	Random rgen;

	public RandomActionSelection(MDPSimple mdp)
	{
		this.mdp = mdp; 
		rgen= new Random();
	}
	@Override
	public Object selectAction(State s) throws PrismException
	{
		//get the integer 
		List<State> statesList = mdp.getStatesList(); 
		int index = -1; 
		State ss; 
		for(int i = 0; i<statesList.size(); i++)
		{
			ss = statesList.get(i); 
			if(ss.compareTo(s)==0)
			{
				index = i; 
				break; 
			}
		}
		if(index == -1)
			throw new PrismException("Cant match State"); 
		int actionIndex = selectAction(index);
		Object action = mdp.getAction(index, actionIndex);
		// TODO Auto-generated method stub
		return action;
	}

	@Override
	public int selectAction(int s) throws PrismException
	{
		//randomly select an action 
		
		//get all the actions 
		int numActions = mdp.getNumChoices(s); 
		//choose one at random 
		int actIndex = rgen.nextInt(numActions);
		// TODO Auto-generated method stub
		return actIndex;
	}

	
}
