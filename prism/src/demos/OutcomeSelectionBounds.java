package demos;

import java.util.ArrayList;
import java.util.Random;

import parser.State;

//selects outcomes based on difference in prob bounds 

public class OutcomeSelectionBounds implements OutcomeSelection
{
	Random rgen = new Random();
	@Override
	public State selectOutcome(State s, Object a)
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public ArrayList<DecisionNode> selectOutcome(ChanceNode c)
	{
		//for now lets just do prob diffs 
		ArrayList<Double> probdiffs = new ArrayList<Double>(); 
		double diffHere = 0;
		double diffSum = 0; 
		for(DecisionNode dn: c.getChildren())
		{
			diffHere = dn.getProbValueTimesTranProb().diff(); 
			probdiffs.add(diffHere);
			diffSum+=diffHere; 
		}
		if(diffSum != 1) {
		for(int i = 0; i<probdiffs.size(); i++)
		{
			probdiffs.set(i, probdiffs.get(i)/diffSum);
		}
		}
		double randProb = rgen.nextDouble(); 
		diffSum = 0; 
		DecisionNode dn=null; 
		for(int i = 0; i<probdiffs.size(); i++)
		{
			diffSum+= probdiffs.get(i); 
			if(diffSum>=randProb)
			{
				dn = c.getChildren().get(i); 
				break; 
			}
		}
		ArrayList<DecisionNode> toret = new ArrayList<DecisionNode>();
		toret.add(dn);
		return toret;
	}

}
