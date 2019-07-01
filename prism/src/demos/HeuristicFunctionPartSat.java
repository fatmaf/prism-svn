package demos;

import java.util.ArrayList;

import demos.MultiAgentProductModelGenerator.RewardCalculation;
import parser.State;
import prism.PrismException;

public class HeuristicFunctionPartSat implements HeuristicFunction
{
	MultiAgentProductModelGenerator maProdModGen;
	Bounds prob;
	Bounds prog;
	ArrayList<Bounds> costs;

	public HeuristicFunctionPartSat(MultiAgentProductModelGenerator jpmg)
	{
		maProdModGen = jpmg;
	}

	@Override
	public void calculateBounds(State s) throws PrismException
	{
		double maxP = 1.0; 

		double maxPr = 1.0; 
	
		if(maProdModGen.isDeadend(s))
		{	maxPr = 0.0; 
		maxP = 0.0;
		}
		if(maProdModGen.isGoal(s))
		{
			maxPr = 0.0; 
			maxP = 1.0; 
		}
		prob = new Bounds(maxP, maProdModGen.getSolutionValue(s, Objectives.Probability, RewardCalculation.MAX));
		prog = new Bounds(maxPr, maProdModGen.getSolutionValue(s, Objectives.Progression, RewardCalculation.MAX));

		Bounds cost = new Bounds(maProdModGen.getSolutionValue(s, Objectives.Cost, RewardCalculation.SUM), 0.0);

		costs = new ArrayList<Bounds>();
		costs.add(cost);

	}

	@Override
	public void calculateBounds(State s, Object a, ArrayList<DecisionNode> dns,THTSNode parent) throws PrismException
	{
		double costHere = maProdModGen.getStateActionReward(s, a, "time", RewardCalculation.MAX);
		double progRew = maProdModGen.getProgressionReward(s, a);
		prob = new Bounds();
		prog = new Bounds();
		int numRews = dns.get(0).getRews().size();
		 costs = new ArrayList<Bounds>();
		Bounds cost;
		for (DecisionNode dn : dns) {
			prob = prob.add(dn.getProbValueTimesTranProb(parent));
			prog = prog.add(dn.getProgValueTimesTranProb(parent));

			for (int i = 0; i < numRews; i++) {
				if (costs.size() <= i) {
					cost = new Bounds();
					costs.add(cost);
				} 
				
					cost = costs.get(i);
				cost = cost.add(dn.getRewValueTimesTranProb(i,parent));
				costs.set(i, cost);

			}

		}
		prog = prog.add(progRew);
		for (int i = 0; i < numRews; i++) {
			cost = costs.get(i);
			cost = cost.add(costHere);
			costs.set(i, cost);

		}

	}

	@Override
	public Bounds getProbabilityBounds()
	{
		// TODO Auto-generated method stub
		return prob;
	}

	@Override
	public Bounds getProgressionBounds()
	{
		// TODO Auto-generated method stub
		return prog;
	}

	@Override
	public ArrayList<Bounds> getRewardBounds()
	{
		// TODO Auto-generated method stub
		return costs;
	}

}
