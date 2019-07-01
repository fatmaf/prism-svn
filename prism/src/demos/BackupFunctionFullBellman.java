package demos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import demos.MultiAgentProductModelGenerator.RewardCalculation;
import parser.State;
import prism.PrismException;

public class BackupFunctionFullBellman implements BackupFunction
{
	ArrayList<Objectives> tieBreakingOrder;
	MultiAgentProductModelGenerator maProdModGen;

	public BackupFunctionFullBellman(MultiAgentProductModelGenerator maProdModGen, ArrayList<Objectives> tieBreakingOrder)
	{
		this.maProdModGen = maProdModGen;
		this.tieBreakingOrder = tieBreakingOrder;
	}

	@Override
	public void backup(ChanceNode n) throws PrismException
	{
		//backup a chance node 
		//tie breaking oder doesnt matter since this is state action 
		//so we'll just back up everything 
		State s = n.getState();
		Object a = n.getAction();
		ArrayList<DecisionNode> dns = n.getChildren();
		double costHere = maProdModGen.getStateActionReward(s, a, "time", RewardCalculation.MAX);
		double progRew = maProdModGen.getProgressionReward(s, a);
		Bounds prob = new Bounds();
		Bounds prog = new Bounds();
		int numRews = dns.get(0).getRews().size();
		ArrayList<Bounds> costs = new ArrayList<Bounds>();
		Bounds cost;
		for (DecisionNode dn : dns) {
			prob = prob.add(dn.getProbValueTimesTranProb(n));
			prog = prog.add(dn.getProgValueTimesTranProb(n));

			for (int i = 0; i < numRews; i++) {
				if (costs.size() <= i) {
					cost = new Bounds();
					costs.add(cost);
				}

				cost = costs.get(i);
				cost = cost.add(dn.getRewValueTimesTranProb(i,n));
				costs.set(i, cost);

			}

		}
		prog = prog.add(progRew);
		for (int i = 0; i < numRews; i++) {
			cost = costs.get(i);
			cost = cost.add(costHere);
			costs.set(i, cost);

		}

		n.updateBounds(prob, prog, costs);

	}

	@Override
	public void backup(DecisionNode n) throws PrismException
	{
		// TODO Auto-generated method stub

		//go over all children get min or max 
		//has to be done in tie breaking order 
		Entry<Object, ArrayList<Bounds>> upperBoundUpdate = Helper.updatedBoundsAndAction(n, true, tieBreakingOrder);
		Entry<Object, ArrayList<Bounds>> lowerBoundUpdate = Helper.updatedBoundsAndAction(n,  false, tieBreakingOrder);
		if (upperBoundUpdate.getValue().size() == 0) {
			throw new PrismException("Ye kya hua?" + n.toString());
		}
		if (lowerBoundUpdate.getValue().size() == 0) {
			throw new PrismException("Ye kya hua?"+ n.toString());
		}
		for (int i = 0; i < tieBreakingOrder.size(); i++) {
			Objectives obj = tieBreakingOrder.get(i);

			switch (obj) {
			case Probability:
				n.setProbValue(new Bounds(upperBoundUpdate.getValue().get(i).upper, lowerBoundUpdate.getValue().get(i).lower));
				break;
			case Progression:
				n.setProg(new Bounds(upperBoundUpdate.getValue().get(i).upper, lowerBoundUpdate.getValue().get(i).lower));
				break;
			case Cost:
				n.setRew(new Bounds(upperBoundUpdate.getValue().get(i).upper, lowerBoundUpdate.getValue().get(i).lower), 0);
				break;
			default:
				throw new PrismException("Unimplemented Bounds update");
			}
		}
		//now just update 

	}

	@Override
	public double residual(DecisionNode n, boolean upperBound, float epsilon) throws PrismException
	{
		if (n.isGoal || n.isDeadend)
			return 0.0;
		Entry<Object, ArrayList<Bounds>> boundUpdate = Helper.updatedBoundsAndAction(n, upperBound, tieBreakingOrder);
		//		Entry<Object, ArrayList<Bounds>> lowerBoundUpdate = Helper.updatedBoundsAndAction(n, false, tieBreakingOrder);
		double res = 0;
		for (int i = 0; i < tieBreakingOrder.size(); i++) {
			Objectives obj = tieBreakingOrder.get(i);
			Bounds b = boundUpdate.getValue().get(i);
			Bounds nb;
			switch (obj) {

			case Probability:
				nb = n.getProbValue();
				break;
			case Progression:
				nb = n.getProg();
				break;
			case Cost:
				nb = n.getRew(0);
				break;
			default:
				throw new PrismException("Unimplemented");
			}
			if (upperBound)
				res = Math.abs(nb.subtractUpper(b));
			else
				res = Math.abs(nb.subtractLower(b));
			if (res > epsilon) {
				break;
			}
		}
		// TODO Auto-generated method stub
		return res;
	}

}
