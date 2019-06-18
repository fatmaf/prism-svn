package demos;

import java.util.ArrayList;

import parser.State;
import prism.PrismException;

/*
 * select action greedily 
 */
public class ActionSelectionGreedy implements ActionSelection
{

	ArrayList<Objectives> tieBreakingOrder;

	public ActionSelectionGreedy(ArrayList<Objectives> tieBreakingOrder)
	{
		this.tieBreakingOrder = tieBreakingOrder;
	}

	@Override
	public Object selectAction(State s) throws PrismException
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int selectAction(int s) throws PrismException
	{
		// TODO Auto-generated method stub
		return -1;
	}

	@Override
	public Object selectActionBound(DecisionNode d, boolean upperBound) throws PrismException
	{
		//for each objective we need a bound thing 
		ArrayList<Bounds> bestQ = new ArrayList<Bounds>();

		Object bestA = null;
		boolean saveTheRest = false;
		if (upperBound) {
			for (Object a : d.getChildren().keySet()) {
				saveTheRest = false;
				ChanceNode c = d.getChild(a);
				if (bestA == null) {
					for (int i = 0; i < tieBreakingOrder.size(); i++) {
						bestQ.add(c.getObjectiveBounds(tieBreakingOrder.get(i)));
					}
					bestA = a;
				} else {
					int stopI = 0;
					for (int i = 0; i < tieBreakingOrder.size(); i++) {
						Bounds here = c.getObjectiveBounds(tieBreakingOrder.get(i));
						if (compareObjectives(tieBreakingOrder.get(i), here.upper, bestQ.get(i).upper)) {
							//then save everything from here 
							bestQ.set(i, here);
							bestA = a;
							saveTheRest = true;
							stopI = i;
							break;
						} else {
							if (equalObjectives(tieBreakingOrder.get(i), here.upper, bestQ.get(i).upper)) {
								continue;
							} else {
								break;
							}
						}
					}
					if (saveTheRest) {
						for (int i = stopI; i < tieBreakingOrder.size(); i++) {
							Bounds here = c.getObjectiveBounds(tieBreakingOrder.get(i));
							bestQ.set(i, here);
						}
					}
				}

			}
		} else {
			for (Object a : d.getChildren().keySet()) {
				saveTheRest = false;
				ChanceNode c = d.getChild(a);
				if (bestA == null) {
					for (int i = 0; i < tieBreakingOrder.size(); i++) {
						bestQ.add(c.getObjectiveBounds(tieBreakingOrder.get(i)));
					}
					bestA = a;
				} else {
					int stopI = 0;
					for (int i = 0; i < tieBreakingOrder.size(); i++) {
						Bounds here = c.getObjectiveBounds(tieBreakingOrder.get(i));
						if (compareObjectives(tieBreakingOrder.get(i), here.lower, bestQ.get(i).lower)) {
							//then save everything from here 
							bestQ.set(i, here);
							bestA = a;
							saveTheRest = true;
							stopI = i;
							break;
						} else {
							if (equalObjectives(tieBreakingOrder.get(i), here.lower, bestQ.get(i).lower)) {
								continue;
							} else {
								break;
							}
						}
					}
					if (saveTheRest) {
						for (int i = stopI; i < tieBreakingOrder.size(); i++) {
							Bounds here = c.getObjectiveBounds(tieBreakingOrder.get(i));
							bestQ.set(i, here);
						}
					}
				}

			}

		}
		// TODO Auto-generated method stub
		return bestA;
	}

	boolean compareObjectives(Objectives obj, double newv, double oldv)
	{
		boolean toret = false;
		switch (obj) {
		case Probability:
			toret = newv > oldv;
			break;
		case Progression:
			toret = newv > oldv;
			break;
		case Cost:
			toret = newv < oldv;
			break;

		}
		return toret;
	}

	boolean equalObjectives(Objectives obj, double newv, double oldv)
	{
		boolean toret = false;
		switch (obj) {
		case Probability:
			toret = newv == oldv;
			break;
		case Progression:
			toret = newv == oldv;
			break;
		case Cost:
			toret = newv == oldv;
			break;

		}
		return toret;
	}

}
