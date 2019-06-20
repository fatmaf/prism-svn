package demos;

import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Map.Entry;

import prism.PrismException;

public class Helper
{
	public static boolean compareObjectives(Objectives obj, double newv, double oldv)
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

	public static boolean equalObjectives(Objectives obj, double newv, double oldv)
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
	
	
	public static Entry<Object,ArrayList<Bounds>> updatedBoundsAndAction(DecisionNode d, boolean upperBound,
			ArrayList<Objectives> tieBreakingOrder) throws PrismException
	{
		//for each objective we need a bound thing 
		ArrayList<Bounds> bestQ = new ArrayList<Bounds>();

		Object bestA = null;
		if(d.isLeafNode())
			return new AbstractMap.SimpleEntry<Object,ArrayList<Bounds>>(bestA,bestQ);
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
						if (Helper.compareObjectives(tieBreakingOrder.get(i), here.upper, bestQ.get(i).upper)) {
							//then save everything from here 
							bestQ.set(i, here);
							bestA = a;
							saveTheRest = true;
							stopI = i;
							break;
						} else {
							if (Helper.equalObjectives(tieBreakingOrder.get(i), here.upper, bestQ.get(i).upper)) {
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
						if (Helper.compareObjectives(tieBreakingOrder.get(i), here.lower, bestQ.get(i).lower)) {
							//then save everything from here 
							bestQ.set(i, here);
							bestA = a;
							saveTheRest = true;
							stopI = i;
							break;
						} else {
							if (Helper.equalObjectives(tieBreakingOrder.get(i), here.lower, bestQ.get(i).lower)) {
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
		Entry<Object,ArrayList<Bounds>> res = new AbstractMap.SimpleEntry<Object,ArrayList<Bounds>>(bestA,bestQ);
		// TODO Auto-generated method stub
		return res;
	}


}
