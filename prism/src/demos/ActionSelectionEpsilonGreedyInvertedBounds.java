package demos;

import java.util.ArrayList;
import java.util.Random;

import parser.State;
import prism.PrismException;

/*
 * select action greedily 
 */
public class ActionSelectionEpsilonGreedyInvertedBounds implements ActionSelection {

	public ArrayList<Objectives> tieBreakingOrder;
	Random rgen = new Random();

	public ActionSelectionEpsilonGreedyInvertedBounds(ArrayList<Objectives> tieBreakingOrder) {
		this.tieBreakingOrder = tieBreakingOrder;

	}

	@Override
	public Object selectAction(State s) throws PrismException {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int selectAction(int s) throws PrismException {
		// TODO Auto-generated method stub
		return -1;
	}

	@Override
	public Object selectActionBound(DecisionNode d, boolean upperBound) throws PrismException {
		// for each objective we need a bound thing
		// for now lets just do prob diffs
		ArrayList<Double> probdiffs = new ArrayList<Double>();
		double diffHere = 0;
		double diffSum = 0;
		Object boundsAction = null;
		if (!d.isLeafNode()) {
			for (Object a : d.getChildren().keySet()) {
				ChanceNode c = d.getChild(a);
				diffHere = c.getProbValue().diff();
				probdiffs.add(diffHere);
				diffSum += diffHere;
			}
			// if the probability bounds didn't give us much
			// lets look at the other ones
			if (diffSum == 0) {
				probdiffs.clear();
				for (Object a : d.getChildren().keySet()) {
					ChanceNode c = d.getChild(a);
					diffHere = c.getProg().diff();
					probdiffs.add(diffHere);
					diffSum += diffHere;
				}
				if (diffSum == 0) {
					for (int rew = 0; rew < d.getMaxRews(); rew++) {
						// if we still didn't get much
						// lets look at the costs
						probdiffs.clear();
						for (Object a : d.getChildren().keySet()) {
							ChanceNode c = d.getChild(a);
							diffHere = c.getRew(rew).diff();
							probdiffs.add(diffHere);
							diffSum += diffHere;
						}
						if (diffSum != 0) {
							break;
						}
					}
				}
			}
			if (diffSum != 1) {
				for (int i = 0; i < probdiffs.size(); i++) {
					probdiffs.set(i, probdiffs.get(i) / diffSum);
					probdiffs.set(i, 1 - probdiffs.get(i));
				}
			}
			double randProb = rgen.nextDouble();
			diffSum = 0;
			
			for (int i = 0; i < probdiffs.size(); i++) {
				diffSum += probdiffs.get(i);
				if (diffSum >= randProb) {
					boundsAction = d.getChildren().get(i);
					break;
				}
			}
		}
		ArrayList<Bounds> bestQ = new ArrayList<Bounds>();
		boolean printStuff = false;
		Object bestA = null;
		boolean saveTheRest = false;
		if (d.isLeafNode())
			return bestA;
		if (upperBound) {

			for (Object a : d.getChildren().keySet()) {

				saveTheRest = false;
				ChanceNode c = d.getChild(a);

				// for the softmax
//				double costHere = c.getObjectiveBounds(Objectives.Cost).lower; 
//				double temperature = (double)c.numVisits/(double)this.maxTrialLen; 
//				double expCost = Math.log((costHere+1)/temperature);
//				actionCosts.add(expCost); 
//				sumOfCosts += expCost;

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
							// then save everything from here
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

				// for the softmax
//				double costHere = c.getObjectiveBounds(Objectives.Cost).lower; 
//				double temperature = (double)c.numVisits/(double)this.maxTrialLen; 
//				double expCost = Math.log((costHere+1)/temperature);
//				actionCosts.add(expCost); 
//				sumOfCosts += expCost;

				if (printStuff) {
					System.out.println(c);
				}
				if (bestA == null) {
					for (int i = 0; i < tieBreakingOrder.size(); i++) {
						bestQ.add(c.getObjectiveBounds(tieBreakingOrder.get(i)));
					}
					bestA = a;
					if (printStuff) {
						System.out.println(bestA.toString());
					}
				} else {
					int stopI = 0;
					for (int i = 0; i < tieBreakingOrder.size(); i++) {
						Bounds here = c.getObjectiveBounds(tieBreakingOrder.get(i));
						if (Helper.compareObjectives(tieBreakingOrder.get(i), here.lower, bestQ.get(i).lower)) {
							// then save everything from here
							bestQ.set(i, here);
							bestA = a;
							saveTheRest = true;
							stopI = i;
							if (printStuff) {
								System.out.println(bestA.toString());
							}
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

		double epsilon = 0.5;
		if (rgen.nextDouble() > epsilon)
			bestA = boundsAction;

//		throw new PrismException("Not implemented.");

		return bestA;
	}

}
