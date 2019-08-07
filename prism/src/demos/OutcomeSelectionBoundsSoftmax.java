package demos;

import java.util.ArrayList;
import java.util.Random;

import parser.State;

//selects outcomes based on difference in prob bounds 

public class OutcomeSelectionBoundsSoftmax implements OutcomeSelection {
	Random rgen = new Random();
	int maxTrialLen = -1; 
	int atomicTrialLen = -1; 
	public OutcomeSelectionBoundsSoftmax(int maxTLen)
	{
		this.maxTrialLen = maxTLen;
		this.atomicTrialLen = this.maxTrialLen;
	}
	public void trialEnded()
	{
		maxTrialLen += atomicTrialLen;
	}
	@Override
	public State selectOutcome(State s, Object a) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public ArrayList<DecisionNode> selectOutcome(ChanceNode c) {
		// for now lets just do prob diffs
		ArrayList<Double> probdiffs = new ArrayList<Double>();
		double diffHere = 0;
		double diffSum = 0;
		ArrayList<Double> temperatures = new ArrayList<Double>(); 
		
		for (DecisionNode dn : c.getChildren()) {
			diffHere = dn.getProbValueTimesTranProb(c).diff();
			probdiffs.add(diffHere);
			diffSum += diffHere;
			temperatures.add((double)dn.numVisits/(double)this.maxTrialLen);
		}
		// if the probability bounds didn't give us much
		// lets look at the other ones
		if (diffSum == 0) {
			probdiffs.clear();
			for (DecisionNode dn : c.getChildren()) {
				diffHere = dn.getProgValueTimesTranProb(c).diff();
				probdiffs.add(diffHere);
				diffSum += diffHere;
			}
			if (diffSum == 0) {
				for (int rew = 0; rew < c.getMaxRews(); rew++) {
					// if we still didn't get much
					// lets look at the costs
					probdiffs.clear();
					for (DecisionNode dn : c.getChildren()) {
						diffHere = dn.getRewValueTimesTranProb(rew, c).diff();
						probdiffs.add(diffHere);
						diffSum += diffHere;
					}
					if (diffSum != 0) {
						break;
					}
				}
			}
		}
		//redoing diffSum
		diffSum = 0; 
		for(int i = 0; i<probdiffs.size(); i++)
		{
			probdiffs.set(i, Math.log(probdiffs.get(i)/temperatures.get(i))); 
			diffSum += probdiffs.get(i);
		}
		if (diffSum != 1) {
			for (int i = 0; i < probdiffs.size(); i++) {
				probdiffs.set(i, probdiffs.get(i) / diffSum);
			}
		}
		double randProb = rgen.nextDouble();
		diffSum = 0;
		DecisionNode dn = null;
		for (int i = 0; i < probdiffs.size(); i++) {
			diffSum += probdiffs.get(i);
			if (diffSum >= randProb) {
				dn = c.getChildren().get(i);
				break;
			}
		}
		ArrayList<DecisionNode> toret = new ArrayList<DecisionNode>();
		toret.add(dn);
		return toret;
	}

}
