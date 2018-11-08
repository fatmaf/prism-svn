package demos;

import java.util.BitSet;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.Model;
import explicit.ProbModelChecker;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MDPRewardsSimple;
import parser.ast.Expression;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.RewardStruct;
import prism.PrismException;
import prism.PrismLog;

public class DAInfo {
	/**
	 * 
	 */
	// private final STAPU stapu;
	// BitSet acceptingStates;
	public BitSet productAcceptingStates;
	public BitSet essentialStates;
	boolean isSafeExpr;
	DA<BitSet, ? extends AcceptanceOmega> da;
	Expression daExpr;
	ExpressionReward daExprRew = null;
	Vector<BitSet> labelBS;
	public MDPRewardsSimple costsModel = null;
	PrismLog mainLog;

	public DAInfo(DAInfo other) {
		if (other.productAcceptingStates != null) {
			this.productAcceptingStates = (BitSet) other.productAcceptingStates.clone();
		}
		if (other.essentialStates != null) {
			this.essentialStates = (BitSet) other.essentialStates.clone();
		}
		this.isSafeExpr = other.isSafeExpr;
		if (other.da != null) {
			this.da = other.da;
		}
		this.daExpr = other.daExpr;
		this.daExprRew = other.daExprRew;
		if (other.labelBS != null) {
			this.labelBS = new Vector<BitSet>();
			for (int labels = 0; labels < other.labelBS.size(); labels++) {
				this.labelBS.add((BitSet) other.labelBS.get(labels).clone());
			}
		}
		if (other.costsModel != null) {
			this.costsModel = new MDPRewardsSimple(other.costsModel);
		}
		this.mainLog = other.mainLog;
	}

	public DAInfo(PrismLog log, Expression expr) {
		// this.stapu = stapu;
		initializeDAInfo(expr);
		mainLog = log;

	}

	public DAInfo(PrismLog log, Expression expr, boolean hasReward) {
		// this.stapu = stapu;
		mainLog = log;
		if (hasReward) {
			initializeDAInfoReward(expr);
		} else
			initializeDAInfo(expr);
	}

	public <M extends Model> LTLProduct<M> constructDAandProductModel(LTLModelChecker mcLTL, ProbModelChecker mcProb,
			ModulesFile modulesFile, AcceptanceType[] accType, M model, BitSet statesOfInterest, boolean allStatesInDFA)
			throws PrismException {
		labelBS = new Vector<BitSet>();
		mcProb.setModulesFile(modulesFile);
		mcProb.setConstantValues(modulesFile.getConstantValues());
		da = mcLTL.constructDAForLTLFormula(mcProb, model, daExpr, labelBS, accType);
		if (!(da.getAcceptance() instanceof AcceptanceReach)) {
			mainLog.println("\nAutomaton is not a DFA... ");
			throw new PrismException("Automaton is not a DFA "+daExpr.toString());
		}
		
		// else {
		// BitSet acceptingStates = ((AcceptanceReach)
		// da.getAcceptance()).getGoalStates();
		// }

		LTLProduct<M> product = mcLTL.constructProductModel(da, model, labelBS, statesOfInterest, allStatesInDFA);

		// update labelBS
		Vector<BitSet> newLabelBS = new Vector<BitSet>();
		for (int bs = 0; bs < labelBS.size(); bs++)
			newLabelBS.add(product.liftFromAutomaton(labelBS.get(bs)));
		productAcceptingStates = ((AcceptanceReach) product.getAcceptance()).getGoalStates();

		// rewards
		if (daExprRew != null) {

			RewardStruct costStruct = (daExprRew).getRewardStructByIndexObject(modulesFile,
					modulesFile.getConstantValues());
			// commenting this out because its giving the error Error: Could not evaluate
			// constant ("failstate", line 166, column 20).
			// we know this is because I'm not intializing this properly cuz i'm lazy and
			// prism code is confusing
			// but its okay we can do this later
			costsModel = (MDPRewardsSimple) mcProb.constructRewards(model, costStruct);
		}
		mainLog.println("Product Model Initial States " + product.getProductModel().getFirstInitialState() + " "
				+ product.getProductModel().getStatesList().get(product.getProductModel().getFirstInitialState()));
		return product;
	}

	public BitSet getEssentialStates(MDP prod) {
		// check if an accepting state is connected to a non accepting state
		// basically I want a bitset of all the edges and and it with !(accepting
		// states), if its not null I know

		BitSet accs = productAcceptingStates; // the accepting states for this da only
		int numstates = prod.getNumStates(); // get the number of states

		BitSet accsCopy = (BitSet) accs.clone(); // make a copy
		essentialStates = new BitSet(numstates); // new essential states
		int setbit = -1;
		for (int s = 0; s < numstates; s++) { // loop over all states
			if (!accs.get(s)) // if not an accepting state
			{
				setbit = accsCopy.nextSetBit(0); // get the first accepting state

				while (setbit != -1) { // loop over all accepting states
					if (prod.isSuccessor(s, setbit)) { // check if this accepting state is a successor of s (which is a
														// non accepting state)
						essentialStates.set(setbit); // if so then its an essential state
						accsCopy.clear(setbit); // remove this accepting state from copy [do we need to do this ??? -
												// yes cuz its checked ?

					}

					setbit = accsCopy.nextSetBit(setbit + 1);
				}
			}
		}
		return essentialStates;

	}

	private void initializeDAInfo(Expression expr) {
		daExpr = expr;
		isSafeExpr = !Expression.isCoSafeLTLSyntactic(daExpr, true);
		if (isSafeExpr) {
			daExpr = Expression.Not(daExpr);
		}

	}

	private void initializeDAInfoReward(Expression expr) {
		daExprRew = (ExpressionReward) expr;
		daExpr = ((ExpressionQuant) expr).getExpression();
		isSafeExpr = !Expression.isCoSafeLTLSyntactic(daExpr, true);
		if (isSafeExpr) {
			daExpr = Expression.Not(daExpr);
		}

	}

	public <M extends Model> void updateStateNumbers(LTLProduct<M> product) {
		int numStates = product.getProductModel().getNumStates();
		BitSet newProductAcceptingStates = new BitSet();
		BitSet newEssentialStates = new BitSet();
		Vector<BitSet> newLabelBS = new Vector<BitSet>();
		for (int bs = 0; bs < labelBS.size(); bs++)
			newLabelBS.add(new BitSet());
		for (int s = 0; s < numStates; s++) {
			int oldstate = product.getModelState(s);
			if (productAcceptingStates.get(oldstate)) {
				newProductAcceptingStates.set(s);
			}
			if (s ==426)
			{
				mainLog.println("check this bit");
			}
			if (essentialStates.get(oldstate)) {
				newEssentialStates.set(s);
			}
			for (int bs = 0; bs < labelBS.size(); bs++) {
				if (labelBS.get(bs).get(oldstate)) {
					newLabelBS.get(bs).set(s);
				}
			}
		}
		productAcceptingStates = newProductAcceptingStates;
		essentialStates = newEssentialStates;
		labelBS = newLabelBS;

	}

}