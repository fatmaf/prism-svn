package demos;

import java.util.BitSet;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.LTLModelChecker;
import explicit.MDPModelChecker;
import explicit.Model;
import explicit.rewards.MDPRewardsSimple;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.RewardStruct;
import prism.PrismException;

public class AutomatonInformation
{

	//so for each automaton/expression 
	//we want to know the following: 
	//the exact expression probability/reward func included 
	//if its a reward function we'll just set a boolean 
	//a boolean for if its a safety expression 
	//just the expression like F x 

	Expression originalExpression = null;
	boolean isRewardExpr = false;
	boolean isSafetyExpr = false;
	Expression strippedExpression = null;
	AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
	int associatedIndexInMDP = -1; 
	DA<BitSet, ? extends AcceptanceOmega> da = null; 
	BitSet acceptingStates = null; 
	MDPRewardsSimple mdpCostsModel  = null; 
	int startState = -1; 
	String daLabel; 
	String originalLabel; 
	
	public AutomatonInformation(Expression expr)
	{
		originalExpression = expr;
		if (expr instanceof ExpressionReward) {
			isRewardExpr = true;
		}
		strippedExpression = ((ExpressionQuant) expr).getExpression();
		isSafetyExpr = !Expression.isCoSafeLTLSyntactic(strippedExpression, true);
		if (isSafetyExpr) {
			strippedExpression = Expression.Not(strippedExpression);
		}
	}
	
	public AutomatonInformation(AutomatonInformation other)
	{
		originalExpression = other.originalExpression;
		isRewardExpr = other.isRewardExpr;
		strippedExpression = other.strippedExpression;
		isSafetyExpr = other.isSafetyExpr;
	}
	
	public Vector<BitSet>  generateDA(LTLModelChecker ltlmc,MDPModelChecker mc,Model model) throws PrismException
	{
		Vector<BitSet> labelBS = new Vector<BitSet>();
		da = ltlmc.constructDAForLTLFormula(mc, model,strippedExpression, labelBS, allowedAcceptance);
		acceptingStates = (BitSet) ((AcceptanceReach) da.getAcceptance()).getGoalStates().clone();
		if(isRewardExpr)
		{
			RewardStruct costStructure = ((ExpressionReward)originalExpression).getRewardStructByIndexObject(mc.getModulesFile(),mc.getConstantValues()); 
			mdpCostsModel = (MDPRewardsSimple) mc.constructRewards(model, costStructure);
		}
		startState = da.getStartState(); 
		return labelBS;
	}

}
