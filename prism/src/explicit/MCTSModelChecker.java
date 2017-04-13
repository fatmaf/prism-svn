//==============================================================================
//	
//	Copyright (c) 2013-
//	Authors:
//	* Dave Parker <david.parker@comlab.ox.ac.uk> (University of Oxford)
//	
//------------------------------------------------------------------------------
//	
//	This file is part of PRISM.
//	
//	PRISM is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.
//	
//	PRISM is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//	
//	You should have received a copy of the GNU General Public License
//	along with PRISM; if not, write to the Free Software Foundation,
//	Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//	
//==============================================================================

package explicit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import parser.Values;
import parser.ast.Expression;
import parser.ast.ExpressionProb;
import parser.ast.ExpressionReward;
import parser.ast.ExpressionTemporal;
import parser.ast.LabelList;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import parser.ast.RelOp;
import parser.ast.RewardStruct;
import prism.ModelGenerator;
import prism.PrismComponent;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismNotSupportedException;
import prism.ProductModelGenerator;
import prism.Result;
import simulator.ModulesFileModelGenerator;

/**
 * MDP model checker based on MCTS
 */
public class MCTSModelChecker extends PrismComponent
{
	// Model file
	private ModulesFile modulesFile;
	// Properties file
	private PropertiesFile propertiesFile;
	// Constants from model
	private Values constantValues;
	// Labels from the model
	private LabelList labelListModel;
	// Labels from the property file
	private LabelList labelListProp;
	
	/**
	 * Constructor.
	 */
	public MCTSModelChecker(PrismComponent parent, ModulesFile modulesFile, PropertiesFile propertiesFile) throws PrismException
	{
		super(parent);
		this.modulesFile = modulesFile;
		this.propertiesFile = propertiesFile;

		// Get combined constant values from model/properties
		constantValues = new Values();
		constantValues.addValues(modulesFile.getConstantValues());
		if (propertiesFile != null)
			constantValues.addValues(propertiesFile.getConstantValues());
		this.labelListModel = modulesFile.getLabelList();
		this.labelListProp = propertiesFile.getLabelList();
	}

	/**
	 * Model check a property.
	 */
	public Result check(Expression expr) throws PrismException
	{
		Result res;
		String resultString;
		long timer;

		// Starting model checking
		timer = System.currentTimeMillis();

		// Do model checking
		res = checkExpression(expr);

		// Model checking complete
		timer = System.currentTimeMillis() - timer;
		mainLog.println("\nModel checking completed in " + (timer / 1000.0) + " secs.");

		// Print result to log
		resultString = "Result";
		if (!("Result".equals(expr.getResultName())))
			resultString += " (" + expr.getResultName().toLowerCase() + ")";
		resultString += ": " + res;
		mainLog.print("\n" + resultString + "\n");

		// Return result
		return res;
	}

	/**
	 * Model check an expression (used recursively).
	 */
	private Result checkExpression(Expression expr) throws PrismException
	{
		Result res;

		// Current range of supported properties is quite limited...
		if (expr instanceof ExpressionReward)
			res = checkExpressionReward((ExpressionReward) expr);
		else
			throw new PrismNotSupportedException("MCTS not yet supported for this operator");

		return res;
	}
	/**
	 * Model check an R operator.
	 */
	private Result checkExpressionReward(ExpressionReward expr) throws PrismException
	{
		System.out.println("LTL"  + expr.getExpression());
		LTLModelChecker ltlMC = new LTLModelChecker(this);
		List<Expression> labelExprs = new ArrayList<Expression>();
		AcceptanceType[] allowedAcceptance = {
				AcceptanceType.RABIN,
				AcceptanceType.REACH
		};
		DA<BitSet,? extends AcceptanceOmega> da = ltlMC.constructExpressionDAForLTLFormula(expr.getExpression(), labelExprs, allowedAcceptance);
		ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, this);
		ProductModelGenerator prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
		
		
		
		DTMCModelChecker mcDTMC;
		
		UCT uct = new UCT(this, prodModelGen, 100);		
		RewardStruct rewStruct = expr.getRewardStructByIndexObject(modulesFile, constantValues);
		uct.setRewardStruct(rewStruct);
		uct.setConstantValues(constantValues);
		DTMC dtmc = uct.buildDTMC(uct.search());
		dtmc.exportToDotFile("/home/bruno/Desktop/dtmc.dot");
		// Create a DTMC model checker (for solving policies)
		mcDTMC = new DTMCModelChecker(null);
		//mcDTMC.inheritSettings(this);
		mcDTMC.setLog(new PrismDevNullLog());
		
		BitSet no, acc;
		no = new BitSet(dtmc.getNumStates());
		acc = new BitSet(dtmc.getNumStates());
		acc = findAccStates(dtmc, da);
		ModelCheckerResult mcCheckProb = mcDTMC.computeReachProbsGaussSeidel(dtmc, no, acc, null, null);
		//mcDTMC.check(dtmc, expr);
		//System.out.println("AHAH#" + expr.getExpression());
		//Result mcCheckProb = mcDTMC.check(dtmc, new ExpressionProb(expr.getExpression(), RelOp.MAX.toString(), null));
		
		mainLog.println("\nThe reward is " + mcCheckProb.soln[0]);
		return new Result(new Double(1));//uct.getReward()
	}
	
	public BitSet findAccStates(DTMC dtmc, DA<BitSet,? extends AcceptanceOmega> da) {
		int i, n;
		BitSet daAcc, acc;
		if (da.getAcceptance() instanceof AcceptanceReach) {
			mainLog.println("\nSkipping BSCC computation since acceptance is defined via goal states...");
			daAcc = ((AcceptanceReach)da.getAcceptance()).getGoalStates();
		} else {
			mainLog.println("\n");
			AcceptanceRabin acceptance = (AcceptanceRabin)da.getAcceptance();
			daAcc = acceptance.get(0).getK();
		}
		
		n = dtmc.getNumStates();
		acc = new BitSet(n);
		for (i = 0; i < n; i++) {
			
		}
		
		return acc;
	}
	
	
	
	
	private Result checkExpressionReward2(ExpressionReward expr) throws PrismException
	{
		System.out.println("LTL"  + expr.getExpression());
		LTLModelChecker ltlMC = new LTLModelChecker(this);
		List<Expression> labelExprs = new ArrayList<Expression>();
		AcceptanceType[] allowedAcceptance = {
				AcceptanceType.RABIN,
				AcceptanceType.REACH
		};
		DA<BitSet,? extends AcceptanceOmega> da = ltlMC.constructExpressionDAForLTLFormula(expr.getExpression(), labelExprs, allowedAcceptance);
		ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, this);
		ProductModelGenerator prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
		
		// Build/print product (just to test)
		ConstructModel constructModel = new ConstructModel(this);
		Model prodModel = constructModel.constructModel(prodModelGen);
		prodModel.exportToPrismExplicitTra(mainLog);
		
		return new Result(new Double(1));
	}
}
