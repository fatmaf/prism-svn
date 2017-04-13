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

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.LTLModelChecker.LTLProduct;
import explicit.ProbModelChecker.SolnMethod;
import parser.State;
import parser.Values;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import parser.ast.Expression;
import parser.ast.ExpressionBinaryOp;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionLiteral;
import parser.ast.ExpressionProb;
import parser.ast.ExpressionReward;
import parser.ast.ExpressionTemporal;
import parser.ast.LabelList;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import parser.ast.RelOp;
import parser.ast.RewardStruct;
import prism.ModelGenerator;
import prism.Pair;
import prism.Prism;
import prism.PrismComponent;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismNotSupportedException;
import prism.PrismSettings;
import prism.ProductModelGenerator;
import prism.Result;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;

/**
 * MDP model checker based on MCTS
 */
public class MCTSModelChecker extends PrismComponent
{
	PrismComponent parent;
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
	
	AcceptanceType[] allowedAcceptance = {
			AcceptanceType.RABIN,
			AcceptanceType.REACH
	};
	
	/**
	 * Constructor.
	 */
	public MCTSModelChecker(PrismComponent parent, ModulesFile modulesFile, PropertiesFile propertiesFile) throws PrismException
	{
		super(parent);
		this.parent = parent;
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
		if (expr instanceof ExpressionProb)
			res = checkExpressionProb((ExpressionProb) expr);
		else
			throw new PrismNotSupportedException("MCTS not yet supported for this operator");

		return res;
	}
	
	private List<Entry<Double, Expression>> splitExpression(Expression expr) throws PrismLangException {
		List<Entry<Double, Expression>> weightedLtlSpecs = new ArrayList<Entry<Double, Expression>>();
		
		
		double weight;
		ExpressionBinaryOp main = (ExpressionBinaryOp)((ExpressionProb)expr).getExpression();
		Expression expr2 = ((ExpressionBinaryOp)main).getOperand2();
		Expression expr1 = ((ExpressionBinaryOp)main).getOperand1();
		Expression weightExpr;
		Expression ltlExpr;
		while (expr1 instanceof ExpressionBinaryOp) {
			weightExpr = ((ExpressionBinaryOp)expr2).getOperand1();
			weight = ((ExpressionLiteral)weightExpr).evaluateDouble();
			ltlExpr = ((ExpressionBinaryOp)expr2).getOperand2();
			Entry<Double, Expression> spec = new Pair<Double, Expression>(weight, ltlExpr);
			weightedLtlSpecs.add(spec);
			
			expr2 = ((ExpressionBinaryOp)expr1).getOperand2();
			expr1 = ((ExpressionBinaryOp)expr1).getOperand1();
			
		
		}
		weight = ((ExpressionLiteral)expr1).evaluateDouble();
		Entry<Double, Expression> spec = new Pair<Double, Expression>(weight, expr2);
		weightedLtlSpecs.add(spec);
		return weightedLtlSpecs;
	}
	
	

	
	


	
	
	private StateValues checkUCTResult(MDP result, Expression expr, DA<BitSet,? extends AcceptanceOmega> da, ProductModelGenerator prodModelGen, String modelName) throws PrismException {
		//DTMCModelChecker mcDTMC;
		ModelCheckerResult policyCheck = null;
			
		//mcDTMC = new DTMCModelChecker(null);
		//mcDTMC.setSettings(this.getSettings());
		//mcDTMC.setLog(new PrismDevNullLog());
		

		
		MDPModelChecker mcMdp = new MDPModelChecker(null);
		mcMdp.setSettings(new PrismSettings());
		mcMdp.setLog(new PrismDevNullLog());
		mcMdp.setGenStrat(true);
		mcMdp.setExportAdv(true);
		mcMdp.setExportAdvFilename("/home/bruno/Desktop/policy.adv");
		
		LTLModelChecker mcLtl = new LTLModelChecker(null);
		mcLtl.setSettings(new PrismSettings());
		mcLtl.setLog(new PrismDevNullLog());
		
		
		BitSet initState = new BitSet(result.getNumStates());
		for (int i = 0; i < result.getNumStates(); i++) {
			if (result.isInitialState(i)) {
				initState.set(i);
			}
		}

		result.findDeadlocks(true);
		LTLProduct<MDP> prod = mcLtl.constructProductMDP(mcMdp, (MDP)result, ((ExpressionProb)expr).getExpression(), initState, allowedAcceptance);
		//List<BitSet> ecs = mcLtl.findAcceptingECsForRabin(prod.getProductModel(), (AcceptanceRabin)prod.getAcceptance());
		BitSet ecs = mcLtl.findAcceptingECStatesForRabin(prod.getProductModel(), (AcceptanceRabin)prod.getAcceptance());
		
		StateValues policyCheckRes = null;
		
		prod.getProductModel().exportToDotFile("/home/bruno/Desktop/prod.dot");

		
				
		BitSet no, acc;
		no = new BitSet(result.getNumStates());
		acc = new BitSet(result.getNumStates());
		//acc = findAccStates(prod.getProductModel(), da, prodModelGen.getVarIndex("_da0"));
		acc = findAccStates(prod.getProductModel(), da, 0); //da variable is always index 0 on an explicit product
		
		
		
		int[] strat = new int[prod.getProductModel().getNumStates()];
		for (int i = 0; i < prod.getProductModel().getNumStates(); i++) {
			strat[i]=-1;
		}
		
		
		
		
		ModelCheckerResult res = mcMdp.computeReachProbs((MDP) prod.getProductModel(), ecs, false);

		
		
		mcMdp.prob1(prod.getProductModel(), ecs, acc, false, strat);
		
		for (int i = 0; i < prod.getProductModel().getNumStates(); i++) {
			if(acc.get(i)) {
				for (int j = 0; j < prod.getProductModel().getNumChoices(i); j++) {
					boolean goodSucc = true;
					Iterator<Integer> succs = prod.getProductModel().getSuccessorsIterator(i, j);
					while (succs.hasNext()) {
						if (!(ecs.get(succs.next()))) {
							goodSucc = false;
						}
					}
					if (goodSucc) {
						strat[i] = j;
						break;
					}
				}
		 	} else {
				int choice = ((MDStrategy)res.strat).getChoiceIndex(i);
				if (choice >= 0) {
					strat[i] = choice;
				}
		 	}
		}
		
		
		//mcMdp.restrictStrategyToReachableStates((MDP)result, strat);
		DTMC policy = new DTMCFromMDPMemorylessAdversary(prod.getProductModel(), strat);
		policy.exportToDotFile("/home/bruno/Desktop/Dropbox/icaps/res/policy" + modelName + ".dot");
		
		if(result instanceof MDP) {		
			//policyCheck = mcMdp.computeReachProbs((MDP)model, acc, false);
			//model = new MDPSparse((MDPSimple)model);
			//Result policyCheckRes = mcMdp.check((MDP)result, expr);
			policyCheckRes =  mcMdp.checkExpressionProb((MDP)result, (ExpressionProb)expr, null);
			//policyCheck = mcMdp.computeReachProbsGaussSeidel((MDP)result, no, acc, false, null, null, strat);
			System.out.println(strat);
			
			//mcMdp.restrictStrategyToReachableStates((MDP)result, strat);
			
			//DTMC policy = new DTMCFromMDPAndMDStrategy((MDP)model, (MDStrategy)policyCheck.strat);
			
		}
		//else {
			//policyCheck = mcDTMC.computeReachProbsGaussSeidel((DTMC)result, no, acc, null, null);
		//}
		
		return policyCheckRes;
	}
	
	/**
	 * Model check a P operator.
	 */
	private Result checkExpressionProb(ExpressionProb expr) throws PrismException
	{
		ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, this);
		
		System.out.println("LTL"  + expr.getExpression());
		LTLModelChecker ltlMC = new LTLModelChecker(this);
		List<Expression> labelExprs = new ArrayList<Expression>();
		
		
		
		List<Entry<Double, Expression>> weightedLtlSpecs = splitExpression(expr);
		int nSpecs = weightedLtlSpecs.size();
		
		List<Double> weights = new ArrayList<Double>(nSpecs);
		List<DA<BitSet,? extends AcceptanceOmega>> automata = new ArrayList<DA<BitSet,? extends AcceptanceOmega>>(nSpecs);		
		List<ProductModelGenerator> prodModelGens = new ArrayList<ProductModelGenerator>(nSpecs);
		
		DA<BitSet,? extends AcceptanceOmega> da = null;
		ProductModelGenerator prodModelGen = null;
		for(Entry<Double, Expression>weightedSpec : weightedLtlSpecs) {
			double weight = weightedSpec.getKey();
			
			da = ltlMC.constructExpressionDAForLTLFormula(weightedSpec.getValue(), labelExprs, allowedAcceptance);
			da.setDistancesToAcc();
			System.out.println(da.getDistsToAcc());
			da.printDot(mainLog);
			prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
			
			weights.add(weight);
			automata.add(da);
			prodModelGens.add(prodModelGen);
		}
		
		ProductModelGenerator nestedProdModelGen = prodModelGens.get(0);
		for (int i = 1; i < nSpecs; i++) {
			nestedProdModelGen = new ProductModelGenerator(nestedProdModelGen, automata.get(i), labelExprs);
		}
			
		
//		ConstructModel constructModel = new ConstructModel(this);
//		explicit.Model model2 = constructModel.constructModel(prodModelGen);
		

				
		MDP merged = null;
		boolean achieved = false;
		Deque<State> queue = new ArrayDeque<State>();
		List<State> analysed = new ArrayList<State>();
		queue.add(nestedProdModelGen.getInitialState());
		analysed.add(nestedProdModelGen.getInitialState());
		while (!achieved) {
			State currentState = queue.poll();
			for (int j = 0; j < 4; j++) {
				UCT uct = new UCT(this, nestedProdModelGen, currentState , 50, automata, weights);
				//RewardStruct rewStruct = expr.getRewardStructByIndexObject(modulesFile, constantValues);
				//uct.setRewardStruct(rewStruct);
				uct.setConstantValues(constantValues);
				Model model = uct.buildDTMC(uct.search()); 
				
				merged = mergeMdps(merged,(MDP)model);
			}
			merged.exportToDotFile("/home/bruno/Desktop/Dropbox/icaps/res/merged.dot");
			Expression pila = new ExpressionProb(weightedLtlSpecs.get(0).getValue(), "max=", null);

			StateValues policyCheck = checkUCTResult(merged, pila, da, nestedProdModelGen, "policy");
			//System.out.println(policyCheck.soln[0]);
			//if (policyCheck.soln[0]>=0.99) {
			if (policyCheck.getDoubleArray()[0] >= 0.99) {
				achieved = true;
			}
			
			for (int k = 0; k < merged.getNumStates(); k++) {
				if (policyCheck.getDoubleArray()[0] == 0 && merged.isDeadlockState(k)) {
				//if (policyCheck.soln[k] == 0 && merged.getNumChoices(k) == 0 && !analysed.contains(merged.getStatesList().get(k))) {
					System.out.println("ADDED " + k);
					queue.add(merged.getStatesList().get(k));
					analysed.add(merged.getStatesList().get(k));
				}
			}
		}
		
		return new Result(new Double(1));//uct.getReward()
	}
	
	public BitSet findAccStates(Model dtmc, DA<BitSet,? extends AcceptanceOmega> da, int daStateIndex) {
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
			if (daAcc.get((int)dtmc.getStatesList().get(i).varValues[daStateIndex])) {
				acc.set(i);
			}
		}
		
		return acc;
	}
	
	
	
	
	public MDP mergeMdps(MDP mdp1, MDP mdp2) {
		if (mdp1 == null) {
			return mdp2;
		}
		
		
		int stateIndex, targetStateIndex;
		String actionName;
		Distribution distrMdp2, distrRes;
		
		
		MDPSimple res = new MDPSimple((MDPSimple)mdp1);
		List<State> states = res.getStatesList();
		
		int i = 0;
		for (State state : mdp2.getStatesList()) {
			stateIndex = states.indexOf(state);
			if (stateIndex == -1) {
				stateIndex = res.addState();
				states.add(state);
			}
			for (int j = 0; j < mdp2.getNumChoices(i); j++) {
				actionName = (String)mdp2.getAction(i, j);
				distrMdp2 = ((MDPSimple)mdp2).getChoice(i, j);
				Iterator<Entry<Integer, Double>> iterator = distrMdp2.iterator();
				
				distrRes = new Distribution();
				while (iterator.hasNext()) {
					Entry<Integer, Double> current = iterator.next();
					State targetState = mdp2.getStatesList().get(current.getKey());
					targetStateIndex = states.indexOf(targetState);
					if (targetStateIndex == -1) {
						targetStateIndex = res.addState();
						states.add(targetState);
					}
					distrRes.add(targetStateIndex, current.getValue());
					
				}
				res.addActionLabelledChoice(stateIndex, distrRes, actionName);
			}
			i++;			
		}
		return res;
		
	}
}
