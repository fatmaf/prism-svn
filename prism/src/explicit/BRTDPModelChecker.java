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
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.BRTDP.SearchState;
import explicit.LTLModelChecker.LTLProduct;
import explicit.ProbModelChecker.TermCrit;
import explicit.UCT.UCTNode;
import explicit.rewards.MDPRewards;
import parser.State;
import parser.Values;
import parser.VarList;
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
import prism.PrismSettings;
import prism.PrismUtils;
import prism.ProductModelGenerator;
import prism.Result;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;

/**
 * MDP model checker based on BRTDP
 */
public class BRTDPModelChecker extends PrismComponent
{
	protected double termCritParam = 1e-8;
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
	
	MDPModelChecker mcMdp;
	boolean useDistCost = false;
	/**
	 * Constructor.
	 */
	public BRTDPModelChecker(PrismComponent parent, ModulesFile modulesFile, PropertiesFile propertiesFile) throws PrismException
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
		
		mcMdp = new MDPModelChecker(null);
		mcMdp.setSettings(new PrismSettings());
		mcMdp.setLog(new PrismDevNullLog());
		mcMdp.setGenStrat(true);
		
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
		//if (expr instanceof ExpressionReward)
			//res = checkExpressionReward((ExpressionReward) expr);
		else
			throw new PrismNotSupportedException("MCTS not yet supported for this operator");

		return res;
	}
	
	/**
	 * Model check a P operator.
	 */
	private Result checkExpressionProb(ExpressionProb expr) throws PrismException
	{
		System.out.println("LTL"  + expr.getExpression());
		LTLModelChecker ltlMC = new LTLModelChecker(this);
		List<Expression> labelExprs = new ArrayList<Expression>();
		AcceptanceType[] allowedAcceptance = {
				AcceptanceType.RABIN,
				AcceptanceType.REACH
		};
		DA<BitSet,? extends AcceptanceOmega> da = ltlMC.constructExpressionDAForLTLFormula(expr.getExpression(), labelExprs, allowedAcceptance);
		da.setDistancesToAcc();
		da.printDot(mainLog);
		ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, this);
		ProductModelGenerator prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
		
		BitSet acc = da.getAccStates();
		BitSet sinkStates = da.getSinkStates();
		
		/*if (true) {
			List<Double> distsToAcc = da.calculateDistsToState(acc.nextSetBit(0));
			BRTDP brtdp = new BRTDP(this, prodModelGen, distsToAcc, sinkStates, 10, 10, false, null);
			LinkedList<SearchState> initStates = new LinkedList<SearchState>();
			// Add initial state(s) to 'explore', 'states' and to the model
			for (State initState : prodModelGen.getInitialStates()) {
				brtdp.doSearch(initState);
			}
			MDP mdp = brtdp.getMdp();
			mdp.exportToDotFile("/home/bruno/Desktop/mdp_brtdp.dot");
			
			BitSet accModel = findAccStates(mdp, da);
			ModelCheckerResult checkRes = checkReachability(mdp, accModel);
			double reachProb = checkRes.soln[0];
			
			System.out.println("MDP num states: " + mdp.getNumStates());
			System.out.println("MDP MAXPROB: " + reachProb);
			
			MDP policies = brtdp.getPolicies();
			policies.exportToDotFile("/home/bruno/Desktop/policy_brtdp.dot");
			
			accModel = findAccStates(policies, da);
			checkRes = checkReachability(policies, accModel);
			reachProb = checkRes.soln[0];
			
			System.out.println("Policies num states: " + policies.getNumStates());
			System.out.println("Policies MAXPROssssB: " + reachProb);
			
			//StateValues res = checkResult(mdp, expr);
			return new Result(reachProb);
		}*/
		
		
		
		// Add initial state(s) to 'explore', 'states' and to the model
		for (int i = acc.nextSetBit(0); i >= 0; i = acc.nextSetBit(i + 1)) {
			BRTDP brtdp = new BRTDP(this, prodModelGen, da.calculateDistsToState(i), sinkStates, 10, 10, false, null);
			for (State initState : prodModelGen.getInitialStates()) {
				brtdp.doSearch(initState);
				//MDP mdp = brtdp.getGreedyPolicy(false);
				MDP mdp = brtdp.getMdp();
				State aux = mdp.getStatesList().get(mdp.getFirstInitialState());
				BitSet accModel = findAccStates(mdp, da);
				ModelCheckerResult checkRes = checkReachability(mdp, accModel);
				double reachProb = checkRes.soln[0];
				System.out.println("CONVERGED");
				System.out.println(reachProb);
				/*
				for (int j = accModel.nextSetBit(0); j >= 0; j = accModel.nextSetBit(j + 1)) {
					SearchState state = (SearchState)mdp.getStatesList().get(j);
					state.configureAccForSearch();
					brtdp.expand(state);
					int daVal = (int)state.varValues[state.varValues.length-1];
					brtdp.doSearch(state);
				}
				*/
			}
			MDP mdp = brtdp.getMdp();
			MDP policy = brtdp.getGreedyPolicy(false);
			System.out.println("MDP num states: " + mdp.getNumStates());
			mdp.exportToDotFile("/home/bruno/Desktop/mdp_brtdp.dot");
			policy.exportToDotFile("/home/bruno/Desktop/policy_brtdp.dot");
			StateValues res = checkResult(mdp, expr);
			System.out.println("PROB:" + res.valuesD[0]);
			res = checkResult(policy, expr);
			System.out.println("PROB:" + res.valuesD[0]);
		}
		
		
		
		
		
		/*UCT uct;
		UCTNode uctRes;
		BitSet originalTerminalStates = new BitSet();
		BitSet prunedTerminalStates = new BitSet();
		int depth = 15;
		int nSamples = 200000;
		
		MDPSimple resMDP2 = null, resMDP = null;
		List<MDPSimple> candidateMDPs = new ArrayList<MDPSimple>();
		List<Double> candidateMDPsReachProbs = new ArrayList<Double>();
		List<BitSet> candidateMDPsAccStates = new ArrayList<BitSet>();
		double reachProb;
		int nCandidates = 0;
		BitSet accModel = null;
		ModelCheckerResult checkRes = null;
		for (int i = acc.nextSetBit(0); i >= 0; i = acc.nextSetBit(i + 1)) {
			List<Double> distsToAcc = da.calculateDistsToState(acc.nextSetBit(0));
			
			uct = new UCT(this, prodModelGen, prodModelGen.getInitialState(), distsToAcc, sinkStates, depth, nSamples, useDistCost, null); //TODO iterative deepening; tweak exploration param, # samples, depth
			uctRes = uct.search();
			resMDP = buildMDP(uctRes, prodModelGen, originalTerminalStates);
			accModel = findAccStates(resMDP, da);
			checkRes = checkReachability(resMDP, accModel);
			resMDP2 = buildPolicyMDP(prodModelGen, resMDP, checkRes, originalTerminalStates, prunedTerminalStates);
			reachProb = checkRes.soln[0];
			int pos = 0;
			if(reachProb > 0) {
				for(pos = 0; pos < candidateMDPsReachProbs.size(); pos++) {
					if (reachProb > candidateMDPsReachProbs.get(pos)) {
						break;
					}
				}
				candidateMDPs.add(pos, resMDP);
				candidateMDPsReachProbs.add(pos, reachProb);
				candidateMDPsAccStates.add(pos, accModel);
				nCandidates++;
			}
		}
		
		
		
	
		depth = 10;
		nSamples = 10000;
		// THIS IS THE VERSION THAT DEEPENS THE HORIZON - IT ALSO ASSUMES THERE IS ONLY ONE POLICY FOR THE ACCEPTING STATE
		accModel = findAccStates(resMDP, da);
		BitSet newTerminalStates = new BitSet();
		BitSet oldTerminalStates = new BitSet();
		while (!prunedTerminalStates.equals(oldTerminalStates)) {
			newTerminalStates = (BitSet)prunedTerminalStates.clone();
			newTerminalStates.xor(oldTerminalStates);
			oldTerminalStates = (BitSet)prunedTerminalStates.clone();
			for (int i = newTerminalStates.nextSetBit(0); i >= 0; i = newTerminalStates.nextSetBit(i + 1)) {
				State startState = resMDP.getStatesList().get(i);
				//TODO add L to sink states so they are avoided
				uct = new UCT(this, prodModelGen, startState, da.calculateDistsToState(acc.nextSetBit(0)), da.getSinkStates(), depth, nSamples, useDistCost, resMDP2); //TODO iterative deepening; tweak exploration param, # samples, depth
	//			uct = new UCT(this, prodModelGen, startState, da.calculateDistsToState((int)startState.varValues[prodModelGen.getNumVars() - 1]), da.getSinkStates(), depth, nSamples); //TODO iterative deepening; tweak exploration param, # samples, depth
				uctRes = uct.search();
				//terminalStates.clear();
				resMDP = buildMDP(uctRes, prunedTerminalStates, resMDP);
			}
		}

		
		StateValues res = checkResult(resMDP, expr);
		//StateValues res = checkUCTResult(uctRes, expr, da, prodModelGen, "policy");
		//DTMC dtmc = buildDTMC(res, prodModelGen);
		//MDP mdp = buildMDP(res, prodModelGen);
		mainLog.println("\nThe probability is " + res.valuesD[0]);
		*/
		return new Result(new Double(1));//uct.getReward()
	}
	
	public BitSet findAccStates(Model model, DA<BitSet,? extends AcceptanceOmega> da) {
		BitSet daAcc = da.getAccStates();
		int daStateIndex = model.getVarList().getIndex("_da0");
		int n = model.getNumStates();
		BitSet acc = new BitSet(n);
		for (int i = 0; i < n; i++) {
			if (daAcc.get((int)model.getStatesList().get(i).varValues[daStateIndex])) {
				acc.set(i);
			}
		}
		
		return acc;
	}
	
	
	
	
	
	
	public MDPSimple buildPolicyMDP(ModelGenerator modelGen, MDPSimple originalMDP, ModelCheckerResult checkRes, BitSet originalTerminalStates, BitSet prunedTerminalStates) throws PrismException {
		MDPSimple prunedMDP = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		Map<Integer, Integer> indexMap = new HashMap<Integer, Integer>();
		int originalStateIndex = 0;
		int prunedStateIndex = 0;
		MDStrategy strat = (MDStrategy)checkRes.strat;
		
		VarList varList = modelGen.createVarList();
		prunedMDP.setVarList(varList);
		
		prunedMDP.addState();
		statesList.add(originalMDP.getStatesList().get(0));
		
		getReachableStates(originalMDP, checkRes, prunedMDP, statesList, indexMap, originalStateIndex, prunedStateIndex, originalTerminalStates, prunedTerminalStates);
		
		for (int i = 0; i < originalMDP.getNumStates(); i++) {
			if (!indexMap.containsKey(i)) {
				if (strat.isChoiceDefined(i)) {
					prunedStateIndex = prunedMDP.addState();
					statesList.add(originalMDP.getStatesList().get(i));
					getReachableStates(originalMDP, checkRes, prunedMDP, statesList, indexMap, i, prunedStateIndex, null, null);
				}
			}
		}

		
		prunedMDP.setStatesList(statesList);
		prunedMDP.exportToDotFile("/home/bruno/Desktop/mdp_prune.dot");
		
		return prunedMDP;
	}
	
	
	public void getReachableStates(MDPSimple originalMDP, ModelCheckerResult checkRes, MDPSimple prunedMDP, List<State> statesList, Map<Integer, Integer> indexMap, int originalStateIndex, int prunedStateIndex, BitSet originalTerminalStates, BitSet prunedTerminalStates) {
		Deque<Integer> originalIndexQueue = new ArrayDeque<Integer>();
		Deque<Integer> prunedIndexQueue = new ArrayDeque<Integer>();
		Iterator<Entry<Integer, Double>> succs;
		Entry<Integer, Double> originalOutcome; 
		int originalSuccIndex;
		Integer prunedSuccIndex;
		Distribution originalDistr, prunedDistr;
		double prob;
		Object action;
		MDStrategy strat = (MDStrategy)checkRes.strat;

		
		originalIndexQueue.push(originalStateIndex);
		prunedIndexQueue.push(prunedStateIndex);
		while (!originalIndexQueue.isEmpty()) {
			originalStateIndex = originalIndexQueue.pop();
			prunedStateIndex = prunedIndexQueue.pop();
			if (strat.isChoiceDefined(originalStateIndex)) {
				originalDistr = originalMDP.getChoice(originalStateIndex, strat.getChoiceIndex(originalStateIndex));
				succs = originalDistr.iterator();
				prunedDistr = new Distribution();
				while (succs.hasNext()) {
					originalOutcome = succs.next();
					originalSuccIndex = originalOutcome.getKey();
					prob = originalOutcome.getValue();
					prunedSuccIndex = indexMap.get(originalSuccIndex);
					if (prunedSuccIndex == null) {
						statesList.add(originalMDP.getStatesList().get(originalSuccIndex));
						prunedSuccIndex = prunedMDP.addState();
						originalIndexQueue.push(originalSuccIndex);
						prunedIndexQueue.push(prunedSuccIndex);
						indexMap.put(originalSuccIndex,  prunedSuccIndex);
					}
					prunedDistr.add(prunedSuccIndex, prob);
				}
				action = strat.getChoiceAction(originalStateIndex);
				prunedMDP.addActionLabelledChoice(prunedStateIndex, prunedDistr, action);
			} else {
				if (originalTerminalStates != null) {
					if (checkRes.soln[originalStateIndex] == 1) {
						prunedTerminalStates.set(prunedStateIndex);
					};
				}
			}
		}
	}
	
	
	private StateValues checkResult(MDP resMDP, Expression expr) throws PrismException {
		mcMdp.setGenStrat(true);
		mcMdp.setExportAdv(true);
		mcMdp.setExportAdvFilename("/home/bruno/Desktop/policy.adv");
		
		resMDP.findDeadlocks(true);
		return mcMdp.checkExpressionProb((MDP)resMDP, (ExpressionProb)expr, null);
	}
	
	private ModelCheckerResult checkReachability(MDP policyUCT, BitSet acc) throws PrismException {
		policyUCT.findDeadlocks(true);
		ModelCheckerResult res = mcMdp.computeReachProbs(policyUCT, acc, false);
		return res;
	}
	
	
}
