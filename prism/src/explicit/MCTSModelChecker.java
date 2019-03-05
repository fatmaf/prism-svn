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
 * MDP model checker based on MCTS
 */
public class MCTSModelChecker extends PrismComponent
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
	private String saveLocation = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/results/mcts";

	
	AcceptanceType[] allowedAcceptance = {
			AcceptanceType.RABIN,
			AcceptanceType.REACH
	};
	
	MDPModelChecker mcMdp;
	boolean useDistCost = false;
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
	
		
		UCT uct;
		UCTNode uctRes;
		BitSet originalTerminalStates = new BitSet();
		BitSet prunedTerminalStates = new BitSet();
		int depth = 30;
		int nSamples = 100000;
		
		MDPSimple resMDP2 = null, resMDP = null;
		List<MDPSimple> candidateMDPs = new ArrayList<MDPSimple>();
		List<Double> candidateMDPsReachProbs = new ArrayList<Double>();
		List<BitSet> candidateMDPsAccStates = new ArrayList<BitSet>();
		double reachProb = 0.0;
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
		
		
		
		
		//DTMC policy = new DTMCFromMDPAndMDStrategy(resMDP, (MDStrategy)checkRes.strat);
		//policy.exportToDotFile("/home/bruno/Desktop/dtmc.dot");
		
		/*
		depth = 10;
		nSamples = 10000;
		for(int i = 0; i < nCandidates; i++) {
			resMDP = candidateMDPs.get(i);
			accModel = candidateMDPsAccStates.get(i);
			for (int j = accModel.nextSetBit(0); j >= 0; j = accModel.nextSetBit(j + 1)) {
				State startState = resMDP.getStatesList().get(j);
				int targetDRAState = (int)startState.varValues[startState.varValues.length-1];
				//TODO add L to sink states so they are avoided
				uct = new UCT(this, prodModelGen, startState, da.calculateDistsToState(acc.nextSetBit(0)), da.getSinkStates(), depth, nSamples, useDistCost); //TODO iterative deepening; tweak exploration param, # samples, depth
				uctRes = uct.search();
				//terminalStates.clear();
				//resMDP = buildMDP(uctRes, prodModelGen, terminalStates);
				//accModel = findAccStates(resMDP, da);
				//accModel.clear(0);
				//reachProb = checkReachability(resMDP, accModel).soln[0];
				resMDP = buildMDP(uctRes, prunedTerminalStates, resMDP);
			}
		}*/

		
		
		//resMDP.findDeadlocks(true);
		
/*
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
		return new Result(reachProb);//uct.getReward()
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
		prunedMDP.exportToDotFile(saveLocation+"mdp_prune.dot");
		
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
	
	

	public MDPSimple buildMDP(UCTNode node, BitSet terminalStates, MDPSimple mdp) throws PrismException{
		int i, currentStateIndex, succStateIndex, nSuccs;
		double prob;
		Distribution distr;
		String action;
		UCTNode currentNode, currentSucc, bestSucc, succs[];
		Deque<UCTNode> nodeQueue = new ArrayDeque<UCTNode>();
		Deque<Integer> indexQueue = new ArrayDeque<Integer>();
		State currentState;
		List<State> statesList = mdp.getStatesList();
		
		nodeQueue.add(node);
		indexQueue.add(statesList.indexOf(node.getState()));

		while(!nodeQueue.isEmpty()) {
			currentNode = nodeQueue.poll();
			currentStateIndex = indexQueue.poll();
			
			if (currentNode.getStateType() == UCT.SINK_STATE) {
				continue;
			}
			
			
			if (useDistCost) {
				if (PrismUtils.doublesAreClose(currentNode.getExpectedRewEstimate(), currentNode.getStateCost()*currentNode.getDepth(), termCritParam, false)) {
					terminalStates.set(currentStateIndex);
					continue;
				}
			} else {
				if(currentNode.getExpectedRewEstimate() == 0) {
					terminalStates.set(currentStateIndex);
					continue;
				}
			}
			
			if (currentNode.getStateType() == UCT.ACC_STATE) {
				terminalStates.set(currentStateIndex);
				continue;
			}
//			System.out.println(currentNode.getState());
//			System.out.println(currentNode.getStateCost());
			
			//find succ corresponding to best decision
			succs = currentNode.getSuccNodes();
			nSuccs = currentNode.getNumSuccs();
			double best;
			if (useDistCost) {
				best = Double.MAX_VALUE;
			} else {
				best = - Double.MAX_VALUE;
			}
			bestSucc = null;
			action = null;
			for (i = 0; i < nSuccs; i++) {
				currentSucc = succs[i];
				if (currentSucc.getNumVisits() > 0) {
					if (useDistCost) {
						if (currentSucc.getExpectedRewEstimate() < best) {
							bestSucc = currentSucc;
							best = currentSucc.getExpectedRewEstimate();
							action = currentSucc.getActionName();
						}
					} else {
						if (currentSucc.getExpectedRewEstimate() > best) { 
							bestSucc = currentSucc;
							best = currentSucc.getExpectedRewEstimate();
							action = currentSucc.getActionName();
						}
					}
				}
			}
			
			
			//add and queue all succs corresponding to possible outcomes for best decision
			if (bestSucc != null) {
				currentNode = bestSucc;
				succs = currentNode.getSuccNodes();
				nSuccs = currentNode.getNumSuccs();
				distr = new Distribution();
				for (i = 0; i < nSuccs; i++) {
					currentSucc = succs[i];
					currentState = currentSucc.getState();
					succStateIndex = statesList.indexOf(currentState);
					if (succStateIndex == -1) {
						succStateIndex = mdp.addState();
						statesList.add(currentState);
					}
					prob = currentSucc.getReachProb();
					distr.add(succStateIndex, prob);
					nodeQueue.add(currentSucc);
					indexQueue.add(succStateIndex);
				}
					mdp.addActionLabelledChoice(currentStateIndex, distr, action);
			}
		}
		mdp.setStatesList(statesList);
		mdp.exportToDotFile(saveLocation+"mdp.dot");

		return mdp;
	}
	
	public MDPSimple buildMDP(UCTNode node, ModelGenerator modelGen, BitSet terminalStates) throws PrismException{
		MDPSimple mdp = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		
		VarList varList = modelGen.createVarList();
		mdp.setVarList(varList);
		
		int currentStateIndex = mdp.addState();
		mdp.addInitialState(currentStateIndex);
		statesList.add(node.getState());
		mdp.setStatesList(statesList);
		mdp = buildMDP(node, terminalStates, mdp);
		return mdp;
	}
	
	private StateValues checkResult(MDP resMDP, Expression expr) throws PrismException {
		mcMdp.setGenStrat(true);
		mcMdp.setExportAdv(true);
		mcMdp.setExportAdvFilename(saveLocation+"policy.adv");
		
		resMDP.findDeadlocks(true);
		return mcMdp.checkExpressionProb((MDP)resMDP, (ExpressionProb)expr, null);
	}
	
	private ModelCheckerResult checkReachability(MDP policyUCT, BitSet acc) throws PrismException {
		policyUCT.findDeadlocks(true);
		ModelCheckerResult res = mcMdp.computeReachProbs(policyUCT, acc, false);
		return res;
	}
	
/*	private ModelCheckerResult checkMaxProg(MDP policyUCT) {
		
		ModelCheckerResult res = mcMdp.computeTotalRewardsMax(policyUCT, MDPRewards mdpRewards, boolean noPositiveECs) throws
	}
	*/
	private StateValues checkUCTResult(UCTNode result, Expression expr, DA<BitSet,? extends AcceptanceOmega> da, ProductModelGenerator prodModelGen, String modelName) throws PrismException {
		//DTMCModelChecker mcDTMC;
		ModelCheckerResult policyCheck = null;
			
		//mcDTMC = new DTMCModelChecker(null);
		//mcDTMC.setSettings(this.getSettings());
		//mcDTMC.setLog(new PrismDevNullLog());
		
		MDP resMDP = buildMDP(result, prodModelGen, new BitSet());
		resMDP.findDeadlocks(true);

		
		MDPModelChecker mcMdp = new MDPModelChecker(null);
		mcMdp.setSettings(new PrismSettings());
		mcMdp.setLog(new PrismDevNullLog());
		mcMdp.setGenStrat(true);
		mcMdp.setExportAdv(true);
		mcMdp.setExportAdvFilename(saveLocation+"policy.adv");
		
		LTLModelChecker mcLtl = new LTLModelChecker(null);
		mcLtl.setSettings(new PrismSettings());
		mcLtl.setLog(new PrismDevNullLog());
		
		
		BitSet initState = new BitSet(resMDP.getNumStates());
		for (int i = 0; i < resMDP.getNumStates(); i++) {
			if (resMDP.isInitialState(i)) {
				initState.set(i);
			}
		}
		
		
		BitSet acc = findAccStates(resMDP, da);
		for (int i = acc.nextSetBit(0); i >= 0; i = acc.nextSetBit(i + 1)) {
			State startState = resMDP.getStatesList().get(i);
		}

		
		
		StateValues policyCheckRes = null;
/** OLD CODE
		resMDP.findDeadlocks(true);
		LTLProduct<MDP> prod = mcLtl.constructProductMDP(mcMdp, resMDP, ((ExpressionProb)expr).getExpression(), initState, allowedAcceptance);
		//List<BitSet> ecs = mcLtl.findAcceptingECsForRabin(prod.getProductModel(), (AcceptanceRabin)prod.getAcceptance());
		BitSet ecs = mcLtl.findAcceptingECStatesForRabin(prod.getProductModel(), (AcceptanceRabin)prod.getAcceptance());
		
		;
		
		prod.getProductModel().exportToDotFile("/home/bruno/Desktop/prod.dot");

		
				
		BitSet no, acc;
		no = new BitSet(resMDP.getNumStates());
		acc = new BitSet(resMDP.getNumStates());
		//acc = findAccStates(prod.getProductModel(), da, prodModelGen.getVarIndex("_da0"));
		acc = findAccStates(prod.getProductModel(), da); 
		
		
		
		int[] strat = new int[prod.getProductModel().getNumStates()];
		for (int i = 0; i < prod.getProductModel().getNumStates(); i++) {
			strat[i]=-1;
		}
		
		
		
		
		ModelCheckerResult res = mcMdp.computeReachProbs((MDP) prod.getProductModel(), ecs, false);

		
		
		BitSet prob1 = mcMdp.prob1(prod.getProductModel(), ecs, acc, false, strat);
		
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
		policy.exportToDotFile("/home/bruno/Desktop/policy" + modelName + ".dot");
*/
		if(resMDP instanceof MDP) {		
			//policyCheck = mcMdp.computeReachProbs((MDP)model, acc, false);
			//model = new MDPSparse((MDPSimple)model);
			//Result policyCheckRes = mcMdp.check((MDP)result, expr);
			policyCheckRes =  mcMdp.checkExpressionProb((MDP)resMDP, (ExpressionProb)expr, null);
			//policyCheck = mcMdp.computeReachProbsGaussSeidel((MDP)result, no, acc, false, null, null, strat);
			//System.out.println(strat);
			
			//mcMdp.restrictStrategyToReachableStates((MDP)result, strat);
			
			//DTMC policy = new DTMCFromMDPAndMDStrategy((MDP)model, (MDStrategy)policyCheck.strat);
			
		}
		//else {
			//policyCheck = mcDTMC.computeReachProbsGaussSeidel((DTMC)result, no, acc, null, null);
		//}
		return policyCheckRes;
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
	
	
	/*	*//**
	 * Model check an R operator.
	 *//*
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
	}*/
	
	
}
