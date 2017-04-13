//==============================================================================
//	
//	Copyright (c) 2013-
//	Authors:
//	* Dave Parker <david.parker@comlab.ox.ac.uk> (University of Oxford)
//  * Frits Dannenberg <frits.dannenberg@cs.ox.ac.uk> (University of Oxford)
//	* Ernst Moritz Hahn <emhahn@cs.ox.ac.uk> (University of Oxford)
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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Random;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import automata.DA;

import parser.ast.Expression;
import parser.ast.ExpressionIdent;
import parser.ast.LabelList;
import parser.ast.RewardStruct;
import parser.type.TypeDouble;
import parser.Values;
import parser.State;
import parser.VarList;
import prism.*;
import prism.Model;

//TODO REMOVE UNEEDED IMPORTS


/**
 * Implementation of the upper confidence bounds applied to trees (UCT) algorithm
 */
public final class UCT extends PrismComponent
{
	/**
	 * Stores a UCT search node.
	 */
	private final static class UCTNode
	{
		/** MDP state representation */
		private State state;
		/** The action that was executed to reach this node */
		private int action;
		private String actionName;
		/** The probability of reaching this node */
		private double reachProb;
		/** true iff it is a decision node */
		private boolean decision;
		/** number of rollouts that have visited this node */
		private int numVisits;
		/** Have the succs of this node been computed*/
		private boolean expanded;
		/** expected reward estimate from this node, based on the previous rollouts */
		private double expectedRewEstimate;
		/** successor UTC nodes */
		private UCTNode[] succNodes;

		
		/**
		 * Constructs an UCT search node object.
		 * 
		 * @param state The MDP state representation
		 * @param action The action that brought the UCT search to this node.
		 */
		UCTNode(State state, int action, String actionName, double reachProb, boolean decision)
		{
			this.decision = decision;
			this.state = state;
			this.action = action;
			this.actionName = actionName;
			this.reachProb = reachProb;
			this.numVisits = 0;
			this.expanded = false;
			this.expectedRewEstimate = 0.0;
			this.succNodes = null;
		}
		
		/**
		 * Gets the MDP state
		 * 
		 * @return MDP state
		 */
		State getState()
		{
			return state;
		}
		
		/**
		 * Gets the action that was executed to reach this UCT node
		 * 
		 * @return action
		 */
		int getAction()
		{
			return action;
		}
		
		/**
		 * Gets the probability to reach this UCT node from its parent
		 * 
		 * @return action
		 */
		double getReachProb()
		{
			return reachProb;
		}
		
		
		/**
		 * Gets the number of times this node has been visited
		 * 
		 * @return MDP state
		 */
		int getNumVisits()
		{
			return numVisits;
		}
		
		/**
		 * Gets the  estimate for the expected reward for this node, based on previous rollouts
		 * 
		 * @return MDP state
		 */
		double getExpectedRewEstimate()
		{
			return expectedRewEstimate;
		}
		
		/**
		 * Gets the children of this UTC node
		 * 
		 * @return MDP state
		 */
		UCTNode[] getSuccNodes()
		{
			return succNodes;
		}
	

		/**
		 * Increment number of visits to this node
		 * 
		 */
		void incrementNumVisits()
		{
			this.numVisits = this.numVisits + 1;
		}
		
		/**
		 * Update reward
		 * 
		 * @param reward state reward to set
		 */
		void updateExpectedRewEstimate(double reward)
		{
			this.expectedRewEstimate = ((double)(this.numVisits - 1)*this.expectedRewEstimate + reward)/(double)(this.numVisits);
		}
		

		/**
		 * Sets successor UCT nodes (i.e., children of this node).
		 * 
		 * @param succNodes successor nodes
		 */
		void setSuccNodes(UCTNode[] succNodes)
		{
			this.succNodes = succNodes;
		}

		/**
		 * Returns number of successor states of this state.
		 * 
		 * @return number of successor states
		 */
		int getNumSuccs()
		{
			if (succNodes == null) {
				return 0;
			} else {
				return succNodes.length;
			}
		}
		
		/**
		 * Checks whether this state has successors or not.
		 * Will be true if and only if successor state array is nonnull.
		 * 
		 * @return whether this state has successors or not
		 */
		boolean hasSuccs()
		{
			return succNodes != null;
		}

		
		/**
		 * Checks whether this node has been visited
		 * 
		 * @return whether this node has been visited
		 */
		boolean isExpanded()
		{
			return expanded;
		}
		
		void setAsExpanded()
		{
			expanded = true;
		}
		
		
		boolean isDecisionNode()
		{
			return decision;
		}
		
		double getUCTScore(double parentVisits, double bias)
		{
			if (numVisits == 0) {
				return Double.MAX_VALUE;
			} else {
				return bias*Math.sqrt(Math.log(parentVisits)/numVisits) + expectedRewEstimate;
			}
		}
	}
		



	/** model exploration component to generate new states */
	private ModelGenerator modelGen;
	/**State to start trials in */
	State initialState;
	/** rollout depth */
	private int depth;
	/** automata list */
	private List<DA<BitSet,? extends AcceptanceOmega>> automata;
	List<Integer> daStateIndices;
	int nSpecs;
	List<Double> weights;
	
	/** reward structure to use for analysis */
	private RewardStruct rewStruct = null;
	/** model constants */
	private Values constantValues = null;
	/**random number generation */
	Random randomGen;
	
	
	
	
	///** maps from state (assignment of variable values) to property object */
	//private LinkedHashMap<State,StateProp> states;

	///** target state set - used for reachability (until or finally properties) */
	//private Expression target;
//	/** set of initial states of the model */
//	private HashSet<State> initStates;
	
	/**
	 * Constructor.
	 */
	public UCT(PrismComponent parent, ModelGenerator modelGen, State initialState, int depth, List<DA<BitSet,? extends AcceptanceOmega>> automata, List<Double> weights) throws PrismException
	{
		super(parent);
		
		this.modelGen = modelGen;
		this.initialState = initialState;
		this.depth = depth;
		this.automata = automata;
		nSpecs = automata.size();
		this.daStateIndices = new ArrayList<Integer>(nSpecs);
		for (int i = 0; i < nSpecs; i++) {
			daStateIndices.add(modelGen.getVarIndex("_da" + i));
		}
		this.weights = weights;
		rewStruct = null;
		constantValues = null;
		randomGen = new Random();
	}


	/**
	 * Sets reward structure to use.
	 * 
	 * @param rewStruct reward structure to use
	 */
	public void setRewardStruct(RewardStruct rewStruct)
	{
		this.rewStruct = rewStruct; //TENHO Q METER AQUI UMA MERDA COM A PROGRESSION EM VEZ DISTO
	}
	
	/**
	 * Sets values for model constants.
	 * 
	 * @param constantValues values for model constants
	 */
	public void setConstantValues(Values constantValues)
	{
		this.constantValues = constantValues;
	}
	

	
	public void expandNode(UCTNode node) throws PrismException {
		int i, nc, nt;
		double prob;
		UCTNode[] succNodes;
		
		modelGen.exploreState(node.getState());
		if (node.isDecisionNode()) {
			nc = modelGen.getNumChoices();
			succNodes = new UCTNode[nc];
			for (i = 0; i < nc; i++) {
				succNodes[i] = new UCTNode(node.state, i, modelGen.getChoiceAction(i).toString(), -1, false);
			}
		}
		else {
			nt = modelGen.getNumTransitions(node.getAction());
//			System.out.println("ACTION:" + modelGen.getTransitionAction(node.getAction()).toString());
			succNodes = new UCTNode[nt];
			for (i = 0; i < nt; i++) {
//				System.out.println(node);
				prob = modelGen.getTransitionProbability(node.getAction(), i);
//				System.out.println("ACTION:" + node.getAction() + " PROB: " + prob);
				State succState = modelGen.computeTransitionTarget(node.getAction(), i);
				succNodes[i] = new UCTNode(succState, node.getAction(), null, prob, true);
				
			}
		}
		node.setSuccNodes(succNodes);
		node.setAsExpanded();
	}
	
	public UCTNode getBestUCTSucc2(UCTNode node, double bias) {
		int i, numSuccs;
		double score, maxScore = -Double.MAX_VALUE;
		UCTNode succNode, bestSucc;
		UCTNode[] succNodes = node.getSuccNodes();
		
		bestSucc = null;
		numSuccs = node.getNumSuccs();
		for (i = 0; i < numSuccs; i++) {
			succNode = succNodes[i];
			score = succNode.getUCTScore(node.getNumVisits(), bias);
			if (score == Double.MAX_VALUE) {
				return succNode;
			}
			if (score > maxScore) {
				maxScore = score;
				bestSucc = succNode;
			}
		}
		return bestSucc;
	}
	
	public UCTNode getBestUCTSucc(UCTNode node, double bias) {
		UCTNode bestSucc = null;
		if (node.isExpanded()) {
			double score, maxScore = -Double.MAX_VALUE;
			UCTNode[] succNodes = node.getSuccNodes();
			
			List<UCTNode> shuffledSuccs = Arrays.asList(succNodes);
			Collections.shuffle(shuffledSuccs);
			
			bestSucc = null;
			for (UCTNode succNode : shuffledSuccs) {
				score = succNode.getUCTScore(node.getNumVisits(), bias);
				if (score == Double.MAX_VALUE) {
					return succNode;
				}
				if (score > maxScore) {
					maxScore = score;
					bestSucc = succNode;
				}
			}
		} else {
			int i;
			try {
				modelGen.exploreState(node.getState());
				i = randomGen.nextInt(modelGen.getNumChoices());
				bestSucc = new UCTNode(node.state, i, modelGen.getChoiceAction(i).toString(), -1, false);
			} catch (PrismException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return bestSucc;
	}
	
	
	
	public double getProgressionRew(State source, State target) {
		double prog = 0;
		for (int i = 0; i < nSpecs; i++) {
			int daStateIndex = daStateIndices.get(i);
			DA<BitSet, ? extends AcceptanceOmega> da = automata.get(i);
			int daSource = (int)source.varValues[daStateIndex];
			int daTarget = (int) target.varValues[daStateIndex];
			prog = prog + weights.get(i)*(da.getDistsToAcc().get(daSource) - da.getDistsToAcc().get(daTarget));
		}
		//return Math.max(prog, 0);
		return prog;
	}
	
	
/*	public double getProgressionRew(State source, State target) {		
		int daSource = (int)source.varValues[daStateIndex];
		int daTarget = (int) target.varValues[daStateIndex];
		double prog = 100*(da.getDistsToAcc().get(daSource) - da.getDistsToAcc().get(daTarget));
		//if (prog < 0.0) 	System.out.println(prog);
		return Math.max(prog, 0);
		//return prog;
	}
	
	
	public double getProgressionRew2(State source, State target) {		
		int daSource = (int)source.varValues[daStateIndex];
		int daTarget = (int) target.varValues[daStateIndex];
		if (PrismUtils.doublesAreEqual(da.getDistsToAcc().get(daTarget), 0.0)) {
			//return 0.0;
			return 100;
		} else {
			//double prog = 100*(da.getDistsToAcc().get(daSource) - da.getDistsToAcc().get(daTarget));
			//if (prog < 0.0) 	System.out.println(prog);
			//return Math.max(prog, 0);
			return 0.0;
			//return prog;
		}
	}*/
	
	public UCTNode sampleSucc(UCTNode node) {
		int i, numSuccs;
		double sampled, currentProbSum = 0.0, prob;
		UCTNode currentSucc = null;
		UCTNode[] succs;
		sampled = randomGen.nextDouble();		
		if (node.isExpanded()) {
			
			numSuccs = node.getNumSuccs();
			succs = node.getSuccNodes();
			for (i = 0; i < numSuccs; i++) {
				currentSucc = succs[i];
				currentProbSum = currentProbSum + currentSucc.getReachProb();
				if (currentProbSum >= sampled) {
					return currentSucc;
					//break;
				}
			}
			System.out.println("FODA_SE");			
		} else {
			try {
				modelGen.exploreState(node.getState());
				numSuccs = modelGen.getNumTransitions(node.getAction());
				for (i = 0; i < numSuccs; i++) {
					prob = modelGen.getTransitionProbability(node.getAction(), i);
					currentProbSum = currentProbSum + prob;
					if (currentProbSum >= sampled) {
						State succState = modelGen.computeTransitionTarget(node.getAction(), i);
						return new UCTNode(succState, node.getAction(), null, prob, true);
					}
				}
			} catch (PrismException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
		return currentSucc;
	}
	
	public double rollout(UCTNode node, int depth, double bias, boolean expand) throws PrismException {
		double res = 0.0;
		UCTNode succNode;
		
		if (depth == 0) {
			return 0.0;
		}
		
/*
		if (!node.isExpanded() && expand) {
  			modelGen.exploreState(node.getState());
			expandNode(node);
			if (node.isDecisionNode()) {
				expand = false;
			}
		} */
		
		if (!node.isExpanded()) {
			expandNode(node);
			//depth = 1;
		}

		node.incrementNumVisits();
		if (node.isDecisionNode()) {
			succNode = getBestUCTSucc(node, bias);
			if (succNode == null) {
				depth = 0;
			} else {
//				modelGen.exploreState(succNode.getState());
//				String actionString =  modelGen.getTransitionAction(succNode.getAction(), 0).toString();
//				modelGen.exploreState(node.getState());
//				res = getReward(node.getState(), actionString);
			}
		} else {
			//String actionString =  modelGen.getChoiceAction(node.getAction()).toString();
			//res = getReward(node.getState(), actionString);
			succNode = sampleSucc(node);
			res = getProgressionRew(node.getState(), succNode.getState());
			depth = depth - 1;
			
		}
		bias = 2;//(node.getExpectedRewEstimate()) + 0.001;
		res = res + rollout(succNode, depth, bias, expand);
		node.updateExpectedRewEstimate(res);
	
		return res;

	}
	
	public UCTNode search() throws PrismException
	{

		
		mainLog.println("\nRunning UCT...");
		UCTNode initNode = new UCTNode(initialState, -1, null, 1, true);
		for (int i = 0; i < 50000; i++) {
//			System.out.println("NODE" + initNode);
			double bias = 2;//initNode.getExpectedRewEstimate();
			if (i%5000 == 0)  System.out.println(initNode.getExpectedRewEstimate());
			//double bias = 10;
			rollout(initNode, this.depth, bias, true);
//			System.out.println(":_____________________");
		}
//		System.out.println("PILA" + initNode.getExpectedRewEstimate());
//		UCTNode finalNode = getBestPolicy(initNode);
		return initNode;
	}
	
	
	public MDP buildDTMC(UCTNode node) throws PrismException{
		DTMCSimple dtmc = new DTMCSimple();
		MDPSimple mdp = new MDPSimple();
		int i, currentStateIndex, succStateIndex, nSuccs;
		double max, prob;
		Distribution distr;
		String action;
		UCTNode currentNode, currentSucc, bestSucc, succs[];
		Deque<UCTNode> nodeQueue = new ArrayDeque<UCTNode>();
		Deque<Integer> indexQueue = new ArrayDeque<Integer>();
		State currentState;
		List<State> statesList = new ArrayList<State>();
		
		VarList varList = modelGen.createVarList();
		dtmc.setVarList(varList);
		mdp.setVarList(varList);
		
		nodeQueue.add(node);
		currentStateIndex = dtmc.addState();
		mdp.addState();
		dtmc.addInitialState(currentStateIndex);
		mdp.addInitialState(currentStateIndex);
		indexQueue.add(currentStateIndex);
		statesList.add(node.getState());
		
		
		
		//getBestPolicy(node);
		
				
		while(!nodeQueue.isEmpty()) {
			currentNode = nodeQueue.poll();
			currentStateIndex = indexQueue.poll();
			
			//find succ corresponding to best decision
			succs = currentNode.getSuccNodes();
			nSuccs = currentNode.getNumSuccs();
			max = -Double.MAX_VALUE;
			bestSucc = null;
			action = null;
			for (i = 0; i < nSuccs; i++) {
				currentSucc = succs[i];
				if (currentSucc.getExpectedRewEstimate() > max) {
					bestSucc = currentSucc;
					max = currentSucc.getExpectedRewEstimate();
					action = currentSucc.actionName;
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
						succStateIndex = dtmc.addState();
						mdp.addState();
						statesList.add(currentState);
					}
					prob = currentSucc.getReachProb();
					distr.add(succStateIndex, prob);

	
					dtmc.setProbability(currentStateIndex, succStateIndex, prob);
	
					nodeQueue.add(currentSucc);
					indexQueue.add(succStateIndex);
									
					//System.out.println(currentState);
				}
				//System.out.println(currentStateIndex);
				//System.out.println(distr);
				//System.out.println(action);
				mdp.addActionLabelledChoice(currentStateIndex, distr, action);
				
			}
			
			
			
		}
	
		mdp.setStatesList(statesList);
		dtmc.setStatesList(statesList);
		System.out.println(varList.getName(0));
		
		
		mdp.exportToDotFile("/home/bruno/Desktop/mdp.dot");
		dtmc.exportToDotFile("/home/bruno/Desktop/dtmc.dot");
		
		return mdp;
	}
	
	
	public UCTNode getBestPolicy(UCTNode node) throws PrismException{
		if (!node.isDecisionNode()) {
			modelGen.exploreState(node.state);
			System.out.println(modelGen.getTransitionAction(node.getAction(),0).toString());
			System.out.println(modelGen.getChoiceAction(node.getAction()).toString());
			System.out.println("_________________");
		}
		double max = -Double.MAX_VALUE;
		UCTNode bestNextNode = null;
		for (int i = 0; i < node.getNumSuccs(); i++) {
			UCTNode currentNode = node.getSuccNodes()[i];
			//if (currentNode.getNumVisits() > max) {
			if (currentNode.getExpectedRewEstimate() > max) {
				bestNextNode = currentNode;
				max = currentNode.getExpectedRewEstimate();
				//max = currentNode.getNumVisits();
			}
		}
		if (bestNextNode != null) {
			return getBestPolicy(bestNextNode);
		} else {
			return node;
		}
	}
	
	
}
	
