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
	public final static class UCTNode
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
		public UCTNode(State state, int action, String actionName, double reachProb, boolean decision)
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
		public State getState()
		{
			return state;
		}
		
		/**
		 * Gets the action that was executed to reach this UCT node
		 * 
		 * @return action
		 */
		public int getAction()
		{
			return action;
		}
		
		/**
		 * Gets the action (string) that was executed to reach this UCT node
		 * 
		 * @return action
		 */
		public String getActionName()
		{
			return actionName;
		}
		
		/**
		 * Gets the probability to reach this UCT node from its parent
		 * 
		 * @return action
		 */
		public double getReachProb()
		{
			return reachProb;
		}
		
		
		/**
		 * Gets the number of times this node has been visited
		 * 
		 * @return MDP state
		 */
		public int getNumVisits()
		{
			return numVisits;
		}
		
		/**
		 * Gets the  estimate for the expected reward for this node, based on previous rollouts
		 * 
		 * @return MDP state
		 */
		public double getExpectedRewEstimate()
		{
			return expectedRewEstimate;
		}
		
		/**
		 * Gets the children of this UTC node
		 * 
		 * @return MDP state
		 */
		public UCTNode[] getSuccNodes()
		{
			return succNodes;
		}
	

		/**
		 * Increment number of visits to this node
		 * 
		 */
		public void incrementNumVisits()
		{
			this.numVisits = this.numVisits + 1;
		}
		
		/**
		 * Update reward
		 * 
		 * @param reward state reward to set
		 */
		public void updateExpectedRewEstimate(double reward)
		{
			this.expectedRewEstimate = ((this.numVisits - 1)*this.expectedRewEstimate + reward)/(this.numVisits);
		}
		

		/**
		 * Sets successor UCT nodes (i.e., children of this node).
		 * 
		 * @param succNodes successor nodes
		 */
		public void setSuccNodes(UCTNode[] succNodes)
		{
			this.succNodes = succNodes;
		}

		/**
		 * Returns number of successor states of this state.
		 * 
		 * @return number of successor states
		 */
		public int getNumSuccs()
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
		public boolean hasSuccs()
		{
			return succNodes != null;
		}

		
		/**
		 * Checks whether this node has been visited
		 * 
		 * @return whether this node has been visited
		 */
		public boolean isExpanded()
		{
			return expanded;
		}
		
		public void setAsExpanded()
		{
			expanded = true;
		}
		
		
		public boolean isDecisionNode()
		{
			return decision;
		}
		
		public double getUCTScore(double parentVisits, double bias)
		{
			if (numVisits == 0) {
				return Double.MAX_VALUE;
			} else {
				return bias*Math.sqrt(Math.log(parentVisits)/numVisits) + expectedRewEstimate;
			}
		}
	}
		



	/** model exploration component to generate new states */
	private ProductModelGenerator modelGen;
	/** rollout depth */
	private int depth;
	
	/** number of rollouts */
	private int nSamples;
	
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
	public UCT(PrismComponent parent, ProductModelGenerator modelGen, int depth, int nSamples) throws PrismException
	{
		super(parent);
		
		this.modelGen = modelGen;
		this.depth = depth;
		this.nSamples = nSamples;
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
		this.rewStruct = rewStruct;
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
	
	public UCTNode getBestUCTSucc(UCTNode node, double bias) {
		UCTNode bestSucc = null;
		if (node.isExpanded()) {
			double score, minScore = Double.MAX_VALUE;
			UCTNode[] succNodes = node.getSuccNodes();
			
			List<UCTNode> shuffledSuccs = Arrays.asList(succNodes);
			Collections.shuffle(shuffledSuccs);
			
			bestSucc = null;
			for (UCTNode succNode : shuffledSuccs) {
				score = succNode.getUCTScore(node.getNumVisits(), bias);
				if (score == Double.MAX_VALUE) {
					return succNode;
				}
				if (score < minScore) {
					minScore = score;
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
	
	public double getReward(State state, String action) throws PrismException{
		//System.out.println("ACTION " + action);
		int numStateItems = rewStruct.getNumItems();
		double res = 0.0;
		for (int i = 0; i < numStateItems; i++) {
			Expression guard = rewStruct.getStates(i);
			if (guard.evaluateBoolean(constantValues, state)) {
				String currentAction = rewStruct.getSynch(i);
				if (action.equals(currentAction)) {
					res = res + rewStruct.getReward(i).evaluateDouble(constantValues, state);
				}
			}
		}
		return res;
	}
	

	
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
	
	public double rollout(UCTNode node, int depth, double bias) throws PrismException {
		double res = 0.0;
		UCTNode succNode;
		
		if (depth == 0) {
			return 0;
		}
		modelGen.exploreState(node.getState());
		if (!node.isExpanded()) {
			expandNode(node);
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
			succNode = sampleSucc(node);
			res = modelGen.getProgressionRew(node.getState(), succNode.getState());
			depth = depth - 1;
			
		}
		res = res + rollout(succNode, depth, bias);
		node.updateExpectedRewEstimate(res);
	
		return res;

	}
	
	public UCTNode search() throws PrismException
	{
		if (!modelGen.hasSingleInitialState())
			throw new PrismException("UCT rquires a single initial state");
		
		mainLog.println("\nRunning UCT...");
		State initState = modelGen.getInitialState();
		UCTNode initNode = new UCTNode(initState, -1, null, 1, true);
		for (int i = 0; i < this.nSamples; i++) {
//			System.out.println("NODE" + initNode);
			//double bias = initNode.getExpectedRewEstimate();
			double bias = 0.1;
			rollout(initNode, this.depth, bias);
//			System.out.println(":_____________________");
		}
//		System.out.println("PILA" + initNode.getExpectedRewEstimate());
//		UCTNode finalNode = getBestPolicy(initNode);
		return initNode;
	}
	
	
	public UCTNode getBestPolicy(UCTNode node) throws PrismException{
		if (!node.isDecisionNode()) {
			modelGen.exploreState(node.state);
			System.out.println(modelGen.getTransitionAction(node.getAction(),0).toString());
			System.out.println(modelGen.getChoiceAction(node.getAction()).toString());
			System.out.println("_________________");
		}
		double max = Double.MIN_VALUE;
		UCTNode bestNextNode = null;
		for (int i = 0; i < node.getNumSuccs(); i++) {
			UCTNode currentNode = node.getSuccNodes()[i];
			if (currentNode.getExpectedRewEstimate() > max) {
				bestNextNode = currentNode;
				max = currentNode.getExpectedRewEstimate();
			}
		}
		if (bestNextNode != null) {
			return getBestPolicy(bestNextNode);
		} else {
			return node;
		}
	}
	
	
}
	
