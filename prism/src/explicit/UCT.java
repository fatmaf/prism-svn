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

import java.util.Arrays;
import java.util.BitSet;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import parser.ast.RewardStruct;
import parser.Values;
import parser.State;
import prism.*;


/**
 * Implementation of the upper confidence bounds applied to trees (UCT) algorithm
 */
public final class UCT extends PrismComponent
{
	protected double termCritParam = 1e-8;
	public static final int UNK_STATE = -1;
	public static final int SINK_STATE = 0;
	public static final int ACC_STATE = 1;
	
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
		/**dist from the the DA component of this state to the DA target state*/
		private double stateCost;
		/** number of rollouts that have visited this node */
		private int numVisits;
		/** Have the succs of this node been computed*/
		private boolean expanded;
		/** expected reward estimate from this node, based on the previous rollouts */
		private double expectedRewEstimate;
		/** successor UTC nodes */
		private UCTNode[] succNodes;
		/** the depth of this node */
		private int depth;
		private int stateType;
		private boolean useBasePolicy;
		
		/**
		 * Constructs an UCT search node object.
		 * 
		 * @param state The MDP state representation
		 * @param action The action that brought the UCT search to this node.
		 */
		public UCTNode(State state, int action, String actionName, double reachProb, double stateCost, boolean decision, int depth)
		{
			this.decision = decision;
			this.state = state;
			this.action = action;
			this.actionName = actionName;
			this.reachProb = reachProb;
			this.stateCost = stateCost;
			this.numVisits = 0;
			this.expanded = false;
			this.expectedRewEstimate = Double.MAX_VALUE;
			this.succNodes = null;
			this.depth = depth;
			this.stateType = UNK_STATE;
			this.useBasePolicy = true;
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
		
		
		public boolean useBasePolicy()
		{
			return useBasePolicy;
		}
		
		public void clearUseBasePolicy()
		{
			useBasePolicy = false;
		}
		
		public boolean isDecisionNode()
		{
			return decision;
		}
		
		public double getStateCost()
		{
			return stateCost;
		}
		
		public double getUCTScore(double parentVisits, double bias, boolean useDistCost)
		{
			if (numVisits == 0) {
				return Double.MAX_VALUE;
			} else {
				if (bias < 10) {
					bias = 10;
				}
				if (useDistCost) {
					return -bias*Math.sqrt(Math.log(parentVisits)/numVisits) + expectedRewEstimate;
				} else {
					return bias*Math.sqrt(Math.log(parentVisits)/numVisits) + expectedRewEstimate;
				}
			}
		}
		
		public int getDepth()
		{
			return depth;
		}
		
		public void setStateType(int stateType)
		{
			this.stateType = stateType;
		}
		
		public int getStateType()
		{
			return stateType;
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
//	/** initial state of the model */
	private State initState;
	
	/**
	 * DA dists to target
	 */
	List<Double> daDists;
	BitSet daSinks;
	boolean useDistCost;
	MDPSimple basePolicy;

	/**
	 * Constructor.
	 */
	public UCT(PrismComponent parent, ProductModelGenerator modelGen, State initState, List<Double> daDists, BitSet daSinks, int depth, int nSamples, boolean useDistCost, MDPSimple basePolicy) throws PrismException
	{
		super(parent);
		
		this.modelGen = modelGen;
		this.depth = depth;
		this.nSamples = nSamples;
		rewStruct = null;
		constantValues = null;
		randomGen = new Random();
		this.initState = initState;
		this.daDists = daDists;
		this.daSinks = daSinks;
		this.useDistCost = useDistCost;
		this.basePolicy = basePolicy;
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
	
	public double getStateCost(State state) {
		int daVal = (int)state.varValues[modelGen.getNumVars() - 1];
		double res = daDists.get(daVal);
		return res;
	}
	
	public double getProgRew(State source, State target) {
		int sourceDaVal = (int)source.varValues[modelGen.getNumVars() - 1];
		int targetDaVal = (int)target.varValues[modelGen.getNumVars() - 1];
		double res = daDists.get(sourceDaVal) - daDists.get(targetDaVal);
		if (res > 0) {
			return res*100;
		} else {
			return res*100;
		}
	}
	
	public boolean isDASink(State state) {
		return daSinks.get((int)state.varValues[modelGen.getNumVars() - 1]);
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
				succNodes[i] = new UCTNode(node.getState(), i, modelGen.getChoiceAction(i).toString(), -1, getStateCost(node.getState()), false, node.getDepth());
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
				succNodes[i] = new UCTNode(succState, node.getAction(), null, prob, getStateCost(succState), true, node.getDepth()-1);
				
			}
		}
		node.setSuccNodes(succNodes);
		node.setAsExpanded();
	}
	
	public UCTNode getBestUCTSucc(UCTNode node, double bias) {
		UCTNode bestSucc = null;
		double score, bestScore;
		if (useDistCost) {
			bestScore =  Double.MAX_VALUE;
		} else {
			bestScore = - Double.MAX_VALUE;
		}
		UCTNode[] succNodes = node.getSuccNodes();
		
		List<UCTNode> shuffledSuccs = Arrays.asList(succNodes);
		Collections.shuffle(shuffledSuccs);
		
		bestSucc = null;
		for (UCTNode succNode : shuffledSuccs) {
			score = succNode.getUCTScore(node.getNumVisits(), bias, useDistCost);
			if (score == Double.MAX_VALUE) {
				return succNode;
			}
			if (useDistCost) {
				if (score < bestScore) {
					bestScore = score;
					bestSucc = succNode;
				}
			} else {
				if (score > bestScore) {
					bestScore = score; 
					bestSucc = succNode;
				}
			}
		}
		return bestSucc;
	}

	public UCTNode sampleSucc(UCTNode node) {
		int i, numSuccs;
		double sampled, currentProbSum = 0.0;
		UCTNode currentSucc = null;
		UCTNode[] succs;
		sampled = randomGen.nextDouble();		
			
		numSuccs = node.getNumSuccs();
		succs = node.getSuccNodes();
		for (i = 0; i < numSuccs; i++) {
			currentSucc = succs[i];
			currentProbSum = currentProbSum + currentSucc.getReachProb();
			if (currentProbSum >= sampled) {
				return currentSucc;
			}
		}
		System.out.println("FODA_SE");
		return currentSucc;
	}
	
	public double rollout(UCTNode node, int depth, double bias, boolean first, boolean selectionFinished) throws PrismException {
		double res = 0.0;
		UCTNode succNode = null;
		
		if (depth == 0) {
			return 0;
		}
		
		if (!node.isExpanded()) {
			expandNode(node);
			if (!node.isDecisionNode()) {
				if (!selectionFinished) {
					node.clearUseBasePolicy();
					selectionFinished = true;
				}
			}
		}


		node.incrementNumVisits();
		if (node.isDecisionNode()) {
			if (isDASink(node.getState())) {
				node.setStateType(SINK_STATE);
				if (useDistCost) {
					res = (node.getStateCost() * depth); 
					//res = depth; 
				} else {
					res = 0;
				}; 
				depth = 0;
			} else {
				if (node.getStateCost() == 0 && !first) {
					//System.out.println("REACHED GOAL");
					node.setStateType(ACC_STATE);
					depth = 0;
				}
				else {
					if(node.useBasePolicy()) {
						succNode = getBestUCTSucc(node, bias);
						//succNode = getSuccFromBasePolicy(node);
					} else {
						succNode = getBestUCTSucc(node, bias);
					}
					if (succNode == null) {
						node.setStateType(SINK_STATE);
						if (useDistCost) {
							res = node.getStateCost() * depth;
							//res = depth;
						} else {
							res = 0;
						}
						depth = 0;
					}
				}
			
			}
		} else {
			succNode = sampleSucc(node);
			if (useDistCost) {
				res = node.getStateCost();
			} else {
				if (first) {
					res = 0;
				} else {
					res = getProgRew(node.getState(), succNode.getState());
				}
			}
			depth = depth - 1;
			
		}
		
		if (useDistCost) {
			bias = depth;
			//bias=node.getExpectedRewEstimate();
		} else {
			//bias = depth/this.depth;
			bias=node.getExpectedRewEstimate();
		}
		
		res = res + rollout(succNode, depth, bias, false, selectionFinished);
		node.updateExpectedRewEstimate(res);
		
		/*
		if (selectionFinished) {
			res = res + doSimulation(succNode.getState(), depth);
		} else {
			
		}
		*/
	
		return res;

	}
	
	public UCTNode getSuccFromBasePolicy(UCTNode node) {
		int nSuccs = node.getNumSuccs();
		if (basePolicy == null) {
			//return node.getSuccNodes()[randomGen.nextInt(nSuccs)];
			return getBestUCTSucc(node, 0);
		}
		
		UCTNode succNode = null;
		boolean isPolicyDefined = false;
		int i;

		
		int numStatesPolicy = basePolicy.getNumStates();
		for (i = 0; i < numStatesPolicy; i++) {
			if(node.getState().equals(basePolicy.getStatesList().get(i))) {
				if (basePolicy.getNumChoices(i) == 1) {
					isPolicyDefined = true;
				}
				break;
			}
		}
		
		if (isPolicyDefined) {
			Object action = basePolicy.getAction(i, 0); //There is only one action, this is an MC really.
			for (i = 0; i < nSuccs; i++) {
				succNode = node.getSuccNodes()[i];
				if (succNode.getActionName().equals(action)) {
					return succNode;
				}
			}
		} else {
			return getBestUCTSucc(node, 0);
			//return node.getSuccNodes()[randomGen.nextInt(nSuccs)];
		}

		System.out.println("FODASE FDP");		
		return succNode;
	}
	
	
	
	public UCTNode search() throws PrismException
	{
		if (!modelGen.hasSingleInitialState())
			throw new PrismException("UCT rquires a single initial state");
		mainLog.println("\nRunning UCT...");
		double bias;
		UCTNode initNode = new UCTNode(initState, -1, null, 1, getStateCost(initState), true, depth);
		for (int i = 0; i < this.nSamples; i++) {
			if (useDistCost) {
				//double bias = 0.1*initNode.getExpectedRewEstimate();
				bias = this.depth;
			} else {
				bias = initNode.getExpectedRewEstimate();
				//bias = 1;
			}
			//long time = System.currentTimeMillis();
			rollout(initNode, this.depth, bias, true, false);
			//time = System.currentTimeMillis() - time;
			//mainLog.println("Sample time: " + time + " mseconds.");
//			System.out.println(":_____________________");
		}
//		System.out.println("PILA" + initNode.getExpectedRewEstimate());
//		UCTNode finalNode = getBestPolicy(initNode);
		return initNode;
	}
	
	/* 	METHODS FOR ONLY ADDING ONE SEARCH NODE PER ROLLOUT. THEY ARE MUCH LESS EFFICIENT THOUGH, I THINK BECAUSE OF OVERHEAD CALLING THE MODEL GENERATOR
	public double doSimulation(State state, int depth) throws PrismException {
		double res = 0;
		State nextState;
		for (int i = 0; i < depth; i++) {
			nextState = getNextSimState(state);
			if (nextState == null) {
				break;
			} else {
				res = res + getProgRew(state, nextState);
				state = nextState;
			}
		}
		return res;
	}
	
	public State getNextSimState(State state) throws PrismException {
		modelGen.exploreState(state);
		int actionIndex = randomGen.nextInt(modelGen.getNumChoices());
		double sampled, currentProbSum = 0.0;
		sampled = randomGen.nextDouble();
			
		int numSuccs = modelGen.getNumTransitions(actionIndex);
		for (int i = 0; i < numSuccs; i++) {
			
			currentProbSum = currentProbSum + modelGen.getTransitionProbability(actionIndex, i);
			if (currentProbSum >= sampled) {
				return  modelGen.computeTransitionTarget(actionIndex, i);
			}
		}
		System.out.println("PILA");
		return null;
	}*/
	
}
