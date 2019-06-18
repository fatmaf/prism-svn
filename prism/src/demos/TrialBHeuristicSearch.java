package demos;

import java.util.ArrayList;
import java.util.Map.Entry;

import demos.MultiAgentProductModelGenerator.RewardCalculation;
import parser.State;
import prism.PrismException;
import prism.PrismLog;

public class TrialBHeuristicSearch
{

	MultiAgentProductModelGenerator maProdModGen;
	ActionSelection actionSelection;
	OutcomeSelection outcomeSelection;
	HeuristicFunction heuristicFunction;
	BackupFunction backupFunction;
	PrismLog mainLog;
	boolean addAllActions; //a flag that is true for RTDP stuff 

	//literally going to follow what the thts paper describes 
	//so yeah 
	public TrialBHeuristicSearch(PrismLog ml, MultiAgentProductModelGenerator mapmg, ActionSelection actSel, OutcomeSelection outSel, HeuristicFunction hFunc,
			BackupFunction backupFunc)
	{
		maProdModGen = mapmg;
		actionSelection = actSel;
		outcomeSelection = outSel;
		heuristicFunction = hFunc;
		//this is really the bounds stuff 
		backupFunction = backupFunc;
		mainLog = ml;
		//so our default is an RTDP variant 
		addAllActions = true;
	}

	public boolean isAddAllActions()
	{
		return addAllActions;
	}

	public void setAddAllActions(boolean addAllActions)
	{
		this.addAllActions = addAllActions;
	}

	public Object doTHTS() throws PrismException
	{
		DecisionNode n0 = getRootNode();
		while (!isSolved(n0) && !isTimedOut()) {
			visitDecisionNode(n0);
		}
		//not set to greedy action 
		return actionSelection.selectAction(n0.getState());

	}

	ChanceNode createChanceNode(DecisionNode dn, Object act, ArrayList<Entry<State, Double>> successors) throws PrismException
	{
		//Step one create a chance node
		//chance nodes have q values 
		//so first we just do like a basic thing 
		ChanceNode cn = new ChanceNode(dn, dn.getState(), act);
		ArrayList<DecisionNode> dns = new ArrayList<DecisionNode>();
		for (Entry<State, Double> succState : successors) {
			DecisionNode sdn = createNodeFromState(cn, succState.getKey(), succState.getValue());
			dns.add(sdn);
		}
		//now we get the bounds 
		heuristicFunction.calculateBounds(dn.getState(), act, dns);

		Bounds prob = heuristicFunction.getProbabilityBounds();
		Bounds prog = heuristicFunction.getProgressionBounds();

		ArrayList<Bounds> costs = heuristicFunction.getRewardBounds();
		cn.updateBounds(prob, prog, costs);
		return cn;

	}

	void visitDecisionNode(DecisionNode dn) throws PrismException
	{
		if (dn != null) {
			if (dn.visited() == 0) {
				//set its children and other things 
				//the brtdp way 
				if (this.addAllActions) {
					//add all the actions 
					ArrayList<Object> actionsList = maProdModGen.getJointActions(dn.getState());
					for (Object action : actionsList) {
						ArrayList<Entry<State, Double>> successors = maProdModGen.getSuccessors(dn.getState(), action);
						ChanceNode cn = createChanceNode(dn, action, successors);
						dn.addChild(action, cn);
					}
				} else {
					throw new PrismException("Not Implemented!!");
				}

			}
			Object a = actionSelection.selectActionBound(dn,false);
			ChanceNode c = dn.getChild(a);
			visitChanceNode(c);
			//		for (ChanceNode c : children) {
			//			visitChanceNode(c);
			//		}
			backupFunction.backup(dn);
		}
	}

	void visitChanceNode(ChanceNode c) throws PrismException
	{
		if (c != null) {
			ArrayList<DecisionNode> children = outcomeSelection.selectOutcome(c);
			for (DecisionNode d : children) {
				visitDecisionNode(d);
			}
			backupFunction.backup(c);
		}
	}

	boolean isSolved(THTSNode n) throws PrismException
	{
		//		throw new PrismException
		mainLog.println("isSolved not Implemented!");
		return false;
	}

	boolean isTimedOut() throws PrismException
	{
		//		throw new PrismException
		mainLog.println("time out not Implemented!");
		return false;
	}

	DecisionNode getRootNode() throws PrismException
	{
		//just get the initial states 
		ArrayList<State> initStates = maProdModGen.createInitialStateFromRobotInitStates();
		//now just use the first initial state 
		State s = initStates.get(0);
		return createNodeFromState(null, s, 1.0);

	}

	DecisionNode createNodeFromState(THTSNode ps, State s, double tprob) throws PrismException
	{
		heuristicFunction.calculateBounds(s);

		Bounds prob = heuristicFunction.getProbabilityBounds();
		Bounds prog = heuristicFunction.getProgressionBounds();

		ArrayList<Bounds> costs = heuristicFunction.getRewardBounds();

		DecisionNode dn = new DecisionNode(ps, s, tprob, prob, prog, costs);
		return dn;
	}

}
