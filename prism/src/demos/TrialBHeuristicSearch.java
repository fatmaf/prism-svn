package demos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.concurrent.TimeUnit;
import java.util.Stack;

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
	boolean forwardBackup;
	ArrayList<Objectives> tieBreakingOrder;
	boolean lrtdp;
	float errorClearance = 0.00001f;
	int maxTrialLength = 200;
	int maxRollOuts = 1000;
	int currentTrailLength = 0;
	int numRollOut = 0; 
	String saveLocation;
	String fn;

	//saving nodes so I dont create new ones all the time 
	HashMap<String, THTSNode> nodesAddedSoFar;

	//literally going to follow what the thts paper describes 
	//so yeah 
	public TrialBHeuristicSearch(PrismLog ml, MultiAgentProductModelGenerator mapmg, ActionSelection actSel, OutcomeSelection outSel, HeuristicFunction hFunc,
			BackupFunction backupFunc, ArrayList<Objectives> tieBreakingOrder, String saveLocation, String fn)
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
		//for brtdp only 
		forwardBackup = true;
		lrtdp = true;
		this.tieBreakingOrder = tieBreakingOrder;
		nodesAddedSoFar = new HashMap<String, THTSNode>();
		this.saveLocation = saveLocation;
		this.fn = fn;
	}

	//set trial length 
	public int setMaxTrialLength()
	{
		maxTrialLength = maProdModGen.getMaxStatesEstimate(); 
			
		//TODO:Implement this
		return maxTrialLength;
	}
	public THTSNode addNodeToHash(THTSNode n)
	{
		String k = n.getShortName();
		return addNodeToHash(n, k);

	}

	public THTSNode addNodeToHash(THTSNode n, String k)
	{
		THTSNode nodeInMap = n;
		boolean added = false;
		if (!nodesAddedSoFar.containsKey(k)) {
			nodesAddedSoFar.put(k, n);
			added = true;
		} else {
			nodeInMap = nodesAddedSoFar.get(k);
		}
		return nodeInMap;
	}

	public boolean checkNodeInHash(String k)
	{
		return nodesAddedSoFar.containsKey(k);
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
		long start = System.nanoTime();

		DecisionNode n0 = getRootNode();
		while (!n0.isSolved() && !isRolloutTimedOut()) {
			this.numRollOut++;
			visitDecisionNode(n0);
		}
		long elapsedTime = System.nanoTime() - start;
		mainLog.println("Solved after " + this.currentTrailLength + " iterations. " + elapsedTime + "ns "
				+ TimeUnit.MILLISECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS) + " ms " + TimeUnit.SECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS)
				+ "s");

		PolicyCreator pc = new PolicyCreator();
		pc.createPolicy(n0, actionSelection, false);
		pc.savePolicy(saveLocation, fn + "_thtsPolicy");
		return actionSelection.selectActionBound(n0, false);

	}

	ChanceNode createChanceNode(DecisionNode dn, Object act, ArrayList<Entry<State, Double>> successors) throws PrismException
	{
		ChanceNode cn = null;
		String k = dn.getState().toString() + act.toString();
		if (checkNodeInHash(k)) {
			cn = (ChanceNode) getNodeFromHash(k);
			//now do you update the abu ? 

		}
		//Step one create a chance node
		//chance nodes have q values 
		//so first we just do like a basic thing 
		cn = new ChanceNode(dn, dn.getState(), act);
		ArrayList<DecisionNode> dns = new ArrayList<DecisionNode>();
		for (Entry<State, Double> succState : successors) {
			DecisionNode sdn = createDecisionNode(cn, succState.getKey(), succState.getValue());
			dns.add(sdn);
		}
		cn.setChildren(dns);
		//now we get the bounds 
		heuristicFunction.calculateBounds(dn.getState(), act, dns, cn);

		Bounds prob = heuristicFunction.getProbabilityBounds();
		Bounds prog = heuristicFunction.getProgressionBounds();

		ArrayList<Bounds> costs = heuristicFunction.getRewardBounds();
		cn.updateBounds(prob, prog, costs);
		addNodeToHash(cn);
		return cn;

	}

	void addAllChanceNodes(DecisionNode dn) throws PrismException
	{
		//add all the actions 
		ArrayList<Object> actionsList = maProdModGen.getJointActions(dn.getState());
		for (Object action : actionsList) {
			ArrayList<Entry<State, Double>> successors = maProdModGen.getSuccessors(dn.getState(), action);
			ChanceNode cn = createChanceNode(dn, action, successors);
			dn.addChild(action, cn);
		}
	}

	void addBachayIfNeeded(DecisionNode dn) throws PrismException
	{
//		if (dn != null) {
//			if (dn.getState().toString().contains("(0),(3),(-1),(3)"))
//				mainLog.println("BC");
//		}
		if (dn.visited() == 0) {
			//set its children and other things 
			//the brtdp way 
			if (this.addAllActions) {

				addAllChanceNodes(dn);

			} else {
				throw new PrismException("Not Implemented!!");
			}

		} else {
			if (dn.getChildren() == null & !dn.isGoal & !dn.isDeadend)
				throw new PrismException("Its visited but has no bachay ? ");
		}
	}

	boolean visitDecisionNode(DecisionNode dn) throws PrismException
	{

		currentTrailLength++;
		boolean dobackup = true;
		if (dn != null && !isTrialTimedOut()) {

//			mainLog.println(dn.toString());
//			if (dn.getState().toString().contains("(0),(1),(-1),(1)"))
//				mainLog.println("Ewwror");
			if (dn.isDeadend | dn.isGoal) {
				if (!dn.isSolved())
					dn.setSolved();
				//				mainLog.println("Goal State reached");

			} else {
				addBachayIfNeeded(dn);
				dn.increaseVisits();
				Object a = actionSelection.selectActionBound(dn, false);

				if (forwardBackup) {
					//backup here for brtdp stuff 
					//cuz we back up twice 

					backupFunction.backup(dn);
				}
				ChanceNode c = dn.getChild(a);
				dobackup = visitChanceNode(c);
				//		for (ChanceNode c : children) {
				//			visitChanceNode(c);
				//		}
			}
			if (lrtdp) {
				if (dobackup) {
					dobackup = checkSolved(dn, errorClearance);
				}
			} else
				backupFunction.backup(dn);
		}
		return dobackup;
	}

	boolean visitChanceNode(ChanceNode c) throws PrismException
	{
		boolean dobackup = true;
		if (c != null & !isTrialTimedOut()) {
//			mainLog.println(c.toString());
			c.increaseVisits();
			ArrayList<DecisionNode> children = outcomeSelection.selectOutcome(c);
			if (forwardBackup) {
				backupFunction.backup(c);
			}
			for (DecisionNode d : children) {
				dobackup = dobackup & visitDecisionNode(d);
			}
			backupFunction.backup(c);
		}
		return dobackup;
	}

	boolean checkSolved(DecisionNode n, float errorClearance) throws PrismException
	{
		boolean upperBound = false;

		ActionSelection actSel = this.actionSelection;
		if (!(actSel instanceof ActionSelectionGreedy))
			actSel = new ActionSelectionGreedy(this.tieBreakingOrder);

		boolean stateSolved = true;
		Stack<DecisionNode> open = new Stack<DecisionNode>();
		Stack<DecisionNode> closed = new Stack<DecisionNode>();

		if (!n.isSolved())
			open.push(n);
		while (!open.isEmpty()) {
			n = open.pop();
			closed.push(n);
			addBachayIfNeeded(n);
			if (backupFunction.residual((DecisionNode) n, upperBound, errorClearance) > errorClearance) {
				stateSolved = false;
				continue;
			}
			Object action = actSel.selectActionBound(n, upperBound);
			ArrayList<DecisionNode> dns = n.getChild(action).getChildren();
			for (DecisionNode dn : dns) {
				if (!dn.isSolved() & !open.contains(dn) & !closed.contains(dn)) {
					open.push(dn);
				}
			}
		}
		if (stateSolved) {
			while (!closed.isEmpty()) {
				n = closed.pop();
				n.setSolved();

			}
		} else {
			while (!closed.isEmpty()) {
				n = closed.pop();
				backupFunction.backup(n);
			}
		}
		return stateSolved;
	}

	boolean isTrialTimedOut() throws PrismException
	{
//		return this.numRollOut >= this.maxRollOuts;
		return this.currentTrailLength > this.maxTrialLength;
	}

	boolean isRolloutTimedOut() 
	{
		return this.numRollOut > this.maxRollOuts;
	}
	
	DecisionNode getRootNode() throws PrismException
	{
		//just get the initial states 
		ArrayList<State> initStates = maProdModGen.createInitialStateFromRobotInitStates();
		//now just use the first initial state 
		State s = initStates.get(0);
		return createDecisionNode(null, s, 1.0);

	}

	DecisionNode createDecisionNode(THTSNode ps, State s, double tprob) throws PrismException
	{
//		if (ps != null) {
//			if (ps.getState().toString().contains("(0),(3),(-1),(3)"))
//				mainLog.println("BC");
//		}
		DecisionNode dn;
		//check if this node exists 
		String k = s.toString();
		if (checkNodeInHash(k)) {
			dn = (DecisionNode) getNodeFromHash(k);
			dn.addParent(ps,tprob);
			return dn;

		}

		heuristicFunction.calculateBounds(s);

		Bounds prob = heuristicFunction.getProbabilityBounds();
		Bounds prog = heuristicFunction.getProgressionBounds();

		ArrayList<Bounds> costs = heuristicFunction.getRewardBounds();

		boolean deadend = maProdModGen.isDeadend(s);
		boolean goal = maProdModGen.isGoal(s);
//		if(deadend)
//			mainLog.println("Deadend: "+s.toString());
		dn = new DecisionNode(ps, s, tprob, prob, prog, costs, deadend, goal);
		addNodeToHash(dn);
		return dn;
	}

	private THTSNode getNodeFromHash(String k)
	{
		return this.nodesAddedSoFar.get(k);
	}

}
