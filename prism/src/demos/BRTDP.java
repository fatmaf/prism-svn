package demos;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Deque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Map.Entry;

import explicit.Distribution;
import explicit.IndexedSet;
import explicit.MDP;
import explicit.MDPSimple;
import explicit.StateStorage;
import parser.State;
import parser.VarList;
import prism.Pair;
import prism.PrismComponent;
import prism.PrismException;
import prism.PrismUtils;
import prism.ProductModelGenerator;
import prism.ProgressDisplay;
import prism.PrismLog;
import prism.PrismSettings;

public class BRTDP {
	public static final double termCritParam = 1e-8;
	public static final int UNK_STATE = -1;
	public static final int SINK_STATE = 0;
	public static final int ACC_STATE = 1;
	
	public static final String BOUND = "acc";
	public static PrismLog mainLog; 
	protected PrismSettings settings = null; 
	
	public final static class Bound
	{	
		private boolean isMinimisation;
		private boolean isInitialised;
		private double lb;
		private double ub;
		private int numActions;
		private  List<Double> lbQ;
		private  List<Double> ubQ;
		
		public Bound(double lb, double ub, boolean isMinimisation) {
			isInitialised = false;
			this.isMinimisation = isMinimisation;
			this.lb = lb;
			this.ub = ub;

		}
		
		public boolean isInitialised() {
			return isInitialised;
		}
		
		public void initialise(int numActions) {
			this.numActions = numActions;
			lbQ = new ArrayList<Double>(numActions);
			ubQ = new ArrayList<Double>(numActions);
			for (int i = 0; i < numActions; i++) {
				lbQ.add(lb);
				ubQ.add(ub);
			}
			isInitialised = true;
		}
		
		public boolean isMinimisation() {
			return isMinimisation;
		}
		
		public double getLb() {
			return lb;
		}
		
		public double getUb() {
			return ub;
		}
		
		public void setLb(double d) {
			lb = d;
		}
		
		public void setUb(double d) {
			ub = d;
		}
		
		public double getLbQ(int action) {
			return lbQ.get(action);
		}
		
		public double getUbQ(int action) {
			return ubQ.get(action);
		}
		
		public void setLbQ(int action, double val) {
			lbQ.set(action, val);
		}
		
		public void setUbQ(int action, double val) {
			ubQ.set(action, val);
		}

		
		public List<Double> getLbQ() {
			return lbQ;
		}
		
		
		public List<Double> getUbQ() {
			return ubQ;
		}
		
		
		public int getNumActions() {
			return numActions;
		}
	}
	
	public final static class SearchState extends State
	{
		/** number of rollouts that have visited this node */
		private int numVisits;
		/** Have the succs of this node been computed*/
		private boolean expanded;
		/** bounds for probability of reaching the goal, progression, and probability of reaching a sink*/
		private Map<String, Bound> bounds; 
		/** is this state solved, i.e., do we need to update its bounds?*/
		private boolean solved;
		/** number of actions enabled in this state */
		private int numActions = -1;
		/** state index in the MDP list of states */
		private int stateId;
		private double stateCost;
		private boolean isDASink;
		/**
		 * Constructs a BRTDP search state object
		 * 
		 * @param state The MDP state representation
		 * @param action The action that brought the UCT search to this node.
		 */
		public SearchState(State state, int stateId, double stateCost, boolean isDASink) 
		{	
			super(state);
			this.stateId = stateId;
			this.stateCost = stateCost;
			this.isDASink = isDASink;
			expanded = false;
			solved = false;
			bounds = new HashMap<String, Bound>();
			bounds.put("acc", new Bound(0.0, 1.0, false));
			bounds.put("prog", new Bound(0.0, stateCost, false));
			bounds.put("sink", new Bound(0.0, 1.0, true));
			if (stateCost == 0) {
				System.out.println("FOUND ACC");
				bounds.get("acc").setLb(1.0);
				bounds.get("prog").setUb(0.0);
				bounds.get("sink").setUb(0.0);
				solved = true;
			}
			if (isDASink) {
				//System.out.println("FOUND SINK");
				bounds.get("acc").setUb(0.0);
				bounds.get("prog").setUb(0.0);
				bounds.get("sink").setLb(1.0);
				solved = true;
				
			}
		}
		

		public Map<String, Bound> getBounds() {
			return bounds;
		}
		
		public void initialise(int numActions) {
			expanded = true;
			this.numActions = numActions;
			if (numActions == 0) {
				bounds.get("acc").setUb(0.0);
				bounds.get("prog").setUb(0.0);
				bounds.get("sink").setLb(1.0);
				solved = true;
			} else {
				for (Iterator<Entry<String, Bound>> it = bounds.entrySet().iterator(); it.hasNext(); ) {
					it.next().getValue().initialise(numActions);
				}
			}
		}
		

		
		public int getStateId() {
			return stateId;
		}
		
		public void setStateId(int id)
		{
			stateId = id;
			
		}
		
		public double getStateCost() {
			return stateCost;
		}
		
		public boolean isDASink() {
			return isDASink;
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
		
		public boolean isSolved() {
			return solved;
		}
	
		
		public void setSolved() {
			solved = true;
		}

		/**
		 * Increment number of visits to this node
		 * 
		 */
		public void incrementNumVisits()
		{
			this.numVisits = this.numVisits + 1;
		}
		
		
		public List<Integer> getBestActions2(List<String> boundNames, List<Boolean> useUbList, boolean aggregate) {
			if (numActions == -1) {
				return null;
			}
			if (numActions == 0) {
				return null;
			}
			List<Integer> bestActions = new ArrayList<Integer>(numActions);
			for (int i = 0; i < numActions; i++) {
				bestActions.add(i);
			}
			return bestActions;
		}
		
		/**
		 * Constructs a list of best actions according to the current bounds
		 * 
		 * @param boundNames The name of the bounds to take into account
		 * @param useUb Whether to use the lower or the upper bound for the corresponding bound
		 * @param aggregate if true then all optimal actions for all boundNames are returned. If false then we assume a decreasing order of priority and use lower priority bounds to tie break a la nested VI
		 */
		public List<Integer> getBestActions(List<String> boundNames, List<Boolean> useUbList, boolean aggregate) {
			if (numActions == -1) {
				return null;
			}
			if (numActions == 0) {
				return null;
			}
			int numBounds = boundNames.size();
			String boundName;
			boolean useUb;
			List<Integer> currentBestActions;
			List<List<Integer>> bestActions = new ArrayList<List<Integer>>(numBounds);
			for (int i = 0; i < numBounds; i++) {
				currentBestActions = new ArrayList<Integer>(numActions);
				boundName = boundNames.get(i);
				useUb = useUbList.get(i);
				for (int j = 0; j < numActions; j++) {
					if (useUb) {
						if (PrismUtils.doublesAreClose(bounds.get(boundName).getUb(), bounds.get(boundName).getUbQ(j), termCritParam, true)) {
							currentBestActions.add(j);
						}
					} else {
						if (PrismUtils.doublesAreClose(bounds.get(boundName).getLb(), bounds.get(boundName).getLbQ(j), termCritParam, true)) {
							currentBestActions.add(j);
						}
					}
				}
				bestActions.add(currentBestActions);
			}
			
			if (aggregate) {
				return aggregateBestActions(bestActions);
			} else {
				return prioritiseBestActions(bestActions);
			}
		}


		private List<Integer> prioritiseBestActions(List<List<Integer>> bestActions)
		{
			int numBounds = bestActions.size();
			List<Integer> res = new ArrayList<Integer>(bestActions.get(0));
			for (List<Integer> currentBestActions : bestActions.subList(1, numBounds)) {
				for(Iterator<Integer> iterator = res.iterator(); iterator.hasNext(); ) {
					if (!currentBestActions.contains(iterator.next())) {
						iterator.remove();
					}
				}
			}
			if (res.isEmpty()) {
				res = bestActions.get(0); //TODO THis is a cheat to cope with cases where the intersection between optimal actions for different bounds is empty.
			}
			return res;
		}

		private List<Integer> aggregateBestActions(List<List<Integer>> bestActions)
		{
			StateStorage<Integer> bestActionsSet = new IndexedSet<Integer>(true);
			for (List<Integer> currentBestActions : bestActions) {
				for (int actionIndex: currentBestActions) {
					bestActionsSet.add(actionIndex);
				}
			}
			return bestActionsSet.toArrayList();
		}
		
		public void configureAccForSearch() {
			bounds = new HashMap<String, Bound>();
			bounds.put("acc", new Bound(0.0, 1.0, false));
			bounds.put("prog", new Bound(0.0, stateCost, false));
			bounds.put("sink", new Bound(0.0, 1.0, true));
			solved = false;
		}
		
	}

	/**random number generation */
	Random randomGen = new Random();
	/**
	 * DA dists to target
	 */
	List<Double> daDists;
	BitSet daSinks;
	boolean useDistCost;
	MDPSimple mdp;
	//int numStates;
	
	List<State> statesList;
	private StateStorage<SearchState> statesSet;
	private ProductModelGenerator modelGen;

	/**
	 * Constructor.
	 */
	public BRTDP(PrismComponent parent, ProductModelGenerator modelGen, List<Double> daDists, BitSet daSinks, int depth, int nSamples, boolean useDistCost, MDPSimple basePolicy) throws PrismException
	{
		this.mainLog = parent.getLog(); 
		this.settings = parent.getSettings();
		
		this.modelGen = modelGen;
		this.daDists = daDists;
		this.daSinks = daSinks;
		this.useDistCost = useDistCost;
		
		// Display a warning if there are unbounded vars
		VarList varList = modelGen.createVarList();
		if (modelGen.containsUnboundedVariables())
			mainLog.printWarning("Model contains one or more unbounded variables: model construction may not terminate");

		//numStates = 0;
		// Create model storage
		this.mdp = new MDPSimple();
		this.mdp.setVarList(varList);
		this.statesSet= new IndexedSet<SearchState>(true);
		this.statesList = new ArrayList<State>();
		
		this.addMdpInitialStates();
	}
	
	public MDP getMdp() {
		return mdp;
	}
	
	public MDP getPolicies() {
		return getGreedyPolicy(true);
	}
	
	public double getStateCost(State state) {
		int daVal = (int)state.varValues[modelGen.getNumVars() - 1];
		double res = daDists.get(daVal);
		return res;
	}

	public boolean isDASink(State state) {
		return daSinks.get((int)state.varValues[modelGen.getNumVars() - 1]);
	}
	
	public void addMdpInitialStates() throws PrismException {
		SearchState initSstate;
		int numStates = 0;
		
		for (State initState : modelGen.getInitialStates()) {
			initSstate = new SearchState(initState, numStates, getStateCost(initState), isDASink(initState));
			addState(initSstate, mdp, statesSet, statesList);
			mdp.addInitialState(numStates);
			numStates++;
		}
	}

	public int addState(SearchState sstate, MDPSimple mdp, StateStorage<SearchState> statesSet, List<State> statesList) {
		int stateId;
		// Is this a new state?
		if (statesSet.add(sstate)) {
			// And to model
			mdp.addState();
			statesList.add(sstate);
		} 
		// Get index of state in state set
		stateId = statesSet.getIndexOfLastAdd();
		return stateId;
	}
	
	public SearchState findState(State state) {
		for (State sstate: statesList) {
			if (state.compareTo(sstate) == 0) {
				return (SearchState)sstate;
			}
		}
		System.out.println("State not found in the search MDP");
		return null;
	}
	
	public SearchState sampleSucc(SearchState sstate, int action) throws PrismException {
		int s = sstate.getStateId();
		int nSuccs = mdp.getNumTransitions(s, action);
		List<SearchState> succs = new ArrayList<SearchState>(nSuccs);
		List<Double> scores = new ArrayList<Double>(nSuccs);
		Distribution outcomes = mdp.getChoice(s, action);
		double totalScore = 0.0;
		for (Iterator<Entry<Integer, Double>> it = outcomes.iterator(); it.hasNext(); ) {
			Entry<Integer, Double> outcome = it.next();
			SearchState succ = (SearchState)statesList.get(outcome.getKey());
			double prob = outcome.getValue();
			Bound accBound = succ.getBounds().get(BOUND);
			//System.out.println("LB :" + accBound.getLb());
			double score = (accBound.getUb() - accBound.getLb())*prob;
			totalScore = totalScore + score;
			scores.add(score);
			succs.add(succ);
		}
		if (totalScore == 0.0) {
			System.out.println(totalScore);
		}
		double sampled = randomGen.nextDouble()*totalScore;
		double currentScore = 0.0;
		for(int i = 0; i < nSuccs; i++) {
			currentScore = currentScore + scores.get(i);
			if (currentScore > sampled) {
				return succs.get(i);
			}
		}
		//System.out.println("FODA_SE");
		return null;
	}
	
	public SearchState sampleSucc2(SearchState sstate, int action) throws PrismException {
		int s = sstate.getStateId();
		int nSuccs = mdp.getNumTransitions(s, action);
		List<SearchState> succs = new ArrayList<SearchState>(nSuccs);
		List<Double> scores = new ArrayList<Double>(nSuccs);
		Distribution outcomes = mdp.getChoice(s, action);
		double totalScore = 0.0;
		for (Iterator<Entry<Integer, Double>> it = outcomes.iterator(); it.hasNext(); ) {
			Entry<Integer, Double> outcome = it.next();
			SearchState succ = (SearchState)statesList.get(outcome.getKey());
			double prob = outcome.getValue();
			//System.out.println("LB :" + accBound.getLb());
			double score = prob;
			totalScore = totalScore + score;
			scores.add(score);
			succs.add(succ);
		}
		if (totalScore == 0.0) {
			System.out.println(totalScore);
		}
		double sampled = randomGen.nextDouble();
		double currentScore = 0.0;
		for(int i = 0; i < nSuccs; i++) {
			currentScore = currentScore + scores.get(i);
			if (currentScore > sampled) {
				return succs.get(i);
			}
		}
		System.out.println("FODA_SE");
		return null;
	}
	
	
	
	public void expand(SearchState sstate) throws PrismException {
		int succId, nc, nt;
		Distribution distr;
		State succState;
		SearchState succSstate;
		
		modelGen.exploreState(sstate);
		nc = modelGen.getNumChoices();
		sstate.initialise(nc);
		for (int i = 0; i < nc; i++) {
			distr = new Distribution();
			// Look at each transition in the choice
			nt = modelGen.getNumTransitions(i);
			for (int j = 0; j < nt; j++) {
				succState = modelGen.computeTransitionTarget(i, j);
				succSstate = new SearchState(succState, -1, getStateCost(succState), isDASink(succState));
				succId = addState(succSstate, mdp, statesSet, statesList);
				succSstate.setStateId(succId);
				// Add transitions to model
				distr.add(succId, modelGen.getTransitionProbability(i, j));
			}
			// For nondet models, add collated transition to model 
			mdp.addActionLabelledChoice(sstate.getStateId(), distr, modelGen.getChoiceAction(i));
		}
	}
	
		
	public void doBellmanBackup(SearchState state, int actionIndex) {
		if (state.isSolved()) {
			return;
		}
		Distribution distr = mdp.getChoice(state.getStateId(), actionIndex);
		Map<String, Bound> bounds = state.getBounds();
		for (Iterator<Entry<String, Bound>> itBounds = bounds.entrySet().iterator(); itBounds.hasNext(); ) {
			double ub = 0.0;
			double lb = 0.0;
			Entry<String, Bound> boundEntry = itBounds.next();
			Bound bound = boundEntry.getValue();
			String boundName = boundEntry.getKey();
			for (Iterator<Entry<Integer, Double>> itDistr = distr.iterator(); itDistr.hasNext(); ) {
				Entry<Integer, Double> outcome = itDistr.next();
				double prob = outcome.getValue();
				int succ = outcome.getKey();
				SearchState succState = (SearchState)statesList.get(succ);
				double progRew = 0;
				if (boundName == "prog") {
					progRew = state.getStateCost() - succState.getStateCost();
				}
				ub = ub + prob*(progRew + succState.getBounds().get(boundName).getUb());
				lb = lb + prob*(progRew + succState.getBounds().get(boundName).getLb());
			}
			bound.setUbQ(actionIndex, ub);
			bound.setLbQ(actionIndex, lb);
			
			
			double bestLb, bestUb;
			if (bound.isMinimisation()){
				bestLb = 2;
				bestUb = 2;
			} else {
				bestLb = -1;
				bestUb = -1;
			}
			//int i = 0;
			//int bestLbId = -1;
			for (double candidateLb : bound.getLbQ()) {
				//System.out.println(candidateUb);
				if ((bound.isMinimisation() && candidateLb < bestLb) || (!bound.isMinimisation() && candidateLb > bestLb)) {
					bestLb = candidateLb;
			//		bestLbId = i;
				}
				//i++;
			}
			bound.setLb(bestLb);
			//bound.setUb(bound.getUbQ().get(bestLbId)); /TODO: Think about this version vs below

			//i = 0;
			for (double candidateUb : bound.getUbQ()) {
				//System.out.println(candidateUb);
				if ((bound.isMinimisation() && candidateUb < bestUb) || (!bound.isMinimisation() && candidateUb > bestUb)) {
					bestUb = candidateUb;
				}
				//i++;
			}
			bound.setUb(bestUb);
			
			if (boundName == BOUND) {
				if (PrismUtils.doublesAreClose(bound.getUb(), bound.getLb(), termCritParam, true)) {
					state.setSolved(); //TODO: make this more general 
				}
			}
		}
	}

	
	public void doSearch(State startState) throws PrismException
	{
		// State storage
		SearchState sstate, startSstate;
		// Explicit model storage
		Deque<Entry<SearchState, Integer>> toBackupStack = new ArrayDeque<Entry<SearchState, Integer>>();
		Entry<SearchState, Integer>stateActionIndexPair;
		
		//int numStates = mdp.getNumStates();
		boolean stop = false;
		int nOuterIters, nInnerIters;
		
		List<Integer>bestActionIndices;
 		int bestActionIndex;
				
		startSstate = findState(startState);
		
		// Starting BRTDP...
		mainLog.print("\nPerforming BTRDP...");
		mainLog.flush();
		ProgressDisplay progress = new ProgressDisplay(mainLog);
		progress.start();

		List<String> boundNames = new ArrayList<String>();
		boundNames.add(BOUND);
		boundNames.add(BOUND);
		//boundNames.add("acc");
		//boundNames.add("acc");
		//boundNames.add("prog");
		//boundNames.add("prog");
		List<Boolean> useUb = new ArrayList<Boolean>();
		useUb.add(true);
		useUb.add(false);
		//useUb.add(true);
		//useUb.add(false);
		// Explore...
		nOuterIters = 0;
		while (!stop) {
			nOuterIters++;
			if (nOuterIters % 100 == 1) {
				System.out.println(nOuterIters);
			}
			sstate = startSstate;
			nInnerIters = 0;
			while ((!sstate.isSolved() && nInnerIters < 1000) || (nInnerIters < 1)) {
				sstate.incrementNumVisits();
				//System.out.println(sstate);
				if(!sstate.isExpanded()) {
					expand(sstate);
				}
				bestActionIndices = sstate.getBestActions(boundNames, useUb, false);
				if (bestActionIndices == null) { 
					break;
				} else {//TODO: Make sure this is ok
					bestActionIndex = bestActionIndices.get(randomGen.nextInt(bestActionIndices.size()));
					stateActionIndexPair = new Pair<SearchState, Integer>(sstate, bestActionIndex);
					doBellmanBackup(sstate, bestActionIndex);
					if (!sstate.isSolved() || nInnerIters < 1) {
						sstate = sampleSucc(sstate, bestActionIndex);
						if (sstate == null) {
							break; 
						}
					}
				}
				nInnerIters++;
			}
			while (!toBackupStack.isEmpty()) {
				stateActionIndexPair = toBackupStack.pop();
				doBellmanBackup(stateActionIndexPair.getKey(), stateActionIndexPair.getValue());
			}
			if (nOuterIters > 100000 || startSstate.isSolved()) {
				stop = true;
			}
		}

		mdp.setStatesList(statesList);
		//policies = getGreedyPolicy(true);
		//policies.exportToDotFile("/home/bruno/Desktop/policy_brtdp.dot");
	}
	
	MDPSimple getGreedyPolicy(boolean useUpperBound) {
		// State storage
		StateStorage<SearchState> policyStatesSet = new IndexedSet<SearchState>(true);
		List<State> policyStatesList = new ArrayList<State>();

		//To analyse queue
		Deque<Integer> queue = new ArrayDeque<Integer>();
		// Misc
		int stateIndex, succStateIndex, src, dest, numStates;
		List<Integer> bestActionIndices;
		double prob;

		// Create model storage
		MDPSimple policyMdp = new MDPSimple();
		policyMdp.setVarList(mdp.getVarList());
		
		
		List<String> boundNames = new ArrayList<String>();
		boundNames.add("acc");
		//boundNames.add("prog");
		List<Boolean> useUb = new ArrayList<Boolean>();
		useUb.add(false);
		//useUb.add(true);

		numStates = 0;
		// Add initial state(s) to 'explore', 'states' and to the model
		for (Integer initState : mdp.getInitialStates()) {
			addState((SearchState)mdp.getStatesList().get(initState), policyMdp, policyStatesSet, policyStatesList);
			policyMdp.addInitialState(numStates);
			queue.add(initState);
			numStates++;
		}
		
		// Explore...
		src = -1;
		while (!queue.isEmpty()) {
			// Pick next state to explore
			// (they are stored in order found so know index is src+1)
			stateIndex = queue.removeFirst();
			src++;
			// Look at each outgoing choice in turn
			SearchState sstate = (SearchState) mdp.getStatesList().get(stateIndex);
			bestActionIndices = sstate.getBestActions(boundNames, useUb, false); 
			if (bestActionIndices != null) {
				for (int bestActionIndex : bestActionIndices) {
					Distribution outcomes = mdp.getChoice(stateIndex, bestActionIndex);
					Distribution outcomesPolicy = new Distribution();
					for(Iterator<Entry<Integer, Double>> succsIt = outcomes.iterator(); succsIt.hasNext(); ) {
						Entry<Integer, Double> currentOutcome = succsIt.next();
						succStateIndex = currentOutcome.getKey();
						SearchState succState = (SearchState) mdp.getStatesList().get(succStateIndex);
						prob = currentOutcome.getValue();
						dest = addState(succState, policyMdp, policyStatesSet, policyStatesList);
						//is state new?
						if (dest == numStates) {
							numStates++;
							queue.add(succStateIndex);
						}
						// Add transitions to model
						outcomesPolicy.add(dest, prob);
					}
					// For nondet models, add collated transition to model 
					policyMdp.addActionLabelledChoice(src, outcomesPolicy, mdp.getAction(stateIndex, bestActionIndex));
				}
			}
		}
		policyMdp.setStatesList(policyStatesList);
		return policyMdp;
	}
	

	
	/**
	 * Construct an explicit-state model and return it.
	 * If {@code justReach} is true, no model is built and null is returned;
	 * the set of reachable states can be obtained with {@link #getStatesList()}.
	 * @param modelGen The ModelGenerator interface providing the model 
	 * @param justReach If true, just build the reachable state set, not the model
	 *//*
	public MDP constructModel() throws PrismException
	{
		// State storage
		StateStorage<State> states;
		LinkedList<State> explore;
		State state, stateNew;
		SearchState sstate;
		// Explicit model storage
		Distribution distr = null;
		// Misc
		int i, j, nc, nt, src, dest;
		long timer;

		
		// Display a warning if there are unbounded vars
		VarList varList = modelGen.createVarList();
		if (modelGen.containsUnboundedVariables())
			mainLog.printWarning("Model contains one or more unbounded variables: model construction may not terminate");

		// Starting reachability...
		mainLog.print("\nPerforming BTRDP...");
		mainLog.flush();
		ProgressDisplay progress = new ProgressDisplay(mainLog);
		progress.start();
		timer = System.currentTimeMillis();

		// Create model storage
		mdp = new MDPSimple();
		mdp.setVarList(varList);

		// Initialise states storage
		states = new IndexedSet<State>(true);
		explore = new LinkedList<State>();
		// Add initial state(s) to 'explore', 'states' and to the model
		for (State initState : modelGen.getInitialStates()) {
			sstate = new SearchState(initState, -1, getStateCost(initState), isDASink(initState));
			explore.add(sstate);
			states.add(sstate);
			mdp.addState();
			mdp.addInitialState(mdp.getNumStates() - 1);
		}
		
		// Explore...
		src = -1;
		while (!explore.isEmpty()) {
			// Pick next state to explore
			// (they are stored in order found so know index is src+1)
			state = explore.removeFirst();
			src++;
			// Explore all choices/transitions from this state
			modelGen.exploreState(state);
			// Look at each outgoing choice in turn
			nc = modelGen.getNumChoices();
			sstate = (SearchState)state;
			sstate.initialise(nc);
			if (getStateCost(sstate) == 0) {
				System.out.println("FOUND ACC");
				sstate.getBounds().get("acc").setLb(1.0);
				sstate.getBounds().get("prog").setUb(0.0);
				sstate.getBounds().get("sink").setUb(0.0);
				sstate.setSolved();
			} else {
				if (isDASink(sstate)) {
					System.out.println("FOUND SINK");
					sstate.getBounds().get("acc").setUb(0.0);
					sstate.getBounds().get("prog").setUb(0.0);
					sstate.getBounds().get("sink").setLb(1.0);
					sstate.setSolved();
				}
			}
			for (i = 0; i < nc; i++) {
				distr = new Distribution();
				// Look at each transition in the choice
				nt = modelGen.getNumTransitions(i);
				for (j = 0; j < nt; j++) {
					stateNew = modelGen.computeTransitionTarget(i, j);
					// Is this a new state?
					sstate = new SearchState(stateNew, -1, getStateCost(stateNew), isDASink(stateNew));
					if (states.add(sstate)) {
						// If so, add to the explore list
						explore.add(sstate);
						// And to model
						mdp.addState();
					}
					// Get index of state in state set
					dest = states.getIndexOfLastAdd();
					// Add transitions to model
					distr.add(dest, modelGen.getTransitionProbability(i, j));
				}
				// For nondet models, add collated transition to model 
				mdp.addActionLabelledChoice(src, distr, modelGen.getChoiceAction(i));
			}
			// Print some progress info occasionally
			progress.updateIfReady(src + 1);
		}

		// Finish progress display
		progress.update(src + 1);
		progress.end(" states");

		// Reachability complete
		mainLog.print("BRTDP finished in " + ((System.currentTimeMillis() - timer) / 1000.0) + " secs.");
		//mainLog.println(states);

		ArrayList<State> statesList = states.toArrayList();

		states.clear();
		states = null;

		mdp.setStatesList(statesList);
		
		SearchState initState2 = (SearchState)statesList.get(mdp.getFirstInitialState());
		while (initState2.getBounds().get("acc").getLb() < 0.71) {
			for (i = 0; i < mdp.getNumStates(); i++) {
				for (j = 0; j < mdp.getNumChoices(i); j++) {
					doBellmanBackup((SearchState)statesList.get(i), j);
				}
			}
			System.out.println(initState2.getBounds().get("acc").getLb());
			System.out.println(initState2.getBounds().get("acc").getUb());
		}
		
		return mdp;

		//attachLabels(modelGen, mdp);
	}
*/

	

}
