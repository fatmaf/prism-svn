package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import explicit.MDPSimple;

import java.util.Random;
import java.util.Set;
import java.util.AbstractMap.SimpleEntry;

import parser.State;
import prism.PrismException;
import prism.PrismLog;
import prism.ProductModelGenerator;

/**
 * UCT for Multiple Robots Following Bruno's code and Planning with MDPs pg 112
 **/
public class MRuctPaper {


private double MINFAILCOST = 0;
//	protected double termCritParam = 1e-8;
//	public static final int UNK_STATE = -1;
//	public static final int SINK_STATE = 0;
//	public static final int ACC_STATE = 1;

private double MAXFAILCOST=0;

	private PrismLog mainLog;
	private ArrayList<ProductModelGenerator> prodModGens;
	private int rollout_depth;
	private int max_num_rollouts;
	private int current_rollout_num;
	private int num_robots;
	private List<Double> da_distToAcc;
	private boolean accFound = false;
	private String da_id_string = "_da0";
	private ArrayList<String> shared_state_names;
	private int current_depth = 0;
	private HashMap<State, ArrayList<State>> jointStateRobotStateMapping;
	State initState = null;
	Random randomGen;
	JointPolicyPaper uctPolicy;

	public MRuctPaper(PrismLog log, ArrayList<ProductModelGenerator> robotProdModelGens, int max_rollouts,
			int rollout_depth, ArrayList<String> shared_state_names, List<Double> dadistToAcc)

	{
		jointStateRobotStateMapping = new HashMap<State, ArrayList<State>>();
		num_robots = robotProdModelGens.size();
		prodModGens = robotProdModelGens;
		this.rollout_depth = rollout_depth;
		this.max_num_rollouts = max_rollouts;
		mainLog = log;
		current_rollout_num = 0;
		if (shared_state_names != null) {
			if (shared_state_names.size() > 0)
				this.shared_state_names = shared_state_names;
			else
				this.shared_state_names = null;
		}
		da_distToAcc = dadistToAcc;
		uctPolicy = new JointPolicyPaper(mainLog, num_robots, 1, 0, null, null);
		randomGen = new Random();
		//so the maximum failure cost should be the number of steps 
		//because with the max cost of other things being 1 you can never do worse 
		//or even smaller ? 
		//possibly 
		MAXFAILCOST = 1;//10/0.2; 
		MINFAILCOST= -MAXFAILCOST; 

	}

	ArrayList<State> getJointStates(ArrayList<ArrayList<State>> succStates) {
		ArrayList<State> jointStates = new ArrayList<State>();
		for (int i = 0; i < succStates.size(); i++)
			jointStates.add(getJointState(succStates.get(i), null));
		return jointStates;
	}

	State getJointState(ArrayList<State> states, BitSet accStates) {

		State temp_joint_state = new State(num_robots + 1);

		for (int i = 0; i < num_robots; i++) {

			State current_model_state = states.get(i);
			// collect non da values
			// knowing that the da value is the last one
			// TODO: shared state stuff
			int da_state_index = prodModGens.get(i).getVarIndex(da_id_string);

			temp_joint_state.setValue(i, current_model_state.substate(0, da_state_index));

			State da_state_in_joint_state = temp_joint_state.substate(num_robots, num_robots + 1);

			if (da_state_in_joint_state.varValues[0] == null) {
				temp_joint_state.setValue(num_robots, current_model_state.substate(da_state_index, da_state_index + 1));
			} else {
				// check if there's a change
				State da_state_in_robot = current_model_state.substate(da_state_index, da_state_index + 1);
				State dastateinjointstate = (State) da_state_in_joint_state.varValues[0];

				if (!dastateinjointstate.equals(da_state_in_robot)) {
					// if there are competing DA states
					// lets take the one whose disttoacc is the smallest ?

					int da_state_in_joint_state_int = (int) dastateinjointstate.varValues[0];
					int dastateinrobot = (int) da_state_in_robot.varValues[0];
					if (!(dastateinrobot == da_state_in_joint_state_int)) {
						double jsacc = da_distToAcc.get(da_state_in_joint_state_int);
						double rsacc = da_distToAcc.get(dastateinrobot);
//						if(jsacc == 0 || rsacc == 0)
//						{accFound = true;
//							
//						}
						if (accStates != null) {
							if (accStates.get(dastateinrobot) || accStates.get(da_state_in_joint_state_int)) {
								accFound = true;
							}
						}
						if (jsacc >= rsacc) {
							temp_joint_state.setValue(num_robots, da_state_in_robot);
						}
					}

				}
			}

		}
		if (!jointStateRobotStateMapping.containsKey(temp_joint_state)) {
			ArrayList<State> cp = new ArrayList<State>();
			cp.addAll(states);
			jointStateRobotStateMapping.put(temp_joint_state, cp);
		}
		return temp_joint_state;
	}

	double getStateCost(State state) {
		// get the da state
		State dastate = state.substate(num_robots, num_robots + 1);
		dastate = (State) dastate.varValues[0];
		int dastateint = (int) dastate.varValues[0];
		double dist = da_distToAcc.get(dastateint);
		if (dist == 0) {
			mainLog.println(state.toString());
			uctPolicy.setAsAccState(state);
			accFound = true;
		}
		return dist;
	}
	
	boolean isGoal(State state) {
		// get the da state
		State dastate = state.substate(num_robots, num_robots + 1);
		dastate = (State) dastate.varValues[0];
		int dastateint = (int) dastate.varValues[0];
		double dist = da_distToAcc.get(dastateint);
		if (dist == 0) {
			mainLog.println(state.toString());
			uctPolicy.setAsAccState(state);
			accFound = true;
			return true;
		}
		return false;
	}
	
	ArrayList<State> getInitialStates() {
		ArrayList<State> states = new ArrayList<State>();
		// creating the initial state
		for (int i = 0; i < num_robots; i++) {
			State current_model_state = null;
			try {
				current_model_state = prodModGens.get(i).getInitialState();
			} catch (PrismException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			states.add(current_model_state);
		}
		return states;

	}
	// building the whole tree

	public ArrayList<Integer> uctsearchwithoutapolicy(BitSet accStates) {
		ArrayList<Integer> accsFound = new ArrayList<Integer>();

		current_rollout_num = 0;
		ArrayList<State> initialStates = getInitialStates();
		// get the initial state
		State temp_joint_state = getJointState(initialStates, null);
		mainLog.println("Joint State:" + temp_joint_state.toString());

		while (current_rollout_num < max_num_rollouts) {
			mainLog.println("Rollout Attempt: " + current_rollout_num);
			// do a rollout
			try {
				int res = rollout(initialStates, current_rollout_num, accStates, false);// rollout_stateactionstateonly(initialStates,
																						// current_rollout_num);
				if (res != -1)
					accsFound.add(res);
			} catch (PrismException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			current_rollout_num++;

		}
		return accsFound;
	}

	/**
	 * get actions for each robot's state
	 * 
	 * @param states = list of states, one per robot, ordered
	 * @return list of actions, one per robot , ordered
	 */
	HashMap<Integer, HashMap<Object, Integer>> getActions(ArrayList<State> states) {
		HashMap<Integer, HashMap<Object, Integer>> actions = new HashMap<Integer, HashMap<Object, Integer>>();
		try {

			for (int i = 0; i < states.size(); i++) {
				HashMap<Object, Integer> robot_actions = new HashMap<Object, Integer>();
				ProductModelGenerator prodModGen = prodModGens.get(i);
				// get the number of choices for this state
				prodModGen.exploreState(states.get(i));
				int numChoices = prodModGen.getNumChoices();
				for (int c = 0; c < numChoices; c++) {
					Object action = prodModGen.getChoiceAction(c);
					robot_actions.put(action, c);
				}
				actions.put(i, robot_actions);

			}
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return actions;
	}

	ArrayList<Object> getActionNamesFromRobotActionNums(int[] combination) throws PrismException {
		ArrayList<Object> actionNames = new ArrayList<Object>();
		for (int i = 0; i < combination.length; i++) {
			ProductModelGenerator prodModGen = prodModGens.get(i);
			Object action = prodModGen.getChoiceAction(combination[i] - 1);
			actionNames.add(action);

		}
		return actionNames;
	}

	ArrayList<Integer> getNumActionsPerRobot(ArrayList<State> states) throws PrismException {

		ArrayList<Integer> numActions = new ArrayList<Integer>();
		for (int i = 0; i < states.size(); i++) {
			ProductModelGenerator prodModGen = prodModGens.get(i);
			prodModGen.exploreState(states.get(i));
			int nc = prodModGen.getNumChoices();
			if (nc == 0)
				nc = 1;
			numActions.add(nc);

		}
		return numActions;
	}

	HashMap<Integer, HashMap<State, Double>> getActionSuccessors(ArrayList<Object> actions,
			HashMap<Integer, HashMap<Object, Integer>> actionsMap) throws PrismException {
		HashMap<Integer, HashMap<State, Double>> succStates = new HashMap<Integer, HashMap<State, Double>>();
		for (int i = 0; i < actions.size(); i++) {
			Object action = actions.get(i);
			HashMap<State, Double> ahashmap = new HashMap<State, Double>();
			if (action != null) {
				ProductModelGenerator prodModGen = prodModGens.get(i);
				int choice = actionsMap.get(i).get(action);
				int nt = prodModGen.getNumTransitions(choice);
				for (int t = 0; t < nt; t++) {
					double prob = prodModGen.getTransitionProbability(choice, t);
					State succState = prodModGen.computeTransitionTarget(choice, t);
					ahashmap.put(succState, prob);

				}
				succStates.put(i, ahashmap);

			}
		}
		return succStates;
	}

	ArrayList<ArrayList<Object>> getJointActions(HashMap<Integer, HashMap<Object, Integer>> actions)
			throws PrismException {
		int[] numActionsPerRobot = new int[num_robots];
		for (int i = 0; i < num_robots; i++) {
			numActionsPerRobot[i] = actions.get(i).size();
			if (numActionsPerRobot[i] == 0)
				throw new PrismException("0 acions for this robot!");
		}
		ArrayList<int[]> res = new ArrayList<int[]>();
		try {
			uctPolicy.generateCombinations(numActionsPerRobot, numActionsPerRobot.clone(), res);
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		ArrayList<ArrayList<Object>> jointActions = new ArrayList<ArrayList<Object>>();
		ArrayList<ArrayList<Object>> actionsKeySet = new ArrayList<ArrayList<Object>>();
		ArrayList<Object> robotKeySet;
		for (int i = 0; i < num_robots; i++) {
			robotKeySet = new ArrayList<Object>();
			robotKeySet.addAll(actions.get(i).keySet());
			actionsKeySet.add(robotKeySet);
		}
		for (int[] r : res) {
			ArrayList<Object> action = new ArrayList<Object>();
			for (int i = 0; i < r.length; i++) {
				robotKeySet = actionsKeySet.get(i);
				action.add(robotKeySet.get(r[i] - 1));
			}
			jointActions.add(action);
		}
		return jointActions;
	}

	Object getJointActionName(ArrayList<Object> jointAction) {
		String name = "";
		for (int i = 0; i < jointAction.size(); i++) {
			if (jointAction.get(i) != null) {
				name += jointAction.get(i).toString();
			}
			if (i < jointAction.size() - 1) {
				name += ",";
			}
		}
		return name;
	}

	ArrayList<Object> splitJointAction(Object jointAction) {
		String[] action_arr = jointAction.toString().split(",");
		ArrayList<Object> actions = new ArrayList<Object>();
		for (int i = 0; i < action_arr.length; i++)
			actions.add((Object) action_arr[i]);
		return actions;
	}

	ArrayList<Double> calculateJointStatesProbs(HashMap<Integer, HashMap<State, Double>> succStates,
			ArrayList<ArrayList<State>> combinations) {
		double normaliser = 0;
		boolean normalised = false;
		ArrayList<Double> probs = new ArrayList<Double>();
		for (int i = 0; i < combinations.size(); i++) {
			double probsHere = 1;
			for (int j = 0; j < combinations.get(i).size(); j++) {
				probsHere *= succStates.get(j).get(combinations.get(i).get(j));
			}
			normaliser += probsHere;
			probs.add(probsHere);
		}
		if (normaliser != 1.0) {
			normalised = true;
			for (int i = 0; i < probs.size(); i++) {
				probs.set(i, probs.get(i) / normaliser);
			}
			mainLog.println("Normalised!!!");
		}
		return probs;
	}

	ArrayList<ArrayList<State>> getJointStateCombinations(HashMap<Integer, HashMap<State, Double>> succStates)
			throws PrismException {

		int[] numStatesPerRobot = new int[num_robots];
		for (int i = 0; i < num_robots; i++) {
			numStatesPerRobot[i] = succStates.get(i).size();
			if (numStatesPerRobot[i] == 0)
				throw new PrismException("0 successor states for this robot!");
		}
		ArrayList<int[]> combinations = new ArrayList<int[]>();
		try {
			uctPolicy.generateCombinations(numStatesPerRobot, numStatesPerRobot.clone(), combinations);

		} catch (PrismException e) {
			e.printStackTrace();

		}
		ArrayList<ArrayList<State>> jointStates = new ArrayList<ArrayList<State>>();
		ArrayList<ArrayList<State>> robotStates = new ArrayList<ArrayList<State>>();
		ArrayList<State> states;
		for (int i = 0; i < num_robots; i++) {
			states = new ArrayList<State>();
			states.addAll(succStates.get(i).keySet());
			robotStates.add(states);
		}
		for (int[] comb : combinations) {
			states = new ArrayList<State>();
			for (int i = 0; i < comb.length; i++) {
				states.add(robotStates.get(i).get(comb[i] - 1));
			}
			jointStates.add(states);
		}
		return jointStates;

	}

	public void rollout_stateactionstateonly(ArrayList<State> initialStates, int current_rollout_num)
			throws PrismException {

		int current_depth = 0;
		ArrayList<Object> simulatedPolicy = new ArrayList<Object>();
		ArrayList<State> currentStates = initialStates;
		while (current_depth < rollout_depth) {

			// atempting s,a,s' only
			State state = getJointState(currentStates, null);
			uctPolicy.addState(state);
			// get all actions for this state
			HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
			ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);
			for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
				Object action = getJointActionName(allPossibleJointActions.get(actionNum));
				if (!uctPolicy.stateVisited(state))
					uctPolicy.initialiseVisits(state, action);
				else
					uctPolicy.increaseVisits(state, action);
			}

			int action_choice = randomGen.nextInt(allPossibleJointActions.size());
			// now lets carry out this action
			HashMap<Integer, HashMap<State, Double>> successors = getActionSuccessors(
					allPossibleJointActions.get(action_choice), allPossibleActionsForState);
			// now we've got to create successor state combinations
			ArrayList<ArrayList<State>> jointSuccessors = getJointStateCombinations(successors);
			ArrayList<State> succStates = getJointStates(jointSuccessors);
			ArrayList<Double> probs = calculateJointStatesProbs(successors, jointSuccessors);
			// add all these to my mdp
			uctPolicy.addAction(state, getJointActionName(allPossibleJointActions.get(action_choice)), succStates,
					probs);
			// now we sample a successor maybe :P I dont know really
			int state_choice = randomGen.nextInt(jointSuccessors.size());
			// do we need to break
			boolean dobreak = false;
			if (state.equals(succStates.get(state_choice))) {
				// break;
				dobreak = true;
			}
			currentStates = jointSuccessors.get(state_choice);
			simulatedPolicy.add(getJointActionName(allPossibleJointActions.get(action_choice)));
			current_depth++;
			if (dobreak)
				break;

		}
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "no_door_example";
		uctPolicy.jointMDP.exportToDotFile(saveplace + filename + "_rollout" + current_rollout_num + ".dot");

	}

	/**
	 * UCT following algorithm 6.2 of Planning with MDPs Mausam et al
	 * 
	 * @param initialStates
	 * @param current_rollout_num
	 * @throws PrismException
	 * 
	 */

	public int rollout(ArrayList<State> initialStates, int current_rollout_num, BitSet accStates, boolean expandAll)
			throws PrismException {

		int current_depth = 0;
		accFound = false;
		ArrayList<Entry<State, Object>> simulatedPolicy = new ArrayList<Entry<State, Object>>();
		ArrayList<Double> cumulativeCost = new ArrayList<Double>();

		ArrayList<State> currentStates = initialStates;
		State state = getJointState(currentStates, accStates);
		initState = new State(state);
		boolean doProb = false;
		boolean minimiseCosts = !doProb;
		double initialCost = 0;
		boolean dotrial = false;
//		if(doProb)
//			initialCost = 1.0;
		cumulativeCost.add(initialCost);
		// trial

		// if we want to do the selection, expansion, backpropagation thing
		// if we need to expand this state,
		// we have to take an action in this state
		// then add the successors of this action

		int cum_cost_counter = current_depth;
		while (current_depth < rollout_depth) {

			boolean expandState = uctPolicy.toExpand(state);
			if ((expandAll && expandState) || !expandAll) {
				// check if this is a state we need to expand
				uctPolicy.addState(state);

			}
			currentStates = getRobotStatesFromJointState(state);
			// get all actions for this state
			HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
			ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);

			if (!dotrial) {
				// if state has not been visited before initialise actions
				if (!uctPolicy.stateVisited(state)) {
					for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
						Object action = getJointActionName(allPossibleJointActions.get(actionNum));
						uctPolicy.initialiseVisitsNoVisit(state, action);
					}
				}
			}
			uctPolicy.printStateDetails(state, minimiseCosts);
			// choose an action
			Object chosen_jointAction = uctPolicy.getBestAction(state, minimiseCosts);

			mainLog.println("Chosen Action " + chosen_jointAction);
			// now lets carry out this action
			HashMap<Integer, HashMap<State, Double>> successors = getActionSuccessors(
					splitJointAction(chosen_jointAction), allPossibleActionsForState);

			// now we've got to create successor state combinations
			ArrayList<ArrayList<State>> jointSuccessors = getJointStateCombinations(successors);
			ArrayList<State> succStates = getJointStates(jointSuccessors);
			ArrayList<Double> probs = calculateJointStatesProbs(successors, jointSuccessors);
			int state_choice = randomGen.nextInt(jointSuccessors.size());

			if (expandState && !dotrial) {
				// add all these to my mdp
				uctPolicy.addAction(state, chosen_jointAction, succStates, probs);
				dotrial = true;
				// or add just this state action pair
				// uctPolicy.addAction(state, chosen_jointAction, succStates.get(state_choice),
				// probs.get(state_choice));
			}
			// now we sample a successor maybe :P I dont know really

//			ArrayList<State> successorStates = jointSuccessors.get(state_choice);

			// now that we know the next state, we say its probability is the cost
//			if(!doProb)
//			cumulativeCost.add(cumulativeCost.get(current_depth)+probs.get(state_choice)*1); //if we were doing least steps for then things with lower prob would mean less steps so no 
			// else
			// doing prob
			// cumulativeCost.add(cumulativeCost.get(current_depth)*probs.get(state_choice));

			simulatedPolicy.add(new AbstractMap.SimpleEntry<State, Object>(state, chosen_jointAction));

			state = succStates.get(state_choice);

			double stateCost = 0;
			boolean doBreak = false;
			if (doProb) {
				if (accFound)
					stateCost = 1;// cumulativeCost.add(cumulativeCost.get(current_depth)+1);
//				else
//					cumulativeCost.add(cumulativeCost.get(current_depth));
			} else {
//				if(accFound)
//					cumulativeCost.add(cumulativeCost.get(current_depth)+getStateCost(state));
//				else
				stateCost = getStateCost(state);// cumulativeCost.add(cumulativeCost.get(current_depth)+getStateCost(state));
			}

			// add the if goal here
			// if currentStates = goal then wohooo
			if (accFound) {
				mainLog.println("Acc found");
				doBreak = true;

			}
			// for now lets not loopback to the same state
			if (simulatedPolicy.get(current_depth).getKey().equals(state)) {
				stateCost += 10000;// a really high number;
				// same states

				doBreak = true;
			}
			if (cumulativeCost.size() <= cum_cost_counter + 1)
				cumulativeCost.add(cumulativeCost.get(current_depth) + stateCost * probs.get(state_choice));
			else {
				double costSofar = cumulativeCost.get(cum_cost_counter);
				costSofar += stateCost * probs.get(state_choice);
				cumulativeCost.remove(cum_cost_counter);
				cumulativeCost.add(costSofar);
			}
			mainLog.println("Cost " + cumulativeCost.get(current_depth + 1));
			current_depth++;
			if (doBreak)
				break;
			if (!dotrial)
				cum_cost_counter++;

		}
		// update things
		int simulatedPolicysize = simulatedPolicy.size();
		for (int i = 0; i < simulatedPolicysize; i++) {
			Entry<State, Object> stateAction = simulatedPolicy.get(i);
			State state_i = stateAction.getKey();
			Object action_i = stateAction.getValue();
			double prevQ = uctPolicy.getQvalue(state_i, action_i, minimiseCosts);
			double stateActionVisits = (double) uctPolicy.getNumVisits(state_i, action_i);
			double newQ = (stateActionVisits * prevQ
					+ (cumulativeCost.get(simulatedPolicysize - 1) - cumulativeCost.get(i))) / (stateActionVisits + 1);

			uctPolicy.updateQValue(state_i, action_i, newQ);
			uctPolicy.increaseVisits(state_i, action_i);
		}
		if (accFound) {
			HashMap<State, Object> policysofar = uctPolicy.getBestPolicySoFar(initState, minimiseCosts);
			mainLog.println(policysofar.toString());
			mainLog.println(simulatedPolicy.toString());
			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
			String filename = "no_door_example";
			uctPolicy.jointMDP.exportToDotFile(saveplace + filename + "_rollout" + current_rollout_num + ".dot");
			MDPSimple policyTree = uctPolicy.extractPolicyTreeAsDotFile(uctPolicy.jointMDP,
					uctPolicy.getStateIndex(initState), minimiseCosts);
			policyTree.exportToDotFile(saveplace + filename + "_policy" + current_rollout_num + ".dot");
			return current_rollout_num;
		}

		return -1;

	}

	/**
	 * UCT following A Survey of Monte Carlo Tree Search Methods
	 * 
	 * @param initialStates
	 * @param current_rollout_num
	 * @throws PrismException
	 * 
	 */

	private boolean isTerminal(State pstate, boolean selfLoop) {
		if (accFound)
			return true;
		if (selfLoop)
			return true;

		return false;
	}

	private State bestChild(State state, boolean minimiseCosts, ArrayList<Entry<State, Object>> path)
			throws PrismException {// basically best child
		// this bit is from
		// https://www.dtic.upf.edu/~hgeffner/uct-aaai-2012.pdf

		Object chosen_jointAction = uctPolicy.getBestAction(state, minimiseCosts);
		// the assumption that we have all the children for this state
		path.add(new AbstractMap.SimpleEntry<State, Object>(state, chosen_jointAction));
		return uctPolicy.simulateAction(state, chosen_jointAction);

	}

	private State treePolicy(State state, boolean minimiseCosts, ArrayList<Entry<State, Object>> path)
			throws PrismException {
		State succState = null;
		State currState = state;
		boolean selfLoop = false;
		while (!isTerminal(currState, selfLoop) && current_depth < rollout_depth) {

			if (uctPolicy.toExpand(currState)) {
				return state;
			} else {
				succState = bestChild(currState, minimiseCosts, path);

			}

			if (currState.equals(succState))
				selfLoop = true;
			currState = succState;
			current_depth++;
		}
		return currState;
	}

	public State simulateAction(State state, Object action) throws PrismException {
		return simulateAction(state, action, false);
	}

	public State simulateAction(State state, Object action, boolean initState) throws PrismException {
		State succState = null;
	
		if (action != null) {
		succState = uctPolicy.simulateAction(state, action);
		if (succState == null) { // we're simply going to do a rollout we dont care about anything else
			if (uctPolicy.robotStateActionIndices.containsKey(state)) {
				if (uctPolicy.robotStateActionIndices.get(state).containsKey(action)) {
					HashMap<Integer, HashMap<State, Double>> succStates = new HashMap<Integer, HashMap<State, Double>>();
					// okay so nowwwwwww
					// we get the action index for each robot
					ArrayList<Integer> actionIndices = uctPolicy.robotStateActionIndices.get(state).get(action);
					// and we can get like the thing from the stuff we have already
					ArrayList<State> robotStates = getRobotStatesFromJointState(state);
					for (int i = 0; i < num_robots; i++) {
						HashMap<State, Double> ahashmap = new HashMap<State, Double>();
						int choice = actionIndices.get(i);
						ProductModelGenerator pmg = prodModGens.get(i);
						pmg.exploreState(robotStates.get(i));
						int nt = pmg.getNumTransitions(choice);
						for (int t = 0; t < nt; t++) {
							double prob = pmg.getTransitionProbability(choice, t);
							State succState1 = pmg.computeTransitionTarget(choice, t);
							ahashmap.put(succState1, prob);

						}
						succStates.put(i, ahashmap);

					}

					ArrayList<ArrayList<State>> jointSuccessors = getJointStateCombinations(succStates);
					ArrayList<State> jointsuccStates = getJointStates(jointSuccessors);
					ArrayList<Double> probs = calculateJointStatesProbs(succStates, jointSuccessors);
					// now we just choose one of these
					// if its the first one we should add these to uctPolicy
					if (initState) {
						uctPolicy.addAction(state, action, jointsuccStates, probs);
					}
					double rand = (new Random()).nextDouble();
					double cprob = 0;
					for (int i = 0; i < probs.size(); i++) {
						cprob += probs.get(i);
						if (cprob >= rand) {
							succState = jointsuccStates.get(i);
							break;
						}
					}
				} else {
					throw new PrismException("Simulation: Action not in robot state action indices for state");
//					return null;
				}
			} else {
				throw new PrismException ("Simulation: State not in robot state action indices");
//				return null;
			}

		}
		}
		else
			throw new PrismException("Passed null action to simulate action");
			
		return succState;
	}

	public double defaultPolicy(State state, ArrayList<Entry<State, Object>> path) throws PrismException {
		State succState = null;
		State currState = state;
		boolean selfLoop = false;
		ArrayList<State> currentStates;
		double reward = 0;
		boolean initState = true;
		while (!isTerminal(currState, selfLoop) && current_depth < rollout_depth) {
			currentStates = getRobotStatesFromJointState(currState);
			if (!uctPolicy.stateActionVisits.containsKey(currState)) {
				// get all actions for this state
				HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
				ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);

				for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
					Object action = getJointActionName(allPossibleJointActions.get(actionNum));
					uctPolicy.initialiseVisitsNoVisit(currState, action);
					uctPolicy.addRobotStateActionIndices(currState, action, allPossibleJointActions.get(actionNum),
							allPossibleActionsForState);
				}
			}
			reward += getStateCost(currState);
			// otherwise just choose an action
			Object action = uctPolicy.getDefaultPolicyAction(currState);
			if (initState) {
				path.add(new AbstractMap.SimpleEntry<State, Object>(currState, action));
			}
			succState = simulateAction(currState, action, initState);
			if (currState.equals(succState))
				selfLoop = true;
			if (initState)
				initState = false;
			currState = succState;

		}
		current_depth++;
		// dont know if this is right
		return reward;
	}

	public void backpropagateReward(ArrayList<Entry<State, Object>> path, double reward, boolean minimiseCost) {
		double rew = reward;
		for (int i = path.size() - 1; i >= 0; i--) {

			Entry<State, Object> sa = path.get(i);
			State state = sa.getKey();
			if (i != path.size() - 1) {
				rew = getStateCost(state);
			}
			Object act = sa.getValue();
			double qvalue = ((rew + uctPolicy.getQvalue(state, act, minimiseCost))
					* (double) uctPolicy.getNumVisits(state, act)) / (double) uctPolicy.getNumVisits(state);

			uctPolicy.updateQValue(state, act, qvalue);

		}
	}

	public int uctsearch(ArrayList<State> initialStates, int current_rollout_num, BitSet accStates)
			throws PrismException {

		ArrayList<State> currentStates = initialStates;
		State currentState = getJointState(currentStates, accStates);
		ArrayList<Entry<State, Object>> stateActionsChosen = new ArrayList<Entry<State, Object>>();
		boolean minimiseCosts = true;
		currentState = treePolicy(currentState, minimiseCosts, stateActionsChosen);
		double reward = defaultPolicy(currentState, stateActionsChosen);

		// save states visited in tree
		backpropagateReward(stateActionsChosen, reward, minimiseCosts);
		// backup function
		// TODO: backup function here

		return -1;

	}

	public ArrayList<Integer> uctsearch(BitSet accStates) {
		ArrayList<Integer> accsFound = new ArrayList<Integer>();

		current_rollout_num = 0;
		ArrayList<State> initialStates = getInitialStates();
		// get the initial state
		State temp_joint_state = getJointState(initialStates, null);
		mainLog.println("Joint State:" + temp_joint_state.toString());

		while (current_rollout_num < max_num_rollouts) {
			mainLog.println("Rollout Attempt: " + current_rollout_num);
			// do a rollout
			try {
				int res = uctsearch(initialStates, current_rollout_num, accStates);// rollout_stateactionstateonly(initialStates,
																					// current_rollout_num);
				if (res != -1)
					accsFound.add(res);
			} catch (PrismException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			current_rollout_num++;

		}
		return accsFound;
	}

	// because sometimes a state just is a state so thats annoying
	private int getIndividualState(State state, int ind, int subind) {
		Object s = state.varValues[ind];
		int sint;
		if (s instanceof State) {
			sint = getIndividualState((State) s, subind, 0);
		} else {
			sint = (int) s;
		}
		return sint;
	}

	private ArrayList<State> getRobotStatesFromJointState(State state) {
		// TODO edit this for multiple state variables but for now lets just go with the
		// usual
		int numRobotStateVars = 2;
		int da_state_val = getIndividualState(state, num_robots, 0);
		int robot_state_val;
		ArrayList<State> robotStates = new ArrayList<State>();
		State robotState;
		for (int i = 0; i < num_robots; i++) {
			robotState = new State(numRobotStateVars);

			for (int j = 0; j < numRobotStateVars - 1; j++) {
				robot_state_val = getIndividualState(state, i, j);
				robotState.setValue(j, robot_state_val);
			}
			robotState.setValue(numRobotStateVars - 1, da_state_val);
			robotStates.add(robotState);

		}
		return robotStates;
	}

	public Object monteCarloPlanning(BitSet accStates, boolean minCost) throws Exception {
		ArrayList<Integer> accsFound = new ArrayList<Integer>();

		current_rollout_num = 0;
		ArrayList<State> initialStates = getInitialStates();
		// get the initial state
		State temp_joint_state = getJointState(initialStates, null);
		mainLog.println("Joint State:" + temp_joint_state.toString());
		Object res = null;
		while (current_rollout_num < max_num_rollouts) {
			mainLog.println("Rollout Attempt: " + current_rollout_num);
			res = monteCarloPlanning(temp_joint_state, minCost);// rollout_stateactionstateonly(initialStates,
			// current_rollout_num);
//				if (res == null)
////					accsFound.add(res);
			current_rollout_num++;

		}
		//lets do the policy tree stuff here 
		String saveplace = MRmcts.TESTSLOC;
				//"/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "no_door_example";
		uctPolicy.jointMDP.exportToDotFile(saveplace + filename + "_mctsmdp.dot");
		MDPSimple policyTree = uctPolicy.extractPolicyTreeAsDotFile(uctPolicy.jointMDP,
				uctPolicy.getStateIndex(temp_joint_state), minCost);
		policyTree.exportToDotFile(saveplace + filename + "_policy.dot");
		
//		return accsFound;
		return res;
	}

	public Object monteCarloPlanning(State state, boolean minCost) throws Exception {
		Object bestAction = null;
		int current_depth = 0;
		accFound = false;
//		while (current_depth < rollout_depth) {	
		search(null, state, current_depth, minCost);
		if(accFound)
			mainLog.println("Acc Found");
//		}
//		bestAction = uctPolicy.selectBestAction(state, current_depth);
		return bestAction;
	}

	public double search(State ps, State state, int depth, boolean minCost) throws Exception {
		String searchInfo = "";
		if (ps != null)
			searchInfo += ps.toString() + "->";
		searchInfo += state.toString();

		double cost_state = 0;
		if(isGoal(state))
		{
			searchInfo += "=" + cost_state + "-ACC";
			mainLog.println(searchInfo);
			return cost_state;
		}
		if (isTerminal(ps, state)) {
			//dont want deadends
			double failcost = MINFAILCOST; 
			if(minCost)
				failcost = MAXFAILCOST;
			cost_state = getStateCost(state)+failcost;
			searchInfo += "=" + cost_state + "-terminal";
			mainLog.println(searchInfo);
			return cost_state;
		}
		if (depth == rollout_depth) {
			cost_state=getStateCost(state);
			searchInfo += "=" + cost_state + "-maxd";
			mainLog.println(searchInfo);
			return cost_state;
		}
		if (isLeaf(state)) {
//			if (!uctPolicy.stateIndices.containsKey(state))
			initialiseVisits(state);
			ArrayList<Entry<State, Object>> trialList = new ArrayList<Entry<State, Object>>();
			cost_state = evaluate(ps, state, depth, true, trialList, minCost);
			// now lets update this state with the reward
			searchInfo += "=" + cost_state + "-rollout";
			mainLog.println(searchInfo);
			mainLog.println("Trial:\n" + trialList.toString());
			return cost_state;
		}
		Object action = selectAction(state, depth, false, minCost);
		State succState = simulateAction(state, action);
		double reward = getStateCost(state);
		double q = reward + search(state, succState, depth + 1, minCost);
		updateValue(state, action, q, depth, minCost);
		cost_state = q;
		searchInfo += "=" + cost_state + "-selection";
		mainLog.println(searchInfo);
		return cost_state;

	}
	
	
	

	void initialiseVisits(State state) throws PrismException {
		if (!uctPolicy.stateIndices.containsKey(state))
			uctPolicy.addState(state);
		boolean mayhaveaction = false;
		boolean initaction = true;
		// use uctPolicy
		ArrayList<State> currentStates = getRobotStatesFromJointState(state);
		HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
		ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);
		if (uctPolicy.stateActionVisits.containsKey(state))
			mayhaveaction = true;
		mainLog.println(state.toString() + "- A=" + allPossibleJointActions.size());
		boolean noinitneeded = false;
		if (mayhaveaction) {
			noinitneeded = (allPossibleJointActions.size() == uctPolicy.stateActionVisits.get(state).keySet().size());
		}
		if (!noinitneeded) {
			for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
				Object action = getJointActionName(allPossibleJointActions.get(actionNum));
				if (mayhaveaction) {
					if (uctPolicy.stateActionVisits.get(state).containsKey(action))
						initaction = false;
				}
				if (initaction) {
					mainLog.println(action.toString());
//			if (!uctPolicy.stateVisited(state))
					uctPolicy.initialiseVisits(state, action);
					uctPolicy.addRobotStateActionIndices(state, action, allPossibleJointActions.get(actionNum),
							allPossibleActionsForState);
				}
//			else
//				uctPolicy.increaseVisits(state, action);
//			}
			}
		}

		uctPolicy.printStateDetails(state, true);
	}

	private void updateValue(State state, Object action, double q, int depth, boolean minCost) {
		// update n(s,a)
		// q(s,a) = q(a,s) + [q-q(a,s)]/n(s,a)
		uctPolicy.increaseVisits(state, action);
		int numVisits = uctPolicy.getNumVisits(state, action);

		double qudpate = uctPolicy.getQvalue(state, action, minCost);
		if (numVisits == 0) {
			if (minCost)
				qudpate = Double.MIN_VALUE;
			else
				qudpate = Double.MAX_VALUE;
		} else
			qudpate += (q - uctPolicy.getQvalue(state, action, minCost))
					/ (double) uctPolicy.getNumVisits(state, action);
		uctPolicy.updateQValue(state, action, qudpate);

	}

//	private boolean isGoal (State state)
//	{
//		if (accFound)
//			return true;
//		else
//			return false;
//	}
	private boolean isTerminal(State pstate, State state) {
		
		if (pstate != null) {
			if (state.equals(pstate))
				return true;
		}

		return false;
	}

	private boolean isLeaf(State state) {
		// its a leaf if
		// it is not in the tree
		// so basically the state hasnt been added
		// which means it should not exist in the
		// uctPolicy stateIndices
		if (uctPolicy.toExpand(state))
			return true;
		else

			return false;

	}

	private double evaluate(State pstate, State state, int depth, boolean firstState,
			ArrayList<Entry<State, Object>> trial, boolean minCost) throws Exception {
		double sumrew = 0;
		State succState = null;
		if (depth == rollout_depth)
			return getStateCost(state);
		if (isGoal(state))
			return 0.0;
		if (isTerminal(pstate, state))
		{
			double failcost = MINFAILCOST; 
			if(minCost)
				failcost=MAXFAILCOST;
			return getStateCost(state)+failcost;
		}
		// do a whole run
		try {
			double reward = getStateCost(state);
			Object action = selectAction(state, depth, true, minCost);
			trial.add(new AbstractMap.SimpleEntry<State, Object>(state, action));
			succState = simulateAction(state, action, firstState);

			sumrew = reward + evaluate(state, succState, depth + 1, false, trial, minCost);
			if (firstState) {
				updateValue(state, action, sumrew, depth, minCost);
			}
		} catch (Exception e) {
			mainLog.println("Error");
			e.printStackTrace();

			mainLog.println(trial.toString());
			if (succState != null)
				mainLog.println(succState.toString());
			throw new Exception(e);

		}
		return sumrew;

	}

	private Object selectAction(State state, int depth, boolean useRolloutPolicy, boolean minCost)
			throws PrismException {
		if (useRolloutPolicy) {
			// choose an action randomly
			if (uctPolicy.stateActionVisits.containsKey(state)) {
				int numActions = uctPolicy.stateActionVisits.get(state).keySet().size();
				// if this isn't the first visit then maybe we can do the exploration
				// expolitation thing
				// how do we know that this isnt the first action
				// the total visits will be 0
				if (uctPolicy.getNumVisits(state) == 0) {
					int chosen_action = (new Random()).nextInt(numActions);
					Object action = uctPolicy.stateActionVisits.get(state).keySet().toArray()[chosen_action];

					return action;
				} else {
					Object action = uctPolicy.getBestAction(state, minCost);
					return action;
				}
			} else {
				// action not added to uctPolicy
				ArrayList<State> currentStates = getRobotStatesFromJointState(state);
				// get all actions
				ArrayList<Integer> numActionsPerRobot;// = getNumActionsPerRobot(currentStates);
				// get the number of combinations
				ArrayList<int[]> res = new ArrayList<int[]>();
				try {
					numActionsPerRobot = getNumActionsPerRobot(currentStates);
					uctPolicy.generateCombinations(numActionsPerRobot, res);
					// now choose a combination at random
					// since we are not following a policy
					int chosen_action_num = (new Random()).nextInt(res.size());
					// now we get that combination
					int[] chosen_action_comb = res.get(chosen_action_num);
					// now we create an action from that
					ArrayList<Object> actionNames = getActionNamesFromRobotActionNums(chosen_action_comb);
					// return the joint action
					// also add these indices
					Object joint_action = getJointActionName(actionNames);
					uctPolicy.addRobotStateActionIndices(state, joint_action, actionNames, chosen_action_comb);

					return joint_action;
				} catch (PrismException e) {
					e.printStackTrace();

				}

			}
		} else {
			return uctPolicy.getBestAction(state, minCost);
		}
		return null;
	}

}
