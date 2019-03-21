package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;

import parser.State;
import prism.PrismException;
import prism.PrismLog;
import prism.ProductModelGenerator;

/**
 * UCT for Multiple Robots Following Bruno's code and Planning with MDPs pg 112
 **/
public class MRuct {

//	protected double termCritParam = 1e-8;
//	public static final int UNK_STATE = -1;
//	public static final int SINK_STATE = 0;
//	public static final int ACC_STATE = 1;

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

	Random randomGen;
	JointPolicy uctPolicy;

	public MRuct(PrismLog log, ArrayList<ProductModelGenerator> robotProdModelGens, int max_rollouts, int rollout_depth,
			ArrayList<String> shared_state_names, List<Double> dadistToAcc)

	{
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
		uctPolicy = new JointPolicy(mainLog, num_robots, 1, 0, null, null);
		randomGen = new Random();

	}

	ArrayList<State> getJointStates(ArrayList<ArrayList<State>> succStates) {
		ArrayList<State> jointStates = new ArrayList<State>();
		for (int i = 0; i < succStates.size(); i++)
			jointStates.add(getJointState(succStates.get(i),null));
		return jointStates;
	}

	State getJointState(ArrayList<State> states,BitSet accStates) {
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
				if (!da_state_in_joint_state.equals(da_state_in_robot)) {
					// if there are competing DA states
					// lets take the one whose disttoacc is the smallest ?
					State dastateinjointstate = (State) da_state_in_joint_state.varValues[0];

					int da_state_in_joint_state_int = (int) dastateinjointstate.varValues[0];
					int dastateinrobot = (int) da_state_in_robot.varValues[0];
					if (!(dastateinrobot == da_state_in_joint_state_int)) {
						double jsacc = da_distToAcc.get(da_state_in_joint_state_int);
						double rsacc = da_distToAcc.get(dastateinrobot);
//						if(jsacc == 0 || rsacc == 0)
//						{accFound = true;
//							
//						}
						if(accStates != null) {
						if(accStates.get(dastateinrobot) || accStates.get(da_state_in_joint_state_int))
						{
							accFound = true;
						}
						}
						if (jsacc > rsacc) {
							temp_joint_state.setValue(num_robots, da_state_in_robot);
						}
					}

				}
			}

		}
		return temp_joint_state;
	}

	double getStateCost(State state)
	{
		//get the da state 
		State dastate = state.substate(num_robots, num_robots+1);
		dastate = (State)dastate.varValues[0];
		int dastateint = (int)dastate.varValues[0]; 
		double dist = da_distToAcc.get(dastateint); 
		if(dist == 0)
		{
			uctPolicy.setAsAccState(state);
		}
		return dist;
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

	public void search(BitSet accStates) {
		current_rollout_num = 0;
		ArrayList<State> initialStates = getInitialStates();
		// get the initial state
		State temp_joint_state = getJointState(initialStates,null);
		mainLog.println("Joint State:" + temp_joint_state.toString());

		while (current_rollout_num < max_num_rollouts) {
			mainLog.println("Rollout Attempt: "+current_rollout_num);
			// do a rollout
			try {
				rollout(initialStates, current_rollout_num,accStates);//rollout_stateactionstateonly(initialStates, current_rollout_num);
			} catch (PrismException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			current_rollout_num++;
		
		}
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
				name +=  jointAction.get(i).toString();
			}
			if (i < jointAction.size() - 1) {
				name += ",";
			}
		}
		return name;
	}

	ArrayList<Object> splitJointAction(Object jointAction)
	{
		 String[] action_arr = jointAction.toString().split(",");
		 ArrayList<Object> actions = new ArrayList<Object>(); 
		 for(int i = 0; i<action_arr.length; i++)
			 actions.add((Object)action_arr[i]);
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
			State state = getJointState(currentStates,null);
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

	public void rollout(ArrayList<State> initialStates, int current_rollout_num,BitSet accStates) throws PrismException {

		int current_depth = 0;
		accFound = false;
		ArrayList<Entry<State, Object>> simulatedPolicy = new ArrayList<Entry<State, Object>>();
		ArrayList<Double> cumulativeCost = new ArrayList<Double>();
		
		ArrayList<State> currentStates = initialStates;
		State state = getJointState(currentStates,accStates);
		boolean doProb = false; 
		boolean minimiseCosts= !doProb;
		double initialCost = 0; 
//		if(doProb)
//			initialCost = 1.0;
		cumulativeCost.add(initialCost);
		// trial
		while (current_depth < rollout_depth) {

			uctPolicy.addState(state);

			// get all actions for this state
			HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
			ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);

			// if state has not been visited before initialise actions
			if (!uctPolicy.stateVisited(state)) {
				for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
					Object action = getJointActionName(allPossibleJointActions.get(actionNum));
					uctPolicy.initialiseVisits(state, action);
				}
			}
			uctPolicy.printStateDetails(state);
			//choose an action 
			Object chosen_jointAction = uctPolicy.getBestAction(state, minimiseCosts);
			mainLog.println("Chosen Action "+chosen_jointAction);
			// now lets carry out this action
			HashMap<Integer, HashMap<State, Double>> successors = getActionSuccessors(splitJointAction(chosen_jointAction), allPossibleActionsForState);
			
			// now we've got to create successor state combinations
			ArrayList<ArrayList<State>> jointSuccessors = getJointStateCombinations(successors);
			ArrayList<State> succStates = getJointStates(jointSuccessors);
			ArrayList<Double> probs = calculateJointStatesProbs(successors, jointSuccessors);
			// add all these to my mdp
			uctPolicy.addAction(state,chosen_jointAction, succStates,probs);
			// now we sample a successor maybe :P I dont know really
			int state_choice = randomGen.nextInt(jointSuccessors.size());

			ArrayList<State> successorStates = jointSuccessors.get(state_choice);

			//now that we know the next state, we say its probability is the cost 
//			if(!doProb)
//			cumulativeCost.add(cumulativeCost.get(current_depth)+probs.get(state_choice)*1); //if we were doing least steps for then things with lower prob would mean less steps so no 
			//else
			//doing prob
			//cumulativeCost.add(cumulativeCost.get(current_depth)*probs.get(state_choice));
			
			
			simulatedPolicy.add(new AbstractMap.SimpleEntry<State, Object>(state,
					chosen_jointAction));
			
//			//update things here 
//			//comment this out later 
//			double prevQ = uctPolicy.getQvalue(state, chosen_jointAction); 
//			double stateActionVisits = (double)uctPolicy.getNumVisits(state, chosen_jointAction);
//			double newQ = (stateActionVisits * prevQ 
//					+ (cumulativeCost.get(current_depth+1)-cumulativeCost.get(current_depth)) )/(stateActionVisits+1);
//			
//			uctPolicy.updateQValue(state, chosen_jointAction, newQ);
//			uctPolicy.increaseVisits(state, chosen_jointAction);
			
			currentStates = successorStates;
			state = getJointState(currentStates,accStates);
		
			double stateCost =0;
			boolean doBreak = false;
			if(doProb)
			{
				if(accFound)
					stateCost=1;//cumulativeCost.add(cumulativeCost.get(current_depth)+1); 
//				else
//					cumulativeCost.add(cumulativeCost.get(current_depth));
			}
			else
			{
//				if(accFound)
//					cumulativeCost.add(cumulativeCost.get(current_depth)+getStateCost(state));
//				else
					stateCost = getStateCost(state);//cumulativeCost.add(cumulativeCost.get(current_depth)+getStateCost(state));
			}
			
			
			
			// add the if goal here
			// if currentStates = goal then wohooo
			if(accFound)
			{	
				mainLog.println("Acc found");
				doBreak = true;
			
			}
			//for now lets not loopback to the same state 
			if(simulatedPolicy.get(current_depth).getKey().equals(state))
			{
				stateCost += 10000;//a really high number;
				//same states
				
				doBreak = true; 
			}
			cumulativeCost.add(cumulativeCost.get(current_depth)+stateCost);
			mainLog.println("Cost "+cumulativeCost.get(current_depth+1));
			current_depth++;
//			if (doBreak)
//				break;

		}
		// update things
		int simulatedPolicysize = simulatedPolicy.size();
		for (int i = 0; i < simulatedPolicysize; i++) {
			Entry<State, Object> stateAction = simulatedPolicy.get(i);
			State state_i = stateAction.getKey();
			Object action_i = stateAction.getValue();
			double prevQ = uctPolicy.getQvalue(state_i, action_i); 
			double stateActionVisits = (double)uctPolicy.getNumVisits(state_i, action_i);
			double newQ = (stateActionVisits * prevQ 
					+ (cumulativeCost.get(simulatedPolicysize-1)-cumulativeCost.get(i)) )/(stateActionVisits+1);
			
			uctPolicy.updateQValue(state_i, action_i, newQ);
			uctPolicy.increaseVisits(state_i, action_i);
		}
		if(accFound) {
		HashMap<State, Object> policysofar = uctPolicy.getBestPolicySoFar(minimiseCosts); 
		mainLog.println(policysofar.toString());
		mainLog.println(simulatedPolicy.toString());
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "no_door_example";
		uctPolicy.jointMDP.exportToDotFile(saveplace + filename + "_rollout" + current_rollout_num + ".dot");
		}

		

	}

}
