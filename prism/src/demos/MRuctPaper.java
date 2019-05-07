package demos;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import acceptance.AcceptanceOmega;
import automata.DA;
import explicit.MDPSimple;

import java.util.Random;
import java.util.Stack;

import parser.State;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.ProductModelGenerator;
import strat.MDStrategy;

/**
 * UCT for Multiple Robots Following Bruno's code and Planning with MDPs pg 112
 **/
public class MRuctPaper
{

	private double MINFAILCOST = 0;
	//	protected double termCritParam = 1e-8;
	//	public static final int UNK_STATE = -1;
	//	public static final int SINK_STATE = 0;
	//	public static final int ACC_STATE = 1;

	private double MAXFAILCOST = 0;

	private PrismLog mainLog;
	private ArrayList<ProductModelGenerator> prodModGens;
	private int rollout_depth;
	private int max_num_rollouts;
	private int current_rollout_num;
	private static int num_robots;
	private List<Double> da_distToAcc;
	private boolean accFound = false;
	private String da_id_string = "_da0";
	private ArrayList<String> shared_state_names;
	private int current_depth = 0;
	private HashMap<State, ArrayList<State>> jointStateRobotStateMapping;
	State initState = null;
	Random randomGen;
	JointPolicyPaper uctPolicy;
	ArrayList<MDStrategy> defaultStrategies;
	ArrayList<List<State>> allRobotsStatesList;
	static ArrayList<Boolean> varlistsDontMatch; //basically keeping track of whether the prodmodgen varlists and mdp varlists match. uff so much mehnat. 
	boolean doProb = true;
	DA<BitSet, ? extends AcceptanceOmega> da; //just to store stuff 

	
	JointPolicyBRTDP brtdpPolicy;
	static ArrayList<ArrayList<HashMap<State,Double>> > probCostBoundsInits ;
	public MRuctPaper(PrismLog log, ArrayList<ProductModelGenerator> robotProdModelGens, int max_rollouts, int rollout_depth,
			ArrayList<String> shared_state_names, List<Double> dadistToAcc,
			ArrayList<MDStrategy> singleRobotSols, ArrayList<List<State>> allRobotsStatesList,
			ArrayList<Boolean> flipedIndices, DA<BitSet, ? extends AcceptanceOmega> da,
			ArrayList<ArrayList<HashMap<State,Double>> > probCostBoundsInitsToCopy )

	{
		probCostBoundsInits = probCostBoundsInitsToCopy;	
		this.da = da;
		varlistsDontMatch = flipedIndices;
		this.allRobotsStatesList = allRobotsStatesList;
		defaultStrategies = singleRobotSols;
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
		brtdpPolicy = new JointPolicyBRTDP(mainLog, num_robots, 1, 0, null, null,da_distToAcc);
		randomGen = new Random();
		// so the maximum failure cost should be the number of steps
		// because with the max cost of other things being 1 you can never do worse
		// or even smaller ?
		// possibly
		MAXFAILCOST = 1;// 10/0.2;
		MINFAILCOST = -MAXFAILCOST;

	}

	static State reverseState(State s)
	{
		Object[] sv = s.varValues;
		int lensv = sv.length;
		State rs = new State(sv.length); 
		for(int i = lensv-1; i>=0; i--)
		{
		
			rs.setValue((lensv-1)-i, sv[i]);
		}
		return rs; 
	}
	//get default prob cost 
	static double getInitProbBound(State js)
	{
		ArrayList<State> robotStates = getRobotStatesFromJointState(js); 
		//now get the first robots state for this 
		State s = robotStates.get(0); 
		if(varlistsDontMatch.get(0))
		{
			//flip it 
			s = reverseState(s);
			
		}
		
		//get that states prob 
		//minProb 
		//since prob is a lower bound do the max 
		double currentprob = probCostBoundsInits.get(0).get(0).get(s);
		double prob = 0.0; 
		for(int i = 1; i<robotStates.size(); i++)
		{
			 s = robotStates.get(i); 
				if(varlistsDontMatch.get(i))
				{
					//flip it 
					s = reverseState(s);
					
				}
				prob = probCostBoundsInits.get(i).get(0).get(s);
				if(prob > currentprob)
					currentprob = prob; 
				
		}
		
		return currentprob; 
		
	}
	//get default prob cost 
	static	double getInitCostBound(State js)
		{
			ArrayList<State> robotStates = getRobotStatesFromJointState(js); 
			//now get the first robots state for this 
			int i = 0; 
			double cost = 0.0; 
			double currentCost = 0.0; 
			
			State s = robotStates.get(i); 
			if(varlistsDontMatch.get(i))
			{
				//flip it 
				s = reverseState(s);
				
			}
			
			//get that states cost
			cost = probCostBoundsInits.get(i).get(1).get(s);
			//do min cost always 
			for(i = 1 ; i<robotStates.size(); i++)
			{
				 s = robotStates.get(i); 
				if(varlistsDontMatch.get(i))
				{
					//flip it 
					s = reverseState(s);
					
				}
				
				//get that states cost
				currentCost = probCostBoundsInits.get(i).get(1).get(s);
				//why am I doing max cost and not min 
				//because we have failure states - and this is from the trimmed single robot solution 
				//so for failure states, one of the robots may have a cost 0 while the other one might have not 0 
				//the actual bound is not 0 , its the other robots 
				//this works in tandem with the prob selection really, 
				//cuz we're choosing the higher prob one which is good, cuz the higher prob one does not fail!! 
				if(cost < currentCost)
					cost = currentCost; 
			}
			return cost; 
			
		}
	ArrayList<State> getJointStates(ArrayList<ArrayList<State>> succStates) throws PrismException
	{
		ArrayList<State> jointStates = new ArrayList<State>();
		for (int i = 0; i < succStates.size(); i++)
			jointStates.add(getJointState(succStates.get(i), null));
		return jointStates;
	}

	State getJointState(ArrayList<State> states, BitSet accStates) throws PrismException
	{

		State temp_joint_state = new State(num_robots + 1);
		BitSet trueLabels = new BitSet();
		for (int i = 0; i < num_robots; i++) {

			State current_model_state = states.get(i);
			// collect non da values
			// knowing that the da value is the last one
			// TODO: shared state stuff
			int da_state_index = prodModGens.get(i).getVarIndex(da_id_string);
			prodModGens.get(i).exploreState(current_model_state);
			ProductModelGenerator pmg = prodModGens.get(i);

			for (int labnum = 0; labnum < pmg.getNumLabelExprs(); labnum++) {
				if (prodModGens.get(i).isExprTrue(labnum))
					trueLabels.set(labnum);
			}
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
		//we may need to update the temp_joint_state 
		//from the temp joint state get the da state 
		int current_da_state = getIndividualState(temp_joint_state, num_robots, 0);
		//now lets do the checking 

		int updated_da_state = this.da.getEdgeDestByLabel(current_da_state, trueLabels);
		if (current_da_state != updated_da_state) {
			State newda = new State(1);
			newda.setValue(0, updated_da_state);
			//then we need to update it 
			temp_joint_state.setValue(num_robots, newda);

		}
		if (!jointStateRobotStateMapping.containsKey(temp_joint_state)) {
			ArrayList<State> cp = new ArrayList<State>();
			cp.addAll(states);
			jointStateRobotStateMapping.put(temp_joint_state, cp);
		}
		return temp_joint_state;
	}

	Entry<Double,Double> getStateCost(State state)
	{
	
	
		State dastate = state.substate(num_robots, num_robots + 1);
		dastate = (State) dastate.varValues[0];
		int dastateint = (int) dastate.varValues[0];
		double dist = da_distToAcc.get(dastateint);
		double prob = 0; 
		if (dist == 0) {
			mainLog.println(state.toString());
			uctPolicy.setAsAccState(state);
			accFound = true;
			if (doProb) {
				prob = 1;
			}
		} 
		

	
		return new AbstractMap.SimpleEntry<Double,Double>(prob,dist);
	}

	double getRobotsReward(State state, Object act,boolean sumCost) throws PrismException
	{
		ArrayList<State> robotStates = getRobotStatesFromJointState(state); 
		ArrayList<Object> robotActions = splitJointAction(act);
		// get the da state
		double cost = 0; 
		if(sumCost) {
		for(int i = 0; i<robotStates.size(); i++)
			cost+= getRobotReward(i,robotStates.get(i),robotActions.get(i));
		}
		else
		{
			//max cost 
			double currentCost =getRobotReward(0,robotStates.get(0),robotActions.get(0));
			for(int i = 1; i<robotStates.size(); i++)
			{
				cost = getRobotReward(i,robotStates.get(i),robotActions.get(i));
				if(currentCost < cost)
					currentCost = cost; 
			}
			cost = currentCost; 
		}
		return cost; 
	}
	double getRobotReward(int rnum,State state,Object act) throws PrismException
	{
		//assuming there is just a single reward 
		ProductModelGenerator pmg = this.prodModGens.get(rnum); 
		double rew = pmg.getStateActionReward(0, state, act);
		return rew; 
	
	}
	boolean isGoal(State state)
	{
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

	ArrayList<State> getInitialStates()
	{
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

	/**
	 * get actions for each robot's state
	 * 
	 * @param states = list of states, one per robot, ordered
	 * @return list of actions, one per robot , ordered
	 */
	HashMap<Integer, HashMap<Object, Integer>> getActions(ArrayList<State> states)
	{
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

	ArrayList<Object> getActionNamesFromRobotActionNums(int[] combination) throws PrismException
	{
		ArrayList<Object> actionNames = new ArrayList<Object>();
		for (int i = 0; i < combination.length; i++) {
			ProductModelGenerator prodModGen = prodModGens.get(i);
			Object action = prodModGen.getChoiceAction(combination[i]);
			actionNames.add(action);

		}
		return actionNames;
	}

	ArrayList<Integer> getNumActionsPerRobot(ArrayList<State> states) throws PrismException
	{

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

	HashMap<Integer, HashMap<State, Double>> getActionSuccessors(ArrayList<Object> actions, HashMap<Integer, HashMap<Object, Integer>> actionsMap)
			throws PrismException
	{
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

	ArrayList<ArrayList<Object>> getJointActions(HashMap<Integer, HashMap<Object, Integer>> actions) throws PrismException
	{
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

	Object getJointActionName(ArrayList<Object> jointAction)
	{
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

	ArrayList<Object> splitJointAction(Object jointAction)
	{
		String[] action_arr = jointAction.toString().split(",");
		ArrayList<Object> actions = new ArrayList<Object>();
		for (int i = 0; i < action_arr.length; i++)
			actions.add((Object) action_arr[i]);
		return actions;
	}

	ArrayList<Double> calculateJointStatesProbs(HashMap<Integer, HashMap<State, Double>> succStates, ArrayList<ArrayList<State>> combinations) throws PrismException
	{
		double normaliser = 0;

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
			for (int i = 0; i < probs.size(); i++) {
				probs.set(i, probs.get(i) / normaliser);
			}
			mainLog.println("Normalised!!!");
			throw new PrismException("Probabilities normalised");
		}
		return probs;
	}

	ArrayList<ArrayList<State>> getJointStateCombinations(HashMap<Integer, HashMap<State, Double>> succStates) throws PrismException
	{

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

	public State simulateAction(State state, Object action) throws PrismException
	{

		return simulateAction(state, action, false);
	}

	public State simulateAction(State state, Object action, boolean initState) throws PrismException
	{
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
							//the joint succStates are leaf states now 
							uctPolicy.setAsLeafState(jointsuccStates);
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
					throw new PrismException("Simulation: State not in robot state action indices");
					//				return null;
				}

			}
		} else
			throw new PrismException("Passed null action to simulate action");

		return succState;
	}

	// because sometimes a state just is a state so thats annoying
	private static int getIndividualState(State state, int ind, int subind)
	{
		Object s = state.varValues[ind];
		int sint;
		if (s instanceof State) {
			sint = getIndividualState((State) s, subind, 0);
		} else {
			sint = (int) s;
		}
		return sint;
	}

	static private ArrayList<State> getRobotStatesFromJointState(State state)
	{
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

	public Object monteCarloPlanning(BitSet accStates, boolean minCost) throws Exception
	{
		//		ArrayList<Integer> accsFound = new ArrayList<Integer>();

		if (doProb)
			minCost = false;

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
		// lets do the policy tree stuff here
		String saveplace = MRmcts.TESTSLOC;
		// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "no_door_example";
		uctPolicy.jointMDP.exportToDotFile(saveplace + filename + "_mctsmdp.dot");
		MDPSimple policyTree = uctPolicy.extractPolicyTreeAsDotFile(uctPolicy.jointMDP, uctPolicy.getStateIndex(temp_joint_state), minCost);
		policyTree.exportToDotFile(saveplace + filename + "_policy.dot");

		//		return accsFound;
		return res;
	}

	public Object monteCarloPlanning(State state, boolean minCost) throws Exception
	{
		Object bestAction = null;
		int current_depth = 0;
		boolean addToTreeInTrial = false;
		accFound = false;
		//		while (current_depth < rollout_depth) {	
		search(null, state, current_depth, minCost, addToTreeInTrial);
		if (accFound)
			mainLog.println("Acc Found");
		//		}
		//		bestAction = uctPolicy.selectBestAction(state, current_depth);
		return bestAction;
	}

	public double search(State ps, State state, int depth, boolean minCost, boolean addToTreeInTrial) throws Exception
	{
		String searchInfo = "";
		if (ps != null)
			searchInfo += ps.toString() + "->";
		searchInfo += state.toString();

		double cost_state = 0;
		if (isGoal(state)) {
			uctPolicy.setAsLeafState(state);
			Entry<Double, Double> probstatecost = getStateCost(state);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
			cost_state = stateCost;
			if(doProb)		
			cost_state = probCost;
			searchInfo += "=" + cost_state + "-ACC";
			mainLog.println(searchInfo);
			return cost_state;
		}
		if (isTerminal(ps, state)) {
			//remove from leaf states 
			//because we're saying terminal = self loop
			uctPolicy.removeFromLeafState(state);
			// dont want deadends
			double failcost = MINFAILCOST;
			if (minCost)
				failcost = MAXFAILCOST;
			if (doProb)
				failcost = 0;
			Entry<Double, Double> probstatecost = getStateCost(state);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
			cost_state = stateCost;
			if(doProb)		
			cost_state = probCost;
			cost_state += failcost;
			searchInfo += "=" + cost_state + "-terminal";
			mainLog.println(searchInfo);
			return cost_state;
		}
		if (depth == rollout_depth) {

			uctPolicy.setAsLeafState(state);
			Entry<Double, Double> probstatecost = getStateCost(state);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
			cost_state = stateCost;
			if(doProb)		
			cost_state = probCost;
			searchInfo += "=" + cost_state + "-maxd";
			mainLog.println(searchInfo);
			return cost_state;
		}
		if (isLeaf(state)) {
			//we're expanding this 
			//so unset it 
			uctPolicy.removeFromLeafState(state);
			//			if (!uctPolicy.stateIndices.containsKey(state))
			initialiseVisits(state, minCost);
			ArrayList<Entry<State, Object>> trialList = new ArrayList<Entry<State, Object>>();
			cost_state = evaluate(ps, state, depth, true, trialList, minCost, addToTreeInTrial);
			// now lets update this state with the reward
			searchInfo += "=" + cost_state + "-rollout";
			mainLog.println(searchInfo);
			mainLog.println("Trial:\n" + trialList.toString());
			return cost_state;
		}
		uctPolicy.removeFromLeafState(state);
		Object action = selectAction(state, depth, false, minCost);
		State succState = simulateAction(state, action, true);
		Entry<Double, Double> probstatecost = getStateCost(state);
		double probCost = probstatecost.getKey();
		double stateCost = probstatecost.getValue();
		double reward = stateCost;
		if(doProb)
			reward = probCost;
		double q = reward + search(state, succState, depth + 1, minCost, addToTreeInTrial);
		updateValue(state, action, q, depth, minCost);
		cost_state = q;
		searchInfo += "=" + cost_state + "-selection";
		mainLog.println(searchInfo);
		return cost_state;

	}

	void initialiseVisits(State state, boolean minCost) throws PrismException
	{
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
					mainLog.println(action.toString() + " added");
					//			if (!uctPolicy.stateVisited(state))
					uctPolicy.initialiseVisits(state, action);
					uctPolicy.addRobotStateActionIndices(state, action, allPossibleJointActions.get(actionNum), allPossibleActionsForState);
				}
				//			else
				//				uctPolicy.increaseVisits(state, action);
				//			}
			}
		}

		uctPolicy.printStateDetails(state, minCost);
	}

	private void updateValue(State state, Object action, double q, int depth, boolean minCost)
	{
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
		} else {
			double temp = (q - qudpate);
			temp = temp / (double) uctPolicy.getNumVisits(state, action);
			qudpate += temp;
		}
		uctPolicy.updateQValue(state, action, qudpate);

	}

	//	private boolean isGoal (State state)
	//	{
	//		if (accFound)
	//			return true;
	//		else
	//			return false;
	//	}
	private boolean isTerminal(State pstate, State state)
	{

		if (pstate != null) {
			if (state.equals(pstate))
				return true;
		}

		return false;
	}

	private boolean isLeaf(State state)
	{
		// its a leaf if
		// it is not in the tree
		// so basically the state hasnt been added
		// which means it should not exist in the
		// uctPolicy stateIndices
		if (uctPolicy.toExpand(state, true))
			return true;
		else

			return false;

	}

	private double evaluate(State pstate, State state, int depth, boolean firstState, ArrayList<Entry<State, Object>> trial, boolean minCost,
			boolean addToTreeInTrial) throws Exception
	{
		double sumrew = 0;
		State succState = null;
		if (depth == rollout_depth)
		{			Entry<Double, Double> probstatecost = getStateCost(state);
		double probCost = probstatecost.getKey();
		double stateCost = probstatecost.getValue();
		if(doProb)
			return probCost;
		
			return stateCost;
		}
		if (isGoal(state)) {
			Entry<Double, Double> probstatecost = getStateCost(state);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
			if(doProb)
				return probCost;
			
				return stateCost;
		}
		if (isTerminal(pstate, state)) {
			double failcost = MINFAILCOST;
			if (minCost)
				failcost = MAXFAILCOST;
			if (doProb)
				failcost = 0;
			Entry<Double, Double> probstatecost = getStateCost(state);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
			if(doProb)
				return probCost + failcost;
			
				return stateCost +failcost;
		}
		// do a whole run
		try {
			Entry<Double, Double> probstatecost = getStateCost(state);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
			
			double reward = stateCost;
			if(doProb)
				reward = probCost;
			Object action = selectAction(state, depth, true, minCost);
			trial.add(new AbstractMap.SimpleEntry<State, Object>(state, action));
			succState = simulateAction(state, action, (firstState || addToTreeInTrial));

			sumrew = reward + evaluate(state, succState, depth + 1, false, trial, minCost, addToTreeInTrial);
			if (firstState || addToTreeInTrial) {
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

	// just writing this because the states seem to flip
	boolean statesEqualFlip(State s1, State s2)
	{
		// flip one of these
		Object[] v1 = s1.varValues;
		Object[] v2 = s2.varValues;
		if (v1.length == v2.length) {
			for (int i = 0; i < v1.length; i++) {
				if ((int) v1[i] != (int) v2[v1.length - 1 - i]) {
					return false;
				}
			}
			return true;

		}
		return false;
	}

	// TODO: it seems that the states are reversed
	// dont know why it happens please fix it later
	int findRobotStateIndex(State state, int rnum)
	{
		List<State> statesList = allRobotsStatesList.get(rnum);
		boolean check = false;
		for (int i = 0; i < statesList.size(); i++) {
			State s = statesList.get(i);
			if (varlistsDontMatch.get(rnum)) {
				check = statesEqualFlip(state, s);
			} else {
				check = s.equals(state);
			}

			if (check) {
				return i;
			}
		}
		return -1;
	}

	
	Object getActionUsingIndividualRobotPolicies(State state) throws PrismException
	{
		ArrayList<State> robotStates = getRobotStatesFromJointState(state);
		ArrayList<Object> robotActions = new ArrayList<Object>();
		int[] actionIndices = new int[num_robots];
		for (int i = 0; i < robotStates.size(); i++) {
			MDStrategy strat = this.defaultStrategies.get(i);
			// we need the mapping
			int index = findRobotStateIndex(robotStates.get(i), i);
			if (index != -1) {
				// strat.initialise(index);
				if (strat.isChoiceDefined(index)) {
					int actionIndex = strat.getChoiceIndex(index);

					Object action = strat.getChoiceAction(index);
					if (action.toString().contains("*"))
						throw new PrismException("* action for this state");
					//if this action is * 
					//we have to choose something else 
					//basically get an action from the robots model 
					//for this state 
					//and then the label too 

					robotActions.add(action);
					actionIndices[i] = actionIndex;
				} else {
					//get a random action 
					ProductModelGenerator prodModGen = this.prodModGens.get(i);
					prodModGen.exploreState(robotStates.get(i));
					int numActions = prodModGen.getNumChoices();
					int action_choice = (new Random()).nextInt(numActions);

					Object action = prodModGen.getChoiceAction(action_choice);
					robotActions.add(action);
					actionIndices[i] = action_choice;
					//					throw new PrismException("Undefined choice"); 
				}

			} else {
				//basically this state is not possible in the single robot model 
				//but possible in the joint robot model 
				//so we have to do the random action thing again 
				//talk about this 
				//get a random action 
				ProductModelGenerator prodModGen = this.prodModGens.get(i);
				prodModGen.exploreState(robotStates.get(i));
				int numActions = prodModGen.getNumChoices();
				int action_choice = (new Random()).nextInt(numActions);

				Object action = prodModGen.getChoiceAction(action_choice);
				robotActions.add(action);
				actionIndices[i] = action_choice;
				//				throw new PrismException("Can not select action according to base strategy - Robot State -1");
			}
			// strat.initialise(s);

		}

		Object joint_action = this.getJointActionName(robotActions);
		uctPolicy.addRobotStateActionIndices(state, joint_action, robotActions, actionIndices);
		return joint_action;

	}

	private Object chooseRandomAction(State state, boolean minCost) throws PrismException
	{
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

	}

	private Object chooseRandomActionNotVisited(State state) throws PrismException
	{
		// action not added to uctPolicy
		ArrayList<State> currentStates = getRobotStatesFromJointState(state);
		// get all actions
		ArrayList<Integer> numActionsPerRobot;// = getNumActionsPerRobot(currentStates);
		// get the number of combinations
		ArrayList<int[]> res = new ArrayList<int[]>();

		numActionsPerRobot = getNumActionsPerRobot(currentStates);
		uctPolicy.generateCombinations(numActionsPerRobot, res);
		// now choose a combination at random
		// since we are not following a policy
		int chosen_action_num = (new Random()).nextInt(res.size());
		// now we get that combination
		int[] chosen_action_comb = res.get(chosen_action_num);
		// fix chosen_action_comb
		for (int i = 0; i < chosen_action_comb.length; i++)
			chosen_action_comb[i]--;
		// now we create an action from that
		ArrayList<Object> actionNames = getActionNamesFromRobotActionNums(chosen_action_comb);
		// return the joint action
		// also add these indices
		Object joint_action = getJointActionName(actionNames);
		uctPolicy.addRobotStateActionIndices(state, joint_action, actionNames, chosen_action_comb);

		return joint_action;
	}

	private Object selectAction(State state, int depth, boolean useRolloutPolicy, boolean minCost) throws PrismException
	{
		boolean useStrategy = true;
		Object joint_action;
		if (useRolloutPolicy) {

			// use single robot policy
			if (useStrategy) {
				// dostuff here
				// convert this state into the state for each robot
				// get each robot's action
				joint_action = this.getActionUsingIndividualRobotPolicies(state);
				// check here to see if it contains a *
				return joint_action;

			} else {
				// choose an action randomly
				if (uctPolicy.stateActionVisits.containsKey(state)) {
					joint_action = chooseRandomAction(state, minCost);
					return joint_action;

				} else {
					joint_action = chooseRandomActionNotVisited(state);
					return joint_action;
				}
			}
		} else {
			return uctPolicy.getBestAction(state, minCost);
		}

	}

	private double updateUpperValue(State state, double probCost,double cost,boolean sumCosts,boolean printVal) throws PrismException
	{
		if(printVal)
			mainLog.println("Updating Upper Values");
		//step 1 
		//get all the actions 
		//check if we dont have actions stored already 
		if (!brtdpPolicy.robotStateActionIndices.containsKey(state)) {
			// action not added to uctPolicy

			ArrayList<State> currentStates = getRobotStatesFromJointState(state);
			HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
			ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);
			for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
				Object action = getJointActionName(allPossibleJointActions.get(actionNum));

				//add these robot action indices to our list 
				brtdpPolicy.addRobotStateActionIndices(state, action, allPossibleJointActions.get(actionNum), allPossibleActionsForState);
				//so we have to add this action 
				addStateActionToMDP(state, action,sumCosts);
				
			}
		}

		//now that we have all the actions 
		//		double stateCost = this.getStateCost(state); 
		Entry<Double, Double> stateUpperV = brtdpPolicy.getMinMaxValue(state, probCost, 
				!doProb, true,brtdpPolicy.isGoal(state),printVal);
		if(printVal)
			mainLog.println("Updating Upper Bounds with "+stateUpperV.toString());
		double updatedProb = stateUpperV.getKey(); 
		double updatedCost = stateUpperV.getValue();
		brtdpPolicy.setProbCostValue(state, updatedProb,updatedCost, true);
		return stateUpperV.getKey();

	}

	double updateLowerValue(State state, double probCost,double cost,boolean printVal) throws PrismException
	{
		if(printVal)
			mainLog.println("Updating Lower Value");
		Entry<Double, Double> stateLowerV = brtdpPolicy.getMinMaxValue(state, probCost,cost,
				!doProb, false,isGoal(state),printVal);
		if(printVal)
			mainLog.println("Updated Lower Bounds "+stateLowerV.toString());
		brtdpPolicy.setProbCostValue(state, stateLowerV.getKey(),stateLowerV.getValue(), false);
		
		return stateLowerV.getKey();
	}

	double updateLowerValueAction(State state, Object act, double probCost,double stateCost,boolean printVal) throws PrismException
	{
		if(printVal)
			mainLog.println("Updating Lower Value Action");
		boolean isgoal = brtdpPolicy.isGoal(state);
//		double stateLowerV = brtdpPolicy.getProbValueIgnoreSelfLoop(state, false, isgoal);
		double probqval = brtdpPolicy.getProbQValue(state, act, probCost, false, isgoal,printVal);
		double costqval = brtdpPolicy.getCostQValue(state, act, stateCost, false,printVal); 
		if(printVal)
			mainLog.println("Updated Lower Bounds with "+act.toString()+":"+probqval+"="+costqval);
		brtdpPolicy.setProbCostValue(state, probqval, costqval, false);
		
		return probqval;
	}

	Object getLowerValueAction(State state, double probCost,double cost,boolean printVal) throws PrismException
	{
		if(printVal)
			mainLog.println("Getting Lower Action");
		if (!brtdpPolicy.robotStateActionIndices.containsKey(state))
			throw new PrismException("State not added to robotStateActionIndices" + state.toString());
		//		double stateCost = getStateCost(state); 
		Object action = brtdpPolicy.getMinMaxAction(state, probCost, !doProb, false,printVal);
		if(printVal)
			mainLog.println("Chose action - "+action.toString());
		return action;
	}

	void addStateActionToMDP(State state, Object action,boolean sumCosts) throws PrismException
	{
		//add this state and action slowly 
		//		//add this action to our list 
		ArrayList<State> currentStates = getRobotStatesFromJointState(state);
		ArrayList<Integer> actionIndices = brtdpPolicy.robotStateActionIndices.get(state).get(action);
		HashMap<Integer, HashMap<State, Double>> succStates = new HashMap<Integer, HashMap<State, Double>>();
		for (int i = 0; i < num_robots; i++) {
			HashMap<State, Double> ahashmap = new HashMap<State, Double>();
			int choice = actionIndices.get(i);
			ProductModelGenerator pmg = prodModGens.get(i);
			pmg.exploreState(currentStates.get(i));
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

		brtdpPolicy.addAction(state, action, jointsuccStates, probs);
		double reward = this.getRobotsReward(state, action,sumCosts);
		brtdpPolicy.addActionReward(state, action,reward );

	}

	void BRTDPTrial(State state, Stack<State> traj, double tau,boolean sumCosts) throws PrismException
	{
		boolean printVal = true; 
		if(MRmcts.TURNOFFALLWRITES)
			printVal = false; 
		//setting a max traj length 
		//just a quick thing really 
		int maxTrajectoryLen = 10; 
		State currState = state;

		double successorsDiff = 1000;
		double stateDiff = brtdpPolicy.getProbValueDiff(state, brtdpPolicy.isGoal(state),true);
		boolean comp = successorsDiff > (stateDiff / tau);
		while (comp) {
			
			if(printVal) {
				mainLog.println("*************************************************************");
				mainLog.println("*************************************************************");
				brtdpPolicy.printAllStatesValues();
				mainLog.println("*************************************************************");
				mainLog.println("*************************************************************");
			mainLog.println("=============================================================");
			mainLog.println("=============================================================");
			}
			traj.push(currState);
			Entry<Double, Double> probstatecost = getStateCost(currState);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
//			double probCost = getStateCost(currState);
			if (printVal)
			{	
				mainLog.println(currState.toString()+":"+probCost+","+stateCost); 
			brtdpPolicy.printStateValues(currState);
			mainLog.println("=============================================================");
			}
			updateUpperValue(currState, probCost,stateCost,sumCosts,printVal);
			if(printVal)
			mainLog.println("=============================================================");
			Object action = getLowerValueAction(currState, probCost,stateCost,printVal);
			if(printVal)
			mainLog.println("=============================================================");
			updateLowerValueAction(currState, action, probCost,stateCost,printVal);
			if(printVal) {
			mainLog.println("=============================================================");
			brtdpPolicy.printStateValues(currState);
			mainLog.println("=============================================================");
			}
			//now we have to get all the successors 
			HashMap<State, Double> succs = brtdpPolicy.getSuccessors(currState, action);
			if (succs == null) {
				//we need to add the action and stuff 
				addStateActionToMDP(currState, action,sumCosts);
				succs = brtdpPolicy.getSuccessors(currState, action);
			}
			//so now we have all the successors 
			//we have to do a loop thing 
			double sumB = 0.0;
			double b = 0.0;
			double rand = new Random().nextDouble();
			double sumProb = 0.0;
			State chosenSuccState = null;
			//assuming the states are ordered by prob 
			HashMap<State,Double> statebs = new HashMap<State,Double>();
			for (State s : succs.keySet()) {
				double prob = succs.get(s);
				double succStateDiff = brtdpPolicy.getProbValueDiff(s, isGoal(s),printVal);
				b = prob * succStateDiff;
				statebs.put(s,b);
				sumB += b;
//				if (sumProb < rand) {
//					sumProb += prob;
//					if (sumProb > rand) {
//						chosenSuccState = s;
//					}
//				}
//				if (rand == 0) { 
//					if (chosenSuccState == null) {
//						chosenSuccState = s;
//					}
//				}

			}
			if(sumB != 1.0) {
			for(State s: statebs.keySet())
			{
			 b = statebs.get(s); 
			 statebs.put(s, b/sumB); 
			}}
			
			for(State s: statebs.keySet())
			{
				//assuming ordered by prob
				if(sumProb < rand)
				{
					sumProb+=statebs.get(s); 
					if(sumProb>rand)
					{
						chosenSuccState =s; 
						break; 
					}
				}
				if(rand == 0)
				{
					chosenSuccState =s; 
					break; 
				}
			}
				
			//now we do the check 

			successorsDiff = sumB;
			stateDiff = brtdpPolicy.getProbValueDiff(state, isGoal(state),printVal);
			comp = successorsDiff > (stateDiff / tau);
			currState = chosenSuccState;
			if(printVal) {
			mainLog.println("=============================================================");
			mainLog.println("=============================================================");
			}
			if(traj.size() > maxTrajectoryLen)
				break;
		}

	}

	void BRTDPBackup(Stack<State> traj,boolean sumCosts) throws PrismException
	{
		boolean printVal = true; 
		if(MRmcts.TURNOFFALLWRITES)
			printVal = false; 
		//do stuff here 
		while (!traj.isEmpty()) {
			State s = traj.pop();
			Entry<Double, Double> probstatecost = getStateCost(s);
			double probCost = probstatecost.getKey();
			double stateCost = probstatecost.getValue();
			if(printVal) {
			mainLog.println(s.toString()+":"+probCost+","+stateCost); 
			brtdpPolicy.printStateValues(s);
			}
			this.updateUpperValue(s, probCost,stateCost,sumCosts,printVal);
			this.updateLowerValue(s, probCost,stateCost,printVal);
			if(printVal)
			brtdpPolicy.printStateValues(s);

		}
	}

	void runBRTDPTrial(State state, double tau) throws PrismException
	{

		boolean sumCosts = false;  //do not sum robot costs, get the max cost instead 
		
		//if sumcosts is true then our upper bound for cost will change 
		//well even now it will not be a true upper bound cuz the other one is moving still 
		//so we have to do something about this 
		Stack<State> trajectory = new Stack<State>();

		mainLog.println("BRTDP Trial");
		BRTDPTrial(state, trajectory, tau,sumCosts);
		mainLog.println("BRTDP Backup");
		BRTDPBackup(trajectory,sumCosts);
	}

	//a clean brtdp implementation 
	//for probs only 
	//v_u = 1 
	//v_l = 0
	public Object doBRTDP(BitSet accStates, boolean minCost) throws Exception
	{
		//		ArrayList<Integer> accsFound = new ArrayList<Integer>();

		if (doProb)
			minCost = false;

		boolean printVal = true; 
		if(MRmcts.TURNOFFALLWRITES)
			printVal = false; 
		current_rollout_num = 0;
		ArrayList<State> initialStates = getInitialStates();
		// get the initial state
		State temp_joint_state = getJointState(initialStates, null);
		mainLog.println("Joint State:" + temp_joint_state.toString());
		Object res = null;
		double alpha = 0.1; //change this later 
		double tau = 50; //they used something between 10 and 100 
		
		double stateDiff = brtdpPolicy.getProbValueDiff(temp_joint_state, isGoal(temp_joint_state),printVal);
		double oldStateDiff = 100; 
		String saveplace = MRmcts.TESTSLOC;
		// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "no_door_example";
		int iter = -1; 
		while (stateDiff > alpha) {
			iter++;
			runBRTDPTrial(temp_joint_state, tau);
			oldStateDiff = stateDiff;
			stateDiff = brtdpPolicy.getProbValueDiff(temp_joint_state, isGoal(temp_joint_state),printVal);
			
			String stateDiffString = ""+stateDiff;
			
			stateDiffString = stateDiffString.replace(".", "_");
			mainLog.println("State Diff "+stateDiff);
			brtdpPolicy.exportAllStatesValues(saveplace + filename + "_brtdpmdp_values_"+stateDiffString+"_"+iter+".txt");
			
			if((stateDiff - oldStateDiff) == 0)
			{
				//there is really no change 
				mainLog.println("No difference in the state difference - so we're done");
				if(iter > 30)
				break; 
			}

//			mainLog.println("Printing State Values for "+stateDiffString); 
//			brtdpPolicy.printAllStatesValues();
			

//			brtdpPolicy.jointMDP.exportToDotFile(saveplace + filename + "_brtdpmdp_"+stateDiffString+".dot");
			PrismFileLog out = new PrismFileLog(saveplace + filename + "_brtdpmdp_"+stateDiffString+".dot");
			brtdpPolicy.jointMDP.exportToDotFile(out, null, true);
			out.close();
			MDPSimple policyTree = brtdpPolicy.extractPolicyTreeAsDotFile(brtdpPolicy.jointMDP, brtdpPolicy.getStateIndex(temp_joint_state), minCost);
//			policyTree.exportToDotFile(saveplace + filename + "_brtdp_policy_"+stateDiffString+".dot");
			out = new PrismFileLog(saveplace + filename + "_brtdp_policy_"+stateDiffString+".dot");
			policyTree.exportToDotFile(out, null, true);
			out.close();
		

		}
		
		
		//		while (current_rollout_num < max_num_rollouts) {
		//			mainLog.println("Rollout Attempt: " + current_rollout_num);
		//			res = monteCarloPlanning(temp_joint_state, minCost);// rollout_stateactionstateonly(initialStates,
		//			// current_rollout_num);
		//			//				if (res == null)
		//			////					accsFound.add(res);
		//			current_rollout_num++;
		//
		//		}
		//		// lets do the policy tree stuff here
				

		//		return accsFound;
		return res;
	}

}
