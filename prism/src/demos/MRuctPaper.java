package demos;

import java.io.IOException;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Map.Entry;

import acceptance.AcceptanceOmega;
import automata.DA;
import explicit.Distribution;
import explicit.MDPSimple;

import java.util.Random;
import java.util.Stack;
import parser.State;
import parser.VarList;
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

	int DASTATEINDEX;
	int SSSTATEINDEX;
	static int TSTATEINDEX;
	static int ACTSTATEINDEX;
	double EPSILON = 10e-6;
	private int numTrials = 0;
	private PrismLog mainLog;
	static private ArrayList<ProductModelGenerator> prodModGens;
	private static int num_robots;
	private List<Double> da_distToAcc;
	private static String da_id_string = "_da0";
	static private ArrayList<String> shared_state_names;

	static List<State> deadendStates;
	static List<State> notDeadendStates;
	private HashMap<State, ArrayList<State>> jointStateRobotStateMapping;
	State initState = null;
	//	Random randomGen;
	//	JointPolicyPaper uctPolicy;
	ArrayList<MDStrategy> defaultStrategies;
	ArrayList<List<State>> allRobotsStatesList;
	static ArrayList<Boolean> varlistsDontMatch; //basically keeping track of whether the prodmodgen varlists and mdp varlists match. uff so much mehnat. 
	boolean doProb = true;
	DA<BitSet, ? extends AcceptanceOmega> da; //just to store stuff 
	static ArrayList<ArrayList<Integer>> sharedStateIndicesForRobots;

	JointPolicyBRTDP brtdpPolicy;
	State referenceSSState = null;
	static ArrayList<ArrayList<HashMap<State, Double>>> probCostBoundsInits;

	public MRuctPaper(PrismLog log, ArrayList<ProductModelGenerator> robotProdModelGens, int max_rollouts, int rollout_depth,
			ArrayList<String> sharedStateNames, List<Double> dadistToAcc, ArrayList<MDStrategy> singleRobotSols, ArrayList<List<State>> allRobotsStatesList,
			ArrayList<Boolean> flipedIndices, DA<BitSet, ? extends AcceptanceOmega> da, ArrayList<ArrayList<HashMap<State, Double>>> probCostBoundsInitsToCopy)

	{

		deadendStates = new ArrayList<State>();
		notDeadendStates = new ArrayList<State>();
		probCostBoundsInits = probCostBoundsInitsToCopy;
		this.da = da;
		varlistsDontMatch = flipedIndices;
		this.allRobotsStatesList = allRobotsStatesList;
		defaultStrategies = singleRobotSols;
//		jointStateRobotStateMapping = new HashMap<State, ArrayList<State>>();
		num_robots = robotProdModelGens.size();
		prodModGens = robotProdModelGens;
		mainLog = log;
		if (sharedStateNames != null) {
			if (sharedStateNames.size() > 0)
				shared_state_names = sharedStateNames;
			else
				shared_state_names = null;
		}
		da_distToAcc = dadistToAcc;
		//		uctPolicy = new JointPolicyPaper(mainLog, num_robots, 1, 0, shared_state_names, null);
		brtdpPolicy = new JointPolicyBRTDP(mainLog, num_robots, 1, 0, shared_state_names, null, da_distToAcc);
		sharedStateIndicesForRobots = new ArrayList<ArrayList<Integer>>();
		//use the prodmodgens to get the number of state features 
		//as well as the shared states list 
		if (shared_state_names != null) {
			for (int i = 0; i < prodModGens.size(); i++) {
				ProductModelGenerator pmg = prodModGens.get(i);

				ArrayList<Integer> sharedStateIndicesForRobot = new ArrayList<Integer>();

				try {
					VarList vl = pmg.createVarList();

					for (int j = 0; j < shared_state_names.size(); j++) {
						if (vl.exists(shared_state_names.get(j))) {
							sharedStateIndicesForRobot.add(vl.getIndex(shared_state_names.get(j)));
						} else {
							//doesn't exist for this robot so its -1 
							//so technically we have like an organised ss thing 
							sharedStateIndicesForRobot.add(-1);
						}
					}
				} catch (PrismException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				sharedStateIndicesForRobots.add(sharedStateIndicesForRobot);
			}
			SSSTATEINDEX = num_robots;
			DASTATEINDEX = SSSTATEINDEX + 1;
			TSTATEINDEX = DASTATEINDEX + 1;
			ACTSTATEINDEX = TSTATEINDEX + 1;
		} else {
			SSSTATEINDEX = -1;
			DASTATEINDEX = num_robots;
			TSTATEINDEX = DASTATEINDEX + 1;
			ACTSTATEINDEX = TSTATEINDEX + 1;
		}

	}

	static State reverseState(State s)
	{
		Object[] sv = s.varValues;
		int lensv = sv.length;
		State rs = new State(sv.length);
		for (int i = lensv - 1; i >= 0; i--) {

			rs.setValue((lensv - 1) - i, sv[i]);
		}
		return rs;
	}

	static boolean isDeadend(State js)
	{
		ArrayList<State> robotStates = getRobotStatesFromJointState(js);
		return isDeadend(js, robotStates);
	}

	static boolean isDeadend(State js, ArrayList<State> robotStates)
	{
		boolean deadend = false;
		if (!deadendStates.contains(js)) {
			if (!notDeadendStates.contains(js)) {
				int numDeadends = 0;
				for (int i = 0; i < robotStates.size(); i++) {
					State robotState = robotStates.get(i);
					HashMap<State, Double> progVals = probCostBoundsInits.get(i).get(2);
					if (progVals.containsKey(robotState)) {
						if (progVals.get(robotState) <= 0) {
							numDeadends++;
						}
					} else {
						//if this state does not exist then we can't get to the goal with this state 
						//then its a deadend 
						numDeadends++;
					}
				}
				if (numDeadends == robotStates.size()) {
					deadend = true;
				}
				if (deadend)
					deadendStates.add(js);
				else
					notDeadendStates.add(js);
			}

		} else {
			deadend = true;
		}

		return deadend;
	}

	//get default prob cost 
	static double getInitProbBound(State js)
	{
		ArrayList<State> robotStates = getRobotStatesFromJointState(js);
		//		boolean isdeadend = isDeadend(js, robotStates);
		HashMap<State, Double> probVals = probCostBoundsInits.get(0).get(0);
		//now get the first robots state for this 
		State s = robotStates.get(0);
		if (varlistsDontMatch.get(0)) {
			//flip it 
			s = reverseState(s);

		}

		//get that states prob 
		//minProb 
		//since prob is a lower bound do the max 
		double currentprob = 0;
		if (probVals.containsKey(s))
			currentprob = probVals.get(s);
		double prob = 0.0;
		for (int i = 1; i < robotStates.size(); i++) {
			probVals = probCostBoundsInits.get(i).get(0);
			prob = 0.0;
			s = robotStates.get(i);
			if (varlistsDontMatch.get(i)) {
				//flip it 
				s = reverseState(s);

			}
			if (probVals.containsKey(s))
				prob = probVals.get(s);
			if (prob > currentprob)
				currentprob = prob;

		}

		//		if (isdeadend)
		//			currentprob = 0;

		return currentprob;

	}

	static double getInitProgBound(State js, boolean lower)
	{
		ArrayList<State> robotStates = getRobotStatesFromJointState(js);
		int boundNum = 2;
		//		boolean isdeadend = isDeadend(js, robotStates);
		HashMap<State, Double> progVals = probCostBoundsInits.get(0).get(boundNum);
		//now get the first robots state for this 
		State s = robotStates.get(0);
		if (varlistsDontMatch.get(0)) {
			//flip it 
			s = reverseState(s);

		}

		//get that states prob 
		//minProb 
		//since prob is a lower bound do the max 
		double currentprog = 0;
		if (progVals.containsKey(s))
			currentprog = progVals.get(s);
		double prog = 0.0;
		for (int i = 1; i < robotStates.size(); i++) {
			progVals = probCostBoundsInits.get(i).get(boundNum);
			prog = 0.0;
			s = robotStates.get(i);
			if (varlistsDontMatch.get(i)) {
				//flip it 
				s = reverseState(s);

			}
			if (progVals.containsKey(s))
				prog = progVals.get(s);

			if (lower) {
				if (prog > currentprog)
					currentprog = prog;
			} else {
				currentprog += prog;
			}
		}

		return currentprog;

	}

	//get default prob cost 
	static double getInitCostBound(State js)
	{

		ArrayList<State> robotStates = getRobotStatesFromJointState(js);
		//		boolean isdeadend = isDeadend(js, robotStates);
		//now get the first robots state for this 
		int i = 0;
		double cost = 0.0;
		double currentCost = 0.0;

		HashMap<State, Double> costVals = probCostBoundsInits.get(i).get(1);
		State s = robotStates.get(i);
		if (varlistsDontMatch.get(i)) {
			//flip it 
			s = reverseState(s);

		}

		//get that states cost
		if (costVals.containsKey(s))
			cost = costVals.get(s);
		//do min cost always 
		for (i = 1; i < robotStates.size(); i++) {
			currentCost = 0.0;
			costVals = probCostBoundsInits.get(i).get(1);
			s = robotStates.get(i);
			if (varlistsDontMatch.get(i)) {
				//flip it 
				s = reverseState(s);

			}

			//get that states cost
			if (costVals.containsKey(s))
				currentCost = costVals.get(s);
			//all this is WRONG the stuff below 
			//why am I doing max cost and not min 
			//because we have failure states - and this is from the trimmed single robot solution 
			//so for failure states, one of the robots may have a cost 0 while the other one might have not 0 
			//the actual bound is not 0 , its the other robots 
			//this works in tandem with the prob selection really, 
			//cuz we're choosing the higher prob one which is good, cuz the higher prob one does not fail!! 
			//			if (cost < currentCost)
			//				cost = currentCost;

			//what we need to do is add the costs 
			//otherwise we are not monotonic 
			cost += currentCost;
		}
		//		//		if (isdeadend)
		//		//			cost = 0;
		//		cost = 1000;
		return cost;

	}

	ArrayList<State> getJointStates(ArrayList<ArrayList<State>> succStates, State tstate, State astate) throws PrismException
	{
		ArrayList<State> jointStates = new ArrayList<State>();
		for (int i = 0; i < succStates.size(); i++)
			jointStates.add(getJointState(succStates.get(i), null, tstate, astate));
		return jointStates;
	}

	static private ArrayList<State> getRobotStatesFromJointState(State state)
	{
		if (shared_state_names == null)
			return getRobotStatesFromJointStateNoSS(state);
		else
			return getRobotStatesFromJointStateWithSS(state);

	}

	static private ArrayList<State> getRobotStatesFromJointStateWithSS(State state)
	{
		// TODO edit this for multiple state variables but for now lets just go with the
		// usual

		State da_state = state.substate(num_robots + 1, num_robots + 2);
		State ss_state = state.substate(num_robots, num_robots + 1);
		State ps_state;

		ArrayList<State> robotStates = new ArrayList<State>();
		State robotState;
		for (int i = 0; i < num_robots; i++) {

			ps_state = state.substate(i, i + 1);

			robotState = createRobotStateFromBrokenStates(ps_state, ss_state, da_state, i);

			robotStates.add(robotState);

		}
		return robotStates;
	}

	static State createRobotStateFromBrokenStates(State ps_state, State ss_state, State da_state, int rnum)
	{
		ArrayList<Integer> robotSharedStateIndices = sharedStateIndicesForRobots.get(rnum);
		int numRobotStateVars = prodModGens.get(rnum).getNumVars();
		State robotState = new State(numRobotStateVars);
		//		int ss_ind = 0;
		int ps_ind = 0;
		int da_state_index = prodModGens.get(rnum).getVarIndex(da_id_string);
		for (int i = 0; i < numRobotStateVars; i++) {
			if (i != da_state_index) {
				if (!robotSharedStateIndices.contains(i)) {
					int ps_state_val = getIndividualState(ps_state, 0, ps_ind);
					robotState.setValue(i, ps_state_val);
					ps_ind++;
				}
			} else {
				int da_state_val = getIndividualState(da_state, 0, 0);
				robotState.setValue(i, da_state_val);
			}
		}
		for (int i = 0; i < robotSharedStateIndices.size(); i++) {
			int ss_state_val = getIndividualState(ss_state, 0, i);
			int ind = robotSharedStateIndices.get(i);
			if (ind != -1)
				robotState.setValue(ind, ss_state_val);
		}
		return robotState;
	}

	//breaks the robot state into shared state and not shared state. 
	State breakRobotState(State state, int rnum)
	{
		// we've saved all the shared state indices 
		ArrayList<Integer> robotSharedStateIndices = sharedStateIndicesForRobots.get(rnum);
		//so basically go over all the states here and create a new state based on the shared state indices 
		Object[] svv = state.varValues;
		State ss = new State(robotSharedStateIndices.size());
		int numps = svv.length - (robotSharedStateIndices.size() + 1);
		State ps = new State(numps);
		int ssind = 0;
		int psind = 0;
		int da_state_index = prodModGens.get(rnum).getVarIndex(da_id_string);
		State ds = new State(1);
		for (int i = 0; i < svv.length; i++) {
			if (i != da_state_index) {
				if (!robotSharedStateIndices.contains(i)) {
					ps.setValue(psind, svv[i]);
					psind++;
				}
			} else {
				ds.setValue(0, svv[i]);
			}
		}
		for (int i = 0; i < robotSharedStateIndices.size(); i++) {
			int ind = robotSharedStateIndices.get(i);
			if (ind != -1) {
				ss.setValue(ssind, svv[ind]);
				ssind++;
			}
		}
		State combinedState = new State(3);
		combinedState.setValue(0, ps);
		combinedState.setValue(1, ss);
		combinedState.setValue(2, ds);
		return combinedState;
	}

	State getJointState(ArrayList<State> states, BitSet accStates, State tstate, State astate) throws PrismException
	{
		if (shared_state_names == null)
			return getJointStateNoSS(states, accStates, tstate, astate);
		else
			return getJointStateWithSS(states, accStates, tstate, astate);
	}

	State getJointStateWithSS(ArrayList<State> states, BitSet accStates, State expectedTState, State expectedAState) throws PrismException
	{

		int state_size = num_robots + 2; //all the variables in the shared states are just in one state 

		State temp_joint_state = new State(state_size);
		BitSet trueLabels = new BitSet();
		ArrayList<State> brokenRobotStates = new ArrayList<State>();
		for (int i = 0; i < num_robots; i++) {

			State current_model_state = states.get(i);
			State brokenRobotState = breakRobotState(current_model_state, i);
			brokenRobotStates.add(brokenRobotState);
			// collect non da values
			// knowing that the da value is the last one
			// TODO: shared state stuff
			//			int da_state_index = prodModGens.get(i).getVarIndex(da_id_string);
			prodModGens.get(i).exploreState(current_model_state);
			ProductModelGenerator pmg = prodModGens.get(i);

			for (int labnum = 0; labnum < pmg.getNumLabelExprs(); labnum++) {
				if (prodModGens.get(i).isExprTrue(labnum))
					trueLabels.set(labnum);
			}
		}
		//now we have all the labels and the broken robot states 
		for (int i = 0; i < brokenRobotStates.size(); i++) {

			//we get the first state from the broken state, thats the current model state 
			State brokenRobotState = brokenRobotStates.get(i);
			State ps = (State) brokenRobotState.varValues[0];
			State ss = (State) brokenRobotState.varValues[1];
			State ds = (State) brokenRobotState.varValues[2];

			temp_joint_state.setValue(i, ps);
			//for the joint state we have 

			temp_joint_state = processForDAState(temp_joint_state, ds, accStates);
			temp_joint_state = processForSSState(temp_joint_state, ss);

		}
		//we may need to update the temp_joint_state 
		//from the temp joint state get the da state 
		int current_da_state = getIndividualState(temp_joint_state, num_robots + 1, 0);
		//now lets do the checking 

		int updated_da_state = this.da.getEdgeDestByLabel(current_da_state, trueLabels);
		if (current_da_state != updated_da_state) {
			State newda = new State(1);
			newda.setValue(0, updated_da_state);
			//then we need to update it 
			temp_joint_state.setValue(num_robots + 1, newda);

		}

		//just adding a 0 
		if (TSTATEINDEX != -1) {
			State tstate = expectedTState;
			if (tstate == null) {
				tstate = new State(num_robots);
				for (int i = 0; i < num_robots; i++)
					tstate.setValue(i, 0.0);

			}
			temp_joint_state.setValue(TSTATEINDEX, tstate);
		}

		if (ACTSTATEINDEX != -1) {
			State astate = expectedAState;
			if (astate == null) {
				astate = new State(num_robots);
				for (int i = 0; i < num_robots; i++)
					astate.setValue(i, "");
			}
		}

//		if (!jointStateRobotStateMapping.containsKey(temp_joint_state)) {
//			ArrayList<State> cp = new ArrayList<State>();
//			cp.addAll(states);
//			jointStateRobotStateMapping.put(temp_joint_state, cp);
//		}
		return temp_joint_state;
	}

	private State processForSSState(State temp_joint_state, State ss)
	{
		if (referenceSSState == null)
			referenceSSState = new State(ss); //working on the assumption that in the beginning everyone has the same state

		State ss_state_in_joint_state = temp_joint_state.substate(num_robots, num_robots + 1);
		if (ss_state_in_joint_state.varValues[0] == null) {
			temp_joint_state.setValue(num_robots, ss);
		} else {
			//check for updates 
			//create a new state if need be 
			//FIXME: Assumption - there is no conflict in these shared states 
			//so basically we check if the state in the joint state doesnt match the current state 
			//and it doesnt match the reference state, we update 
			Object[] ssv = ss.varValues;
			Object[] ssjv = getIndividualState(ss_state_in_joint_state.varValues, 0);
			Object[] ssrefv = referenceSSState.varValues;
			boolean updateSS = false;
			for (int i = 0; i < ssv.length; i++) {
				if ((int) ssv[i] != (int) ssjv[i]) {

					//if they are not the same then 
					//check if ssv is the same as the reference 
					if ((int) ssv[i] == (int) ssrefv[i]) {
						//if it is then set the value of ss[i] to that of ssjv[i] 
						ss.setValue(i, ssjv[i]);
					} else {
						//if they are not the same 
						//then this ss is new information 
						//so we need to update the ss 
						if (!updateSS)
							updateSS = true;
					}
				}

			}
			if (updateSS) {
				temp_joint_state.setValue(num_robots, ss);
			}

		}

		// TODO Auto-generated method stub
		return temp_joint_state;
	}

	State processForDAState(State temp_joint_state, State ds, BitSet accStates)
	{
		State da_state_in_joint_state = temp_joint_state.substate(num_robots + 1, num_robots + 2);
		if (da_state_in_joint_state.varValues[0] == null) {
			temp_joint_state.setValue(num_robots + 1, ds);
		} else {
			// check if there's a change
			State da_state_in_robot = ds;
			State dastateinjointstate = (State) da_state_in_joint_state.varValues[0];

			if (!dastateinjointstate.equals(da_state_in_robot)) {
				// if there are competing DA states
				// lets take the one whose disttoacc is the smallest ?

				int da_state_in_joint_state_int = (int) dastateinjointstate.varValues[0];
				int dastateinrobot = (int) da_state_in_robot.varValues[0];
				if (!(dastateinrobot == da_state_in_joint_state_int)) {
					double jsacc = da_distToAcc.get(da_state_in_joint_state_int);
					double rsacc = da_distToAcc.get(dastateinrobot);

					if (accStates != null) {
						if (accStates.get(dastateinrobot) || accStates.get(da_state_in_joint_state_int)) {
						}
					}
					if (jsacc >= rsacc) {
						temp_joint_state.setValue(num_robots + 1, da_state_in_robot);
					}
				}

			}
		}
		return temp_joint_state;

	}

	State getJointStateNoSS(ArrayList<State> states, BitSet accStates, State expectedTState, State expectedAState) throws PrismException
	{

		int state_size = num_robots + 1 + 1 + 1; //da, time
		State temp_joint_state = new State(state_size);
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

			State da_state_in_joint_state = temp_joint_state.substate(this.DASTATEINDEX, this.DASTATEINDEX + 1);

			if (da_state_in_joint_state.varValues[0] == null) {
				temp_joint_state.setValue(this.DASTATEINDEX, current_model_state.substate(da_state_index, da_state_index + 1));
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
							}
						}
						if (jsacc >= rsacc) {
							temp_joint_state.setValue(this.DASTATEINDEX, da_state_in_robot);
						}
					}

				}
			}

		}
		//we may need to update the temp_joint_state 
		//from the temp joint state get the da state 
		int current_da_state = getIndividualState(temp_joint_state, this.DASTATEINDEX, 0);
		//now lets do the checking 

		int updated_da_state = this.da.getEdgeDestByLabel(current_da_state, trueLabels);
		if (current_da_state != updated_da_state) {
			State newda = new State(1);
			newda.setValue(0, updated_da_state);
			//then we need to update it 
			temp_joint_state.setValue(this.DASTATEINDEX, newda);

		}
		//just adding a 0 
		if (TSTATEINDEX != -1) {
			State tstate = expectedTState;
			if (tstate == null) {
				tstate = new State(num_robots);
				for (int i = 0; i < num_robots; i++)
					tstate.setValue(i, 0.0);

			}
			temp_joint_state.setValue(TSTATEINDEX, tstate);
		}
		if (ACTSTATEINDEX != -1) {
			State astate = expectedAState;
			if (astate == null) {
				astate = new State(num_robots);
				for (int i = 0; i < num_robots; i++)
					astate.setValue(i, "");
			}
			temp_joint_state.setValue(ACTSTATEINDEX, astate);
		}

//		if (!jointStateRobotStateMapping.containsKey(temp_joint_state)) {
//			ArrayList<State> cp = new ArrayList<State>();
//			cp.addAll(states);
//			jointStateRobotStateMapping.put(temp_joint_state, cp);
//		}
		return temp_joint_state;
	}

	//	Entry<Double, Double> getStateCost(State state)
	//	{
	//
	//		//		State dastate = state.substate(num_robots, num_robots + 1);
	//		//		dastate = (State) dastate.varValues[0];
	//		//		int dastateint = (int) dastate.varValues[0];
	//		int dastateint = getDAState(state);
	//		double dist = da_distToAcc.get(dastateint);
	//		double prob = 0;
	//		if (dist == 0) {
	//			mainLog.println(state.toString());
	//			if (doProb) {
	//				prob = 1;
	//			}
	//		}
	//
	//		return new AbstractMap.SimpleEntry<Double, Double>(prob, dist);
	//	}

	double getRobotsReward(State state, Object act, boolean sumCost) throws PrismException
	{
		ArrayList<State> robotStates = getRobotStatesFromJointState(state);
		ArrayList<Object> robotActions = splitJointAction(act);
		// get the da state
		double cost = 0;
		if (state.toString().contains("-1),(-1"))
			cost = 0;
		if (!isDeadend(state) && !brtdpPolicy.isGoal(state)) {
			if (sumCost) {
				for (int i = 0; i < robotStates.size(); i++)
					cost += getRobotReward(i, robotStates.get(i), robotActions.get(i));
			} else {
				//max cost 
				double currentCost = getRobotReward(0, robotStates.get(0), robotActions.get(0));
				for (int i = 1; i < robotStates.size(); i++) {
					cost = getRobotReward(i, robotStates.get(i), robotActions.get(i));
					if (currentCost < cost)
						currentCost = cost;
				}
				cost = currentCost;
			}
		}
		return cost;
	}

	double getRobotReward(int rnum, State state, Object act) throws PrismException
	{
		//assuming there is just a single reward 
		ProductModelGenerator pmg = prodModGens.get(rnum);
		double rew = pmg.getStateActionReward(0, state, act);
		return rew;

	}

	int getDAState(State state)
	{
		if (shared_state_names == null)
			return getIndividualState(state, num_robots, 0);
		else
			return getIndividualState(state, num_robots + 1, 0);
	}

	boolean isGoal(State state)
	{
		// get the da state

		int dastateint = getDAState(state);
		double dist = da_distToAcc.get(dastateint);
		if (dist == 0) {
			mainLog.println(state.toString());
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

	HashMap<Integer, HashMap<Object, Double>> getRobotActionCosts(ArrayList<State> states, HashMap<Integer, HashMap<Object, Integer>> actions)
			throws PrismException
	{
		HashMap<Integer, HashMap<Object, Double>> actioncosts = new HashMap<Integer, HashMap<Object, Double>>();
		//for each state, for each action 
		for (int i = 0; i < states.size(); i++) {
			ProductModelGenerator prodModGen = prodModGens.get(i);
			HashMap<Object, Integer> robotActions = actions.get(i);
			State robotState = states.get(i);
			prodModGen.exploreState(robotState);
			HashMap<Object, Double> robotActionCosts = new HashMap<Object, Double>();
			for (Object act : robotActions.keySet()) {
				double rew = prodModGen.getStateActionReward(0, robotState, act);
				robotActionCosts.put(act, rew);
			}
			actioncosts.put(i, robotActionCosts);

		}
		return actioncosts;
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

	/*
	 * The real money is here sorry 
	 * so this is where the magic needs to happen 
	 * when we're creating combinations, we need the action durations 
	 * which are costs 
	 * and then we're not going to touch the generate combinations function 
	 * but we'll touch the other stuff 
	 */
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
			brtdpPolicy.generateCombinations(numActionsPerRobot, numActionsPerRobot.clone(), res);
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
				//so this is the magic really 
				Object act = robotKeySet.get(r[i] - 1);
				action.add(act);

			}

			jointActions.add(action);
		}
		return jointActions;
	}

	ArrayList<Object> getJointActionsTStates(ArrayList<ArrayList<Object>> jointActions, HashMap<Integer, HashMap<Object, Double>> actionCosts,
			State pTState, State pAState) throws PrismException
	{
		ArrayList<Object> toret = new ArrayList<Object>();
		ArrayList<State> tstates = new ArrayList<State>();
		ArrayList<State> astates = new ArrayList<State>();
		ArrayList<Object> actionsToSkip = new ArrayList<Object>();
		ArrayList<ArrayList<Object>> updatedActions = new ArrayList<ArrayList<Object>>();
		boolean skipAction = false;

		for (ArrayList<Object> ja : jointActions) {
			skipAction = false;
			//for each joint action 
			double[] actionCostsHere = new double[ja.size()];
			double smallestCost = -1;
			for (int i = 0; i < ja.size(); i++) {
				Object pAct = pAState.varValues[i];
				Object jaAct = ja.get(i);
				if (pAct.toString() != "") {
					if (!pAct.toString().contains(jaAct.toString())) {
						actionsToSkip.add(ja);
						skipAction = true;
					}
				}
				Object act = ja.get(i);
				if (actionCosts != null) {
					double cost = actionCosts.get(i).get(act);
					actionCostsHere[i] = cost;
					if (smallestCost == -1) {
						smallestCost = cost;
					} else {
						if (cost < smallestCost) {
							smallestCost = cost;
						}
					}
				}
			}

			//			if (actionCosts != null) {
			//				//create the action's time state here 
			State tstate = new State(num_robots);
			State aState = new State(num_robots);

			for (int i = 0; i < actionCostsHere.length; i++) {
				//updating smallest cost 
				double t = (double) pTState.varValues[i];
				if (actionCostsHere[i] - (t + smallestCost) < 0)
					smallestCost = (t + smallestCost) - actionCostsHere[i];
			}
			if (smallestCost == 0)
				mainLog.println("time step = 0!! ");

			for (int i = 0; i < actionCostsHere.length; i++) {
				double t = (double) pTState.varValues[i];
				double tn = t + smallestCost;
				double ac = 0.0;
				if (tn == actionCostsHere[i]) {
					ac = 0.0;

					aState.setValue(i, "");

				} else {
					if (tn > actionCostsHere[i]) {
						if (!skipAction) {
							throw new PrismException(tn + ">" + actionCostsHere[i] + ":" + ja.toString() + " " + pTState.toString() + "," + pAState.toString());
						}
					} else {
						ac = t + smallestCost;
						aState.setValue(i, ja.get(i));

					}
				}
				//				double ac = actionCostsHere[i] - (smallestCost + t);

				//				if(ac == 0)
				//					ac = 0.0; 
				if (ac < 0) {
					//					mainLog.println
					throw new PrismException("time less than 0" + ac);
				}

				//				if (ac == 0)
				tstate.setValue(i, ac);
				//				else
				//				{
				//					
				////					tstate.setValue(i, 1);
				//				}
			}
			if(!skipAction) {
			tstates.add(tstate);
			astates.add(aState);
			updatedActions.add(ja);
			}
			//		}

		}

		toret.add(tstates);
		toret.add(astates);
		toret.add(updatedActions);
		if (actionsToSkip.size() > 0) {
			
			mainLog.println(actionsToSkip);
			
		}
		return toret;

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

	ArrayList<Double> calculateJointStatesProbs(HashMap<Integer, HashMap<State, Double>> succStates, ArrayList<ArrayList<State>> combinations)
			throws PrismException
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
			brtdpPolicy.generateCombinations(numStatesPerRobot, numStatesPerRobot.clone(), combinations);

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

	//	public State simulateAction(State state, Object action) throws PrismException
	//	{
	//
	//		return simulateAction(state, action, false);
	//	}

	//	public State simulateAction(State state, Object action, boolean initState) throws PrismException
	//	{
	//		State succState = null;
	//
	//		if (action != null) {
	//			succState = uctPolicy.simulateAction(state, action);
	//			if (succState == null) { // we're simply going to do a rollout we dont care about anything else
	//				if (uctPolicy.robotStateActionIndices.containsKey(state)) {
	//					if (uctPolicy.robotStateActionIndices.get(state).containsKey(action)) {
	//						HashMap<Integer, HashMap<State, Double>> succStates = new HashMap<Integer, HashMap<State, Double>>();
	//						// okay so nowwwwwww
	//						// we get the action index for each robot
	//						ArrayList<Integer> actionIndices = uctPolicy.robotStateActionIndices.get(state).get(action);
	//						// and we can get like the thing from the stuff we have already
	//						ArrayList<State> robotStates = getRobotStatesFromJointState(state);
	//						for (int i = 0; i < num_robots; i++) {
	//							HashMap<State, Double> ahashmap = new HashMap<State, Double>();
	//							int choice = actionIndices.get(i);
	//							ProductModelGenerator pmg = prodModGens.get(i);
	//							pmg.exploreState(robotStates.get(i));
	//							int nt = pmg.getNumTransitions(choice);
	//							for (int t = 0; t < nt; t++) {
	//								double prob = pmg.getTransitionProbability(choice, t);
	//								State succState1 = pmg.computeTransitionTarget(choice, t);
	//								ahashmap.put(succState1, prob);
	//
	//							}
	//							succStates.put(i, ahashmap);
	//
	//						}
	//
	//						ArrayList<ArrayList<State>> jointSuccessors = getJointStateCombinations(succStates);
	//						ArrayList<State> jointsuccStates = getJointStates(jointSuccessors);
	//						ArrayList<Double> probs = calculateJointStatesProbs(succStates, jointSuccessors);
	//						// now we just choose one of these
	//						// if its the first one we should add these to uctPolicy
	//						if (initState) {
	//							uctPolicy.addAction(state, action, jointsuccStates, probs);
	//							//the joint succStates are leaf states now 
	//							uctPolicy.setAsLeafState(jointsuccStates);
	//						}
	//						double rand = (new Random()).nextDouble();
	//						double cprob = 0;
	//						for (int i = 0; i < probs.size(); i++) {
	//							cprob += probs.get(i);
	//							if (cprob >= rand) {
	//								succState = jointsuccStates.get(i);
	//								break;
	//							}
	//						}
	//					} else {
	//						throw new PrismException("Simulation: Action not in robot state action indices for state");
	//						//					return null;
	//					}
	//				} else {
	//					throw new PrismException("Simulation: State not in robot state action indices");
	//					//				return null;
	//				}
	//
	//			}
	//		} else
	//			throw new PrismException("Passed null action to simulate action");
	//
	//		return succState;
	//	}

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

	// when the varvalues result in a state
	private static Object[] getIndividualState(Object[] vv, int ind)
	{
		Object[] fvv;
		Object s = vv;
		if (ind != -1)
			s = vv[ind];
		if (s instanceof State) {
			fvv = getIndividualState(((State) s).varValues, -1);
		} else {
			if (vv instanceof Object[])
				fvv = vv;
			else
				fvv = (Object[]) s;
		}
		return fvv;
	}

	static private ArrayList<State> getRobotStatesFromJointStateNoSS(State state)
	{
		// TODO edit this for multiple state variables but for now lets just go with the
		// usual
		int numRobotStateVars = -1;
		int da_state_val = getIndividualState(state, num_robots, 0);
		int robot_state_val;
		ArrayList<State> robotStates = new ArrayList<State>();
		State robotState;
		for (int i = 0; i < num_robots; i++) {
			numRobotStateVars = prodModGens.get(i).getNumVars();
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

	static private State getTStateFromJointState(State state)
	{
		return (State) state.varValues[TSTATEINDEX];
	}

	static private State getAStateFromJointState(State state)
	{
		return (State) state.varValues[ACTSTATEINDEX];
	}

	//	public Object monteCarloPlanning(BitSet accStates, boolean minCost) throws Exception
	//	{
	//		//		ArrayList<Integer> accsFound = new ArrayList<Integer>();
	//
	//		if (doProb)
	//			minCost = false;
	//
	//		current_rollout_num = 0;
	//		ArrayList<State> initialStates = getInitialStates();
	//		// get the initial state
	//		State temp_joint_state = getJointState(initialStates, null);
	//		mainLog.println("Joint State:" + temp_joint_state.toString());
	//		Object res = null;
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
	//		String saveplace = MRmcts.TESTSLOC;
	//		// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
	//		String filename = "no_door_example";
	//		uctPolicy.jointMDP.exportToDotFile(saveplace + filename + "_mctsmdp.dot");
	//		MDPSimple policyTree = uctPolicy.extractPolicyTreeAsDotFile(uctPolicy.jointMDP, uctPolicy.getStateIndex(temp_joint_state), minCost);
	//		policyTree.exportToDotFile(saveplace + filename + "_policy.dot");
	//
	//		//		return accsFound;
	//		return res;
	//	}

	//	public Object monteCarloPlanning(State state, boolean minCost) throws Exception
	//	{
	//		Object bestAction = null;
	//		int current_depth = 0;
	//		boolean addToTreeInTrial = false;
	//		accFound = false;
	//		//		while (current_depth < rollout_depth) {	
	//		search(null, state, current_depth, minCost, addToTreeInTrial);
	//		if (accFound)
	//			mainLog.println("Acc Found");
	//		//		}
	//		//		bestAction = uctPolicy.selectBestAction(state, current_depth);
	//		return bestAction;
	//	}

	//	public double search(State ps, State state, int depth, boolean minCost, boolean addToTreeInTrial) throws Exception
	//	{
	//		String searchInfo = "";
	//		if (ps != null)
	//			searchInfo += ps.toString() + "->";
	//		searchInfo += state.toString();
	//
	//		double cost_state = 0;
	//		if (isGoal(state)) {
	//			uctPolicy.setAsLeafState(state);
	//			Entry<Double, Double> probstatecost = getStateCost(state);
	//			double probCost = probstatecost.getKey();
	//			double stateCost = probstatecost.getValue();
	//			cost_state = stateCost;
	//			if (doProb)
	//				cost_state = probCost;
	//			searchInfo += "=" + cost_state + "-ACC";
	//			mainLog.println(searchInfo);
	//			return cost_state;
	//		}
	//		if (isTerminal(ps, state)) {
	//			//remove from leaf states 
	//			//because we're saying terminal = self loop
	//			uctPolicy.removeFromLeafState(state);
	//			// dont want deadends
	//			double failcost = MINFAILCOST;
	//			if (minCost)
	//				failcost = MAXFAILCOST;
	//			if (doProb)
	//				failcost = 0;
	//			Entry<Double, Double> probstatecost = getStateCost(state);
	//			double probCost = probstatecost.getKey();
	//			double stateCost = probstatecost.getValue();
	//			cost_state = stateCost;
	//			if (doProb)
	//				cost_state = probCost;
	//			cost_state += failcost;
	//			searchInfo += "=" + cost_state + "-terminal";
	//			mainLog.println(searchInfo);
	//			return cost_state;
	//		}
	//		if (depth == rollout_depth) {
	//
	//			uctPolicy.setAsLeafState(state);
	//			Entry<Double, Double> probstatecost = getStateCost(state);
	//			double probCost = probstatecost.getKey();
	//			double stateCost = probstatecost.getValue();
	//			cost_state = stateCost;
	//			if (doProb)
	//				cost_state = probCost;
	//			searchInfo += "=" + cost_state + "-maxd";
	//			mainLog.println(searchInfo);
	//			return cost_state;
	//		}
	//		if (isLeaf(state)) {
	//			//we're expanding this 
	//			//so unset it 
	//			uctPolicy.removeFromLeafState(state);
	//			//			if (!uctPolicy.stateIndices.containsKey(state))
	//			initialiseVisits(state, minCost);
	//			ArrayList<Entry<State, Object>> trialList = new ArrayList<Entry<State, Object>>();
	//			cost_state = evaluate(ps, state, depth, true, trialList, minCost, addToTreeInTrial);
	//			// now lets update this state with the reward
	//			searchInfo += "=" + cost_state + "-rollout";
	//			mainLog.println(searchInfo);
	//			mainLog.println("Trial:\n" + trialList.toString());
	//			return cost_state;
	//		}
	//		uctPolicy.removeFromLeafState(state);
	//		Object action = selectAction(state, depth, false, minCost);
	//		State succState = simulateAction(state, action, true);
	//		Entry<Double, Double> probstatecost = getStateCost(state);
	//		double probCost = probstatecost.getKey();
	//		double stateCost = probstatecost.getValue();
	//		double reward = stateCost;
	//		if (doProb)
	//			reward = probCost;
	//		double q = reward + search(state, succState, depth + 1, minCost, addToTreeInTrial);
	//		updateValue(state, action, q, depth, minCost);
	//		cost_state = q;
	//		searchInfo += "=" + cost_state + "-selection";
	//		mainLog.println(searchInfo);
	//		return cost_state;
	//
	//	}

	//	void initialiseVisits(State state, boolean minCost) throws PrismException
	//	{
	//		if (!uctPolicy.stateIndices.containsKey(state))
	//			uctPolicy.addState(state);
	//		boolean mayhaveaction = false;
	//		boolean initaction = true;
	//		// use uctPolicy
	//		ArrayList<State> currentStates = getRobotStatesFromJointState(state);
	//		HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
	//		ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);
	//		if (uctPolicy.stateActionVisits.containsKey(state))
	//			mayhaveaction = true;
	//		mainLog.println(state.toString() + "- A=" + allPossibleJointActions.size());
	//		boolean noinitneeded = false;
	//		if (mayhaveaction) {
	//			noinitneeded = (allPossibleJointActions.size() == uctPolicy.stateActionVisits.get(state).keySet().size());
	//		}
	//		if (!noinitneeded) {
	//			for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
	//				Object action = getJointActionName(allPossibleJointActions.get(actionNum));
	//				if (mayhaveaction) {
	//					if (uctPolicy.stateActionVisits.get(state).containsKey(action))
	//						initaction = false;
	//				}
	//				if (initaction) {
	//					mainLog.println(action.toString() + " added");
	//					//			if (!uctPolicy.stateVisited(state))
	//					uctPolicy.initialiseVisits(state, action);
	//					uctPolicy.addRobotStateActionIndices(state, action, allPossibleJointActions.get(actionNum), allPossibleActionsForState);
	//				}
	//				//			else
	//				//				uctPolicy.increaseVisits(state, action);
	//				//			}
	//			}
	//		}
	//
	//		uctPolicy.printStateDetails(state, minCost);
	//	}

	//	private void updateValue(State state, Object action, double q, int depth, boolean minCost)
	//	{
	//		// update n(s,a)
	//		// q(s,a) = q(a,s) + [q-q(a,s)]/n(s,a)
	//		uctPolicy.increaseVisits(state, action);
	//		int numVisits = uctPolicy.getNumVisits(state, action);
	//
	//		double qudpate = uctPolicy.getQvalue(state, action, minCost);
	//		if (numVisits == 0) {
	//			if (minCost)
	//				qudpate = Double.MIN_VALUE;
	//			else
	//				qudpate = Double.MAX_VALUE;
	//		} else {
	//			double temp = (q - qudpate);
	//			temp = temp / (double) uctPolicy.getNumVisits(state, action);
	//			qudpate += temp;
	//		}
	//		uctPolicy.updateQValue(state, action, qudpate);
	//
	//	}

	//	private boolean isGoal (State state)
	//	{
	//		if (accFound)
	//			return true;
	//		else
	//			return false;
	//	}
	//	private boolean isTerminal(State pstate, State state)
	//	{
	//
	//		if (pstate != null) {
	//			if (state.equals(pstate))
	//				return true;
	//		}
	//
	//		return false;
	//	}

	//	private boolean isLeaf(State state)
	//	{
	//		// its a leaf if
	//		// it is not in the tree
	//		// so basically the state hasnt been added
	//		// which means it should not exist in the
	//		// uctPolicy stateIndices
	//		if (uctPolicy.toExpand(state, true))
	//			return true;
	//		else
	//
	//			return false;

	//	}

	//	private double evaluate(State pstate, State state, int depth, boolean firstState, ArrayList<Entry<State, Object>> trial, boolean minCost,
	//			boolean addToTreeInTrial) throws Exception
	//	{
	//		double sumrew = 0;
	//		State succState = null;
	//		if (depth == rollout_depth) {
	//			Entry<Double, Double> probstatecost = getStateCost(state);
	//			double probCost = probstatecost.getKey();
	//			double stateCost = probstatecost.getValue();
	//			if (doProb)
	//				return probCost;
	//
	//			return stateCost;
	//		}
	//		if (isGoal(state)) {
	//			Entry<Double, Double> probstatecost = getStateCost(state);
	//			double probCost = probstatecost.getKey();
	//			double stateCost = probstatecost.getValue();
	//			if (doProb)
	//				return probCost;
	//
	//			return stateCost;
	//		}
	//		if (isTerminal(pstate, state)) {
	//			double failcost = MINFAILCOST;
	//			if (minCost)
	//				failcost = MAXFAILCOST;
	//			if (doProb)
	//				failcost = 0;
	//			Entry<Double, Double> probstatecost = getStateCost(state);
	//			double probCost = probstatecost.getKey();
	//			double stateCost = probstatecost.getValue();
	//			if (doProb)
	//				return probCost + failcost;
	//
	//			return stateCost + failcost;
	//		}
	//		// do a whole run
	//		try {
	//			Entry<Double, Double> probstatecost = getStateCost(state);
	//			double probCost = probstatecost.getKey();
	//			double stateCost = probstatecost.getValue();
	//
	//			double reward = stateCost;
	//			if (doProb)
	//				reward = probCost;
	//			Object action = selectAction(state, depth, true, minCost);
	//			trial.add(new AbstractMap.SimpleEntry<State, Object>(state, action));
	//			succState = simulateAction(state, action, (firstState || addToTreeInTrial));
	//
	//			sumrew = reward + evaluate(state, succState, depth + 1, false, trial, minCost, addToTreeInTrial);
	//			if (firstState || addToTreeInTrial) {
	//				updateValue(state, action, sumrew, depth, minCost);
	//			}
	//
	//		} catch (Exception e) {
	//			mainLog.println("Error");
	//			e.printStackTrace();
	//
	//			mainLog.println(trial.toString());
	//			if (succState != null)
	//				mainLog.println(succState.toString());
	//			throw new Exception(e);
	//
	//		}
	//		return sumrew;
	//
	//	}

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

	//	Object getActionUsingIndividualRobotPolicies(State state) throws PrismException
	//	{
	//		ArrayList<State> robotStates = getRobotStatesFromJointState(state);
	//		ArrayList<Object> robotActions = new ArrayList<Object>();
	//		int[] actionIndices = new int[num_robots];
	//		for (int i = 0; i < robotStates.size(); i++) {
	//			MDStrategy strat = this.defaultStrategies.get(i);
	//			// we need the mapping
	//			int index = findRobotStateIndex(robotStates.get(i), i);
	//			if (index != -1) {
	//				// strat.initialise(index);
	//				if (strat.isChoiceDefined(index)) {
	//					int actionIndex = strat.getChoiceIndex(index);
	//
	//					Object action = strat.getChoiceAction(index);
	//					if (action.toString().contains("*"))
	//						throw new PrismException("* action for this state");
	//					//if this action is * 
	//					//we have to choose something else 
	//					//basically get an action from the robots model 
	//					//for this state 
	//					//and then the label too 
	//
	//					robotActions.add(action);
	//					actionIndices[i] = actionIndex;
	//				} else {
	//					//get a random action 
	//					ProductModelGenerator prodModGen = this.prodModGens.get(i);
	//					prodModGen.exploreState(robotStates.get(i));
	//					int numActions = prodModGen.getNumChoices();
	//					int action_choice = (new Random()).nextInt(numActions);
	//
	//					Object action = prodModGen.getChoiceAction(action_choice);
	//					robotActions.add(action);
	//					actionIndices[i] = action_choice;
	//					//					throw new PrismException("Undefined choice"); 
	//				}
	//
	//			} else {
	//				//basically this state is not possible in the single robot model 
	//				//but possible in the joint robot model 
	//				//so we have to do the random action thing again 
	//				//talk about this 
	//				//get a random action 
	//				ProductModelGenerator prodModGen = this.prodModGens.get(i);
	//				prodModGen.exploreState(robotStates.get(i));
	//				int numActions = prodModGen.getNumChoices();
	//				int action_choice = (new Random()).nextInt(numActions);
	//
	//				Object action = prodModGen.getChoiceAction(action_choice);
	//				robotActions.add(action);
	//				actionIndices[i] = action_choice;
	//				//				throw new PrismException("Can not select action according to base strategy - Robot State -1");
	//			}
	//			// strat.initialise(s);
	//
	//		}
	//
	//		Object joint_action = this.getJointActionName(robotActions);
	//		uctPolicy.addRobotStateActionIndices(state, joint_action, robotActions, actionIndices);
	//		return joint_action;
	//
	//	}

	//	private Object chooseRandomAction(State state, boolean minCost) throws PrismException
	//	{
	//		int numActions = uctPolicy.stateActionVisits.get(state).keySet().size();
	//		// if this isn't the first visit then maybe we can do the exploration
	//		// expolitation thing
	//		// how do we know that this isnt the first action
	//		// the total visits will be 0
	//		if (uctPolicy.getNumVisits(state) == 0) {
	//			int chosen_action = (new Random()).nextInt(numActions);
	//			Object action = uctPolicy.stateActionVisits.get(state).keySet().toArray()[chosen_action];
	//
	//			return action;
	//		} else {
	//			Object action = uctPolicy.getBestAction(state, minCost);
	//			return action;
	//		}
	//
	//	}

	//	private Object chooseRandomActionNotVisited(State state) throws PrismException
	//	{
	//		// action not added to uctPolicy
	//		ArrayList<State> currentStates = getRobotStatesFromJointState(state);
	//		// get all actions
	//		ArrayList<Integer> numActionsPerRobot;// = getNumActionsPerRobot(currentStates);
	//		// get the number of combinations
	//		ArrayList<int[]> res = new ArrayList<int[]>();
	//
	//		numActionsPerRobot = getNumActionsPerRobot(currentStates);
	//		uctPolicy.generateCombinations(numActionsPerRobot, res);
	//		// now choose a combination at random
	//		// since we are not following a policy
	//		int chosen_action_num = (new Random()).nextInt(res.size());
	//		// now we get that combination
	//		int[] chosen_action_comb = res.get(chosen_action_num);
	//		// fix chosen_action_comb
	//		for (int i = 0; i < chosen_action_comb.length; i++)
	//			chosen_action_comb[i]--;
	//		// now we create an action from that
	//		ArrayList<Object> actionNames = getActionNamesFromRobotActionNums(chosen_action_comb);
	//		// return the joint action
	//		// also add these indices
	//		Object joint_action = getJointActionName(actionNames);
	//		uctPolicy.addRobotStateActionIndices(state, joint_action, actionNames, chosen_action_comb);
	//
	//		return joint_action;
	//	}

	//	private Object selectAction(State state, int depth, boolean useRolloutPolicy, boolean minCost) throws PrismException
	//	{
	//		boolean useStrategy = true;
	//		Object joint_action;
	//		if (useRolloutPolicy) {
	//
	//			// use single robot policy
	//			if (useStrategy) {
	//				// dostuff here
	//				// convert this state into the state for each robot
	//				// get each robot's action
	//				joint_action = this.getActionUsingIndividualRobotPolicies(state);
	//				// check here to see if it contains a *
	//				return joint_action;
	//
	//			} else {
	//				// choose an action randomly
	//				if (uctPolicy.stateActionVisits.containsKey(state)) {
	//					joint_action = chooseRandomAction(state, minCost);
	//					return joint_action;
	//
	//				} else {
	//					joint_action = chooseRandomActionNotVisited(state);
	//					return joint_action;
	//				}
	//			}
	//		} else {
	//			return uctPolicy.getBestAction(state, minCost);
	//		}
	//
	//	}

	void addStateAction(State state, Object action, boolean sumCosts) throws PrismException
	{
		ArrayList<State> currentStates = getRobotStatesFromJointState(state);
		State tstate = getTStateFromJointState(state);
		State astate = getAStateFromJointState(state);
		HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
		HashMap<Integer, HashMap<Object, Double>> allPossibleActionsForStateCosts = getRobotActionCosts(currentStates, allPossibleActionsForState);
		ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);
		ArrayList<Object> atstates = 
				getJointActionsTStates(allPossibleJointActions, allPossibleActionsForStateCosts, tstate, astate);
		ArrayList<State> tstates = (ArrayList<State>)atstates.get(0);
		ArrayList<State> astates = (ArrayList<State>)atstates.get(1);
		ArrayList<ArrayList<Object>> updatedJointActions = (ArrayList<ArrayList<Object>>)atstates.get(2);
		
		allPossibleJointActions  = updatedJointActions;
		
		//hmmmm 
		//print all these tstates here 
		mainLog.println(tstates);
		for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
			Object act = getJointActionName(allPossibleJointActions.get(actionNum));

			if (act.toString().contains(action.toString())) {
				State tstateAction = tstates.get(actionNum);
				State astateAction = astates.get(actionNum);
				//add these robot action indices to our list 
				boolean added = brtdpPolicy.addRobotStateActionIndices(state, action, allPossibleJointActions.get(actionNum),
						allPossibleActionsForState,
						tstateAction);
				//so we have to add this action 
				if (added) {

					addStateActionToMDP(state, action, sumCosts, tstateAction, astateAction);
				}
			}

		}
	}

	/**
	 * Basically this is the money 
	 * @param state
	 * @param sumCosts
	 * @throws PrismException
	 */
	void addStateActions(State state, boolean sumCosts) throws PrismException
	{
		if (state.toString().contains("(0),(4),(2),(0.0,1.0),(,v4_6)"))
			mainLog.println("Pause here");
		ArrayList<State> currentStates = getRobotStatesFromJointState(state);

		State currentTState = getTStateFromJointState(state);
		State currentAState = getAStateFromJointState(state);
		HashMap<Integer, HashMap<Object, Integer>> allPossibleActionsForState = getActions(currentStates);
		HashMap<Integer, HashMap<Object, Double>> allPossibleActionsForStateCosts = getRobotActionCosts(currentStates, allPossibleActionsForState);
		//TODO: come back here!!! 
		ArrayList<ArrayList<Object>> allPossibleJointActions = getJointActions(allPossibleActionsForState);

//		ArrayList<ArrayList<State>> atstates =
//				getJointActionsTStates(allPossibleJointActions, allPossibleActionsForStateCosts, currentTState, currentAState);
//		//clean up the joint actions and the a and t states 
//
//		ArrayList<State> tstates = atstates.get(0);
//		ArrayList<State> astates = atstates.get(1);
//		
		ArrayList<Object> atstates = 
				getJointActionsTStates(allPossibleJointActions, allPossibleActionsForStateCosts, currentTState, currentAState);
		ArrayList<State> tstates = (ArrayList<State>)atstates.get(0);
		ArrayList<State> astates = (ArrayList<State>)atstates.get(1);
		ArrayList<ArrayList<Object>> updatedJointActions = (ArrayList<ArrayList<Object>>)atstates.get(2);
		
		allPossibleJointActions  = updatedJointActions;
		
		//hmmmm 
		//print all these tstates here 
		mainLog.println(tstates);

		for (int actionNum = 0; actionNum < allPossibleJointActions.size(); actionNum++) {
			Object action = getJointActionName(allPossibleJointActions.get(actionNum));
			State tstate = tstates.get(actionNum);
			State astate = astates.get(actionNum);
			//add these robot action indices to our list 
			boolean added = brtdpPolicy.addRobotStateActionIndices(state, action, allPossibleJointActions.get(actionNum), allPossibleActionsForState, tstate);
			//so we have to add this action 
			if (added) {

				addStateActionToMDP(state, action, sumCosts, tstate, astate);

			}

		}
	}

	private double updateUpperValue(State state, boolean sumCosts, boolean printVal) throws PrismException
	{
		if (printVal)
			mainLog.println("Updating Upper Values");
		//step 1 
		//get all the actions 
		//check if we dont have actions stored already 
		if (!brtdpPolicy.robotStateActionIndices.containsKey(state)) {
			// action not added to uctPolicy
			addStateActions(state, sumCosts);

		}

		//now that we have all the actions 
		//		double stateCost = this.getStateCost(state); 
		Entry<Object, ArrayList<Double>> rett = brtdpPolicy.getMinMaxValue(state, !doProb, true, printVal);
		ArrayList<Double> stateUpperV = rett.getValue();
		if (printVal)
			mainLog.println("Updating Upper Bounds with " + stateUpperV.toString());
		double updatedProb = stateUpperV.get(0);
		double updatedProg = stateUpperV.get(1);
		double updatedCost = stateUpperV.get(2);
		brtdpPolicy.setProbRewValues(state, updatedProb, updatedProg, updatedCost, true);
		return updatedProb;

	}

	double updateLowerValue(State state, boolean sumCosts, boolean printVal) throws PrismException
	{
		if (printVal)
			mainLog.println("Updating Lower Value");
		if (!brtdpPolicy.robotStateActionIndices.containsKey(state)) {
			// action not added to uctPolicy
			addStateActions(state, sumCosts);

		}

		Entry<Object, ArrayList<Double>> rett = brtdpPolicy.getMinMaxValue(state, !doProb, false, printVal);
		ArrayList<Double> stateLowerV = rett.getValue();
		if (printVal)
			mainLog.println("Updated Lower Bounds " + stateLowerV.toString());
		brtdpPolicy.setProbRewValues(state, stateLowerV.get(0), stateLowerV.get(1), stateLowerV.get(2), false);

		return stateLowerV.get(0);
	}

	double updateLowerValueAction(State state, Object act, boolean printVal) throws PrismException
	{
		if (printVal)
			mainLog.println("Updating Lower Value Action");
		//		double stateLowerV = brtdpPolicy.getProbValueIgnoreSelfLoop(state, false, isgoal);
		double probqval = brtdpPolicy.getProbQValue(state, act, false, printVal);
		double costqval = brtdpPolicy.getCostQValue(state, act, false, printVal);
		double progqval = brtdpPolicy.getProgQValue(state, act, false, printVal);
		if (printVal)
			mainLog.println("Updated Lower Bounds with " + act.toString() + ":" + probqval + "," + progqval + "," + costqval);
		brtdpPolicy.setProbRewValues(state, probqval, progqval, costqval, false);

		return probqval;
	}

	double updateUpperValueAction(State state, Object act, boolean printVal) throws PrismException
	{
		if (printVal)
			mainLog.println("Updating Upper Value Action");
		//		double stateLowerV = brtdpPolicy.getProbValueIgnoreSelfLoop(state, false, isgoal);
		double probqval = brtdpPolicy.getProbQValue(state, act, true, printVal);
		double costqval = brtdpPolicy.getCostQValue(state, act, true, printVal);
		double progqval = brtdpPolicy.getProgQValue(state, act, true, printVal);
		if (printVal)
			mainLog.println("Updated Upper Bounds with " + act.toString() + ":" + probqval + "," + progqval + "," + costqval);
		brtdpPolicy.setProbRewValues(state, probqval, progqval, costqval, true);

		return probqval;
	}

	Object getLowerValueAction(State state, boolean printVal) throws PrismException
	{
		if (printVal)
			mainLog.println("Getting Lower Action");
		if (!brtdpPolicy.robotStateActionIndices.containsKey(state))
			throw new PrismException("State not added to robotStateActionIndices" + state.toString());
		//		double stateCost = getStateCost(state); 
		Entry<Object, ArrayList<Double>> rett = brtdpPolicy.getMinMaxValue(state, !doProb, false, printVal);
		Object action = rett.getKey();
		if (printVal)
			mainLog.println("Chose action - " + action.toString());
		return action;
	}

	Object getUpperValueAction(State state, boolean printVal) throws PrismException
	{
		if (printVal)
			mainLog.println("Getting Upper Action");
		if (!brtdpPolicy.robotStateActionIndices.containsKey(state))
			throw new PrismException("State not added to robotStateActionIndices" + state.toString());
		//		double stateCost = getStateCost(state); 

		Entry<Object, ArrayList<Double>> rett = brtdpPolicy.getMinMaxValue(state, !doProb, true, printVal);
		Object action = rett.getKey();
		if (printVal)
			mainLog.println("Chose action - " + action.toString());
		return action;
	}

	/*
	 * okay no this is the money pit 
	 * THIS IS TRUE 
	 * we do things here 
	 */

	void addStateActionToMDP(State state, Object action, boolean sumCosts, State updatedTState, State updatedAState) throws PrismException
	{
		//add this state and action slowly 
		//		//add this action to our list 
		//		boolean checkhere = false;
		//		if (action.toString().contains("failed,v4_6"))
		//			checkhere = true;
		ArrayList<State> currentStates = getRobotStatesFromJointState(state);
		if (!brtdpPolicy.robotStateActionIndices.get(state).containsKey(action)) {
			addStateAction(state, action, sumCosts);
		}
		ArrayList<Integer> actionIndices = brtdpPolicy.robotStateActionIndices.get(state).get(action);

		HashMap<Integer, HashMap<State, Double>> succStates = new HashMap<Integer, HashMap<State, Double>>();
		for (int i = 0; i < num_robots; i++) {
			boolean dontupdatestate = false;
			if (updatedTState != null) {
				double tstate = (double) updatedTState.varValues[i];
				if (tstate != 0) {
					//					mainLog.println("figure out what to do here");

					dontupdatestate = true;
				} else {
					String astate = updatedAState.varValues[i].toString();
					if (astate != "")
						dontupdatestate = true;
				}
				//get the time here and then dont explore it for the robot which hasnt completed its task 
				//and thats it really 
				//this changes the combinations we'll get too 
				//so we'll have to change those 
			}
			HashMap<State, Double> ahashmap = new HashMap<State, Double>();
			if (!dontupdatestate) {

				int choice = actionIndices.get(i);
				ProductModelGenerator pmg = prodModGens.get(i);
				pmg.exploreState(currentStates.get(i));
				int nt = pmg.getNumTransitions(choice);
				for (int t = 0; t < nt; t++) {
					double prob = pmg.getTransitionProbability(choice, t);
					State succState1 = pmg.computeTransitionTarget(choice, t);
					ahashmap.put(succState1, prob);

				}
			} else {
				ahashmap.put(currentStates.get(i), 1.0);
			}
			succStates.put(i, ahashmap);

		}

		ArrayList<ArrayList<State>> jointSuccessors = getJointStateCombinations(succStates);
		ArrayList<State> jointsuccStates = getJointStates(jointSuccessors, updatedTState, updatedAState);
		ArrayList<Double> probs = calculateJointStatesProbs(succStates, jointSuccessors);
		// now we just choose one of these
		// if its the first one we should add these to uctPolicy

		brtdpPolicy.addAction(state, action, jointsuccStates, probs);
		double reward = this.getRobotsReward(state, action, sumCosts);
		brtdpPolicy.addActionReward(state, action, reward);

	}

	void removeFromLeafStates(State s)
	{
		//get state index 
		int sIndex = brtdpPolicy.getStateIndex(s);
		if (sIndex != -1) {
			if (brtdpPolicy.leafStates.get(sIndex))
				brtdpPolicy.leafStates.set(sIndex, false);
		}
	}

	void BRTDPTrial(State state, Stack<State> traj, double tau, boolean sumCosts, int maxTrialLen) throws PrismException, IOException
	{
		numTrials++;
		boolean printVal = true;
		if (MRmcts.TURNOFFALLWRITES)
			printVal = false;
		//setting a max traj length 
		//just a quick thing really 
		//		int maxTrajectoryLen = 10;
		State currState = state;
		//export all state values 
		brtdpPolicy.exportAllStatesValues(MRmcts.TESTSLOC + "values_" + numTrials + "-" + traj.size() + ".txt");
		//		ArrayList<State> seenStates = new ArrayList<State>();
		double successorsDiff = 1000;
		double stateDiff = brtdpPolicy.getProbValueDiff(state, brtdpPolicy.isGoal(state), true);
		boolean comp = successorsDiff > (stateDiff / tau);
		while (comp) {

			//			if (!seenStates.contains(currState)) {
			//				seenStates.add(currState);
			//			} else {
			//				if (brtdpPolicy.isSolved(currState))
			//					break;
			//				else {
			//					if (checkSolved(currState, EPSILON, sumCosts, printVal))
			//						break;
			//				}
			//			}
			if (brtdpPolicy.isSolved(currState)) {
				if (!brtdpPolicy.hasSuccessors(currState))
					brtdpPolicy.addToLeafStates(currState);

				break;
			}
			if (traj.size() > maxTrialLen) {
				if (!brtdpPolicy.hasSuccessors(currState))
					brtdpPolicy.addToLeafStates(currState);

				break;
			}
			if (printVal) {
				mainLog.println("*************************************************************");
				mainLog.println("*************************************************************");
				brtdpPolicy.printAllStatesValues();
				mainLog.println("*************************************************************");
				mainLog.println("*************************************************************");
				mainLog.println("=============================================================");
				mainLog.println("=============================================================");
			}
			traj.push(currState);
			//			Entry<Double, Double> probstatecost = getStateCost(currState);
			//			double probCost = probstatecost.getKey();
			//			double stateCost = probstatecost.getValue();
			//			double probCost = getStateCost(currState);
			if (printVal) {
				mainLog.println(currState.toString());
				//				mainLog.println(currState.toString() + ":" + probCost + "," + stateCost);
				brtdpPolicy.printStateValues(currState);
				mainLog.println("=============================================================");
			}
			//			updateLowerValue(currState, sumCosts, printVal);
			updateUpperValue(currState, sumCosts, printVal);
			brtdpPolicy.exportAllStatesValues(MRmcts.TESTSLOC + "values_" + numTrials + "-" + traj.size() + "_0.txt");
			if (printVal)
				mainLog.println("=============================================================");
			Object action =
					//					getUpperValueAction(currState,printVal);//
					getLowerValueAction(currState, printVal);
			brtdpPolicy.exportAllStatesValues(MRmcts.TESTSLOC + "values_" + numTrials + "-" + traj.size() + "_1.txt");
			if (printVal)
				mainLog.println("=============================================================");
			updateLowerValueAction(currState, action, printVal);
			//			updateUpperValueAction(currState, action,printVal);
			brtdpPolicy.exportAllStatesValues(MRmcts.TESTSLOC + "values_" + numTrials + "-" + traj.size() + "_2.txt");
			if (printVal) {
				mainLog.println("=============================================================");
				brtdpPolicy.printStateValues(currState);
				mainLog.println("=============================================================");
			}
			//now we have to get all the successors 
			HashMap<State, Double> succs = brtdpPolicy.getSuccessors(currState, action);
			if (succs == null) {
				//we need to add the action and stuff 
				addStateActionToMDP(currState, action, sumCosts, null, null);
				succs = brtdpPolicy.getSuccessors(currState, action);
			}
			if (succs != null) {
				//remove from leaf states 
				removeFromLeafStates(currState);
			}
			//so now we have all the successors 
			//we have to do a loop thing 
			double sumB = 0.0;
			double b = 0.0;
			double rand = new Random().nextDouble();
			double sumProb = 0.0;
			State chosenSuccState = null;
			//assuming the states are ordered by prob 
			HashMap<State, Double> statebs = new HashMap<State, Double>();
			for (State s : succs.keySet()) {
				double prob = succs.get(s);
				double succStateDiff = brtdpPolicy.getProbValueDiff(s, brtdpPolicy.isGoal(s), printVal);
				b = prob * succStateDiff;
				statebs.put(s, b);
				sumB += b;
				//				if(!brtdpPolicy.hasSuccessors(s))
				//					brtdpPolicy.addToLeafStates(s);

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
			if (sumB != 1.0) {
				for (State s : statebs.keySet()) {
					b = statebs.get(s);
					statebs.put(s, b / sumB);
				}
			}

			for (State s : statebs.keySet()) {
				//assuming ordered by prob
				if (sumProb < rand) {
					sumProb += statebs.get(s);
					if (sumProb > rand) {
						chosenSuccState = s;
						break;
					}
				}
				if (rand == 0) {
					chosenSuccState = s;
					break;
				}
			}

			//now we do the check 

			successorsDiff = sumB;
			stateDiff = brtdpPolicy.getProbValueDiff(state, brtdpPolicy.isGoal(state), printVal);
			comp = successorsDiff > (stateDiff / tau);
			currState = chosenSuccState;
			if (printVal) {
				mainLog.println("=============================================================");
				mainLog.println("=============================================================");
			}

			brtdpPolicy.exportAllStatesValues(MRmcts.TESTSLOC + "values_" + numTrials + "-" + traj.size() + "_3.txt");
		}

	}

	void BRTDPBackup(Stack<State> traj, double epsilon, boolean sumCosts) throws PrismException
	{
		//update this with checksolved 

		boolean printVal = true;

		if (MRmcts.TURNOFFALLWRITES)
			printVal = false;
		//do stuff here 
		while (!traj.isEmpty()) {
			State s = traj.pop();
			if (!brtdpPolicy.isSolved(s)) {
				if (!checkSolved(s, epsilon, sumCosts, printVal))
					break;
			}
		}
		//		while (!traj.isEmpty()) {
		//			//backup everyone else too 
		//			State s = traj.pop();
		//			doStateBackup(s, sumCosts, printVal);
		//		}
	}

	double residual(State s, boolean upperBound, boolean sumCosts, boolean allCosts) throws PrismException
	{
		boolean isgoal = brtdpPolicy.isGoal(s);
		double probValue = brtdpPolicy.getProbValue(s, upperBound, false);
		double progValue = brtdpPolicy.getProgValue(s, upperBound, false);
		double costValue = brtdpPolicy.getCostValue(s, upperBound, false);
		//		Entry<Double, Double> probstatecost = getStateCost(s);
		//		double probCost = probstatecost.getKey();
		//		double stateCost = probstatecost.getValue();
		if (!brtdpPolicy.robotStateActionIndices.containsKey(s)) {
			// action not added to uctPolicy
			addStateActions(s, sumCosts);

		}
		//		if(isgoal)
		//			isgoal=true; 

		Entry<Object, ArrayList<Double>> rett = brtdpPolicy.getMinMaxValue(s, !doProb, upperBound, true);

		ArrayList<Double> probcostQVal = rett.getValue();
		double probQVal = probcostQVal.get(0);
		double progQVal = probcostQVal.get(1);
		double costQVal = probcostQVal.get(2);

		double res = 0;
		if (upperBound) {
			//upperBound should always decrease
			//old > new
			res = probValue - probQVal;

		} else {
			res = probQVal - probValue;
		}
		if (allCosts) {
			if (res == 0) {
				if (upperBound) {
					res = progValue - progQVal;
				} else {
					res = progQVal - progValue;
				}
				if (res == 0) {
					if (upperBound) {
						res = costValue - costQVal;

					} else {
						res = costQVal - costValue;
					}
				}
			}
		}
		//		if (bothCosts) {
		//			if (res == 0) {
		//				if (upperBound) {
		//					res = costValue - costQVal;
		//				} else
		//					res = costQVal - costValue;
		//			}
		//		}
		if (res < 0) {
			//			throw new PrismException
			mainLog.println("residual less than 0:" + res + " - " + s.toString() + ", u:" + upperBound);
			res = res * -1;
		}
		return res;
	}

	boolean checkSolved(State currState, double absError, boolean sumCosts, boolean printVal) throws PrismException
	{

		boolean retVal = true;
		Stack<State> open = new Stack<State>();
		Stack<State> closed = new Stack<State>();
		open.push(currState);
		State s;
		boolean considerBothCostsInRes = true;
		while (!open.isEmpty()) {
			s = open.pop();
			closed.push(s);
			if (
			//    residual(s, true, sumCosts,considerBothCostsInRes) > absError | 
			residual(s, false, sumCosts, considerBothCostsInRes) > absError) {
				retVal = false;
				continue;
			}
			//			Entry<Double, Double> probstatecost = getStateCost(s);
			//			double probCost = probstatecost.getKey();
			//			double stateCost = probstatecost.getValue();
			Object bestAction = getUpperValueAction(s, printVal);
			HashMap<State, Double> succs = brtdpPolicy.getSuccessors(s, bestAction);
			if (succs == null) {
				if (!brtdpPolicy.isGoal(s)) {
					if (bestAction != null) {
						if (!bestAction.toString().contains("*")) {
							//we need to add the action and stuff 
							addStateActionToMDP(s, bestAction, sumCosts, null, null);
							succs = brtdpPolicy.getSuccessors(s, bestAction);
						}
					}
				}
			}
			if (succs != null) {
				for (State succ : succs.keySet()) {
					if (!brtdpPolicy.isSolved(succ)) {

						if (!open.contains(succ) && !closed.contains(succ)) {
							open.push(succ);
						}
					}

				}
			}
		}
		if (retVal == true) {

			while (!closed.isEmpty()) {
				s = closed.pop();
				brtdpPolicy.stateSolved.put(s, true);
			}
		} else {
			while (!closed.isEmpty()) {
				s = closed.pop();
				doStateBackup(s, sumCosts, printVal);
			}
		}
		return retVal;

	}

	void doStateBackup(State s, boolean sumCosts, boolean printVal) throws PrismException
	{
		//		Entry<Double, Double> probstatecost = getStateCost(s);
		//		double probCost = probstatecost.getKey();
		//		double stateCost = probstatecost.getValue();
		if (printVal) {
			mainLog.println(s.toString() /*+ ":" + probCost + "," + stateCost*/);
			brtdpPolicy.printStateValues(s);
		}
		this.updateUpperValue(s, sumCosts, printVal);
		this.updateLowerValue(s, sumCosts, printVal);
		if (printVal)
			brtdpPolicy.printStateValues(s);
	}

	void runBRTDPTrial(State state, double tau, int maxTrialLen, boolean sumCosts) throws PrismException, IOException
	{

		//do not sum robot costs, get the max cost instead 

		//if sumcosts is true then our upper bound for cost will change 
		//well even now it will not be a true upper bound cuz the other one is moving still 
		//so we have to do something about this 
		Stack<State> trajectory = new Stack<State>();

		mainLog.println("BRTDP Trial");
		BRTDPTrial(state, trajectory, tau, sumCosts, maxTrialLen);
		mainLog.println("BRTDP Backup");
		BRTDPBackup(trajectory, EPSILON, sumCosts);
	}

	//a clean brtdp implementation 
	//for probs only 
	//v_u = 1 
	//v_l = 0
	public Object doBRTDP(String fn, String sp, BitSet accStates, boolean minCost, int maxTrialLen) throws Exception
	{
		//		ArrayList<Integer> accsFound = new ArrayList<Integer>();

		if (doProb)
			minCost = false;

		boolean printVal = true;
		if (MRmcts.TURNOFFALLWRITES)
			printVal = false;
		referenceSSState = null; // this is important to do cuz like we have a reference state 

		ArrayList<State> initialStates = getInitialStates();
		// get the initial state
		State temp_joint_state = getJointState(initialStates, null, null, null);
		mainLog.println("Joint State:" + temp_joint_state.toString());
		Object res = null;
		double alpha = 0.1; //change this later 
		double tau = 50; //they used something between 10 and 100 

		double stateDiff = brtdpPolicy.getProbValueDiff(temp_joint_state, brtdpPolicy.isGoal(temp_joint_state), printVal);
		double oldStateDiff = 100;
		String saveplace = sp;
		// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = fn;
		int iter = -1;
		int numSameStateDiff = 0;
		boolean sumCosts = false;
		//		int maxSameStateDiffs = 20;
		//		int maxTrialLen = //size of single mdp * num robots; 
		while (stateDiff > alpha && !brtdpPolicy.isSolved(temp_joint_state)) {
			iter++;
			runBRTDPTrial(temp_joint_state, tau, maxTrialLen, sumCosts);
			oldStateDiff = stateDiff;
			stateDiff = brtdpPolicy.getProbValueDiff(temp_joint_state, brtdpPolicy.isGoal(temp_joint_state), printVal);

			String stateDiffString = "" + stateDiff;

			stateDiffString = stateDiffString.replace(".", "_");
			mainLog.println("State Diff " + stateDiff);
			brtdpPolicy.exportAllStatesValues(saveplace + filename + "_brtdpmdp_values_" + stateDiffString + "_" + iter + ".txt");

			if ((stateDiff - oldStateDiff) == 0) {
				//there is really no change 
				mainLog.println("No difference in the state difference - so we're done");
				numSameStateDiff++;
				//				if (numSameStateDiff > maxSameStateDiffs)

			} else {
				if (numSameStateDiff != 0)
					numSameStateDiff = 0;
			}

			PrismFileLog out = new PrismFileLog(saveplace + filename + "_brtdpmdp_" + stateDiffString + "_" + iter + ".dot");
			brtdpPolicy.jointMDP.exportToDotFile(out, null, true);
			out.close();
			MDPSimple policyTree = extractPolicyTreeAsDotFile(brtdpPolicy.jointMDP, brtdpPolicy.getStateIndex(temp_joint_state), minCost, true, sumCosts);

			out = new PrismFileLog(saveplace + filename + "_brtdp_policy_" + stateDiffString + "_" + iter + "u.dot");
			policyTree.exportToDotFile(out, null, true);
			out.close();
			policyTree = extractPolicyTreeAsDotFile(brtdpPolicy.jointMDP, brtdpPolicy.getStateIndex(temp_joint_state), minCost, false, sumCosts);

			out = new PrismFileLog(saveplace + filename + "_brtdp_policy_" + stateDiffString + "_" + iter + "l.dot");
			policyTree.exportToDotFile(out, null, true);
			out.close();
			//			if (brtdpPolicy.isSolved(temp_joint_state))
			//				break;
		}

		PrismFileLog out = new PrismFileLog(saveplace + filename + "_brtdpmdp_final.dot");
		brtdpPolicy.jointMDP.exportToDotFile(out, null, true);
		out.close();
		MDPSimple policyTree = extractPolicyTreeAsDotFile(brtdpPolicy.jointMDP, brtdpPolicy.getStateIndex(temp_joint_state), minCost, true, sumCosts);

		out = new PrismFileLog(saveplace + filename + "_brtdp_policy_final_u.dot");
		policyTree.exportToDotFile(out, null, true);
		out.close();
		policyTree = extractPolicyTreeAsDotFile(brtdpPolicy.jointMDP, brtdpPolicy.getStateIndex(temp_joint_state), minCost, false, sumCosts);

		out = new PrismFileLog(saveplace + filename + "_brtdp_policy_final_l.dot");
		policyTree.exportToDotFile(out, null, true);
		out.close();

		mainLog.println("Max Iterations "+iter);
		return res;
	}

	public MDPSimple extractPolicyTreeAsDotFile(MDPSimple mdp, int initialState, boolean minimiseCosts, boolean upperBound, boolean sumCosts)
	{

		MDPSimple policyTree = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		HashMap<Integer, Integer> stateLabels = new HashMap<Integer, Integer>();
		//		Arrays.fill(stateLabels, -1);
		Queue<Integer> stateQ = new LinkedList<Integer>();
		stateQ.add(initialState);
		int state, ps, choice;
		Object action = null;
		BitSet visited = new BitSet();

		while (!stateQ.isEmpty()) {
			action = null;
			state = stateQ.remove();
			if (!visited.get(state)) {
				visited.set(state);
				if (!stateLabels.containsKey(state)) {
					stateLabels.put(state, policyTree.addState());
					statesList.add(mdp.getStatesList().get(state));
				}
				ps = stateLabels.get(state);
				State statestate = brtdpPolicy.getStateFromIndex(state);
				try {
					Entry<Object, ArrayList<Double>> rett = brtdpPolicy.getMinMaxValue(statestate, minimiseCosts, upperBound, false);//getBestAction(statestate, minimiseCosts);
					if (rett == null) {
						if (!brtdpPolicy.isGoal(statestate))
						//							action = null; 
						//						else
						{
							if (!isDeadend(statestate))
							//								action = null; 
							//							else 
							{
								mainLog.println("Attempting to add actions to state");
								addStateActions(statestate, sumCosts);
								rett = brtdpPolicy.getMinMaxValue(statestate, minimiseCosts, upperBound, false);//getBestAction(statestate, minimiseCosts);

							}
						}

					}
					if (rett != null)
						action = rett.getKey();
				} catch (PrismException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				if (action != null) {
					if (brtdpPolicy.stateActionIndices.containsKey(statestate)) {
						if (brtdpPolicy.stateActionIndices.get(statestate).containsKey(action)) {
							choice = brtdpPolicy.stateActionIndices.get(statestate).get(action);
							if (choice > -1) {
								Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(state, choice);
								Distribution distr = new Distribution();
								while (tranIter.hasNext()) {
									Entry<Integer, Double> csp = tranIter.next();
									int childstate = csp.getKey();
									double stateProb = csp.getValue();
									//									if (stateLabels[childstate] == -1) {
									//										stateLabels[childstate] = policyTree.addState();
									//										statesList.add(mdp.getStatesList().get(childstate));
									//									}
									if (!stateLabels.containsKey(childstate)) {
										stateLabels.put(childstate, policyTree.addState());
										statesList.add(mdp.getStatesList().get(childstate));
									}
									int cs = stateLabels.get(childstate);
									distr.add(cs, stateProb);
									stateQ.add(childstate);
								}
								policyTree.addActionLabelledChoice(ps, distr, action.toString() + brtdpPolicy.getStateValues(statesList.get(ps)));
							}

						}
					}
				}

			}
		}
		policyTree.setStatesList(statesList);

		return policyTree;
	}
}
