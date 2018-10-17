package demos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Queue;

import explicit.MDPSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationIntUnbounded;
import prism.PrismLangException;
import prism.PrismLog;
import strat.MDStrategyArray;

/*
 * An attempt to clean up code 
 * and make this more doable 
 * A new class for creating and storing the joint policy
 * so maybe this isnt the smartest name
 */
public class JointPolicyBuilder {

	/*
	 * Storing state realted information parent state, robot number, probability etc
	 * just for ease of use
	 */
	public class StateExtended implements Comparable<StateExtended> {
		protected int parentState;
		protected int parentStateRobot;
		protected int childState;
		protected int childStateRobot;
		protected double parentToChildTransitionProbability;
		protected String action;

		public StateExtended(int ps, int psr, int cs, int csr, double prob, String a) {
			parentState = ps;
			parentStateRobot = psr;
			childState = cs;
			childStateRobot = csr;
			parentToChildTransitionProbability = prob;
			action = a;
		}

		public StateExtended(int s, double prob, String a) {
			childState = s;
			parentToChildTransitionProbability = prob;
			action = a;
		}

		public StateExtended(StateExtended other) {
			this.parentState = other.parentState;
			this.parentStateRobot = other.parentStateRobot;
			this.childState = other.childState;
			this.childStateRobot = other.childStateRobot;
			this.parentToChildTransitionProbability = other.parentToChildTransitionProbability;
			this.action = other.action;
		}

		@Override
		public int compareTo(StateExtended other) {
			double comp = this.parentToChildTransitionProbability - other.parentToChildTransitionProbability;
			int res = 0;
			if (comp > 0)
				res = -1;
			else {
				if (comp < 0) {
					res = 1;
				}
			}
			return res;
		}

		@Override
		public String toString() {
			return "StateInfo [parentState=" + parentState + ", parentStateRobot=" + parentStateRobot + ", childState="
					+ childState + ", childStateRobot=" + childStateRobot + ", parentToChildTransitionProbability="
					+ parentToChildTransitionProbability + ", action=" + action + "]";
		}

	}
	// elements
	// store the joint policy - mdp
	// things to help us remember stuff for the mdp
	// map - indicies and states
	// shared variable indices list

	int numRobots;
	int numTasks;
	int numSharedStates;
	ArrayList<String> sharedStatesNamesList;
	ArrayList<String> isolatedStatesNamesList;

	protected MDPSimple jointMDP;

	PrismLog mainLog;

	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList, VarList seqTeamMDPVarList,
			PrismLog log) {
		ArrayList<String> isolatedStatesList = new ArrayList<String>();
		for (int i = 0; i < seqTeamMDPVarList.getNumVars(); i++) {
			String name = seqTeamMDPVarList.getName(i);
			if ((!sharedStatesList.contains(name)) && (!name.contains("da")) && (name != "r")) {
				isolatedStatesList.add(name);
			}
		}
		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, log);
	}

	public JointPolicyBuilder(int nrobots, int ntasks, ArrayList<String> sharedStatesList,
			ArrayList<String> isolatedStatesList, PrismLog log) {
		initialize(nrobots, ntasks, sharedStatesList, isolatedStatesList, log);
	}

	private VarList createVarList() {
		VarList varlist = new VarList();

		// end of the list - isolated robot states in order
		// we're making the assumption that all isolated states can be lumped into one
		String decname;
		try {

			for (int i = numRobots - 1; i >= 0; i--) {
				decname = "r" + i;
				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
			}

			for (int i = 0; i < numSharedStates; i++) {
				decname = sharedStatesNamesList.get(i);
				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
			}
			for (int i = 0; i < numTasks; i++) {
				decname = "da" + i;
				varlist.addVar(0, new Declaration(decname, new DeclarationIntUnbounded()), 1, null);
			}

		} catch (PrismLangException e) {
			e.printStackTrace();
		}

		return varlist;

	}

	private void initialize(int nrobots, int ntasks, ArrayList<String> sharedStatesList,
			ArrayList<String> isolatedStatesList, PrismLog log) {
		numRobots = nrobots;
		numTasks = ntasks;
		numSharedStates = sharedStatesList.size();
		sharedStatesNamesList = sharedStatesList;
		isolatedStatesNamesList = isolatedStatesList;
		mainLog = log;

		jointMDP = new MDPSimple();
		jointMDP.setVarList(createVarList());
		jointMDP.setStatesList(new ArrayList<State>());

	}

	// the real thing
	// input: \pi_seq, s_J or \bar(s_G)
	protected void buildJointPolicyFromSequentialPolicy(MDStrategyArray strat, MDPSimple mdp, int initialState,
			boolean isJointState) {
		Queue<State> jointStateQueue = null;
		if (jointStateQueue != null) {
			jointStateQueue.add(null); // add the joint state to the queue
			while (!jointStateQueue.isEmpty()) {

				State currentJointState = jointStateQueue.remove();

				//if failure 
				//continue; 
				//else 
				//do below
				int[] robotStatesInSeqTeamMDP = extractIndividualRobotStatesFromJointState(currentJointState);
				
				if (getNewTaskAllocation())
					getTaskAllocationForAllRobots(strat, mdp, robotStatesInSeqTeamMDP);

				Object mostProbableTaskAllocationStateValues = getStateIndexValuesForTaskAllocationForAllRobots();

				int[] modifiedRobotStatesInSeqTeamMDP = modifyRobotStatesToReflectExpectedTaskCompletion(
						robotStatesInSeqTeamMDP, mostProbableTaskAllocationStateValues);

				Object[] combinations = getActionAndSuccStatesAllRobots(strat, modifiedRobotStatesInSeqTeamMDP);

				Object action = combinations[0];
				ArrayList<State> succStatesQueue=null; 
				ArrayList<Double> succStatesProbQueue = null;
				for (Object combination : combinations) {
					{
						int[] newSuccStatesForJointState = modifyRobotStatesToUndoExpectedTaskCompletion(modifiedRobotStatesInSeqTeamMDP,mostProbableTaskAllocationStateValues);
						State succJointState = createJointState(newSuccStatesForJointState);
						succStatesQueue.add(succJointState);
						succStatesProbQueue.add(0.0);//addPROB here
					}
					
					//add to mdp 

				}

			}
		}

	}

	private boolean getNewTaskAllocation() {
		return true;
	}

	private int[] modifyRobotStatesToReflectExpectedTaskCompletion(int[] robotStatesInSeqTeamMDP,
			Object mostProbableTaskAllocationStateValues) {
		// TODO Auto-generated method stub
		return null;
	}

	private int[] modifyRobotStatesToUndoExpectedTaskCompletion(int[] robotStatesInSeqTeamMDP,
			Object mostProbableTaskAllocationStateValues) {
		// TODO Auto-generated method stub
		return null;
	}

	protected HashMap<Integer, ArrayList<Entry<Integer, Integer>>> getStateIndexValuesForTaskAllocationForAllRobots() {
		return null;
	}

	protected void getTaskAllocationForRobot(MDStrategyArray strat, MDPSimple mdp, int initialState) {
		// I'm not sure what this will return get
	}

	protected void getTaskAllocationForAllRobots(MDStrategyArray strat, MDPSimple mdp, int[] initialStates) {

	}

	protected void getTaskAllocationForAllRobotsFollowingPath(MDStrategyArray strat, MDPSimple mdp, int initialstate) {

	}

	protected void getActionChoice(MDStrategyArray strat, int state) {

	}

	protected void getActionChoiceAllRobots(MDStrategyArray strat, int[] states) {

	}

	protected Object[] getActionAndSuccStatesAllRobots(MDStrategyArray strat, int[] states) {
		return null;

	}

	protected void getTranIter(int choice, int state, MDPSimple mdp) {

	}

	protected void tranIterToArrayList(Iterator<Entry<Integer, Double>> tranIter) {

	}

	protected State createJointState(int[] states) {
		return null;

	}

	protected int[] extractIndividualRobotStatesFromJointState(State jointState) {
		return null;
	}

	protected void createSuccessorStatesCombinations() {

	}

}
