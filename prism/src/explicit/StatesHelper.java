package explicit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Vector;

import parser.State;

public class StatesHelper {
	static int robotVar = 0;
	static int mdpVar;
	public static int failState = -1; 
	public static int BADVALUE = -2; 

	public static BitSet addLinkInMDP(MDPSimple mdp, int[] mdpMap, List<State> statesList,
			ArrayList<Integer> states, ArrayList<Double> probs, int parentState, Object action,
			String additionalText, BitSet accStates, BitSet finalAcceptingStates) {
		accStates = addStateMDP(mdp, mdpMap, statesList, BADVALUE, parentState, accStates, finalAcceptingStates);
		Distribution distr = new Distribution();

		for (int s = 0; s < states.size(); s++) {
			accStates = addStateMDP(mdp, mdpMap, statesList, BADVALUE, states.get(s), accStates,
					finalAcceptingStates);
			distr.add(mdpMap[states.get(s)], probs.get(s));
		}
		int numChoices = mdp.getNumChoices(mdpMap[parentState]);
		if (action == null)
			mdp.addActionLabelledChoice(mdpMap[parentState], distr, additionalText + "." + numChoices);
		else {
			mdp.addActionLabelledChoice(mdpMap[parentState], distr,
					additionalText + "_" + action.toString() + "." + numChoices);}
		return accStates;
	}

	public static BitSet addStateMDP(MDPSimple mdp, int[] mdpMap, List<State> states, int stateNotAdded, int state,
			BitSet accStates, BitSet finalAcceptingStates) {
		if (mdpMap[state] == stateNotAdded) {
			// add to states list
			mdpMap[state] = mdp.getNumStates();
			mdp.statesList.add(states.get(state)); // should work
			mdp.addState();
			if (finalAcceptingStates.get(state)) {
				accStates.set(mdpMap[state]);
			}

		}
		return accStates;
	}
	
	public static boolean areEqual(State s1, State s2, Vector<Integer> indicesToMatch) {
		boolean result = true;

		Object[] s1Array = s1.varValues;
		Object[] s2Array = s2.varValues;

		for (int i = 0; i < indicesToMatch.size(); i++) {
			int ind = indicesToMatch.get(i);
			if ((int) s1Array[ind] != (int) s2Array[ind]) {
				result = false;
				break;
			}
		}

		return result;
	}

	public static boolean checkMDPs(MDPSimple mdps[], int[][] mdpMaps, List<State> states, int[] robotInitStates,
			int state, BitSet[] accStates, BitSet acceptingStates) {
		boolean res = true;
		for (int r = 0; r < mdps.length; r++) {
			if (mdps[r].getNumStates() == 0) {
				if (robotInitStates == null) {
					res = false;
					break;
				} else {
					int currState = robotInitStates[r];
					accStates[r] = addStateMDP(mdps[r], mdpMaps[r], states, BADVALUE, currState, accStates[r],
							acceptingStates);
					mdps[r].addInitialState(0);
				}
			}
		}
		if (!res)
			System.out.println("ERROR HERE!!!");
		return res;
	}

	public static Object[] createRefStateObject() {
		// get the first state
		Object[] ref = new Object[mdpVar + 1];
		for (int i = 0; i < ref.length; i++) {
			ref[i] = 0;
		}
		return ref;
	}

	public static int getExactlyTheSameState(Object s1v[], List<State> states) {
		int res = BADVALUE;
		for (int s = 0; s < states.size(); s++) {
			if (statesAreEqual(s1v, states.get(s))) {
				res = s;
				break;
			}
		}
		return res;
	}

	public static int getIndexValueFromState(State s1, int index) {
		return (int) s1.varValues[index];
	}

	public static int getMDPStateFromState(State s1) {
		Object[] stateVar = s1.varValues;
		return (int) stateVar[mdpVar];

	}

	public static Object[] getMergedState(State s1, State s2, int statesToKeeps2[]) {
		// int res = BADVALUE;
		Object s1v[] = s1.varValues.clone();
		Object s2v[] = s2.varValues;
		for (int i = 0; i < statesToKeeps2.length; i++) {
			s1v[statesToKeeps2[i]] = s2v[statesToKeeps2[i]];
		}
		// res = getExactlyTheSameState(s1v);
		return s1v;
	}

	public static int getMergedStateRobotMDP(State s1, State s2, List<State> states) {
		int statesToKeeps2[] = { robotVar, mdpVar };
		Object[] mergedState = getMergedState(s1, s2, statesToKeeps2);
		return getExactlyTheSameState(mergedState, states);

	}

	public static int getRobotNumberFromState(State s1) {
		Object[] stateVar = s1.varValues;
		return (int) stateVar[robotVar];

	}


	public static boolean isFailState(State s1) {
		int ind = mdpVar;
		if (getIndexValueFromState(s1, ind) == failState)
			return true;
		else
			return false;
	}

	public static void setMDPVar(int var) {
		mdpVar = var;
	}

	public static boolean statesAreEqual(Object s1v[], State s2) {
		boolean res = true;
		Object s2v[] = s2.varValues;
		int numVar = s2v.length;
		for (int v = 0; v < numVar; v++) {
			if ((int) s1v[v] != (int) s2v[v]) {
				res = false;
				break;
			}
		}
		return res;
	}

	public static boolean statesHaveTheSameAutomataProgress(State s1, State s2) {
		// hardcoding this here
		Vector<Integer> indicesToMatch = new Vector<Integer>();
		int numVar = s1.varValues.length;

		// we know that the ones at the end are robot num and mdp state so we can skip
		// those

		for (int i = 1; i < numVar - 1; i++) {
			indicesToMatch.add(i);
		}
		return areEqual(s1, s2, indicesToMatch);
	}

	public static Object[] XORStates(State s1, State s2, Object[] refState
	/* a state with the initial states of all DA */) {
		// so we need to exclude the robot and the mdp states from this
		Object[] s1v = s1.varValues;
		Object[] s2v = s2.varValues;
		Object[] resS = new Object[s1v.length - 2]; // hard coding this
		for (int i = robotVar + 1; i < mdpVar; i++) {
			if (s1v[i] != s2v[i])
				resS[i - 1] = s2v[i];
			else
				resS[i - 1] = refState[i];
		}
		return resS;
	}

}