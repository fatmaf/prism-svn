package demos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Vector;

import automata.DA;
import explicit.Distribution;
import explicit.MDP;
import explicit.MDPSimple;
import explicit.rewards.MDPRewards;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import strat.Strategy;

public class StatesHelper {
	static int robotVar = 0;
	static int mdpVarStart;
	public static int failState = -1;
	public static int BADVALUE = -2;
	public static String savePlace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	public static String folder;
	public static int numMdpVars;
	public static PrismLog mainLog;

	public static BitSet addLinkInMDP(MDPSimple mdp, int[] mdpMap, List<State> statesList,int state,
			double prob, int parentState, Object action, String additionalText, BitSet updatedBitSet,
			BitSet statesToUpdate)
	{
		ArrayList<Integer> states = new ArrayList<Integer>(); 
		states.add(state); 
		ArrayList<Double> probs = new ArrayList<Double>(); 
		probs.add(prob);
		return addLinkInMDP(mdp, mdpMap, statesList,  states,
				 probs, parentState, action,  additionalText,  updatedBitSet,
				 statesToUpdate) ;
	}
	public static BitSet addLinkInMDP(MDPSimple mdp, int[] mdpMap, List<State> statesList, ArrayList<Integer> states,
			ArrayList<Double> probs, int parentState, Object action, String additionalText, BitSet updatedBitSet,
			BitSet statesToUpdate) {
		return addLinkInMDP(mdp, mdpMap, statesList,  states,
				 probs, parentState, action,  additionalText,  updatedBitSet,
				 statesToUpdate,false) ;
	}
	public static BitSet addLinkInMDP(MDPSimple mdp, int[] mdpMap, List<State> statesList, ArrayList<Integer> states,
			ArrayList<Double> probs, int parentState, Object action, String additionalText, BitSet updatedBitSet,
			BitSet statesToUpdate,boolean parentStateInMDPAlready) {
		if(!parentStateInMDPAlready)
		updatedBitSet = addStateMDP(mdp, mdpMap, statesList, BADVALUE, parentState, updatedBitSet, statesToUpdate);
		Distribution distr = new Distribution();

		for (int s = 0; s < states.size(); s++) {
			if (statesToUpdate != null)
				updatedBitSet = addStateMDP(mdp, mdpMap, statesList, BADVALUE, states.get(s), updatedBitSet,
						statesToUpdate);
			else
				addStateMDP(mdp, mdpMap, statesList, BADVALUE, states.get(s), updatedBitSet, statesToUpdate);
			distr.add(mdpMap[states.get(s)], probs.get(s));
		}
		if(!parentStateInMDPAlready)
		parentState = mdpMap[parentState];
		int numChoices = mdp.getNumChoices(parentState);
		if (action == null)
			mdp.addActionLabelledChoice(parentState, distr, additionalText + "." + numChoices);
		else {
			mdp.addActionLabelledChoice(parentState, distr,
					additionalText + "_" + action.toString() + "." + numChoices);
		}
		return updatedBitSet;
	}

	public static BitSet addStateMDP(MDPSimple mdp, int[] mdpMap, List<State> states, int stateNotAdded, int state,
			BitSet updatedBitSet, BitSet statesToUpdate) {
		if (mdpMap[state] == stateNotAdded) {
			// add to states list
			mdpMap[state] = mdp.getNumStates();
			mdp.getStatesList().add(states.get(state)); // should work
			mdp.addState();
			if (statesToUpdate != null) {
				if (statesToUpdate.get(state)) {
					updatedBitSet.set(mdpMap[state]);
				}
			}

		}
		return updatedBitSet;
	}

	public static int addStateMDP(MDPSimple mdp, int[] mdpMap, List<State> states, int state) {
		int stateNotAdded = BADVALUE;
		if (mdpMap[state] == stateNotAdded) {
			// add to states list
			mdpMap[state] = mdp.getNumStates();
			mdp.getStatesList().add(states.get(state)); // should work
			mdp.addState();

		}
		return mdpMap[state];
	}

	public static boolean areEqual(Object[] s1, Object[] s2) {
		int maxLen = Math.max(s1.length, s2.length);
		boolean equal = true;
		for (int i = 0; i < maxLen; i++) {
			if (s1[i] != s2[i]) {
				equal = false;
				break;
			}
		}
		return equal;

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
			mainLog.println("ERROR HERE!!!");
		return res;
	}

	public static Object[] getDAStatesFromState(State state) {
		// so return everything from after robotVar upto mdpVarStart
		Object[] stateObj = state.varValues;
		Object[] toret = Arrays.copyOfRange(stateObj, robotVar + 1, mdpVarStart);
		return toret;
	}

	public static Object[] getSharedStatesFromState(State state, VarList list, ArrayList<String> sharedVars) {
		Object[] stateObj = state.varValues;
		Object[] toret = new Object[sharedVars.size()];
		int index = -1;
		int numnull = 0;
		for (int i = 0; i < sharedVars.size(); i++) {
			index = list.getIndex(sharedVars.get(i));
			if (index != -1)
				toret[i] = stateObj[index];
			else
				numnull++;
		}
		if (numnull == sharedVars.size())
			toret = null;
		return toret;

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
		return (int) stateVar[mdpVarStart];

	}

	public static Object[] getMDPStateFromState(State s1, VarList varlist, ArrayList<String> notSharedVarsList) {
		Object[] stateVar = s1.varValues;
		int numVarsNotShared = /* varlist.getNumVars() - */notSharedVarsList.size();
		Object[] mdpState = new Object[numVarsNotShared];
		for (int i = 0; i < numVarsNotShared; i++) {
			int index = varlist.getIndex(notSharedVarsList.get(i));
			if (index != 1)
				mdpState[i] = stateVar[index];
		}
		return mdpState;
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
		int statesToKeeps2[] = { robotVar, mdpVarStart };
		Object[] mergedState = getMergedState(s1, s2, statesToKeeps2);
		return getExactlyTheSameState(mergedState, states);

	}

	public static int getRobotNumberFromState(State s1) {
		Object[] stateVar = s1.varValues;
		return (int) stateVar[robotVar];

	}

	public static String getSaveplace() {
		return savePlace;
	}

	// check to see if s1 is a substring of masterstring starting from index 0
	public static boolean isEntireStringASubstring(String s1, String masterString) {
		return masterString.regionMatches(0, s1, 0, s1.length());

	}

	public static boolean isFailState(State s1) {
		int ind = mdpVarStart;
		if (getIndexValueFromState(s1, ind) == failState)
			return true;
		else
			return false;
	}

	public static MDPSimple mapRewardToMDP(MDPSimple mdp, MDPRewards rew, BitSet setToMark, BitSet updatedBitSet) {
		MDPSimple rewMDP = new MDPSimple();
		rewMDP.setStatesList(new ArrayList<State>());
		rewMDP.setVarList(mdp.getVarList());
		int[] mdpMap = new int[mdp.getNumStates()];
		Arrays.fill(mdpMap, BADVALUE);

		// for each state and action in the MDP
		// add a new state and action and annotate with a reward

		for (int i = 0; i < mdp.getNumStates(); i++) {
			int numChoices = mdp.getNumChoices(i);
			for (int j = 0; j < numChoices; j++) {

				ArrayList<Integer> nextStates = new ArrayList<Integer>();
				ArrayList<Double> nextStatesProbs = new ArrayList<Double>();
				Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(i, j);
				while (tranIter.hasNext()) {
					Entry<Integer, Double> stateprob = tranIter.next();
					nextStates.add(stateprob.getKey());
					nextStatesProbs.add(stateprob.getValue());
				}
				double rewval = rew.getTransitionReward(i, j);
				double staterewval = rew.getStateReward(i);
				updatedBitSet = addLinkInMDP(rewMDP, mdpMap, mdp.getStatesList(), nextStates, nextStatesProbs, i,
						mdp.getAction(i, j), "rt:" + rewval + "rs:" + staterewval, updatedBitSet, setToMark);
			}

		}

		return rewMDP;
	}

	/**
	 * takes mask and multiplies each element in s1 with corresponding elem in mask
	 * 
	 * @param mask
	 * @param s1
	 * @return
	 */
	public static Object[] multiplyWithMask(int[] mask, Object[] s1, Object[] initStates) {
		int maxLen = Math.max(mask.length, s1.length);
		Object[] res = new Object[maxLen];
		for (int i = 0; i < maxLen; i++) {
			if (mask[i] == 0)
				res[i] = initStates[i];
			else
				res[i] = s1[i];

		}
		return res;
	}

	/**
	 * based on the assumption that all DA states start with 0 being the initial
	 * state and that when we're going to "OR" stuff we'll just take s2 values (not
	 * bigger of the two) this may not always be true ofcourse
	 * 
	 * @param s1
	 * @param s2
	 */
	public static Object[] ORIntegers(Object[] s1, Object[] s2, Object[] initStates) {
		int maxLen = Math.max(s1.length, s2.length);
		Object[] res = s2.clone();
		int initState = 0; // FIXME:FATMA bug here this is why you're getting loops
		if (initStates == null) {
			initStates = new Object[maxLen];
			Arrays.fill(initStates, initState);
		}

		for (int i = 0; i < maxLen; i++) {
			if ((int) res[i] == (int) initStates[i])
				res[i] = s1[i];
		}
		return res;
	}

	public static void saveBitSet(BitSet bitset, String anotherfolder, String name, boolean saveinsaveplace) {
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = getSaveplace();
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location + "_" + name);
		out.println(bitset.toString());
		out.close();
	}

	/**
	 * @param anotherfolder
	 *            the location of the tile to save
	 * 
	 * @param saveplace
	 *            alternate location
	 * @param da
	 *            The da
	 * @param name
	 *            filename
	 * 
	 * @param saveinsaveplace
	 *            save in predefined save location (set to true) if false saves in
	 *            same location as adversary
	 * @throws PrismException
	 */
	public static void saveDA(DA da, String anotherfolder, String name, boolean saveinsaveplace) throws PrismException {
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = getSaveplace();
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location + "_" + name + ".dot");
		da.printDot(out);
		out.close();

	}

	public static void saveHashMap(HashMap map, String anotherfolder, String name, boolean saveinsaveplace) {
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = getSaveplace();
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location + "_" + name);
		Iterator it = map.entrySet().iterator();
		while (it.hasNext()) {
			Map.Entry pair = (Map.Entry) it.next();
			out.println(pair.getKey() + ":" + pair.getValue());

		}
		// out.print(map);
		out.close();

	}

	/**
	 * @param anotherfolder
	 *            the location of the tile to save
	 * 
	 * @param saveplace
	 *            alternate location
	 * @param mdp
	 *            The mdp
	 * 
	 * @param statesToMark
	 *            a bitset for states you'd like to highlight in the mdp
	 * 
	 * @param name
	 *            filename
	 * 
	 * @param saveinsaveplace
	 *            save in predefined save location (set to true) if false saves in
	 *            same location as adversary
	 */
	public static void saveMDP(MDP mdp, BitSet statesToMark, String anotherfolder, String name,
			boolean saveinsaveplace) {
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = getSaveplace();
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location + "_" + name + ".dot");
		mdp.exportToDotFile(out, statesToMark, true);
		out.close();

	}

	/**
	 * @param anotherfolder
	 *            the location of the tile to save
	 * 
	 * @param saveplace
	 *            alternate location
	 * @param mdp
	 *            The mdp
	 * 
	 * @param statesToMark
	 *            a bitset for states you'd like to highlight in the mdp
	 * 
	 * @param name
	 *            filename
	 * 
	 * @param saveinsaveplace
	 *            save in predefined save location (set to true) if false saves in
	 *            same location as adversary
	 */
	public static void saveMDPstatra(MDP mdp, String anotherfolder, String name, boolean saveinsaveplace) {
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = getSaveplace();
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location + "_" + name + ".tra");
		mdp.exportToPrismExplicitTra(out);
		out.close();
		out = new PrismFileLog(location + "_" + name + ".sta");
		try {
			mdp.exportStates(Prism.EXPORT_PLAIN, mdp.getVarList(), out);
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		out.close();

	}

	public static void saveReward(MDPSimple mdp, MDPRewardsSimple rew, BitSet statesToMark, String anotherfolder,
			String name, boolean saveinsaveplace) {
		BitSet updatedStatesToMark = new BitSet();
		MDPSimple rewMDP = mapRewardToMDP(mdp, rew, statesToMark, updatedStatesToMark);
		saveMDP(rewMDP, updatedStatesToMark, anotherfolder, name, saveinsaveplace);
		saveMDPstatra(rewMDP, anotherfolder, name, saveinsaveplace);
	}

	/**
	 * @param anotherfolder
	 *            the location of the tile to save
	 * 
	 * @param saveplace
	 *            alternate location
	 * @param strat
	 *            The strategy
	 * 
	 * @param statesToMark
	 *            a bitset for states you'd like to highlight in the mdp
	 * 
	 * @param name
	 *            filename
	 * 
	 * @param saveinsaveplace
	 *            save in predefined save location (set to true) if false saves in
	 *            same location as adversary
	 */
	public static void saveStrategy(Strategy strat, BitSet statesToMark, String anotherfolder, String name,
			boolean saveinsaveplace) {
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = getSaveplace();
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location + "_" + name + ".dot");
		strat.exportDotFile(out);
		out.close();
		out = new PrismFileLog(location + "_" + name + ".actions");
		strat.exportActions(out);
		out.close();
		out = new PrismFileLog(location + "_" + name + ".tra");
		strat.exportInducedModel(out);
		out.close();

	}

	public static void setFolder(String folderVal) {
		folder = folderVal;
	}

	public static void setMDPVar(int var) {
		mdpVarStart = var;
	}

	public static void setNumMDPVars(int mdpvars) {
		numMdpVars = mdpvars;
	}

	public static void setSavePlace(String saveplace) {
		savePlace = saveplace;
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

	/*
	 * written to check for the same automata state for a seq team mdp assuming the
	 * automata states start at 1 and end at mdpVarStart
	 */
	public static boolean statesHaveTheSameAutomataProgress(State s1, State s2) {
		// hardcoding this here
		Vector<Integer> indicesToMatch = new Vector<Integer>();
		int numVar = mdpVarStart;// s1.varValues.length;

		// we know that the ones at the end are robot num and mdp state so we can skip
		// those

		for (int i = 1; i < numVar; i++) {
			indicesToMatch.add(i);
		}
		return areEqual(s1, s2, indicesToMatch);
	}

	/*
	 * written to check for the same automata state for a non seq team mdp assuming
	 * the automata states start at 0 and end at length -mdpIndex
	 */
	public static boolean statesHaveTheSameAutomataProgress(State s1, State s2, int mdpIndex) {
		// hardcoding this here
		Vector<Integer> indicesToMatch = new Vector<Integer>();
		int numVar = s1.varValues.length - mdpIndex;

		// we know that the ones at the end are robot num and mdp state so we can skip
		// those

		for (int i = 0; i < numVar; i++) {
			indicesToMatch.add(i);
		}
		return areEqual(s1, s2, indicesToMatch);
	}

	/**
	 * Returns a mask thing, where indices that have the same values are 0 and ones
	 * that have different values are 1 so then you can do what you want if they
	 * aren't the same length you get the smaller of the two duh
	 * 
	 * @param s1
	 * @param s2
	 * @return
	 */
	public static int[] XORIntegers(Object[] s1, Object[] s2) {
		int maxLen = Math.max(s1.length, s2.length);
		int[] mask = new int[maxLen];
		for (int i = 0; i < maxLen; i++) {
			if ((int) s1[i] == (int) s2[i])
				mask[i] = 0;
			else
				mask[i] = 1;
		}
		return mask;

	}

	public static Object[] XORStates(State s1, State s2, Object[] refState
	/* a state with the initial states of all DA */) {
		// so we need to exclude the robot and the mdp states from this
		Object[] s1v = s1.varValues;
		Object[] s2v = s2.varValues;
		Object[] resS = new Object[s1v.length - (1 + numMdpVars)]; // hard coding this
		for (int i = robotVar + 1; i < mdpVarStart; i++) {
			if (s1v[i] != s2v[i])
				resS[i - 1] = s2v[i];
			else
				resS[i - 1] = refState[i];
		}
		return resS;
	}

}