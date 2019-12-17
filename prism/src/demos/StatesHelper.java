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
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;

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

public class StatesHelper
{
	static int robotVar = 0;
	static int mdpVarStart;
	public static int failState = -1;
	public static int BADVALUE = -2;
	public static String savePlace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	public static String folder;
	public static int numMdpVars;
	public static PrismLog mainLog;

	public static boolean areEqual(State s1, State s2, Vector<Integer> indicesToMatch)
	{
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

	//new
	public static Object[] getDAStatesFromState(State state, VarList varlist, int numTasks)
	{
		// so return everything from after robotVar upto mdpVarStart
		Object[] stateObj = state.varValues;
		Object[] toRet = new Object[numTasks];
		int task = 0;
		for (int i = 0; i < varlist.getNumVars(); i++) {
			String name = varlist.getName(i);
			if (name.contains("da")) {
				toRet[task] = stateObj[i];
				task++;
			}
		}

		return toRet;
	}

	public static Object[] getSharedStatesFromState(State state, VarList varlist, ArrayList<String> sharedVars)
	{
		Object[] stateObj = state.varValues;
		Object[] toret = new Object[sharedVars.size()];
		int index = -1;
		int numnull = 0;

		//		for (int i = 0; i<varlist.getNumVars(); i++)
		//		{
		//			String name = varlist.getName(i); 
		//			if(sharedVars.contains(name))
		//			{
		//				index++;
		//				toret[index] = stateObj[i]; 
		//
		//			}
		//		}
		for (int i = 0; i < sharedVars.size(); i++) {
			index = varlist.getIndex(sharedVars.get(i));
			if (index != -1)
				toret[i] = stateObj[index];
			else
				numnull++;
		}
		//		if (index == -1)
		//			numnull = sharedVars.size();
		//		
		if (numnull == sharedVars.size())
			toret = null;
		return toret;

	}

	public static boolean stateIsDeadend(MDP mdp, int s) throws PrismException
	{
		boolean isdeadend = false;
		int numChoices = mdp.getNumChoices(s);
		if (numChoices == 0) {
			isdeadend = true;
		} else if (numChoices == 1) {
			int numTransitions = mdp.getNumTransitions(s, 0);
			if (numTransitions == 1) {
				//is it a self loop ?? 
				Iterator<Entry<Integer, Double>> choiceIter = mdp.getTransitionsIterator(s, 0);

				while (choiceIter.hasNext()) {
					Entry<Integer, Double> nextS = choiceIter.next();
					if (nextS.getKey() == s) {
						isdeadend = true;

					} else {
						if (isdeadend)
							throw new PrismException(
									"We thought we detected a deadend but apparently its a bit mysterious " + mdp.getStatesList().get(s).toString());
					}

				}
			}
		}
		return isdeadend;
	}

	public static int getExactlyTheSameState(Object s1v[], List<State> states)
	{
		int res = BADVALUE;
		for (int s = 0; s < states.size(); s++) {
			if (statesAreEqual(s1v, states.get(s))) {
				res = s;
				break;
			}
		}
		return res;
	}

	public static int getIndexValueFromState(State s1, int index)
	{
		return (int) s1.varValues[index];
	}

	//in use
	public static Object[] getMDPStateFromState(State s1, VarList varlist, ArrayList<String> notSharedVarsList)
	{
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

	public static int getRobotNumberFromSeqTeamMDPState(State s1)
	{
		Object[] stateVar = s1.varValues;
		return (int) stateVar[robotVar];

	}

	public static String getSaveplace()
	{
		return savePlace;
	}

	public static boolean isFailState(State s1)
	{
		int ind = mdpVarStart;
		if (getIndexValueFromState(s1, ind) == failState)
			return true;
		else
			return false;
	}

	public static String getLocation()
	{
		String temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replaceAll(".tra", "");
		String location = getSaveplace();
		temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
		location += temp;
		return location;
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
	public static void saveMDP(MDP mdp, BitSet statesToMark, String anotherfolder, String name, boolean saveinsaveplace)
	{
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replace(".tra", "");
		String location = temp;
		if (saveinsaveplace) {
			location = getSaveplace();
			temp = temp.substring(temp.lastIndexOf("/") + 1, temp.length());
			location += temp;
		}
		name = name.replace(" ", "_");
		mainLog.println("Saving MDP to " + location + "_" + name + ".dot");
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
	public static void saveMDPstatra(MDP mdp, String anotherfolder, String name, boolean saveinsaveplace)
	{
		String temp = anotherfolder;
		if (temp == "")
			temp = folder;
		temp = temp.replace("adv", "");
		temp = temp.replace(".tra", "");
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
			if(mdp.getVarList() != null)
			mdp.exportStates(Prism.EXPORT_PLAIN, mdp.getVarList(), out);
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		out.close();

	}

	public static void setFolder(String folderVal)
	{
		folder = folderVal;
	}

	public static void setMDPVar(int var)
	{
		mdpVarStart = var;
	}

	public static void setNumMDPVars(int mdpvars)
	{
		numMdpVars = mdpvars;
	}

	public static void setSavePlace(String saveplace)
	{
		savePlace = saveplace;
	}

	public static boolean statesAreEqual(Object s1v[], State s2)
	{
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
	public static boolean statesHaveTheSameAutomataProgress(State s1, State s2)
	{
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

	/**
	 * Returns a mask thing, where indices that have the same values are 0 and ones
	 * that have different values are 1 so then you can do what you want if they
	 * aren't the same length you get the smaller of the two duh
	 * 
	 * @param s1
	 * @param s2
	 * @return
	 */
	public static int[] XORIntegers(Object[] s1, Object[] s2)
	{
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

	public static int[] XORIntegers(State firstStateState, State lastStateState)
	{
		// TODO Auto-generated method stub
		return XORIntegers(firstStateState.varValues, lastStateState.varValues);

	}

}