//==============================================================================
//	
//	Copyright (c) 2002-
//	Authors:
//	* Dave Parker <david.parker@comlab.ox.ac.uk> (University of Oxford)
//	
//------------------------------------------------------------------------------
//	
//	This file is part of PRISM.
//	
//	PRISM is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.
//	
//	PRISM is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//	
//	You should have received a copy of the GNU General Public License
//	along with PRISM; if not, write to the Free Software Foundation,
//	Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//	
//==============================================================================

package explicit;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.AbstractMap;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Stack;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import common.IterableBitSet;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MCRewards;
import explicit.rewards.MCRewardsFromMDPRewards;
import explicit.rewards.MDPRewards;
import explicit.rewards.MDPRewardsSimple;
import explicit.rewards.Rewards;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.DeclarationIntUnbounded;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.RewardStruct;
import parser.type.TypeInt;
import prism.Prism;
import prism.PrismComponent;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.PrismUtils;
import strat.MDStrategy;
import strat.MDStrategyArray;
import strat.Strategy;


//testing my class 
import explicit.probPolicyTest;

/**
 * Explicit-state model checker for Markov decision processes (MDPs).
 */
public class MDPModelChecker extends ProbModelChecker {
	/**
	 * Create a new MDPModelChecker, inherit basic state from parent (unless null).
	 */
	boolean run_tests = false;
	String saveplace = System.getProperty("user.dir") + "/tests/decomp_tests/temp/";
	boolean printHighlights = true; 
	boolean printDetails = false; 

	public MDPModelChecker(PrismComponent parent) throws PrismException {
		super(parent);
	}

	// Model checking functions

	@Override
	protected StateValues checkExpressionFunc(Model model, ExpressionFunc expr, BitSet statesOfInterest)
			throws PrismException {
		switch (expr.getNameCode()) {
		case ExpressionFunc.PARTIAL: {
			// checkPartialSat(model, expr, statesOfInterest);
			// mainLog.println("------------------------SkippingNow----------------------------------------");
			return ltlDecomp(model, expr, statesOfInterest);
		}
		default:
			return super.checkExpressionFunc(model, expr, statesOfInterest);
		}
	}

	private BitSet essentialStates(BitSet accs, MDP prod) {
		// check if an accepting state is connected to a non accepting state
		// basically I want a bitset of all the edges and and it with !(accepting
		// states), if its not null I know
		int numstates = prod.getNumStates();

		BitSet accsCopy = (BitSet) accs.clone();
		BitSet essentialaccs = new BitSet(numstates);
		int setbit = -1;
		for (int s = 0; s < numstates; s++) {
			if (!accs.get(s)) // if not an accepting state
			{
				// if(prod.someSuccessorsInSet(s, accs)) //check if any accepting state is in
				// the succ set
				setbit = accsCopy.nextSetBit(0);

				while (setbit != -1) {
					if (prod.isSuccessor(s, setbit)) {
						essentialaccs.set(setbit);
						accsCopy.clear(setbit);
					}

					setbit = accsCopy.nextSetBit(setbit + 1);
				}
			}
		}
		return essentialaccs;

	}

	protected Object[] constructMdpDaProd(int numrobots, int numOp, MDP model, LTLModelChecker mcLtls[],
			DA<BitSet, ? extends AcceptanceOmega> das[], Expression ltls[], int robot_model) throws PrismException {
		// int robot_model = 1; //0 = two_room_robot_f, 1 = two_room_robot_f_extended
		LTLModelChecker.LTLProduct<MDP> product = null;
		MDP productMdp;
		Vector<BitSet> labelBSs[];
		String saveplace = System.getProperty("user.dir") + "/tests/decomp_tests/temp/";
		BitSet initstates[] = new BitSet[numrobots];
		BitSet safeltl_accstates = new BitSet(numOp); // these tell us the bad states
		//
		int numres = 5;
		Object res[] = new Object[numres];
		labelBSs = new Vector[numOp];
		BitSet accs[] = new BitSet[numOp];
		BitSet essentialaccs[] = new BitSet[numOp];

		// build DFA
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };

		// for all the automata save the accepting states

		// build product
		// time to build weighted skipping product
		long time = System.currentTimeMillis();

		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		for (int r = 0; r < numrobots; r++) {
			// for(int da = 0; da<numOp; da++)
			// {
			initstates[r] = new BitSet(numStates);
			// }
		}
		for (int i = 0; i < numStates; i++) {
			// bsInit.set(i, model.isInitialState(i));
			initstates[0].set(i, model.isInitialState(i));
			// lets set all the states
			bsInit.set(i);
		}
		for (int r = 1; r < numrobots; r++) {
			if (robot_model == 0)
				initstates[r].set(2);// hardcoding this realy
			else if (robot_model == 1) {
				if (r == 1)
					initstates[r].set(2);
				if (r == 2)
					initstates[r].set(4);
			}
		}

		model.exportToDotFile(saveplace + "mdp.dot");
		BitSet allAccStates[] = new BitSet[numOp]; // so there are bad states here too that are being converted always
		BitSet essentialStates[] = new BitSet[numOp]; // the essential states basically states that are connected to
														// atleast one non accepting state of the same dfa
		productMdp = (MDP) model;
		for (int danum = 0; danum < numOp; danum++) {

			labelBSs[danum] = new Vector<BitSet>();
			// check if expression is cosafe
			boolean iscosafe = Expression.isCoSafeLTLSyntactic(ltls[danum], true);

			if (!iscosafe) {
				if(printHighlights)
				mainLog.println(ltls[danum].toString() + " is not cosafe, converting it to cosafe");
				ltls[danum] = Expression.Not(ltls[danum]);
				safeltl_accstates.set(danum);
				if(printHighlights)
				mainLog.println("--->" + ltls[danum].toString());
			}
			das[danum] = mcLtls[danum].constructDAForLTLFormula(this, productMdp, ltls[danum], labelBSs[danum],
					allowedAcceptance);

			try {
				File fn = new File(saveplace + "da" + danum + "_"
						+ ltls[danum].toString().replace(" ", "").replace("\"", "") + ".dot");
				if(printHighlights)
				mainLog.println("Saving to file " + fn.getName());
				PrintStream out;
				out = new PrintStream(new FileOutputStream(fn));
				das[danum].printDot(out);
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			// TODO check if I need so many model checker instances

			if (!(das[danum].getAcceptance() instanceof AcceptanceReach)) { // a safety condition is not really a
																			// reachability problem so if you have a
																			// safety condition
				if(printHighlights)
					mainLog.println("\nAutomaton is not a DFA. .");
				if (!(das[danum].getAcceptance() instanceof AcceptanceRabin)) // should be an or thing but fix it later
				{
					if(printHighlights)
					mainLog.println("\nAutomaton is not a DRA either..Breaking");
					// Dummy return vector
					return null;
					// return new StateValues(TypeInt.getInstance(), model);
				} else {
					safeltl_accstates.set(danum);

					// do nothing. find out what to do here
					if(printHighlights)
					mainLog.println("Need to get accepting states for this rabin automata");
					continue;
				}
			} else {
				accs[danum] = ((AcceptanceReach) das[danum].getAcceptance()).getGoalStates();
				if(printDetails)
				mainLog.println(
						"Accepting States bit set for " + ltls[danum].toString() + " : " + accs[danum].toString());
				if(printHighlights)
				mainLog.println("Labels Bitset " + labelBSs[danum].toString());
			}
			if(printHighlights)
			mainLog.println("Before product " + danum + " model states: " + productMdp.getNumStates() + " da states "
					+ das[danum].size());

			product = mcLtls[danum].constructProductModel(das[danum], productMdp, labelBSs[danum], null);

			numStates = product.getProductModel().getNumStates();
			if(printHighlights)
			mainLog.println("After product " + danum + " product states: " + numStates);
			if(printHighlights)
			mainLog.println("Product" + danum + " accepting states: "
					+ ((AcceptanceReach) product.getAcceptance()).getGoalStates().toString());
			allAccStates[danum] = ((AcceptanceReach) product.getAcceptance()).getGoalStates();

			essentialaccs[danum] = essentialStates(allAccStates[danum], product.getProductModel());

			// convert to new model if possible
			int oldstate = -1;
			BitSet tempstates, tempstates2;
			numStates = product.getProductModel().getNumStates();
			// need to convert the init state for the first one too
			if (danum == 0) {
				// for(int da = 0; da<numOp; da++) {
				for (int r = 0; r < numrobots; r++) {
					BitSet tempinitstate = new BitSet(numStates);
					for (int s = 0; s < numStates; s++) {
						oldstate = product.getModelState(s);
						if (initstates[r].get(oldstate)) {
							tempinitstate.set(s);
						}
					}
					initstates[r] = (BitSet) tempinitstate.clone();
				}
			}
			for (int tempnum = 0; tempnum < danum; tempnum++) {
				// assumption - since each model is a product of the previous product and the da
				// the old states are states of the previous product
				// we get those and save them as new states of this product. any old state that
				// has the same value will be an accepting state
				tempstates = new BitSet(numStates);
				tempstates2 = new BitSet(numStates);
				BitSet tempinitstates[] = new BitSet[numrobots];
				for (int r = 0; r < numrobots; r++)
					tempinitstates[r] = new BitSet(numStates);
				for (int s = 0; s < numStates; s++) {
					oldstate = product.getModelState(s);
					if (allAccStates[tempnum].get(oldstate)) {
						tempstates.set(s);
					}
					if (essentialaccs[tempnum].get(oldstate)) {
						tempstates2.set(s);
					}
					for (int r = 0; r < numrobots; r++) {
						if (initstates[r].get(oldstate))
							tempinitstates[r].set(s);
					}
				}
				allAccStates[tempnum] = (BitSet) tempstates.clone();
				essentialaccs[tempnum] = (BitSet) tempstates2.clone();
				// for each robot update the initial state too
				for (int r = 0; r < numrobots; r++)
					initstates[r] = (BitSet) tempinitstates[r].clone();

			}
			// printing all acc states saved
			for (int tempnum = 0; tempnum <= danum; tempnum++) {
				if(printHighlights)
				mainLog.println(tempnum + ": " + allAccStates[tempnum].toString());
				if(printHighlights)
				mainLog.println(tempnum + ": " + essentialaccs[tempnum].toString());
			}
			product.getProductModel().exportToDotFile(saveplace + "p" + danum + ".dot");
			product.getProductModel().exportToPrismExplicitTra(saveplace + "p" + danum + ".tra");
			PrismFileLog out = new PrismFileLog(saveplace + "p" + danum + ".sta");
			VarList newVarList = (VarList) modulesFile.createVarList().clone();
			String daVar = "_da";
			while (newVarList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}
			newVarList.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
			product.getProductModel().exportStates(Prism.EXPORT_PLAIN, newVarList, out);
			out.close();
			productMdp = product.getProductModel();

		}
		if(printHighlights)
		mainLog.println("DFAs with bad states = " + safeltl_accstates.toString());
		// filter out the bad states from the accepting states
		for (int tempnum = 0; tempnum < numOp; tempnum++) {
			if (safeltl_accstates.get(tempnum)) {
				// then check for all others and just unset them
				// maybe a bitwise operation
				for (int tempnum2 = 0; tempnum2 < numOp; tempnum2++) {
					if (tempnum2 != tempnum) {
						// xor then and
						BitSet temp = (BitSet) allAccStates[tempnum2].clone();
						temp.xor(allAccStates[tempnum]);
						temp.and(allAccStates[tempnum2]);
						allAccStates[tempnum2] = (BitSet) temp.clone();
					}
				}
			}
		}
		// TODO: JUST DOING THIS TO CHECK SOMETHING
		// justinitstates[0].set(18,false);
		// justinitstates[1].set(14,false);

		// printing all acc states saved
		for (int tempnum = 0; tempnum < numOp; tempnum++) {
			if(printHighlights)
			mainLog.println(tempnum + ": " + allAccStates[tempnum].toString());
		}
		for (int r = 0; r < numrobots; r++) {
			// for(int da=0; da<numOp; da++) {
			if(printHighlights)
			mainLog.println("r" + r + " init states: " + initstates[r].toString());
			// }
		}

		res[0] = product;
		res[1] = allAccStates;
		res[2] = initstates;
		res[3] = safeltl_accstates;
		res[4] = essentialaccs;// justinitstates;
		return res;

	}

	private List<Integer> findChangedStateIndices(State x1, State x2, int nv) {
		List<Integer> res = new ArrayList<Integer>();
		Object x1v[] = x1.varValues;
		Object x2v[] = x2.varValues;
		for (int v = 1; v < nv - 1; v++)
			if (((int) x1v[v]) != ((int) x2v[v])) {
				res.add(v);
			}

		return res;
	}

	private ArrayList<Entry<Integer, Integer>> findChangedStateIndices(State x1, State x2, int nv, boolean retValsx2) {

		ArrayList<Entry<Integer, Integer>> res = new ArrayList<Entry<Integer, Integer>>();

		// List<Integer> res = new ArrayList<Integer>();
		// List<Integer> res2 = new ArrayList<Integer>();

		Object x1v[] = x1.varValues;
		Object x2v[] = x2.varValues;
		for (int v = 1; v < nv - 1; v++)
			if (((int) x1v[v]) != ((int) x2v[v])) {
				int val = (int) x1v[v];
				if (retValsx2)
					val = (int) x2v[v];
				Entry<Integer, Integer> tmp = new AbstractMap.SimpleEntry(v, val);
				res.add(tmp);

			}

		return res;
	}

	// create a new state replacing all the values in changedIndices with val
	private Object[] getNewState(State x1, List<Integer> changedIndices, int nv, List<Integer> val) {

		Object x1v[] = x1.varValues;
		Object x1vp[] = x1v.clone();
		for (int v = 0; v < changedIndices.size(); v++) {
			x1vp[changedIndices.get(v)] = val.get(v);
		}
		return x1vp;
	}

	private Object[] getNewState(State x1, List<Entry<Integer, Integer>> changedIndices) {
		Object x1v[] = x1.varValues;
		Object x1vp[] = x1v.clone();
		for (int v = 0; v < changedIndices.size(); v++) {
			int ind = changedIndices.get(v).getKey();
			int val = changedIndices.get(v).getValue();
			x1vp[ind] = val;
		}
		return x1vp;

	}

	private int findSameState(Object x1v[], List<State> states, int nv) {
		int res = -1;
		// go over all states and find the one that equals the new one
		for (int s = 0; s < states.size(); s++) {
			if (sameDAVals(x1v, states.get(s), nv)) {
				res = s;
				break;
			}
		}
		return res;
	}

	private int findSameState(Object x1v[], List<State> states) {
		int res = -1;
		int nv = x1v.length;
		// go over all states and find the one that equals the new one
		for (int s = 0; s < states.size(); s++) {
			if (sameDAVals(x1v, states.get(s), nv)) {
				res = s;
				break;
			}
		}
		return res;
	}

	private int findSameStateExcept(State x1, List<Integer> changedIndices, List<State> states, int nv,
			List<Integer> val) {
		int res = -1;
		// go over all states and find the one that equals the new one
		Object x1v[] = getNewState(x1, changedIndices, nv, val);
		for (int s = 0; s < states.size(); s++) {
			if (sameDAVals(x1v, states.get(s), nv)) {
				res = s;
				break;
			}
		}
		return res;
	}

	private int getMergedState(State x1, State x2, List<State> states, int nv) {
		// keep the last and first bits of x2 and the rest of x1
		Object x1v[] = x1.varValues.clone();
		Object x2v[] = x2.varValues;
		x1v[0] = x2v[0];
		x1v[nv - 1] = x2v[nv - 1];
		int res = -1;
		for (int s = 0; s < states.size(); s++) {
			if (sameDAVals(x1v, states.get(s), nv)) {
				res = s;
				break;
			}
		}
		return res;

	}

	private int findSameStateExcept(State x1, List<Integer> changedIndices, List<State> states, int nv, int val_) {
		List<Integer> val = new ArrayList<Integer>();
		for (int i = 0; i < changedIndices.size(); i++)
			val.add(val_);
		return findSameStateExcept(x1, changedIndices, states, nv, val);
	}

	private int findSameStateExcept(State x1, List<Integer> changedIndices, List<State> states, int nv) {

		return findSameStateExcept(x1, changedIndices, states, nv, 0);
	}

	private int newInitState(MDPSimple sumprod, int cs2[], double probs[]) {
		int cs1 = sumprod.getNumStates();
		sumprod.addState();

		Distribution prodDistr = new Distribution();
		if(printDetails)
		mainLog.print(" NI:" + cs1 + "->");
		for (int i = 0; i < cs2.length; i++) {
			prodDistr.add(cs2[i], probs[i]);
			if(printDetails)
			mainLog.print(cs2[i] + ",");
		}

		sumprod.addActionLabelledChoice(cs1, prodDistr, "dummy_state");
		return cs1;

	}

	private void addlink(MDPSimple sumprod, int cs1, int cs2[], double probs[], int r, int ps) {
		String extras = "e";
		Distribution prodDistr = new Distribution();
		// linking in r-1 to r
		if(printDetails)
		mainLog.print("\t SER " + cs1 + " -> ");
		for (int i = 0; i < cs2.length; i++) {
			prodDistr.add(cs2[i], probs[i]);
			if(printDetails)
			mainLog.print(cs2[i] + ",");
		}
		sumprod.addActionLabelledChoice(cs1, prodDistr, "switch_" + extras + "r" + (r - 1) + "_r" + r);

	}

	private void addlink(MDPSimple sumprod, int cs1, int cs2[], double probs[], int r) {
		String extras = "e";
		Distribution prodDistr = new Distribution();
		// linking in r-1 to r
		if(printDetails)
		mainLog.print("\t SER " + cs1 + " -> ");
		for (int i = 0; i < cs2.length; i++) {
			prodDistr.add(cs2[i], probs[i]);
			if(printDetails)
			mainLog.print(cs2[i] + ",");
		}
		sumprod.addActionLabelledChoice(cs1, prodDistr, "switch_" + extras + "r" + (r - 1) + "_r" + r);

	}
	
	private void addlink(MDPSimple sumprod, int cs1, int cs2, int r) {
		int cs2arr[] = new int[1]; 
		cs2arr[0] = cs2; 
		double probarr[] = new double[1]; 
		probarr[0] = 1.0; 
 		addlink(sumprod,cs1,cs2arr,probarr,r);
	}

	private boolean isfailState(State x1, int nv) {
		Object x1v[] = x1.varValues;
		// TODO: I know this in this case find a better way to do this!!!
		boolean failstate = ((int) x1v[nv - 1] == 9);
		return failstate;

	}

	private void printState(State x1, int nv) {

		Object x1v[] = x1.varValues;
		for (int i = 0; i < nv; i++)
			mainLog.print((int) x1v[i]);
	}

	// check whether two states are in the same da state
	// exclude last var as being the state variable
	private int getIndValFromState(State x1, int ind) {
		Object arr[] = x1.varValues;
		int res = -1;
		if (ind < arr.length)
			res = (int) arr[ind];
		return res;

	}

	// check whether two states are in the same da state

	private boolean sameDAVals(Object x1v[], State x2, int numVar) {
		boolean isSame = true;
		// mainLog.print("Comparing ");
		// printState(x1,numVar);
		// mainLog.print(" : ");
		// printState(x2,numVar);

		Object x2v[] = x2.varValues;
		for (int v = 0; v < numVar; v++)
			if (((int) x1v[v]) != ((int) x2v[v])) {
				isSame = false;
				break;
			}
		// mainLog.print(":"+isSame+"\n");
		return isSame;
	}

	// check whether two states are in the same da state
	// exclude last var as being the state variable
	private boolean sameDAVals(Object[] x1v2, State x2, int numVar, int stval) {
		boolean isSame = true;
		// mainLog.print("Comparing ");
		// printState(x1,numVar);
		// mainLog.print(" : ");
		// printState(x2,numVar);

		Object x1v[] = x1v2.clone();
		Object x2v[] = x2.varValues;
		for (int v = stval; v < numVar - 1; v++)
			if (((int) x1v[v]) != ((int) x2v[v])) {
				isSame = false;
				break;
			}
		// mainLog.print(":"+isSame+"\n");
		return isSame;
	}

	// check whether two states are in the same da state
	// exclude last var as being the state variable
	private boolean sameDAVals(State x1, State x2, int numVar) {
		boolean isSame = true;
		// mainLog.print("Comparing ");
		// printState(x1,numVar);
		// mainLog.print(" : ");
		// printState(x2,numVar);

		Object x1v[] = x1.varValues;
		Object x2v[] = x2.varValues;
		for (int v = 1; v < numVar - 1; v++)
			if (((int) x1v[v]) != ((int) x2v[v])) {
				isSame = false;
				break;
			}
		// mainLog.print(":"+isSame+"\n");
		return isSame;
	}

	// check whether two states have the same mpd value
	// we know the last bit of the state has the same mdp value
	// if considerRobot = true, it'll care that the first and last are the same
	// otherwise just the last.
	private boolean sameMDPVals(State x1, State x2, int numVar, boolean considerRobot) {
		boolean isSame = true;

		Object x1v[] = x1.varValues;
		Object x2v[] = x2.varValues;
		int rval = 0;
		if (considerRobot) {
			if (!(x1v[rval] == x2v[rval]))
				isSame = false;
		}
		if (!(x1v[numVar - 1] == x2v[numVar - 1]))
			isSame = false;

		return isSame;
	}

	// check whether two states have the same mpd value
	// we know the last bit of the state has the same mdp value
	// if considerRobot = true, it'll care that the first and last are the same
	// otherwise just the last.
	private boolean sameMDPDAVals(State x1, State x2, int numVar, boolean considerRobot, List<Integer> ci,
			boolean considerx2) {
		boolean isSame = true;

		Object xv[][] = new Object[2][];
		xv[0] = x1.varValues;
		xv[1] = x2.varValues;

		int toconsider = 0;
		if (considerx2) {
			toconsider = 1;
		}
		int rval = 0;
		if (considerRobot) {
			if (!(xv[0][rval] == xv[1][rval]))
				isSame = false;
		}
		if (considerx2) {
			for (int da = 0; da < ci.size(); da++) {
				if (!(xv[toconsider][ci.get(da)] == xv[0][ci.get(da)])) {
					isSame = false;
					break;
				}
			}
		}
		if (!(xv[0][numVar - 1] == xv[1][numVar - 1]))
			isSame = false;

		return isSame;
	}

	private boolean isRealInitState(State x1, int dainits[], int numVar) {
		boolean res = true;
		// printState(x1,numVar);
		Object x1v[] = x1.varValues;
		for (int v = 0; v < numVar - 1; v++) {
			if (((int) x1v[v]) != dainits[v]) {
				res = false;
				break;
			}
		}
		return res;
	}

	protected Object[] constructSumMDP(int numOp, int numrobots, BitSet allAccStates[], BitSet initstates[],
			MDP productMdp, BitSet safeltl, BitSet essentialAccStates[]) throws PrismException {

		int numres = 5;
		Object res[] = new Object[numres];
		int numStates = productMdp.getNumStates();
		String saveplace = System.getProperty("user.dir") + "/tests/decomp_tests/temp/";

		BitSet finalAccStates = new BitSet(numStates);
		BitSet badStates = new BitSet(numStates);
		BitSet allSwitchStates = new BitSet(numStates);
		BitSet rinitstates[] = new BitSet[numrobots];

		finalAccStates.set(0, numStates);
		badStates.set(0, numStates, false);

		// finalAccStates are states that are in all the acc states
		// switch states are the ones that are not
		// badStates are the ones in safeltl

		for (int i = 0; i < numOp; i++) {
			if (!safeltl.get(i)) {
				allSwitchStates.or(essentialAccStates[i]);
				finalAccStates.and(allAccStates[i]);
			} else {
				badStates.or(allAccStates[i]);
			}
		}
		// make sure you get rid of the badStates
		allSwitchStates.andNot(badStates);
		finalAccStates.andNot(badStates);
		allSwitchStates.andNot(finalAccStates);
		// init states //get rid of all the bad states
		for (int i = 0; i < numrobots; i++)
			initstates[i].andNot(badStates);
		// now lets get the varlist

		int numVars = productMdp.getVarList().getNumVars();
		List<State> states = productMdp.getVarList().getAllStates();
		List<State> pstates = productMdp.getStatesList();

		// hardcoding this but add it later //you have to get these when you do the
		// product
		int dainits[] = new int[numVars];
		for (int i = 0; i < numVars; i++)
			dainits[i] = 0;

		// printing stuff
		if(printHighlights)
		mainLog.println("Bad States:" + badStates.toString());
		if(printHighlights)
		mainLog.println("Final States:" + finalAccStates.toString());
		if(printHighlights)
		mainLog.println("Switch States:" + allSwitchStates.toString());
		for (int i = 0; i < numrobots; i++) {
			if(printHighlights)
			mainLog.println("Init States" + i + " :" + initstates[i].toString());
			rinitstates[i] = new BitSet(numStates);
			int numis = initstates[i].cardinality();
			int setbit = initstates[i].nextSetBit(0);
			for (int j = 0; j < numis; j++) {
				if (setbit != -1) {
					boolean isinitstate = isRealInitState(pstates.get(setbit), dainits, numVars);
					if(printHighlights)
					mainLog.println(":" + setbit + ":" + isinitstate);
					if (isinitstate)
						rinitstates[i].set(setbit);
					setbit = initstates[i].nextSetBit(setbit + 1);
				}

			}
		}

		// this is just over designing but probably making my life easier
		// maybe in the future we have different agent models so everything will change
		// that will not be okay cuz this is just a workaround
		// it should be da0xda1 and so on and the switch states being the final states
		// for each da
		// maybe I will find a way to do this. make all this mehnat raigan

		BitSet switches[] = new BitSet[numrobots];
		BitSet finals[] = new BitSet[numrobots];
		BitSet badstates[] = new BitSet[numrobots];

		MDP[] productRobots = new MDP[numrobots];
		// now lets make a copy of this mdp
		for (int r = 0; r < numrobots; r++) {
			productRobots[r] = productMdp; // i dont know if this is a deep copy
			switches[r] = (BitSet) allSwitchStates.clone();
			finals[r] = (BitSet) finalAccStates.clone();
			badstates[r] = (BitSet) badStates.clone();
		}

		VarList newVarList = null;

		if (productMdp.getVarList() != null) {
			VarList varList = productMdp.getVarList();
			// Create a (new, unique) name for the variable that will represent
			// DA states
			String daVar = "_da";
			while (varList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}

			newVarList = (VarList) varList.clone();
			// NB: if DA only has one state, we add an extra dummy state
			DeclarationInt daint = new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(numrobots - 1, 1)));
			Declaration decl = new Declaration(daVar, daint);
			newVarList.addVar(0, decl, 1, productMdp.getConstantValues());
		}

		// we need to make a new product which will basically be a sum of the old ones
		// so maxstates = numStates*numrobots
		// for each product mdp add transitions to switch states
		MDPSimple sumprod = new MDPSimple();
		MDPSimple sumprod_noswitches; // = new MDPSimple();
		sumprod.setVarList(newVarList);
		ArrayList<State> prodStatesList = null, daStatesList = null;

		if (productRobots[0].getStatesList() != null) {
			prodStatesList = new ArrayList<State>();
			daStatesList = new ArrayList<State>(numrobots);
			for (int i = 0; i < numrobots; i++) {
				daStatesList.add(new State(1).setValue(0, i));
			}
		}

		// newstate = r*NUMSTATES + s
		// lets make copies
		// TODO: LOOK AT THE STUFF FOR NEWSTATE. IT KIND OF MEANS THAT EACH ROBOT HAS TO
		// HAVE THE SAME AMOUNT OF STATES. WHICH WILL NOT ALWAYS BE THE CASE
		// WE NEED TO COME UP WITH BETTER INDEXING. REMEMBER. okay cool :P
		// should be using a map maybe need to check this
		int prodNumStates = numrobots * numStates;
		int map[] = new int[prodNumStates];
		BitSet newAccStates = new BitSet(prodNumStates);
		BitSet newbadStates = new BitSet(prodNumStates);
		Arrays.fill(map, -1);
		//
		int expStates = 0;
		int expChoices = 0;
		for (int r = 0; r < numrobots; r++) {
			// go over the entire product mdp for robot r
			MDP pr = productRobots[r];
			numStates = pr.getNumStates();
			if(printDetails)
			mainLog.println("Robot " + r + " states " + numStates + " choices " + pr.getNumChoices());
			expStates += numStates;
			expChoices += pr.getNumChoices();
			int numchoices = -1;
			int s_ = -1;
			for (int s = 0; s < numStates; s++) {
				sumprod.addState();
				int cs = (r * numStates) + s;

				if (finals[r].get(s)) {
					newAccStates.set(cs);
				}
				if (badstates[r].get(s)) {
					newbadStates.set(cs);
				}
				map[cs] = sumprod.getNumStates() - 1;

				// add as init state ? //only add the first robot's init state as an init state
				// otherwise no
				if (r == 0 && rinitstates[r].get(s)) {
					sumprod.addInitialState(map[cs]);
					// sumprod.isInitialState(cs);
				}
				numchoices = pr.getNumChoices(s);
				if (prodStatesList != null) {
					// Store state information for the product
					prodStatesList.add(new State(daStatesList.get(r), pr.getStatesList().get(s)));
				}
				Iterator<Map.Entry<Integer, Double>> iter;

				// if (!badStatesF.get(s)) {
				for (int j = 0; j < numchoices; j++) {
					iter = pr.getTransitionsIterator(s, j);

					int ns = 0;
					Distribution prodDistr = new Distribution();
					while (iter.hasNext()) {
						Map.Entry<Integer, Double> e = iter.next();
						s_ = e.getKey();
						double prob = e.getValue();
						ns = (r * numStates) + s_;
						prodDistr.add(ns, prob);
					}

					sumprod.addActionLabelledChoice(map[cs], prodDistr, pr.getAction(s, j) + "_r" + r);
					// mainLog.println("Adding (" + s + "," + s_ + ")" + cs + " " +
					// prodDistr.toString() + " "
					// + pr.getAction(s, j) + "_r" + r);
				}
				// }
				// else
				// {
				// //if its a bad state add a self loop and not other transitions
				// //dont want to add the state if no other states go to it though, maybe I
				// should count or something
				// Distribution prodDistr = new Distribution();
				// prodDistr.add(cs, 1.0); //self loop
				// sumprod.addActionLabelledChoice(cs, prodDistr, "bad_state"+ "_r" + r);
				// }

			}
		}

		sumprod_noswitches = new MDPSimple(sumprod);// .copyFrom(sumprod);
		// need to assign this a states list

		// now we're going to add the switch transitions
		// just link up all the switch states in all the mdps
		// TODO fix this... link switch states based on the da (use daswitchstates)
		// so change all of this ? // TODO!!!!
		int totswitches = 0;
		double switchprob = 1.0;
		if(printDetails)
		mainLog.println("Switch States now");
		int numSwitchStates = allSwitchStates.cardinality();
		int numInitStates = 0;
		int setbit_r = allSwitchStates.nextSetBit(0);
		int setbit_r1 = 0;
		int cs2, cs1;
		Distribution prodDistr = null;
		for (int r = 1; r < numrobots; r++) {
			numInitStates = initstates[r].cardinality();

			setbit_r = allSwitchStates.nextSetBit(0);
			while (setbit_r != -1) {
				setbit_r1 = initstates[r].nextSetBit(0);
				while (setbit_r1 != -1) {
					if (sameDAVals(pstates.get(setbit_r1), pstates.get(setbit_r), numVars)) {
						prodDistr = new Distribution();
						cs2 = r * numStates + setbit_r1;
						cs1 = (r - 1) * numStates + setbit_r;
						prodDistr.add(map[cs2], switchprob);
						sumprod.addActionLabelledChoice(map[cs1], prodDistr, "switch_" + (r - 1) + "_" + r);
						if(printHighlights)
						mainLog.println("SS:" + setbit_r + "->" + setbit_r1 + ":" + map[cs1] + "->" + map[cs2]);
						totswitches++;
					}
					setbit_r1 = initstates[r].nextSetBit(setbit_r1 + 1);
				}

				setbit_r = allSwitchStates.nextSetBit(setbit_r + 1);
				// if (setbit_r == 34)
				// mainLog.println("Check Here");
			}
		}
		if (prodStatesList != null) {
			sumprod.setStatesList(prodStatesList);
			sumprod_noswitches.setVarList((VarList) sumprod.varList.clone());
			sumprod_noswitches.setStatesList((ArrayList<State>) prodStatesList.clone());
		}
		// fix deadlocks
		sumprod.findDeadlocks(true);

		sumprod.exportToDotFile(saveplace + "finalprod.dot");
		sumprod.exportToPrismExplicitTra(saveplace + "finalp" + ".tra");
		if(printHighlights)
		mainLog.println("Num States " + sumprod.getNumStates() + " choices " + sumprod.getNumChoices());
		if(printHighlights)
		mainLog.println("Switch States " + allSwitchStates.cardinality());
		if(printHighlights)
		mainLog.println("Exp Num States " + expStates + " choices " + expChoices + " switches " + totswitches + " = "
				+ (expChoices + totswitches));

		if (getExportProductTrans()) {
			if(printHighlights)
			mainLog.println(
					"\nExporting product transition matrix to file \"" + getExportProductTransFilename() + "\"...");
			sumprod.exportToPrismExplicitTra(getExportProductTransFilename());
		}
		if (getExportProductStates()) {
			if(printHighlights)
			mainLog.println("\nExporting product state space to file \"" + getExportProductStatesFilename() + "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductStatesFilename());
			VarList newVarList2 = (VarList) modulesFile.createVarList().clone();
			String daVar = "_da";
			while (newVarList2.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}
			newVarList2.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
			sumprod.exportStates(Prism.EXPORT_PLAIN, newVarList2, out);
			out.close();
		}

		res[0] = sumprod;
		res[1] = newAccStates;// accStatesF;
		res[2] = newbadStates;
		res[3] = sumprod_noswitches;
		res[4] = allSwitchStates;

		// res[0] = sumprod;
		// res[1] = newAccStates;//accStatesF;
		// res[2] = newBadStates;

		return res;
	}

	private int updateSwitches(BitSet allSwitchStates[], int numrobots, List<State> pstates, int numVars,
			BitSet initstates[], MDPSimple sumprod) {
		int totswitches = 0;
		double switchprob = 1.0;
		if(printDetails)
		mainLog.println("Switch States now");
		int numSwitchStates = allSwitchStates[0].cardinality(); //given that all of them have the same switches
		int numInitStates = 0;
		int setbit_r = allSwitchStates[0].nextSetBit(0); //fix this - what if they dont have the same num switches
		int setbit_r1 = 0;
		int cs2, cs1;
		Distribution prodDistr = null;
		for (int r = 1; r < numrobots; r++) {
			numInitStates = initstates[r].cardinality();

			setbit_r = allSwitchStates[r-1].nextSetBit(0);
			while (setbit_r != -1) {
				setbit_r1 = initstates[r].nextSetBit(0);
				while (setbit_r1 != -1) {
					if (sameDAVals(pstates.get(setbit_r1), pstates.get(setbit_r), numVars)) {
						prodDistr = new Distribution();
						cs2 = setbit_r1;
						cs1 = setbit_r;
						prodDistr.add(cs2, switchprob);
						sumprod.addActionLabelledChoice(cs1, prodDistr, "switch_" + (r - 1) + "_" + r);
						// mainLog.println("SS:" + setbit_r + "->" + setbit_r1 + ":" +cs1 + "->" + cs2);
						totswitches++;
					}
					setbit_r1 = initstates[r].nextSetBit(setbit_r1 + 1);
				}

				setbit_r = allSwitchStates[r-1].nextSetBit(setbit_r + 1);
				// if (setbit_r == 34)
				// mainLog.println("Check Here");
			}
		}
		if(printHighlights)
		mainLog.println("Max switches:"+totswitches);
		return totswitches;
	}

	private Object[] updateSwitches(BitSet allSwitchStates, int numrobots, List<State> pstates, int numVars,
			BitSet initstates[], int numStates, MDPSimple sumprod, int map[]) {
		Object res[] = new Object[2];
		int totswitches = 0;
		double switchprob = 1.0;
		if(printDetails)
		mainLog.println("Switch States now");
		BitSet updatedSwitches[] = new BitSet[numrobots];//(BitSet) allSwitchStates.clone();
		int numSwitchStates = allSwitchStates.cardinality();
		int numInitStates = 0;
		int setbit_r = allSwitchStates.nextSetBit(0);
		int setbit_r1 = 0;
		int cs2, cs1;
		Distribution prodDistr = null;
		updatedSwitches[0] = (BitSet)allSwitchStates.clone();
		for (int r = 1; r < numrobots; r++) {
			numInitStates = initstates[r].cardinality();
			updatedSwitches[r]=new BitSet(numStates);
			setbit_r = allSwitchStates.nextSetBit(0);
			while (setbit_r != -1) {
				updatedSwitches[r].set((r) * numStates + setbit_r); // this is for me //i'm updating for the next robots //
																	// so for use in the future.
				setbit_r1 = initstates[r].nextSetBit(0);
				while (setbit_r1 != -1) {
					if (sameDAVals(pstates.get(setbit_r1), pstates.get(setbit_r), numVars)) {
						prodDistr = new Distribution();
						cs2 = r * numStates + setbit_r1;
						cs1 = (r - 1) * numStates + setbit_r;
						prodDistr.add(map[cs2], switchprob);
						sumprod.addActionLabelledChoice(map[cs1], prodDistr, "switch_" + (r - 1) + "_" + r);
						if (printDetails)
						mainLog.println("SS:" + setbit_r + "->" + setbit_r1 + ":" + map[cs1] + "->" + map[cs2]);
						totswitches++;
					}
					setbit_r1 = initstates[r].nextSetBit(setbit_r1 + 1);
				}

				setbit_r = allSwitchStates.nextSetBit(setbit_r + 1);
				// if (setbit_r == 34)
				// mainLog.println("Check Here");
			}
		}
		res[0] = totswitches;
		res[1] = updatedSwitches;
		return res;
	}

	protected Object[] constructSumMDP_new(int numOp, int numrobots, BitSet allAccStates[], BitSet initstates[],
			MDP productMdp, BitSet safeltl, BitSet essentialAccStates[]) throws PrismException {

		int numres = 5;
		Object res[] = new Object[numres];
		int numStates = productMdp.getNumStates();
		String saveplace = System.getProperty("user.dir") + "/tests/decomp_tests/temp/";

		BitSet finalAccStates = new BitSet(numStates);
		BitSet badStates = new BitSet(numStates);
		BitSet allSwitchStates = new BitSet(numStates);
		BitSet rinitstates[] = new BitSet[numrobots];

		finalAccStates.set(0, numStates);
		badStates.set(0, numStates, false);

		// finalAccStates are states that are in all the acc states
		// switch states are the ones that are not
		// badStates are the ones in safeltl

		for (int i = 0; i < numOp; i++) {
			if (!safeltl.get(i)) {
				allSwitchStates.or(essentialAccStates[i]);
				finalAccStates.and(allAccStates[i]);
			} else {
				badStates.or(allAccStates[i]);
			}
		}
		// make sure you get rid of the badStates
		allSwitchStates.andNot(badStates);
		finalAccStates.andNot(badStates);
		allSwitchStates.andNot(finalAccStates);
		// init states //get rid of all the bad states
		for (int i = 0; i < numrobots; i++)
			initstates[i].andNot(badStates);
		// now lets get the varlist

		int numVars = productMdp.getVarList().getNumVars();
		List<State> states = productMdp.getVarList().getAllStates();
		List<State> pstates = productMdp.getStatesList();

		// hardcoding this but add it later //you have to get these when you do the
		// product
		int dainits[] = new int[numVars];
		for (int i = 0; i < numVars; i++)
			dainits[i] = 0;

		// printing stuff
		mainLog.println("Bad States:" + badStates.toString());
		mainLog.println("Final States:" + finalAccStates.toString());
		mainLog.println("Switch States:" + allSwitchStates.toString());
		for (int i = 0; i < numrobots; i++) {
			mainLog.println("Init States" + i + " :" + initstates[i].toString());
			rinitstates[i] = new BitSet(numStates);
			int numis = initstates[i].cardinality();
			int setbit = initstates[i].nextSetBit(0);
			for (int j = 0; j < numis; j++) {
				if (setbit != -1) {
					boolean isinitstate = isRealInitState(pstates.get(setbit), dainits, numVars);
					mainLog.println(":" + setbit + ":" + isinitstate);
					if (isinitstate)
						rinitstates[i].set(setbit);
					setbit = initstates[i].nextSetBit(setbit + 1);
				}

			}
		}

		// this is just over designing but probably making my life easier
		// maybe in the future we have different agent models so everything will change
		// that will not be okay cuz this is just a workaround
		// it should be da0xda1 and so on and the switch states being the final states
		// for each da
		// maybe I will find a way to do this. make all this mehnat raigan

		BitSet switches[] = new BitSet[numrobots];
		BitSet finals[] = new BitSet[numrobots];
		BitSet badstates[] = new BitSet[numrobots];

		MDP[] productRobots = new MDP[numrobots];
		// now lets make a copy of this mdp
		for (int r = 0; r < numrobots; r++) {
			productRobots[r] = productMdp; // i dont know if this is a deep copy
			switches[r] = (BitSet) allSwitchStates.clone();
			finals[r] = (BitSet) finalAccStates.clone();
			badstates[r] = (BitSet) badStates.clone();
		}

		VarList newVarList = null;

		if (productMdp.getVarList() != null) {
			VarList varList = productMdp.getVarList();
			// Create a (new, unique) name for the variable that will represent
			// DA states
			String daVar = "_da";
			while (varList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}

			newVarList = (VarList) varList.clone();
			// NB: if DA only has one state, we add an extra dummy state
			DeclarationInt daint = new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(numrobots - 1, 1)));
			Declaration decl = new Declaration(daVar, daint);
			newVarList.addVar(0, decl, 1, productMdp.getConstantValues());
		}

		// we need to make a new product which will basically be a sum of the old ones
		// so maxstates = numStates*numrobots
		// for each product mdp add transitions to switch states
		MDPSimple sumprod = new MDPSimple();
		MDPSimple sumprod_noswitches;// = new MDPSimple();
		sumprod.setVarList(newVarList);
		ArrayList<State> prodStatesList = null, daStatesList = null;

		if (productRobots[0].getStatesList() != null) {
			prodStatesList = new ArrayList<State>();
			daStatesList = new ArrayList<State>(numrobots);
			for (int i = 0; i < numrobots; i++) {
				daStatesList.add(new State(1).setValue(0, i));
			}
		}

		// newstate = r*NUMSTATES + s
		// lets make copies
		// TODO: LOOK AT THE STUFF FOR NEWSTATE. IT KIND OF MEANS THAT EACH ROBOT HAS TO
		// HAVE THE SAME AMOUNT OF STATES. WHICH WILL NOT ALWAYS BE THE CASE
		// WE NEED TO COME UP WITH BETTER INDEXING. REMEMBER. okay cool :P
		// should be using a map maybe need to check this
		int prodNumStates = numrobots * numStates;
		int map[] = new int[prodNumStates];
		BitSet newAccStates = new BitSet(prodNumStates);
		BitSet newbadStates = new BitSet(prodNumStates);
		Arrays.fill(map, -1);
		//
		int expStates = 0;
		int expChoices = 0;
		for (int r = 0; r < numrobots; r++) {
			// go over the entire product mdp for robot r
			MDP pr = productRobots[r];
			numStates = pr.getNumStates();
			mainLog.println("Robot " + r + " states " + numStates + " choices " + pr.getNumChoices());
			expStates += numStates;
			expChoices += pr.getNumChoices();
			int numchoices = -1;
			int s_ = -1;
			for (int s = 0; s < numStates; s++) {
				sumprod.addState();
				int cs = (r * numStates) + s;

				if (finals[r].get(s)) {
					newAccStates.set(cs);
				}
				if (badstates[r].get(s)) {
					newbadStates.set(cs);
				}
				map[cs] = sumprod.getNumStates() - 1;

				// add as init state ? //only add the first robot's init state as an init state
				// otherwise no
				if (r == 0 && rinitstates[r].get(s)) {
					sumprod.addInitialState(map[cs]);
					// sumprod.isInitialState(cs);
				}
				numchoices = pr.getNumChoices(s);
				if (prodStatesList != null) {
					// Store state information for the product
					prodStatesList.add(new State(daStatesList.get(r), pr.getStatesList().get(s)));
				}
				Iterator<Map.Entry<Integer, Double>> iter;

				// if (!badStatesF.get(s)) {
				for (int j = 0; j < numchoices; j++) {
					iter = pr.getTransitionsIterator(s, j);

					int ns = 0;
					Distribution prodDistr = new Distribution();
					while (iter.hasNext()) {
						Map.Entry<Integer, Double> e = iter.next();
						s_ = e.getKey();
						double prob = e.getValue();
						ns = (r * numStates) + s_;
						prodDistr.add(ns, prob);
					}

					sumprod.addActionLabelledChoice(map[cs], prodDistr, pr.getAction(s, j) + "_r" + r);
					// mainLog.println("Adding (" + s + "," + s_ + ")" + cs + " " +
					// prodDistr.toString() + " "
					// + pr.getAction(s, j) + "_r" + r);
				}
				// }
				// else
				// {
				// //if its a bad state add a self loop and not other transitions
				// //dont want to add the state if no other states go to it though, maybe I
				// should count or something
				// Distribution prodDistr = new Distribution();
				// prodDistr.add(cs, 1.0); //self loop
				// sumprod.addActionLabelledChoice(cs, prodDistr, "bad_state"+ "_r" + r);
				// }

			}
		}

		sumprod_noswitches = new MDPSimple(sumprod);

		// now we're going to add the switch transitions
		// just link up all the switch states in all the mdps
		// TODO fix this... link switch states based on the da (use daswitchstates)
		// so change all of this ? // TODO!!!!
		// int totswitches
		Object updatedSwitchesRes[] = updateSwitches(allSwitchStates, numrobots, pstates, numVars, initstates,
				numStates, sumprod, map);
		int totswitches = (int) updatedSwitchesRes[0];
		BitSet updatedSwitches[] = (BitSet[]) updatedSwitchesRes[1];
		if (prodStatesList != null) {
			sumprod.setStatesList(prodStatesList);
			sumprod_noswitches.setVarList((VarList) sumprod.varList.clone());
			sumprod_noswitches.setStatesList((ArrayList<State>) prodStatesList.clone());

		}
		// fix deadlocks
		sumprod.findDeadlocks(true);

		sumprod.exportToDotFile(saveplace + "finalprod.dot");
		sumprod.exportToPrismExplicitTra(saveplace + "finalp" + ".tra");
		mainLog.println("Num States " + sumprod.getNumStates() + " choices " + sumprod.getNumChoices());
		mainLog.println("Switch States " + allSwitchStates.cardinality());

		mainLog.println("Exp Num States " + expStates + " choices " + expChoices + " switches " + totswitches + " = "
				+ (expChoices + totswitches));

		if (getExportProductTrans()) {
			mainLog.println(
					"\nExporting product transition matrix to file \"" + getExportProductTransFilename() + "\"...");
			sumprod.exportToPrismExplicitTra(getExportProductTransFilename());
		}
		if (getExportProductStates()) {
			mainLog.println("\nExporting product state space to file \"" + getExportProductStatesFilename() + "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductStatesFilename());
			VarList newVarList2 = (VarList) modulesFile.createVarList().clone();
			String daVar = "_da";
			while (newVarList2.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}
			newVarList2.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
			sumprod.exportStates(Prism.EXPORT_PLAIN, newVarList2, out);
			out.close();
		}

		res[0] = sumprod;
		res[1] = newAccStates;// accStatesF;
		res[2] = newbadStates;
		res[3] = sumprod_noswitches;
		res[4] = updatedSwitches;// allSwitchStates;

		// res[0] = sumprod;
		// res[1] = newAccStates;//accStatesF;
		// res[2] = newBadStates;

		return res;
	}

	int getIthSetBit(BitSet ss, int n) {
		int res = -1;
		for (int i = 0; i <= n; i++) {
			res = ss.nextSetBit(res + 1);
		}
		return res;

	}

	// when you're ready to make a castle from the sand (box)
	@SuppressWarnings("unchecked")
	protected StateValues ltlDecomp(Model model, ExpressionFunc expr, BitSet statesOfInterest) throws PrismException {

		LTLModelChecker mcLtls[];
		LTLModelChecker.LTLProduct<MDP> product = null;
		MDP productMdp;
		DA<BitSet, ? extends AcceptanceOmega> das[];
		// Vector<BitSet> labelBSs[];
		ModelCheckerResult res = null;
		// BitSet switchStates = null;
		BitSet accStatesF = null;
		BitSet allAccStates[];

		// String saveplace = System.getProperty("user.dir") +
		// "/tests/decomp_tests/temp/";

		int numrobots = 2;
		int example_number = 1; // 0 = not extended , 1 = extended
		if (example_number == 1)
			numrobots = 3;
		BitSet initstates[] = new BitSet[numrobots];
		BitSet essentialstates[] = new BitSet[numrobots];

		// Get LTL spec
		// Number of Operands
		int numOp = expr.getNumOperands();
		Expression ltls[] = new Expression[numOp];
		das = new DA[numOp];
		// labelBSs = new Vector[numOp];
		mcLtls = new LTLModelChecker[numOp];
		BitSet safeltl = new BitSet(numOp);

		for (int i = 0; i < numOp; i++) {
			if (expr.getOperand(i) instanceof ExpressionQuant)
				ltls[i] = ((ExpressionQuant) expr.getOperand(i)).getExpression();
			mcLtls[i] = new LTLModelChecker(this);
		}

		System.out.println("--------------------------------------------------------------");
		System.out.println("The flat MDP model has " + model.getNumStates() + " states");
		for (int i = 0; i < numOp; i++)
			System.out.println("The specification is " + ltls[i].toString());
		System.out.println("Generating optimal policy...");
		System.out.println(" ");

		Object resMdp[] = constructMdpDaProd(numrobots, numOp, (MDP) model, mcLtls, das, ltls, example_number);
		product = (LTLModelChecker.LTLProduct<MDP>) resMdp[0];
		allAccStates = (BitSet[]) resMdp[1];
		initstates = (BitSet[]) resMdp[2];
		safeltl = (BitSet) resMdp[3];
		essentialstates = (BitSet[]) resMdp[4];
		productMdp = product.getProductModel();

		// TODO constructSumMdp()
		resMdp = constructSumMDP_new(numOp, numrobots, allAccStates, initstates, productMdp, safeltl, essentialstates);
		// ret sumprod and accStates
		MDPSimple sumprod = (MDPSimple) resMdp[0];
		accStatesF = (BitSet) resMdp[1];
		BitSet badStates = (BitSet) resMdp[2];
		badStates.flip(0, sumprod.numStates);
		MDPSimple sumprod_noswitches = (MDPSimple) resMdp[3];
		BitSet switchStates[] = (BitSet[]) resMdp[4];
		mainLog.println("Using compute until");

		// TODO: what is the right way to do this
		genStrat = true;
		res = computeUntilProbs(sumprod, badStates, accStatesF, false);

		// unfoldPolicy(res, sumprod, accStatesF, numrobots, sumprod_noswitches,
		// switchStates);
		unfoldPolicyClean(res, sumprod, accStatesF, numrobots, 
				sumprod_noswitches, switchStates, badStates);
//		unfoldPolicy_new_bits(res, sumprod, accStatesF, numrobots, sumprod_noswitches, 
//				switchStates, badStates, initstates,true);
		sumprod.exportToDotFile(saveplace + "second_sumprod.dot");
		sumprod.exportToPrismExplicit(saveplace + "second_sumprod");

		res = computeUntilProbs(sumprod, badStates, accStatesF, false);

		unfoldPolicy(res, sumprod, accStatesF, numrobots, sumprod_noswitches, switchStates, false);

		// unfoldPolicy(res, sumprod,accStatesF,numrobots);
		StateValues probsProduct = StateValues.createFromDoubleArray(res.soln, sumprod);
		// Mapping probabilities in the original model
		StateValues probs = product.projectToOriginalModel(probsProduct);
		// Get final prob result
		double maxProb = probs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nMaximum probability to satisfy specification is " + maxProb);

		// Output vector over product, if required
		if (getExportProductVector()) {
			mainLog.println("\nExporting product solution vector matrix to file \"" + getExportProductVectorFilename()
					+ "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductVectorFilename());
			probsProduct.print(out, false, false, false, false);
			out.close();
		}

		// Mapping probabilities in the original model
		probs = product.projectToOriginalModel(probsProduct);
		probsProduct.clear();

		return probs; // costs
	}

	protected void unfoldPolicy(ModelCheckerResult res, MDPSimple sumprod, BitSet accStatesF, int numrobots,
			MDPSimple sumprod_noswitches, BitSet[] switches) throws PrismException {
		unfoldPolicy(res, sumprod, accStatesF, numrobots, sumprod_noswitches, switches, true);
	}
	

	protected void unfoldPolicy(ModelCheckerResult res, MDPSimple sumprod, BitSet accStatesF, int numrobots,
			MDPSimple sumprod_noswitches, BitSet[] switchStates, boolean addLinks) throws PrismException {

		Strategy strat = res.strat;
		ArrayList<ArrayList<Map.Entry<Integer, Integer>>> pol[] = new ArrayList[numrobots];
		for (int i = 0; i < numrobots; i++) {
			pol[i] = new ArrayList();
		}
		Iterator<Integer> initstates = sumprod.getInitialStates().iterator();

		ArrayList<Integer> newinitstates = new ArrayList<Integer>();
		while (initstates.hasNext()) {
			int inits = (int) initstates.next();
			strat.initialise(inits);
			int state = inits;
			int choices = sumprod.getNumChoices(state);

			Stack<Map.Entry<Integer, Integer>> st = new Stack();
			Stack time = new Stack();
			int timecount = 0;
			BitSet stuckStates = new BitSet(sumprod.numStates);
			BitSet stuckStateRobot[] = new BitSet[numrobots];
			for (int i = 0; i < numrobots; i++) {
				stuckStateRobot[i] = new BitSet(sumprod.numStates);
			}
			BitSet discovered = new BitSet(sumprod.numStates);

			List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
			int numVar = sumprod.varList.getNumVars();
			int whichRobot = -1;
			int maxtimecount = -1;
			int oldtimecount = timecount;
			Map.Entry<Integer, Integer> temp = new AbstractMap.SimpleEntry(state, -1);
			st.push(temp);
			time.push(timecount);
			Object action = strat.getChoiceAction();
			if (action == null || "*".equals(action))
				continue;
			boolean switched = false;
			boolean switcheder = false;
			int parentstate = -1;
			while (!st.isEmpty()) {
				Map.Entry<Integer, Integer> temp1 = st.pop();
				state = temp1.getKey();
				Map.Entry<Integer, Integer> temp3;
				parentstate = temp1.getValue();
				whichRobot = getIndValFromState(states.get(state), 0);
				timecount = (int) time.pop();
				while (pol[whichRobot].size() < (timecount + 1)) {
					pol[whichRobot].add(new ArrayList<Map.Entry<Integer, Integer>>());
				}

				if (!discovered.get(state) & !accStatesF.get(state)) {
					strat.initialise(state);
					action = strat.getChoiceAction();
					// if(action.toString() == "*")
					if (action == null || "*".equals(action)) {
						stuckStates.set(state);
						stuckStateRobot[whichRobot].set(state);
					}
					if (action.toString().contains("switch_er")) {
						oldtimecount = timecount;
						timecount = timecount - 1;
						switched = true;
						switcheder = true;
					} else if (action.toString().contains("switch")) {
						oldtimecount = timecount;
						timecount = -1;
						switched = true;
					}
					if (switched)
						mainLog.print(oldtimecount + ":r" + whichRobot + ":" + state + ":" + action.toString());
					else
						mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":" + action.toString());

					discovered.set(state);
					choices = sumprod.getNumChoices(state);
					for (int c = 0; c < choices; c++) {
						if (action.equals(sumprod.getAction(state, c))) {
							Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(state, c);
							// double mprob = 0;
							while (iter.hasNext()) {
								Entry<Integer, Double> sp = iter.next();
								temp3 = new AbstractMap.SimpleEntry(sp.getKey(), state);
								st.push(temp3);
								// if (switcheder)
								// time.push(timecount);
								// else
								maxtimecount = Math.max(timecount + 1, maxtimecount);

								time.push(timecount + 1);
							}
						}
					}
				} else if (accStatesF.get(state)) {
					mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":!");
				}
				mainLog.print("\n");
				if (!switched)// if (timecount != -1)
					pol[whichRobot].get(timecount).add(temp1);
				else // if (!switcheder)
				{
					pol[whichRobot].get(oldtimecount).add(temp1);
					switched = false;
					switcheder = false;
				}
				// else if (switcheder)
				// {
				// pol[whichRobot].get(timecount).add(state); switcheder=false;
				// }
			}
			mainLog.println("Stuck States: " + stuckStates.toString());
			// link a stuck state with some prob to another stuck state or a not stuck state
			// this will be the round robin approach
			// if stuck state, save state and robot number
			// for next robot link this state to next robot's states
			if (addLinks) {
				// sumprod.clearInitialStates();
				List<Integer> ci = null;
				for (int t = 0; t <= maxtimecount; t++) {
					mainLog.print(t + "\t");
					for (int r = 0; r < numrobots; r++) {
						ArrayList<ArrayList<Entry<Integer, Integer>>> polt = pol[r];

						if (polt.size() > t) {
							mainLog.print(r + ":");
							ArrayList<Entry<Integer, Integer>> polti = polt.get(t);
							for (int s = 0; s < polti.size(); s++) {
								int rs = polti.get(s).getKey();

								mainLog.print(rs + ",");
								if (stuckStates.get(rs)) {
									int ps = polti.get(s).getValue();
									// //because I know there are only two states
									// //TODO: figure out a smarter way to do this
									newinitstates.add(rs);

									// i know this is a repeat but needs to happen
									int nextr = (r + 1) % numrobots;

									ArrayList<Entry<Integer, Integer>> polti2;
									// exploiting the chain thing. if nextr < r
									// then we know that most of our tasks in r have not been acheived
									// so we need to find the change in state between r & nextr
									// as well as r and r's ps
									int nextrt = t;
									if (!(pol[nextr].size() > nextrt)) {
										nextrt = pol[nextr].size() - 1;

									}
									polti2 = pol[nextr].get(nextrt);
									int otherRobotStates[] = new int[polti2.size()];
									double probs[] = new double[polti2.size()];
									// add link
									for (int ors = 0; ors < polti2.size(); ors++) {
										int otherstate = polti2.get(ors).getKey();
										// if (ps!= -1)
										// { //ci=
										// findChangedStateIndices(states.get(rs),states.get(otherstate),numVar);
										if (nextr < r) {
											ci = findChangedStateIndices(states.get(rs), states.get(otherstate),
													numVar);
											otherstate = findSameStateExcept(states.get(otherstate), ci, states, numVar,
													1);
										} else {// get the t'th stuck state
											int ss = getIthSetBit(stuckStateRobot[nextr], nextrt - 1);
											ci = findChangedStateIndices(states.get(rs), states.get(ss), numVar);
											otherstate = findSameStateExcept(states.get(otherstate), ci, states,
													numVar);
										}
										// }
										otherRobotStates[ors] = otherstate;
										if (isfailState(states.get(otherstate), numVar)) {
											probs[ors] = 0.2; // hardcoding this
										} else
											probs[ors] = 0.8;
									}
									// the link is not from rs to otherRobotStates but from rs to otherRobotStates
									// where the da for rs is the same.
									addlink(sumprod, rs, otherRobotStates, probs, nextr, ps);
								}
							}
							mainLog.print("\t");
						}
					}
					mainLog.print("\n");
				}

				sumprod.findDeadlocks(true);
			}
		}
		sumprod.clearInitialStates();
		for (int s = 0; s < newinitstates.size(); s++)
			sumprod.addInitialState(newinitstates.get(s));
	}
	public class robotTimeState{
		public int rnum; 
		public int time; 
		public int state; 
		public int pstate;
		public robotTimeState()
		{
			rnum = -1;
			time = -1; 
			state = -1; 
			pstate = -1;
		}
		public robotTimeState(int r, int t, int s)
		{
			rnum = r; 
			time = t; 
			state = s; 
			pstate = -1;
		}
		public robotTimeState(int r, int t, int s,int p)
		{
			rnum = r; 
			time = t; 
			state = s; 
			pstate = p;
		}
		public void print()
		{
			mainLog.print("r:"+rnum+",t:"+time+",s:"+state+"<-"+pstate);
		}
		public String toString_()
		{
			return "r:"+rnum+",t:"+time+",s:"+state;
		}
		@Override
		public String toString() {
			return "[r=" + rnum + ", t=" + time + ", s=" + state + ", p=" + pstate + "]";
		}
		
	};


	private class policyUnfolded {
		
		public ArrayList<Map.Entry<robotTimeState,ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>>>> bigpol;
		public ArrayList<ArrayList<Map.Entry<Integer, Integer>>> pol[];
		public ArrayList<ArrayList<Map.Entry<Integer, Integer>>> partialpol[];
		public int timesUnfolded; 
		public BitSet stuckStates;
		public Queue<robotTimeState> stuckStack; 
		public HashMap<Integer,robotTimeState> doneStates; 
		public ArrayList<ArrayList<Integer>> stuckStateRobot;
		public ArrayList<ArrayList<Integer>> stuckStateRobotTime;
		public BitSet policyComputedFor[];
		public robotTimeState currpol; 
		int maxtimecount;
		public MDPSimple polmdp; 
		public int polmap[]; 
		int statenotaddedvalue; 
		
		protected ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> createNewPol(int combstates[][],int newstates[],int newstate, List<State> states,int numVar,int numrobots,int curr_robot)
		{
			int r = curr_robot;
			int states_now[] = combstates[newstate]; 
			states_now[r] = newstates[newstate];
			int rs = states_now[r];
			ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>> tempol= new ArrayList(); 
			for(int rt=0; rt<numrobots; rt++)
			{
				tempol.add(new ArrayList());
				if(rt!=r) {
				int state_now = combstates[newstate][rt];
			states_now[rt]=getMergedState(states.get(rs), states.get(state_now), states, numVar);
			}
				tempol.get(rt).add(new ArrayList()); 
				tempol.get(rt).get(0).add(new AbstractMap.SimpleEntry(states_now[rt], rs));
			}
			return tempol;

		}
		protected ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> createNewPol(int initstates[],int curr_robotstate, List<State> states,int numVar,int numrobots,int curr_robot)
		{
			int r = curr_robot;
			int states_now[] = initstates;//combstates[newstate]; 
			states_now[r] = curr_robotstate;
			int rs = states_now[r];
			ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>> tempol= new ArrayList(); 
			for(int rt=0; rt<numrobots; rt++)
			{
				tempol.add(new ArrayList());
				if(rt!=r) {
				int state_now = states_now[rt];
			states_now[rt]=getMergedState(states.get(rs), states.get(state_now), states, numVar);
			}
				tempol.get(rt).add(new ArrayList()); 
				tempol.get(rt).get(0).add(new AbstractMap.SimpleEntry(states_now[rt], rs));
			}
			return tempol;

		}
		public void addTobigPol(robotTimeState inits_rts,ArrayList<ArrayList<Map.Entry<Integer, Integer>>> tpol[] ,int initstater[])
		{
			if(inits_rts.state == 527)
				mainLog.println("527 here - this is added twice check why");
			ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> ppCopy = copyPoltoList(tpol,initstater,inits_rts); 
		bigpol.add(new AbstractMap.SimpleEntry(inits_rts, ppCopy));	//TODO this tomorrow 
			
		}
		public void addTobigPol(robotTimeState inits_rts,ArrayList<ArrayList<Map.Entry<Integer, Integer>>> tpol[] )
		{
			if(inits_rts.state == 527)
				mainLog.println("527 here - this is added twice check why");
			ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> ppCopy = copyPoltoList(tpol); 
		bigpol.add(new AbstractMap.SimpleEntry(inits_rts, ppCopy));
			
		}
		public boolean bigpolHasState(int state)
		{
			boolean res=false;
			for(int i =0; i<bigpol.size(); i++)
			{
				if (bigpol.get(i).getKey().state==state)
				{
					res=true;
					break;
				}
			}
			return res; 
		}
		public Entry<robotTimeState, ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>>> getPolForStateFromBigPol(int state)
		{
			for(int i =0; i<bigpol.size(); i++)
			{
				if (bigpol.get(i).getKey().state==state)
				{
					return (bigpol.get(i));
				}
			}
			return null; 
		}
		public void copyPolForStateToPol(robotTimeState rts)
		{
			int state = rts.state;
			ArrayList<ArrayList<Entry<Integer, Integer>>>[] temppol = getPolForState(state);
			if (temppol!=null)
			{	pol = temppol; 
				currpol = rts;
			}
			else
				mainLog.println("Haven't created a policy for "+state);
		}
		public void copyPolForStateToPol(int state)
		{
			ArrayList<ArrayList<Entry<Integer, Integer>>>[] temppol = getPolForState(state);
			if (temppol!=null)
				pol = temppol; 
			else
				mainLog.println("Haven't created a policy for "+state);
		}
		
		public ArrayList<ArrayList<Map.Entry<Integer, Integer>>>[] getPolForState_setCurPol(int state)
		{	
			//get it from bigpol 
			//convert it to something for pol 
			
			Entry<robotTimeState, ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>>> poltempr = getPolForStateFromBigPol( state);
			if(poltempr != null) {
				currpol = poltempr.getKey();
			ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> poltemp = poltempr.getValue();
			ArrayList<ArrayList<Map.Entry<Integer, Integer>>> poltemp2[] = new ArrayList[poltemp.size()];
			for(int i = 0; i<poltemp2.length; i++)
			{
				ArrayList<ArrayList<Map.Entry<Integer, Integer>>> copy = new ArrayList<ArrayList<Map.Entry<Integer, Integer>>>();
				ArrayList<ArrayList<Entry<Integer, Integer>>> tocopy = poltemp.get(i);
				
				for(int j = 0; j<tocopy.size(); j++)
				{
					ArrayList<Entry<Integer, Integer>> tocopy2 = tocopy.get(j); 
					ArrayList<Entry<Integer, Integer>> copy2 = new ArrayList<Entry<Integer, Integer>>();
					for(int k=0; k<tocopy2.size(); k++)
					{
						Entry<Integer, Integer> temp = tocopy2.get(k);
						copy2.add(new AbstractMap.SimpleEntry(temp.getKey(), temp.getValue()));
					}
					copy.add(copy2);
					
				}
				poltemp2[i]=copy;
			}
			return poltemp2;
			}
			else 
				return null;
		}

		
		public ArrayList<ArrayList<Map.Entry<Integer, Integer>>>[] getPolForState(int state)
		{	
			//get it from bigpol 
			//convert it to something for pol 
			Entry<robotTimeState, ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>>> poltempr = getPolForStateFromBigPol( state);
			if(poltempr != null) {
			ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> poltemp = poltempr.getValue();
			ArrayList<ArrayList<Map.Entry<Integer, Integer>>> poltemp2[] = new ArrayList[poltemp.size()];
			for(int i = 0; i<poltemp2.length; i++)
			{
				ArrayList<ArrayList<Map.Entry<Integer, Integer>>> copy = new ArrayList<ArrayList<Map.Entry<Integer, Integer>>>();
				ArrayList<ArrayList<Entry<Integer, Integer>>> tocopy = poltemp.get(i);
				
				for(int j = 0; j<tocopy.size(); j++)
				{
					ArrayList<Entry<Integer, Integer>> tocopy2 = tocopy.get(j); 
					ArrayList<Entry<Integer, Integer>> copy2 = new ArrayList<Entry<Integer, Integer>>();
					for(int k=0; k<tocopy2.size(); k++)
					{
						Entry<Integer, Integer> temp = tocopy2.get(k);
						copy2.add(new AbstractMap.SimpleEntry(temp.getKey(), temp.getValue()));
					}
					copy.add(copy2);
					
				}
				poltemp2[i]=copy;
			}
			return poltemp2;
			}
			else 
				return null;
		}

		public ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>> copyPoltoList(ArrayList<ArrayList<Map.Entry<Integer, Integer>>>[] polToCopy,int initstater[],robotTimeState inits_rts)
		{
			ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>> ppCopy = new ArrayList();
			for(int i =0; i<polToCopy.length; i++)
			{
				ppCopy.add(new ArrayList<ArrayList<Map.Entry<Integer, Integer>>>()); 
				for(int j = 0; j<polToCopy[i].size(); j++)
				{
					ArrayList<Map.Entry<Integer,Integer>> tempList = new ArrayList<Map.Entry<Integer,Integer>>();
					ArrayList<Map.Entry<Integer,Integer>> tocopy = polToCopy[i].get(j);
					for(int k=0; k<tocopy.size(); k++)
					{
						Map.Entry<Integer, Integer> tempentry = new AbstractMap.SimpleEntry(tocopy.get(k).getKey().intValue(), tocopy.get(k).getValue().intValue());
						tempList.add(tempentry);
					}
					ppCopy.get(i).add(tempList);
				}
				//adding a state 
				if(ppCopy.get(i).size()==0)
				{
					ArrayList<Entry<Integer, Integer>> temparr = new ArrayList<Entry<Integer, Integer>> ();//ppCopy.get(i).get(0);
					temparr.add(new AbstractMap.SimpleEntry(initstater[i], inits_rts.state));
//					if (temparr.size()==0)
//					{
						ppCopy.get(i).add(temparr);
//					}
				}
			}
			return ppCopy;
			
		}

		
		public ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>> copyPoltoList(ArrayList<ArrayList<Map.Entry<Integer, Integer>>>[] polToCopy)
		{
			ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>> ppCopy = new ArrayList();
			for(int i =0; i<polToCopy.length; i++)
			{
				ppCopy.add(new ArrayList<ArrayList<Map.Entry<Integer, Integer>>>()); 
				for(int j = 0; j<polToCopy[i].size(); j++)
				{
					ArrayList<Map.Entry<Integer,Integer>> tempList = new ArrayList<Map.Entry<Integer,Integer>>();
					ArrayList<Map.Entry<Integer,Integer>> tocopy = polToCopy[i].get(j);
					for(int k=0; k<tocopy.size(); k++)
					{
						Map.Entry<Integer, Integer> tempentry = new AbstractMap.SimpleEntry(tocopy.get(k).getKey().intValue(), tocopy.get(k).getValue().intValue());
						tempList.add(tempentry);
					}
					ppCopy.get(i).add(tempList);
				}
			}
			return ppCopy;
			
		}
		public ArrayList<ArrayList<Map.Entry<Integer, Integer>>>[] copyPol(ArrayList<ArrayList<Map.Entry<Integer, Integer>>>[] polToCopy)
		{
			ArrayList<ArrayList<Map.Entry<Integer, Integer>>> ppCopy[] = new ArrayList[polToCopy.length];
			for(int i =0; i<polToCopy.length; i++)
			{
				ppCopy[i] = new ArrayList<ArrayList<Map.Entry<Integer, Integer>>>(); 
				for(int j = 0; j<polToCopy[i].size(); j++)
				{
					ArrayList<Map.Entry<Integer,Integer>> tempList = new ArrayList<Map.Entry<Integer,Integer>>();
					ArrayList<Map.Entry<Integer,Integer>> tocopy = polToCopy[i].get(j);
					for(int k=0; k<tocopy.size(); k++)
					{
						Map.Entry<Integer, Integer> tempentry = new AbstractMap.SimpleEntry(tocopy.get(k).getKey().intValue(), tocopy.get(k).getValue().intValue());
						tempList.add(tempentry);
					}
					ppCopy[i].add(tempList);
				}
				if(polToCopy[i].size()==0)
					ppCopy[i].add(new ArrayList());
			}
			return ppCopy;
			
		}
		public void printBigPol()
		{
			for(int i = 0; i<bigpol.size(); i++)
			{
				mainLog.println("S:"+bigpol.get(i).getKey());
				printpol(bigpol.get(i).getValue());
			}
		}

		protected void printpol(ArrayList<ArrayList<ArrayList<Map.Entry<Integer, Integer>>>> tpol)
		{
	
			for(int t = 0; t<tpol.size(); t++)
			{
				for(int r=0; r<tpol.get(t).size(); r++)
				{
								mainLog.print(r+":");

					
					for(int s=0; s<tpol.get(t).get(r).size(); s++)
					{
						mainLog.print(tpol.get(t).get(r).get(s)+",");
					}
				}
				mainLog.print("\t");
			}
		
			
			mainLog.print("\n");

		}
		protected void print()
		{
			int t=0; 
			while(t<=maxtimecount) {
				mainLog.print("\n"+t+":");
			for(int r = 0; r<pol.length; r++)
			{
				if(t<pol[r].size())
				{				mainLog.print(r+":");

					
					for(int s=0; s<pol[r].get(t).size(); s++)
					{
						mainLog.print(pol[r].get(t).get(s)+",");
					}
				}
				mainLog.print("\t");
			}
			t++;
			}
			mainLog.print("\n");
			mainLog.println("Stuck States:"+stuckStack.toString());
			mainLog.println("Policy Computed For");
			int sumComputed = 0; 
			for(int r = 0; r<policyComputedFor.length; r++)
			{
				mainLog.println(r+":"+policyComputedFor[r].toString());
				sumComputed+=policyComputedFor[r].cardinality();
			}
			mainLog.println("Num stuck states "+stuckStates.cardinality()+" sum policy computed for "+sumComputed);
			mainLog.println("Policy Unfolded for "+timesUnfolded+" times");
		}
		public policyUnfolded(int numrobots, MDPSimple sumprod, Strategy strat, BitSet accStatesF) {
			timesUnfolded++;
			statenotaddedvalue = -1;
			bigpol = new ArrayList();
			doneStates = new HashMap<Integer,robotTimeState>();
			polmap = new int[sumprod.numStates]; 
			Arrays.fill(polmap, statenotaddedvalue);
			polmdp = new MDPSimple();
			getPolicyTime(numrobots, sumprod, strat, accStatesF);
		}
		public boolean addToPolArray(int robotnum,int time, Entry<Integer,Integer> stateAndParent)
		{
			ArrayList<Entry<Integer, Integer>> tpol = pol[robotnum].get(time); 

			boolean toadd = true;
			for(int i=0; i<tpol.size(); i++)
			{
				
				Entry<Integer, Integer> currtol = tpol.get(i);
				if (currtol.getValue().intValue()==stateAndParent.getValue().intValue() && currtol.getKey().intValue()==stateAndParent.getKey().intValue())
				{
					toadd=false;
					break;
				}
			}
			if (toadd)
			{
				pol[robotnum].get(time).add(stateAndParent);
			}
			return toadd;
			
		}

		public void updatePolicy(int numrobots, MDPSimple sumprod, Strategy strat, BitSet accStatesF,int timecount,int parent_state) {
			Iterator<Integer> initstates = sumprod.getInitialStates().iterator();
			robotTimeState inits_rts = null;
			// = new MDPSimple();
			timesUnfolded++;
			while (initstates.hasNext()) {
				int inits = (int) initstates.next();
				strat.initialise(inits);
				int state = inits;
				int choices = sumprod.getNumChoices(state);
				
				Stack<Map.Entry<Integer, Integer>> st = new Stack();
				Stack time = new Stack();
//				int timecount = 0;
				BitSet discovered = new BitSet(sumprod.numStates);

				List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
				int whichRobot = -1;
//				maxtimecount = -1;
				int oldtimecount = timecount;
				whichRobot = getIndValFromState(states.get(state), 0);
				inits_rts = new robotTimeState(whichRobot,timecount,state,parent_state);
				Map.Entry<Integer, Integer> temp = new AbstractMap.SimpleEntry(state, parent_state);
				st.push(temp);
				time.push(timecount);
				Object action = strat.getChoiceAction();
				if (action == null || "*".equals(action))
					continue;
				boolean switched = false;
				while (!st.isEmpty()) {
					Map.Entry<Integer, Integer> temp1 = st.pop();
					state = temp1.getKey();
					Map.Entry<Integer, Integer> temp3;
					whichRobot = getIndValFromState(states.get(state), 0);
					timecount = (int) time.pop();
					while (pol[whichRobot].size() < (timecount + 1)) {
						pol[whichRobot].add(new ArrayList<Map.Entry<Integer, Integer>>());
					}

					if (!discovered.get(state) & !accStatesF.get(state)) {
						strat.initialise(state);
						action = strat.getChoiceAction();
						// if(action.toString() == "*")
						if (action == null || "*".equals(action)) {
							stuckStates.set(state);
							(stuckStateRobot.get(whichRobot)).add(state);
							(stuckStateRobotTime.get(whichRobot)).add(timecount);
							stuckStack.add(new robotTimeState(whichRobot,timecount,state,parent_state));
						}
						if (action.toString().contains("switch_er")) {
							oldtimecount = timecount;
							timecount = timecount - 1;
							switched = true;
						} else if (action.toString().contains("switch")) {
							oldtimecount = timecount;
							timecount = -1;
							switched = true;
						}
						if (switched)
							mainLog.print(oldtimecount + ":r" + whichRobot + ":" + state + ":" + action.toString());
						else
							mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":" + action.toString());

						discovered.set(state);
						choices = sumprod.getNumChoices(state);
						for (int c = 0; c < choices; c++) {
							if (action.equals(sumprod.getAction(state, c))) {
								Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(state, c);
								// double mprob = 0;
								while (iter.hasNext()) {
									Entry<Integer, Double> sp = iter.next();
									temp3 = new AbstractMap.SimpleEntry(sp.getKey(), state);
									st.push(temp3);
									// if (switcheder)
									// time.push(timecount);
									// else
									maxtimecount = Math.max(timecount + 1, maxtimecount);

									time.push(timecount + 1);
								}
							}
						}
					} else if (accStatesF.get(state)) {
						mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":!");
					}
					mainLog.print("\n");
					if (!switched)// if (timecount != -1)
						addToPolArray(whichRobot,timecount,temp1);//pol[whichRobot].get(timecount).add(temp1);
					else // if (!switcheder)
					{
						addToPolArray(whichRobot,oldtimecount,temp1);//pol[whichRobot].get(oldtimecount).add(temp1);
						switched = false;
					}
				}
				addTobigPol(inits_rts,pol);

			}

		}

		
		public void updatePolicy(int numrobots, MDPSimple sumprod, Strategy strat, BitSet accStatesF,int parent_state) {
			
			int timecount = 0; 
			pol = new ArrayList[numrobots];
			for (int i = 0; i < numrobots; i++) {
				pol[i] = new ArrayList();
			}

			Iterator<Integer> initstates = sumprod.getInitialStates().iterator();
			List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
			int whichRobot = -1;

			// = new MDPSimple();
			timesUnfolded++;
			int inits;
			int state;
			int choices;

			Stack<Map.Entry<Integer, Integer>> st;
			Stack time;

			BitSet discovered;

			while (initstates.hasNext()) {

				//int 
				inits = (int) initstates.next();
				if(inits==69)
					mainLog.print("state = 69, check what happens");
				strat.initialise(inits);
				//int 
				state = inits;
				//int 
				choices = sumprod.getNumChoices(state);

				//Stack<Map.Entry<Integer, Integer>>
				st = new Stack();
				//Stack
				time = new Stack();

				//BitSet 
				discovered = new BitSet(sumprod.numStates);

//				maxtimecount = -1;
				whichRobot = getIndValFromState(states.get(state), 0);
				robotTimeState inits_rts = new robotTimeState(whichRobot,timecount,state,parent_state);
				stuckStates = new BitSet(sumprod.numStates);
				stuckStates = new BitSet(sumprod.numStates);
				stuckStateRobot = new ArrayList<ArrayList<Integer>>(); // i'm giving up - lets find a new way to do
																		// this, this is wasting too much time!
				stuckStateRobotTime = new ArrayList<ArrayList<Integer>>();
			//	policyComputedFor = new BitSet[numrobots];
				for (int i = 0; i < numrobots; i++) {
					stuckStateRobot.add(new ArrayList<Integer>());
					stuckStateRobotTime.add(new ArrayList<Integer>());
				//	policyComputedFor[i] = new BitSet(sumprod.numStates);
				}

				int oldtimecount = timecount;
				Map.Entry<Integer, Integer> temp = new AbstractMap.SimpleEntry(state, parent_state);
				st.push(temp);
				time.push(timecount);
				Object action = strat.getChoiceAction();
				if (action == null || "*".equals(action))
				{			stuckStates.set(state);
				
				(stuckStateRobot.get(whichRobot)).add(state);
				(stuckStateRobotTime.get(whichRobot)).add(timecount);
				if(!(policyComputedFor[whichRobot].get(state)))
				stuckStack.add(new robotTimeState(whichRobot,timecount,state,parent_state));
		
					continue;}
				//potential place to add the policyComputedFor 
				//TODO:
				policyComputedFor[whichRobot].set(state);
				boolean switched = false;
				while (!st.isEmpty()) {
					Map.Entry<Integer, Integer> temp1 = st.pop();
					state = temp1.getKey();
					addstatetopolmdp(state);
						Distribution prodDistr = new Distribution();
					Map.Entry<Integer, Integer> temp3;
					whichRobot = getIndValFromState(states.get(state), 0);
					timecount = (int) time.pop();
					while (pol[whichRobot].size() < (timecount + 1)) {
						pol[whichRobot].add(new ArrayList<Map.Entry<Integer, Integer>>());
					}

					if (!discovered.get(state) & !accStatesF.get(state)) {
						strat.initialise(state);
						action = strat.getChoiceAction();
						// if(action.toString() == "*")
						if (action == null || "*".equals(action)) {
							stuckStates.set(state);
							(stuckStateRobot.get(whichRobot)).add(state);
							(stuckStateRobotTime.get(whichRobot)).add(timecount);
							stuckStack.add(new robotTimeState(whichRobot,timecount,state,inits));
						}
						if (action.toString().contains("switch_er")) {
							oldtimecount = timecount;
							timecount = timecount - 1;
							switched = true;
						} else if (action.toString().contains("switch")) {
							oldtimecount = timecount;
							timecount = -1;
							switched = true;
						}
						if (switched)
							mainLog.print(oldtimecount + ":r" + whichRobot + ":" + state + ":" + action.toString());
						else
							mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":" + action.toString());

						discovered.set(state);
						choices = sumprod.getNumChoices(state);
						for (int c = 0; c < choices; c++) {
							if (action.equals(sumprod.getAction(state, c))) {
								Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(state, c);
								// double mprob = 0;
								prodDistr = new Distribution();
								while (iter.hasNext()) {
									Entry<Integer, Double> sp = iter.next();
									addstatetopolmdp(sp.getKey());
									prodDistr.add(polmap[sp.getKey()], sp.getValue());
									temp3 = new AbstractMap.SimpleEntry(sp.getKey(), state);
									st.push(temp3);
									// if (switcheder)
									// time.push(timecount);
									// else
									maxtimecount = Math.max(timecount + 1, maxtimecount);

									time.push(timecount + 1);
								}
							}
						}
					} else if (accStatesF.get(state)) {
						mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":!");
					}
					mainLog.print("\n");
					if (!switched)// if (timecount != -1)
						addToPolArray(whichRobot,timecount,temp1);//pol[whichRobot].get(timecount).add(temp1);
					else // if (!switcheder)
					{
						addToPolArray(whichRobot,oldtimecount,temp1);//pol[whichRobot].get(oldtimecount).add(temp1);
						switched = false;
					}
					polmdp.addActionLabelledChoice(polmap[state], prodDistr, action);
				}
				addTobigPol(inits_rts,pol);
			}

		}

		public void addstatetopolmdp(int state)
		{
			
			if (polmap[state]==statenotaddedvalue) {
				polmap[state] = polmdp.numStates; 
				polmdp.addState();}

		}
		public void printpolmap()
		{
			mainLog.println("Printing Pol Map");
			for(int i = 0; i< polmap.length; i++)
			{
				if(polmap[i]!=statenotaddedvalue)
				{
					mainLog.println(i+":"+polmap[i]);
				}
			}
		}
		public void updatePolicy(int numrobots, MDPSimple sumprod, Strategy strat, BitSet accStatesF,int parent_state,int istaterobot[]) {
			
			int timecount = 0; 
			pol = new ArrayList[numrobots];
			for (int i = 0; i < numrobots; i++) {
				pol[i] = new ArrayList();
			}
			robotTimeState inits_rts = null;
			Iterator<Integer> initstates = sumprod.getInitialStates().iterator();
			List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
			int whichRobot = -1;
			int numVar = sumprod.varList.getNumVars();

			// = new MDPSimple();
			timesUnfolded++;
			int inits;
			int state;
			int choices;

			Stack<Map.Entry<Integer, Integer>> st;
			Stack time;

			BitSet discovered;

			while (initstates.hasNext()) {

				//int 
				inits = (int) initstates.next();
		
				if(inits==69)
					mainLog.print("state = 69, check what happens");
				strat.initialise(inits);
				//int 
				state = inits;
				//int 
				choices = sumprod.getNumChoices(state);

				//Stack<Map.Entry<Integer, Integer>>
				st = new Stack();
				//Stack
				time = new Stack();

				//BitSet 
				discovered = new BitSet(sumprod.numStates);

//				maxtimecount = -1;
				whichRobot = getIndValFromState(states.get(state), 0);
				inits_rts = new robotTimeState(whichRobot,timecount,inits,parent_state);
				stuckStates = new BitSet(sumprod.numStates);
				stuckStates = new BitSet(sumprod.numStates);
				stuckStateRobot = new ArrayList<ArrayList<Integer>>(); // i'm giving up - lets find a new way to do
																		// this, this is wasting too much time!
				stuckStateRobotTime = new ArrayList<ArrayList<Integer>>();
			//	policyComputedFor = new BitSet[numrobots];
				for (int i = 0; i < numrobots; i++) {
					stuckStateRobot.add(new ArrayList<Integer>());
					stuckStateRobotTime.add(new ArrayList<Integer>());
				//	policyComputedFor[i] = new BitSet(sumprod.numStates);
				}
				Distribution prodDistr=null;// = new Distribution();


				int oldtimecount = timecount;
				Map.Entry<Integer, Integer> temp = new AbstractMap.SimpleEntry(state, parent_state);
				st.push(temp);
				time.push(timecount);
				Object action = strat.getChoiceAction();
				if (action == null || "*".equals(action))
				{			stuckStates.set(state);
				
				(stuckStateRobot.get(whichRobot)).add(state);
				(stuckStateRobotTime.get(whichRobot)).add(timecount);
				if(!(policyComputedFor[whichRobot].get(state)))
				{				ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> poltoadd = createNewPol(istaterobot,state,states,numVar,numrobots,whichRobot);
				addTobigPol(new robotTimeState(whichRobot,timecount,state,parent_state), poltoadd);
				int nextrobothere = (whichRobot+1)%numrobots;
				robotTimeState nextrobotstate = new robotTimeState(nextrobothere,timecount,istaterobot[nextrobothere],state);
				stuckStack.add(nextrobotstate);
				policyComputedFor[whichRobot].set(state);
				mainLog.println("\n============\nAdding "+nextrobotstate.toString()+" as successor to "+state+"\n============\n"); 
				addstatetopolmdp(state); 
				addstatetopolmdp(nextrobotstate.state);
				prodDistr = new Distribution(); 
				prodDistr.add(polmap[nextrobotstate.state], 1.0); //TODO: hardcoding this 
				polmdp.addActionLabelledChoice(polmap[state], prodDistr, "sf_"+whichRobot+"x"+state+"_"+nextrobothere+"x"+nextrobotstate.state);
				
				}
				//TODO: here instead of adding this state to the stuck stack, i should add its successor to the stuck stack 
					continue;}
				//potential place to add the policyComputedFor 
				//TODO:
				policyComputedFor[whichRobot].set(state);
				boolean switched = false;
				while (!st.isEmpty()) {
					Map.Entry<Integer, Integer> temp1 = st.pop();
					state = temp1.getKey();
					addstatetopolmdp(state);
					Map.Entry<Integer, Integer> temp3;
					whichRobot = getIndValFromState(states.get(state), 0);
					timecount = (int) time.pop();
					while (pol[whichRobot].size() < (timecount + 1)) {
						pol[whichRobot].add(new ArrayList<Map.Entry<Integer, Integer>>());
					}

					if (!discovered.get(state) & !accStatesF.get(state)) {
						strat.initialise(state);
						action = strat.getChoiceAction();
						// if(action.toString() == "*")
						if (action == null || "*".equals(action)) {
							stuckStates.set(state);
							(stuckStateRobot.get(whichRobot)).add(state);
							(stuckStateRobotTime.get(whichRobot)).add(timecount);
							stuckStack.add(new robotTimeState(whichRobot,timecount,state,inits));
						}
						if (action.toString().contains("switch_er")) {
							oldtimecount = timecount;
							timecount = timecount - 1;
							switched = true;
						} else if (action.toString().contains("switch")) {
							oldtimecount = timecount;
							timecount = -1;
							switched = true;
						}
						if (switched)
							mainLog.print(oldtimecount + ":r" + whichRobot + ":" + state + ":" + action.toString());
						else
							mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":" + action.toString());

						discovered.set(state);
						choices = sumprod.getNumChoices(state);
						for (int c = 0; c < choices; c++) {
							if (action.equals(sumprod.getAction(state, c))) {
								prodDistr = new Distribution();
								Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(state, c);
								// double mprob = 0;
								while (iter.hasNext()) {
									Entry<Integer, Double> sp = iter.next();
									addstatetopolmdp(sp.getKey());
									prodDistr.add(polmap[sp.getKey()], sp.getValue());
									temp3 = new AbstractMap.SimpleEntry(sp.getKey(), state);
									st.push(temp3);
									// if (switcheder)
									// time.push(timecount);
									// else
									maxtimecount = Math.max(timecount + 1, maxtimecount);

									time.push(timecount + 1);
								}
							}
						}
					} else if (accStatesF.get(state)) {
						mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":!");
					}
					mainLog.print("\n");
					if (!switched)// if (timecount != -1)
						addToPolArray(whichRobot,timecount,temp1);//pol[whichRobot].get(timecount).add(temp1);
					else // if (!switcheder)
					{
						addToPolArray(whichRobot,oldtimecount,temp1);//pol[whichRobot].get(oldtimecount).add(temp1);
						switched = false;
					}
					polmdp.addActionLabelledChoice(polmap[state], prodDistr, action);
				}
				addTobigPol(inits_rts,pol,istaterobot);
			}

		}

		public void getPolicyTime(int numrobots, MDPSimple sumprod, Strategy strat, BitSet accStatesF) {
			timesUnfolded++;
			pol = new ArrayList[numrobots];
			stuckStack = new LinkedList<robotTimeState>();
			for (int i = 0; i < numrobots; i++) {
				pol[i] = new ArrayList();
			}
			Iterator<Integer> initstates = sumprod.getInitialStates().iterator();

			// = new MDPSimple();

			ArrayList<Integer> newinitstates = new ArrayList<Integer>();
			while (initstates.hasNext()) {
				int inits = (int) initstates.next();
				strat.initialise(inits);
				int state = inits;
				int choices = sumprod.getNumChoices(state);

				Stack<Map.Entry<Integer, Integer>> st = new Stack();
				Stack time = new Stack();
				int timecount = 0;
				stuckStates = new BitSet(sumprod.numStates);	//fix this later please , this works cuz I have one initial state only
				stuckStates = new BitSet(sumprod.numStates);
				stuckStateRobot = new ArrayList<ArrayList<Integer>>(); // i'm giving up - lets find a new way to do
																		// this, this is wasting too much time!
				stuckStateRobotTime = new ArrayList<ArrayList<Integer>>();
				policyComputedFor = new BitSet[numrobots];
				for (int i = 0; i < numrobots; i++) {
					stuckStateRobot.add(new ArrayList<Integer>());
					stuckStateRobotTime.add(new ArrayList<Integer>());
					policyComputedFor[i] = new BitSet(sumprod.numStates);
				}

				BitSet discovered = new BitSet(sumprod.numStates);

				List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
				int numVar = sumprod.varList.getNumVars();
				int whichRobot = -1;
				maxtimecount = -1;
				int oldtimecount = timecount;
				whichRobot = getIndValFromState(states.get(state), 0);
				robotTimeState inits_rts = new robotTimeState(whichRobot,timecount,state,-1);
				currpol = inits_rts; //TODO: only cuz there is one initial state
				Map.Entry<Integer, Integer> temp = new AbstractMap.SimpleEntry(state, -1);
				st.push(temp);
				time.push(timecount);
				Object action = strat.getChoiceAction();
				if (action == null || "*".equals(action))
					continue;
				boolean switched = false;
				boolean switcheder = false;
				int parentstate = -1;
				while (!st.isEmpty()) {
					Map.Entry<Integer, Integer> temp1 = st.pop();
					state = temp1.getKey();
					addstatetopolmdp(state);
					Distribution prodDistr = new Distribution();
					Map.Entry<Integer, Integer> temp3;
					parentstate = temp1.getValue();
					whichRobot = getIndValFromState(states.get(state), 0);
					timecount = (int) time.pop();
					while (pol[whichRobot].size() < (timecount + 1)) {
						pol[whichRobot].add(new ArrayList<Map.Entry<Integer, Integer>>());
					}

					if (!discovered.get(state) & !accStatesF.get(state)) {
						strat.initialise(state);
						action = strat.getChoiceAction();
						// if(action.toString() == "*")
						if (action == null || "*".equals(action)) {
							stuckStates.set(state);
							// stuckStateRobot[whichRobot].set(state);
							(stuckStateRobot.get(whichRobot)).add(state);
							(stuckStateRobotTime.get(whichRobot)).add(timecount);
							stuckStack.add(new robotTimeState(whichRobot,timecount,state));

						}
						if (action.toString().contains("switch_er")) {
							oldtimecount = timecount;
							timecount = timecount - 1;
							switched = true;
							switcheder = true;
						} else if (action.toString().contains("switch")) {
							oldtimecount = timecount;
							timecount = -1;
							switched = true;
						}
						if (switched)
							mainLog.print(oldtimecount + ":r" + whichRobot + ":" + state + ":" + action.toString());
						else
							mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":" + action.toString());

						discovered.set(state);
						choices = sumprod.getNumChoices(state);
						for (int c = 0; c < choices; c++) {
							if (action.equals(sumprod.getAction(state, c))) {
								prodDistr = new Distribution();
								Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(state, c);
								// double mprob = 0;
								while (iter.hasNext()) {
									Entry<Integer, Double> sp = iter.next();
									temp3 = new AbstractMap.SimpleEntry(sp.getKey(), state);
									addstatetopolmdp(sp.getKey());
									prodDistr.add(polmap[sp.getKey()], sp.getValue());
									st.push(temp3);
									// if (switcheder)
									// time.push(timecount);
									// else
									maxtimecount = Math.max(timecount + 1, maxtimecount);

									time.push(timecount + 1);
								}
							}
						}
					} else if (accStatesF.get(state)) {
						mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":!");
					}
					mainLog.print("\n");
					if (!switched)// if (timecount != -1)
						addToPolArray(whichRobot,timecount,temp1);//						pol[whichRobot].get(timecount).add(temp1);
					else // if (!switcheder)
					{
						addToPolArray(whichRobot,oldtimecount,temp1);//pol[whichRobot].get(oldtimecount).add(temp1);
						switched = false;
						switcheder = false;
					}
					polmdp.addActionLabelledChoice(polmap[state], prodDistr, action);
					
				}
				addTobigPol(inits_rts,pol);
			}
		}
		public void addTobigPol(robotTimeState inits_rts,
				ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> poltoadd) {
			// TODO Auto-generated method stub
			if(inits_rts.state == 527)
				mainLog.println("527 here - this is added twice check why");
		bigpol.add(new AbstractMap.SimpleEntry(inits_rts, poltoadd));
			
		}
		
		public void addStatesToPol(int ps, int cstates[])
		{
			//adding all states with prob 1 
			addstatetopolmdp(ps); 
			String os = "";
			Distribution distr = new Distribution(); 
			//cheating here again just to test this because I really dont care about the values 
			double probval = 1.0/(double)(cstates.length); 
			for(int i = 0; i<cstates.length; i++)
			{
				addstatetopolmdp(cstates[i]);
				distr.add(polmap[cstates[i]], probval);
				os+=cstates[i]+"_";
			}
			polmdp.addActionLabelledChoice(polmap[ps], distr, "f"+ps+"x"+os);
		}
	};
	
	protected int[] getOtherStatesForRobot(ArrayList<Entry<Integer, Integer>> polti2,int rs, List<State> states,int numVar)
	{
		int otherRobotStates[] = new int[polti2.size()];
		double probs[] = new double[polti2.size()];

		for (int ors = 0; ors < polti2.size(); ors++) { // for each state at that time
			// for the next robot
			int otherstate = polti2.get(ors).getKey();
//			if (nextr < r) {
//				List<Integer> ci = findChangedStateIndices(states.get(rs), states.get(otherstate), numVar);
//				otherstate = findSameStateExcept(states.get(otherstate), ci, states, numVar, 1);
//			} else {// get the t'th stuck state
			//basically get the state for nextr such that the da bits are those of r. 
				int otherstatetemp = getMergedState(states.get(rs), states.get(otherstate), states, numVar);// findSameStateExcept(states.get(otherstate),
																											// ci,
																											// states,
				// numVar);
				// if its not -1 if it is just add a self loop //this is cheating // you should
				// just skip it
				if (!(otherstatetemp == -1))
					otherstate = otherstatetemp;
				else
					otherstate = rs;
//			}
			otherRobotStates[ors] = otherstate;
			if ((isfailState(states.get(otherstate), numVar))) {
				probs[ors] = 0.2; // hardcoding this
			} else
				probs[ors] = 0.8;
		}
		return otherRobotStates;
	}

	//the clean version of updateTeamAutomatonForState 
	//cuz that was just a misleading name okay 
	protected Object[] addFailSwitchGetUpdatedInitialStates(probPolicyTest policy,MDPSimple teamAutomatonNoSwitches,
			policyState current_robot_state, int combstates[][], int combNo,int numrobots)	//dont need the policy because already have that information in combstates
	{
		int numres = 2;
		Object res[] = new Object[numres]; 
		//This is a single link, there will only be one link to the state of the next robot 
		//The probabilities for this need to be calculated elsewhere 
		//So essentially this new state becomes the initial state 
		//There should be a link from the parent state to this current_robot_state (that should take care of the probabilities)
		int nextRobot = (current_robot_state.rnum+1)% numrobots; 
		int nextRobotState = policy.getAllStates().getMergedStateRobotMDP(current_robot_state.state,combstates[combNo][nextRobot]); 
		
		addlink(teamAutomatonNoSwitches,current_robot_state.state,nextRobotState,nextRobot);
		
		//update initial states 
		BitSet newInitStates[] = new BitSet[numrobots]; 
		int actualInitStates[] = new int[numrobots];
		//for each robot get a list of possible inital states, 
		//same robotmdp but different da values 
		for(int r=0; r<numrobots; r++)
		{
			if(r == current_robot_state.rnum)
				actualInitStates[r]=current_robot_state.state; 
			else
				actualInitStates[r] = policy.getAllStates().getMergedStateRobotMDP(current_robot_state.state,combstates[combNo][r]);
			newInitStates[r] = (BitSet)policy.getAllStates().getStatesWithSameRobotMDPState(actualInitStates[r]).clone();
			
		}
		res[0]= newInitStates; 
		res[1]=actualInitStates; 
		return res; 
		
		
	}
	
	//the clean version of updateTeamAutomatonForState 
	//cuz that was just a misleading name okay 
	protected Object[] addFailSwitchGetUpdatedInitialStates(probPolicyTest policy,MDPSimple teamAutomatonNoSwitches,
			policyState current_robot_state, int[] statesForRobots)	//dont need the policy because already have that information in combstates
	{
		int numres = 3;
		int numrobots = policy.tAutomaton.numRobots; 
		Object res[] = new Object[numres]; 
		//This is a single link, there will only be one link to the state of the next robot 
		//The probabilities for this need to be calculated elsewhere 
		//So essentially this new state becomes the initial state 
		//There should be a link from the parent state to this current_robot_state (that should take care of the probabilities)
		int nextRobot = (current_robot_state.rnum+1)% numrobots; 
		int nextRobotState = policy.getAllStates().getMergedStateRobotMDP(current_robot_state.state,statesForRobots[nextRobot]); 
		
		addlink(teamAutomatonNoSwitches,current_robot_state.state,nextRobotState,nextRobot);
		
		//update initial states 
		BitSet newInitStates[] = new BitSet[numrobots]; 
		int actualInitStates[] = new int[numrobots];
		//for each robot get a list of possible inital states, 
		//same robotmdp but different da values 
		for(int r=0; r<numrobots; r++)
		{
			if(r == current_robot_state.rnum)
				actualInitStates[r]=current_robot_state.state; 
			else
				actualInitStates[r] = policy.getAllStates().getMergedStateRobotMDP(current_robot_state.state,statesForRobots[r]);
			newInitStates[r] = (BitSet)policy.getAllStates().getStatesWithSameRobotMDPState(actualInitStates[r]).clone();
			
		}
		res[0]= newInitStates; 
		res[1]=actualInitStates; 
		res[2]=nextRobotState;
		return res; 
		
		
	}

	protected Object[] updateTeamAutomatonForState(ArrayList<Entry<Integer, Integer>> polti2, MDPSimple sumprod_new,
			List<State> states, int rs, int r, int nextr, int numVar, int combstates[][], int newstate, int numrobots) {
		int numres = 2;
		Object res[] = new Object[numres];
		
		int basicInitStates[] = new int[numrobots];
		int otherRobotStates[] = new int[polti2.size()];
		double probs[] = new double[polti2.size()];

		// add links
		for (int ors = 0; ors < polti2.size(); ors++) { // for each state at that time
			// for the next robot
			int otherstate = polti2.get(ors).getKey();
//			if (nextr < r) {
//				List<Integer> ci = findChangedStateIndices(states.get(rs), states.get(otherstate), numVar);
//				otherstate = findSameStateExcept(states.get(otherstate), ci, states, numVar, 1);
//			} else {// get the t'th stuck state
			//basically get the state for nextr such that the da bits are those of r. 
				int otherstatetemp = getMergedState(states.get(rs), states.get(otherstate), states, numVar);// findSameStateExcept(states.get(otherstate),
																											// ci,
																											// states,
				// numVar);
				// if its not -1 if it is just add a self loop //this is cheating // you should
				// just skip it
				if (!(otherstatetemp == -1))
					otherstate = otherstatetemp;
				else
					otherstate = rs;
//			}
			otherRobotStates[ors] = otherstate;
			if ((isfailState(states.get(otherstate), numVar))) {
				probs[ors] = 0.2; // hardcoding this
			} else
				probs[ors] = 0.8;
		}
		// the link is not from rs to otherRobotStates but from rs to otherRobotStates
		// where the da for rs is the same.
		// each time we have a new init state, we have new links

		addlink(sumprod_new, rs, otherRobotStates, probs, nextr);

		// update the switches
		// TODO: update switches here
		// updateswitches()
		BitSet initstates_now[] = new BitSet[numrobots];
		// for each robot that is not next r get the states at time t
		// for nextr get the states at time nextrt
		int sn_numStates = sumprod_new.numStates;
		for (int rt = 0; rt < numrobots; rt++) {
			initstates_now[rt] = new BitSet(sn_numStates);

			if (rt == r) // if i'm at the current robot (whose thing has failed and is
			// the one i'm starting in, I only need one state)
			{
				basicInitStates[rt]=rs;
				initstates_now[rt].or(getStatesWithSameMDPState(states, rs, numVar));
			} else {
				int state_now = combstates[newstate][rt];
				basicInitStates[rt]=getMergedState(states.get(rs), states.get(state_now), states, numVar);
				initstates_now[rt].or(getStatesWithSameMDPState(states, state_now, numVar));

				// }
			}
		}

		res[0]=initstates_now; 
		res[1]=basicInitStates;
		return res;//initstates_now;
	}
	protected void computeProbAndUpdateTeamAutomatonForState(probPolicyTest policy, 
			policyState currState,int[] stateForRobot ) throws PrismException
	{
		MDPSimple newTeamAutomaton = policy.tAutomaton.getNewTeamAutomatonWithSwitches(currState.state);
		
		//add the links 
		Object updatedInitStates[]=addFailSwitchGetUpdatedInitialStates(
				policy,newTeamAutomaton,currState,stateForRobot);

		BitSet updatedPossibleInitStatesForSwitches[] = (BitSet[])updatedInitStates[0]; 
		int updatedInitStatesRobot[] = (int[])updatedInitStates[1];
		int nextRobotstate = (int)updatedInitStates[2];

		//update switches using this information 
		updateSwitches(policy.tAutomaton.switchstatesRobots,policy.tAutomaton.numRobots,
				policy.tAutomaton.allStates.states,
				policy.tAutomaton.allStates.numVar,updatedPossibleInitStatesForSwitches,
				newTeamAutomaton);
		//now we have the updated automaton 
	
		newTeamAutomaton.findDeadlocks(true);
		//save stuff just for debugging 
		newTeamAutomaton.exportToDotFile(saveplace + "sumprod_new" + currState.state + ".dot");
		newTeamAutomaton.exportToPrismExplicitTra(saveplace + "sumprod_new" + currState + ".tra");
		
		//compute the probabilities again 
		ModelCheckerResult res = computeUntilProbs(newTeamAutomaton, policy.tAutomaton.statesToAvoid,
				policy.tAutomaton.finalAccStates,false);
		
		//add the result to the policy 
		policy.unfoldPolicy(newTeamAutomaton, res.strat, 0, currState.pstate, nextRobotstate,updatedInitStatesRobot);
		
	}

	protected void updateTeamAutomatonForStatesClean(probPolicyTest policy, MDPSimple teamAutomaton,
			policyState currState) throws PrismException
	{
		//given a policy (which has a team automaton with no switches and other info)
		//teamAutomaton and current state update the team automaton iteratively and 
		//generate policies for each state 
		
		//step 1 get all the indices that have changed at this point 
		//outer most array list - robot 
		//next one - states for each robot at time of currState 
		//last array list - index changed and corresponding value at final state 
		//need to double check this okay 
		ArrayList<ArrayList<Entry<policyState, ArrayList<Entry<Integer, Integer>>>>> 
		changedIndicesForRobotStates = policy.getChangedIndicesAcrossPolicyAtTime(currState);
		
		//from the changed Indices generate a list of possible combinations 
		Object combstateslist[] = createRobotCombs(policy.tAutomaton.numRobots,currState.rnum,
				changedIndicesForRobotStates);
		
		//okay I know this is just weird but this is the best I've got right now okay 
		//and if I fix more I just get more confused!! so i'm going to stop here okay 
		//okay so here we're taking these combinations and just getting new states 
		//combstates serves as a list of states
		//newstates are the new states for this robot 
		ArrayList<ArrayList<Entry<Integer, Integer>>> combinations = (ArrayList<ArrayList<Entry<Integer, Integer>>>) combstateslist[0];
		int combstates[][] = (int[][]) combstateslist[1];
		double combstatesProbs[] = (double[])combstateslist[2];
		// now lets use these combinations to get new states
		int newstates[] = policy.getStatesFromCombinations(combinations, currState.state); 
		
		policy.addLinkInPolicyMDP(newstates, combstatesProbs, currState.state, null,true);
		//updateTeamAutomatonForState
		for(int s=0; s<newstates.length; s++)
		{
			//create the new state //same robot, same time, different state, parent state = this state
			policyState stateToCompute = new policyState(currState.rnum,currState.time,newstates[s],currState.state,null,currState.associatedStartState);
			//make sure you have the right policy state 
			policy.setCurrentPolicyToPolicyFromAllPolicies(currState.associatedStartState);
			//update team automaton and compute probability 
			computeProbAndUpdateTeamAutomatonForState(policy,stateToCompute,combstates[s]);
			//TODO: no notion of whether this policy has been computed before or not 
			//which needs to be fixed otherwise things wont work 
		}

		
		
		//TODO: come back here1 
		
		
	}
	protected void unfoldPolicyClean(ModelCheckerResult res, MDPSimple sumprod, BitSet accStatesF, 
			int numrobots,
			MDPSimple sumprod_noswitches, BitSet[] switches, BitSet badstates) throws PrismException
	{
		Strategy strat = res.strat;	//save the strategy
		probPolicyTest policy = new probPolicyTest(numrobots,accStatesF,badstates,
				sumprod_noswitches,switches,mainLog);	//initalize policy variables not the actual policy
		policy.unfoldPolicy(sumprod, strat, 0, -1);		//compute the policy now , the time is 0 and there is no parent state
		
		//while all policies have not been computed 
		while(!policy.stuckStatesQueue.isEmpty())
		{
			policyState computeForState = policy.stuckStatesQueue.remove(); 
			
			//set the current policy to be the one for this particular state 
			//so we know where to get our information from 
			policy.setCurrentPolicyToPolicyFromAllPolicies(computeForState.associatedStartState);
			
			updateTeamAutomatonForStatesClean(policy, sumprod,computeForState);
			
		}
		policy.savePolicyMDPToFile(saveplace, "policyMDP");
		
	}
	protected void updateTeamAutomatonForStates(policyUnfolded polc,MDPSimple sumprod, int numrobots,
			MDPSimple sumprod_noswitches, BitSet[] switches,robotTimeState rts, BitSet badstates,BitSet accStatesF)
			throws PrismException {

		ArrayList<ArrayList<Entry<Integer, Integer>>>[] pol = polc.pol;
		BitSet stuckStates = (BitSet) polc.stuckStates.clone();
		int maxtimecount = polc.maxtimecount;
		List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
		int numVar = sumprod.varList.getNumVars();
		int curr_robot_state=rts.state; int robot_num=rts.rnum; int curr_time=rts.time;
		int rs = curr_robot_state;
		int t = curr_time;
		int r = robot_num;
		ArrayList<ArrayList<Entry<Integer, Integer>>> polt = pol[r];
		ArrayList<Entry<Integer, Integer>> polti=null;
		if(polt.size()>t)
			polti = polt.get(t);
		else
		{
			mainLog.println("Error here- check - array not as big as expected");
			t = polt.size()-1; 
			if(t>=0)
				polti = polt.get(t);
		}
		mainLog.print(rs + ",");
		int nextr = (r + 1) % numrobots;

		ArrayList<Entry<Integer, Integer>> polti2;
		// exploiting the chain thing. if nextr < r
		// then we know that most of our tasks in r have not been acheived
		// so we need to find the change in state between r & nextr
		// as well as r and r's ps
		int nextrt = t;
		if (!(pol[nextr].size() > nextrt)) {
			nextrt = pol[nextr].size() - 1;
			if (nextrt==-1)
			{
				mainLog.println("Error here - check - array not as big as expected  ");
			}
		}
		Object cirobots[] = getChangedIndicesAcrossPolicy(pol, numrobots, t, r, nextr, states, numVar);
//		ArrayList<ArrayList<Entry<Integer, ArrayList<Entry<Integer, Integer>>>>>  changedIndicesForRobotStates = ppt.getChangedIndicesAcrossPolicyAtTime(new policyState(r,t,rs,rts.pstate));
		Object combstateslist[] = createRobotCombs(numrobots, r, cirobots);
//		Object combstateslist[] = createRobotCombs(numrobots, r, changedIndicesForRobotStates);

		ArrayList<ArrayList<Entry<Integer, Integer>>> combinations = (ArrayList<ArrayList<Entry<Integer, Integer>>>) combstateslist[0];
		int combstates[][] = (int[][]) combstateslist[1];
		// now lets use these combinations to get new states
		int newstates[] = getStatesFromCombinations(combinations, states, rs);
		//ppt.getStatesFromCombinations(combinations, rs); 

		polti2 = pol[nextr].get(nextrt);
		if(polti2.size()>2)
		{
			mainLog.println("Look here");
		}
		int otherRobotStates[] = new int[polti2.size()];
		double probs[] = new double[polti2.size()];
		polc.addStatesToPol(rs, newstates); //this is adding states to the mdp pol mdp 
		// we now have newstates len init states
		for (int newstate = 0; newstate < newstates.length; newstate++) {
			rs = newstates[newstate];
			
			if (!polc.policyComputedFor[r].get(rs)) {
				mainLog.println("Computing Policy for "+rs);
			MDPSimple sumprod_new = new MDPSimple(sumprod_noswitches);
			sumprod_new.setVarList((VarList) sumprod_noswitches.varList.clone());
			sumprod_new.setStatesList(sumprod_noswitches.getStatesList());
			sumprod_new.clearInitialStates();
			sumprod_new.addInitialState(rs);
			//so nothing says you're computing the policy for this state at the moment and this robot 

			Object initstateinfo[]= updateTeamAutomatonForState(polti2, sumprod_new, states, rs, r, nextr, numVar,
					combstates, newstate, numrobots);
//			policyState temprts = new policyState(rts.rnum,rts.time,rs,rts.state);
//				Object updatedInitStates[]=addFailSwitchGetUpdatedInitialStates(ppt,sumprod_new,temprts,combstates,newstate,numrobots);

			BitSet initstates_now[] = (BitSet[])initstateinfo[0];
			int initstates_robot[]= (int[])initstateinfo[1];
//			BitSet updatedPossibleInitStatesForSwitches[] = (BitSet[])updatedInitStates[0]; 
//			int updatedInitStatesRobot[] = (int[])updatedInitStates[1];
			updateSwitches(switches, numrobots, states, numVar, initstates_now,sumprod_new);
//					updatedPossibleInitStatesForSwitches, sumprod_new);
			sumprod_new.findDeadlocks(true);
			sumprod_new.exportToDotFile(saveplace + "sumprod_new" + rs + ".dot");
			sumprod_new.exportToPrismExplicitTra(saveplace + "sumprod_new" + rs + ".tra");
			ModelCheckerResult res = computeUntilProbs(sumprod_new, badstates, accStatesF, false);
			polc.updatePolicy(numrobots, sumprod_new, res.strat,
					accStatesF,curr_robot_state,initstates_robot); //so the time should be the next bit right ? so t+1 
//			ppt.unfoldPolicy(sumprod_new, res.strat,0 , curr_robot_state); //the time is always 0 regardless so do i even need it here ? 
		}
			else
			{
				if(!polc.bigpolHasState(rs))
				{
					ArrayList<ArrayList<ArrayList<Entry<Integer, Integer>>>> poltoadd = polc.createNewPol(combstates,newstates,newstate,states,numVar,numrobots,r);
					//createNewPol(int initstates[],int curr_robotstate, List<State> states,int numVar,int numrobots,int curr_robot)
					polc.addTobigPol(new robotTimeState(r,t,rs,curr_robot_state), poltoadd);
					polc.policyComputedFor[r].set(rs);
					int otherRobotstates[] = getOtherStatesForRobot(polti2,rs,states,numVar); 
					for(int i=0; i<otherRobotstates.length; i++)
					{
						int os = otherRobotstates[i];
						int rnum = getIndValFromState(states.get(os), 0);
						if(!polc.policyComputedFor[rnum].get(os))
						{robotTimeState currstate = new robotTimeState(rnum,t,os,rs);
						polc.stuckStack.add(currstate);	//kind of hardcoding this bit, which is a problem, the nextrt bit 
						}
						//but now we need something about the init states at this time 
						//basically I want that line from bigpol which has all the info for this state 
						}
					
					}
				}
		}
			
		
		polc.print();
		polc.printBigPol();
	}


	protected void unfoldPolicy_new_bits(ModelCheckerResult res, MDPSimple sumprod, BitSet accStatesF, int numrobots,
			MDPSimple sumprod_noswitches, BitSet[] switches, BitSet badstates, BitSet[] oldinitstates, boolean addLinks)
			throws PrismException {

		Strategy strat = res.strat;
		policyUnfolded polc = new policyUnfolded(numrobots, sumprod, strat, accStatesF);
//		probPolicyTest newPolVar=new probPolicyTest(numrobots, accStatesF, badstates,
//				sumprod_noswitches,switches,mainLog);
//		newPolVar.unfoldPolicy(sumprod, strat, 0, -1);
		ArrayList<ArrayList<Entry<Integer, Integer>>>[] pol = polc.pol;
		BitSet stuckStates = (BitSet) polc.stuckStates.clone();
		ArrayList<ArrayList<Integer>> stuckStatesRobot = polc.stuckStateRobot;
		ArrayList<ArrayList<Integer>> stuckStatesRobotTime = polc.stuckStateRobotTime;
		int maxtimecount = polc.maxtimecount;
		List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
		int numVar = sumprod.varList.getNumVars();
		MDPSimple sumprod_new;
		mainLog.println("Stuck States: " + stuckStates.toString());
		// link a stuck state with some prob to another stuck state or a not stuck state
		// this will be the round robin approach
		// if stuck state, save state and robot number
		// for next robot link this state to next robot's states
		if (addLinks) {
//			for (int robot_num = 0; robot_num < polc.stuckStateRobot.size(); robot_num++) {
//				for (int stnum = 0; stnum < polc.stuckStateRobot.get(robot_num).size(); stnum++) {
//					int ss = polc.stuckStateRobot.get(robot_num).get(stnum);
//					if(!polc.policyComputedFor[robot_num].get(ss)) {
//					int curr_time = stuckStatesRobotTime.get(robot_num).get(stnum);
			while(!polc.stuckStack.isEmpty()) {
					robotTimeState rts = polc.stuckStack.remove(); 
					ArrayList<ArrayList<Entry<Integer, Integer>>>[] oldpol=null;
					if (rts.pstate==-1)
						oldpol= polc.copyPol(polc.pol);
				
				int ss = rts.state; 
				if(ss == 69)
					mainLog.println("69 here");
				int robot_num = rts.rnum; 
				int curr_time = rts.time; 
				if (!polc.policyComputedFor[robot_num].get(ss)) {
					polc.policyComputedFor[robot_num].set(ss);
					mainLog.println("Computing for "+rts.toString_());
					if(rts.pstate!=-1)
						{mainLog.println("Getting policy for "+rts.pstate);
						polc.copyPolForStateToPol(rts.pstate);
						
						}
					updateTeamAutomatonForStates(polc, sumprod, numrobots, sumprod_noswitches, switches, rts,badstates,accStatesF);
					if(rts.pstate==-1)
					polc.pol = polc.copyPol(oldpol);
					}
			}
					// for each stuckstate update team automaton

					// res = computeUntilProbs(sumprod_new, badstates, accStatesF, false);
					// unfoldPolicy_new(res, sumprod_new, accStatesF, numrobots, sumprod_noswitches,
					// switches, badstates, initstates_now,
					// true);
//				}
//			}
			
			polc.printpolmap();
			polc.polmdp.exportToDotFile(saveplace+"policytry.dot");
			polc.polmdp.exportToPrismExplicitTra(saveplace+"policytry.tra");

		}
	}

	protected void unfoldPolicy_new(ModelCheckerResult res, MDPSimple sumprod, BitSet accStatesF, int numrobots,
			MDPSimple sumprod_noswitches, BitSet[] switches, BitSet badstates, BitSet oldinitstates[], boolean addLinks)
			throws PrismException {

		Strategy strat = res.strat;
		ArrayList<ArrayList<Map.Entry<Integer, Integer>>> pol[] = new ArrayList[numrobots];
		for (int i = 0; i < numrobots; i++) {
			pol[i] = new ArrayList();
		}
		Iterator<Integer> initstates = sumprod.getInitialStates().iterator();

		MDPSimple sumprod_new; // = new MDPSimple();

		ArrayList<Integer> newinitstates = new ArrayList<Integer>();
		while (initstates.hasNext()) {
			int inits = (int) initstates.next();
			strat.initialise(inits);
			int state = inits;
			int choices = sumprod.getNumChoices(state);

			Stack<Map.Entry<Integer, Integer>> st = new Stack();
			Stack time = new Stack();
			int timecount = 0;
			BitSet stuckStates = new BitSet(sumprod.numStates);
			BitSet stuckStateRobot[] = new BitSet[numrobots];
			for (int i = 0; i < numrobots; i++) {
				stuckStateRobot[i] = new BitSet(sumprod.numStates);
			}
			BitSet discovered = new BitSet(sumprod.numStates);

			List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
			int numVar = sumprod.varList.getNumVars();
			int whichRobot = -1;
			int maxtimecount = -1;
			int oldtimecount = timecount;
			Map.Entry<Integer, Integer> temp = new AbstractMap.SimpleEntry(state, -1);
			st.push(temp);
			time.push(timecount);
			Object action = strat.getChoiceAction();
			if (action == null || "*".equals(action))
				continue;
			boolean switched = false;
			boolean switcheder = false;
			int parentstate = -1;
			while (!st.isEmpty()) {
				Map.Entry<Integer, Integer> temp1 = st.pop();
				state = temp1.getKey();
				Map.Entry<Integer, Integer> temp3;
				parentstate = temp1.getValue();
				whichRobot = getIndValFromState(states.get(state), 0);
				timecount = (int) time.pop();
				while (pol[whichRobot].size() < (timecount + 1)) {
					pol[whichRobot].add(new ArrayList<Map.Entry<Integer, Integer>>());
				}

				if (!discovered.get(state) & !accStatesF.get(state)) {
					strat.initialise(state);
					action = strat.getChoiceAction();
					// if(action.toString() == "*")
					if (action == null || "*".equals(action)) {
						stuckStates.set(state);
						stuckStateRobot[whichRobot].set(state);
					}
					if (action.toString().contains("switch_er")) {
						oldtimecount = timecount;
						timecount = timecount - 1;
						switched = true;
						switcheder = true;
					} else if (action.toString().contains("switch")) {
						oldtimecount = timecount;
						timecount = -1;
						switched = true;
					}
					if (switched)
						mainLog.print(oldtimecount + ":r" + whichRobot + ":" + state + ":" + action.toString());
					else
						mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":" + action.toString());

					discovered.set(state);
					choices = sumprod.getNumChoices(state);
					for (int c = 0; c < choices; c++) {
						if (action.equals(sumprod.getAction(state, c))) {
							Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(state, c);
							// double mprob = 0;
							while (iter.hasNext()) {
								Entry<Integer, Double> sp = iter.next();
								temp3 = new AbstractMap.SimpleEntry(sp.getKey(), state);
								st.push(temp3);
								// if (switcheder)
								// time.push(timecount);
								// else
								maxtimecount = Math.max(timecount + 1, maxtimecount);

								time.push(timecount + 1);
							}
						}
					}
				} else if (accStatesF.get(state)) {
					mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":!");
				}
				mainLog.print("\n");
				if (!switched)// if (timecount != -1)
					pol[whichRobot].get(timecount).add(temp1);
				else // if (!switcheder)
				{
					pol[whichRobot].get(oldtimecount).add(temp1);
					switched = false;
					switcheder = false;
				}
			}
			mainLog.println("Stuck States: " + stuckStates.toString());
			// link a stuck state with some prob to another stuck state or a not stuck state
			// this will be the round robin approach
			// if stuck state, save state and robot number
			// for next robot link this state to next robot's states
			if (addLinks) {
				List<Integer> ci = null;
				for (int t = 0; t <= maxtimecount; t++) {
					mainLog.print(t + "\t");
					for (int r = 0; r < numrobots; r++) {
						ArrayList<ArrayList<Entry<Integer, Integer>>> polt = pol[r];

						if (polt.size() > t) {
							mainLog.print(r + ":");
							ArrayList<Entry<Integer, Integer>> polti = polt.get(t);
							for (int s = 0; s < polti.size(); s++) { // go over all the states at this time
								int rs = polti.get(s).getKey();

								mainLog.print(rs + ",");
								if (stuckStates.get(rs)) {
									int ps = polti.get(s).getValue(); // if stuck state get its parent state
									// //because I know there are only two states
									// //TODO: figure out a smarter way to do this
									// newinitstates.add(rs);

									// i know this is a repeat but needs to happen
									int nextr = (r + 1) % numrobots;

									ArrayList<Entry<Integer, Integer>> polti2;
									// exploiting the chain thing. if nextr < r
									// then we know that most of our tasks in r have not been acheived
									// so we need to find the change in state between r & nextr
									// as well as r and r's ps
									int nextrt = t;
									if (!(pol[nextr].size() > nextrt)) {
										nextrt = pol[nextr].size() - 1;

									}
									Object cirobots[] = getChangedIndicesAcrossPolicy(pol, numrobots, t, r, nextr,
											states, numVar);
									Object combstateslist[] = createRobotCombs(numrobots, r, cirobots);
									ArrayList<ArrayList<Entry<Integer, Integer>>> combinations = (ArrayList<ArrayList<Entry<Integer, Integer>>>) combstateslist[0];
									int combstates[][] = (int[][]) combstateslist[1];
									// now lets use these combinations to get new states
									int newstates[] = getStatesFromCombinations(combinations, states, rs);

									polti2 = pol[nextr].get(nextrt);
									int otherRobotStates[] = new int[polti2.size()];
									double probs[] = new double[polti2.size()];

									// we now have newstates len init states
									for (int newstate = 0; newstate < newstates.length; newstate++) {
										rs = newstates[newstate];

										sumprod_new = new MDPSimple(sumprod_noswitches);
										sumprod_new.setVarList((VarList) sumprod_noswitches.varList.clone());
										sumprod_new.setStatesList(sumprod_noswitches.getStatesList());
										sumprod_new.clearInitialStates();
										sumprod_new.addInitialState(rs);
										// add links
										for (int ors = 0; ors < polti2.size(); ors++) { // for each state at that time
																						// for the next robot
											int otherstate = polti2.get(ors).getKey();
											if (nextr < r) {
												ci = findChangedStateIndices(states.get(rs), states.get(otherstate),
														numVar);
												otherstate = findSameStateExcept(states.get(otherstate), ci, states,
														numVar, 1);
											} else {// get the t'th stuck state
												// int ss =
												// combstates[newstate][nextr];//getIthSetBit(stuckStateRobot[nextr],
												// nextrt - 1);
												// ci = findChangedStateIndices(states.get(rs), states.get(ss), numVar);
												int otherstatetemp = getMergedState(states.get(rs),
														states.get(otherstate), states, numVar);// findSameStateExcept(states.get(otherstate),
																								// ci, states,
												// numVar);
												// if its not -1 if it is just add a self loop //this is cheating // you
												// should just skip it
												if (!(otherstatetemp == -1))
													otherstate = otherstatetemp;
												else
													otherstate = rs;
											}
											otherRobotStates[ors] = otherstate;
											if ((isfailState(states.get(otherstate), numVar))) {
												probs[ors] = 0.2; // hardcoding this
											} else
												probs[ors] = 0.8;
										}
										// what if you cant find any of the states ?? otherRobotStates = -1 ? skip this
										// bit then
										// if (otherRobotStates[0] == -1)
										// continue;
										// the link is not from rs to otherRobotStates but from rs to otherRobotStates
										// where the da for rs is the same.
										// each time we have a new init state, we have new links

										addlink(sumprod_new, rs, otherRobotStates, probs, nextr, ps);
										// }

										// update the switches
										// TODO: update switches here
										// updateswitches()
										BitSet initstates_now[] = new BitSet[numrobots];
										// for each robot that is not next r get the states at time t
										// for nextr get the states at time nextrt
										int sn_numStates = sumprod_new.numStates;
										for (int rt = 0; rt < numrobots; rt++) {
											initstates_now[rt] = new BitSet(sn_numStates);
											// TODO; add the get alll states function here
											int ct = t; // current time for this robot
											if (rt == nextr)
												ct = nextrt;
											if (pol[rt].size() <= ct)
												ct = pol[rt].size() - 1;
											if (rt == r) // if i'm at the current robot (whose thing has failed and is
															// the one i'm starting in, I only need one state)
											{
												initstates_now[rt].or(getStatesWithSameMDPState(states, rs, numVar));
											} else {
												// ArrayList<Entry<Integer, Integer>> nstates = (pol[rt]).get(ct);
												// We also want the first state at time t=0 so that we can exclude any
												// states that do not have those DAVals
												// int state_time0 = (((pol[rt]).get(0)).get(0)).getKey();
												// for(int ns_ = 0; ns_ < nstates.size(); ns_++)
												// {
												// int state_now = nstates.get(ns_).getKey();
												int state_now = combstates[newstate][rt];
												initstates_now[rt]
														.or(getStatesWithSameMDPState(states, state_now, numVar));

												// }
											}
										}

										updateSwitches(switches, numrobots, states, numVar, initstates_now,
												sumprod_new);
										sumprod_new.findDeadlocks(true);
										sumprod_new.exportToDotFile(saveplace + "sumprod_new" + rs + ".dot");
										sumprod_new.exportToPrismExplicitTra(saveplace + "sumprod_new" + rs + ".tra");

										res = computeUntilProbs(sumprod_new, badstates, accStatesF, false);
										unfoldPolicy_new(res, sumprod_new, accStatesF, numrobots, sumprod_noswitches,
												switches, badstates, initstates_now, true);
									}
								}

							}
							// }
							mainLog.print("\t");
						}
					}
					mainLog.print("\n");
				}

				sumprod.findDeadlocks(true);
			}
		}
		sumprod.clearInitialStates();
		for (int s = 0; s < newinitstates.size(); s++)
			sumprod.addInitialState(newinitstates.get(s));
	}

	protected void unfoldPolicy_new_old(ModelCheckerResult res, MDPSimple sumprod, BitSet accStatesF, int numrobots,
			MDPSimple sumprod_noswitches, BitSet[] switches, BitSet badstates, BitSet oldinitstates[], boolean addLinks)
			throws PrismException {

		Strategy strat = res.strat;
		ArrayList<ArrayList<Map.Entry<Integer, Integer>>> pol[] = new ArrayList[numrobots];
		for (int i = 0; i < numrobots; i++) {
			pol[i] = new ArrayList();
		}
		Iterator<Integer> initstates = sumprod.getInitialStates().iterator();

		MDPSimple sumprod_new; // = new MDPSimple();

		ArrayList<Integer> newinitstates = new ArrayList<Integer>();
		while (initstates.hasNext()) {
			int inits = (int) initstates.next();
			strat.initialise(inits);
			int state = inits;
			int choices = sumprod.getNumChoices(state);

			Stack<Map.Entry<Integer, Integer>> st = new Stack();
			Stack time = new Stack();
			int timecount = 0;
			BitSet stuckStates = new BitSet(sumprod.numStates);
			BitSet stuckStateRobot[] = new BitSet[numrobots];
			for (int i = 0; i < numrobots; i++) {
				stuckStateRobot[i] = new BitSet(sumprod.numStates);
			}
			BitSet discovered = new BitSet(sumprod.numStates);

			List<State> states = sumprod.statesList; // i know this , the robot index is always the first one
			int numVar = sumprod.varList.getNumVars();
			int whichRobot = -1;
			int maxtimecount = -1;
			int oldtimecount = timecount;
			Map.Entry<Integer, Integer> temp = new AbstractMap.SimpleEntry(state, -1);
			st.push(temp);
			time.push(timecount);
			Object action = strat.getChoiceAction();
			if (action == null || "*".equals(action))
				continue;
			boolean switched = false;
			boolean switcheder = false;
			int parentstate = -1;
			while (!st.isEmpty()) {
				Map.Entry<Integer, Integer> temp1 = st.pop();
				state = temp1.getKey();
				Map.Entry<Integer, Integer> temp3;
				parentstate = temp1.getValue();
				whichRobot = getIndValFromState(states.get(state), 0);
				timecount = (int) time.pop();
				while (pol[whichRobot].size() < (timecount + 1)) {
					pol[whichRobot].add(new ArrayList<Map.Entry<Integer, Integer>>());
				}

				if (!discovered.get(state) & !accStatesF.get(state)) {
					strat.initialise(state);
					action = strat.getChoiceAction();
					// if(action.toString() == "*")
					if (action == null || "*".equals(action)) {
						stuckStates.set(state);
						stuckStateRobot[whichRobot].set(state);
					}
					if (action.toString().contains("switch_er")) {
						oldtimecount = timecount;
						timecount = timecount - 1;
						switched = true;
						switcheder = true;
					} else if (action.toString().contains("switch")) {
						oldtimecount = timecount;
						timecount = -1;
						switched = true;
					}
					if (switched)
						mainLog.print(oldtimecount + ":r" + whichRobot + ":" + state + ":" + action.toString());
					else
						mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":" + action.toString());

					discovered.set(state);
					choices = sumprod.getNumChoices(state);
					for (int c = 0; c < choices; c++) {
						if (action.equals(sumprod.getAction(state, c))) {
							Iterator<Entry<Integer, Double>> iter = sumprod.getTransitionsIterator(state, c);
							// double mprob = 0;
							while (iter.hasNext()) {
								Entry<Integer, Double> sp = iter.next();
								temp3 = new AbstractMap.SimpleEntry(sp.getKey(), state);
								st.push(temp3);
								// if (switcheder)
								// time.push(timecount);
								// else
								maxtimecount = Math.max(timecount + 1, maxtimecount);

								time.push(timecount + 1);
							}
						}
					}
				} else if (accStatesF.get(state)) {
					mainLog.print(timecount + ":r" + whichRobot + ":" + state + ":!");
				}
				mainLog.print("\n");
				if (!switched)// if (timecount != -1)
					pol[whichRobot].get(timecount).add(temp1);
				else // if (!switcheder)
				{
					pol[whichRobot].get(oldtimecount).add(temp1);
					switched = false;
					switcheder = false;
				}
				// else if (switcheder)
				// {
				// pol[whichRobot].get(timecount).add(state); switcheder=false;
				// }
			}
			mainLog.println("Stuck States: " + stuckStates.toString());
			// link a stuck state with some prob to another stuck state or a not stuck state
			// this will be the round robin approach
			// if stuck state, save state and robot number
			// for next robot link this state to next robot's states
			if (addLinks) {
				// sumprod.clearInitialStates();
				List<Integer> ci = null;
				for (int t = 0; t <= maxtimecount; t++) {
					mainLog.print(t + "\t");
					for (int r = 0; r < numrobots; r++) {
						ArrayList<ArrayList<Entry<Integer, Integer>>> polt = pol[r];

						if (polt.size() > t) {
							mainLog.print(r + ":");
							ArrayList<Entry<Integer, Integer>> polti = polt.get(t);
							for (int s = 0; s < polti.size(); s++) { // go over all the states at this time
								int rs = polti.get(s).getKey();

								mainLog.print(rs + ",");
								if (stuckStates.get(rs)) {
									int ps = polti.get(s).getValue(); // if stuck state get its parent state
									// //because I know there are only two states
									// //TODO: figure out a smarter way to do this
									newinitstates.add(rs);

									// i know this is a repeat but needs to happen
									int nextr = (r + 1) % numrobots;

									ArrayList<Entry<Integer, Integer>> polti2;
									// exploiting the chain thing. if nextr < r
									// then we know that most of our tasks in r have not been acheived
									// so we need to find the change in state between r & nextr
									// as well as r and r's ps
									int nextrt = t;
									if (!(pol[nextr].size() > nextrt)) {
										nextrt = pol[nextr].size() - 1;

									}
									Object cirobots[] = getChangedIndicesAcrossPolicy(pol, numrobots, t, r, nextr,
											states, numVar);

									createRobotCombs(numrobots, r, cirobots);

									polti2 = pol[nextr].get(nextrt);
									int otherRobotStates[] = new int[polti2.size()];
									double probs[] = new double[polti2.size()];
									// add link
									for (int ors = 0; ors < polti2.size(); ors++) {
										int otherstate = polti2.get(ors).getKey();
										// if (ps!= -1)
										// { //ci=
										// findChangedStateIndices(states.get(rs),states.get(otherstate),numVar);
										if (nextr < r) {
											ci = findChangedStateIndices(states.get(rs), states.get(otherstate),
													numVar);
											otherstate = findSameStateExcept(states.get(otherstate), ci, states, numVar,
													1);
										} else {// get the t'th stuck state
											int ss = getIthSetBit(stuckStateRobot[nextr], nextrt - 1);
											ci = findChangedStateIndices(states.get(rs), states.get(ss), numVar);
											otherstate = findSameStateExcept(states.get(otherstate), ci, states,
													numVar);
										}
										// }
										otherRobotStates[ors] = otherstate;
										if (isfailState(states.get(otherstate), numVar)) {
											probs[ors] = 0.2; // hardcoding this
										} else
											probs[ors] = 0.8;
									}
									// the link is not from rs to otherRobotStates but from rs to otherRobotStates
									// where the da for rs is the same.
									// each time we have a new init state, we have new links
									sumprod_new = new MDPSimple(sumprod_noswitches);
									sumprod_new.setVarList((VarList) sumprod_noswitches.varList.clone());
									sumprod_new.setStatesList(sumprod_noswitches.getStatesList());

									addlink(sumprod_new, rs, otherRobotStates, probs, nextr, ps);

									sumprod_new.clearInitialStates();
									sumprod_new.addInitialState(rs);
									// update the switches
									// TODO: update switches here
									// updateswitches()
									BitSet initstates_now[] = new BitSet[numrobots];
									// for each robot that is not next r get the states at time t
									// for nextr get the states at time nextrt
									int sn_numStates = sumprod_new.numStates;
									for (int rt = 0; rt < numrobots; rt++) {
										initstates_now[rt] = new BitSet(sn_numStates);
										// TODO; add the get alll states function here
										int ct = t; // current time for this robot
										if (rt == nextr)
											ct = nextrt;
										if (pol[rt].size() <= ct)
											ct = pol[rt].size() - 1;
										if (rt == r) // if i'm at the current robot (whose thing has failed and is the
														// one i'm starting in, I only need one state)
										{
											initstates_now[rt].or(getStatesWithSameMDPState(states, rs, numVar));
										} else {
											ArrayList<Entry<Integer, Integer>> nstates = (pol[rt]).get(ct);
											// We also want the first state at time t=0 so that we can exclude any
											// states that do not have those DAVals
											int state_time0 = (((pol[rt]).get(0)).get(0)).getKey();
											// if(rt <= nextr) {
											for (int ns_ = 0; ns_ < nstates.size(); ns_++) {
												int state_now = nstates.get(ns_).getKey();
												// if (rt > nextr) {
												// List<Integer> ci2 =
												// findChangedStateIndices(states.get(state_time0),states.get(state_now),numVar);
												//
												// initstates_now[rt].or(getStatesWithSameMDPState(states,state_now,numVar,ci,true));
												// }
												// else
												// {
												initstates_now[rt]
														.or(getStatesWithSameMDPState(states, state_now, numVar));
												// }

											}
											// }
											// else
											// {
											// //convert the old states to new ones basically oldnumstates =
											// numstates/numrobots
											// //so if you want to assume the other robots (the coming ones) haven't
											// moved.
											// //not the best idea because everyone has moved so you know
											// int oldnumstates = states.size()/numrobots;
											// int nextsetbit = oldinitstates[rt].nextSetBit(0);
											// BitSet tempbs = new BitSet(states.size());
											// while(nextsetbit!=-1)
											// {
											// tempbs.set((rt*oldnumstates)+nextsetbit);
											// nextsetbit = oldinitstates[rt].nextSetBit(nextsetbit+1);
											// }
											// initstates_now[rt] = tempbs;//(BitSet)oldinitstates[rt].clone();
											//
											// }
										}
									}

									// int totswitches=
									updateSwitches(switches, numrobots, states, numVar, initstates_now, sumprod_new);
									sumprod_new.findDeadlocks(true);
									sumprod_new.exportToDotFile(saveplace + "sumprod_new" + rs + ".dot");
									sumprod_new.exportToPrismExplicitTra(saveplace + "sumprod_new" + rs + ".tra");

									res = computeUntilProbs(sumprod_new, badstates, accStatesF, false);

								}
							}
							mainLog.print("\t");
						}
					}
					mainLog.print("\n");
				}

				sumprod.findDeadlocks(true);
			}
		}
		sumprod.clearInitialStates();
		for (int s = 0; s < newinitstates.size(); s++)
			sumprod.addInitialState(newinitstates.get(s));
	}

	private Object[] getChangedIndicesAcrossPolicy(ArrayList<ArrayList<Map.Entry<Integer, Integer>>> pol[],
			int numrobots, int t, int cur_r, int nextr, List<State> states, int numVar) {
		Object res[] = new Object[numrobots];
		ArrayList<ArrayList<Integer>> robotChangedIndices = new ArrayList<ArrayList<Integer>>();
		
		// for each robot get the state at that time and the state at time 0
		int ct = t; // current time for robot
		int maxt = 0;
		ArrayList<Entry<Integer, ArrayList<Entry<Integer, Integer>>>> rvals;
		for (int r = 0; r < numrobots; r++) {
			ArrayList<Integer> changedIndices = new ArrayList<Integer>();
			
			ArrayList<ArrayList<Entry<Integer, Integer>>> polr = pol[r];
			maxt = polr.size();
			if (ct >= maxt)
				ct = maxt - 1;
			int state_0 = ((polr.get(0)).get(0)).getKey(); // return the first state for this robot, we know there will
															// only be one
			// there might be two and i'm not thinking about that
			// TODO: think about this shit
			ArrayList<Entry<Integer, Integer>> states_now = ((polr.get(ct)));
			rvals = new ArrayList<Entry<Integer, ArrayList<Entry<Integer, Integer>>>>();
			int states_now_size = states_now.size();

			for (int s = 0; s < states_now_size; s++) {
				int state_now = states_now.get(s).getKey();
				ArrayList<Entry<Integer, Integer>> cires = findChangedStateIndices(states.get(state_0),
						states.get(state_now), numVar, true);

				mainLog.print("\nR:" + r + " " + state_0 + "->" + state_now + ": ");
				for (int tp = 0; tp < cires.size(); tp++) {
					mainLog.print(cires.get(tp).getKey() + "," + cires.get(tp).getValue());
					changedIndices.add(cires.get(tp).getKey());
				}
				rvals.add(new AbstractMap.SimpleEntry(state_now, cires));
				mainLog.print("\n");
			}
			//go over all the changed indices and get the value for that state. 
			for(int s=0; s<rvals.size(); s++)
			{
				ArrayList<Entry<Integer, Integer>> currci = rvals.get(s).getValue();
				if(currci.size()!=changedIndices.size())
				{
					//we want stuff for all values //something is missing 
					for(int ci=0; ci<changedIndices.size(); ci++)
					{
						int ci_key = changedIndices.get(ci); 
						if (!entrylistContains(currci,ci_key)) {
						int ci_val=getIndValFromState(states.get(rvals.get(s).getKey()), ci_key);
						currci.add(new AbstractMap.SimpleEntry(ci_key, ci_val));
						}
					}
				}
			}
			robotChangedIndices.add(changedIndices);
			res[r] = rvals;
		}

		return res;

	}
	private boolean entrylistContains(ArrayList<Entry<Integer,Integer>> arayylist,int key)
			{
		boolean res = false; 
		for(int i=0; i<arayylist.size(); i++)
		{
			if (arayylist.get(i).getKey()==key)
				{res = true;
				break;
				}
		}
		return res;
			}
	private BitSet getStatesWithSameMDPState(List<State> states, int state, int numVar) {
		BitSet res = new BitSet(states.size());
		State x1 = states.get(state);
		boolean considerRobot = true;
		for (int s = 0; s < states.size(); s++) {
			// if (sameMDPDAVals(states.get(s), x1, numVar,considerRobot,ci,considerx2)
			if (sameMDPVals(states.get(s), x1, numVar, considerRobot))
				res.set(s);
		}
		return res;

	}

	private BitSet getStatesWithSameMDPState(List<State> states, int state, int numVar, List<Integer> ci,
			boolean considerda) {
		BitSet res = new BitSet(states.size());
		State x1 = states.get(state);
		boolean considerRobot = true;
		for (int s = 0; s < states.size(); s++) {
			if (sameMDPDAVals(states.get(s), x1, numVar, considerRobot, ci, considerda)) {
				// if (sameMDPVals(states.get(s),x1,numVar,considerRobot))
				res.set(s);
			}
		}
		return res;

	}

	private Object[] createRobotCombs(int numrobots, int cur_r, Object cirobots[]) {
		int numres = 2;
		Object res[] = new Object[numres];
		int maxcomb = (int) Math.pow(2.0, (double) (numrobots - 1));
		int combstates[][] = new int[maxcomb][numrobots];
		// combstates = new int [maxcomb][numrobots];
		int counter[] = new int[numrobots];
		ArrayList<Entry<Integer, ArrayList<Entry<Integer, Integer>>>> rvals;
		for (int i = 0; i < numrobots; i++) {
			counter[i] = 0;
		}
		ArrayList<ArrayList<Entry<Integer, Integer>>> combinations = new ArrayList<ArrayList<Entry<Integer, Integer>>>();

		int combnum = 0;
		int rcounter = 1;
		mainLog.print("\n");
		while (combnum < maxcomb) {
			ArrayList<Entry<Integer, Integer>> currcombination = new ArrayList<Entry<Integer, Integer>>();
			for (int r = 0; r < numrobots; r++) {

				if (r != cur_r) {
					rvals = (ArrayList<Entry<Integer, ArrayList<Entry<Integer, Integer>>>>) cirobots[r];
					int rsize = rvals.size();
					mainLog.print(" " + r + ":");
					mainLog.print(rvals.get(counter[r]).getKey() + "," + rvals.get(counter[r]).getValue());
					currcombination.addAll(rvals.get(counter[r]).getValue());
					combstates[combnum][r] = rvals.get(counter[r]).getKey();
					if (combnum % (rcounter + 1) == 0)
						counter[r] = (counter[r] + 1) % rsize;
					rcounter = (rcounter + 1) % (numrobots - 1);

				}
			}
			combnum++;
			combinations.add(currcombination);
			mainLog.print("\n");
		}
		res[0] = combinations;
		res[1] = combstates;
		return res;// combinations;
	}


	private int calculateMaxCombsNum(int numrobots, int cur_r, ArrayList<ArrayList<Entry<policyState, ArrayList<Entry<Integer, Integer>>>>> cirobots)
	{
		int res = 1; 
		for(int i = 0; i<numrobots; i++)
		{
			if(i != cur_r)
			res = res*cirobots.get(i).size();
		}
		return res;
	}
	private Object[] createRobotCombs(int numrobots, int cur_r, ArrayList<ArrayList<Entry<policyState, ArrayList<Entry<Integer, Integer>>>>> cirobots) {
		int numres = 3;
		Object res[] = new Object[numres];
		int maxcomb = calculateMaxCombsNum(numrobots,cur_r,cirobots);//(int) Math.pow(2.0, (double) (numrobots - 1));	// this causes problems because that is not the real number of max combs
		int combstates[][] = new int[maxcomb][numrobots];
		double combstatesProbs[] = new double[maxcomb];
		// combstates = new int [maxcomb][numrobots];
		int counter[] = new int[numrobots];
		ArrayList<Entry<policyState, ArrayList<Entry<Integer, Integer>>>> rvals;
		for (int i = 0; i < numrobots; i++) {
			counter[i] = 0;
		}
		ArrayList<ArrayList<Entry<Integer, Integer>>> combinations = new ArrayList<ArrayList<Entry<Integer, Integer>>>();

		int combnum = 0;
		int rcounter = 1;
		mainLog.print("\n");
		while (combnum < maxcomb) {
			ArrayList<Entry<Integer, Integer>> currcombination = new ArrayList<Entry<Integer, Integer>>();
			double probHere = 1; 
			for (int r = 0; r < numrobots; r++) {

				if (r != cur_r) {
					rvals = (ArrayList<Entry<policyState, ArrayList<Entry<Integer, Integer>>>>) cirobots.get(r);
					int rsize = rvals.size();
					mainLog.print(" " + r + ":");
					mainLog.print(rvals.get(counter[r]).getKey() + "," + rvals.get(counter[r]).getValue());
					currcombination.addAll(rvals.get(counter[r]).getValue());
					combstates[combnum][r] = rvals.get(counter[r]).getKey().state;
					probHere *= rvals.get(counter[r]).getKey().getProb();
					if (combnum % (rcounter + 1) == 0)
						counter[r] = (counter[r] + 1) % rsize;
					rcounter = (rcounter + 1) % (numrobots - 1);

				}
			}
			combstatesProbs[combnum] = probHere;
			combnum++;
			
			combinations.add(currcombination);
			mainLog.print("\n");
		}
		res[0] = combinations;
		res[1] = combstates;
		res[2] = combstatesProbs;
		return res;// combinations;
	}

	private int[] getStatesFromCombinations(ArrayList<ArrayList<Entry<Integer, Integer>>> combinations,
			List<State> states, int cs) {
		int numcomb = combinations.size();
		int newstates[] = new int[numcomb];
		for (int comb = 0; comb < numcomb; comb++) {
			Object[] newstate = getNewState(states.get(cs), combinations.get(comb));
			newstates[comb] = findSameState(newstate, states);
		}
		return newstates;

	}

	// computes weightedSkipping for an MDP (a mix of two papers by M L et. al )
	@SuppressWarnings("unchecked")
	protected StateValues weightedSkipping(Model model, ExpressionFunc expr, BitSet statesOfInterest)
			throws PrismException {
		LTLModelChecker mcLtl;
		StateValues probsProduct, probs, costsProduct, costs, distSatProduct, distSat;
		MDPModelChecker mcProduct;
		LTLModelChecker.LTLProduct<MDP> product;
		MDP productMdp;
		DA<BitSet, ? extends AcceptanceOmega> da;
		Vector<BitSet> labelBS;
		MDPRewardsSimple skipcosts = null; // to save the skipping costs
		MDPRewardsSimple prodcosts = null; // to save the actual product costs (added tranistions )

		// For LTL model checking routines
		mcLtl = new LTLModelChecker(this);

		// Get LTL spec
		ExpressionReward exprRew = (ExpressionReward) expr.getOperand(0);
		Expression ltl = exprRew.getExpression();

		System.out.println("--------------------------------------------------------------");
		System.out.println("The flat MDP model has " + model.getNumStates() + " states");
		System.out.println("The specification is " + ltl.toString());
		System.out.println("Generating optimal policy...");
		System.out.println(" ");

		// Build model costs
		RewardStruct costStruct = exprRew.getRewardStructByIndexObject(modulesFile, modulesFile.getConstantValues());
		mainLog.println("Building cost structure...");
		// int r = exprRew.getRewardStructIndexByIndexObject(modelInfo, constantValues);
		// //TODO fix this
		Rewards costsModel = constructRewards(model, costStruct);

		// build DFA
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
		labelBS = new Vector<BitSet>();
		da = mcLtl.constructDAForLTLFormula(this, model, ltl, labelBS, allowedAcceptance);

		// Specifiying costs -----------------------------------------
		int numAPs = da.getAPList().size(); // to specify the costs
		// for each AP there is a cost for skipping it

		double apCosts[] = new double[numAPs];
		// hardcoding this TODO find a better way to specify costs
		double noavoidcost = 1;
		double nogoalcost = 2000;
		for (int i = 0; i < numAPs; i++)
			apCosts[i] = noavoidcost; // because I know what the LTL looks like and the general formulation
		apCosts[0] = 1;
		for (int i = 1; i < numAPs; i++) {
			apCosts[i] = nogoalcost;
		}
		// -----------------------------------------

		// Printing Costs just to confirm
		System.out.print("Costs for LTL Spec:");
		for (int i = 0; i < numAPs; i++)
			System.out.print(" " + da.getAPList().get(i) + ":" + apCosts[i]);
		System.out.println();

		if (!(da.getAcceptance() instanceof AcceptanceReach)) {
			mainLog.println("\nAutomaton is not a DFA. Breaking.");
			// Dummy return vector
			return new StateValues(TypeInt.getInstance(), model);
		}

		// build product
		// time to build weighted skipping product
		long time = System.currentTimeMillis();

		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		for (int i = 0; i < numStates; i++) {
			bsInit.set(i, model.isInitialState(i));
		}

		// construct the weighted skipping product model as well as the skipping costs
		// and product costs
		Object arr[] = mcLtl.constructWeightedProductModelP(da, (MDP) model, labelBS, bsInit,
				(MDPRewardsSimple) costsModel, apCosts);

		product = (LTLProduct<MDP>) arr[0];
		skipcosts = (MDPRewardsSimple) arr[1];
		prodcosts = (MDPRewardsSimple) arr[2];
		int badstate = (int) arr[3]; // added a bad state so that all actions go to this place

		time = System.currentTimeMillis() - time;
		mainLog.println("Time to generate product mdp with costs: " + time / 1000.0 + " seconds.");

		// Find accepting states + compute reachability probabilities

		BitSet acc;
		if (product.getAcceptance() instanceof AcceptanceReach) {
			mainLog.println("\nSkipping accepting MEC computation since acceptance is defined via goal states...");
			acc = ((AcceptanceReach) product.getAcceptance()).getGoalStates();
		} else {
			mainLog.println("\nFinding accepting MECs...");
			acc = new LTLModelChecker(null).findAcceptingECStates(product.getProductModel(), product.getAcceptance());
		}

		// Output product, if required
		if (getExportProductTrans()) {
			mainLog.println(
					"\nExporting product transition matrix to file \"" + getExportProductTransFilename() + "\"...");
			product.getProductModel().exportToPrismExplicitTra(getExportProductTransFilename());
		}
		if (getExportProductStates()) {
			mainLog.println("\nExporting product state space to file \"" + getExportProductStatesFilename() + "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductStatesFilename());
			VarList newVarList = (VarList) modulesFile.createVarList().clone();
			String daVar = "_da";
			while (newVarList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}
			newVarList.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
			product.getProductModel().exportStates(Prism.EXPORT_PLAIN, newVarList, out);
			out.close();
		}

		// TODO remember the bad state, find a better way to do this
		mcProduct = new MDPModelChecker(this);
		mcProduct.inheritSettings(this);

		if (product.getProductModel().getNumStates() > 10000) {
			mainLog.println("\nChanging product to MDPSparse...");
			productMdp = new MDPSparse((MDPSimple) product.getProductModel());
		} else {

			productMdp = product.getProductModel();
		}
		mainLog.println("Product Size");
		mainLog.println("States: " + productMdp.getNumStates() + " Tranistions: " + productMdp.getNumTransitions()
				+ " Choices: " + productMdp.getNumChoices() + " Max Choices: " + productMdp.getMaxNumChoices());
		// added an extra bad state in the product mdp

		// make sure its not part of the accepting state (because its a bit hacky)
		// do not compute VI for all target states
		BitSet progStates = (BitSet) acc.clone();
		progStates.flip(0, productMdp.getNumStates());
		progStates.clear(badstate);
		acc.clear(badstate);

		// compute nested VI
		mainLog.println("\nComputing reachability probability, expected dist to sat and expected cost...");
		time = System.currentTimeMillis();
		ModelCheckerPartialSatResult res = mcProduct.computeNestedValIterWS(productMdp, acc, skipcosts, prodcosts,
				progStates);
		time = System.currentTimeMillis() - time;
		mainLog.println("Time for computing reachability probability, expected dist to sat and expected cost: "
				+ time / 1000.0 + " seconds.");

		probsProduct = StateValues.createFromDoubleArray(res.solnProb, productMdp);
		// Mapping probabilities in the original model
		probs = product.projectToOriginalModel(probsProduct);
		// Get final prob result // this should always be 1 because you can skip
		double maxProb = probs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nMaximum probability to satisfy specification is " + maxProb);

		if (getExportProductVector()) {
			mainLog.println("\nExporting success probabilites over product to file \""
					+ PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 1) + "\"...");
			PrismFileLog out = new PrismFileLog(
					PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 1));
			probsProduct.print(out, false, false, false, false);
			out.close();
		}

		distSatProduct = StateValues.createFromDoubleArray(res.solnProg, productMdp);
		distSat = product.projectToOriginalModel(distSatProduct);
		double minDistSat = distSat.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println(
				"\nFor p = " + maxProb + ", the minimum expected distance to satisfy specification is " + minDistSat);

		if (getExportProductVector()) {
			mainLog.println("\nExporting expected progression reward over product to file \""
					+ PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 2) + "\"...");
			PrismFileLog out = new PrismFileLog(
					PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 2));
			distSatProduct.print(out, false, false, false, false);
			out.close();
		}

		costsProduct = StateValues.createFromDoubleArray(res.solnCost, productMdp);
		costs = product.projectToOriginalModel(costsProduct);
		double minCost = costs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb + ", distsat = " + +minDistSat
				+ " the minimum expected  cummulative cost to satisfy specification is " + minCost);
		// System.out.println("Probability to find objects: " + maxProb);
		// System.out.println("Expected progression reward: " + maxRew);
		// System.out.println("Expected time to execute task: " + minCost);
		// System.out.println("--------------------------------------------------------------");
		// TODO what ??
		if (getExportProductVector()) {
			mainLog.println("\nExporting expected times until no more progression over product to file \""
					+ PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 3) + "\"...");
			PrismFileLog out = new PrismFileLog(
					PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 3));
			costsProduct.print(out, false, false, false, false);
			out.close();
		}

		return costs;
	}

	protected StateValues checkPartialSat(Model model, ExpressionFunc expr, BitSet statesOfInterest)
			throws PrismException {
		LTLModelChecker mcLtl;
		StateValues probsProduct, probs, costsProduct, costs, rewsProduct, rews;
		MDPModelChecker mcProduct;
		LTLModelChecker.LTLProduct<MDP> product;
		MDP productMdp;
		DA<BitSet, ? extends AcceptanceOmega> da;
		Vector<BitSet> labelBS;

		String saveplace = System.getProperty("user.dir") + "/dotfiles/";
		// For LTL model checking routines
		mcLtl = new LTLModelChecker(this);

		// Get LTL spec
		ExpressionReward exprRew = (ExpressionReward) expr.getOperand(0);
		Expression ltl = exprRew.getExpression();
		// System.out.println("--------------------------------------------------------------");
		// //System.out.println("The flat MDP model has " + model.getNumStates()
		// + " states");
		System.out.println("The specification is " + ltl.toString());
		// System.out.println("Generating optimal policy...");
		// System.out.println(" ");

		// Build model costs
		RewardStruct costStruct = exprRew.getRewardStructByIndexObject(modulesFile, modulesFile.getConstantValues());
		mainLog.println("Building cost structure...");
		Rewards costsModel = constructRewards(model, costStruct);

		// build DFA
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
		labelBS = new Vector<BitSet>();
		da = mcLtl.constructDAForLTLFormula(this, model, ltl, labelBS, allowedAcceptance);

		if (!(da.getAcceptance() instanceof AcceptanceReach)) {
			mainLog.println("\nAutomaton is not a DFA. Breaking.");
			// Dummy return vector
			return new StateValues(TypeInt.getInstance(), model);
		}
		// calculate distances to accepting states
		long time = System.currentTimeMillis();
		da.setDistancesToAcc();
		time = System.currentTimeMillis() - time;
		mainLog.println("\nAutomaton state distances to an accepting state: " + da.getDistsToAcc());
		mainLog.println("Time for DFA distance to acceptance metric calculation: " + time / 1000.0 + " seconds.");

		// build product
		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		for (int i = 0; i < numStates; i++) {
			bsInit.set(i, model.isInitialState(i));
		}
		product = mcLtl.constructProductModel(da, (MDP) model, labelBS, bsInit);
		if (run_tests) {
			File theDir = new File(saveplace);

			// if the directory does not exist, create it
			if (!theDir.exists()) {
				System.out.println("creating directory: " + theDir.getName());
				boolean result = false;

				try {
					theDir.mkdir();
					result = true;
				} catch (SecurityException se) {
					// handle it
				}
				if (result) {
					System.out.println("DIR created");
				}
			}
			product.productModel.exportToDotFile(saveplace + "actual_prod.dot");

		}

		// System.out.println("The product MDP has " +
		// product.getProductModel().getNumStates() + " states");

		// Find accepting states + compute reachability probabilities
		BitSet acc;
		if (product.getAcceptance() instanceof AcceptanceReach) {
			mainLog.println("\nSkipping accepting MEC computation since acceptance is defined via goal states...");
			acc = ((AcceptanceReach) product.getAcceptance()).getGoalStates();
		} else {
			mainLog.println("\nFinding accepting MECs...");
			acc = mcLtl.findAcceptingECStates(product.getProductModel(), product.getAcceptance());
		}

		time = System.currentTimeMillis();
		// Build progression rewards on product
		MDPRewards progRewards = product.liftProgressionFromAutomaton(da.getDistsToAcc());
		time = System.currentTimeMillis() - time;
		mainLog.println(
				"Time for lifting progression reward from automaton to product: " + time / 1000.0 + " seconds.");

		time = System.currentTimeMillis();
		// Build trimmed product costs
		MDPRewards prodCosts = ((MDPRewards) costsModel).liftFromModel(product);
		time = System.currentTimeMillis() - time;
		mainLog.println(
				"Time for lifting cost function from original model to product: " + time / 1000.0 + " seconds.");

		BitSet progStates = progressionTrim(product, (MDPRewardsSimple) progRewards, (MDPRewardsSimple) prodCosts);

		// Output product, if required
		if (getExportProductTrans()) {
			mainLog.println(
					"\nExporting product transition matrix to file \"" + getExportProductTransFilename() + "\"...");
			product.getProductModel().exportToPrismExplicitTra(getExportProductTransFilename());
		}
		if (getExportProductStates()) {
			mainLog.println("\nExporting product state space to file \"" + getExportProductStatesFilename() + "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductStatesFilename());
			VarList newVarList = (VarList) modulesFile.createVarList().clone();
			String daVar = "_da";
			while (newVarList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}
			newVarList.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
			product.getProductModel().exportStates(Prism.EXPORT_PLAIN, newVarList, out);
			out.close();
		}

		mcProduct = new MDPModelChecker(this);
		mcProduct.inheritSettings(this);

		if (product.getProductModel().getNumStates() > 10000) {
			mainLog.println("\nChanging product to MDPSparse...");
			productMdp = new MDPSparse((MDPSimple) product.getProductModel());
		} else {
			productMdp = product.getProductModel();
		}

		mainLog.println("\nComputing reachability probability, expected progression, and expected cost...");
		ModelCheckerPartialSatResult res = mcProduct.computeNestedValIter(productMdp, acc, progRewards, prodCosts,
				progStates);
		probsProduct = StateValues.createFromDoubleArray(res.solnProb, productMdp);

		// Mapping probabilities in the original model
		probs = product.projectToOriginalModel(probsProduct);
		// Get final prob result
		double maxProb = probs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nMaximum probability to satisfy specification is " + maxProb);

		if (getExportProductVector()) {
			mainLog.println("\nExporting success probabilites over product to file \""
					+ PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 1) + "\"...");
			PrismFileLog out = new PrismFileLog(
					PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 1));
			probsProduct.print(out, false, false, false, false);
			out.close();
		}

		rewsProduct = StateValues.createFromDoubleArray(res.solnProg, productMdp);
		rews = product.projectToOriginalModel(rewsProduct);
		double maxRew = rews.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb
				+ ", the maximum expected cummulative reward to satisfy specification is " + maxRew);

		if (getExportProductVector()) {
			mainLog.println("\nExporting expected progression reward over product to file \""
					+ PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 2) + "\"...");
			PrismFileLog out = new PrismFileLog(
					PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 2));
			rewsProduct.print(out, false, false, false, false);
			out.close();
		}

		costsProduct = StateValues.createFromDoubleArray(res.solnCost, productMdp);
		costs = product.projectToOriginalModel(costsProduct);
		double minCost = costs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb + ", r = " + +maxRew
				+ " the minimum expected  cummulative cost to satisfy specification is " + minCost);
		// System.out.println("Probability to find objects: " + maxProb);
		// System.out.println("Expected progression reward: " + maxRew);
		// System.out.println("Expected time to execute task: " + minCost);
		// System.out.println("--------------------------------------------------------------");
		if (getExportProductVector()) {
			mainLog.println("\nExporting expected times until no more progression over product to file \""
					+ PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 3) + "\"...");
			PrismFileLog out = new PrismFileLog(
					PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 3));
			costsProduct.print(out, false, false, false, false);
			out.close();
		}

		return costs;

	}

	public BitSet progressionTrim(LTLModelChecker.LTLProduct<MDP> product, MDPRewardsSimple progRewards,
			MDPRewardsSimple prodCosts) {
		MDP productModel = product.getProductModel();
		int numStates = productModel.getNumStates();
		List<HashSet<Integer>> predList = new ArrayList<HashSet<Integer>>(numStates);
		Deque<Integer> queue = new ArrayDeque<Integer>();
		BitSet progStates = new BitSet(numStates);
		long time;

		time = System.currentTimeMillis();

		// init predList and queue
		for (int i = 0; i < numStates; i++) {
			predList.add(new HashSet<Integer>());
			for (int j = 0; j < productModel.getNumChoices(i); j++) {
				if (progRewards.getTransitionReward(i, j) > 0.0) {
					queue.add(i);
					progStates.set(i);
					break;
				}
			}
		}

		Iterator<Integer> successorsIt;
		HashSet<Integer> statePreds;
		// set predList
		for (int i = 0; i < numStates; i++) {
			successorsIt = productModel.getSuccessorsIterator(i);
			while (successorsIt.hasNext()) {
				statePreds = predList.get(successorsIt.next());
				statePreds.add(i);
			}
		}

		// set
		int currentState;
		while (!queue.isEmpty()) {
			currentState = queue.poll();
			for (int pred : predList.get(currentState)) {
				if (!progStates.get(pred)) {
					queue.add(pred);
					progStates.set(pred);
				}
			}
		}

		int nTrims = 0;
		// trim rewards according to progression metric TODO: THis can be
		// removed because now we return the progStates
		/*
		 * for(int i = 0; i < numStates; i++) { if(!progStates.get(i)) {
		 * prodCosts.clearRewards(i); nTrims++; } }
		 */

		time = System.currentTimeMillis() - time;
		mainLog.println(
				"\nCleared costs for " + nTrims + " states where no more progression towards goal is possible.");
		mainLog.println("Time for cost trimming: " + time / 1000.0 + " seconds.");
		return progStates;
	}

	@Override
	protected StateValues checkProbPathFormulaLTL(Model model, Expression expr, boolean qual, MinMax minMax,
			BitSet statesOfInterest) throws PrismException {
		LTLModelChecker mcLtl;
		StateValues probsProduct, probs;
		MDPModelChecker mcProduct;
		LTLModelChecker.LTLProduct<MDP> product;

		// For min probabilities, need to negate the formula
		// (add parentheses to allow re-parsing if required)
		if (minMax.isMin()) {
			expr = Expression.Not(Expression.Parenth(expr.deepCopy()));
		}

		// For LTL model checking routines
		mcLtl = new LTLModelChecker(this);

		// Build product of MDP and automaton
		AcceptanceType[] allowedAcceptance = { AcceptanceType.BUCHI, AcceptanceType.RABIN,
				AcceptanceType.GENERALIZED_RABIN, AcceptanceType.REACH };
		product = mcLtl.constructProductMDP(this, (MDP) model, expr, statesOfInterest, allowedAcceptance);

		// Output product, if required
		if (getExportProductTrans()) {
			mainLog.println(
					"\nExporting product transition matrix to file \"" + getExportProductTransFilename() + "\"...");
			product.getProductModel().exportToPrismExplicitTra(getExportProductTransFilename());
		}
		if (getExportProductStates()) {
			mainLog.println("\nExporting product state space to file \"" + getExportProductStatesFilename() + "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductStatesFilename());
			VarList newVarList = (VarList) modulesFile.createVarList().clone();
			String daVar = "_da";
			while (newVarList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}
			newVarList.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
			product.getProductModel().exportStates(Prism.EXPORT_PLAIN, newVarList, out);
			out.close();
		}

		// Find accepting states + compute reachability probabilities
		BitSet acc;
		if (product.getAcceptance() instanceof AcceptanceReach) {
			mainLog.println("\nSkipping accepting MEC computation since acceptance is defined via goal states...");
			acc = ((AcceptanceReach) product.getAcceptance()).getGoalStates();
		} else {
			mainLog.println("\nFinding accepting MECs...");
			acc = mcLtl.findAcceptingECStates(product.getProductModel(), product.getAcceptance());
		}
		mainLog.println("\nComputing reachability probabilities...");
		mcProduct = new MDPModelChecker(this);
		mcProduct.inheritSettings(this);
		ModelCheckerResult res = mcProduct.computeReachProbs(product.getProductModel(), acc, false);
		probsProduct = StateValues.createFromDoubleArray(res.soln, product.getProductModel());

		// Subtract from 1 if we're model checking a negated formula for regular
		// Pmin
		if (minMax.isMin()) {
			probsProduct.timesConstant(-1.0);
			probsProduct.plusConstant(1.0);
		}

		// Output vector over product, if required
		if (getExportProductVector()) {
			mainLog.println("\nExporting product solution vector matrix to file \"" + getExportProductVectorFilename()
					+ "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductVectorFilename());
			probsProduct.print(out, false, false, false, false);
			out.close();
		}

		// Mapping probabilities in the original model
		probs = product.projectToOriginalModel(probsProduct);
		probsProduct.clear();

		return probs;
	}

	/**
	 * Compute rewards for a co-safe LTL reward operator.
	 */
	protected StateValues checkRewardCoSafeLTL(Model model, Rewards modelRewards, Expression expr, MinMax minMax,
			BitSet statesOfInterest) throws PrismException {
		LTLModelChecker mcLtl;
		MDPRewards productRewards;
		StateValues rewardsProduct, rewards;
		MDPModelChecker mcProduct;
		LTLModelChecker.LTLProduct<MDP> product;

		// For LTL model checking routines
		mcLtl = new LTLModelChecker(this);

		// Build product of MDP and automaton
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
		product = mcLtl.constructProductMDP(this, (MDP) model, expr, statesOfInterest, allowedAcceptance);

		// Adapt reward info to product model
		productRewards = ((MDPRewards) modelRewards).liftFromModel(product);

		// Output product, if required
		if (getExportProductTrans()) {
			mainLog.println(
					"\nExporting product transition matrix to file \"" + getExportProductTransFilename() + "\"...");
			product.getProductModel().exportToPrismExplicitTra(getExportProductTransFilename());
		}
		if (getExportProductStates()) {
			mainLog.println("\nExporting product state space to file \"" + getExportProductStatesFilename() + "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductStatesFilename());
			VarList newVarList = (VarList) modulesFile.createVarList().clone();
			String daVar = "_da";
			while (newVarList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}
			newVarList.addVar(0, new Declaration(daVar, new DeclarationIntUnbounded()), 1, null);
			product.getProductModel().exportStates(Prism.EXPORT_PLAIN, newVarList, out);
			out.close();
		}

		// Find accepting states + compute reachability rewards
		BitSet acc;
		if (product.getAcceptance() instanceof AcceptanceReach) {
			// For a DFA, just collect the accept states
			mainLog.println("\nSkipping end component detection since DRA is a DFA...");
			acc = ((AcceptanceReach) product.getAcceptance()).getGoalStates();
		} else {
			// Usually, we have to detect end components in the product
			mainLog.println("\nFinding accepting end components...");
			acc = mcLtl.findAcceptingECStates(product.getProductModel(), product.getAcceptance());
		}
		mainLog.println("\nComputing reachability rewards...");
		mcProduct = new MDPModelChecker(this);
		mcProduct.inheritSettings(this);
		ModelCheckerResult res = mcProduct.computeReachRewards(product.getProductModel(), productRewards, acc,
				minMax.isMin());
		rewardsProduct = StateValues.createFromDoubleArray(res.soln, product.getProductModel());

		// Output vector over product, if required
		if (getExportProductVector()) {
			mainLog.println("\nExporting product solution vector matrix to file \"" + getExportProductVectorFilename()
					+ "\"...");
			PrismFileLog out = new PrismFileLog(getExportProductVectorFilename());
			rewardsProduct.print(out, false, false, false, false);
			out.close();
		}

		// Mapping rewards in the original model
		rewards = product.projectToOriginalModel(rewardsProduct);
		rewardsProduct.clear();

		return rewards;
	}

	// Numerical computation functions

	/**
	 * Compute next=state probabilities. i.e. compute the probability of being in a
	 * state in {@code target} in the next step.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 */
	public ModelCheckerResult computeNextProbs(MDP mdp, BitSet target, boolean min) throws PrismException {
		ModelCheckerResult res = null;
		int n;
		double soln[], soln2[];
		long timer;

		timer = System.currentTimeMillis();

		// Store num states
		n = mdp.getNumStates();

		// Create/initialise solution vector(s)
		soln = Utils.bitsetToDoubleArray(target, n);
		soln2 = new double[n];

		// Next-step probabilities
		mdp.mvMultMinMax(soln, min, soln2, null, false, null);

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln2;
		res.numIters = 1;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Given a value vector x, compute the probability: v(s) = min/max sched [
	 * Sum_s' P_sched(s,s')*x(s') ] for s labeled with a, v(s) = 0 for s not labeled
	 * with a.
	 *
	 * Clears the StateValues object x.
	 *
	 * @param tr
	 *            the transition matrix
	 * @param a
	 *            the set of states labeled with a
	 * @param x
	 *            the value vector
	 * @param min
	 *            compute min instead of max
	 */
	public double[] computeRestrictedNext(MDP mdp, BitSet a, double[] x, boolean min) {
		int n;
		double soln[];

		// Store num states
		n = mdp.getNumStates();

		// initialized to 0.0
		soln = new double[n];

		// Next-step probabilities multiplication
		// restricted to a states
		mdp.mvMultMinMax(x, min, soln, a, false, null);

		return soln;
	}

	/**
	 * Compute reachability probabilities. i.e. compute the min/max probability of
	 * reaching a state in {@code target}.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 */
	public ModelCheckerResult computeReachProbs(MDP mdp, BitSet target, boolean min) throws PrismException {
		return computeReachProbs(mdp, null, target, min, null, null);
	}

	/**
	 * Compute until probabilities. i.e. compute the min/max probability of reaching
	 * a state in {@code target}, while remaining in those in {@code remain}.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param remain
	 *            Remain in these states (optional: null means "all")
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 */
	public ModelCheckerResult computeUntilProbs(MDP mdp, BitSet remain, BitSet target, boolean min)
			throws PrismException {
		return computeReachProbs(mdp, remain, target, min, null, null);
	}

	/**
	 * Compute reachability/until probabilities. i.e. compute the min/max
	 * probability of reaching a state in {@code target}, while remaining in those
	 * in {@code remain}.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param remain
	 *            Remain in these states (optional: null means "all")
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (may be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values). Also, 'known' values
	 *            cannot be passed for some solution methods, e.g. policy iteration.
	 */
	public ModelCheckerResult computeReachProbs(MDP mdp, BitSet remain, BitSet target, boolean min, double init[],
			BitSet known) throws PrismException {
		ModelCheckerResult res = null;
		BitSet no, yes;
		int n, numYes, numNo;
		long timer, timerProb0, timerProb1;
		int strat[] = null;
		// Local copy of setting
		MDPSolnMethod mdpSolnMethod = this.mdpSolnMethod;

		// Switch to a supported method, if necessary
		if (mdpSolnMethod == MDPSolnMethod.LINEAR_PROGRAMMING) {
			mdpSolnMethod = MDPSolnMethod.GAUSS_SEIDEL;
			mainLog.printWarning("Switching to MDP solution method \"" + mdpSolnMethod.fullName() + "\"");
		}

		// Check for some unsupported combinations
		if (mdpSolnMethod == MDPSolnMethod.VALUE_ITERATION && valIterDir == ValIterDir.ABOVE) {
			if (!(precomp && prob0))
				throw new PrismException("Precomputation (Prob0) must be enabled for value iteration from above");
			if (!min)
				throw new PrismException("Value iteration from above only works for minimum probabilities");
		}
		if (mdpSolnMethod == MDPSolnMethod.POLICY_ITERATION
				|| mdpSolnMethod == MDPSolnMethod.MODIFIED_POLICY_ITERATION) {
			if (known != null) {
				throw new PrismException("Policy iteration methods cannot be passed 'known' values for some states");
			}
		}

		// Start probabilistic reachability
		timer = System.currentTimeMillis();
		mainLog.println("\nStarting probabilistic reachability (" + (min ? "min" : "max") + ")...");

		// Check for deadlocks in non-target state (because breaks e.g. prob1)
		mdp.checkForDeadlocks(target);

		// Store num states
		n = mdp.getNumStates();

		// Optimise by enlarging target set (if more info is available)
		if (init != null && known != null && !known.isEmpty()) {
			BitSet targetNew = (BitSet) target.clone();
			for (int i : new IterableBitSet(known)) {
				if (init[i] == 1.0) {
					targetNew.set(i);
				}
			}
			target = targetNew;
		}

		// If required, export info about target states
		if (getExportTarget()) {
			BitSet bsInit = new BitSet(n);
			for (int i = 0; i < n; i++) {
				bsInit.set(i, mdp.isInitialState(i));
			}
			List<BitSet> labels = Arrays.asList(bsInit, target);
			List<String> labelNames = Arrays.asList("init", "target");
			mainLog.println("\nExporting target states info to file \"" + getExportTargetFilename() + "\"...");
			exportLabels(mdp, labels, labelNames, Prism.EXPORT_PLAIN, new PrismFileLog(getExportTargetFilename()));
		}

		// If required, create/initialise strategy storage
		// Set choices to -1, denoting unknown
		// (except for target states, which are -2, denoting arbitrary)
		if (genStrat || exportAdv) {
			strat = new int[n];
			for (int i = 0; i < n; i++) {
				strat[i] = target.get(i) ? -2 : -1;
			}
		}

		// Precomputation
		timerProb0 = System.currentTimeMillis();
		if (precomp && prob0) {
			no = prob0(mdp, remain, target, min, strat);
		} else {
			no = new BitSet();
		}
		timerProb0 = System.currentTimeMillis() - timerProb0;
		timerProb1 = System.currentTimeMillis();
		if (precomp && prob1) {
			yes = prob1(mdp, remain, target, min, strat);
		} else {
			yes = (BitSet) target.clone();
		}
		timerProb1 = System.currentTimeMillis() - timerProb1;

		// Print results of precomputation
		numYes = yes.cardinality();
		numNo = no.cardinality();
		mainLog.println("target=" + target.cardinality() + ", yes=" + numYes + ", no=" + numNo + ", maybe="
				+ (n - (numYes + numNo)));

		// If still required, store strategy for no/yes (0/1) states.
		// This is just for the cases max=0 and min=1, where arbitrary choices
		// suffice (denoted by -2)
		if (genStrat || exportAdv) {
			if (min) {
				for (int i = yes.nextSetBit(0); i >= 0; i = yes.nextSetBit(i + 1)) {
					if (!target.get(i))
						strat[i] = -2;
				}
			} else {
				for (int i = no.nextSetBit(0); i >= 0; i = no.nextSetBit(i + 1)) {
					strat[i] = -2;
				}
			}
		}

		// Compute probabilities (if needed)
		if (numYes + numNo < n) {
			switch (mdpSolnMethod) {
			case VALUE_ITERATION:
				res = computeReachProbsValIter(mdp, no, yes, min, init, known, strat);
				break;
			case GAUSS_SEIDEL:
				res = computeReachProbsGaussSeidel(mdp, no, yes, min, init, known, strat);
				break;
			case POLICY_ITERATION:
				res = computeReachProbsPolIter(mdp, no, yes, min, strat);
				break;
			case MODIFIED_POLICY_ITERATION:
				res = computeReachProbsModPolIter(mdp, no, yes, min, strat);
				break;
			default:
				throw new PrismException("Unknown MDP solution method " + mdpSolnMethod.fullName());
			}
		} else {
			res = new ModelCheckerResult();
			res.soln = Utils.bitsetToDoubleArray(yes, n);
		}

		// Finished probabilistic reachability
		timer = System.currentTimeMillis() - timer;
		mainLog.println("Probabilistic reachability took " + timer / 1000.0 + " seconds.");

		// Store strategy
		if (genStrat) {
			res.strat = new MDStrategyArray(mdp, strat);
		}

		// Export adversary
		if (exportAdv) {
			// Prune strategy
			restrictStrategyToReachableStates(mdp, strat);
			// Export
			// don't overwrite the file if it exists.
			PrismLog out = new PrismFileLog(exportAdvFilename, true);
			new DTMCFromMDPMemorylessAdversary(mdp, strat).exportToPrismExplicitTra(out);
			out.close();
		}

		// Update time taken
		res.timeTaken = timer / 1000.0;
		res.timeProb0 = timerProb0 / 1000.0;
		res.timePre = (timerProb0 + timerProb1) / 1000.0;

		return res;
	}

	/**
	 * Prob0 precomputation algorithm. i.e. determine the states of an MDP which,
	 * with min/max probability 0, reach a state in {@code target}, while remaining
	 * in those in {@code remain}. {@code min}=true gives Prob0E, {@code min}=false
	 * gives Prob0A. Optionally, for min only, store optimal (memoryless) strategy
	 * info for 0 states.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param remain
	 *            Remain in these states (optional: null means "all")
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 */
	public BitSet prob0(MDP mdp, BitSet remain, BitSet target, boolean min, int strat[]) {
		int n, iters;
		BitSet u, soln, unknown;
		boolean u_done;
		long timer;

		// Start precomputation
		timer = System.currentTimeMillis();
		mainLog.println("Starting Prob0 (" + (min ? "min" : "max") + ")...");

		// Special case: no target states
		if (target.cardinality() == 0) {
			soln = new BitSet(mdp.getNumStates());
			soln.set(0, mdp.getNumStates());
			return soln;
		}

		// Initialise vectors
		n = mdp.getNumStates();
		u = new BitSet(n);
		soln = new BitSet(n);

		// Determine set of states actually need to perform computation for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(target);
		if (remain != null)
			unknown.and(remain);

		// Fixed point loop
		iters = 0;
		u_done = false;
		// Least fixed point - should start from 0 but we optimise by
		// starting from 'target', thus bypassing first iteration
		u.or(target);
		soln.or(target);
		while (!u_done) {
			iters++;
			// Single step of Prob0
			mdp.prob0step(unknown, u, min, soln);
			// Check termination
			u_done = soln.equals(u);
			// u = soln
			u.clear();
			u.or(soln);
		}

		// Negate
		u.flip(0, n);

		// Finished precomputation
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Prob0 (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		// If required, generate strategy. This is for min probs,
		// so it can be done *after* the main prob0 algorithm (unlike for
		// prob1).
		// We simply pick, for all "no" states, the first choice for which all
		// transitions stay in "no"
		if (strat != null) {
			for (int i = u.nextSetBit(0); i >= 0; i = u.nextSetBit(i + 1)) {
				int numChoices = mdp.getNumChoices(i);
				for (int k = 0; k < numChoices; k++) {
					if (mdp.allSuccessorsInSet(i, k, u)) {
						strat[i] = k;
						continue;
					}
				}
			}
		}

		return u;
	}

	/**
	 * Prob1 precomputation algorithm. i.e. determine the states of an MDP which,
	 * with min/max probability 1, reach a state in {@code target}, while remaining
	 * in those in {@code remain}. {@code min}=true gives Prob1A, {@code min}=false
	 * gives Prob1E. Optionally, for max only, store optimal (memoryless) strategy
	 * info for 1 states.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param remain
	 *            Remain in these states (optional: null means "all")
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 */
	public BitSet prob1(MDP mdp, BitSet remain, BitSet target, boolean min, int strat[]) {
		int n, iters;
		BitSet u, v, soln, unknown;
		boolean u_done, v_done;
		long timer;

		// Start precomputation
		timer = System.currentTimeMillis();
		mainLog.println("Starting Prob1 (" + (min ? "min" : "max") + ")...");

		// Special case: no target states
		if (target.cardinality() == 0) {
			return new BitSet(mdp.getNumStates());
		}

		// Initialise vectors
		n = mdp.getNumStates();
		u = new BitSet(n);
		v = new BitSet(n);
		soln = new BitSet(n);

		// Determine set of states actually need to perform computation for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(target);
		if (remain != null)
			unknown.and(remain);

		// Nested fixed point loop
		iters = 0;
		u_done = false;
		// Greatest fixed point
		u.set(0, n);
		while (!u_done) {
			v_done = false;
			// Least fixed point - should start from 0 but we optimise by
			// starting from 'target', thus bypassing first iteration
			v.clear();
			v.or(target);
			soln.clear();
			soln.or(target);
			while (!v_done) {
				iters++;
				// Single step of Prob1
				if (min)
					mdp.prob1Astep(unknown, u, v, soln);
				else
					mdp.prob1Estep(unknown, u, v, soln, null);
				// Check termination (inner)
				v_done = soln.equals(v);
				// v = soln
				v.clear();
				v.or(soln);
			}
			// Check termination (outer)
			u_done = v.equals(u);
			// u = v
			u.clear();
			u.or(v);
		}

		// If we need to generate a strategy, do another iteration of the inner
		// loop for this
		// We could do this during the main double fixed point above, but we
		// would generate surplus
		// strategy info for non-1 states during early iterations of the outer
		// loop,
		// which are not straightforward to remove since this method does not
		// know which states
		// already have valid strategy info from Prob0.
		// Notice that we only need to look at states in u (since we already
		// know the answer),
		// so we restrict 'unknown' further
		unknown.and(u);
		if (!min && strat != null) {
			v_done = false;
			v.clear();
			v.or(target);
			soln.clear();
			soln.or(target);
			while (!v_done) {
				mdp.prob1Estep(unknown, u, v, soln, strat);
				v_done = soln.equals(v);
				v.clear();
				v.or(soln);
			}
			u_done = v.equals(u);
		}

		// Finished precomputation
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Prob1 (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		return u;
	}

	/**
	 * A variation of the original function Compute reachability probabilities using
	 * value iteration. Optionally, store optimal (memoryless) strategy info.
	 * 
	 * @param progStates
	 *            Just the traget states
	 * @param mdp
	 *            The MDP
	 * @param no
	 *            Probability 0 states
	 * @param yes
	 *            Probability 1 states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (will be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values.
	 */
	protected ModelCheckerPartialSatResult computeNestedValIterWS(MDP trimProdMdp, BitSet target,
			MDPRewards progRewards, MDPRewards prodCosts, BitSet progStates) throws PrismException {
		ModelCheckerPartialSatResult res;
		int i, n, iters, numYes, numNo;
		double initValProb, initValRew, initValCost;
		double solnProb[], soln2Prob[];
		double solnProg[], soln2Prog[];
		double solnCost[], soln2Cost[];
		boolean done;
		BitSet no, yes, unknown;
		long timerVI, timerProb0, timerProb1, timerGlobal;
		int strat[] = null;
		boolean min = false;

		timerGlobal = System.currentTimeMillis();

		// Check for deadlocks in non-target state (because breaks e.g. prob1)
		trimProdMdp.checkForDeadlocks(target);

		// Store num states
		n = trimProdMdp.getNumStates();

		// If required, export info about target states
		if (getExportTarget()) {
			BitSet bsInit = new BitSet(n);
			for (i = 0; i < n; i++) {
				bsInit.set(i, trimProdMdp.isInitialState(i));
			}
			List<BitSet> labels = Arrays.asList(bsInit, target);
			List<String> labelNames = Arrays.asList("init", "target");
			mainLog.println("\nExporting target states info to file \"" + getExportTargetFilename() + "\"...");
			PrismLog out = new PrismFileLog(getExportTargetFilename());
			exportLabels(trimProdMdp, labels, labelNames, Prism.EXPORT_PLAIN, out);
			out.close();
		}

		// If required, create/initialise strategy storage
		// Set choices to -1, denoting unknown
		// (except for target states, which are -2, denoting arbitrary)
		if (genStrat || exportAdv) {
			strat = new int[n];
			for (i = 0; i < n; i++) {
				strat[i] = target.get(i) ? -2 : -1;
			}
		}

		// Precomputation
		timerProb0 = System.currentTimeMillis();
		if (precomp && prob0) {
			no = prob0(trimProdMdp, null, target, min, strat);
		} else {
			no = new BitSet();
		}

		// //fixing state choices with total prob less than 1
		// //go over each state choice and if prob less than 1, add an action + cost +
		// other stuff to
		// //the first no state
		// for (int stf = 0; stf<n; stf++)
		// {
		// for(int c = 0; c<trimProdMdp.getNumChoices(stf); c++)
		// {
		// double totalprob = 0;
		// Iterator<Map.Entry<Integer, Double>> iter =
		// trimProdMdp.getTransitionsIterator(stf, c);
		// while(iter.hasNext())
		// {
		// totalprob=totalprob+ ((iter.next()).getValue());
		// }
		// if (totalprob<1)
		// {
		// double remprob = 1.0 - totalprob;
		// int badstate= no.nextClearBit(0);
		//
		// }
		// }
		// }

		timerProb0 = System.currentTimeMillis() - timerProb0;
		timerProb1 = System.currentTimeMillis();
		if (precomp && prob1) {
			yes = prob1(trimProdMdp, null, target, min, strat);
		} else {
			yes = (BitSet) target.clone();
		}
		timerProb1 = System.currentTimeMillis() - timerProb1;

		BitSet inf = prob1(trimProdMdp, null, target, !min, strat);
		inf.flip(0, n);

		// Print results of precomputation
		numYes = yes.cardinality();
		numNo = no.cardinality();
		mainLog.println("target=" + target.cardinality() + ", yes=" + numYes + ", no=" + numNo + ", maybe="
				+ (n - (numYes + numNo)));

		// If still required, store strategy for no/yes (0/1) states.
		// This is just for the cases max=0 and min=1, where arbitrary choices
		// suffice (denoted by -2)
		if (genStrat || exportAdv) {
			if (min) {
				for (i = yes.nextSetBit(0); i >= 0; i = yes.nextSetBit(i + 1)) {
					if (!target.get(i))
						strat[i] = -2;
				}
			} else {
				for (i = no.nextSetBit(0); i >= 0; i = no.nextSetBit(i + 1)) {
					strat[i] = -2;
				}
			}
		}

		// Start value iteration
		timerVI = System.currentTimeMillis();
		mainLog.println("Starting prioritised value iteration (" + (min ? "min" : "max") + ")...");

		// Create solution vector(s)
		solnProb = new double[n];
		solnProg = new double[n];
		solnCost = new double[n];

		// Initialise solution vectors to initVal
		// where initVal is 0.0 or 1.0, depending on whether we converge from
		// below/above.
		initValProb = 0.0;
		initValRew = 0.0;
		initValCost = 0.0;

		// (valIterDir == ValIterDir.BELOW) ? 0.0 : 1.0;

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(yes);
		unknown.andNot(no);
		for (i = 0; i < n; i++) {
			solnProb[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : initValProb;
			// solnProb[i] = target.get(i) ? 1.0 : no.get(i) ? 0.0 :
			// initValProb;
			// solnProg[i] = target.get(i) ? 0.0 : Double.POSITIVE_INFINITY;//
			// initValRew;
			solnProg[i] = target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : 0.0;
			solnCost[i] = initValCost;
		}

		// Start iterations
		iters = 0;
		done = false;

		int j;
		int numChoices;
		double currentProbVal, currentProgVal, currentCostVal;
		boolean sameProb, sameProg, sameCost;

		while (!done && iters < maxIters) {
			iters++;
			done = true;
			for (i = 0; i < n; i++) {

				if (progStates.get(i)) {
					numChoices = trimProdMdp.getNumChoices(i);
					for (j = 0; j < numChoices; j++) {

						currentProbVal = trimProdMdp.mvMultJacSingle(i, j, solnProb);
						currentProgVal = trimProdMdp.mvMultRewSingle(i, j, solnProg, progRewards);
						currentCostVal = trimProdMdp.mvMultRewSingle(i, j, solnCost, prodCosts);
						sameProg = PrismUtils.doublesAreClose(currentProgVal, solnProg[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						sameCost = PrismUtils.doublesAreClose(currentCostVal, solnCost[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						if (!sameProg && currentProgVal < solnProg[i]) {
							done = false;
							solnProb[i] = currentProbVal;
							solnProg[i] = currentProgVal;
							solnCost[i] = currentCostVal;
							if (genStrat || exportAdv) {
								strat[i] = j;
							}
						} else {
							if (sameProg) {
								if (!sameCost && currentCostVal < solnCost[i]) {
									done = false;
									solnCost[i] = currentCostVal;
									solnProb[i] = currentProbVal;
									if (genStrat || exportAdv) {
										strat[i] = j;
									}
								}
							}
						}
					}
				}
			}
		}

		// Finished value iteration
		timerVI = System.currentTimeMillis() - timerVI;
		mainLog.print("Prioritised value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timerVI / 1000.0 + " seconds.");

		timerGlobal = System.currentTimeMillis() - timerGlobal;
		mainLog.println("Overall policy calculation took  " + timerGlobal / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done && errorOnNonConverge) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		res = new ModelCheckerPartialSatResult();
		// Store strategy
		if (genStrat) {
			res.strat = new MDStrategyArray(trimProdMdp, strat);
		}
		// Export adversary
		if (exportAdv) {
			// Prune strategy
			// restrictStrategyToReachableStates(trimProdMdp, strat);
			// Export
			PrismLog out = new PrismFileLog(exportAdvFilename);
			new DTMCFromMDPMemorylessAdversary(trimProdMdp, strat).exportToPrismExplicitTra(out);
			out.close();
		}

		// Return results
		res.solnProb = solnProb;
		res.solnProg = solnProg;
		res.solnCost = solnCost;
		res.numIters = iters;
		res.timeTaken = timerGlobal / 1000.0;
		return res;
	}

	/**
	 * Compute reachability probabilities using value iteration. Optionally, store
	 * optimal (memoryless) strategy info.
	 * 
	 * @param progStates
	 * @param mdp
	 *            The MDP
	 * @param no
	 *            Probability 0 states
	 * @param yes
	 *            Probability 1 states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (will be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values.
	 */
	protected ModelCheckerPartialSatResult computeNestedValIter(MDP trimProdMdp, BitSet target, MDPRewards progRewards,
			MDPRewards prodCosts, BitSet progStates) throws PrismException {
		ModelCheckerPartialSatResult res;
		int i, n, iters, numYes, numNo;
		double initValProb, initValRew, initValCost;
		double solnProb[], soln2Prob[];
		double solnProg[], soln2Prog[];
		double solnCost[], soln2Cost[];
		boolean done;
		BitSet no, yes, unknown;
		long timerVI, timerProb0, timerProb1, timerGlobal;
		int strat[] = null;
		boolean min = false;

		timerGlobal = System.currentTimeMillis();

		// Check for deadlocks in non-target state (because breaks e.g. prob1)
		trimProdMdp.checkForDeadlocks(target);

		// Store num states
		n = trimProdMdp.getNumStates();

		// If required, export info about target states
		if (getExportTarget()) {
			BitSet bsInit = new BitSet(n);
			for (i = 0; i < n; i++) {
				bsInit.set(i, trimProdMdp.isInitialState(i));
			}
			List<BitSet> labels = Arrays.asList(bsInit, target);
			List<String> labelNames = Arrays.asList("init", "target");
			mainLog.println("\nExporting target states info to file \"" + getExportTargetFilename() + "\"...");
			PrismLog out = new PrismFileLog(getExportTargetFilename());
			exportLabels(trimProdMdp, labels, labelNames, Prism.EXPORT_PLAIN, out);
			out.close();
		}

		// If required, create/initialise strategy storage
		// Set choices to -1, denoting unknown
		// (except for target states, which are -2, denoting arbitrary)
		if (genStrat || exportAdv) {
			strat = new int[n];
			for (i = 0; i < n; i++) {
				strat[i] = target.get(i) ? -2 : -1;
			}
		}

		// Precomputation
		timerProb0 = System.currentTimeMillis();
		if (precomp && prob0) {
			no = prob0(trimProdMdp, null, target, min, strat);
		} else {
			no = new BitSet();
		}
		timerProb0 = System.currentTimeMillis() - timerProb0;
		timerProb1 = System.currentTimeMillis();
		if (precomp && prob1) {
			yes = prob1(trimProdMdp, null, target, min, strat);
		} else {
			yes = (BitSet) target.clone();
		}
		timerProb1 = System.currentTimeMillis() - timerProb1;

		// Print results of precomputation
		numYes = yes.cardinality();
		numNo = no.cardinality();
		mainLog.println("target=" + target.cardinality() + ", yes=" + numYes + ", no=" + numNo + ", maybe="
				+ (n - (numYes + numNo)));

		// If still required, store strategy for no/yes (0/1) states.
		// This is just for the cases max=0 and min=1, where arbitrary choices
		// suffice (denoted by -2)
		if (genStrat || exportAdv) {
			if (min) {
				for (i = yes.nextSetBit(0); i >= 0; i = yes.nextSetBit(i + 1)) {
					if (!target.get(i))
						strat[i] = -2;
				}
			} else {
				for (i = no.nextSetBit(0); i >= 0; i = no.nextSetBit(i + 1)) {
					strat[i] = -2;
				}
			}
		}

		// Start value iteration
		timerVI = System.currentTimeMillis();
		mainLog.println("Starting prioritised value iteration (" + (min ? "min" : "max") + ")...");

		// Create solution vector(s)
		solnProb = new double[n];
		// soln2Prob = new double[n];
		solnProg = new double[n];
		// soln2Prog = new double[n];
		solnCost = new double[n];
		// soln2Cost = new double[n];

		// Initialise solution vectors to initVal
		// where initVal is 0.0 or 1.0, depending on whether we converge from
		// below/above.
		initValProb = 0.0;
		initValRew = 0.0;
		initValCost = 0.0;

		// (valIterDir == ValIterDir.BELOW) ? 0.0 : 1.0;

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(yes);
		unknown.andNot(no);
		for (i = 0; i < n; i++) {
			// solnProb[i] = soln2Prob[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 :
			// initValProb;
			// solnProg[i] = soln2Prog[i] = initValRew;
			// solnCost[i] = soln2Cost[i] = initValCost;
			solnProb[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : initValProb;
			solnProg[i] = initValRew;
			solnCost[i] = initValCost;
		}

		// Start iterations
		iters = 0;
		done = false;

		int j;
		int numChoices;
		double currentProbVal, currentProgVal, currentCostVal;
		boolean sameProb, sameProg, sameCost;

		while (!done && iters < maxIters) {
			iters++;
			done = true;
			for (i = 0; i < n; i++) {
				if (progStates.get(i)) {
					numChoices = trimProdMdp.getNumChoices(i);
					for (j = 0; j < numChoices; j++) {
						currentProbVal = trimProdMdp.mvMultJacSingle(i, j, solnProb);
						currentProgVal = trimProdMdp.mvMultRewSingle(i, j, solnProg, progRewards);
						currentCostVal = trimProdMdp.mvMultRewSingle(i, j, solnCost, prodCosts);
						sameProb = PrismUtils.doublesAreClose(currentProbVal, solnProb[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						sameProg = PrismUtils.doublesAreClose(currentProgVal, solnProg[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						sameCost = PrismUtils.doublesAreClose(currentCostVal, solnCost[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						if (!sameProb && currentProbVal > solnProb[i]) {
							done = false;
							solnProb[i] = currentProbVal;
							solnProg[i] = currentProgVal;
							solnCost[i] = currentCostVal;
							if (genStrat || exportAdv) {
								strat[i] = j;
							}
						} else {
							if (sameProb) {
								if (!sameProg && currentProgVal > solnProg[i]) {
									done = false;
									// solnProb[i] = currentProbVal;
									solnProg[i] = currentProgVal;
									solnCost[i] = currentCostVal;
									if (genStrat || exportAdv) {
										strat[i] = j;
									}
								} else {
									if (sameProg) {
										if (!sameCost && currentCostVal < solnCost[i]) {
											done = false;
											// solnProb[i] = currentProbVal;
											// solnProg[i] = currentProgVal;
											solnCost[i] = currentCostVal;
											if (genStrat || exportAdv) {
												strat[i] = j;
											}
										}
									}
								}
							}
						}
					}
				}
			}
			// Check termination
			// done = PrismUtils.doublesAreClose(solnProb, soln2Prob,
			// termCritParam, termCrit == TermCrit.ABSOLUTE);
			// done = done && PrismUtils.doublesAreClose(solnProg, soln2Prog,
			// termCritParam, termCrit == TermCrit.ABSOLUTE);
			// done = done && PrismUtils.doublesAreClose(solnCost, soln2Cost,
			// termCritParam, termCrit == TermCrit.ABSOLUTE);

			// Save previous iter
			// soln2Prob = solnProb.clone();
			// soln2Prog = solnProg.clone();
			// soln2Cost = solnCost.clone();
		}

		// Finished value iteration
		timerVI = System.currentTimeMillis() - timerVI;
		mainLog.print("Prioritised value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timerVI / 1000.0 + " seconds.");

		timerGlobal = System.currentTimeMillis() - timerGlobal;
		mainLog.println("Overall policy calculation took  " + timerGlobal / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done && errorOnNonConverge) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		res = new ModelCheckerPartialSatResult();
		// Store strategy
		if (genStrat) {
			res.strat = new MDStrategyArray(trimProdMdp, strat);
		}
		// Export adversary
		if (exportAdv) {
			// Prune strategy
			// restrictStrategyToReachableStates(trimProdMdp, strat);
			// Export
			PrismLog out = new PrismFileLog(exportAdvFilename);
			new DTMCFromMDPMemorylessAdversary(trimProdMdp, strat).exportToPrismExplicitTra(out);
			out.close();
		}

		// Return results
		res.solnProb = solnProb;
		res.solnProg = solnProg;
		res.solnCost = solnCost;
		res.numIters = iters;
		res.timeTaken = timerGlobal / 1000.0;
		return res;
	}

	/**
	 * Compute reachability probabilities using value iteration. Optionally, store
	 * optimal (memoryless) strategy info.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param no
	 *            Probability 0 states
	 * @param yes
	 *            Probability 1 states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (will be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values.
	 */
	protected ModelCheckerResult computeReachProbsValIter(MDP mdp, BitSet no, BitSet yes, boolean min, double init[],
			BitSet known, int strat[]) throws PrismException {
		ModelCheckerResult res;
		BitSet unknown;
		int i, n, iters;
		double soln[], soln2[], tmpsoln[], initVal;
		boolean done;
		long timer;
		// Start value iteration
		timer = System.currentTimeMillis();
		mainLog.println("Starting value iteration (" + (min ? "min" : "max") + ")...");

		// Store num states
		n = mdp.getNumStates();

		// Create solution vector(s)
		soln = new double[n];
		soln2 = (init == null) ? new double[n] : init;

		// Initialise solution vectors. Use (where available) the following in
		// order of preference:
		// (1) exact answer, if already known; (2) 1.0/0.0 if in yes/no; (3)
		// passed in initial value; (4) initVal
		// where initVal is 0.0 or 1.0, depending on whether we converge from
		// below/above.
		initVal = (valIterDir == ValIterDir.BELOW) ? 0.0 : 1.0;
		if (init != null) {
			if (known != null) {
				for (i = 0; i < n; i++)
					soln[i] = soln2[i] = known.get(i) ? init[i] : yes.get(i) ? 1.0 : no.get(i) ? 0.0 : init[i];
			} else {
				for (i = 0; i < n; i++)
					soln[i] = soln2[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : init[i];
			}
		} else {
			for (i = 0; i < n; i++)
				soln[i] = soln2[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : initVal;
		}

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(yes);
		unknown.andNot(no);
		if (known != null)
			unknown.andNot(known);

		// Start iterations
		iters = 0;
		done = false;
		while (!done && iters < maxIters) {
			iters++;
			// Matrix-vector multiply and min/max ops
			mdp.mvMultMinMax(soln, min, soln2, unknown, false, strat);
			// Check termination
			done = PrismUtils.doublesAreClose(soln, soln2, termCritParam, termCrit == TermCrit.ABSOLUTE);
			// Swap vectors for next iter
			tmpsoln = soln;
			soln = soln2;
			soln2 = tmpsoln;
		}

		// Finished value iteration
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done && errorOnNonConverge) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = iters;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Compute reachability probabilities using Gauss-Seidel (including Jacobi-style
	 * updates).
	 * 
	 * @param mdp
	 *            The MDP
	 * @param no
	 *            Probability 0 states
	 * @param yes
	 *            Probability 1 states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (will be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values.
	 */
	protected ModelCheckerResult computeReachProbsGaussSeidel(MDP mdp, BitSet no, BitSet yes, boolean min,
			double init[], BitSet known, int strat[]) throws PrismException {
		ModelCheckerResult res;
		BitSet unknown;
		int i, n, iters;
		double soln[], initVal, maxDiff;
		boolean done;
		long timer;

		// Start value iteration
		timer = System.currentTimeMillis();
		mainLog.println("Starting Gauss-Seidel (" + (min ? "min" : "max") + ")...");

		// Store num states
		n = mdp.getNumStates();

		// Create solution vector
		soln = (init == null) ? new double[n] : init;

		// Initialise solution vector. Use (where available) the following in
		// order of preference:
		// (1) exact answer, if already known; (2) 1.0/0.0 if in yes/no; (3)
		// passed in initial value; (4) initVal
		// where initVal is 0.0 or 1.0, depending on whether we converge from
		// below/above.
		initVal = (valIterDir == ValIterDir.BELOW) ? 0.0 : 1.0;
		if (init != null) {
			if (known != null) {
				for (i = 0; i < n; i++)
					soln[i] = known.get(i) ? init[i] : yes.get(i) ? 1.0 : no.get(i) ? 0.0 : init[i];
			} else {
				for (i = 0; i < n; i++)
					soln[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : init[i];
			}
		} else {
			for (i = 0; i < n; i++)
				soln[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : initVal;
		}

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(yes);
		unknown.andNot(no);
		if (known != null)
			unknown.andNot(known);

		// Start iterations
		iters = 0;
		done = false;
		while (!done && iters < maxIters) {
			iters++;
			// Matrix-vector multiply
			maxDiff = mdp.mvMultGSMinMax(soln, min, unknown, false, termCrit == TermCrit.ABSOLUTE, strat);
			// Check termination
			done = maxDiff < termCritParam;
		}

		// Finished Gauss-Seidel
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Gauss-Seidel");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done && errorOnNonConverge) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = iters;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Compute reachability probabilities using policy iteration. Optionally, store
	 * optimal (memoryless) strategy info.
	 * 
	 * @param mdp:
	 *            The MDP
	 * @param no:
	 *            Probability 0 states
	 * @param yes:
	 *            Probability 1 states
	 * @param min:
	 *            Min or max probabilities (true=min, false=max)
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 */
	protected ModelCheckerResult computeReachProbsPolIter(MDP mdp, BitSet no, BitSet yes, boolean min, int strat[])
			throws PrismException {
		ModelCheckerResult res;
		int i, n, iters, totalIters;
		double soln[], soln2[];
		boolean done;
		long timer;
		DTMCModelChecker mcDTMC;
		DTMC dtmc;

		// Re-use solution to solve each new policy (strategy)?
		boolean reUseSoln = true;

		// Start policy iteration
		timer = System.currentTimeMillis();
		mainLog.println("Starting policy iteration (" + (min ? "min" : "max") + ")...");

		// Create a DTMC model checker (for solving policies)
		mcDTMC = new DTMCModelChecker(this);
		mcDTMC.inheritSettings(this);
		mcDTMC.setLog(new PrismDevNullLog());

		// Store num states
		n = mdp.getNumStates();

		// Create solution vectors
		soln = new double[n];
		soln2 = new double[n];

		// Initialise solution vectors.
		for (i = 0; i < n; i++)
			soln[i] = soln2[i] = yes.get(i) ? 1.0 : 0.0;

		// If not passed in, create new storage for strategy and initialise
		// Initial strategy just picks first choice (0) everywhere
		if (strat == null) {
			strat = new int[n];
			for (i = 0; i < n; i++)
				strat[i] = 0;
		}
		// Otherwise, just initialise for states not in yes/no
		// (Optimal choices for yes/no should already be known)
		else {
			for (i = 0; i < n; i++)
				if (!(no.get(i) || yes.get(i)))
					strat[i] = 0;
		}

		// Start iterations
		iters = totalIters = 0;
		done = false;
		while (!done) {
			iters++;
			// Solve induced DTMC for strategy
			dtmc = new DTMCFromMDPMemorylessAdversary(mdp, strat);
			res = mcDTMC.computeReachProbsGaussSeidel(dtmc, no, yes, reUseSoln ? soln : null, null);
			soln = res.soln;
			totalIters += res.numIters;
			// Check if optimal, improve non-optimal choices
			mdp.mvMultMinMax(soln, min, soln2, null, false, null);
			done = true;
			for (i = 0; i < n; i++) {
				// Don't look at no/yes states - we may not have strategy info
				// for them,
				// so they might appear non-optimal
				if (no.get(i) || yes.get(i))
					continue;
				if (!PrismUtils.doublesAreClose(soln[i], soln2[i], termCritParam, termCrit == TermCrit.ABSOLUTE)) {
					done = false;
					List<Integer> opt = mdp.mvMultMinMaxSingleChoices(i, soln, min, soln2[i]);
					// Only update strategy if strictly better
					if (!opt.contains(strat[i]))
						strat[i] = opt.get(0);
				}
			}
		}

		// Finished policy iteration
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Policy iteration");
		mainLog.println(" took " + iters + " cycles (" + totalIters + " iterations in total) and " + timer / 1000.0
				+ " seconds.");

		// Return results
		// (Note we don't add the strategy - the one passed in is already there
		// and might have some existing choices stored for other states).
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = totalIters;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Compute reachability probabilities using modified policy iteration.
	 * 
	 * @param mdp:
	 *            The MDP
	 * @param no:
	 *            Probability 0 states
	 * @param yes:
	 *            Probability 1 states
	 * @param min:
	 *            Min or max probabilities (true=min, false=max)
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 */
	protected ModelCheckerResult computeReachProbsModPolIter(MDP mdp, BitSet no, BitSet yes, boolean min, int strat[])
			throws PrismException {
		ModelCheckerResult res;
		int i, n, iters, totalIters;
		double soln[], soln2[];
		boolean done;
		long timer;
		DTMCModelChecker mcDTMC;
		DTMC dtmc;

		// Start value iteration
		timer = System.currentTimeMillis();
		mainLog.println("Starting modified policy iteration (" + (min ? "min" : "max") + ")...");

		// Create a DTMC model checker (for solving policies)
		mcDTMC = new DTMCModelChecker(this);
		mcDTMC.inheritSettings(this);
		mcDTMC.setLog(new PrismDevNullLog());

		// Limit iters for DTMC solution - this implements "modified" policy
		// iteration
		mcDTMC.setMaxIters(100);
		mcDTMC.setErrorOnNonConverge(false);

		// Store num states
		n = mdp.getNumStates();

		// Create solution vectors
		soln = new double[n];
		soln2 = new double[n];

		// Initialise solution vectors.
		for (i = 0; i < n; i++)
			soln[i] = soln2[i] = yes.get(i) ? 1.0 : 0.0;

		// If not passed in, create new storage for strategy and initialise
		// Initial strategy just picks first choice (0) everywhere
		if (strat == null) {
			strat = new int[n];
			for (i = 0; i < n; i++)
				strat[i] = 0;
		}
		// Otherwise, just initialise for states not in yes/no
		// (Optimal choices for yes/no should already be known)
		else {
			for (i = 0; i < n; i++)
				if (!(no.get(i) || yes.get(i)))
					strat[i] = 0;
		}

		// Start iterations
		iters = totalIters = 0;
		done = false;
		while (!done) {
			iters++;
			// Solve induced DTMC for strategy
			dtmc = new DTMCFromMDPMemorylessAdversary(mdp, strat);
			res = mcDTMC.computeReachProbsGaussSeidel(dtmc, no, yes, soln, null);
			soln = res.soln;
			totalIters += res.numIters;
			// Check if optimal, improve non-optimal choices
			mdp.mvMultMinMax(soln, min, soln2, null, false, null);
			done = true;
			for (i = 0; i < n; i++) {
				// Don't look at no/yes states - we don't store strategy info
				// for them,
				// so they might appear non-optimal
				if (no.get(i) || yes.get(i))
					continue;
				if (!PrismUtils.doublesAreClose(soln[i], soln2[i], termCritParam, termCrit == TermCrit.ABSOLUTE)) {
					done = false;
					List<Integer> opt = mdp.mvMultMinMaxSingleChoices(i, soln, min, soln2[i]);
					strat[i] = opt.get(0);
				}
			}
		}

		// Finished policy iteration
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Modified policy iteration");
		mainLog.println(" took " + iters + " cycles (" + totalIters + " iterations in total) and " + timer / 1000.0
				+ " seconds.");

		// Return results
		// (Note we don't add the strategy - the one passed in is already there
		// and might have some existing choices stored for other states).
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = totalIters;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Construct strategy information for min/max reachability probabilities. (More
	 * precisely, list of indices of choices resulting in min/max.) (Note: indices
	 * are guaranteed to be sorted in ascending order.)
	 * 
	 * @param mdp
	 *            The MDP
	 * @param state
	 *            The state to generate strategy info for
	 * @param target
	 *            The set of target states to reach
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param lastSoln
	 *            Vector of values from which to recompute in one iteration
	 */
	public List<Integer> probReachStrategy(MDP mdp, int state, BitSet target, boolean min, double lastSoln[])
			throws PrismException {
		double val = mdp.mvMultMinMaxSingle(state, lastSoln, min, null);
		return mdp.mvMultMinMaxSingleChoices(state, lastSoln, min, val);
	}

	/**
	 * Compute bounded reachability probabilities. i.e. compute the min/max
	 * probability of reaching a state in {@code target} within k steps.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param target
	 *            Target states
	 * @param k
	 *            Bound
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 */
	public ModelCheckerResult computeBoundedReachProbs(MDP mdp, BitSet target, int k, boolean min)
			throws PrismException {
		return computeBoundedReachProbs(mdp, null, target, k, min, null, null);
	}

	/**
	 * Compute bounded until probabilities. i.e. compute the min/max probability of
	 * reaching a state in {@code target}, within k steps, and while remaining in
	 * states in {@code remain}.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param remain
	 *            Remain in these states (optional: null means "all")
	 * @param target
	 *            Target states
	 * @param k
	 *            Bound
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 */
	public ModelCheckerResult computeBoundedUntilProbs(MDP mdp, BitSet remain, BitSet target, int k, boolean min)
			throws PrismException {
		return computeBoundedReachProbs(mdp, remain, target, k, min, null, null);
	}

	/**
	 * Compute bounded reachability/until probabilities. i.e. compute the min/max
	 * probability of reaching a state in {@code target}, within k steps, and while
	 * remaining in states in {@code remain}.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param remain
	 *            Remain in these states (optional: null means "all")
	 * @param target
	 *            Target states
	 * @param k
	 *            Bound
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (may be overwritten)
	 * @param results
	 *            Optional array of size k+1 to store (init state) results for each
	 *            step (null if unused)
	 */
	public ModelCheckerResult computeBoundedReachProbs(MDP mdp, BitSet remain, BitSet target, int k, boolean min,
			double init[], double results[]) throws PrismException {
		ModelCheckerResult res = null;
		BitSet unknown;
		int i, n, iters;
		double soln[], soln2[], tmpsoln[];
		long timer;

		// Start bounded probabilistic reachability
		timer = System.currentTimeMillis();
		mainLog.println("\nStarting bounded probabilistic reachability (" + (min ? "min" : "max") + ")...");

		// Store num states
		n = mdp.getNumStates();

		// Create solution vector(s)
		soln = new double[n];
		soln2 = (init == null) ? new double[n] : init;

		// Initialise solution vectors. Use passed in initial vector, if present
		if (init != null) {
			for (i = 0; i < n; i++)
				soln[i] = soln2[i] = target.get(i) ? 1.0 : init[i];
		} else {
			for (i = 0; i < n; i++)
				soln[i] = soln2[i] = target.get(i) ? 1.0 : 0.0;
		}
		// Store intermediate results if required
		// (compute min/max value over initial states for first step)
		if (results != null) {
			// TODO: whether this is min or max should be specified somehow
			results[0] = Utils.minMaxOverArraySubset(soln2, mdp.getInitialStates(), true);
		}

		// Determine set of states actually need to perform computation for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(target);
		if (remain != null)
			unknown.and(remain);

		// Start iterations
		iters = 0;
		while (iters < k) {
			iters++;
			// Matrix-vector multiply and min/max ops
			mdp.mvMultMinMax(soln, min, soln2, unknown, false, null);
			// Store intermediate results if required
			// (compute min/max value over initial states for this step)
			if (results != null) {
				// TODO: whether this is min or max should be specified somehow
				results[iters] = Utils.minMaxOverArraySubset(soln2, mdp.getInitialStates(), true);
			}
			// Swap vectors for next iter
			tmpsoln = soln;
			soln = soln2;
			soln2 = tmpsoln;
		}

		// Finished bounded probabilistic reachability
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Bounded probabilistic reachability (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.lastSoln = soln2;
		res.numIters = iters;
		res.timeTaken = timer / 1000.0;
		res.timePre = 0.0;
		return res;
	}

	/**
	 * Compute expected cumulative (step-bounded) rewards. i.e. compute the min/max
	 * reward accumulated within {@code k} steps.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 */
	public ModelCheckerResult computeCumulativeRewards(MDP mdp, MDPRewards mdpRewards, int k, boolean min)
			throws PrismException {
		ModelCheckerResult res = null;
		int i, n, iters;
		long timer;
		double soln[], soln2[], tmpsoln[];

		// Start expected cumulative reward
		timer = System.currentTimeMillis();
		mainLog.println("\nStarting expected cumulative reward (" + (min ? "min" : "max") + ")...");

		// Store num states
		n = mdp.getNumStates();

		// Create/initialise solution vector(s)
		soln = new double[n];
		soln2 = new double[n];
		for (i = 0; i < n; i++)
			soln[i] = soln2[i] = 0.0;

		// Start iterations
		iters = 0;
		while (iters < k) {
			iters++;
			// Matrix-vector multiply and min/max ops
			mdp.mvMultRewMinMax(soln, mdpRewards, min, soln2, null, false, null);
			// Swap vectors for next iter
			tmpsoln = soln;
			soln = soln2;
			soln2 = tmpsoln;
		}

		// Finished value iteration
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Expected cumulative reward (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = iters;
		res.timeTaken = timer / 1000.0;

		return res;
	}

	/**
	 * Compute expected instantaneous reward, i.e. compute the min/max expected
	 * reward of the states after {@code k} steps.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param k
	 *            the number of steps
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 */
	public ModelCheckerResult computeInstantaneousRewards(MDP mdp, MDPRewards mdpRewards, final int k, boolean min) {
		ModelCheckerResult res = null;
		int i, n, iters;
		double soln[], soln2[], tmpsoln[];
		long timer;

		// Store num states
		n = mdp.getNumStates();

		// Start backwards transient computation
		timer = System.currentTimeMillis();
		mainLog.println("\nStarting backwards instantaneous rewards computation...");

		// Create solution vector(s)
		soln = new double[n];
		soln2 = new double[n];

		// Initialise solution vectors.
		for (i = 0; i < n; i++)
			soln[i] = mdpRewards.getStateReward(i);

		// Start iterations
		for (iters = 0; iters < k; iters++) {
			// Matrix-vector multiply
			mdp.mvMultMinMax(soln, min, soln2, null, false, null);
			// Swap vectors for next iter
			tmpsoln = soln;
			soln = soln2;
			soln2 = tmpsoln;
		}

		// Finished backwards transient computation
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Backwards transient instantaneous rewards computation");
		mainLog.println(" took " + iters + " iters and " + timer / 1000.0 + " seconds.");

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.lastSoln = soln2;
		res.numIters = iters;
		res.timeTaken = timer / 1000.0;
		res.timePre = 0.0;
		return res;
	}

	/**
	 * Compute expected reachability rewards.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 */
	public ModelCheckerResult computeReachRewards(MDP mdp, MDPRewards mdpRewards, BitSet target, boolean min)
			throws PrismException {
		return computeReachRewards(mdp, mdpRewards, target, min, null, null);
	}

	/**
	 * Compute expected reachability rewards. i.e. compute the min/max reward
	 * accumulated to reach a state in {@code target}.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param target
	 *            Target states
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (may be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values). Also, 'known' values
	 *            cannot be passed for some solution methods, e.g. policy iteration.
	 */
	public ModelCheckerResult computeReachRewards(MDP mdp, MDPRewards mdpRewards, BitSet target, boolean min,
			double init[], BitSet known) throws PrismException {
		ModelCheckerResult res = null;
		BitSet inf;
		int n, numTarget, numInf;
		long timer, timerProb1;
		int strat[] = null;
		// Local copy of setting
		MDPSolnMethod mdpSolnMethod = this.mdpSolnMethod;

		// Switch to a supported method, if necessary
		if (!(mdpSolnMethod == MDPSolnMethod.VALUE_ITERATION || mdpSolnMethod == MDPSolnMethod.GAUSS_SEIDEL
				|| mdpSolnMethod == MDPSolnMethod.POLICY_ITERATION)) {
			mdpSolnMethod = MDPSolnMethod.GAUSS_SEIDEL;
			mainLog.printWarning("Switching to MDP solution method \"" + mdpSolnMethod.fullName() + "\"");
		}

		// Check for some unsupported combinations
		if (mdpSolnMethod == MDPSolnMethod.POLICY_ITERATION) {
			if (known != null) {
				throw new PrismException("Policy iteration methods cannot be passed 'known' values for some states");
			}
		}

		// Start expected reachability
		timer = System.currentTimeMillis();
		mainLog.println("\nStarting expected reachability (" + (min ? "min" : "max") + ")...");

		// Check for deadlocks in non-target state (because breaks e.g. prob1)
		mdp.checkForDeadlocks(target);

		// Store num states
		n = mdp.getNumStates();
		// Optimise by enlarging target set (if more info is available)
		if (init != null && known != null && !known.isEmpty()) {
			BitSet targetNew = (BitSet) target.clone();
			for (int i : new IterableBitSet(known)) {
				if (init[i] == 1.0) {
					targetNew.set(i);
				}
			}
			target = targetNew;
		}

		// If required, export info about target states
		if (getExportTarget()) {
			BitSet bsInit = new BitSet(n);
			for (int i = 0; i < n; i++) {
				bsInit.set(i, mdp.isInitialState(i));
			}
			List<BitSet> labels = Arrays.asList(bsInit, target);
			List<String> labelNames = Arrays.asList("init", "target");
			mainLog.println("\nExporting target states info to file \"" + getExportTargetFilename() + "\"...");
			exportLabels(mdp, labels, labelNames, Prism.EXPORT_PLAIN, new PrismFileLog(getExportTargetFilename()));
		}

		// If required, create/initialise strategy storage
		// Set choices to -1, denoting unknown
		// (except for target states, which are -2, denoting arbitrary)
		if (genStrat || exportAdv || mdpSolnMethod == MDPSolnMethod.POLICY_ITERATION) {
			strat = new int[n];
			for (int i = 0; i < n; i++) {
				strat[i] = target.get(i) ? -2 : -1;
			}
		}

		// Precomputation (not optional)
		timerProb1 = System.currentTimeMillis();
		inf = prob1(mdp, null, target, !min, strat);
		inf.flip(0, n);
		timerProb1 = System.currentTimeMillis() - timerProb1;

		// Print results of precomputation
		numTarget = target.cardinality();
		numInf = inf.cardinality();
		mainLog.println("target=" + numTarget + ", inf=" + numInf + ", rest=" + (n - (numTarget + numInf)));

		// If required, generate strategy for "inf" states.
		if (genStrat || exportAdv || mdpSolnMethod == MDPSolnMethod.POLICY_ITERATION) {
			if (min) {
				// If min reward is infinite, all choices give infinity
				// So the choice can be arbitrary, denoted by -2;
				for (int i = inf.nextSetBit(0); i >= 0; i = inf.nextSetBit(i + 1)) {
					strat[i] = -2;
				}
			} else {
				// If max reward is infinite, there is at least one choice
				// giving infinity.
				// So we pick, for all "inf" states, the first choice for which
				// some transitions stays in "inf".
				for (int i = inf.nextSetBit(0); i >= 0; i = inf.nextSetBit(i + 1)) {
					int numChoices = mdp.getNumChoices(i);
					for (int k = 0; k < numChoices; k++) {
						if (mdp.someSuccessorsInSet(i, k, inf)) {
							strat[i] = k;
							continue;
						}
					}
				}
			}
		}

		// Compute rewards
		switch (mdpSolnMethod) {
		case VALUE_ITERATION:
			res = computeReachRewardsValIter(mdp, mdpRewards, target, inf, min, init, known, strat);
			break;
		case GAUSS_SEIDEL:
			res = computeReachRewardsGaussSeidel(mdp, mdpRewards, target, inf, min, init, known, strat);
			break;
		case POLICY_ITERATION:
			res = computeReachRewardsPolIter(mdp, mdpRewards, target, inf, min, strat);
			break;
		default:
			throw new PrismException("Unknown MDP solution method " + mdpSolnMethod.fullName());
		}

		// Store strategy
		if (genStrat) {
			res.strat = new MDStrategyArray(mdp, strat);
		}
		// Export adversary
		if (exportAdv) {
			// Prune strategy
			restrictStrategyToReachableStates(mdp, strat);
			// Export
			PrismLog out = new PrismFileLog(exportAdvFilename);
			new DTMCFromMDPMemorylessAdversary(mdp, strat).exportToPrismExplicitTra(out);
			out.close();
		}

		// Finished expected reachability
		timer = System.currentTimeMillis() - timer;
		mainLog.println("Expected reachability took " + timer / 1000.0 + " seconds.");

		// Update time taken
		res.timeTaken = timer / 1000.0;
		res.timePre = timerProb1 / 1000.0;

		return res;
	}

	/**
	 * Compute expected reachability rewards using value iteration. Optionally,
	 * store optimal (memoryless) strategy info.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param target
	 *            Target states
	 * @param inf
	 *            States for which reward is infinite
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (will be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values.
	 */
	protected ModelCheckerResult computeReachRewardsValIter(MDP mdp, MDPRewards mdpRewards, BitSet target, BitSet inf,
			boolean min, double init[], BitSet known, int strat[]) throws PrismException {
		ModelCheckerResult res;
		BitSet unknown;
		int i, n, iters;
		double soln[], soln2[], tmpsoln[];
		boolean done;
		long timer;

		// Start value iteration
		timer = System.currentTimeMillis();
		mainLog.println("Starting value iteration (" + (min ? "min" : "max") + ")...");

		// Store num states
		n = mdp.getNumStates();

		// Create solution vector(s)
		soln = new double[n];
		soln2 = (init == null) ? new double[n] : init;

		// Initialise solution vectors. Use (where available) the following in
		// order of preference:
		// (1) exact answer, if already known; (2) 0.0/infinity if in
		// target/inf; (3) passed in initial value; (4) 0.0
		if (init != null) {
			if (known != null) {
				for (i = 0; i < n; i++)
					soln[i] = soln2[i] = known.get(i) ? init[i]
							: target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : init[i];
			} else {
				for (i = 0; i < n; i++)
					soln[i] = soln2[i] = target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : init[i];
			}
		} else {
			for (i = 0; i < n; i++)
				soln[i] = soln2[i] = target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : 0.0;
		}

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(target);
		unknown.andNot(inf);
		if (known != null)
			unknown.andNot(known);

		// Start iterations
		iters = 0;
		done = false;
		while (!done && iters < maxIters) {
			// mainLog.println(soln);
			iters++;
			// Matrix-vector multiply and min/max ops
			mdp.mvMultRewMinMax(soln, mdpRewards, min, soln2, unknown, false, strat);
			// Check termination
			done = PrismUtils.doublesAreClose(soln, soln2, termCritParam, termCrit == TermCrit.ABSOLUTE);
			// Swap vectors for next iter
			tmpsoln = soln;
			soln = soln2;
			soln2 = tmpsoln;
		}

		// Finished value iteration
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done && errorOnNonConverge) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = iters;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Compute expected reachability rewards using Gauss-Seidel (including
	 * Jacobi-style updates). Optionally, store optimal (memoryless) strategy info.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param target
	 *            Target states
	 * @param inf
	 *            States for which reward is infinite
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (will be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values.
	 */
	protected ModelCheckerResult computeReachRewardsGaussSeidel(MDP mdp, MDPRewards mdpRewards, BitSet target,
			BitSet inf, boolean min, double init[], BitSet known, int strat[]) throws PrismException {
		ModelCheckerResult res;
		BitSet unknown;
		int i, n, iters;
		double soln[], maxDiff;
		boolean done;
		long timer;

		// Start value iteration
		timer = System.currentTimeMillis();
		mainLog.println("Starting Gauss-Seidel (" + (min ? "min" : "max") + ")...");

		// Store num states
		n = mdp.getNumStates();

		// Create solution vector(s)
		soln = (init == null) ? new double[n] : init;

		// Initialise solution vector. Use (where available) the following in
		// order of preference:
		// (1) exact answer, if already known; (2) 0.0/infinity if in
		// target/inf; (3) passed in initial value; (4) 0.0
		if (init != null) {
			if (known != null) {
				for (i = 0; i < n; i++)
					soln[i] = known.get(i) ? init[i]
							: target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : init[i];
			} else {
				for (i = 0; i < n; i++)
					soln[i] = target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : init[i];
			}
		} else {
			for (i = 0; i < n; i++)
				soln[i] = target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : 0.0;
		}

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(target);
		unknown.andNot(inf);
		if (known != null)
			unknown.andNot(known);

		// Start iterations
		iters = 0;
		done = false;
		while (!done && iters < maxIters) {
			// mainLog.println(soln);
			iters++;
			// Matrix-vector multiply and min/max ops
			maxDiff = mdp.mvMultRewGSMinMax(soln, mdpRewards, min, unknown, false, termCrit == TermCrit.ABSOLUTE,
					strat);
			// Check termination
			done = maxDiff < termCritParam;
		}

		// Finished Gauss-Seidel
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Gauss-Seidel (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timer / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done && errorOnNonConverge) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = iters;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Compute expected reachability rewards using policy iteration. The array
	 * {@code strat} is used both to pass in the initial strategy for policy
	 * iteration, and as storage for the resulting optimal strategy (if needed).
	 * Passing in an initial strategy is required when some states have infinite
	 * reward, to avoid the possibility of policy iteration getting stuck on an
	 * infinite-value strategy.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param target
	 *            Target states
	 * @param inf
	 *            States for which reward is infinite
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 */
	protected ModelCheckerResult computeReachRewardsPolIter(MDP mdp, MDPRewards mdpRewards, BitSet target, BitSet inf,
			boolean min, int strat[]) throws PrismException {
		ModelCheckerResult res;
		int i, n, iters, totalIters;
		double soln[], soln2[];
		boolean done;
		long timer;
		DTMCModelChecker mcDTMC;
		DTMC dtmc;
		MCRewards mcRewards;

		// Re-use solution to solve each new policy (strategy)?
		boolean reUseSoln = true;

		// Start policy iteration
		timer = System.currentTimeMillis();
		mainLog.println("Starting policy iteration (" + (min ? "min" : "max") + ")...");

		// Create a DTMC model checker (for solving policies)
		mcDTMC = new DTMCModelChecker(this);
		mcDTMC.inheritSettings(this);
		mcDTMC.setLog(new PrismDevNullLog());

		// Store num states
		n = mdp.getNumStates();

		// Create solution vector(s)
		soln = new double[n];
		soln2 = new double[n];

		// Initialise solution vectors.
		for (i = 0; i < n; i++)
			soln[i] = soln2[i] = target.get(i) ? 0.0 : inf.get(i) ? Double.POSITIVE_INFINITY : 0.0;

		// If not passed in, create new storage for strategy and initialise
		// Initial strategy just picks first choice (0) everywhere
		if (strat == null) {
			strat = new int[n];
			for (i = 0; i < n; i++)
				strat[i] = 0;
		}

		// Start iterations
		iters = totalIters = 0;
		done = false;
		while (!done && iters < maxIters) {
			iters++;
			// Solve induced DTMC for strategy
			dtmc = new DTMCFromMDPMemorylessAdversary(mdp, strat);
			mcRewards = new MCRewardsFromMDPRewards(mdpRewards, strat);
			res = mcDTMC.computeReachRewardsValIter(dtmc, mcRewards, target, inf, reUseSoln ? soln : null, null);
			soln = res.soln;
			totalIters += res.numIters;
			// Check if optimal, improve non-optimal choices
			mdp.mvMultRewMinMax(soln, mdpRewards, min, soln2, null, false, null);
			done = true;
			for (i = 0; i < n; i++) {
				// Don't look at target/inf states - we may not have strategy
				// info for them,
				// so they might appear non-optimal
				if (target.get(i) || inf.get(i))
					continue;
				if (!PrismUtils.doublesAreClose(soln[i], soln2[i], termCritParam, termCrit == TermCrit.ABSOLUTE)) {
					done = false;
					List<Integer> opt = mdp.mvMultRewMinMaxSingleChoices(i, soln, mdpRewards, min, soln2[i]);
					// Only update strategy if strictly better
					if (!opt.contains(strat[i]))
						strat[i] = opt.get(0);
				}
			}
		}

		// Finished policy iteration
		timer = System.currentTimeMillis() - timer;
		mainLog.print("Policy iteration");
		mainLog.println(" took " + iters + " cycles (" + totalIters + " iterations in total) and " + timer / 1000.0
				+ " seconds.");

		// Return results
		res = new ModelCheckerResult();
		res.soln = soln;
		res.numIters = totalIters;
		res.timeTaken = timer / 1000.0;
		return res;
	}

	/**
	 * Construct strategy information for min/max expected reachability. (More
	 * precisely, list of indices of choices resulting in min/max.) (Note: indices
	 * are guaranteed to be sorted in ascending order.)
	 * 
	 * @param mdp
	 *            The MDP
	 * @param mdpRewards
	 *            The rewards
	 * @param state
	 *            The state to generate strategy info for
	 * @param target
	 *            The set of target states to reach
	 * @param min
	 *            Min or max rewards (true=min, false=max)
	 * @param lastSoln
	 *            Vector of values from which to recompute in one iteration
	 */
	public List<Integer> expReachStrategy(MDP mdp, MDPRewards mdpRewards, int state, BitSet target, boolean min,
			double lastSoln[]) throws PrismException {
		double val = mdp.mvMultRewMinMaxSingle(state, lastSoln, mdpRewards, min, null);
		return mdp.mvMultRewMinMaxSingleChoices(state, lastSoln, mdpRewards, min, val);
	}

	/**
	 * Restrict a (memoryless) strategy for an MDP, stored as an integer array of
	 * choice indices, to the states of the MDP that are reachable under that
	 * strategy.
	 * 
	 * @param mdp
	 *            The MDP
	 * @param strat
	 *            The strategy
	 */
	public BitSet restrictStrategyToReachableStates(MDP mdp, int strat[]) {
		BitSet restrict = new BitSet();
		BitSet explore = new BitSet();
		// Get initial states
		for (int is : mdp.getInitialStates()) {
			restrict.set(is);
			explore.set(is);
		}
		// Compute reachable states (store in 'restrict')
		boolean foundMore = true;
		while (foundMore) {
			foundMore = false;
			for (int s = explore.nextSetBit(0); s >= 0; s = explore.nextSetBit(s + 1)) {
				explore.set(s, false);
				if (strat[s] >= 0) {
					Iterator<Map.Entry<Integer, Double>> iter = mdp.getTransitionsIterator(s, strat[s]);
					while (iter.hasNext()) {
						Map.Entry<Integer, Double> e = iter.next();
						int dest = e.getKey();
						if (!restrict.get(dest)) {
							foundMore = true;
							restrict.set(dest);
							explore.set(dest);
						}
					}
				}
			}
		}
		// Set strategy choice for non-reachable state to -1
		int n = mdp.getNumStates();
		for (int s = restrict.nextClearBit(0); s < n; s = restrict.nextClearBit(s + 1)) {
			strat[s] = -3;
		}
		return restrict;
	}

	public BitSet restrictStrategyToReachableStates(MDP mdp, MDStrategy strat) {
		int n, intStrat[];
		BitSet res;

		n = strat.getNumStates();
		intStrat = new int[n];

		for (int i = 0; i < n; i++) {
			intStrat[i] = strat.getChoiceIndex(i);
		}
		res = restrictStrategyToReachableStates(mdp, intStrat);
		strat = new MDStrategyArray(mdp, intStrat);
		return res;
	}

	/**
	 * Simple test program.
	 */
	public static void main(String args[]) {
		MDPModelChecker mc;
		MDPSimple mdp;
		ModelCheckerResult res;
		BitSet init, target;
		Map<String, BitSet> labels;
		boolean min = true;
		try {
			mc = new MDPModelChecker(null);
			mdp = new MDPSimple();
			mdp.buildFromPrismExplicit(args[0]);
			mdp.addInitialState(0);
			// System.out.println(mdp);
			labels = StateModelChecker.loadLabelsFile(args[1]);
			// System.out.println(labels);
			init = labels.get("init");
			target = labels.get(args[2]);
			if (target == null)
				throw new PrismException("Unknown label \"" + args[2] + "\"");
			for (int i = 3; i < args.length; i++) {
				if (args[i].equals("-min"))
					min = true;
				else if (args[i].equals("-max"))
					min = false;
				else if (args[i].equals("-nopre"))
					mc.setPrecomp(false);
			}
			res = mc.computeReachProbs(mdp, target, min);
			System.out.println(res.soln[init.nextSetBit(0)]);
		} catch (PrismException e) {
			System.out.println(e);
		}
	}
}
