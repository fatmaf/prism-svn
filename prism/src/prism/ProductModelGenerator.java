//==============================================================================
//	
//	Copyright (c) 2002-
//	Authors:
//	* Dave Parker <d.a.parker@cs.bham.ac.uk> (University of Birmingham)
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

package prism;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceReach;
import automata.DA;
import parser.State;
import parser.Values;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.Expression;
import parser.ast.RewardStruct;
import parser.type.Type;
import parser.type.TypeInt;

public class ProductModelGenerator implements ModelGenerator
{
	protected ModelGenerator modelGen = null;
	/** The DA used to build the product */
	protected DA<BitSet, ? extends AcceptanceOmega> da = null;
	/** The expressions for the labels (APs) in the DA */
	protected List<Expression> labelExprs = null;
	
	/** Variable name for DA state */
	protected String daVar;
	/** Number of APs in the DA */
	protected int numAPs;
	/** Number of variables (num model vars + 1) */
	protected int numVars; 
	/** Variable names */
	protected List<String> varNames;
	/** Variable types */
	protected List<Type> varTypes;
	/** Name for new acceptance label */
	protected String accLabel;
	/** Label names */
	protected List<String> labelNames;
	
	/** BitSet */
	protected BitSet bsLabels;
	
	/** State to be explored in product */
	protected State exploreState;
	/** The model part of exploreState */
	protected State exploreModelState;
	/** The DA part of exploreState */
	protected int exploreDaState;
	
	// Constructor(s)
	
	public ProductModelGenerator(ModelGenerator modelGen, DA<BitSet, ? extends AcceptanceOmega> da, List<Expression> labelExprs)
	{
		this.modelGen = modelGen;
		this.da = da;
		this.labelExprs = labelExprs;
		// Create a (new, unique) name for the variable that will represent DA states
		int daNumber = 0;
		daVar = "_da" + daNumber;
		while (modelGen.getVarIndex(daVar) != -1) {
			daNumber = daNumber + 1;
			daVar = "_da" + daNumber;
		}
		// Store additional info
		numAPs = da.getAPList().size();
		numVars = modelGen.getNumVars() + 1;
		varNames = new ArrayList<>();
		varNames.addAll(modelGen.getVarNames());
		varNames.add(daVar);
		varTypes = new ArrayList<>();
		varTypes.addAll(modelGen.getVarTypes());
		varTypes.add(TypeInt.getInstance());
		accLabel = "_acc";
		labelNames = new ArrayList<String>(modelGen.getLabelNames());
		labelNames.add(accLabel);
		// Temporary storage
		bsLabels = new BitSet(numAPs);
	}

	// Accessors
	
	public String getDAVarName()
	{
		return daVar;
	}
	
	/**
	 * Assuming the product is build with a reach acceptance,
	 * is the state currently being explored a goal state?
	 */
	public boolean isReachAcceptanceGoalState()
	{
		AcceptanceOmega acc = da.getAcceptance();
		if (!(acc instanceof AcceptanceReach)) {
			return false;
		}
		AcceptanceReach accReach = (AcceptanceReach) acc;
		return accReach.getGoalStates().get(exploreDaState);
	}
	
	public boolean isReachAcceptanceGoalState(State state)
	{
		AcceptanceOmega acc = da.getAcceptance();
		if (!(acc instanceof AcceptanceReach)) {
			return false;
		}
		AcceptanceReach accReach = (AcceptanceReach) acc;
		return accReach.getGoalStates().get(((Integer) state.varValues[numVars - 1]).intValue());
	}
	
	// Methods to implement ModelGenerator
	
	@Override
	public ModelType getModelType()
	{
		return modelGen.getModelType();
	}

	@Override
	public void setSomeUndefinedConstants(Values someValues) throws PrismException
	{
		modelGen.setSomeUndefinedConstants(someValues);
	}

	@Override
	public Values getConstantValues()
	{
		return modelGen.getConstantValues();
	}

	@Override
	public boolean containsUnboundedVariables()
	{
		return modelGen.containsUnboundedVariables();
	}

	@Override
	public int getNumVars()
	{
		return modelGen.getNumVars() + 1;
	}

	@Override
	public List<String> getVarNames()
	{
		return varNames;
	}

	@Override
	public List<Type> getVarTypes()
	{
		return varTypes;
	}

	@Override
	public int getVarIndex(String name)
	{
		return varNames.indexOf(name);
	}

	@Override
	public String getVarName(int i)
	{
		return varNames.get(i);
	}

	@Override
	public int getNumLabels()
	{
		return labelNames.size();
	}

	@Override
	public List<String> getLabelNames()
	{
		return labelNames;
	}

	@Override
	public int getLabelIndex(String name)
	{
		return getLabelNames().indexOf(name);
	}

	@Override
	public String getLabelName(int i) throws PrismException
	{
		try {
			return getLabelNames().get(i);
		} catch (IndexOutOfBoundsException e) {
			throw new PrismException("Label number \"" + i + "\" not defined");
		}
	}

	@Override
	public int getNumRewardStructs()
	{
		return modelGen.getNumRewardStructs();
	}

	@Override
	public List<String> getRewardStructNames()
	{
		return modelGen.getRewardStructNames();
	}

	@Override
	public int getRewardStructIndex(String name)
	{
		return modelGen.getRewardStructIndex(name);
	}

	@Override
	public RewardStruct getRewardStruct(int i)
	{
		return modelGen.getRewardStruct(i);
	}

	@Override
	public boolean rewardStructHasTransitionRewards(int i)
	{
		return modelGen.rewardStructHasTransitionRewards(i);
	}
	
	@Override
	public VarList createVarList() throws PrismException
	{
		VarList varListModel = modelGen.createVarList();
		VarList varList = (VarList) varListModel.clone();
		// NB: if DA only has one state, we add an extra dummy state
		Declaration decl = new Declaration(daVar, new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(da.size() - 1, 1))));
		try {
			varList.addVar(decl, 1, null);
		} catch (PrismLangException e) {
			// Shouldn't happen
			return null;
		}
		return varList;
	}
	
	@Override
	public boolean hasSingleInitialState() throws PrismException
	{
		return modelGen.hasSingleInitialState();
	}

	@Override
	public List<State> getInitialStates() throws PrismException
	{
		List<State> initStates = new ArrayList<>();
		for (State sInit : modelGen.getInitialStates()) {
			initStates.add(new State(sInit, new State(1).setValue(0, getDASuccessor(da.getStartState(), sInit))));
		}
		return initStates;
	}

	@Override
	public State getInitialState() throws PrismException
	{
		State sInit = modelGen.getInitialState();
		return new State(sInit, new State(1).setValue(0, getDASuccessor(da.getStartState(), sInit)));
	}

	@Override
	public void exploreState(State exploreState) throws PrismException
	{
		this.exploreState = exploreState;
		exploreModelState = exploreState.substate(0, numVars - 1);
		modelGen.exploreState(exploreModelState);
		exploreDaState = ((Integer) exploreState.varValues[numVars - 1]).intValue();
	}

	@Override
	public State getExploreState()
	{
		return exploreState;
	}

	@Override
	public int getNumChoices() throws PrismException
	{
		return modelGen.getNumChoices();
	}

	@Override
	public int getNumTransitions() throws PrismException
	{
		return modelGen.getNumTransitions();
	}

	@Override
	public int getNumTransitions(int i) throws PrismException
	{
		return modelGen.getNumTransitions(i);
	}

	@Override
	public Object getTransitionAction(int i) throws PrismException
	{
		return modelGen.getTransitionAction(i);
	}

	@Override
	public Object getTransitionAction(int i, int offset) throws PrismException
	{
		return modelGen.getTransitionAction(i, offset);
	}

	@Override
	public Object getChoiceAction(int i) throws PrismException
	{
		return modelGen.getChoiceAction(i);
	}

	@Override
	public double getTransitionProbability(int i, int offset) throws PrismException
	{
		return modelGen.getTransitionProbability(i, offset);
	}

	@Override
	public State computeTransitionTarget(int i, int offset) throws PrismException
	{
		State sTarget = modelGen.computeTransitionTarget(i, offset);
		return new State(sTarget, new State(1).setValue(0, getDASuccessor(exploreDaState, sTarget)));
	}

	@Override
	public boolean isLabelTrue(String label) throws PrismException
	{	System.out.println("ACC");
		if (accLabel.equalsIgnoreCase(label)) {
			return isReachAcceptanceGoalState(); //TODO non acceptance
		} else {
			return modelGen.isLabelTrue(label); 
		}
	}

	@Override
	public boolean isLabelTrue(int i) throws PrismException
	{
		if (i == modelGen.getNumLabels()) {
			return isReachAcceptanceGoalState(); //TODO non acceptance
		} else {
			return modelGen.isLabelTrue(i); 
		}
	}

	@Override
	public double getStateReward(int r, State state) throws PrismException
	{
		if (isReachAcceptanceGoalState(state)) {
			return 0.0;
		} else {
			return modelGen.getStateReward(r, state.substate(0, numVars - 1));
		}
	}

	@Override
	public double getStateActionReward(int r, State state, Object action) throws PrismException
	{
		if (isReachAcceptanceGoalState(state)) {
			return 0.0;
		} else {
			return modelGen.getStateActionReward(r, state.substate(0, numVars - 1), action);
		}
	}

	// Utility methods
	
	/**
	 * Find the successor of state {@code q} in the DA, taking the edge whose labelling matches the state {@code s}.
	 */
	protected int getDASuccessor(int q, State s) throws PrismException
	{
		// Create BitSet representing APs (labels) satisfied by state s
		for (int k = 0; k < numAPs; k++) {
			bsLabels.set(k, labelExprs.get(Integer.parseInt(da.getAPList().get(k).substring(1))).evaluateBoolean(s));
		}
		// Find/return successor
		return da.getEdgeDestByLabel(q, bsLabels);
	}
	
	public double getProgressionRew(State source, State target) {
		int daSource = (int)source.varValues[numVars - 1];
		int daTarget = (int)target.varValues[numVars - 1];
		double prog = 100*(da.getDistsToAcc().get(daSource) - da.getDistsToAcc().get(daTarget));
		//if (prog < 0.0) 	System.out.println(prog);
		double res = da.getDistsToAcc().get(daTarget);
		if (res < 0.3) {
			System.out.println(res);
		}
		//return Math.max(prog, 0);
		//return prog;
		return res;
	}
}
