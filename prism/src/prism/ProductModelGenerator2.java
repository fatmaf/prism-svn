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
import strat.MDStrategy;

public class ProductModelGenerator2 implements ModelGenerator
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
	
	protected StateVector accProbs;
	protected NondetModel prodModel;
	
	protected BitSet activeActionsBS;	
	protected List<Object> activeActions;
	// Constructor(s)
	
	public ProductModelGenerator2(ModelGenerator modelGen, DA<BitSet, ? extends AcceptanceOmega> da, List<Expression> labelExprs, StateVector accProbs, NondetModel prodModel)
	{
		this.modelGen = modelGen;
		this.da = da;
		this.labelExprs = labelExprs;
		// Create a (new, unique) name for the variable that will represent DA states
		daVar = "_da";
		while (modelGen.getVarIndex(daVar) != -1) {
			daVar = "_" + daVar;
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
		
		this.prodModel = prodModel;
		this.accProbs = accProbs;
		
		activeActionsBS = new BitSet(prodModel.getSynchs().size());
		activeActions = new ArrayList<Object>();
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
		activeActionsBS.clear();
		activeActions.clear();
		
		
		StateList states =  prodModel.getReachableStates();
		Values values = states.getFirstAsValues();
		List<String> stringStates = states.exportToStringList();
		int currentIndex = prodModel.getReachableStates().getIndexOfState(exploreState);
		StateListMTBDD test = (StateListMTBDD)prodModel.getStartStates();
		Values pila = test.getFirstAsValues();
		String ddd = pila.getName(3);
		int sss=prodModel.getVarIndex(pila.getName(3));
		System.out.println(test.getClass());
		
		
		if (!Double.isFinite((Double)accProbs.getValue(currentIndex))) {
			System.out.println("FODASE");
		}
		
		for (int i = 0; i < modelGen.getNumChoices(); i++) {
			boolean actionActive = true;
			for (int j = 0; j < modelGen.getNumTransitions(i); j++) {
				State target = computeTransitionTarget2(i, j);
				//System.out.println(target);
				//System.out.println(prodModel.getVarList().getName(11));
				int targetIndex = prodModel.getReachableStates().getIndexOfState(target);
				Double currentProb = (Double)accProbs.getValue(targetIndex);
				if (isReachAcceptanceGoalState(exploreState) || !Double.isFinite((Double)accProbs.getValue(targetIndex))) {
				//if (isReachAcceptanceGoalState(exploreState) || (Double)accProbs.getValue(targetIndex) < 0.99) {
					//System.out.println((Double)accProbs.getValue(targetIndex));
					actionActive = false;
					break;
				} else {
						if(((Double)accProbs.getValue(targetIndex)).equals(new Double("3.1883E-319"))) {
							System.out.println((Double)accProbs.getValue(targetIndex)); 
						}
						
					
				}
			}
			if(actionActive) {
				Object action = modelGen.getChoiceAction(i);
				System.out.println(action);
				activeActions.add(action);
				activeActionsBS.set(i);
			} else {
				Object action = modelGen.getChoiceAction(i);
				//System.out.println(action);
			}
		}
				
	}
	
	
	
	
	
	

	@Override
	public State getExploreState()
	{
		return exploreState;
	}


	
	@Override
	public int getNumChoices() throws PrismException
	{
		return activeActionsBS.cardinality();
	}

	@Override
	public int getNumTransitions() throws PrismException
	{		

		int res = 0;
		for (int i = 0; i < getNumChoices(); i++) {
			res =  res + getNumTransitions(i);
		}
		return res;
	}
	

	@Override
	public int getNumTransitions(int i) throws PrismException
	{
		int originalIndex = getOriginalIndex(i);
		return modelGen.getNumTransitions(originalIndex);
	}

	@Override
	public Object getTransitionAction(int i) throws PrismException
	{
		int originalIndex = getOriginalIndex(i);
		return modelGen.getTransitionAction(originalIndex);
	}

	@Override
	public Object getTransitionAction(int i, int offset) throws PrismException
	{
		int originalIndex = getOriginalIndex(i);
		return modelGen.getTransitionAction(originalIndex, offset);
	}

	@Override
	public Object getChoiceAction(int i) throws PrismException
	{
		int originalIndex = getOriginalIndex(i);
		return modelGen.getChoiceAction(originalIndex);
	}

	@Override
	public double getTransitionProbability(int i, int offset) throws PrismException
	{
		int originalIndex = getOriginalIndex(i);
		return modelGen.getTransitionProbability(originalIndex, offset);
	}

	public State computeTransitionTarget2(int i, int offset) throws PrismException
	{
		State sTarget = modelGen.computeTransitionTarget(i, offset);
		return new State(sTarget, new State(1).setValue(0, getDASuccessor(exploreDaState, sTarget)));
	}
	
	@Override
	public State computeTransitionTarget(int i, int offset) throws PrismException
	{
		int originalIndex = getOriginalIndex(i);
		State sTarget = modelGen.computeTransitionTarget(originalIndex, offset);
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
		if (isReachAcceptanceGoalState(state) || !Double.isFinite((Double)accProbs.getValue(prodModel.getReachableStates().getIndexOfState(state)))) {
			return 0.0;
		} else {
			return modelGen.getStateReward(r, state.substate(0, numVars - 1));
		}
	}

	@Override
	public double getStateActionReward(int r, State state, Object action) throws PrismException
	{
/*		System.out.println("AHAH");
		System.out.println(state);
		System.out.println(prodModel.getReachableStates().size());
		try {
			Thread.sleep(1);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}*/
		if (isReachAcceptanceGoalState(state) || !Double.isFinite((Double)accProbs.getValue(prodModel.getReachableStates().getIndexOfState(state)))) {
			System.out.println("ACC");
			return 0.0;
		} else {
			return modelGen.getStateActionReward(r, state.substate(0, numVars - 1), action);
		}
	}
	

	
	protected int getOriginalIndex(int i)
	{
		int originalIndex = activeActionsBS.nextSetBit(0);
		for (int j=0; j < i; j++) {
			originalIndex = activeActionsBS.nextSetBit(originalIndex + 1);
		}
		return originalIndex;
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
		return da.getEdgeDestByLabel(da.getStartState(), bsLabels);
	}
}
