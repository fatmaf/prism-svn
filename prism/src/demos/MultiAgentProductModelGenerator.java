package demos;
/*
 * generates a joint product model 
 * from multiple single product models
 * for now it takes as input an array of single agent loaders
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;

import acceptance.AcceptanceOmega;
import automata.DA;
import explicit.MDPSimple;
import parser.State;
import prism.PrismException;
import prism.PrismLog;
import prism.ProductModelGenerator;

public class MultiAgentProductModelGenerator
{

	ArrayList<ProductModelGenerator> prodModGens;
	ArrayList<SingleAgentLoader> singleAgentLoaders;
	MDPSimple jointMDP;
	int numAgents;
	DA<BitSet, ? extends AcceptanceOmega> da;
	PrismLog mainLog;
	boolean printMessages;
	boolean hasSharedStates;

	private int stateSize; 
	private int sharedStateInd; 
	private int daStateInd; 
	
	public MultiAgentProductModelGenerator(PrismLog mainLog, ArrayList<SingleAgentLoader> sals, DA<BitSet, ? extends AcceptanceOmega> da)
	{
		printMessages = true;
		hasSharedStates = false;
		this.mainLog = mainLog;
		String message = "";
		if (printMessages)
			message = "Initialising Joint PMG\n";
		if (sals != null) {
			singleAgentLoaders = sals;
			numAgents = sals.size();
			if (printMessages)
				message += "Single Agent Loaders Added";
			prodModGens = new ArrayList<ProductModelGenerator>();
			for (int i = 0; i < singleAgentLoaders.size(); i++) {
				prodModGens.add(singleAgentLoaders.get(i).prodModelGen);
				hasSharedStates = hasSharedStates | singleAgentLoaders.get(i).hasSharedStates();

			}
			if (printMessages)
				message += "\nProduct Model Gens loaded from SALs";
		}
		this.da = da;
		message += "\nDA saved";

		//getting state sizes etc 
		 stateSize = numAgents + 1; //numagents + da 
		sharedStateInd = -1;
		if (this.hasSharedStates) {
			sharedStateInd = numAgents;
			stateSize += 1;

		}
		daStateInd = stateSize - 1;
		
		if (printMessages)
			mainLog.println(message);
	}

	public void setPrintMessagesOn()
	{
		this.printMessages = true;
	}

	public void setPrintMessagesOff()
	{
		this.printMessages = false;
	}

	State processDAStates(ArrayList<State> robotStates) throws PrismException
	{
		BitSet stateLabels = new BitSet();
		BitSet robotStateLabels;
		int jsDAState = -1;
		double currentDistToAcc = 0.0;
		for (int i = 0; i < numAgents; i++) {
			SingleAgentLoader sal = singleAgentLoaders.get(i);
			State robotState = robotStates.get(i);

			robotStateLabels = sal.getStateLabels(robotState);
			stateLabels.or(robotStateLabels);

			int daState = sal.getDAStateAsInt(robotState);
			if (jsDAState == -1) {
				jsDAState = daState;
				currentDistToAcc = sal.prodModelGen.getDaDistanceCost(robotState);
			} else {
				double robotDistToAcc = sal.prodModelGen.getDaDistanceCost(robotState);
				if (currentDistToAcc > robotDistToAcc) {
					jsDAState = daState;
					currentDistToAcc = robotDistToAcc;
				}
			}
		}

		//update as per state labels 
		jsDAState = da.getEdgeDestByLabel(jsDAState, stateLabels);

		State jsDAS = new State(1);
		jsDAS.setValue(0, jsDAState);

		return jsDAS;
	}

	State processSharedStates(ArrayList<State> sharedStates)
	{
		State referenceState = sharedStates.get(0);
		Object[] rSV = referenceState.varValues;
		State currentState = new State(referenceState);
		Object[] cSV = currentState.varValues;

		for (int i = 1; i < sharedStates.size(); i++) {
			State ssR = sharedStates.get(i);
			Object[] ssRV = ssR.varValues;
			for (int j = 0; j < ssRV.length; j++) {
				if ((int) cSV[j] != (int) ssRV[j]) {
					if ((int) rSV[j] != (int) ssRV[j]) {
						cSV[j] = ssRV[j];
					}
				}
			}
		}
		for (int j = 0; j < cSV.length; j++) {
			currentState.setValue(j, cSV[j]);
		}
		return currentState;
	}

	State createJointState(ArrayList<State> robotStates) throws PrismException
	{

		int stateSize = numAgents + 1; //numagents + da 
		int sharedStateInd = -1;
		if (this.hasSharedStates) {
			sharedStateInd = numAgents;
			stateSize += 1;

		}
		int daStateInd = stateSize - 1;

		SingleAgentLoader sal;

		ArrayList<State> sharedStates = null;
		if (hasSharedStates) {
			sharedStates = new ArrayList<State>();
		}
		State js = new State(stateSize);

		//for each robot state get the private state 
		for (int i = 0; i < numAgents; i++) {
			sal = singleAgentLoaders.get(i);
			State robotState = robotStates.get(i);
			State privateState = sal.getPrivateState(robotState);
			js.setValue(i, privateState);

			if (hasSharedStates) {
				State sharedState = sal.getSharedState(robotState);
				sharedStates.add(sharedState);
			}

		}
		State jsDAState = processDAStates(robotStates);
		js.setValue(daStateInd, jsDAState);
		if (hasSharedStates) {
			State sharedState = processSharedStates(robotStates);
			js.setValue(sharedStateInd, sharedState);
		}

		return js;

	}
	
	Object createJointActionFromRobotActions(ArrayList<Object> actions)
	{
		String diff = ","; 
		String ja = ""; 
		for(int i = 0; i<actions.size()-1; i++)
		{
			ja+=actions.get(i).toString()+diff; 
		}
		ja+=actions.get(actions.size()-1);
		return ja; 
	}
	ArrayList<Object> getRobotActionsFromJointAction(Object ja)
	{
		//split the action at the diff 
		String diff = ","; 
		String jaS = ja.toString(); 
		String[] actions= jaS.split(diff);
		
		ArrayList<Object> robotActions = new ArrayList<Object>(Arrays.asList(actions));
		return robotActions;
	}
	State getDAState(State js)
	{
		Object[] jsV = js.varValues; 
		State daState = (State)jsV[daStateInd]; 
		return daState; 
	}
	State getSSState(State js)
	{
		Object[] jsV = js.varValues; 
		State ss = (State)jsV[sharedStateInd]; 
		return ss; 
	}
	State getRobotState(State js, int rnum)
	{
		Object[] jsV = js.varValues; 
		State rs = (State)jsV[rnum]; 
		return rs; 
	}
	ArrayList<State> getRobotStatesFromJointState(State js) throws PrismException
	{
		ArrayList<State> robotStates = new ArrayList<State>(); 
		//get the da state 
		//get the ss state 
		State daState = getDAState(js); 
		State ssState = null; 
		if(hasSharedStates)
			ssState = getSSState(js);
		for(int i = 0; i<numAgents; i++)
		{
			State ps = getRobotState(js,i);
			State robotState = singleAgentLoaders.get(i).createRobotState(ps, ssState, daState);
			robotStates.add(robotState);
		}
		return robotStates; 
	}
	
	ArrayList<Entry<State,Double>> getSuccessors(State js, Object ja) throws PrismException
	{
		ArrayList<Entry<State,Double>> successors = new ArrayList<Entry<State,Double>>(); 
		ArrayList<ArrayList<Entry<State,Double>>> robotSuccessors = getRobotSuccessors(js,ja); 
		ArrayList<Integer> numRobotSuccessors = countRobotSuccessors(robotSuccessors ); 
		ArrayList<int[]> successorCombinations = generateCombinations(numRobotSuccessors); 

		
		//for each robot we have to get its sucessors 
		//then we have to make combinations 
		//then we have to move on 
		
		//now we make successors ? 
		for(int i = 0; i<successorCombinations.size(); i++)
		{
			//step 1 get the robot states you need 
			//robotStates 
			ArrayList<State> robotStates = new ArrayList<State>(); 
			int[] currentCombination = successorCombinations.get(i);
			double stateProbability = 1.0; 
			for(int r = 0; r<currentCombination.length; r++)
			{
				Entry<State, Double> robotStateProb = robotSuccessors.get(r).get(currentCombination[r]); 
				State robotState = robotStateProb.getKey(); 
				robotStates.add(robotState); 
				
				double prob = robotStateProb.getValue(); 
				stateProbability *= prob; 
				
			}
			State successorJS = createJointState(robotStates); 
			successors.add(new AbstractMap.SimpleEntry<State,Double>(successorJS,stateProbability));
		}
		
		
		return successors; 
	}
	
	ArrayList<Integer> countRobotSuccessors(ArrayList<ArrayList<Entry<State,Double>>> rs)
	{
		ArrayList<Integer> numRobotSuccessors = new ArrayList<Integer>(); 
		for(int i = 0; i<rs.size(); i++)
		{
			numRobotSuccessors.add(rs.get(i).size());
		}
		return numRobotSuccessors; 
	}
	ArrayList<ArrayList<Entry<State,Double>>> getRobotSuccessors(State js, Object ja) throws PrismException
	{
		ArrayList<ArrayList<Entry<State,Double>>> toret = new ArrayList<ArrayList<Entry<State,Double>>>(); 
		ArrayList<State> robotStates = getRobotStatesFromJointState(js); 
		ArrayList<Object> robotActions = getRobotActionsFromJointAction(ja);
		for(int i = 0; i<numAgents; i++)
		{
			ArrayList<Entry<State,Double>> robotSuccessors = 
					singleAgentLoaders.get(i).getStateActionSuccessors(robotStates.get(i), robotActions.get(i));
		toret.add(robotSuccessors); 
		}
		return toret; 
	}
	  ArrayList<int[]>  generateCombinations(ArrayList<Integer> numActionsPerRobot) throws PrismException
	{
		 ArrayList<int[]> res = new ArrayList<int[]>();
		int[] counter = new int[numActionsPerRobot.size()];
		for (int i = 0; i < counter.length; i++)
			counter[i] = numActionsPerRobot.get(i);
		int[] original = counter.clone();
		int numP = generateCombinations(counter, 0, original.length - 1, original, 0, res);
		int estimatedC = getNumberOfCombinations(original);
		if (res.size() != estimatedC) {
			mainLog.println("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
			throw new PrismException("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
		}
		return res; 
	}

	protected void generateCombinations(int counter[], int original[], ArrayList<int[]> res) throws PrismException
	{
		int numP = generateCombinations(counter, 0, original.length - 1, original, 0, res);
		int estimatedC = getNumberOfCombinations(original);
		if (res.size() != estimatedC) {
			mainLog.println("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
			throw new PrismException("ERROR - the number of expected combinations was " + estimatedC + ", generated " + res.size());
		}
	}

	int getNumberOfCombinations(int[] arr)
	{
		int num = 1;
		for (int i = 0; i < arr.length; i++)
			num *= arr[i];
		return num;
	}

	int generateCombinations(int[] arr, int start, int end, int[] orig, int numC, ArrayList<int[]> res)
	{
		if (start == end) {
			while (arr[start] != 0) {

				res.add(arr.clone());
				arr[start]--;
				numC++;
			}
			arr[start] = orig[start];
		} else {
			while (arr[start] != 0) {
				numC = generateCombinations(arr, start + 1, end, orig, numC, res);
				arr[start]--;
			}
			arr[start] = orig[start];
		}
		return numC;
	}


}
