package demos;
/*
 * generates a joint product model 
 * from multiple single product models
 * for now it takes as input an array of single agent loaders
 */

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.AbstractMap;
import java.util.Map.Entry;
import java.util.Set;

import acceptance.AcceptanceOmega;
import automata.DA;
import explicit.MDPSimple;
import param.Function;
import parser.State;
import prism.PrismException;
import prism.PrismLog;
import prism.ProductModelGenerator;

public class MultiAgentProductModelGenerator
{
	int MAXTRIALLEN = 10000;
	int MAXROLLOUTS = 10000;

	ArrayList<ProductModelGenerator> prodModGens;
	ArrayList<SingleAgentLoader> singleAgentLoaders;
	MDPCreator mdpCreator;
	int numAgents;
	DA<BitSet, ? extends AcceptanceOmega> da;
	PrismLog mainLog;
	boolean printMessages;
	boolean hasSharedStates;
	boolean buildMDP;

	private int stateSize;
	private int sharedStateInd;
	private int daStateInd;
	//just so I dont have to do this over and over 
	//TODO: use this later if things are slow 
	private HashMap<State, ArrayList<State>> jointStateToRobotStates;

	public enum RewardCalculation {
		SUM, MAX, MIN
	}

	public MultiAgentProductModelGenerator(PrismLog mainLog, ArrayList<SingleAgentLoader> sals, DA<BitSet, ? extends AcceptanceOmega> da, boolean buildMDP)
	{
		printMessages = true;
		hasSharedStates = false;
		this.buildMDP = buildMDP;
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

		if (buildMDP)
			mdpCreator = new MDPCreator(mainLog);
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

	public ArrayList<State> createInitialStateFromRobotInitStates() throws PrismException
	{
		//get the init states of all the robots 
		ArrayList<List<State>> initStates = new ArrayList<List<State>>();
		ArrayList<Integer> numStates = new ArrayList<Integer>();
		ArrayList<State> jointInitStates = new ArrayList<State>();
		for (int i = 0; i < numAgents; i++) {
			List<State> initStatesForRobot = getAgent(i).prodModelGen.getInitialStates();
			initStates.add(initStatesForRobot);
			numStates.add(initStatesForRobot.size());
		}
		ArrayList<int[]> combinations = generateCombinations(numStates);
		//create combinations 

		for (int i = 0; i < combinations.size(); i++) {

			ArrayList<State> robotStates = new ArrayList<State>();
			int[] currentCombination = combinations.get(i);

			for (int r = 0; r < currentCombination.length; r++) {
				State robotState = initStates.get(r).get(currentCombination[r] - 1);

				robotStates.add(robotState);

			}
			State initJS = createJointState(null, robotStates);
			if (buildMDP) {
				mdpCreator.setInitialState(initJS);
			}
			jointInitStates.add(initJS);
		}
		if (printMessages) {
			mainLog.println("Initial States: ");
			for (int i = 0; i < jointInitStates.size(); i++) {
				mainLog.println(jointInitStates.get(i).toString());
			}
		}
		return jointInitStates;
	}

	//cuz i cant be bothered to type the whole thing
	//get a single agent loader
	SingleAgentLoader getAgent(int i)
	{
		return singleAgentLoaders.get(i);
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

	State processSharedStates(State pSS, ArrayList<State> sharedStates)
	{
		State referenceState;
		if (pSS == null)
			referenceState = sharedStates.get(0);
		else
			referenceState = pSS;
		Object[] rSV = referenceState.varValues;
		State currentState = new State(referenceState);
		Object[] cSV = currentState.varValues;

		for (int i = 0; i < sharedStates.size(); i++) {
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

	State createJointState(State ps, ArrayList<State> robotStates) throws PrismException
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
			State pSS = null;
			if (ps != null)
				pSS = getSSState(ps);
			State sharedState = processSharedStates(pSS, sharedStates);
			js.setValue(sharedStateInd, sharedState);
		}

		return js;

	}

	Object createJointActionFromRobotActions(ArrayList<Object> actions)
	{
		String diff = ",";
		String ja = "";
		for (int i = 0; i < actions.size() - 1; i++) {
			ja += actions.get(i).toString() + diff;
		}
		ja += actions.get(actions.size() - 1);
		return ja;
	}

	ArrayList<Object> getRobotActionsFromJointAction(Object ja)
	{
		//split the action at the diff 
		String diff = ",";
		String jaS = ja.toString();
		String[] actions = jaS.split(diff);

		ArrayList<Object> robotActions = new ArrayList<Object>(Arrays.asList(actions));
		return robotActions;
	}

	int getDAStateAsInt(State js)
	{
		Object[] jsV = js.varValues;
		State daState = (State) jsV[daStateInd];
		jsV = daState.varValues;
		int daStateInt = (int) jsV[0];
		return daStateInt;
	}

	State getDAState(State js)
	{
		Object[] jsV = js.varValues;
		State daState = (State) jsV[daStateInd];
		return daState;
	}

	State getSSState(State js)
	{
		Object[] jsV = js.varValues;
		State ss = (State) jsV[sharedStateInd];
		return ss;
	}

	State getRobotState(State js, int rnum)
	{
		Object[] jsV = js.varValues;
		State rs = (State) jsV[rnum];
		return rs;
	}

	ArrayList<State> getRobotStatesFromJointState(State js) throws PrismException
	{
		ArrayList<State> robotStates = new ArrayList<State>();
		//get the da state 
		//get the ss state 
		State daState = getDAState(js);
		State ssState = null;
		if (hasSharedStates)
			ssState = getSSState(js);
		for (int i = 0; i < numAgents; i++) {
			State ps = getRobotState(js, i);
			State robotState = singleAgentLoaders.get(i).createRobotState(ps, ssState, daState);
			robotStates.add(robotState);
		}
		return robotStates;
	}

	ArrayList<Object> getJointActions(State js) throws PrismException
	{
		//get all the joint actions for this state 
		ArrayList<State> robotStates = getRobotStatesFromJointState(js);

		//for each robot State 
		//get all the actions 
		ArrayList<HashMap<Object, Integer>> robotsActions = new ArrayList<HashMap<Object, Integer>>();
		ArrayList<ArrayList<Object>> allRobotActions = new ArrayList<ArrayList<Object>>();
		ArrayList<Integer> numRobotActions = new ArrayList<Integer>();
		for (int i = 0; i < numAgents; i++) {
			State robotState = robotStates.get(i);
			HashMap<Object, Integer> robotActions = getAgent(i).getActionsForState(robotState);
			int size = robotActions.size();

			robotsActions.add(robotActions);

			numRobotActions.add(size);
			ArrayList<Object> thisRobotsActions = new ArrayList<Object>();
			thisRobotsActions.addAll(robotActions.keySet());
			allRobotActions.add(thisRobotsActions);
		}
		//now we get all the combinations 
		ArrayList<Object> jointActions = new ArrayList<Object>();
		ArrayList<int[]> jointActionsCombinations = generateCombinations(numRobotActions);
		for (int i = 0; i < jointActionsCombinations.size(); i++) {
			ArrayList<Object> actionsForThisCombination = new ArrayList<Object>();
			int[] currentCombination = jointActionsCombinations.get(i);
			for (int r = 0; r < currentCombination.length; r++) {
				actionsForThisCombination.add(allRobotActions.get(r).get(currentCombination[r] - 1));
			}
			Object ja = createJointActionFromRobotActions(actionsForThisCombination);
			jointActions.add(ja);

		}
		return jointActions;
	}

	ArrayList<Entry<State, Double>> getSuccessors(State js, Object ja) throws PrismException
	{
		ArrayList<Entry<State, Double>> successors = new ArrayList<Entry<State, Double>>();
		ArrayList<ArrayList<Entry<State, Double>>> robotSuccessors = getRobotSuccessors(js, ja);
		ArrayList<Integer> numRobotSuccessors = countRobotSuccessors(robotSuccessors);
		ArrayList<int[]> successorCombinations = generateCombinations(numRobotSuccessors);

		//for each robot we have to get its successors 
		//then we have to make combinations 
		//then we have to move on 

		//now we make successors ? 
		ArrayList<State> successorJSs = new ArrayList<State>();
		for (int i = 0; i < successorCombinations.size(); i++) {
			//step 1 get the robot states you need 
			//robotStates 
			ArrayList<State> robotStates = new ArrayList<State>();
			int[] currentCombination = successorCombinations.get(i);
			double stateProbability = 1.0;
			for (int r = 0; r < currentCombination.length; r++) {
				Entry<State, Double> robotStateProb = robotSuccessors.get(r).get(currentCombination[r] - 1);
				State robotState = robotStateProb.getKey();
				robotStates.add(robotState);

				double prob = robotStateProb.getValue();
				stateProbability *= prob;

			}
			State successorJS = createJointState(js, robotStates);
			//sometimes we need to merge successors 
			//basically recalculate the probabilities and merge states 
			if(successorJSs.contains(successorJS))
			{
				int successorToModifyIndex = -1; 
				//find that state in the successors list 
				for(int s = 0; s<successors.size(); s++)
				{
					if(successors.get(s).getKey().compareTo(successorJS)==0)
					{
						successorToModifyIndex = s; 
						break; 
					}
				}
				if(successorToModifyIndex != -1)
				{
					
		//just sum the probabilities really 
					Entry<State, Double> toupdate = successors.get(successorToModifyIndex);
					double previousProb = toupdate.getValue(); 
					double updatedProb = stateProbability + previousProb; 
					toupdate.setValue(updatedProb); 
					successors.set(successorToModifyIndex, toupdate);
				}
			}
			else {
			successorJSs.add(successorJS);
			successors.add(new AbstractMap.SimpleEntry<State, Double>(successorJS, stateProbability));
			}
		}
	
		
		if (buildMDP) {
			mdpCreator.addAction(js, ja, successors);
		}
		return successors;
	}

	double getDADistanceCost(State s)
	{
		int daState = getDAStateAsInt(s);
		return da.getDistsToAcc().get(daState);
	}

	double getProgressionReward(State s, Object a) throws PrismException
	{
		ArrayList<Entry<State, Double>> successors = getSuccessors(s, a);
		double currentStateDistance = getDADistanceCost(s);
		double rew = 0;
		double successorStateDistance;

		for (Entry<State, Double> sa : successors) {
			successorStateDistance = getDADistanceCost(sa.getKey());
			rew += sa.getValue() * (Math.max(currentStateDistance - successorStateDistance, 0.0));
		}
		return rew;

	}

	double getSolutionValue(State s, Objectives obj, RewardCalculation calcMethod) throws PrismException
	{
		double reward;
		ArrayList<State> robotStates = getRobotStatesFromJointState(s);

		switch (calcMethod) {
		case SUM:
			reward = getSolutionValueSum(robotStates, obj);
			break;
		case MAX:
			reward = getSolutionValueMax(robotStates, obj);
			break;
		case MIN:
			reward = getSolutionValueMin(robotStates, obj);
			break;
		default:
			throw new PrismException("Calculation Method not implemented");

		}

		return reward;
	}

	private double getSolutionValueMin(ArrayList<State> robotStates, Objectives obj) throws PrismException
	{
		State robotState = robotStates.get(0);
		double val = getAgent(0).getSolutionValue(robotState, obj);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			double rval = getAgent(i).getSolutionValue(robotState, obj);
			if (rval < val)
				val = rval;
		}
		// TODO Auto-generated method stub
		return val;
	}

	private double getSolutionValueMax(ArrayList<State> robotStates, Objectives obj) throws PrismException
	{
		State robotState = robotStates.get(0);
		double val = getAgent(0).getSolutionValue(robotState, obj);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			double rval = getAgent(i).getSolutionValue(robotState, obj);
			if (rval > val)
				val = rval;
		}
		// TODO Auto-generated method stub
		return val;
	}

	private double getSolutionValueSum(ArrayList<State> robotStates, Objectives obj) throws PrismException
	{
		State robotState = robotStates.get(0);
		double val = getAgent(0).getSolutionValue(robotState, obj);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			double rval = getAgent(i).getSolutionValue(robotState, obj);
			val += rval;
		}
		// TODO Auto-generated method stub
		return val;
	}

	//	double getProgressionReward(State s, Object a)
	//	{
	//		double rew = 0.0;
	//		double currentStateDistance = this.getDADistanceCost(s);
	//		ArrayList<Entry<State, Double>> succs = getSuccessors(s, a);
	//		if (succs != null) {
	//			for (Entry<State, Double> succ : succs) {
	//				double succStateDistance = getDADistanceCost(succ.getKey());
	//				rew += succ.getValue() * Math.max(currentStateDistance - succStateDistance, 0.0);
	//			}
	//		}
	//		return rew;
	//	}

	double getStateReward(State s, String rew, RewardCalculation calculationMethod) throws PrismException
	{
		double reward;
		ArrayList<State> robotStates = getRobotStatesFromJointState(s);

		switch (calculationMethod) {
		case SUM:
			reward = getStateRewardSum(robotStates, rew);
			break;
		case MAX:
			reward = getStateRewardMax(robotStates, rew);
			break;
		case MIN:
			reward = getStateRewardMin(robotStates, rew);
			break;
		default:
			throw new PrismException("Calculation Method not implemented");

		}

		return reward;

	}

	double getStateRewardSum(ArrayList<State> robotStates, String rew) throws PrismException
	{
		State robotState = robotStates.get(0);
		double reward = getAgent(0).getStateReward(robotState, rew);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			reward += getAgent(i).getStateReward(robotState, rew);
		}
		return reward;
	}

	double getStateRewardMax(ArrayList<State> robotStates, String rew) throws PrismException
	{
		State robotState = robotStates.get(0);
		double reward = getAgent(0).getStateReward(robotState, rew);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			double agentRew = getAgent(i).getStateReward(robotState, rew);
			if (agentRew > reward)
				reward = agentRew;
		}
		return reward;
	}

	double getStateRewardMin(ArrayList<State> robotStates, String rew) throws PrismException
	{
		State robotState = robotStates.get(0);
		double reward = getAgent(0).getStateReward(robotState, rew);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			double agentRew = getAgent(i).getStateReward(robotState, rew);
			if (agentRew < reward)
				reward = agentRew;
		}
		return reward;
	}

	double getStateActionReward(State s, Object a, String rew, RewardCalculation calculationMethod) throws PrismException
	{
		double reward;
		ArrayList<State> robotStates = getRobotStatesFromJointState(s);
		ArrayList<Object> robotActions = getRobotActionsFromJointAction(a);

		switch (calculationMethod) {
		case SUM:
			reward = getStateActionRewardSum(robotStates, robotActions, rew);
			break;
		case MAX:
			reward = getStateActionRewardSum(robotStates, robotActions, rew);
			break;
		case MIN:
			reward = getStateActionRewardSum(robotStates, robotActions, rew);
			break;
		default:
			throw new PrismException("Calculation Method not implemented");

		}

		return reward;

	}

	double getStateActionRewardSum(ArrayList<State> robotStates, ArrayList<Object> robotActions, String rew) throws PrismException
	{
		State robotState = robotStates.get(0);
		Object robotAction = robotActions.get(0);
		double reward = getAgent(0).getStateActionReward(robotState, robotAction, rew);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			robotAction = robotActions.get(i);
			reward += getAgent(i).getStateActionReward(robotState, robotAction, rew);
		}
		return reward;
	}

	double getStateActionRewardMax(ArrayList<State> robotStates, ArrayList<Object> robotActions, String rew) throws PrismException
	{
		State robotState = robotStates.get(0);
		Object robotAction = robotActions.get(0);
		double reward = getAgent(0).getStateActionReward(robotState, robotAction, rew);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			robotAction = robotActions.get(i);
			double agentRew = getAgent(i).getStateActionReward(robotState, robotAction, rew);
			if (agentRew > reward)
				reward = agentRew;
		}
		return reward;
	}

	double getStateActionRewardMin(ArrayList<State> robotStates, ArrayList<Object> robotActions, String rew) throws PrismException
	{
		State robotState = robotStates.get(0);
		Object robotAction = robotActions.get(0);
		double reward = getAgent(0).getStateActionReward(robotState, robotAction, rew);
		for (int i = 1; i < numAgents; i++) {
			robotState = robotStates.get(i);
			robotAction = robotActions.get(i);
			double agentRew = getAgent(i).getStateActionReward(robotState, robotAction, rew);
			if (agentRew < reward)
				reward = agentRew;
		}
		return reward;
	}

	ArrayList<Integer> countRobotSuccessors(ArrayList<ArrayList<Entry<State, Double>>> rs)
	{
		ArrayList<Integer> numRobotSuccessors = new ArrayList<Integer>();
		for (int i = 0; i < rs.size(); i++) {
			numRobotSuccessors.add(rs.get(i).size());
		}
		return numRobotSuccessors;
	}

	ArrayList<ArrayList<Entry<State, Double>>> getRobotSuccessors(State js, Object ja) throws PrismException
	{
		ArrayList<ArrayList<Entry<State, Double>>> toret = new ArrayList<ArrayList<Entry<State, Double>>>();
		ArrayList<State> robotStates = getRobotStatesFromJointState(js);
		ArrayList<Object> robotActions = getRobotActionsFromJointAction(ja);
		for (int i = 0; i < numAgents; i++) {
			ArrayList<Entry<State, Double>> robotSuccessors = singleAgentLoaders.get(i).getStateActionSuccessors(robotStates.get(i), robotActions.get(i));
			toret.add(robotSuccessors);
		}
		return toret;
	}

	ArrayList<int[]> generateCombinations(ArrayList<Integer> numActionsPerRobot) throws PrismException
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

	void generateCombinations(int counter[], int original[], ArrayList<int[]> res) throws PrismException
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

	public MDPSimple getBuiltMDP()
	{
		return mdpCreator.mdp;
	}

	public void saveBuiltMDP(String sl, String fn)
	{
		mdpCreator.saveMDP(sl, fn);
	}

	public boolean isGoal(State s)
	{
		double daDist = this.getDADistanceCost(s);
		return (daDist == 0);
	}

	public boolean isDeadend(State s) throws PrismException
	{
		int numProg0 = 0;
		ArrayList<State> robotStates = this.getRobotStatesFromJointState(s);
		for (int i = 0; i < numAgents; i++) {
			State rs = robotStates.get(i);
			if (getAgent(i).isDeadend(rs))
				numProg0++;
		}
		boolean deadend = (numProg0 == numAgents);
		//I dont really need this but its good for when we dont have like initialised stuff 
		boolean sinkState = da.isSinkState(this.getDAStateAsInt(s));

		if (deadend | sinkState)
			return true;
		else
			return false;
	}

	public int getMaxStatesEstimate()
	{
		//just go over all the sals and the da multiply 
		int maxStates = da.size();
		int numNoEstimate = 0;
		int maxAgentStates = da.size();
		for (int i = 0; i < numAgents; i++) {
			int ms = getAgent(i).getMaxStatesEstimate();
			if (ms != -1) {
				int temp = maxStates *ms; 
				if(temp >= 0)
				maxStates = temp;
				if (maxAgentStates > ms) {
					maxAgentStates = ms;
				}
			} else
				numNoEstimate++;
		}
		if (numNoEstimate > 0) {
			for (int i = 0; i < numNoEstimate; i++) {
				int temp = maxStates * maxAgentStates; 
				if(temp >=0 )
					maxStates = temp; 
//				maxStates *= maxAgentStates;
			}
		}
		// TODO Auto-generated method stub
		maxStates = Math.min(maxStates,MAXTRIALLEN);
		return maxStates;
	}

}
