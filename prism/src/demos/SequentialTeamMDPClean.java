package demos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.Map.Entry;
import java.util.concurrent.TimeUnit;

import explicit.MDPSimple;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.Expression;
import prism.PrismException;

public class SequentialTeamMDPClean
{
	int numRobots = -1;
	ArrayList<AutomatonInformation> internalAutomatonList;// = new ArrayList<AutomatonInformation>(); 
	ArrayList<BitSet> agentsEssentialStates;
	ArrayList<BitSet> agentsInitialStates;
	ArrayList<BitSet> agentsAvoidStates;
	ArrayList<BitSet> agentsAccStates;
	BitSet statesToAvoid;
	MDPCreator teamMDPCreator;
	MDPSimple teamMDPWithSwitches;
	private ArrayList<MDPRewardsSimple> rewardsWithSwitches;
	private ArrayList<BitSet> agentsFailStates; 

	//an on the fly thing really 
	public SequentialTeamMDPClean(SingleAgentNestedProductMDPClean singleAgentNestedP, int numRobots) throws Exception
	{

		this.numRobots = numRobots;//singleAgentNestedPs.size();

		//so from a list of singleAgentNestedProducts 
		//we're going to create the sequential team mdp 
		//which is just a mapping 
		//things we need here are 
		//the switch states 
		//the accepting states 
		//progression rewards 
		//other rewards 
		//thats it 

		//for the other rewards we need to be able to get the corresponding mdp state 
		//so we basically need the mdp states list 
		//and then create a new state from that 
		//we will only have to do this once 
		//so what we'll do is store a temporary product mdp - state mdp mapping 

		//lets begin 
		//step 1 we create the varlist 

		VarList teamVL = createTeamMDPVarList(singleAgentNestedP);
		System.out.println(teamVL.toString());
		teamMDPCreator = new MDPCreator();
		teamMDPCreator.mdp.setVarList(teamVL);

		//first lets just do the team mdp 
		//we'll keep track of accepting states and essential states on the way 
		agentsEssentialStates = new ArrayList<BitSet>();
		agentsInitialStates = new ArrayList<BitSet>();
		agentsAvoidStates = new ArrayList<BitSet>();
		statesToAvoid = new BitSet();
		agentsFailStates = new ArrayList<BitSet>();
		agentsAccStates = new ArrayList<BitSet>();
		
		addRobot(0,singleAgentNestedP);
	
	}

	public void addRobot(int r, SingleAgentNestedProductMDPClean singleAgentNestedP)
	{
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		//System.getProperty("user.dir");
		String modelLocation = dir;
		String resLoc = dir + "results/";
		////profile
		long startTime = System.currentTimeMillis();

		MDPSimple agentMDP = (MDPSimple) singleAgentNestedP.finalProduct;
		HashMap<Integer, AutomatonInformation> agentAutomatonInfo = singleAgentNestedP.internalAutomatonInfo;

		BitSet avoidStates = addAgentToTeamMDP(r, agentMDP, teamMDPCreator, agentAutomatonInfo, agentsEssentialStates, agentsInitialStates, agentsAvoidStates);
		statesToAvoid.or(avoidStates);

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		System.out.println("\nAdded " + r + " to teamMDP " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
		//end profiling
		System.out.println(teamMDPCreator.mdp.infoStringTable());
		teamMDPCreator.saveMDP(resLoc, "meh"+r);

	}
	public SequentialTeamMDPClean(ArrayList<SingleAgentNestedProductMDPClean> singleAgentNestedPs) throws Exception
	{
		numRobots = singleAgentNestedPs.size();

		//so from a list of singleAgentNestedProducts 
		//we're going to create the sequential team mdp 
		//which is just a mapping 
		//things we need here are 
		//the switch states 
		//the accepting states 
		//progression rewards 
		//other rewards 
		//thats it 

		//for the other rewards we need to be able to get the corresponding mdp state 
		//so we basically need the mdp states list 
		//and then create a new state from that 
		//we will only have to do this once 
		//so what we'll do is store a temporary product mdp - state mdp mapping 

		//lets begin 
		//step 1 we create the varlist 

		VarList teamVL = createTeamMDPVarList(singleAgentNestedPs.get(0));
		System.out.println(teamVL.toString());
		teamMDPCreator = new MDPCreator();
		teamMDPCreator.mdp.setVarList(teamVL);

		//first lets just do the team mdp 
		//we'll keep track of accepting states and essential states on the way 
		agentsEssentialStates = new ArrayList<BitSet>();
		agentsInitialStates = new ArrayList<BitSet>();
		agentsAvoidStates = new ArrayList<BitSet>();
		statesToAvoid = new BitSet();
		agentsFailStates = new ArrayList<BitSet>();
		agentsAccStates = new ArrayList<BitSet>();
		
		int r = 0;

		while (!singleAgentNestedPs.isEmpty()) {
			////profile
			long startTime = System.currentTimeMillis();

			MDPSimple agentMDP = (MDPSimple) singleAgentNestedPs.get(0).finalProduct;
			HashMap<Integer, AutomatonInformation> agentAutomatonInfo = singleAgentNestedPs.get(0).internalAutomatonInfo;
			singleAgentNestedPs.remove(0);
			BitSet avoidStates = addAgentToTeamMDP(r, agentMDP, teamMDPCreator, agentAutomatonInfo, agentsEssentialStates, agentsInitialStates, agentsAvoidStates);
			statesToAvoid.or(avoidStates);

			long stopTime = System.currentTimeMillis();
			long runTime = stopTime - startTime;
			System.out.println("\nAdded " + r + " to teamMDP " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
			//end profiling

			r++;

		}

	}

	int updatedAgentDAMapping(int agentStateIndex)
	{
		return agentStateIndex + 1;
	}

	int countTasksCompleted(State ps, State cs, HashMap<Integer, AutomatonInformation> agentAutomatonInfo)
	{
		int numDAaccs = 0;

		for (int daInd : agentAutomatonInfo.keySet()) {
			int teamDAInd = updatedAgentDAMapping(daInd);
			if (!agentAutomatonInfo.get(daInd).isSafetyExpr) {
				int csVal = (int) cs.varValues[teamDAInd];
				if (agentAutomatonInfo.get(daInd).acceptingStates.get(csVal)) {
					int psVal = (int) ps.varValues[teamDAInd];
					if (psVal != csVal) {
						numDAaccs++;
					}
				}
			} else {
				int csVal = (int) cs.varValues[teamDAInd];
				if (agentAutomatonInfo.get(daInd).acceptingStates.get(csVal)) {
					numDAaccs = -1;
					break;
				}
			}
		}

		return numDAaccs;
	}

	boolean isAccepting(int numDAaccs, int numDAs)
	{
		return (numDAaccs == numDAs);
	}

	public BitSet addAgentToTeamMDP(int r, MDPSimple agentMDP, MDPCreator teamMDPCreator, HashMap<Integer, AutomatonInformation> agentAutomatonInfo,
			ArrayList<BitSet> agentsEssentialStates, ArrayList<BitSet> agentsInitialStates, ArrayList<BitSet> agentsAvoidStates)
	{
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		//System.getProperty("user.dir");
		String modelLocation = dir;
		String resLoc = dir + "results/";
		teamMDPCreator.saveMDP(agentMDP, resLoc, "agentnp"+r, null);
		BitSet agentEssStates = new BitSet();
		BitSet agentStatesToAvoid = new BitSet();
		BitSet agentAccStates = new BitSet();
		BitSet agentInitStates = new BitSet();
		BitSet agentFailStates = new BitSet(); 
		
		int[] map = new int[agentMDP.getNumStates()];
		Arrays.fill(map, -1);
		for (int s = 0; s < agentMDP.getNumStates(); s++) {
			////profile
//			long startTime = System.currentTimeMillis();

			State teamStateState;
			int teamState = map[s];
			//for each state create a new state 
			if (teamState == -1) {
				teamStateState = new State(new State(1).setValue(0, r), agentMDP.getStatesList().get(s));
				map[s] = teamMDPCreator.getStateIndex(teamStateState);
				teamState = map[s];
			} else {
				teamStateState = teamMDPCreator.mdp.getStatesList().get(teamState);
			}

			if (agentMDP.isInitialState(s))
				agentInitStates.set(teamState);

			
			int numChoices = agentMDP.getNumChoices(s);
			boolean potentialFailState = (numChoices == 0 | numChoices==1);
			ArrayList<Entry<Integer, Double>> successorsList = new ArrayList<Entry<Integer, Double>>();
			ArrayList<Integer> essentialStateCounts = new ArrayList<Integer>();
			ArrayList<Boolean> acceptingStateCounts = new ArrayList<Boolean>();
			for (int choice = 0; choice < numChoices; choice++) {
				int numSucc = 0;
				boolean potentialSelfLoop = false; 
				Iterator<Entry<Integer, Double>> tranIter = agentMDP.getTransitionsIterator(s, choice);
				Object action = agentMDP.getAction(s, choice);
				State nextTeamStateState;
				while (tranIter.hasNext()) {
					Entry<Integer, Double> nextStateProbPair = tranIter.next();
					int s1 = nextStateProbPair.getKey();
					double prob = nextStateProbPair.getValue();
					int nextTeamState = map[s1];
					if (nextTeamState == -1) {
						nextTeamStateState = new State(new State(1).setValue(0, r), agentMDP.getStatesList().get(s1));
						map[s1] = teamMDPCreator.getStateIndex(nextTeamStateState);
						nextTeamState = map[s1];

					} else {
						nextTeamStateState = teamMDPCreator.mdp.getStatesList().get(nextTeamState);
					}

					if(teamState == nextTeamState)
						potentialSelfLoop = true; 
					//check stuff here 
					int numDAaccs = countTasksCompleted(teamStateState, nextTeamStateState, agentAutomatonInfo);
					if (numDAaccs > 0)
						agentEssStates.set(nextTeamState);
					if (numDAaccs == -1)
						agentStatesToAvoid.set(nextTeamState);

					boolean isAcc = isAccepting(numDAaccs, agentAutomatonInfo.size() - 1);
					if (isAcc)
						agentAccStates.nextSetBit(nextTeamState);
					essentialStateCounts.add(numDAaccs);
					acceptingStateCounts.add(isAcc);
					SimpleEntry<Integer, Double> updatedSPPair = new AbstractMap.SimpleEntry<Integer, Double>(s1, prob);
					successorsList.add(updatedSPPair);
					numSucc++; 
				}

				if(potentialFailState)
				{
					boolean isFailState = false; 
					if(numSucc == 1)
					{
						if (potentialSelfLoop)
						{
							isFailState = true; 
						}
					}
					else
					{
						if(numSucc == 0)
						{
							if(!agentAccStates.get(teamState))
								isFailState = true; 
						}
					}
					if(isFailState)
						agentFailStates.set(teamState);
				}
				teamMDPCreator.addAction(teamState, action, successorsList, essentialStateCounts, acceptingStateCounts, 0.0);
		
//				teamMDPCreator.saveMDP(resLoc, "meh"+r+action.toString());
			}

//			long stopTime = System.currentTimeMillis();
//			long runTime = stopTime - startTime;
//			System.out.println("\nProcessed state: " + s + " of " + agentMDP.getNumStates() + " " + runTime + "ms" + "("
//					+ TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
//			//end profiling

		}
		agentsEssentialStates.add((BitSet) agentEssStates.clone());
		agentsAvoidStates.add((BitSet) agentStatesToAvoid.clone());
		agentsInitialStates.add((BitSet) agentInitStates.clone());
		agentsFailStates.add((BitSet)agentFailStates.clone());
		agentsAccStates.add((BitSet)agentAccStates.clone());

		return agentStatesToAvoid;
	}
	
	public void printStuff()
	{
		System.out.println("Essential States"); 
		for(int i = 0; i<numRobots; i++)
			System.out.println(agentsEssentialStates.get(i).toString());
		System.out.println("All Essential States"); 
		System.out.println(teamMDPCreator.essStates.toString());
		
		System.out.println("Initial States"); 
		for(int i = 0; i<numRobots; i++)
			System.out.println(agentsInitialStates.get(i).toString());
		System.out.println("Avoid States"); 
		for(int i = 0; i<numRobots; i++)
			System.out.println(agentsAvoidStates.get(i).toString());
		System.out.println("Fail States"); 
		for(int i = 0; i<numRobots; i++)
			System.out.println(agentsFailStates.get(i).toString());
		System.out.println("Agent Acc States");
		for(int i = 0; i<numRobots; i++)
			System.out.println(agentsAccStates.get(i).toString());
		System.out.println("All Acc States"); 
		System.out.println(teamMDPCreator.accStates.toString());
		
	}

	VarList createTeamMDPVarList(SingleAgentNestedProductMDPClean singleAgentNestedP) throws Exception
	{

		VarList vl = new VarList();
		//the assumption is that the sanps all have the same da labels and everything 
		//so we can just do with one 
		//and they all have the same shared states too 

		VarList svl = singleAgentNestedP.finalProduct.getVarList();
		int maxDAs = singleAgentNestedP.internalAutomatonInfo.size();
		int countDA = 0;
		String daVar;
		Declaration decl;
		for (int v = 0; v < svl.getNumVars(); v++) {
			daVar = svl.getName(v);
			decl = svl.getDeclaration(v);
			int module = svl.getModule(v);
			//if this isnt in the vl mapping then its a da duh 
			if (daVar.contains("da")) {

				//its a da 

				AutomatonInformation daInfo = singleAgentNestedP.internalAutomatonInfo.get(countDA);
				//we know that the daCount is reversed 
				String newName = daInfo.daLabel;
				if (!(daInfo.originalLabel.contentEquals(daVar)))
					throw new PrismException("The DA matching is messed up!!!");
				decl.setName(newName);
				countDA++;

			}

			vl.addVar(decl, module, singleAgentNestedP.finalProduct.getConstantValues());
		}

		//now add one for the robot number 
		daVar = "r";
		DeclarationInt daint = new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(numRobots - 1, 1)));
		decl = new Declaration(daVar, daint);
		vl.addVar(0, decl, 1, singleAgentNestedP.finalProduct.getConstantValues());

		return vl;
	}

	public void addSwitchesAndSetInitialState(int firstRobot, boolean includefailstatesinswitches, boolean completeSwitchRing)
	{
		int[] robotStates = new int[numRobots];
		for (int i = 0; i < numRobots; i++) {
			// these are not the real states but just states that reflect the mdp state
			// this of course is not nice at all
			// it is a silly thing to do and should be fixed
			robotStates[i] = agentsInitialStates.get(i).nextSetBit(0);
		}
		addSwitchesAndSetInitialState(firstRobot, robotStates, includefailstatesinswitches,completeSwitchRing);
		
	}

	private void addSwitchesAndSetInitialState(int firstRobot, int[] robotStates, boolean includefailstatesinswitches, boolean completeSwitchRing)
	{
		MDPSimple teamMDPTemplate = teamMDPCreator.mdp; 
		// set initial state as those from the first robot
				// you still need to check if the robot has failed
				initializeTeamMDPFromTemplate();
				BitSet initialStatesFirstRobot = agentsInitialStates.get(firstRobot);
				// get the state from the coresponding model
//				int state = agentMDPs.get(firstRobot).finalProduct.getProductModel().getFirstInitialState();
//				state = agentMDPsToSeqTeamMDPStateMapping.get(firstRobot)[state];
//				if (initialStatesFirstRobot.get(state))
//					teamMDPWithSwitches.addInitialState(state);
				//^ I am not sure what the point of that was anyway. 
				//like we form the first initial state ourselves so you know ? 
				//wtf man 
				
				boolean[] isFailedState = new boolean[numRobots];
				for (int i = 0; i < numRobots; i++) {
					// check if there are any fail states
					isFailedState[i] = StatesHelper.isFailState(teamMDPTemplate.getStatesList().get(robotStates[i]));
					// also if you have an initial state that is an accepting state
					// you need to figure out what to do with it
					// because it could be the first robot - which is fine cuz thats the initial
					// state
					// but if its not you need to get rid of all states where that thing is not an
					// accepting state
					// what i mean to say is suppose v1 is a goal, and r2 is on v1 for the first
					// time ever
					// you need to know this stuff and account for it
					// technically check labels and stuff ??? I dont know I have to think about this
					// TODO: INITIAL STATE STUFF FOR LATER see above
				}

				addSwitchTransitions(firstRobot, isFailedState, includefailstatesinswitches,completeSwitchRing);
		
	}

	private int addSwitchTransitions(int firstRobot, boolean[] isFailedState, boolean includefailstatesinswitches, boolean completeSwitchRing)
	{
		int totalSwitches = 0;
		boolean addSwitches = true; // just going to use this to make sure that the first robot doesnt get a switch
		for (int r = 0; r < numRobots; r++) {
			int fromRobot = r;
			int toRobot = (r + 1) % numRobots;
			// dont add switches to the first robot
			if (!completeSwitchRing)
			addSwitches = (toRobot != firstRobot);
			if (addSwitches) {

				BitSet fromRobotEssentialStates = (BitSet) agentsEssentialStates.get(fromRobot).clone();
				if (isFailedState[fromRobot])
					fromRobotEssentialStates = agentsInitialStates.get(fromRobot);
				// adding an else so we have switch states from intial states too
				else {
					fromRobotEssentialStates.or(agentsInitialStates.get(fromRobot));
					if (includefailstatesinswitches) {
						fromRobotEssentialStates.or(agentsFailStates.get(fromRobot));
					}
				}

				BitSet toRobotInitialStates = agentsInitialStates.get(toRobot);
//				totalSwitches += addSwitchTransitionsBetweenRobots(toRobot, fromRobot, fromRobotEssentialStates,
//						toRobotInitialStates);
			}
		}
//		teamMDPWithSwitches.findDeadlocks(true);
		return totalSwitches;
		
	}

	private void initializeTeamMDPFromTemplate()
	{
		MDPSimple teamMDPTemplate = teamMDPCreator.mdp; 
		teamMDPWithSwitches = new MDPSimple(teamMDPTemplate);
		teamMDPWithSwitches.setStatesList(teamMDPTemplate.getStatesList());
		if(teamMDPCreator.expectedTaskCompletionRewards==null)
			teamMDPCreator.createRewardStructures();
		
		rewardsWithSwitches = new ArrayList<MDPRewardsSimple>(teamMDPCreator.getRewardsInArray());

		
	}

}
