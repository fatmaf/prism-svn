package explicit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Vector;

import explicit.rewards.MDPRewardsSimple;
import parser.State;
import prism.PrismException;

class SequentialTeamMDP {
	/**
	 * 
	 */
	private final STAPU stapu;
	ArrayList<SingleAgentNestedProductMDP> agentMDPs;
	ArrayList<int[]> agentMDPsToSeqTeamMDPStateMapping;
	BitSet acceptingStates;
	BitSet statesToAvoid;
	Vector<BitSet> essentialStates;
	Vector<BitSet> initialStates;
	MDPSimple teamMDPTemplate;
	ArrayList<MDPRewardsSimple> teamRewardsTemplate;
	MDPSimple teamMDPWithSwitches;
	ArrayList<MDPRewardsSimple> rewardsWithSwitches;
	int numRobots;

	public SequentialTeamMDP(STAPU stapu, int nRobots) {
		this.stapu = stapu;
		essentialStates = new Vector<BitSet>();
		initialStates = new Vector<BitSet>();
		numRobots = nRobots;
	}

	
	public void addSwitchesAndSetInitialState(int firstRobot) throws PrismException {
		int[] robotStates = new int[numRobots]; 
		for(int i = 0; i<numRobots; i++)
		{	
			//these are not the real states but just states that reflect the mdp state 
			//this of course is not nice at all 
			//it is a silly thing to do and should be fixed 
			robotStates[i] = initialStates.get(i).nextSetBit(0); 
		}
		addSwitchesAndSetInitialState(firstRobot,robotStates); 
		}


	public void addSwitchesAndSetInitialState(int firstRobot, int[] robotStates) throws PrismException {
		// set initial state as those from the first robot
		//you still need to check if the robot has failed 
		initializeTeamMDPFromTemplate();
		BitSet initialStatesFirstRobot = initialStates.get(firstRobot);
		// get the state from the coresponding model
		int state = agentMDPs.get(firstRobot).finalProduct.getProductModel().getFirstInitialState();
		state = agentMDPsToSeqTeamMDPStateMapping.get(firstRobot)[state];
		if (initialStatesFirstRobot.get(state))
			teamMDPWithSwitches.addInitialState(state);
		boolean[] isFailedState=new boolean[numRobots]; 
		for(int i = 0; i<numRobots; i++) {
			//check if there are any fail states 
			isFailedState[i] =  StatesHelper.isFailState(teamMDPTemplate.getStatesList().get(robotStates[i]));
			//also if you have an initial state that is an accepting state 
			//you need to figure out what to do with it 
			//because it could be the first robot - which is fine cuz thats the initial state 
			//but if its not you need to get rid of all states where that thing is not an accepting state 
			//what i mean to say is suppose v1 is a goal, and r2 is on v1 for the first time ever 
			//you need to know this stuff and account for it 
			//technically check labels and stuff ??? I dont know I have to think about this 
			//TODO: INITIAL STATE STUFF FOR LATER see above 
		}
		
		addSwitchTransitions(firstRobot,isFailedState);
	}
	public void addSwitchTransitions(int firstRobot,boolean[] hasFailed) throws PrismException {

		int totalSwitches = 0;
		boolean addSwitches = true; // just going to use this to make sure that the first robot doesnt get a switch
		for (int r = 0; r < agentMDPs.size(); r++) {
			int fromRobot = r;
			int toRobot = (r + 1) % agentMDPs.size();
			// dont add switches to the first robot
			addSwitches = (toRobot != firstRobot);
			if (addSwitches) {
				
				BitSet fromRobotEssentialStates = essentialStates.get(fromRobot);
				if (hasFailed[fromRobot])
					fromRobotEssentialStates = initialStates.get(fromRobot); 
				
				BitSet toRobotInitialStates = initialStates.get(toRobot);
				totalSwitches += addSwitchTransitionsBetweenRobots(toRobot, fromRobot,fromRobotEssentialStates,toRobotInitialStates);}
		}
		teamMDPWithSwitches.findDeadlocks(true);

	}
	private int addSwitchTransitionsBetweenRobots(int toRobot, int fromRobot,BitSet fromRobotEssentialStates, BitSet toRobotInitialStates) {
		// from the essential states of from robot
		// to the initial states of to robot
		int totalSwitches = 0;
		double switchProb = 1.0;

		int fromRobotState = fromRobotEssentialStates.nextSetBit(0);
		int toRobotState;// = toRobotInitialStates.nextSetBit(0);

		while (fromRobotState != -1) {
			State fromRobotStateVar = teamMDPTemplate.getStatesList().get(fromRobotState);
			toRobotState = toRobotInitialStates.nextSetBit(0);
			while (toRobotState != -1) {

				State toRobotStateVar = teamMDPTemplate.getStatesList().get(toRobotState);
				// add a link if the automaton progress is the same
				if (StatesHelper.statesHaveTheSameAutomataProgress(fromRobotStateVar, toRobotStateVar)) {
					Distribution distr = new Distribution();
					distr.add(toRobotState, switchProb);
					teamMDPWithSwitches.addActionLabelledChoice(fromRobotState, distr,
							"switch_" + fromRobot + "-" + toRobot);
					// TODO check if we need a reward for a switch
					for (int i = 0; i < rewardsWithSwitches.size(); i++) {
						int numChoice = teamMDPWithSwitches.getNumChoices(fromRobotState) - 1;
						rewardsWithSwitches.get(i).addToTransitionReward(fromRobotState, numChoice, 0.0);
					}
					totalSwitches++;
					// making the assumption that since the to and from states are actually states
					// from the mdp
					// where we need too match progress
					// this can only happen once per state
					// hence this break
					// TODO: verify this
					break;
				}
				toRobotState = toRobotInitialStates.nextSetBit(toRobotState + 1);
			}
			fromRobotState = fromRobotEssentialStates.nextSetBit(fromRobotState + 1);
		}
		this.stapu.mainLogRef.println(totalSwitches == fromRobotEssentialStates.cardinality());
		return totalSwitches;
	}

	private BitSet convertAgentMDPStateToTeamState(BitSet agentStates,int r)
	{
		BitSet teamStates = new BitSet(); 
		
		int setBit = agentStates.nextSetBit(0); 
		while(setBit!=-1)
		{
			int teamBit = agentMDPsToSeqTeamMDPStateMapping.get(r)[setBit]; 
			//interesting caveat bith (in urdu with the thay) 
			if (!statesToAvoid.get(teamBit))
				{teamStates.set(teamBit);
				State s1 = teamMDPTemplate.getStatesList().get(teamBit);
				s1.toString();
				}
			setBit = agentStates.nextSetBit(setBit+1);
		}
		return teamStates;
		
	}

	public int getNestedProductStateFromTeamState(int state,int rNum)
	{
		int nestedProductState = StatesHelper.BADVALUE; 
		int[] mapping = agentMDPsToSeqTeamMDPStateMapping.get(rNum);
		for(int agentState=0; agentState<mapping.length; agentState++)
		{
			if(mapping[agentState]==state)
			{	nestedProductState = agentState;
			break;}
		}
		return nestedProductState; 
	}
	

	public void initializeTeamMDPFromTemplate() {
		teamMDPWithSwitches = new MDPSimple(teamMDPTemplate);
		teamMDPWithSwitches.setStatesList(teamMDPTemplate.getStatesList());
		rewardsWithSwitches = new ArrayList<MDPRewardsSimple>(teamRewardsTemplate);

		// remember MDPs are not so easy to copy !!!
		// so we need to fix this

	}


//	public void addSwitchTransitions(int firstRobot) throws PrismException {
//
//		int totalSwitches = 0;
//		boolean addSwitches = true; // just going to use this to make sure that the first robot doesnt get a switch
//		for (int r = 0; r < agentMDPs.size(); r++) {
//			int fromRobot = r;
//			int toRobot = (r + 1) % agentMDPs.size();
//			// dont add switches to the first robot
//			addSwitches = (toRobot != firstRobot);
//			if (addSwitches) {
//
//				BitSet fromRobotEssentialStates = essentialStates.get(fromRobot);
//				BitSet toRobotInitialStates = initialStates.get(toRobot);
//				totalSwitches += addSwitchTransitionsBetweenRobots(toRobot, fromRobot,fromRobotEssentialStates,toRobotInitialStates);}
//		}
//		teamMDPWithSwitches.findDeadlocks(true);
//
//	}

	public void setInitialStates(int[] robotStates) {
		initialStates = new Vector<BitSet>();
		for (int r = 0; r < robotStates.length; r++) {
			SingleAgentNestedProductMDP mdp = agentMDPs.get(r);
			int mdpState = getNestedProductStateFromTeamState(robotStates[r],r);
			BitSet initialStatesForRobot = mdp.getAndSetInitialStates(mdpState, false);
			BitSet initialStatesForRobotInTeam = (BitSet)convertAgentMDPStateToTeamState(initialStatesForRobot,r).clone();
//			if (initialStates.size() > r)
//				initialStates.set(r, initialStatesForRobot);
//			else
				initialStates.add(initialStatesForRobotInTeam);
		}
	}

}