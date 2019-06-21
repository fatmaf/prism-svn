package demos;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Stack;

import acceptance.AcceptanceOmega;
import automata.DA;
import parser.State;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

public class testMATHTS
{

	public static void main(String[] args)
	{
		try {
			new testMATHTS().run();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void testTHTS(MultiAgentProductModelGenerator jpmg,PrismLog ml,String sv, String fn) throws PrismException 
	{
		ArrayList<Objectives> tieBreakingOrder = new ArrayList<Objectives>(); 
		tieBreakingOrder.add(Objectives.Probability); 
		tieBreakingOrder.add(Objectives.Progression); 
		tieBreakingOrder.add(Objectives.Cost);
		ActionSelection actionSelection = new ActionSelectionGreedy(tieBreakingOrder);
		OutcomeSelection outcomeSelection = new OutcomeSelectionBounds();
		HeuristicFunction heuristicFunction = new HeuristicFunctionPartSat(jpmg);
		BackupFunction backupFunction = new BackupFunctionFullBellman(jpmg,tieBreakingOrder);
		
		TrialBHeuristicSearch thts = new TrialBHeuristicSearch(ml,jpmg,
				actionSelection,
				outcomeSelection,
				heuristicFunction,
				backupFunction,
				tieBreakingOrder, 
				sv, 
				fn
				);
		Object a = thts.doTHTS();
		ml.println(a.toString());
	}
	public void run() throws FileNotFoundException, PrismException
	{
		loadSingleAgent();
	}

	public void testMAPMG(PrismLog mainLog,
			ArrayList<SingleAgentLoader> sals, 
			DA<BitSet, ? extends AcceptanceOmega> da,
			String saveplace, String fn) throws PrismException
	{
		boolean buildMDP = true; 
		MultiAgentProductModelGenerator jpmg = 
				new MultiAgentProductModelGenerator(mainLog,sals,da,buildMDP); 
		ArrayList<State> initStates = jpmg.createInitialStateFromRobotInitStates();
		Stack<State> toVisit = new Stack<State>();
		Stack<State> visited = new Stack<State>();
		toVisit.addAll(initStates);
		boolean checkhere = false; 
		while(!toVisit.isEmpty())
		{
			State js = toVisit.pop();
			visited.add(js);
			//get the actions 
			ArrayList<Object> jas = jpmg.getJointActions(js);
			mainLog.println("Printing actions");
			for(int i = 0; i<jas.size(); i++)
			{
				Object ja = jas.get(i);
				mainLog.println(ja.toString()); 
				if(ja.toString().contains("cd"))
					checkhere = true; 
				//get successors 
				ArrayList<Entry<State, Double>> successors = jpmg.getSuccessors(js, ja);
				for(int j = 0; j<successors.size(); j++)
				{
					State ss = successors.get(j).getKey();
					if(!toVisit.contains(ss) && !visited.contains(ss))
					{
						toVisit.add(ss);
					}
					mainLog.println(successors.get(j).toString());
				}
			}
		}
		jpmg.saveBuiltMDP(saveplace, fn+"_all");
		//now lets test the policy creator 
		//and the random action selection thing 
		ActionSelectionRandom randomActionSelector = new ActionSelectionRandom(jpmg.getBuiltMDP());
		PolicyCreator pc = new PolicyCreator(); 
		pc.createPolicy(jpmg.getBuiltMDP(), randomActionSelector); 
		pc.savePolicy(saveplace, fn+"_random_policy");
		testTHTS(jpmg,mainLog,saveplace,fn);
		
		
	}
	public void loadSingleAgent() throws PrismException, FileNotFoundException
	{

		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "tiny_example_permtrap";//"no_door_example";	
		String TESTSLOC = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/results/";

		// Create a log for PRISM output (hidden or stdout)
		// PrismLog mainLog = new PrismDevNullLog();

		PrismLog mainLog = new PrismFileLog("stdout");
		Long startTime = System.currentTimeMillis();

		// Initialise PRISM engine

		Prism prism = new Prism(mainLog);
		prism.initialise();
		prism.setEngine(Prism.EXPLICIT);

		ArrayList<String> filenames = new ArrayList<String>();
		filenames.add(saveplace + filename + "0.prism");
		filenames.add(saveplace + filename + "1.prism");
		String propfilename = saveplace + filename + ".props";
		DA<BitSet, ? extends AcceptanceOmega> da=null; 
		ArrayList<SingleAgentLoader> sals = new ArrayList<SingleAgentLoader>();
		ArrayList<String> ssl = new ArrayList<String>(); 
		ssl.add("door");
		ssl = null;
		for (int i = 0; i < filenames.size(); i++) {
			String modelName = filenames.get(i);
			SingleAgentLoader sal = new SingleAgentLoader(prism, mainLog, filename + i, modelName, propfilename, TESTSLOC,ssl);
			sal.setUp();
			sal.solutionProdModelVarListsAreSynced();
			da = sal.getSingleAgentModelGenReturnDA();
			sal.solutionProdModelVarListsAreSynced();
			sal.solveUsingVI();
			sal.solutionProdModelVarListsAreSynced();
			sal.solveUsingPartialSatisfaction();
			sal.solutionProdModelVarListsAreSynced();
			sal.cleanUp();
			State s = sal.prodModelGen.getInitialState();
			double d = sal.getSolutionValue(s, Objectives.Probability);
			d = sal.getSolutionValue(s, Objectives.Progression);
			d = sal.getSolutionValue(s, Objectives.Cost);
			boolean isdeadend = sal.isDeadend(s);
			sal.getDAState(s); 
			sal.getDAStateAsInt(s); 
			sal.getSharedState(s); 
			sal.getPrivateState(s);
			mainLog.println(d);
			sals.add(sal);
			//not tested the create robot state function 

		}
		testMAPMG(mainLog,sals,da,TESTSLOC,filename); 
		

	}
}
