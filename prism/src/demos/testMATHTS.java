package demos;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.concurrent.TimeUnit;
import java.util.Stack;

import acceptance.AcceptanceOmega;
import automata.DA;
import demos.ResultsTiming.varIDs;
import parser.State;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

public class testMATHTS
{

	String testsLocation = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
	
	String resultsLocation = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/results/";
	
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
//		testSingleAgentLoader();
		String filename = "grid_3_topomap_sim_doors";//"tiny_example_permtrap";//"no_door_example";	
//		doTHTS(filename,2,3,2);
		runTest();
	}

	public void testMAPMG(PrismLog mainLog,
			ArrayList<SingleAgentLoader> sals, 
			DA<BitSet, ? extends AcceptanceOmega> da,
			String saveplace, String fn) throws PrismException
	{
		boolean buildMDP = true; 
		MultiAgentProductModelGenerator jpmg = 
				new MultiAgentProductModelGenerator(mainLog,sals,da,buildMDP); 
//		ArrayList<State> initStates = jpmg.createInitialStateFromRobotInitStates();
//		Stack<State> toVisit = new Stack<State>();
//		Stack<State> visited = new Stack<State>();
//		toVisit.addAll(initStates);
//		boolean checkhere = false; 
//		while(!toVisit.isEmpty())
//		{
//			State js = toVisit.pop();
//			visited.add(js);
//			//get the actions 
//			ArrayList<Object> jas = jpmg.getJointActions(js);
//			mainLog.println("Printing actions");
//			for(int i = 0; i<jas.size(); i++)
//			{
//				Object ja = jas.get(i);
//				mainLog.println(ja.toString()); 
//				if(ja.toString().contains("cd"))
//					checkhere = true; 
//				//get successors 
//				ArrayList<Entry<State, Double>> successors = jpmg.getSuccessors(js, ja);
//				for(int j = 0; j<successors.size(); j++)
//				{
//					State ss = successors.get(j).getKey();
//					if(!toVisit.contains(ss) && !visited.contains(ss))
//					{
//						toVisit.add(ss);
//					}
//					mainLog.println(successors.get(j).toString());
//				}
//			}
//		}
//		jpmg.saveBuiltMDP(saveplace, fn+"_all");
//		//now lets test the policy creator 
//		//and the random action selection thing 
//		ActionSelectionRandom randomActionSelector = new ActionSelectionRandom(jpmg.getBuiltMDP());
//		PolicyCreator pc = new PolicyCreator(); 
//		pc.createPolicy(jpmg.getBuiltMDP(), randomActionSelector); 
//		pc.savePolicy(saveplace, fn+"_random_policy");
		testTHTS(jpmg,mainLog,saveplace,fn);
		
		
	}
	public void testSingleAgentLoader() throws PrismException, FileNotFoundException
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
	
	public void doTHTS(String resLoc,String modelLoc,String filename,int numRobots,int numTasks,int numDoors,int numFS) throws PrismException, FileNotFoundException
	{
		

		// Create a log for PRISM output (hidden or stdout)
		// PrismLog mainLog = new PrismDevNullLog();


		PrismLog mainLog = new PrismFileLog("stdout");
		ResultsTiming resSaver = new ResultsTiming(mainLog,filename,resLoc,true);
		
		resSaver.recordInits(numRobots, "Robots", varIDs.numrobots);
		resSaver.recordInits(numTasks, "Tasks", varIDs.numtasks);
		resSaver.recordInits(numDoors, "Doors", varIDs.numdoors);
		resSaver.recordInits(numFS, "Failstates", varIDs.failstates);
		
		Long startTime = System.nanoTime();
		boolean buildMDP = false; 
		// Initialise PRISM engine
		
		resSaver.setGlobalStartTime();
//		resSaver.setScopeStartTime();
		resSaver.setLocalStartTime();
		
		Prism prism = new Prism(mainLog);
		prism.initialise();
		prism.setEngine(Prism.EXPLICIT);

		ArrayList<String> filenames = new ArrayList<String>();
		for(int i = 0; i<numRobots; i++) {
		filenames.add(modelLoc + filename + i+".prism");
//		filenames.add(testsLocation + filename + i+".prism");
		}
		String propfilename = modelLoc + filename + ".props";
		DA<BitSet, ? extends AcceptanceOmega> da=null; 
		ArrayList<SingleAgentLoader> sals = new ArrayList<SingleAgentLoader>();
		ArrayList<String> ssl = null; 
		if(numDoors > 0)
		{
			ssl = new ArrayList<String>(); 
			for(int i = 0; i<numDoors; i++)
			{
				ssl.add("door"+i);
			}
		}

		
//		ssl = null;
//		ssl.add("door");

//		numrobots, numtasks, numreallocstates, teammdpstates, 
//		teammdptransitions, numreallocationsincode, totalcomputationtime, 
//		totalteammdpcreationtime, allnestedproductcreationtime, allreallocationstime, 
//		productcreation, reallocations, jointpolicycreation, nestedproductstates, nestedproducttimes, 
//		numdoors, modelloadingtimes, totalmodelloadingtime, teammdptimeonly, 
//		singleagentsolutiontimes,
		for (int i = 0; i < filenames.size(); i++) {
			resSaver.setScopeStartTime();
			String modelName = filenames.get(i);
			SingleAgentLoader sal = new SingleAgentLoader(prism, mainLog, filename + i, modelName, propfilename, resultsLocation,ssl);
		
			sal.setUp();
			da = sal.getSingleAgentModelGenReturnDA();

//			maxStateEstimate *= da.size(); 
//			sal.setMaxStatesEstimate(maxStateEstimate);
			sals.add(sal);

			resSaver.recordTime("model loading time "+modelName, varIDs.modelloadingtimes,true);
		}
		resSaver.recordTime("all models loading time", varIDs.totalmodelloadingtime, false);
		resSaver.setLocalStartTime();

		for(int i = 0; i<sals.size(); i++)
		{
			resSaver.setScopeStartTime();
			SingleAgentLoader sal = sals.get(i);
			sal.solveUsingPartialSatisfaction();
			sal.solutionProdModelVarListsAreSynced();
			sal.cleanUp();
			
			int maxStateEstimate = sal.getMaxStatesEstimate(); 
			resSaver.recordTime("Single Agent Solution "+i, varIDs.singleagentsolutiontimes,true);
			resSaver.recordValues(maxStateEstimate, "Single Agent "+i+" States", varIDs.nestedproductstates);
		}
		resSaver.recordTime("Total Single Agent Solution Time",varIDs.allnestedproductcreationtime,false);
		resSaver.setScopeStartTime();
		long elapsedTime = System.nanoTime() - startTime;
		mainLog.println("Single Agent Models Loaded and Solved " + elapsedTime + "ns "
				+ TimeUnit.MILLISECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS)+" ms "
				+ TimeUnit.SECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS) + "s");
	
		MultiAgentProductModelGenerator jpmg = 
				new MultiAgentProductModelGenerator(mainLog,sals,da,buildMDP); 
		
		
		ArrayList<Objectives> tieBreakingOrder = new ArrayList<Objectives>(); 
		tieBreakingOrder.add(Objectives.Probability); 
		tieBreakingOrder.add(Objectives.Progression); 
		tieBreakingOrder.add(Objectives.Cost);
		ActionSelection actionSelection = new ActionSelectionGreedy(tieBreakingOrder);
		OutcomeSelection outcomeSelection = new OutcomeSelectionBounds();
		HeuristicFunction heuristicFunction = new HeuristicFunctionPartSat(jpmg);
		BackupFunction backupFunction = new BackupFunctionFullBellman(jpmg,tieBreakingOrder);
		
		TrialBHeuristicSearch thts = new TrialBHeuristicSearch(mainLog,jpmg,
				actionSelection,
				outcomeSelection,
				heuristicFunction,
				backupFunction,
				tieBreakingOrder, 
				resLoc, 
				filename
				);
		int mt = thts.setMaxTrialLength();
		resSaver.recordTime("Multi Agent Product Initialization", varIDs.jointmodelgentime, true);
		mainLog.println("Max Trial Length: "+mt);
		Object a = thts.doTHTS();
		mainLog.println(a.toString());
	
		elapsedTime = System.nanoTime() - startTime;
		mainLog.println("Completed in " + elapsedTime + "ns "
				+ TimeUnit.MILLISECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS)+" ms "
				+ TimeUnit.SECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS) + "s");
		resSaver.writeResults();
	}

	public void runTest()
	{
///r2/t10/fs0/d3/grid_3_topomap1.prism
		// saving filenames etc
		String dir = System.getProperty("user.dir");
		String modelLocation = dir + "/tests/decomp_tests/";
		modelLocation = dir + "/tests/autogen_testfiles/";

		String path = modelLocation;
		//	    goalsRange = [2,4,6,8,10]
		//	    	    agentsRange = [2,4,6,8,10]
		//	    	    fsRange = [0,2,4,8,16,32]
		//	    	    ssRange = [0,1,2,3,4]
		ArrayList<String> modelnames = new ArrayList<String>();
		
		String modelname = "grid_3_topomap";
		modelnames.add(modelname);
		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_task_list= new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list= new HashMap<String, Integer>();
		ArrayList<String> allModelLocs = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();
		int[] goalsRange = { 2 , 4, 6, 8/*, 10*/ };
		int[] agentsRange = { 2 , 4, 6, 8, 10 };
		int[] fsRange = { 0, 2 , 4, 8, 16, 32 };
		int[] ssRange = { 0, 1, 2, 3, 4 };
		for (int numAgents : agentsRange) {
			for (int numGoals : goalsRange) {
				for (int numFailStates : fsRange) {
					for (int numDoors : ssRange) {
						String thisModelLoc = modelLocation + modelname + "/" + "r" + numAgents + "/t" + (numGoals) + "/fs" + (numFailStates) + "/d"
								+ (numDoors) + "/";
						allModelLocs.add(thisModelLoc);
						String example_id = modelname + "_r" + numAgents + "_t" + (numGoals) + "_fs" + (numFailStates) + "_d" + numDoors;
						if (numDoors == 0)
							example_has_door_list.put(example_id, false);
						else
							example_has_door_list.put(example_id, true);

						example_num_robot_list.put(example_id, numAgents);
						example_num_door_list.put(example_id, numDoors);
						example_num_task_list.put(example_id, numGoals);
						example_num_fs_list.put(example_id, numFailStates);
						example_ids.add(example_id);

					}
				}
			}
		}

		ArrayList<String> modelsTested = new ArrayList<String>();
		try {
			for (int i = 123; i < allModelLocs.size(); i++) {

				String thisModelLoc = allModelLocs.get(i);
				String example_name = modelname;
				String example_id = example_ids.get(i);
				System.out.println(thisModelLoc);
				System.out.println(example_id);
				modelsTested.add(example_id);
//				StatesHelper.setSavePlace(thisModelLoc + "results/");
				System.out.println("Example "+i+" of "+allModelLocs.size());
//				if(true)
//				throw new FileNotFoundException();
//				runOneExample(example_name, example_id, example_has_door_list, example_num_door_list, example_num_robot_list, thisModelLoc, true);
//				if(true)
				doTHTS(thisModelLoc+"results/",thisModelLoc,example_name,example_num_robot_list.get(example_id),
						example_num_task_list.get(example_id),example_num_door_list.get(example_id),example_num_fs_list.get(example_id));
				
				System.out.println("Example "+i+" of "+allModelLocs.size()+ " completed");
//				throw new PrismException("3");
			}
		} catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
		System.out.println("Models Tested: " + modelsTested.size());
		for (int i = 0; i < modelsTested.size(); i++) {
			System.out.println(modelsTested.get(i));
		}
		System.out.println("Models Tested: " + modelsTested.size());
}
}
