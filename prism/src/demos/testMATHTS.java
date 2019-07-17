package demos;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
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

public class testMATHTS {

	String testsLocation = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";

	String resultsLocation = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/results/thts_res";

	public static void main(String[] args) {
		try {
			new testMATHTS().run();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void testTHTS(MultiAgentProductModelGenerator jpmg, PrismLog ml, String sv, String fn)
			throws PrismException {
		ArrayList<Objectives> tieBreakingOrder = new ArrayList<Objectives>();
		tieBreakingOrder.add(Objectives.Probability);
		tieBreakingOrder.add(Objectives.Progression);
		tieBreakingOrder.add(Objectives.Cost);
		ActionSelection actionSelection = new ActionSelectionGreedy(tieBreakingOrder);
		OutcomeSelection outcomeSelection = new OutcomeSelectionBounds();
		HeuristicFunction heuristicFunction = new HeuristicFunctionPartSat(jpmg);
		BackupFunction backupFunction = new BackupFunctionFullBellman(jpmg, tieBreakingOrder);

		TrialBHeuristicSearch thts = new TrialBHeuristicSearch(ml, jpmg, actionSelection, outcomeSelection,
				heuristicFunction, backupFunction, tieBreakingOrder, sv, fn, null);
		Object a = thts.doTHTS("");
		ml.println(a.toString());
	}

	public void run() throws PrismException, IOException {
//		testSingleAgentLoader();
//		String filename = "grid_3_topomap_sim_doors";// "tiny_example_permtrap";//"no_door_example";
//		int robots = 4;
//		int tasks = 7;
//		int doors = 2;
//		int fs = 7;
//		for (robots = 2; robots <= 4; robots++) {
//			doTHTS(resultsLocation, testsLocation, filename, robots, tasks, doors, fs);
//
//		}
//		System.out.println("all done");
//		runGUITest();
//		runTest();
		runGUISimpleTests();
//		runDebugOne();
	}

	public void testMAPMG(PrismLog mainLog, ArrayList<SingleAgentLoader> sals, DA<BitSet, ? extends AcceptanceOmega> da,
			String saveplace, String fn) throws PrismException {
		boolean buildMDP = true;
		MultiAgentProductModelGenerator jpmg = new MultiAgentProductModelGenerator(mainLog, sals, da, buildMDP);
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
		testTHTS(jpmg, mainLog, saveplace, fn);

	}

	public void testSingleAgentLoader() throws PrismException, FileNotFoundException {

		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "tiny_example_permtrap";// "no_door_example";
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
		DA<BitSet, ? extends AcceptanceOmega> da = null;
		ArrayList<SingleAgentLoader> sals = new ArrayList<SingleAgentLoader>();
		ArrayList<String> ssl = new ArrayList<String>();
		ssl.add("door");
		ssl = null;
		for (int i = 0; i < filenames.size(); i++) {
			String modelName = filenames.get(i);
			SingleAgentLoader sal = new SingleAgentLoader(prism, mainLog, filename + i, modelName, propfilename,
					TESTSLOC, ssl);
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
			// not tested the create robot state function

		}
		testMAPMG(mainLog, sals, da, TESTSLOC, filename);

	}

	public void doTHTS(String resLoc, String modelLoc, String filename, int numRobots, int numTasks, int numDoors,
			int numFS) throws PrismException, FileNotFoundException {

		// Create a log for PRISM output (hidden or stdout)
		// PrismLog mainLog = new PrismDevNullLog();

		PrismLog mainLog = new PrismFileLog("stdout");
		ResultsTiming resSaver = new ResultsTiming(mainLog, filename, resLoc, true);

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
		for (int i = 0; i < numRobots; i++) {
			filenames.add(modelLoc + filename + i + ".prism");
//		filenames.add(testsLocation + filename + i+".prism");
		}
		String propfilename = modelLoc + filename + ".props";
		DA<BitSet, ? extends AcceptanceOmega> da = null;
		ArrayList<SingleAgentLoader> sals = new ArrayList<SingleAgentLoader>();
		ArrayList<String> ssl = null;
		if (numDoors > 0) {
			ssl = new ArrayList<String>();
			for (int i = 0; i < numDoors; i++) {
				ssl.add("door" + i);
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
			SingleAgentLoader sal = new SingleAgentLoader(prism, mainLog, filename + i, modelName, propfilename,
					resLoc, ssl);

			sal.setUp();
			da = sal.getSingleAgentModelGenReturnDA();

			resSaver.recordTime("model loading time " + modelName, varIDs.modelloadingtimes, true);
			resSaver.setLocalStartTime();
			
			sal.solveUsingPartialSatisfaction();
			sal.solutionProdModelVarListsAreSynced();

//			maxStateEstimate *= da.size(); 
//			sal.setMaxStatesEstimate(maxStateEstimate);
			
			sal.cleanUp();

			int maxStateEstimate = sal.getMaxStatesEstimate();
			resSaver.recordTime("Single Agent Solution " + i, varIDs.singleagentsolutiontimes, true);
			resSaver.recordValues(maxStateEstimate, "Single Agent " + i + " States", varIDs.nestedproductstates);
			
			sals.add(sal);

			
		}
		resSaver.recordTime("all models loading time", varIDs.totalmodelloadingtime, false);
		resSaver.recordTime("Total Single Agent Solution Time", varIDs.allnestedproductcreationtime, false);
//		resSaver.setLocalStartTime();

//		for (int i = 0; i < sals.size(); i++) {
//			resSaver.setScopeStartTime();
//			SingleAgentLoader sal = sals.get(i);
//			sal.solveUsingPartialSatisfaction();
//			sal.solutionProdModelVarListsAreSynced();
//			sal.cleanUp();
//
//			int maxStateEstimate = sal.getMaxStatesEstimate();
//			resSaver.recordTime("Single Agent Solution " + i, varIDs.singleagentsolutiontimes, true);
//			resSaver.recordValues(maxStateEstimate, "Single Agent " + i + " States", varIDs.nestedproductstates);
//		}
//		resSaver.recordTime("Total Single Agent Solution Time", varIDs.allnestedproductcreationtime, false);
		resSaver.setScopeStartTime();
		long elapsedTime = System.nanoTime() - startTime;
		mainLog.println("Single Agent Models Loaded and Solved " + elapsedTime + "ns "
				+ TimeUnit.MILLISECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS) + " ms "
				+ TimeUnit.SECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS) + "s");

		MultiAgentProductModelGenerator jpmg = new MultiAgentProductModelGenerator(mainLog, sals, da, buildMDP);

		ArrayList<Objectives> tieBreakingOrder = new ArrayList<Objectives>();
		tieBreakingOrder.add(Objectives.Probability);
		tieBreakingOrder.add(Objectives.Progression);
		tieBreakingOrder.add(Objectives.Cost);
		ActionSelection actionSelection = new ActionSelectionGreedy(tieBreakingOrder);
		OutcomeSelection outcomeSelection = new OutcomeSelectionBounds();
		HeuristicFunction heuristicFunction = new HeuristicFunctionPartSat(jpmg);
		BackupFunction backupFunction = new BackupFunctionFullBellman(jpmg, tieBreakingOrder);

		TrialBHeuristicSearch thts = new TrialBHeuristicSearch(mainLog, jpmg, actionSelection, outcomeSelection,
				heuristicFunction, backupFunction, tieBreakingOrder, resLoc, filename, resSaver);
		int mt = thts.setMaxTrialLength();
		resSaver.recordTime("Multi Agent Product Initialization", varIDs.jointmodelgentime, true);
		mainLog.println("Max Trial Length: " + mt);
		Entry<Object, HashMap<String, Double>> res = thts.doTHTS(resSaver.gettrialName());
		Object a = res.getKey();
		resSaver.saveValues(res.getValue());
		if (a != null)
			mainLog.println(a.toString());

		elapsedTime = System.nanoTime() - startTime;
		mainLog.println(
				"Completed in " + elapsedTime + "ns " + TimeUnit.MILLISECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS)
						+ " ms " + TimeUnit.SECONDS.convert(elapsedTime, TimeUnit.NANOSECONDS) + "s");
		resSaver.writeResults();
	}

	public void runTest() {
///r2/t10/fs0/d3/grid_3_topomap1.prism
		// saving filenames etc
		String dir = System.getProperty("user.dir");
		String modelLocation = dir + "/tests/decomp_tests/";
		modelLocation = dir + "/tests/autogen_testfiles/";

		String path = modelLocation;
		// goalsRange = [2,4,6,8,10]
		// agentsRange = [2,4,6,8,10]
		// fsRange = [0,2,4,8,16,32]
		// ssRange = [0,1,2,3,4]
		ArrayList<String> modelnames = new ArrayList<String>();

		String modelname = "grid_3_topomap";
		modelnames.add(modelname);
		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_task_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list = new HashMap<String, Integer>();
		ArrayList<String> allModelLocs = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();
		int[] goalsRange = { 2, 4, 6, 8/* , 10 */ };
		int[] agentsRange = { 2, 4, 6, 8, 10 };
		int[] fsRange = { 0, 2, 4, 8, 16, 32 };
		int[] ssRange = { 0, 1, 2, 3, 4 };
		for (int numAgents : agentsRange) {
			for (int numGoals : goalsRange) {
				for (int numFailStates : fsRange) {
					for (int numDoors : ssRange) {
						String thisModelLoc = modelLocation + modelname + "/" + "r" + numAgents + "/t" + (numGoals)
								+ "/fs" + (numFailStates) + "/d" + (numDoors) + "/";
						allModelLocs.add(thisModelLoc);
						String example_id = modelname + "_r" + numAgents + "_t" + (numGoals) + "_fs" + (numFailStates)
								+ "_d" + numDoors;
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
			for (int i = 219; i < allModelLocs.size(); i++) {

				String thisModelLoc = allModelLocs.get(i);
				String example_name = modelname;
				String example_id = example_ids.get(i);
				System.out.println(thisModelLoc);
				System.out.println(example_id);
				modelsTested.add(example_id);
//				StatesHelper.setSavePlace(thisModelLoc + "results/");
				System.out.println("Example " + i + " of " + allModelLocs.size());
//				if(true)
//				throw new FileNotFoundException();
//				runOneExample(example_name, example_id, example_has_door_list, example_num_door_list, example_num_robot_list, thisModelLoc, true);
//				if(true)
				doTHTS(thisModelLoc + "results/", thisModelLoc, example_name, example_num_robot_list.get(example_id),
						example_num_task_list.get(example_id), example_num_door_list.get(example_id),
						example_num_fs_list.get(example_id));

				System.out.println("Example " + i + " of " + allModelLocs.size() + " completed");
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

	public void runGUISimpleTests() throws IOException {
		// saving filenames etc
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		// System.getProperty("user.dir");
	
	
		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_goals_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list = new HashMap<String, Integer>();
		ArrayList<String> examples = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();

		int numRobots = 3;
		int numFS = 0;
		int numGoals = 3;
		int numDoors = 0;
		// simpleTests/g5_r3_t3_d0_fs0.png simpleTests/g5_r3_t3_d0_fs3.png
		// simpleTests/g5_r3_t3_d2_fs3.png
		String example = "g5_r3_t3_d0_fs0";// "test_grid_nodoors_nofs";
		String example_id = example;// example + "r" + numRobots;//cumberland_doors;
		String example_to_run = example;// cumberland_doors;
//		g7_r5_t6_d0_fs3.prop 
		
		numRobots = 5;
		numGoals = 6;
		numDoors = 0;
		numFS = 3;
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d0_fs3";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

// g7_r5_t6_d2_fs2.prop  
		numRobots = 5;
		numGoals = 6;
		numDoors = 2;
		numFS = 2;
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d2_fs2";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);
//g7_r5_t6_d3_fs2.prop  g7_r5_t6_d4_fs2.prop
//		g5_r3_t3_d0_fs3.prop  g7_r5_t6_d0_fs5.prop  g7_r5_t6_d2_fs3.prop  g7_r5_t6_d3_fs3.prop  g7_r5_t6_d4_fs3.prop
//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 3;
		numFS = 2;

		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d3_fs2";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

		// g7_r5_t6_d4_fs2.prop
//		g5_r3_t3_d0_fs3.prop  g7_r5_t6_d0_fs5.prop  g7_r5_t6_d2_fs3.prop  g7_r5_t6_d3_fs3.prop  g7_r5_t6_d4_fs3.prop
//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 4;
		numFS = 2;

		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d4_fs2";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

//		g5_r3_t3_d0_fs3.prop  g7_r5_t6_d0_fs5.prop  g7_r5_t6_d2_fs3.prop  g7_r5_t6_d3_fs3.prop  g7_r5_t6_d4_fs3.prop
//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 3;
		numGoals = 3;
		numDoors = 0;
		numFS = 2;

		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g5_r3_t3_d0_fs3";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);


//		g7_r5_t6_d0_fs5.prop  g7_r5_t6_d2_fs3.prop  g7_r5_t6_d3_fs3.prop  g7_r5_t6_d4_fs3.prop
//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 0;
		numFS = 5;

		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d0_fs5";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);


		//g7_r5_t6_d2_fs3.prop  g7_r5_t6_d3_fs3.prop  g7_r5_t6_d4_fs3.prop
//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 2;
		numFS = 3;

		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d2_fs3";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

// g7_r5_t6_d3_fs3.prop  g7_r5_t6_d4_fs3.prop
//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 3;
		numFS = 3;
		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d3_fs3";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

		//g7_r5_t6_d4_fs3.prop
//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 4;
		numFS = 3;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d4_fs3";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);


//		g5_r3_t3_d2_fs3.prop  g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 3;
		numDoors = 2;
		numFS = 3;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g5_r3_t3_d2_fs3";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);


		//g7_r5_t6_d1_fs1.prop  g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 1;
		numFS = 1;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d1_fs1";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);


//g7_r5_t6_d2_fs4.prop  g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 2;
		numFS = 4;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d2_fs4";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

//g7_r5_t6_d3_fs4.prop  g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 3;
		numFS = 4;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d3_fs4";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);
		
//g7_r5_t6_d4_fs4.prop
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 4;
		numFS = 4;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d4_fs4";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);
		
//		g7_r5_t6_d0_fs1.prop  g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 0;
		numFS = 1;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d0_fs1";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);
		
//g7_r5_t6_d2_fs1.prop  g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 2;
		numFS = 1;

		
		// simpleTests/g5_r3_t3_d0_fs3.png simpleTests/g5_r3_t3_d2_fs3.png
		example = "g7_r5_t6_d2_fs1";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

		// g7_r5_t6_d3_fs1.prop  g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 3;
		numFS = 1;

		

		example = "g7_r5_t6_d3_fs1";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

//g7_r5_t6_d4_fs1.prop

		numRobots = 5;
		numGoals = 6;
		numDoors = 4;
		numFS = 1;

		

		example = "g7_r5_t6_d4_fs1";// "test_grid_nodoors_nofs";
		example_id = example;// example + "r" + numRobots;//cumberland_doors;
		example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);
		
		int maxGoals = 9;
		ArrayList<String> testsDone = new ArrayList<String>();
		int maxFiles = examples.size() * 3 * 4;
		int testCount = 0;
		try {
			for (int i = 0; i < examples.size(); i++) {
				example_to_run = examples.get(i);
				example_id = example_ids.get(i);

				int maxRobots = example_num_robot_list.get(example_id);
				maxGoals = example_num_goals_list.get(example_id)+1;
				for (int r = 2; r <= maxRobots; r += 2) {
					for (int g = 2; g <= maxGoals; g += 2) {

						System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" + example_id + " r" + r + " g" + g
								+ ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
						System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" + testCount + " of " + maxFiles + " : "
								+ ((double) (testCount + 1) / (double) maxFiles)
								+ ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
						reWritePropsFileGUISimpleTests(dir, example_to_run, numGoals);
						doTHTS(dir + "results/thts", dir, example_to_run, r, g, example_num_door_list.get(example_id),
								example_num_fs_list.get(example_id));

						testCount++;
						testsDone.add(example_id);
						System.out.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" + example_id + " r" + r + " g" + g
								+ "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
					}
				}
			}

		} catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}

		System.out.println("Num tests: " + testCount);
	}
	public void runDebugOne() throws IOException {
		// saving filenames etc
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles/";
		// System.getProperty("user.dir");
	
	
		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_goals_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list = new HashMap<String, Integer>();
		ArrayList<String> examples = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();

		int numRobots = 2;
		int numFS = 0;
		int numGoals = 2;
		int numDoors = 0;
		// simpleTests/g5_r3_t3_d0_fs0.png simpleTests/g5_r3_t3_d0_fs3.png
		// simpleTests/g5_r3_t3_d2_fs3.png
		String example = "simple2_robot";// "test_grid_nodoors_nofs";
		String example_id = example;// example + "r" + numRobots;//cumberland_doors;
		String example_to_run = example;// cumberland_doors;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);
		
		examples.add(example_id);
		example_ids.add(example);



		int maxGoals = 9;
		ArrayList<String> testsDone = new ArrayList<String>();
		int maxFiles = examples.size() * 3 * 4;
		int testCount = 0;
		try {
			for (int i = 0; i < examples.size(); i++) {
				example_to_run = examples.get(i);
				example_id = example_ids.get(i);

				int maxRobots = example_num_robot_list.get(example_id);
				maxGoals = example_num_goals_list.get(example_id);
				for (int r = 2; r <= maxRobots; r += 2) {
					for (int g = 2; g <= maxGoals; g += 2) {

						System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" + example_id + " r" + r + " g" + g
								+ ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
						System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" + testCount + " of " + maxFiles + " : "
								+ ((double) (testCount + 1) / (double) maxFiles)
								+ ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
//						reWritePropsFileGUISimpleTests(dir, example_to_run, numGoals);
						doTHTS(dir + "results/thts", dir, example_to_run, r, g, example_num_door_list.get(example_id),
								example_num_fs_list.get(example_id));

						testCount++;
						testsDone.add(example_id);
						System.out.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" + example_id + " r" + r + " g" + g
								+ "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
					}
				}
			}

		} catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}

		System.out.println("Num tests: " + testCount);
	}

	public void runGUITest() throws IOException {
		// saving filenames etc
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/guiFiles/";
		// System.getProperty("user.dir");

		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list = new HashMap<String, Integer>();
		ArrayList<String> examples = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();

		int numRobots = 4;
		int numFS = 0;
		int numGoals = 4;
		int numDoors = 0;
		String example;
		String example_id;

//
//		numRobots = 1;
//		numFS = 0;
//		numGoals = 4;
//		numDoors = 0;
//		example = "3gridsimple";
//		example_id = example;
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//		
//		numRobots = 1;
//		numFS = 0;
//		numGoals = 4;
//		numDoors = 0;
//		example = "simple";
//		example_id = "simple";
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		numRobots = 2;
//		numFS = 0;
//		numGoals = 4;
//		numDoors = 0;
//		example = "simple2";
//		example_id = "simple2";
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		numRobots = 1;
//		numFS = 0;
//		numGoals = 4;
//		numDoors = 0;
//		example = "simple_avoid";
//		example_id = "simple_avoid";
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		numRobots = 2;
//		numFS = 0;
//		numGoals = 4;
//		numDoors = 0;
//		example = "simple2_avoid";
//		example_id = "simple2_avoid";
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		numRobots = 2;
//		numFS = 0;
//		numGoals = 4;
//		numDoors = 0;
//		example = "simple2_avoid2";
//		example_id = "simple2_avoid2";
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		numRobots = 2;
//		numFS = 2;
//		numGoals = 4;
//		numDoors = 0;
//		example = "simple2_avoid2_fs2";
//		example_id = "simple2_avoid2_fs2";
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		numRobots = 2;
//		numFS = 2;
//		numGoals = 4;
//		numDoors = 0;
//		example = "simple2_avoid2_fs2_nofsatgoal";
//		example_id = "simple2_avoid2_fs2_nofsatgoal";
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		//square5_avoid2_fs1
//
//		numRobots = 2;
//		numFS = 1;
//		numGoals = 4;
//		numDoors = 0;
//		example = "square5_avoid2_fs1";
//		example_id = example;
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//
//		numRobots = 2;
//		numFS = 3;
//		numGoals = 4;
//		numDoors = 0;
//		example = "square5_avoid2_fs3";
//		example_id = example;
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
//

		// office_0fs_2doors.prop

//		numRobots = 6;
//		numFS = 0;
//		numGoals = 8;
//		numDoors = 2;
//		example = "office_0fs_2doors";
//		example_id = example;
//
//		example_has_door_list.put(example_id, numDoors > 0);
//		example_num_door_list.put(example_id, numDoors);
//		example_num_robot_list.put(example_id, numRobots);
//		example_num_fs_list.put(example_id, numFS);
//		examples.add(example_id);
//		example_ids.add(example);
		// office_0fs_5doors.prop office_4fs_nodoors.prop office_8fs_1door.prop
		// office_0fs_3doors.prop office_0fs_6doors.prop office_6fs_1door.prop
		// office_8fs_nodoors.prop
		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 0;
		numGoals = 8;
		numDoors = 5;
		example = "office_0fs_5doors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);
		// office_4fs_nodoors.prop office_8fs_1door.prop
		// office_0fs_3doors.prop office_0fs_6doors.prop office_6fs_1door.prop
		// office_8fs_nodoors.prop
		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 4;
		numGoals = 8;
		numDoors = 0;
		example = "office_4fs_nodoors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_8fs_1door.prop
		// office_0fs_3doors.prop office_0fs_6doors.prop office_6fs_1door.prop
		// office_8fs_nodoors.prop
		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 8;
		numGoals = 8;
		numDoors = 1;
		example = "office_8fs_1door";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_0fs_3doors.prop office_0fs_6doors.prop office_6fs_1door.prop
		// office_8fs_nodoors.prop
		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 0;
		numGoals = 8;
		numDoors = 3;
		example = "office_0fs_3doors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_0fs_6doors.prop office_6fs_1door.prop office_8fs_nodoors.prop
		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 0;
		numGoals = 8;
		numDoors = 6;
		example = "office_0fs_6doors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_6fs_1door.prop office_8fs_nodoors.prop
		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 6;
		numGoals = 8;
		numDoors = 1;
		example = "office_6fs_1door";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_8fs_nodoors.prop
		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 8;
		numGoals = 8;
		numDoors = 0;
		example = "office_8fs_nodoors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_0fs_4doors.prop office_2fs_nodoors.prop office_6fs_nodoors.prop
		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 0;
		numGoals = 8;
		numDoors = 4;
		example = "office_0fs_4doors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_2fs_nodoors.prop office_6fs_nodoors.prop office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 2;
		numGoals = 8;
		numDoors = 0;
		example = "office_2fs_nodoors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_6fs_nodoors.prop office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 6;
		numGoals = 8;
		numDoors = 0;
		example = "office_6fs_nodoors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		// office_nofs_nodoors.prop
		//
		numRobots = 6;
		numFS = 0;
		numGoals = 8;
		numDoors = 0;
		example = "office_nofs_nodoors";
		example_id = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		examples.add(example_id);
		example_ids.add(example);

		String example_to_run;

		int maxGoals = 9;
		ArrayList<String> testsDone = new ArrayList<String>();
		int testCount = 0;
		int maxFiles = examples.size() * 3 * 4;
		try {
			for (int i = 0; i < examples.size(); i++) {
				example_to_run = examples.get(i);
				example_id = example_ids.get(i);

				int maxRobots = example_num_robot_list.get(example_id);
//				int g = 4; 
//				int r = maxRobots; 
				for (int r = 2; r <= maxRobots; r += 2) {
					for (int g = 2; g <= maxGoals; g += 2) {

						System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" + example_id + " r" + r + " g" + g
								+ ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
						System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" + testCount + " of " + maxFiles + " : "
								+ ((double) (testCount + 1) / (double) maxFiles)
								+ ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
						reWritePropsFile(dir + example_id + ".props", g);
						doTHTS(dir + "results/thts", dir, example_to_run, r, g, example_num_door_list.get(example_id),
								example_num_fs_list.get(example_id));

						testCount++;
						testsDone.add(example_id);
						System.out.println("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" + example_id + " r" + r + " g" + g
								+ "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
					}
				}
			}

		} catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}

		// mainLog.println("Num tests: " + testCount);
	}

	public void reWritePropsFileGUISimpleTests(String loc, String fn, int numGoals) throws IOException {

		HashMap<String, HashMap<Integer, String>> goalStrings = new HashMap<String, HashMap<Integer, String>>();
		HashMap<Integer, String> thisHashMap = new HashMap<Integer, String>();
		String f = "g5_r3_t3_d2_fs3";
		String gs = "Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]";
		thisHashMap.put(2, "Pmax=? [ (F (s=0) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");

		goalStrings.put(f, thisHashMap);
		

//		g5_r3_t3_d0_fs0.props
//		Pmax=? [ (F (s=0) ) & (F (s=2) ) & (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]
		f = "g5_r3_t3_d0_fs0";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2, 
				"Pmax=? [ (F (s=0) )  & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g5_r3_t3_d0_fs3.props
//		Pmax=? [ (F (s=0) ) & (F (s=2) ) & (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]
		f = "g5_r3_t3_d0_fs3";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2, 
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g5_r3_t3_d2_fs3.props
//		Pmax=? [ (F (s=0) ) & (F (s=2) ) & (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]
		f = "g5_r3_t3_d2_fs3";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2, 
				"Pmax=? [ (F (s=0) )  & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) ) & (F (s=2) ) & (F (s=4) ) & (G ( ! (s=11) ) )  & (G ( ! (s=14) ) )  & (G ( ! (s=17) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d0_fs1.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d0_fs1";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);
		//		g7_r5_t6_d0_fs3.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d0_fs3";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d0_fs5.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d0_fs5";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d1_fs1.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d1_fs1";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d2_fs1.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d2_fs1";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d2_fs2.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d2_fs2";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d2_fs3.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d2_fs3";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d2_fs4.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d2_fs4";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d3_fs1.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d3_fs1";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d3_fs2.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d3_fs2";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d3_fs3.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d3_fs3";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d3_fs4.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d3_fs4";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d4_fs1.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d4_fs1";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d4_fs2.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d4_fs2";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d4_fs3.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d4_fs3";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		//		g7_r5_t6_d4_fs4.props
//		Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]
		f = "g7_r5_t6_d4_fs4";
		thisHashMap = new HashMap<Integer, String>();
		thisHashMap.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=4) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		thisHashMap.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");

		thisHashMap.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=4) )& (F (s=6) )& (F (s=27) )& (F (s=28) )& (F (s=30) ) & (G ( ! (s=14) ) )  & (G ( ! (s=16) ) ) ]");
		goalStrings.put(f, thisHashMap);

		
		
		

		String goalString = goalStrings.get(fn).get(numGoals);

		System.out.println(goalString);

		FileWriter fileWriter = new FileWriter(loc + fn + ".props");
		PrintWriter printWriter = new PrintWriter(fileWriter);
		printWriter.print(goalString + "\n");

		printWriter.close();
	}

	public void reWritePropsFile(String fn, int numGoals) throws IOException {
		HashMap<Integer, String> goalStrings = new HashMap<Integer, String>();
		goalStrings.put(13,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) )& (F (s=10) )& (F (s=84) )& (F (s=85) )& (F (s=86) )& (F (s=87) )& (F (s=88) )& (F (s=89) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(12,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) )& (F (s=10) )& (F (s=84) )& (F (s=85) )& (F (s=86) )& (F (s=87) )& (F (s=88) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(11,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) )& (F (s=10) )& (F (s=84) )& (F (s=85) )& (F (s=86) )& (F (s=87) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(10,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) )& (F (s=10) )& (F (s=84) )& (F (s=85) )& (F (s=86) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(9,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) )& (F (s=10) )& (F (s=84) )& (F (s=85) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(8,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) )& (F (s=10) )& (F (s=84) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(7,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) )& (F (s=10) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(6,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) )& (F (s=8) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(5,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) )& (F (s=6) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(4,
				"Pmax=? [ (F (s=0) )& (F (s=2) )& (F (s=4) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(3,
				"Pmax=? [ (F (s=0) )& (F (s=2) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");
		goalStrings.put(2,
				"Pmax=? [ (F (s=0) ) & (G ( ! (s=19) ) )  & (G ( ! (s=23) ) )  & (G ( ! (s=25) ) )  & (G ( ! (s=29) ) )  & (G ( ! (s=31) ) )  & (G ( ! (s=35) ) )  & (G ( ! (s=59) ) )  & (G ( ! (s=60) ) )  & (G ( ! (s=65) ) )  & (G ( ! (s=66) ) ) ]");

		String goalString = goalStrings.get(numGoals);

		System.out.println(goalString);

		FileWriter fileWriter = new FileWriter(fn);
		PrintWriter printWriter = new PrintWriter(fileWriter);
		printWriter.print(goalString + "\n");

		printWriter.close();
	}
}
