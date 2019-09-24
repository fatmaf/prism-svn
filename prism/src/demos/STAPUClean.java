package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import demos.ResultsTiming.varIDs;
import explicit.MDP;
import explicit.Model;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;

public class STAPUClean
{
	public static void main(String[] args)
	{

		STAPUClean stapu = new STAPUClean();
		stapu.runGUISimpleTestsOne();
	}

	private PrismLog mainLog;
	private Prism prism;

	public double[] runGUISimpleTestsOne()
	{
		double[] res = null;
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		//System.getProperty("user.dir");
		String modelLocation = dir;
		String resLoc = dir + "results/";
		HashMap<String, Boolean> example_has_door_list = new HashMap<String, Boolean>();
		HashMap<String, Integer> example_num_door_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_robot_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_goals_list = new HashMap<String, Integer>();
		HashMap<String, Integer> example_num_fs_list = new HashMap<String, Integer>();
		ArrayList<String> examples = new ArrayList<String>();
		ArrayList<String> example_ids = new ArrayList<String>();

		int numRobots = 2;
		int numFS = 2;//5;//1;
		int numGoals = 3;//6;//4;
		int numDoors = 0;//2;

		String example =  "g4_r2_t3_d0_fs2";//"g5_r2_t5_d0_fs5";//"g5_r2_t3_d2_fs1";
		String example_id = example;
		String example_to_run = example;

		example_has_door_list.put(example_id, numDoors > 0);
		example_num_door_list.put(example_id, numDoors);
		example_num_robot_list.put(example_id, numRobots);
		example_num_fs_list.put(example_id, numFS);
		example_num_goals_list.put(example_id, numGoals);

		examples.add(example_id);
		example_ids.add(example);

		//		numRobots = 4;
		//		numFS = 8;
		//		numGoals = 7;
		//		numDoors = 4;
		//		//simpleTests/g5_r3_t3_d0_fs0.png  simpleTests/g5_r3_t3_d0_fs3.png  simpleTests/g5_r3_t3_d2_fs3.png
		//		example = "g10_r4_t6_d4_fs8";//"g5_r3_t3_d0_fs0";//"test_grid_nodoors_nofs";
		//		example_id = example;//example + "r" + numRobots;//cumberland_doors; 
		//		example_to_run = example;//cumberland_doors; 
		//
		//		example_has_door_list.put(example_id, numDoors > 0);
		//		example_num_door_list.put(example_id, numDoors);
		//		example_num_robot_list.put(example_id, numRobots);
		//		example_num_fs_list.put(example_id, numFS);
		//		example_num_goals_list.put(example_id, numGoals);
		//
		//		examples.add(example_id);
		//		example_ids.add(example);

		for (int i = 0; i < examples.size(); i++) {
			example_to_run = examples.get(i);
			example_id = example_ids.get(i);

			int maxRobots = example_num_robot_list.get(example_id);
			int maxGoals = example_num_goals_list.get(example_id);
			int maxFS = example_num_fs_list.get(example_id);
			int maxDoors = example_num_door_list.get(example_id);

			try {

				res = runSTAPUScenarioDoors(modelLocation, example_to_run, maxRobots, maxDoors, maxFS, maxGoals, resLoc, false);

			} catch (Exception e) {
				e.printStackTrace();
			}

		}
		return res;

	}

	void initialisePrism() throws PrismException
	{
		// Create a log for PRISM output (hidden or stdout)
		// PrismLog mainLog = new PrismDevNullLog();
		PrismLog mainLog = new PrismFileLog("stdout");
		this.mainLog = mainLog;
		StatesHelper.mainLog = mainLog;

		// Initialise PRISM engine
		Prism prism = new Prism(mainLog);
		this.prism = prism;
		prism.initialise();
		prism.setEngine(Prism.EXPLICIT);

	}

	public double[] runSTAPUScenarioDoors(String modelLocation, String exampleName, int numRobots, int numDoors, int numFS, int numGoals,
			String resultsLocation, boolean doReallocations) throws Exception
	{
		double[] res = null;

		//Setting up
		String filename = exampleName;

		//these affect the switch transitions and hence the solution 
		boolean includefailstatesinswitches = false;
		boolean matchsharedstatesinswitch = false;
		boolean completeSwitchRing = false;

		ArrayList<String> sharedVarsList = new ArrayList<String>();
		for (int door = 0; door < numDoors; door++) {
			sharedVarsList.add("door" + door);
		}
		ArrayList<String> filenames = new ArrayList<String>();

		for (int i = 0; i < numRobots; i++)
			filenames.add(filename + i);

		ArrayList<Model> models = new ArrayList<Model>();
		ArrayList<PropertiesFile> propFiles = new ArrayList<PropertiesFile>();
		ArrayList<ModulesFile> modulesFiles = new ArrayList<ModulesFile>();

		initialisePrism();
		////profile
		long startTime = System.currentTimeMillis();

		int maxMDPVars = loadPrismFiles(modelLocation, filename, models, propFiles, modulesFiles, filenames);

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		System.out.println("\nLoading all modles took: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
		//end profiling

		Expression expr = propFiles.get(0).getProperty(0);
		mainLog.println(expr.toString());

		res = doSTAPU(models, (ExpressionFunc) expr, null, prism, modulesFiles, propFiles, sharedVarsList, includefailstatesinswitches,
				matchsharedstatesinswitch, completeSwitchRing, numGoals, doReallocations, resultsLocation, filename);
		return res;

	}

	private ArrayList<AutomatonInformation> processLTLExpressions(ArrayList<Expression> ltlExpressions)
	{
		ArrayList<AutomatonInformation> automatonList = new ArrayList<AutomatonInformation>();
		AutomatonInformation automatonInfo;
		for (int exprNum = 0; exprNum < ltlExpressions.size(); exprNum++) {
			automatonInfo = new AutomatonInformation(ltlExpressions.get(exprNum));
			automatonList.add(automatonInfo);
		}
		return automatonList;
	}

	private double[] doSTAPU(ArrayList<Model> models, ExpressionFunc expr, Object object, Prism prism2, ArrayList<ModulesFile> modulesFiles,
			ArrayList<PropertiesFile> propertiesFiles, ArrayList<String> sharedVarsList, boolean includefailstatesinswitches, boolean matchsharedstatesinswitch,
			boolean completeSwitchRing, int numGoals, boolean doReallocations, String resLoc, String filename) throws Exception
	{
		boolean allStatesInDFA = true; //when creating the product model do all states in the dfa 
		int numRobots = models.size();
		////profile
		long startTime = System.currentTimeMillis();

		ArrayList<Expression> ltlExpressions = getLTLExpressionsLimit(expr, numGoals);
		//processExpressions 
		ArrayList<AutomatonInformation> automatonList = processLTLExpressions(ltlExpressions);

		long stopTime = System.currentTimeMillis();
		long runTime = stopTime - startTime;
		System.out.println("\nCreating expression list: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
		//end profiling

		//now we can create a nestedProductModel 
		//I'm just going to rewrite this 

		////profile
		startTime = System.currentTimeMillis();

		int firstRobot = 0;
		//technically there is just one automatonList but lets say we give this to each agent 
		SequentialTeamMDPClean seqTeamMDP = createTeamMDPOnTheFly(numRobots, models, modulesFiles, propertiesFiles, automatonList, allStatesInDFA);

		//		seqTeamMDP.addSwitchesAndSetInitialState(firstRobot, includefailstatesinswitches, completeSwitchRing);
		stopTime = System.currentTimeMillis();
		runTime = stopTime - startTime;
		System.out.println("\nSeqTeamMDP: " + runTime + "ms" + "(" + TimeUnit.SECONDS.convert(runTime, TimeUnit.MILLISECONDS) + "s)\n");
		//end profiling
		seqTeamMDP.printStuff();
		seqTeamMDP.teamMDPCreator.saveMDP(resLoc, filename + "_teammdp");
		seqTeamMDP.teamMDPCreator.saveMDP(seqTeamMDP.teamMDPCreator.mdp, resLoc, filename + "_teammdp_ess", seqTeamMDP.teamMDPCreator.essStates);
		seqTeamMDP.teamMDPCreator.saveMDP(seqTeamMDP.teamMDPCreator.mdp, resLoc, filename + "_teammdp_acc", seqTeamMDP.teamMDPCreator.accStates);
		seqTeamMDP.teamMDPCreator.saveMDP(seqTeamMDP.teamMDPCreator.mdp, resLoc, filename + "_teammdp_avoid", seqTeamMDP.statesToAvoid);

		return null;
	}

	protected ArrayList<SingleAgentNestedProductMDPClean> createSingleAgentNestedProductsHomogenous(int numRobots, ArrayList<Model> models,
			ArrayList<ModulesFile> modulesFiles, ArrayList<PropertiesFile> propertiesFiles, ArrayList<AutomatonInformation> automatonList,
			boolean allStatesInDFA) throws PrismException
	{
		//technically there is just one automatonList but lets say we give this to each agent 
		ArrayList<SingleAgentNestedProductMDPClean> singleAgentNestedPs = new ArrayList<SingleAgentNestedProductMDPClean>();

		int robot = 0;
		//do this for a single robot 
		//and then flag it as something for other robots 
		Model mdp = models.get(robot);
		ModulesFile modulesFile = modulesFiles.get(robot);
		PropertiesFile propertiesFile = propertiesFiles.get(robot);

		SingleAgentNestedProductMDPClean singleAgentNestedProduct = new SingleAgentNestedProductMDPClean(prism, automatonList, modulesFile, propertiesFile, mdp,
				allStatesInDFA);

		singleAgentNestedProduct.setHomogenous(models);
		singleAgentNestedPs.add(singleAgentNestedProduct);

		return singleAgentNestedPs;
	}

	protected SequentialTeamMDPClean createTeamMDPOnTheFly(int numRobots, ArrayList<Model> models, ArrayList<ModulesFile> modulesFiles,
			ArrayList<PropertiesFile> propertiesFiles, ArrayList<AutomatonInformation> automatonList, boolean allStatesInDFA) throws Exception
	{
		//technically there is just one automatonList but lets say we give this to each agent 
		//		ArrayList<SingleAgentNestedProductMDPClean> singleAgentNestedPs = new ArrayList<SingleAgentNestedProductMDPClean>();
		SequentialTeamMDPClean seqTeamMDP = null;
		//for each robot 
		for (int robot = 0; robot < numRobots; robot++) {
			Model mdp = models.get(robot);
			ModulesFile modulesFile = modulesFiles.get(robot);
			PropertiesFile propertiesFile = propertiesFiles.get(robot);

			SingleAgentNestedProductMDPClean singleAgentNestedProduct = new SingleAgentNestedProductMDPClean(prism, automatonList, modulesFile, propertiesFile,
					mdp, allStatesInDFA);
			if (seqTeamMDP == null) {
				seqTeamMDP = new SequentialTeamMDPClean(singleAgentNestedProduct, numRobots);
			} else {
				seqTeamMDP.addRobot(robot, singleAgentNestedProduct);
			}
		}
		return seqTeamMDP;

	}

	protected ArrayList<SingleAgentNestedProductMDPClean> createSingleAgentNestedProducts(int numRobots, ArrayList<Model> models,
			ArrayList<ModulesFile> modulesFiles, ArrayList<PropertiesFile> propertiesFiles, ArrayList<AutomatonInformation> automatonList,
			boolean allStatesInDFA) throws PrismException
	{
		//technically there is just one automatonList but lets say we give this to each agent 
		ArrayList<SingleAgentNestedProductMDPClean> singleAgentNestedPs = new ArrayList<SingleAgentNestedProductMDPClean>();

		//for each robot 
		for (int robot = 0; robot < numRobots; robot++) {
			Model mdp = models.get(robot);
			ModulesFile modulesFile = modulesFiles.get(robot);
			PropertiesFile propertiesFile = propertiesFiles.get(robot);

			SingleAgentNestedProductMDPClean singleAgentNestedProduct = new SingleAgentNestedProductMDPClean(prism, automatonList, modulesFile, propertiesFile,
					mdp, allStatesInDFA);

			singleAgentNestedPs.add(singleAgentNestedProduct);
		}
		return singleAgentNestedPs;
	}

	/**
	 * Return a list of expressions
	 */
	protected ArrayList<Expression> getLTLExpressionsLimit(ExpressionFunc expr, int lim) throws PrismException
	{
		int numOp = expr.getNumOperands();
		String exprString = "";
		ExpressionFunc conjunctionOfExpressions = null;
		ArrayList<Expression> ltlExpressions = new ArrayList<Expression>(numOp);
		for (int exprNum = 0; exprNum < numOp; exprNum++) {
			if (expr.getOperand(exprNum) instanceof ExpressionQuant) {
				ltlExpressions.add((expr.getOperand(exprNum)));
				exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
				if (Expression.isCoSafeLTLSyntactic(expr.getOperand(exprNum))) {
					if (conjunctionOfExpressions == null) {
						conjunctionOfExpressions = new ExpressionFunc(((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString());
					}
				}
			}

		}

		ArrayList<Expression> ltlExpressionsLimited = ltlExpressions;

		if (lim != numOp - 1) {
			//limiting the expressions 
			ltlExpressionsLimited = new ArrayList<Expression>(lim);
			exprString = "";
			for (int exprNum = 0; exprNum < numOp; exprNum++) {
				Expression current = ltlExpressions.get(exprNum);
				if (exprNum < (lim - 1)) {
					exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
					ltlExpressionsLimited.add(current);
				}
				if (exprNum == (numOp - 1)) {
					exprString += ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression().toString();
					ltlExpressionsLimited.add(current);
				}
			}
		}

		mainLog.println("LTL Mission: " + exprString);
		return ltlExpressionsLimited;
	}

	private int loadPrismFiles(String modelLocation, String filename, ArrayList<Model> models, ArrayList<PropertiesFile> propFiles,
			ArrayList<ModulesFile> modulesFiles, ArrayList<String> filenames) throws PrismException, FileNotFoundException
	{
		int maxMDPVars = 0;

		for (int files = 0; files < filenames.size(); files++) {

			// Parse and load a PRISM model from a file
			ModulesFile modulesFile = prism.parseModelFile(new File(modelLocation + filenames.get(files) + ".prism"));
			modulesFiles.add(modulesFile);
			prism.loadPRISMModel(modulesFile);

			// Parse and load a properties model for the model
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(modelLocation + filename + ".prop"));

			// Get PRISM to build the model and then extract it

			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit();
			int numVars = mdp.getVarList().getNumVars();
			if (numVars > maxMDPVars)
				maxMDPVars = numVars;

			models.add(mdp);
			propFiles.add(propertiesFile);

		}
		return maxMDPVars;

	}
}
