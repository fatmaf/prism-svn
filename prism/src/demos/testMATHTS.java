package demos;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;

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

	public void run() throws FileNotFoundException, PrismException
	{
		loadSingleAgent();
	}

	public void loadSingleAgent() throws PrismException, FileNotFoundException
	{

		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
		String filename = "tiny_example_permtrap_noun";//"no_door_example";	
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
		ssl.add("s");
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
		MultiAgentProductModelGenerator jpmg = 
				new MultiAgentProductModelGenerator(mainLog,sals,da); 
		

	}
}
