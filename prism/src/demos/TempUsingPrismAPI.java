package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.BitSet;
import java.util.Vector;
import java.util.Map.Entry;

import acceptance.AcceptanceOmega;
import automata.DA;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.StateValues;
import parser.ast.Expression;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.Result;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;

public class TempUsingPrismAPI
{
	Prism prism;
	PrismLog mainLog;
	MDPModelChecker mc;
	MDP mdp;
	DA<BitSet, ? extends AcceptanceOmega> da;
	
	

	public void setMCExportOptionsAll(String saveplace, String filename)
	{
		mc.setExportAdv(true);
		mc.setExportAdvFilename(saveplace + "results/xai_" + filename + ".dot");
		mc.setExportProductVector(true);
		mc.setExportProductVectorFilename(saveplace + "results/xai_" + filename + "_res.tra");
		mc.setGenStrat(true);
		mc.setExportProductStates(true);
		mc.setExportProductTrans(true);
		mc.setExportProductStatesFilename(saveplace + "results/xai_" + filename + ".sta");
		mc.setExportProductTransFilename(saveplace + "results/xai_" + filename + ".tra");
	}
	
	public PropertiesFile readModel(String saveplace, String filename) throws FileNotFoundException, PrismException
	{
		mainLog = new PrismFileLog("stdout");

		// Initialise PRISM engine 
		prism = new Prism(mainLog);

		prism.initialise();
		prism.setEngine(Prism.EXPLICIT);

		//load the mdp for a single model 

		String modelFileName = saveplace + filename + ".prism";
		ModulesFile modulesFile = prism.parseModelFile(new File(modelFileName));
		prism.loadPRISMModel(modulesFile);
		PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + ".props"));
		prism.buildModel();
		mdp = (MDP) prism.getBuiltModelExplicit();

		//set up the model checker 
		mc = new MDPModelChecker(prism);
		mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));
		return propertiesFile;

	}
	
	public void doStuff() throws FileNotFoundException, PrismException
	{
		String saveplace = "/home/fatma/Data/PhD/melb/prism/";//"/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		String filename = "domainTemplate";//"g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";

		PropertiesFile propertiesFile = readModel(saveplace, filename);

		ExpressionReward exprRew = (ExpressionReward) propertiesFile.getProperty(0);
		mainLog.println(exprRew.toString());

		setMCExportOptionsAll(saveplace, filename);

		Expression expr = exprRew.getExpression();
		
		Entry<MDP, MDStrategy> prodStratPair = mc.checkPartialSatExprReturnStrategy(mdp, expr, exprRew, null);
		MDStrategy strat = prodStratPair.getValue();
		MDP productMDP = prodStratPair.getKey();
		PolicyCreator pc = new PolicyCreator();
		pc.createPolicy(productMDP, strat);
		pc.savePolicy(saveplace + "results/", "xai_" + filename + "_policy");

		
//		StateValues res = mc.checkExpression(mdp, exprRew, null);
//		Result result = mc.getResult();
////		Result result = mc.check(mdp, propertiesFile.getProperty(0));
//		// mc.computeReachProbs(mdp, target, min)
//		System.out.println(result.getResult());
	}
	public void run()
	{
		try {
			doStuff();
		} catch (FileNotFoundException | PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	//main function 
		public static void main(String[] args)
		{
			new TempUsingPrismAPI().run();
		}
}
