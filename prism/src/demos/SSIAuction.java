package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;

import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.ModelCheckerResult;
import explicit.StateValues;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.Result;
import simulator.ModulesFileModelGenerator;

public class SSIAuction
{

	public static void main(String[] args)
	{
		try {
			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
			String filename = "g7x3_r2_t3_d0_fs1";//"robot";
			// Create a log for PRISM output (hidden or stdout)
			//PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);

			prism.initialise();

			// Parse and load a PRISM model from a file
			ModulesFile modulesFile = prism.parseModelFile(new File(saveplace + filename + "0.prism"));
			prism.loadPRISMModel(modulesFile);

			// Parse and load a properties model for the model
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + ".prop"));

			// Get PRISM to build the model and then extract it
			prism.setEngine(Prism.EXPLICIT);
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit();

			// Model check the first property from the file using the model checker
			for (int i = 0; i < propertiesFile.getNumProperties(); i++)
				mainLog.println(propertiesFile.getProperty(i));
			ExpressionFunc expr = (ExpressionFunc) propertiesFile.getProperty(0);
			int numOp = expr.getNumOperands();
			ArrayList<Expression> ltlExpressions = new ArrayList<Expression>(numOp);
			MDPModelChecker mc = new MDPModelChecker(prism);
//			mc.setConstantValues(propertiesFile.getConstantValues());
			mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));
			
			
		
			
			ExpressionReward rewExpr = null; 
			for (int exprNum = 0; exprNum < numOp; exprNum++) {
				ltlExpressions.add((expr.getOperand(exprNum)));
				//full expression 
				mainLog.println(ltlExpressions.get(exprNum));
				if(ltlExpressions.get(exprNum) instanceof ExpressionReward)
					rewExpr = (ExpressionReward) ltlExpressions.get(exprNum);
				//just the formula 
				boolean min = false;
				mainLog.println(((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression());
				Result result = mc.check(mdp, ltlExpressions.get(exprNum));
				// mc.computeReachProbs(mdp, target, min)
				System.out.println(result.getResult());
//				StateValues meh = mc.checkPartialSatExposed(mdp,  ltlExpressions.get(exprNum), null);
				StateValues meh = mc.checkPartialSatExposed(mdp, 
						((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression(),rewExpr, null);
//				BitSet statesGoal = mc.checkExpression(mdp, ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression(), null).getBitSet();
//				ModelCheckerResult result = mc.computeReachProbs(mdp, statesGoal, min);
//				double probsGoal[] = result.soln;
//				System.out.println(Arrays.toString(probsGoal));
			}


		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
