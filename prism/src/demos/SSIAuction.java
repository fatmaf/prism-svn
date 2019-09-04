package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Map.Entry;

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
import strat.Strategy;

public class SSIAuction
{

	public Entry<Integer, Double> getSingleAgentBid(MDP agentMDP, ArrayList<Expression> taskSet, Expression agentTasks, ExpressionReward rewardExpr,
			MDPModelChecker mc) throws PrismException
	{
		Expression bidPick = null;
		double bidValue = 100000;
		//		Expression robotTasks = null;
		Expression currentTask = null;
		int indexOfChosenTask = -1;

		for (int exprNum = 0; exprNum < taskSet.size(); exprNum++) {
			currentTask = taskSet.get(exprNum);
			if (agentTasks != null)
				currentTask = Expression.And(agentTasks, currentTask);
			StateValues nvicosts = mc.checkPartialSatExposed(agentMDP, currentTask, rewardExpr, null);
			double costInInitState = nvicosts.getDoubleArray()[agentMDP.getFirstInitialState()];
			if (costInInitState < bidValue) {
				bidPick = taskSet.get(exprNum);
				bidValue = costInInitState;
				indexOfChosenTask = exprNum;
			}

		}
		return new AbstractMap.SimpleEntry<Integer, Double>(indexOfChosenTask, bidValue);
	}

	public Entry<ExpressionReward, ArrayList<Expression>> processProperties(PropertiesFile propertiesFile, PrismLog mainLog)
	{
		// Model check the first property from the file using the model checker
		for (int i = 0; i < propertiesFile.getNumProperties(); i++)
			mainLog.println(propertiesFile.getProperty(i));
		ExpressionFunc expr = (ExpressionFunc) propertiesFile.getProperty(0);
		int numOp = expr.getNumOperands();
		ArrayList<Expression> ltlExpressions = new ArrayList<Expression>(numOp);

		ExpressionReward rewExpr = null;
		for (int exprNum = 0; exprNum < numOp; exprNum++) {
			ltlExpressions.add((expr.getOperand(exprNum)));
			//full expression 
			mainLog.println(ltlExpressions.get(exprNum));
			if (ltlExpressions.get(exprNum) instanceof ExpressionReward)
				rewExpr = (ExpressionReward) ltlExpressions.get(exprNum);

		}
		Expression safetyExpr = ((ExpressionQuant) ltlExpressions.get(numOp - 1)).getExpression();
		ArrayList<Expression> taskSet = new ArrayList<Expression>();

		for (int exprNum = 0; exprNum < numOp - 1; exprNum++) {

			Expression currentExpr = ((ExpressionQuant) ltlExpressions.get(exprNum)).getExpression();
			Expression currentExprWithSafetyExpr = Expression.And(currentExpr, safetyExpr);
			taskSet.add(currentExprWithSafetyExpr);
		}
		return new AbstractMap.SimpleEntry<ExpressionReward, ArrayList<Expression>>(rewExpr, taskSet);
	}

	public void run()
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
			prism.setEngine(Prism.EXPLICIT);

			int numRobots = 2; //numRobots

			ArrayList<MDP> mdps = new ArrayList<MDP>();
			ArrayList<MDPModelChecker> mcs = new ArrayList<MDPModelChecker>();
			PropertiesFile propertiesFile = null;

			//load the files 
			//could be a seprate function 
			for (int i = 0; i < numRobots; i++) {
				String modelFileName = saveplace + filename + i + ".prism";
				ModulesFile modulesFile = prism.parseModelFile(new File(modelFileName));
				prism.loadPRISMModel(modulesFile);
				propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + ".prop"));
				prism.buildModel();
				MDP mdp = (MDP) prism.getBuiltModelExplicit();
				mdps.add(mdp);
				MDPModelChecker mc = new MDPModelChecker(prism);
				mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));
				mcs.add(mc);
			}

			//do things to the properties 

			ArrayList<Expression> robotsTasks = new ArrayList<Expression>();
			for (int i = 0; i < numRobots; i++)
				robotsTasks.add(null);
			Entry<ExpressionReward, ArrayList<Expression>> processedProperties = processProperties(propertiesFile, mainLog);
			ExpressionReward rewExpr = processedProperties.getKey();

			ArrayList<Expression> taskSet = processedProperties.getValue();

			while (!taskSet.isEmpty()) {

				int bestBidIndex = -1;
				double bestBidValue = 10000;
				Expression bestBidExpression = null;
				int bestBidRobotIndex = -1;

				for (int i = 0; i < numRobots; i++) {
					Entry<Integer, Double> bid = getSingleAgentBid(mdps.get(i), taskSet, robotsTasks.get(i), rewExpr, mcs.get(i));
					int bidTaskIndex = bid.getKey();
					double bidValue = bid.getValue();

					if (bidValue < bestBidValue) {
						bestBidValue = bidValue;
						bestBidIndex = bidTaskIndex;
						bestBidRobotIndex = i;
						bestBidExpression = taskSet.get(bestBidIndex);
					}
				}
				Expression robotTasks = robotsTasks.get(bestBidRobotIndex);
				if (robotTasks != null) {
					bestBidExpression = Expression.And(robotTasks, bestBidExpression);

				}
				robotsTasks.set(bestBidRobotIndex, bestBidExpression);
				taskSet.remove(bestBidIndex);

			}

			//print task distribution and get strategy 
			ArrayList<Strategy> nviStrategies = new ArrayList<Strategy>();
			mainLog.println("Assigned Tasks");
			for (int i = 0; i < numRobots; i++) {
				mainLog.println(i + ":" + robotsTasks.get(i).toString());
				mcs.get(i).setGenStrat(true);
				Strategy nviStrategy = mcs.get(i).checkPartialSatExprReturnStrategy(mdps.get(i), robotsTasks.get(i), 
						rewExpr, null);
				
				nviStrategies.add(nviStrategy);
			}

			
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void main(String[] args)
	{

		new SSIAuction().run();
	}
}
