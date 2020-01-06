package demos;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Vector;
import java.util.Map.Entry;

import explicit.MDP;
import explicit.ModelCheckerPartialSatResult;
import parser.State;
import parser.VarList;
import parser.ast.Expression;
import parser.ast.ExpressionReward;
import parser.ast.PropertiesFile;
import prism.PrismException;
import prism.PrismFileLog;
import strat.MDStrategy;

//kind of the engine for my xai stuff incorporating ltl 
public class XAIfsm
{
	//main method
	public static void main(String[] args)
	{
		XAIfsm xaifsm = new XAIfsm();
		try {
			xaifsm.run();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void doLTLViolationStuff(String saveplace, String filename, boolean noappend) throws PrismException, FileNotFoundException
	{
		XAITemp xaiLtlBit = new XAITemp();

		PropertiesFile propertiesFile = xaiLtlBit.readModel(saveplace, filename);

		ExpressionReward exprRew = (ExpressionReward) propertiesFile.getProperty(0);
		xaiLtlBit.mainLog.println(exprRew.toString());

		xaiLtlBit.setMCExportOptionsAll(saveplace, filename);

		Expression expr = exprRew.getExpression();

		double discount = 1.0;
		XAIvi vi = new XAIvi();
		//			for (int i = 0; i < sa.mdp.getVarList().getNumVars(); i++)
		//				System.out.println(sa.mdp.getVarList().getName(i));
		ModelCheckerPartialSatResult optimalPolicy = vi.nviexposed(xaiLtlBit.prism, xaiLtlBit.mainLog, xaiLtlBit.mdp, expr, exprRew, null,
				xaiLtlBit.modulesFile, xaiLtlBit.mc, discount);

		//		Entry<MDP, MDStrategy> prodStratPair = xaiLtlBit.mc.checkPartialSatExprReturnStrategy(xaiLtlBit.mdp, expr, exprRew, null);
		MDStrategy strat = (MDStrategy) optimalPolicy.strat;// prodStratPair.getValue();
		MDP productMDP = vi.productmdp;//prodStratPair.getKey();
		PolicyCreator pc = new PolicyCreator();
		pc.createPolicy(productMDP, strat);
		pc.savePolicy(saveplace + "results/", "xai_" + filename + "_policy");
		Vector<BitSet> labelBS = new Vector<BitSet>();
		xaiLtlBit.convertPropertyToDA(expr, labelBS);

		xaiLtlBit.mdp = productMDP;
		//initialise to get labels for actions 

		xaiLtlBit.mainLog.println(expr.toString());

		ArrayList<BitSet> statelabels = xaiLtlBit.setStateLabels(labelBS, xaiLtlBit.mdp);

		HashMap<State, HashMap<Object, ArrayList<BitSet>>> actionLabels = xaiLtlBit.getStateActionLabels(statelabels, xaiLtlBit.mdp);
		HashMap<State, ArrayList<Object>> sinkStateActionLabels = xaiLtlBit.getStateActionLabelsSinkStates(statelabels, xaiLtlBit.mdp);
		try {
			xaiLtlBit.openDotFile(saveplace + "results/", "xai_" + filename + "_policy.dot");
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		try {
			String userInput = "";
			String exitString = "exit";
			while (!userInput.contentEquals(exitString)) {
				BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
				System.out.println("List actions in state: (exit to exit)");

				userInput = reader.readLine();
				if (!userInput.contentEquals(exitString)) {
					System.out.println("Displaying actions for first state that resembles " + userInput);
					State chosenState = null;
					//
					for (State stateHere : actionLabels.keySet()) {

						if (stateHere.toString().contains(userInput)) {

							xaiLtlBit.mainLog.println(actionLabels.get(stateHere).keySet().toString());
							chosenState = stateHere;
							break;
						}
					}
					System.out.println("Why not action?");
					userInput = reader.readLine();
					System.out.println("Why not action " + userInput);
					if (chosenState != null) {
						Object chosenAction = null;
						HashMap<Object, ArrayList<BitSet>> actionLabelsForThisState = actionLabels.get(chosenState);
						for (Object a : actionLabelsForThisState.keySet()) {
							if (a.toString().contains(userInput)) {
								chosenAction = a;
								break;
							}
						}
						if (chosenAction != null) {
							//					System.out.println(actionLabelsForThisState.get(chosenAction).toString());
							boolean actionOkay = true;
							if (sinkStateActionLabels.containsKey(chosenState)) {
								if (sinkStateActionLabels.get(chosenState).contains(chosenAction)) {
									actionOkay = false;
								}
							}
							if (!actionOkay) {
								xaiLtlBit.mainLog.println(chosenAction.toString() + " violates the LTL specification in state " + chosenState.toString());
							} else {
								xaiLtlBit.mainLog.println(chosenAction.toString() + " DOES NOT violate the LTL specification in state " + chosenState.toString()
										+ " so there's probably another reason!" + "\nOr it was chosen.");

							}
						}
					}
				}
			}

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void identifySwingStatesContextAvoid(String saveplace, String ofn, boolean noappend) throws Exception

	{

		//example 
		//		String saveplace = "/home/fatma/Data/PhD/melb/prism/joint/";
		//		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/xaiTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";

		HashMap<String, Double> variableWeights = new HashMap<String, Double>();
		variableWeights.put("p1", 1.0);
		variableWeights.put("p2", 1.0);
		variableWeights.put("da", 1.0);
		ArrayList<String> nonMDPDistVars = new ArrayList<String>();
		//		nonMDPDistVars.add("turn");
		nonMDPDistVars.add("p1");
		nonMDPDistVars.add("p2");

		String[] filenames = new String[] { ofn };
		XAIStateInformation prevSSTop = null;
		XAIStateInformation prevSSBottom = null;

		XAIPathCreator previousPath = null;
		VarList otherVL = null;
		for (String filename : filenames) {

			if (previousPath != null) {
				otherVL = previousPath.pc.mdpCreator.mdp.getVarList();

			}

			double discount = 1.0;

			//			boolean noappend = false;
			XAIStateAnalyser sa = new XAIStateAnalyser();
			sa.nonMDPDistVars = nonMDPDistVars;

			//load model 
			sa.readModel(saveplace, filename, noappend);
			variableWeights.put("mdp", (double) sa.mdp.getNumStates());
			sa.variableWeights = variableWeights;
			//get distance for mdp states 
			HashMap<State, HashMap<State, Double>> mdpDistshm = null;

			//process reward
			ExpressionReward exprRew = (ExpressionReward) sa.propertiesFile.getProperty(0);
			sa.mainLog.println(exprRew.toString());

			//preparing to get the optimal policy 
			sa.setMCExportOptionsAll(saveplace, filename);
			Vector<BitSet> labelBS = new Vector<BitSet>();
			XAITemp xaiLtlBit = new XAITemp();
			xaiLtlBit.mainLog = sa.mainLog; 
			xaiLtlBit.mc = sa.mc; 
			xaiLtlBit.prism = sa.prism; 
			xaiLtlBit.modulesFile = sa.modulesFile; 
			xaiLtlBit.mdp = sa.mdp; 

			Expression expr = exprRew.getExpression();

			XAIvi vi = new XAIvi();
			//			for (int i = 0; i < sa.mdp.getVarList().getNumVars(); i++)
			//				System.out.println(sa.mdp.getVarList().getName(i));
			ModelCheckerPartialSatResult optimalPolicy = vi.nviexposed(sa.prism, sa.mainLog, sa.mdp, expr, exprRew, null, sa.modulesFile, sa.mc, discount);

			sa.dasinkStates = vi.daSinkStates;
			sa.da = vi.origda;
			
			xaiLtlBit .processDA(sa.da, labelBS);

			ArrayList<BitSet> statelabels = xaiLtlBit.setStateLabels(labelBS,vi.productmdp);

			HashMap<State, HashMap<Object, ArrayList<BitSet>>> actionLabels = xaiLtlBit.getStateActionLabels(statelabels, vi.productmdp);
			HashMap<State, ArrayList<Object>> sinkStateActionLabels = xaiLtlBit.getStateActionLabelsSinkStates(statelabels,vi.productmdp);
			
			//save the product mdp 
			PrismFileLog pfl = new PrismFileLog(saveplace + "results/" + "xai_" + filename + "_prodmpd.dot");
			vi.productmdp.exportToDotFile(pfl, null, true);
			pfl.close();

			//now create a path from the optimal policy 
			XAIPathCreator optimalPolicyPaths = new XAIPathCreator();
			optimalPolicyPaths.createPathPolicy(0, vi.productmdp, optimalPolicy.strat, saveplace + "results/", "xai_" + filename + "optimalPath",
					vi.progRewards, vi.costRewards, vi.accStates);

			HashMap<State, ArrayList<Entry<State, Double>>> optimalPolicyStateDistances = sa.calculateStateDistancesOnTheFly(optimalPolicyPaths, mdpDistshm,
					sa.mdp);
			HashMap<State, ArrayList<Entry<State, Double>>> optimalPolicyStateDistancesFromAllStates = sa.calculateStateDistancesOnTheFly(optimalPolicyPaths,
					vi.productmdp, mdpDistshm, sa.mdp);
			//first we need to identify the "swing" states 
			ArrayList<XAIStateInformation> ssQ = sa.getSwingStatesListByCost(vi, discount, optimalPolicyPaths);
			boolean printStuff = false;
			//so now we have these ranked 
			//so lets print out the most influential one 

			if (printStuff) {
				System.out.println("All states ranked:");
				for (int i = 0; i < ssQ.size(); i++) {

					System.out.println(ssQ.get(i));

				}
			}
			//so the swing states are kind of at the ends of the list 
			//so we'll just pick those 
			XAIStateInformation ssTop = ssQ.get(0);
			XAIStateInformation ssBottom = ssQ.get(ssQ.size() - 1);

			System.out.println("Swing States");
			System.out.println("Top: " + ssTop.toString());

			if (printStuff) {
				sa.listClosestStatesToStateOnTheFly(ssTop, null, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(ssTop, null, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp,
						vi.productmdp, optimalPolicy);
			}
			if (prevSSTop != null) {
				System.out.println("Listing states to closest prev state " + prevSSTop.toString());
				sa.listClosestStatesToStateOnTheFly(prevSSTop, otherVL, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(prevSSTop, otherVL, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp,
						vi.productmdp, optimalPolicy);
			}
			if (prevSSTop == null)
				prevSSTop = ssTop;
			System.out.println("Bottom: " + ssBottom.toString());

			if (printStuff) {
				sa.listClosestStatesToStateOnTheFly(ssBottom, null, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(ssBottom, null, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp,
						vi.productmdp, optimalPolicy);
			}
			if (prevSSBottom != null) {

				System.out.println("Listing states to closest prev state " + prevSSBottom.toString());
				sa.listClosestStatesToStateOnTheFly(prevSSBottom, otherVL, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(prevSSBottom, otherVL, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp,
						vi.productmdp, optimalPolicy);

			}
			if (prevSSBottom == null)
				prevSSBottom = ssBottom;
			XAIDoContrast xaicont = new XAIDoContrast();
			if (previousPath != null) {
				XAIPathCreator altPath = sa.createAlternatePath(previousPath, filename, vi, optimalPolicy, saveplace);

				//				ModelCheckerPartialSatResult altPathPolicy = vi.doPathNVI(sa.prism, sa.mainLog, altPath, sa.mc, discount);

				//				HashMap<State, ArrayList<Entry<State, Double>>> altPolicyStateDistances = sa.calculateStateDistancesOnTheFly(altPath, mdpDistshm,
				//						sa.mdp);
				//				HashMap<State, ArrayList<Entry<State, Double>>> altPolicyStateDistancesFromAllStates = sa.calculateStateDistancesOnTheFly(altPath,
				//						vi.productmdp, mdpDistshm, sa.mdp);
				//first we need to identify the "swing" states 
				ArrayList<XAIStateInformation> altssQ = sa.getSwingStatesListByCost(vi, discount, altPath);
				XAIStateInformation altssTop = altssQ.get(0);
				XAIStateInformation altssBottom = altssQ.get(altssQ.size() - 1);

				System.out.println("Alt Path Swing States");
				System.out.println("Top: " + altssTop.toString());
				System.out.println(xaicont.textTemplate(altssTop));
				System.out.println("Bottom: " + altssBottom.toString());
				System.out.println(xaicont.textTemplate(altssBottom));
				double rangeAlt = xaicont.compareRelativeCostDifference(altssTop, altssBottom);
				System.out.println(rangeAlt);
				System.out.println("Previous Path Swing States");
				System.out.println("Top:" + ssTop.toString());
				System.out.println(xaicont.textTemplate(ssTop));
				System.out.println("Bottom:" + ssBottom.toString());
				System.out.println(xaicont.textTemplate(ssBottom));
				double range = xaicont.compareRelativeCostDifference(ssTop, ssBottom);
				System.out.println(range);
				if (range == 0)
					System.out.println("Each state in the path has a uniform contribution to the path");

				System.out.println("The difference between the most and least costly states in the alternate path and that in the other path is "
						+ Math.abs(range - rangeAlt));

			}
			if (previousPath == null)
				previousPath = optimalPolicyPaths;
		}

	}

	//the actual main method 
	public void run() throws Exception
	{

		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/xaiTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		String filename = "xai_r1_d1_g1_fs1notgoal_avoid";//"g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";
		boolean noappend = false;
		doLTLViolationStuff(saveplace, filename, noappend);
		identifySwingStatesContextAvoid(saveplace, filename, noappend);

	}

}
