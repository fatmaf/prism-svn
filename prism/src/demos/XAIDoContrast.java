package demos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

import explicit.MDP;
import explicit.ModelCheckerPartialSatResult;
import parser.State;
import parser.ast.Expression;
import parser.ast.ExpressionReward;
import prism.PrismException;
import prism.PrismFileLog;

public class XAIDoContrast
{
	//main function 
	public static void main(String[] args)
	{
		XAIDoContrast docontrast = new XAIDoContrast();
		try {
			docontrast.identifySwingStatesSurveillance();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void identifySwingStatesSurveillance() throws Exception
	{

		//example 
		String saveplace = "/home/fatma/Data/PhD/melb/prism/joint/";

		ArrayList<String> nonMDPDistVars = new ArrayList<String>();
//		nonMDPDistVars.add("turn");
		nonMDPDistVars.add("p1");
		nonMDPDistVars.add("p2");
		String[] filenames = new String[] { "scenario3", "scenario3_alt" };
		XAIStateInformation prevSSTop = null;
		XAIStateInformation prevSSBottom = null;

		XAIPathCreator previousPath = null;
		for (String filename : filenames) {

			double discount = 1.0;

			boolean noappend = true;
			XAIStateAnalyser sa = new XAIStateAnalyser();
			sa.nonMDPDistVars = nonMDPDistVars;

			//load model 
			sa.readModel(saveplace, filename, noappend);

			//get distance for mdp states 
			HashMap<State, HashMap<State, Double>> mdpDistshm = null;

			//process reward
			ExpressionReward exprRew = (ExpressionReward) sa.propertiesFile.getProperty(0);
			sa.mainLog.println(exprRew.toString());

			//preparing to get the optimal policy 
			sa.setMCExportOptionsAll(saveplace, filename);

			Expression expr = exprRew.getExpression();

			XAIvi vi = new XAIvi();
			//			for (int i = 0; i < sa.mdp.getVarList().getNumVars(); i++)
			//				System.out.println(sa.mdp.getVarList().getName(i));
			ModelCheckerPartialSatResult optimalPolicy = vi.nviexposed(sa.prism, sa.mainLog, sa.mdp, expr, exprRew, null, sa.modulesFile, sa.mc, discount);

			sa.dasinkStates = vi.daSinkStates;
			sa.da = vi.origda;
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
			ArrayList<XAIStateInformation> ssQ = sa.getSwingStatesListByCost(vi,discount, optimalPolicyPaths);
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
				sa.listClosestStatesToStateOnTheFly(ssTop, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(ssTop, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp, vi.productmdp,
						optimalPolicy);
			}
			if (prevSSTop != null) {
				System.out.println("Listing states to closest prev state " + prevSSTop.toString());
				sa.listClosestStatesToStateOnTheFly(prevSSTop, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(prevSSTop, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp, vi.productmdp,
						optimalPolicy);
			}
			if (prevSSTop == null)
				prevSSTop = ssTop;
			System.out.println("Bottom: " + ssBottom.toString());

			if (printStuff) {
				sa.listClosestStatesToStateOnTheFly(ssBottom, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm,  sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(ssBottom, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp, vi.productmdp,
						optimalPolicy);
			}
			if (prevSSBottom != null) {
			
				System.out.println("Listing states to closest prev state " + prevSSBottom.toString());
				sa.listClosestStatesToStateOnTheFly(prevSSBottom, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(prevSSBottom, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp,
						vi.productmdp, optimalPolicy);
			

			}
			if (prevSSBottom == null)
				prevSSBottom = ssBottom;

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
				System.out.println("Bottom: " + altssBottom.toString());

			}
			if (previousPath == null)
				previousPath = optimalPolicyPaths;
		}

	}

	public void identifySwingStates() throws Exception
	{

		XAIStateAnalyser sa = new XAIStateAnalyser();
		//example 
		String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/xaiTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		saveplace = "/home/fatma/Data/PhD/melb/prism/";
		String filenamec = "xai_r2_i3_pickplace_fs0_0_";//"xai_r1_d1_g1_key";//"xai_r1_d1_g1_key";//_fs1notgoal_noback";//"g5_r2_t3_d2_fs1";//"g7x3_r2_t3_d0_fs1";//"robot";

		filenamec = "scenario0";//"domainTemplate_updated";
		String[] filenames = new String[] { "scenario0", "scenario0_alt" };
		for (String filename : filenames) {
			HashMap<String, Double> discountVals = new HashMap<String, Double>();
			discountVals.put("xai_r1_d1_g1", 1.0);
			discountVals.put("xai_r1_d1_g1_key", 0.9);
			discountVals.put("xai_r2_i2_pickplace_fs0_0_", 1.0);
			discountVals.put("xai_r2_i3_pickplace_fs0_0_", 1.0);
			double discount = 1.0;
			if (discountVals.containsKey(filename)) {
				discountVals.get(filename);
			}
			boolean noappend = true; //set to true for the ones labeled xai 
			//load model 
			sa.readModel(saveplace, filename, noappend);

			//get distance for mdp states 
			HashMap<State, HashMap<State, Double>> mdpDistshm = null;//mdpDijkstra(mdp);
			if (!noappend)
				mdpDistshm = sa.mdpDijkstra(sa.mdp);
			//process reward
			ExpressionReward exprRew = (ExpressionReward) sa.propertiesFile.getProperty(0);
			sa.mainLog.println(exprRew.toString());

			//preparing to get the optimal policy 
			sa.setMCExportOptionsAll(saveplace, filename);

			Expression expr = exprRew.getExpression();

			XAIvi vi = new XAIvi();
			ModelCheckerPartialSatResult optimalPolicy = vi.nviexposed(sa.prism, sa.mainLog, sa.mdp, expr, exprRew, null, sa.modulesFile, sa.mc, discount);
			sa.dasinkStates = vi.daSinkStates;
			sa.da = vi.origda;
			//save the product mdp 
			PrismFileLog pfl = new PrismFileLog(saveplace + "results/" + "xai_" + filename + "_prodmpd.dot");
			vi.productmdp.exportToDotFile(pfl, null, true);
			pfl.close();

			//now create a path from the optimal policy 
			XAIPathCreator optimalPolicyPaths = new XAIPathCreator();
			optimalPolicyPaths.createPathPolicy(0, vi.productmdp, optimalPolicy.strat, saveplace + "results/", "xai_" + filename + "optimalPath",
					vi.progRewards, vi.costRewards, vi.accStates);
			if (!noappend) {
				HashMap<State, ArrayList<Entry<State, Double>>> optimalPolicyStateDistances = sa.calculateStateDistances(optimalPolicyPaths, mdpDistshm);
				HashMap<State, ArrayList<Entry<State, Double>>> allStateDistances = sa.calculateStateDistances(vi.productmdp, mdpDistshm);

				//first we need to identify the "swing" states 
				ArrayList<XAIStateInformation> ssQ = sa.getSwingStatesListByCost(vi, discount, optimalPolicyPaths);

				//so now we have these ranked 
				//so lets print out the most influential one 
				for (int i = 0; i < ssQ.size(); i++) {

					System.out.println(ssQ.get(i));
				}
				//so the swing states are kind of at the ends of the list 
				//so we'll just pick those 
				XAIStateInformation ssTop = ssQ.get(0);
				XAIStateInformation ssBottom = ssQ.get(ssQ.size() - 1);
				sa.listClosestStatesToState(ssTop, optimalPolicyStateDistances, optimalPolicyPaths);

				sa.listClosestStatesToState(ssTop, allStateDistances, null);
				sa.listClosestStatesToState(ssBottom, optimalPolicyStateDistances, optimalPolicyPaths);

				XAIPathCreator alternatePolicyPaths = sa.createAlternatePath(filename, vi, optimalPolicy, saveplace);
				ArrayList<XAIStateInformation> alternateSSQ = sa.getSwingStatesListByCost(vi, discount, alternatePolicyPaths);

				for (int i = 0; i < alternateSSQ.size(); i++) {

					System.out.println(alternateSSQ.get(i));
				}
				HashMap<State, ArrayList<Entry<State, Double>>> alternatePolicyStateDistances = sa.calculateStateDistances(alternatePolicyPaths, mdpDistshm);

				XAIStateInformation ssTopAlt = alternateSSQ.get(0);
				XAIStateInformation ssBottomAlt = alternateSSQ.get(alternateSSQ.size() - 1);
				sa.listClosestStatesToState(ssTopAlt, alternatePolicyStateDistances, alternatePolicyPaths);

				sa.listClosestStatesToState(ssBottomAlt, alternatePolicyStateDistances, alternatePolicyPaths);
			}
		}
	}

	public void identifySwingStatesContext() throws Exception
	{

		//example 
		String saveplace = "/home/fatma/Data/PhD/melb/prism/";

		ArrayList<String> nonMDPDistVars = new ArrayList<String>();
		nonMDPDistVars.add("turn");
		nonMDPDistVars.add("photo1");
		nonMDPDistVars.add("photo2");
		String[] filenames = new String[] { "scenario1", "scenario1_alt" };
		XAIStateInformation prevSSTop = null;
		XAIStateInformation prevSSBottom = null;

		XAIPathCreator previousPath = null;
		for (String filename : filenames) {

			double discount = 1.0;

			boolean noappend = true;
			XAIStateAnalyser sa = new XAIStateAnalyser();
			sa.nonMDPDistVars = nonMDPDistVars;

			//load model 
			sa.readModel(saveplace, filename, noappend);

			//get distance for mdp states 
			HashMap<State, HashMap<State, Double>> mdpDistshm = null;

			//process reward
			ExpressionReward exprRew = (ExpressionReward) sa.propertiesFile.getProperty(0);
			sa.mainLog.println(exprRew.toString());

			//preparing to get the optimal policy 
			sa.setMCExportOptionsAll(saveplace, filename);

			Expression expr = exprRew.getExpression();

			XAIvi vi = new XAIvi();
			//			for (int i = 0; i < sa.mdp.getVarList().getNumVars(); i++)
			//				System.out.println(sa.mdp.getVarList().getName(i));
			ModelCheckerPartialSatResult optimalPolicy = vi.nviexposed(sa.prism, sa.mainLog, sa.mdp, expr, exprRew, null, sa.modulesFile, sa.mc, discount);

			sa.dasinkStates = vi.daSinkStates;
			sa.da = vi.origda;
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
				sa.listClosestStatesToState(ssTop, optimalPolicyStateDistances, optimalPolicyPaths);
				sa.listClosestStatesToState(ssTop, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths);
			}
			if (prevSSTop != null) {
				System.out.println("Listing states to closest prev state " + prevSSTop.toString());
				sa.listClosestStatesToStateOnTheFly(prevSSTop, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(prevSSTop, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp, vi.productmdp,
						optimalPolicy);
			}
			if (prevSSTop == null)
				prevSSTop = ssTop;
			System.out.println("Bottom: " + ssBottom.toString());

			if (printStuff) {
				sa.listClosestStatesToState(ssBottom, optimalPolicyStateDistances, optimalPolicyPaths);
				sa.listClosestStatesToState(ssBottom, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths);
			}
			if (prevSSBottom != null) {
				System.out.println("Listing states to closest prev state " + prevSSBottom.toString());
				sa.listClosestStatesToStateOnTheFly(prevSSBottom, optimalPolicyStateDistances, optimalPolicyPaths, mdpDistshm, sa.mdp, null, null);
				sa.listClosestStatesToStateOnTheFly(prevSSBottom, optimalPolicyStateDistancesFromAllStates, optimalPolicyPaths, mdpDistshm, sa.mdp,
						vi.productmdp, optimalPolicy);

			}
			if (prevSSBottom == null)
				prevSSBottom = ssBottom;

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
				System.out.println("Bottom: " + altssBottom.toString());

			}
			if (previousPath == null)
				previousPath = optimalPolicyPaths;
		}

	}

}
