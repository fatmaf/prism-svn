//==============================================================================
//	
//	Copyright (c) 2017-
//	Authors:
//	* Dave Parker <d.a.parker@cs.bham.ac.uk> (University of Birmingham)
//	
//------------------------------------------------------------------------------
//	
//	This file is part of PRISM.
//	
//	PRISM is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.
//	
//	PRISM is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//	
//	You should have received a copy of the GNU General Public License
//	along with PRISM; if not, write to the Free Software Foundation,
//	Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//	
//==============================================================================

// MA BRTDP and MCTS 
// Billie Eilish - bury's a friend 

package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import cern.colt.Arrays;
import explicit.LTLModelChecker;
import explicit.LTLModelChecker.LTLProduct;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.ModelCheckerResult;
import explicit.ProbModelChecker;
import parser.State;
import parser.VarList;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionProb;
import parser.ast.ExpressionQuant;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.ModelChecker;
import prism.Prism;
import prism.PrismComponent;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismLog;
import prism.ProductModelGenerator;
import prism.Result;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;
import strat.Strategy;

/**
 * We get some rules to follow that and this these and those no one knows
 * 
 * No one knows
 *
 */
public class MRmcts {
	public static final String TESTSLOC = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/results/mcts/";

	public static void main(String[] args) {
		try {
			new MRmcts().run();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public MDStrategy getSingleRobotSolution(Prism prism, LTLModelChecker ltlMC, ExpressionProb expr,
			AcceptanceType[] allowedAcceptance, ArrayList<List<State>> allRobotsStatesList, ArrayList<VarList> varlists)
			throws PrismException {

		prism.buildModel();
		MDP mdp = (MDP) prism.getBuiltModelExplicit();

		MDPModelChecker mc = new MDPModelChecker(prism);
		mc.setGenStrat(true);
		mc.setExportAdv(true);

		Vector<BitSet> labelBS = new Vector<BitSet>();
		ProbModelChecker pmc = new ProbModelChecker(prism);

//		LTLProduct<MDP> prod = ltlMC.constructProductMDP(pmc, mdp, expr.getExpression(), null, allowedAcceptance);
		DA<BitSet, ? extends AcceptanceOmega> tempda = ltlMC.constructDAForLTLFormula(pmc, mdp, expr.getExpression(),
				labelBS, allowedAcceptance);
		LTLProduct<MDP> prod = ltlMC.constructProductModel(tempda, mdp, labelBS, null);

		MDP prodmdp = prod.getProductModel();
		varlists.add(prodmdp.getVarList());
		allRobotsStatesList.add(prodmdp.getStatesList());
		BitSet acc = ((AcceptanceReach) prod.getAcceptance()).getGoalStates();

		ModelCheckerResult result = mc.computeReachProbs(prodmdp, acc, false);
		MDStrategy strat = (MDStrategy) (result.strat);
		return strat;
	}

	public void run() throws Exception {
		try {

			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
			String filename = "tiny_example";//"no_door_example";

			// Create a log for PRISM output (hidden or stdout)
			// PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");
			Long startTime = System.currentTimeMillis();
			// Initialise PRISM engine
			Prism prism = new Prism(mainLog);
			prism.initialise();
			ArrayList<String> filenames = new ArrayList<String>();
			filenames.add(saveplace + filename + "0.prism");
			filenames.add(saveplace + filename + "1.prism");
			String propfilename = saveplace + filename + ".prop";
			prism.setEngine(Prism.EXPLICIT);
			ArrayList<ProductModelGenerator> prodModGens = new ArrayList<ProductModelGenerator>();
			ArrayList<List<State>> allRobotsStatesList = new ArrayList<List<State>>();

			ExpressionProb expr = null;
			DA<BitSet, ? extends AcceptanceOmega> da = null;
			LTLModelChecker ltlMC;// = new LTLModelChecker(prism);
			AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
			ArrayList<MDStrategy> singleRobotSolutions = new ArrayList<MDStrategy>();
			ArrayList<Boolean> flipedIndices = new ArrayList<Boolean>();
			// we need to create a base policy too
			// so we have to do basic prism model checking
			MDP mdp = null;
			int fileno = 0;
			for (fileno = 0; fileno < filenames.size(); fileno++) {
				List<Expression> labelExprs = new ArrayList<Expression>();
				ModulesFile modulesFile = prism.parseModelFile(new File(filenames.get(fileno)));
				prism.loadPRISMModel(modulesFile);

				// i can not use the same da to generate all models
				// i get an outofbounds index error for the first state when i do get init state
				// on that prod mod
				// soo no to this if above
				PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(propfilename));
//				ExpressionFunc exprfunc = (ExpressionFunc) propertiesFile.getProperty(0);
//				ExpressionQuant exprq = (ExpressionQuant) propertiesFile.getProperty(0); 

				expr = (ExpressionProb) propertiesFile.getProperty(0);

				//

				ltlMC = new LTLModelChecker(prism);
				da = ltlMC.constructExpressionDAForLTLFormula(expr.getExpression(), labelExprs, allowedAcceptance);

				ArrayList<VarList> varlists = new ArrayList<VarList>();

				MDStrategy strat = getSingleRobotSolution(prism, ltlMC, expr, allowedAcceptance, allRobotsStatesList,
						varlists);
				singleRobotSolutions.add(strat);

//					Result result = mc.check(mdp, expr);//mc.computeReachProbs((MDP)prodmdp.getProductModel(), target, min)

//					System.out.println(result.getResult()); 
//				}
				da.setDistancesToAcc();
				da.printDot(mainLog);

				PrismFileLog out = new PrismFileLog(TESTSLOC + "mrmctsda" + fileno + ".dot");
				da.printDot(out);
				out.close();

				ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, prism);
				ProductModelGenerator prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
				
				prodModGens.add(prodModelGen);
				// comparing indices of the mdp and prodmodgen stuff
				VarList vl = varlists.get(0);
				VarList pmdvl = prodModelGen.createVarList();
				// lets match these
				String dastring = "da";
				int mdpdanum = -1;
				for (int i = 0; i < vl.getNumVars(); i++) {
					if (vl.getName(i).contains(dastring)) {
						mdpdanum = i;
						break;
					}
				}
				int pmddanum = -1;
				for (int i = 0; i < pmdvl.getNumVars(); i++) {
					if (pmdvl.getName(i).contains(dastring)) {
						pmddanum = i;
						break;
					}
				}
				if (mdpdanum != pmddanum) {
					// do stuff here
					// i have no idea what
					flipedIndices.add(true);
//					flipedIndices.add(false);
				} else {
					flipedIndices.add(false);
//					flipedIndices.add(true);
				}

			}

			// now do something with these
			BitSet acc = da.getAccStates();
			BitSet sinkStates = da.getSinkStates();
			int max_rollouts = 1000;
			// the nodoorexample has 8 steps for 1 robot
			// so lets do 10 steps + slack = 30
			int rollout_depth = 30;
			boolean minCost = false;
			MRuctPaper uct = new MRuctPaper(prism.getLog(), prodModGens, max_rollouts, rollout_depth, null,
					da.getDistsToAcc(), singleRobotSolutions, allRobotsStatesList,flipedIndices,da);
//			uct.uctsearch(acc);
//			ArrayList<Integer> solfoundinrollout = uct.uctsearchwithoutapolicy(acc);
			uct.monteCarloPlanning(acc, minCost);
			long searchOverTime = System.currentTimeMillis();
			if(uct.uctPolicy.accStates.cardinality()> 0)
			{
				//we can compute upper and lower bounds 
				//upper bounds - all leaf states are also acc states 
				//lower bounds just acc states 
				uct.uctPolicy.jointMDP.findDeadlocks(true);
				BitSet accStates = (BitSet)uct.uctPolicy.accStates.clone(); 
				MDPModelChecker mdpMC = new MDPModelChecker(prism); 
				ModelCheckerResult upperB = mdpMC.computeReachProbs(uct.uctPolicy.jointMDP, accStates, false);
				
				accStates.or(uct.uctPolicy.leafStates);
				ModelCheckerResult lowerB = mdpMC.computeReachProbs(uct.uctPolicy.jointMDP, accStates, false);
				mainLog.println(uct.uctPolicy.leafStates.toString());
				//TODO: double check if this leaf state stuff works 
				
				mainLog.println("Result: " + Arrays.toString(upperB.soln));
				mainLog.println("Result: " + Arrays.toString(lowerB.soln));
				
			}
			//going to print out the state info 
			for(State s: uct.uctPolicy.stateActionQvalues.keySet())
			{
				uct.uctPolicy.printStateDetails(s, minCost);
			}
//			if (uct.uctPolicy.accStates.cardinality() > 0) {
//				MDPModelChecker mdpMC = new MDPModelChecker(prism);
//				uct.uctPolicy.jointMDP.findDeadlocks(true);
//				ModelCheckerResult res = mdpMC.computeReachProbs(uct.uctPolicy.jointMDP, uct.uctPolicy.accStates,
//						false);
//				uct.uctPolicy.jointMDP.exportToDotFile(saveplace + filename + "sol.dot", uct.uctPolicy.accStates);
//				mainLog.println("Result: " + Arrays.toString(res.soln));
//				MDPSimple policyTree = uct.uctPolicy.extractPolicyTreeAsDotFile(uct.uctPolicy.jointMDP,
//						uct.uctPolicy.getStateIndex(uct.initState), true);
//				policyTree.exportToDotFile(saveplace + filename + "_policy_sol.dot");
////				mainLog.println("Accepting States found in rollouts: " + Arrays.toString(solfoundinrollout.toArray()));
//			}

			long endTime = System.currentTimeMillis();
			mainLog.println("Search Time: " + (searchOverTime - startTime) / 1000.0 + "s" + "\nTotal Time:"
					+ (endTime - startTime) / 1000.0 + "s");
			// Close down PRISM
			prism.closeDown();

		} catch (PrismException | FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
	}
}