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

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceType;
import automata.DA;
import cern.colt.Arrays;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.ModelCheckerResult;
import explicit.ProbModelChecker;
import parser.State;
import parser.ast.Expression;
import parser.ast.ExpressionProb;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismComponent;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismLog;
import prism.ProductModelGenerator;
import prism.Result;
import simulator.ModulesFileModelGenerator;

/**
 * We get some rules to follow that and this these and those no one knows
 * 
 * No one knows
 *
 */
public class MRmcts {
	public static void main(String[] args) {
		new MRmcts().run();
	}

	public void run() {
		try {

			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
			String filename = "no_door_example";

			// Create a log for PRISM output (hidden or stdout)
			// PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");

			// Initialise PRISM engine
			Prism prism = new Prism(mainLog);
			prism.initialise();
			ArrayList<String> filenames = new ArrayList<String>();
			filenames.add(saveplace + filename + "0.prism");
			filenames.add(saveplace + filename + "1.prism");
			String propfilename = saveplace + filename + ".prop";
			prism.setEngine(Prism.EXPLICIT);
			ArrayList<ProductModelGenerator> prodModGens = new ArrayList<ProductModelGenerator>();

			ExpressionProb expr = null;
			DA<BitSet, ? extends AcceptanceOmega> da=null;
			LTLModelChecker ltlMC;// = new LTLModelChecker(prism);
			AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };

			int fileno = 0;
			for (fileno = 0; fileno < filenames.size(); fileno++) {
				List<Expression> labelExprs = new ArrayList<Expression>();
				ModulesFile modulesFile = prism.parseModelFile(new File(filenames.get(fileno)));
				prism.loadPRISMModel(modulesFile);
//				if(expr == null) {
				// have not investigated this 
				//but i can not use the same da to generate all models 
				//i get an outofbounds index error for the first state when i do get init state on that prod mod 
				//soo no to this if above 
				PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(propfilename));
				expr = (ExpressionProb) propertiesFile.getProperty(0);
				
				ltlMC = new LTLModelChecker(prism);
				da = ltlMC.constructExpressionDAForLTLFormula(expr.getExpression(), labelExprs,
						allowedAcceptance);
				da.setDistancesToAcc();
				da.printDot(mainLog);
				PrismFileLog out = new PrismFileLog(saveplace+"mrmctsda"+fileno+".dot");
				da.printDot(out);
				out.close();
//				}
				ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, prism);
				ProductModelGenerator prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
				prodModGens.add(prodModelGen);
			}
			//now do something with these 
			BitSet acc = da.getAccStates();
			BitSet sinkStates = da.getSinkStates();
			int max_rollouts = 10000; 
			//the nodoorexample has 8 steps for 1 robot 
			//so lets do 10 steps + slack = 30
			int rollout_depth = 30;
			MRuct uct = new MRuct(prism.getLog(),prodModGens,max_rollouts,rollout_depth,null,da.getDistsToAcc());
			uct.search(acc);
			if(uct.uctPolicy.accStates.cardinality() > 0) {
			MDPModelChecker mdpMC = new MDPModelChecker(prism);
			uct.uctPolicy.jointMDP.findDeadlocks(true);
			ModelCheckerResult res = mdpMC.computeReachProbs(uct.uctPolicy.jointMDP,uct.uctPolicy.accStates, false);
			mainLog.println("Result: "+Arrays.toString(res.soln));
			}
			mainLog.print("acha");
//			UCT uct;
//			demos.UCT.UCTNode uctRes;
//			BitSet originalTerminalStates = new BitSet();
//			BitSet prunedTerminalStates = new BitSet();
//			int depth = 30;
//			int nSamples = 100000;
//			
//			MDPSimple resMDP2 = null, resMDP = null;
//			List<MDPSimple> candidateMDPs = new ArrayList<MDPSimple>();
//			List<Double> candidateMDPsReachProbs = new ArrayList<Double>();
//			List<BitSet> candidateMDPsAccStates = new ArrayList<BitSet>();
//			double reachProb = 0.0;
//			int nCandidates = 0;
//			BitSet accModel = null;
//			ModelCheckerResult checkRes = null;
//			int current_acc = acc.nextSetBit(0);
//			boolean useDistCost = false; 
//			
////			for (int current_acc = acc.nextSetBit(0); current_acc >= 0; current_acc = acc.nextSetBit(current_acc + 1)) {
//				List<Double> distsToAcc = da.calculateDistsToState(acc.nextSetBit(0));
				
//				uct = new UCT(prism.getLog(), prodModelGen, prodModelGen.getInitialState(), distsToAcc, sinkStates, depth, nSamples, useDistCost, null); //TODO iterative deepening; tweak exploration param, # samples, depth
//				uctRes = uct.search();
//				resMDP = buildMDP(uctRes, prodModelGen, originalTerminalStates);
//				accModel = findAccStates(resMDP, da);
//				checkRes = checkReachability(resMDP, accModel);
//				resMDP2 = buildPolicyMDP(prodModelGen, resMDP, checkRes, originalTerminalStates, prunedTerminalStates);
//				reachProb = checkRes.soln[0];
//				int pos = 0;
//				if(reachProb > 0) {
//					for(pos = 0; pos < candidateMDPsReachProbs.size(); pos++) {
//						if (reachProb > candidateMDPsReachProbs.get(pos)) {
//							break;
//						}
//					}
//					candidateMDPs.add(pos, resMDP);
//					candidateMDPsReachProbs.add(pos, reachProb);
//					candidateMDPsAccStates.add(pos, accModel);
//					nCandidates++;
//				}
//			}

			

			// da.printDot(new PrismFileLog(saveplace+"da.dot"));
			// int numStateVars = mdps.size() + 1;
			//
			// ArrayList<String> stateLabels = new ArrayList<String>();
			// ArrayList<HashMap<String,BitSet>> mdpStateLabels = new
			// ArrayList<HashMap<String,BitSet>>();
			// State jointState = new State(numStateVars);
			// //get all initial states
			// for(int i = 0; i<mdps.size(); i++)
			// {
			//
			// jointState.setValue(i, mdps.get(i).getFirstInitialState());
			// HashMap<String,BitSet> labelsMap = new HashMap<String,BitSet>();
			// for(String lab: mdps.get(i).getLabels())
			// {
			// labelsMap.put(lab,mdps.get(i).getLabelStates(lab));
			// for (Expression expr : labelExprs)
			// {
			//
			// if(labelsMap.get(lab).get(mdps.get(i).getFirstInitialState()))
			// {
			// stateLabels.add(lab);
			// }
			// }
			//
			// }
			// mdpStateLabels.add(labelsMap);
			// }
			// jointState.setValue(numStateVars-1, da.getStartState());
			// //there may be a case where one of the robots puts us in a state that we
			// weren't aware of
			// //you know that whole starting in a goal state thing
			// //best to deal with it here.
			//
			//
			//
			// mainLog.println(jointState.toString());


			// Close down PRISM
			prism.closeDown();

		} catch (PrismException | FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
	}
}