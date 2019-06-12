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
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Map.Entry;
import java.util.Vector;
import java.util.Arrays;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.Distribution;
import explicit.LTLModelChecker;
import explicit.LTLModelChecker.LTLProduct;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.ModelCheckerPartialSatResult;
import explicit.ModelCheckerResult;
import explicit.ProbModelChecker;
import explicit.StateValues;
import explicit.rewards.MDPRewardsSimple;
import parser.State;
import parser.VarList;
import parser.ast.Expression;
import parser.ast.ExpressionProb;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.ModelGenerator;
import prism.ModelInfo;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.ProductModelGenerator;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;
import strat.Strategy;

/**
 * We get some rules to follow that and this these and those no one knows
 * 
 * No one knows
 *
 */
public class MRmcts
{
	public static final String TESTSLOC = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/results/";
	public static boolean TURNOFFALLWRITES = false;

	public static void main(String[] args)
	{
		try {
			new MRmcts().run();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public MDPSimple extractPolicyTreeAsDotFile(MDP mdp, int initialState, Strategy strat)
	{

		MDPSimple policyTree = new MDPSimple();
		List<State> statesList = new ArrayList<State>();
		int[] stateLabels = new int[mdp.getNumStates()];
		Arrays.fill(stateLabels, -1);
		Queue<Integer> stateQ = new LinkedList<Integer>();
		stateQ.add(initialState);
		int state, ps, choice;
		Object action = null;
		BitSet visited = new BitSet();

		while (!stateQ.isEmpty()) {
			state = stateQ.remove();
			if (!visited.get(state)) {
				visited.set(state);
				if (stateLabels[state] == -1) {
					stateLabels[state] = policyTree.addState();
					statesList.add(mdp.getStatesList().get(state));
				}
				ps = stateLabels[state];
				strat.initialise(state);
				action = strat.getChoiceAction();
				choice = -1;
				if (action != null) {
					int numchoices = mdp.getNumChoices(state);
					for (int i = 0; i < numchoices; i++) {
						if (action.equals(mdp.getAction(state, i))) {
							choice = i;
							break;
						}
					}

					if (choice > -1) {
						Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(state, choice);
						Distribution distr = new Distribution();
						while (tranIter.hasNext()) {
							Entry<Integer, Double> csp = tranIter.next();
							int childstate = csp.getKey();
							double stateProb = csp.getValue();
							if (stateLabels[childstate] == -1) {
								stateLabels[childstate] = policyTree.addState();
								statesList.add(mdp.getStatesList().get(childstate));
							}
							int cs = stateLabels[childstate];
							distr.add(cs, stateProb);
							stateQ.add(childstate);
						}
						policyTree.addActionLabelledChoice(ps, distr, action.toString());
					}

				}
			}
		}

		policyTree.setStatesList(statesList);

		return policyTree;
	}

	public HashMap<State, HashMap<String, Double>> doPartialSatNVI(Prism prism, String filename, MDP productMdp, BitSet acc, MDPRewardsSimple progRewards,
			MDPRewardsSimple prodCosts, BitSet progStates, HashMap<State, HashMap<String, Double>> resStuff, boolean upperBound) throws PrismException
	{
		PrismLog mainLog = prism.getMainLog();
		MDPModelChecker mcProduct = new MDPModelChecker(prism);

		mcProduct.setGenStrat(true);
		mcProduct.setExportAdv(true);
		//		ArrayList<HashMap<State,Double>>
		mainLog.println("\nComputing reachability probability, expected progression, and expected cost...");
		ModelCheckerPartialSatResult res = mcProduct.computeNestedValIter(productMdp, acc, progRewards, prodCosts, progStates);

		StateValues probsProduct = StateValues.createFromDoubleArray(res.solnProb, productMdp);
		StateValues progsProduct = StateValues.createFromDoubleArray(res.solnProg, productMdp);
		StateValues costsProduct = StateValues.createFromDoubleArray(res.solnCost, productMdp);

		String stringPrefix = "l";
		if (upperBound)
			stringPrefix = "u";
		String pString = stringPrefix + "p";
		String progString = stringPrefix + "prog";
		String costString = stringPrefix + "c";

		HashMap<State, HashMap<String, Double>> partSatRes;
		if (resStuff == null)
			partSatRes = new HashMap<State, HashMap<String, Double>>();
		else
			partSatRes = resStuff;

		for (int i = 0; i < probsProduct.getSize(); i++) {
			//			res.strat.initialise(i);
			//			mainLog.println("" + productMdp.getStatesList().get(i) + ":" + probsProduct.getDoubleArray()[i] + "," + progsProduct.getDoubleArray()[i] + ","
			//					+ costsProduct.getDoubleArray()[i] + "-" + res.strat.getChoiceAction());

			HashMap<String, Double> stateVals;
			if (resStuff == null)
				stateVals = new HashMap<String, Double>();
			else
				stateVals = partSatRes.get(productMdp.getStatesList().get(i));
			stateVals.put(pString, probsProduct.getDoubleArray()[i]);
			stateVals.put(progString, progsProduct.getDoubleArray()[i]);
			stateVals.put(costString, costsProduct.getDoubleArray()[i]);
			partSatRes.put(productMdp.getStatesList().get(i), stateVals);
		}
		MDPSimple policyTree = extractPolicyTreeAsDotFile(productMdp, 0, res.strat);

		PrismFileLog out = new PrismFileLog(TESTSLOC + filename + "_brtdp_policy_NVI.dot");
		policyTree.exportToDotFile(out, null, true);
		out.close();
		// Mapping probabilities in the original model
		//		probs = productMdp.projectToOriginalModel(probsProduct);		
		//		//Get final prob result
		//		double maxProb=probs.getDoubleArray()[model.getFirstInitialState()];
		//		mainLog.println("\nMaximum probability to satisfy specification is " + maxProb);
		//		
		//		if (getExportProductVector()) {
		//			mainLog.println("\nExporting success probabilites over product to file \"" + PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 1) + "\"...");
		//            PrismFileLog out = new PrismFileLog(PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 1));
		//            probsProduct.print(out, false, false, false, false);
		//            out.close();
		//        }
		//				
		//		rewsProduct = StateValues.createFromDoubleArray(res.solnProg, productMdp);
		//		rews = product.projectToOriginalModel(rewsProduct); 
		//		double maxRew = rews.getDoubleArray()[model.getFirstInitialState()];
		//		mainLog.println("\nFor p = " + maxProb + ", the maximum expected progression reward is " + maxRew);
		//		
		//		if (getExportProductVector()) {
		//			mainLog.println("\nExporting expected progression reward over product to file \"" + PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 2) + "\"...");
		//            PrismFileLog out = new PrismFileLog(PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 2));
		//            rewsProduct.print(out, false, false, false, false);
		//            out.close();
		//        }
		//		
		//		costsProduct = StateValues.createFromDoubleArray(res.solnCost, productMdp);		
		//		costs = product.projectToOriginalModel(costsProduct);	
		//		double minCost = costs.getDoubleArray()[model.getFirstInitialState()];
		//		mainLog.println("\nFor p = " + maxProb + ", r = " +  + maxRew + " the minimum expected cummulative cost until no more progression reward can be gathered is " + minCost);
		////		System.out.println("Probability to find objects: " + maxProb);
		////		System.out.println("Expected progression reward: " + maxRew);
		////		System.out.println("Expected time to execute task: " + minCost);
		////		System.out.println("--------------------------------------------------------------");
		//        if (getExportProductVector()) {
		//        	mainLog.println("\nExporting expected times until no more progression over product to file \"" + PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 3) + "\"...");
		//            PrismFileLog out = new PrismFileLog(PrismUtils.addCounterSuffixToFilename(getExportProductVectorFilename(), 3));
		//            costsProduct.print(out, false, false, false, false);
		//            out.close();
		//        }
		//        List<State> statesList = productMdp.getStatesList();
		//    	HashMap<State,Double> probValues = new HashMap<State,Double>();
		//		HashMap<State,Double> costValues = new HashMap<State,Double>();
		//		for(int i = 0; i<statesList.size(); i++)
		//		{
		//			State s = statesList.get(i); 
		//			probValues.put(s, (double)probsProduct.getValue(i));
		//			costValues.put(s,(double)costsProduct.getValue(i));
		//		}
		//		 varlist.add(productMdp.getVarList()); 
		//
		//
		//		result.add(probValues); 
		//		result.add(costValues);
		//		
		return partSatRes;
	}

	public MDStrategy getSingleRobotSolution(Prism prism, LTLModelChecker ltlMC, ExpressionProb expr, AcceptanceType[] allowedAcceptance,
			ArrayList<List<State>> allRobotsStatesList, ArrayList<VarList> varlists) throws PrismException
	{

		prism.buildModel();
		MDP mdp = (MDP) prism.getBuiltModelExplicit();

		MDPModelChecker mc = new MDPModelChecker(prism);
		mc.setGenStrat(true);
		mc.setExportAdv(true);

		Vector<BitSet> labelBS = new Vector<BitSet>();
		ProbModelChecker pmc = new ProbModelChecker(prism);

		//		LTLProduct<MDP> prod = ltlMC.constructProductMDP(pmc, mdp, expr.getExpression(), null, allowedAcceptance);
		DA<BitSet, ? extends AcceptanceOmega> tempda = ltlMC.constructDAForLTLFormula(pmc, mdp, expr.getExpression(), labelBS, allowedAcceptance);
		LTLProduct<MDP> prod = ltlMC.constructProductModel(tempda, mdp, labelBS, null);

		MDP prodmdp = prod.getProductModel();
		varlists.add(prodmdp.getVarList());
		allRobotsStatesList.add(prodmdp.getStatesList());
		BitSet acc = ((AcceptanceReach) prod.getAcceptance()).getGoalStates();

		ModelCheckerResult result = mc.computeReachProbs(prodmdp, acc, false);
		MDStrategy strat = (MDStrategy) (result.strat);
		return strat;
	}

	public ArrayList<HashMap<State, Double>> getSingleRobotPartialSatSol(String filename, Prism prism, LTLModelChecker ltlMC, ExpressionProb expr,
			ModelInfo modelInfo, PropertiesFile propertiesFile, ModelGenerator modelGen, ArrayList<VarList> varlist) throws PrismException
	{
		prism.buildModel();
		MDP mdp = (MDP) prism.getBuiltModelExplicit();

		MDPModelChecker mc = new MDPModelChecker(prism);

		mc.setGenStrat(true);
		mc.setExportAdv(true);

		mc.setModulesFileAndPropertiesFile(modelInfo, propertiesFile, modelGen);
		boolean exportAdv = true;
		String savePlace = TESTSLOC + filename + "_partsat";

		ArrayList<HashMap<State, Double>> result = mc.checkPartialSatForBounds(mdp, expr.getExpression(), null, varlist, exportAdv, savePlace);

		return result;

	}

	public void doPartialSatOnly(String saveplace, String filename, Prism prism)
	{
		String modfile = saveplace + filename + ".prism";
		String propfile = saveplace + filename + ".props";
		ModulesFile modulesFile;
		try {
			modulesFile = prism.parseModelFile(new File(modfile));
			prism.loadPRISMModel(modulesFile);
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(propfile));
			ExpressionProb expr = (ExpressionProb) propertiesFile.getProperty(0);

			//

			LTLModelChecker ltlMC = new LTLModelChecker(prism);

			ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, prism);

			prism.loadModelGenerator(prismModelGen);
			ArrayList<VarList> vlt = new ArrayList<VarList>();

			ArrayList<HashMap<State, Double>> probCostValues = getSingleRobotPartialSatSol(filename, prism, ltlMC, expr, modulesFile, propertiesFile,
					prismModelGen, vlt);

		} catch (FileNotFoundException | PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	public void doBRTDP() throws Exception
	{
		try {

			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
			String filename = "tiny_example_permtrap_noun";//"no_door_example";
			ArrayList<ArrayList<HashMap<State, Double>>> probCostBoundsInits = new ArrayList<ArrayList<HashMap<State, Double>>>();
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
			String propfilename = saveplace + filename + ".props";
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
			//			MDP mdp = null;
			int maxStates = 0;
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

				MDStrategy strat = getSingleRobotSolution(prism,
						ltlMC, expr, allowedAcceptance,
						allRobotsStatesList, varlists);

				singleRobotSolutions.add(strat);

				//					Result result = mc.check(mdp, expr);//mc.computeReachProbs((MDP)prodmdp.getProductModel(), target, min)

				//					System.out.println(result.getResult()); 
				//				}
				da.setDistancesToAcc();
				da.printDot(mainLog);
				if (!TURNOFFALLWRITES) {
					PrismFileLog out = new PrismFileLog(TESTSLOC + filename + "_mrmctsda" + fileno + ".dot");
					da.printDot(out);
					out.close();
				}
				ModulesFileModelGenerator prismModelGen = new ModulesFileModelGenerator(modulesFile, prism);

				ProductModelGenerator prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
				prism.loadModelGenerator(prismModelGen);
				ArrayList<VarList> vlt = new ArrayList<VarList>();
				ArrayList<HashMap<State, Double>> probCostValues = getSingleRobotPartialSatSol(filename + fileno, prism, ltlMC, expr, modulesFile,
						propertiesFile, prismModelGen, vlt);

				if (probCostValues.get(0).size() > maxStates)
					maxStates = probCostValues.get(0).size();
				prodModGens.add(prodModelGen);
				// comparing indices of the mdp and prodmodgen stuff
				VarList vl = vlt.get(0);//varlists.get(0);
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
					//					flipedIndices.add(true);
					//fix stuff here  
					//restructure probcostinits so that it matches stuff 

					//basically for all the states, force it to be what is in the pmdval 
					//so get the current index of the variable 
					//get the index of the variable in pmdval 
					//and for each state just switch 
					HashMap<String, Entry<Integer, Integer>> vlTopmdvl = new HashMap<String, Entry<Integer, Integer>>();
					for (int i = 0; i < vl.getNumVars(); i++) {
						String name = vl.getName(i);
						if (name.contains("da")) {
							if (name.equalsIgnoreCase("_da"))
								name = "_da0";
						}
						int pmdvlIndex = pmdvl.getIndex(name);
						vlTopmdvl.put(name, new AbstractMap.SimpleEntry<Integer, Integer>(i, pmdvlIndex));
					}
					//now that we have this mapping, lets fix it for all our states 
					ArrayList<HashMap<State, Double>> newProbCostValues = new ArrayList<HashMap<State, Double>>();
					for (int i = 0; i < probCostValues.size(); i++) {
						HashMap<State, Double> updatedValues = new HashMap<State, Double>();
						HashMap<State, Double> oldvalues = probCostValues.get(i);
						//now update the values 
						for (State s : oldvalues.keySet()) {
							Object[] varVals = s.varValues;
							//switching state values as per instructions 
							State newS = new State(vl.getNumVars());
							for (String n : vlTopmdvl.keySet()) {
								Entry<Integer, Integer> p = vlTopmdvl.get(n);
								newS.setValue(p.getValue(), varVals[p.getKey()]);
							}
							updatedValues.put(newS, oldvalues.get(s));
						}
						newProbCostValues.add(updatedValues);
					}
					probCostValues = newProbCostValues;
					flipedIndices.add(false);
					//					flipedIndices.add(false);
				} else {
					flipedIndices.add(false);
					//					flipedIndices.add(true);
				}
				probCostBoundsInits.add(probCostValues);

			}

			// now do something with these
			BitSet acc = da.getAccStates();
			BitSet sinkStates = da.getSinkStates();
			int max_rollouts = 1000;
			// the nodoorexample has 8 steps for 1 robot
			// so lets do 10 steps + slack = 30
			int rollout_depth = 30;
			//			boolean minCost = false;
			ArrayList<String> sharedStatesNames = null;
			if (filename.contains("door")) {
				sharedStatesNames = new ArrayList<String>();
				sharedStatesNames.add("door");
			}
			MRuctPaper brtdp = new MRuctPaper(prism.getLog(), prodModGens, max_rollouts, rollout_depth, sharedStatesNames, da.getDistsToAcc(),
					singleRobotSolutions, allRobotsStatesList, flipedIndices, da, probCostBoundsInits);

			//			double epsilon = 10e-5;
			//			TURNOFFALLWRITES=true;
			//			for(int i = 0; i<100; i++) {
//			int i = 0;

			brtdp.doBRTDP(filename, TESTSLOC, acc, false, maxStates);
			
			

//			brtdp.brtdpPolicy.clearRewards();
//			brtdp.brtdpPolicy.createCostRewardStructure();
//			brtdp.brtdpPolicy.createProgressionRewards();
//			brtdp.brtdpPolicy.progressionTrim();
//			brtdp.brtdpPolicy.jointMDP.findDeadlocks(true);
//			HashMap<State, HashMap<String, Double>> resStuff = null;
//			resStuff = doPartialSatNVI(prism, filename + "_" + i + "l", brtdp.brtdpPolicy.jointMDP, brtdp.brtdpPolicy.accStates, brtdp.brtdpPolicy.progRewards,
//					brtdp.brtdpPolicy.costRewards, brtdp.brtdpPolicy.progStates, resStuff, false);
//			BitSet upperBoundSet = (BitSet) brtdp.brtdpPolicy.accStates.clone();
//			upperBoundSet.or(brtdp.brtdpPolicy.leafStates);
//			resStuff = doPartialSatNVI(prism, filename + "_" + i + "u", brtdp.brtdpPolicy.jointMDP, upperBoundSet, brtdp.brtdpPolicy.progRewards,
//					brtdp.brtdpPolicy.costRewards, brtdp.brtdpPolicy.progStates, resStuff, true);
//			brtdp.brtdpPolicy.resetStateSolved();
			//			if(brtdp.brtdpPolicy.doStateValuesDiff(epsilon))
			//				break;
			//			brtdp.brtdpPolicy.saveOldStateValues();
			//			brtdp.brtdpPolicy.updateProbCostValues(resStuff);
			//			}
			//			
			//			brtdp.brtdpPolicy.clearRewards();
			//			brtdp.brtdpPolicy.createCostRewardStructure();
			//			brtdp.brtdpPolicy.createProgressionRewards();
			//			brtdp.brtdpPolicy.progressionTrim();
			//			brtdp.brtdpPolicy.jointMDP.findDeadlocks(true);
			//			
			//			doPartialSatNVI(prism, filename+"_final",brtdp.brtdpPolicy.jointMDP, brtdp.brtdpPolicy.accStates, brtdp.brtdpPolicy.progRewards, brtdp.brtdpPolicy.costRewards,
			//					brtdp.brtdpPolicy.progStates,null,false);
			//			
			long endTime = System.currentTimeMillis();
			mainLog.println("\nTotal Time:" + (endTime - startTime) / 1000.0 + "s");
			// Close down PRISM
//			doPartialSatOnly(saveplace,filename+"_prod",prism);
			prism.closeDown();

		} catch (PrismException | FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
	}

	public void run() throws Exception
	{
		//	doUCT();
		doBRTDP();
	}
}