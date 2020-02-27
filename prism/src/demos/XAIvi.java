package demos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.MDPSimple;
import explicit.MDPSparse;
import explicit.Model;
import explicit.ModelCheckerPartialSatResult;
import explicit.StateValues;
import explicit.ProbModelChecker.TermCrit;
import explicit.rewards.MDPRewards;
import explicit.rewards.MDPRewardsSimple;
import explicit.rewards.Rewards;
import parser.ast.Expression;
import parser.ast.ExpressionBinaryOp;
import parser.ast.ExpressionReward;
import parser.ast.ExpressionTemporal;
import parser.ast.ExpressionUnaryOp;
import parser.ast.ModulesFile;
import parser.ast.RewardStruct;
import parser.type.TypeBool;
import parser.type.TypePathBool;
import prism.Prism;
import prism.PrismException;
import prism.PrismLog;
import prism.PrismUtils;
import strat.MDStrategyArray;

public class XAIvi {

	public HashMap<Integer, HashMap<Integer, Double>> stateActionProbValues = new HashMap<Integer, HashMap<Integer, Double>>();
	public HashMap<Integer, HashMap<Integer, Double>> stateActionProgValues = new HashMap<Integer, HashMap<Integer, Double>>();
	public HashMap<Integer, HashMap<Integer, Double>> stateActionCostValues = new HashMap<Integer, HashMap<Integer, Double>>();

	MDP productmdp;
	protected TermCrit termCrit = TermCrit.RELATIVE;
	// Parameter for iterative numerical method termination criteria
	protected double termCritParam = 1e-8;
	MDPRewardsSimple progRewards;
	MDPRewardsSimple costRewards;

	BitSet accStates;

	BitSet daSinkStates;
	DA<BitSet, ? extends AcceptanceOmega> origda;
	Vector<BitSet> origdaLabelBS;

	HashMap<Expression, Integer> actualExprNumMap = null;
	
	ArrayList<ArrayList<XAIMdpOption>> optionsList = null;

	public BitSet getSinkStates(DA<BitSet, ? extends AcceptanceOmega> da) {
		BitSet daaccStates;
		// Check if its a DFA
		if (da.getAcceptance() instanceof AcceptanceReach) {
			daaccStates = ((AcceptanceReach) da.getAcceptance()).getGoalStates();
		} else if (da.getAcceptance() instanceof AcceptanceRabin) {
			daaccStates = da.getRabinAccStates();
		} else {
			return null;
		}

		BitSet sinkStates = new BitSet();
		for (int i = 0; i < da.size(); i++) {
			boolean isSinkState = false;
			int numEdges = da.getNumEdges(i);
			int numEdgesSrcToSrc = da.getNumEdges(i, i);
			if (numEdges == numEdgesSrcToSrc) {
				isSinkState = true;

			}
			if (numEdges == 0)
				isSinkState = true;
			if (isSinkState) {
				if (daaccStates.get(i) == false)
					sinkStates.set(i);
			}
		}
		return sinkStates;
	}

	public boolean saveExprNumInMap(Expression expr, Expression opExpr, HashMap<Expression, Integer> exprNumMap,
			HashMap<Expression, Integer> actualExprNumMap) {
		boolean saved = false;
		if (opExpr.getType() instanceof TypeBool) {
			if (!exprNumMap.containsKey(opExpr)) {
				Expression exprC = opExpr.deepCopy();
				Expression.isNot(exprC);
				if (!exprNumMap.containsKey(exprC)) {
					exprNumMap.put(opExpr, exprNumMap.size());
				} else {
					exprNumMap.put(opExpr, exprNumMap.get(exprC));
				}
			}
			actualExprNumMap.put(expr, exprNumMap.get(opExpr));
			saved = true;
		}
		return saved;
	}

	public void tryingToGetExprConversionPrism(Expression ltl, HashMap<Expression, Integer> exprNumMap,
			HashMap<Expression, Integer> actualExprNumMap) {
		// copied from ltl model checker and modified
		Expression expr = ltl;
		if (!saveExprNumInMap(expr, expr, exprNumMap, actualExprNumMap)) {
//			//there is no checking for a repeated not 
//			if (!exprNumMap.containsKey(expr)) {
//				Expression exprC = expr.deepCopy();
//				Expression.isNot(exprC);
//				if (!exprNumMap.containsKey(exprC)) {
//					exprNumMap.put(expr, exprNumMap.size());
//				} else {
//					exprNumMap.put(expr, exprNumMap.get(exprC));
//				}
//			}
//			actualExprNumMap.put(expr, exprNumMap.get(expr));
//			
//		} else {
			// A path formula (recurse, modify, return)
			if (expr.getType() instanceof TypePathBool) {
				if (expr instanceof ExpressionBinaryOp) {
					ExpressionBinaryOp exprBinOp = (ExpressionBinaryOp) expr;
					if (!saveExprNumInMap(expr, exprBinOp.getOperand1(), exprNumMap, actualExprNumMap)) {

						tryingToGetExprConversionPrism(exprBinOp.getOperand1(), exprNumMap, actualExprNumMap);
					}
					if (!saveExprNumInMap(expr, exprBinOp.getOperand2(), exprNumMap, actualExprNumMap)) {
						tryingToGetExprConversionPrism(exprBinOp.getOperand2(), exprNumMap, actualExprNumMap);
					}
				} else if (expr instanceof ExpressionUnaryOp) {
					ExpressionUnaryOp exprUnOp = (ExpressionUnaryOp) expr;
					if (!saveExprNumInMap(expr, exprUnOp.getOperand(), exprNumMap, actualExprNumMap)) {
						tryingToGetExprConversionPrism(exprUnOp.getOperand(), exprNumMap, actualExprNumMap);
					}
				} else if (expr instanceof ExpressionTemporal) {
					ExpressionTemporal exprTemp = (ExpressionTemporal) expr;
					if (exprTemp.getOperand1() != null) {
						if (!saveExprNumInMap(expr, exprTemp.getOperand1(), exprNumMap, actualExprNumMap)) {
							tryingToGetExprConversionPrism(exprTemp.getOperand1(), exprNumMap, actualExprNumMap);
						}
					}
					if (exprTemp.getOperand2() != null) {
						if (!saveExprNumInMap(expr, exprTemp.getOperand2(), exprNumMap, actualExprNumMap)) {

							tryingToGetExprConversionPrism(exprTemp.getOperand2(), exprNumMap, actualExprNumMap);
						}
					}
				}
			}
		}
	}

	public ArrayList<ArrayList<XAIMdpOption>> doOptionsVI(DA<BitSet, ? extends AcceptanceOmega> da, LTLModelChecker.LTLProduct<MDP> product,
			int maxIters, PrismLog mainLog, MDPModelChecker mc, MDPRewards prodCosts, double discount, String saveplace,
			Vector<BitSet> labelBS) throws PrismException {

		
		ArrayList<ArrayList<XAIMdpOption>> optionsWorld = new ArrayList<ArrayList<XAIMdpOption>>();
		MDP productMDP = (MDP) product.getProductModel();
		int numStatesprod = productMDP.getNumStates();
		// doing this vi here
		// first we get all the states that are acc states
		// so we get the bitsets for each da state
		Vector<BitSet> daBSs = new Vector<BitSet>();
		ArrayList<ArrayList<Integer>> daStates = new ArrayList<ArrayList<Integer>>();
		for (int daS = 0; daS < da.size(); daS++) {
			daBSs.add(new BitSet());
			daStates.add(new ArrayList<Integer>());
		}

		int currentautomatonState;

		for (int productState = 0; productState < numStatesprod; productState++) {
			currentautomatonState = product.getAutomatonState(productState);
			
			daBSs.get(currentautomatonState).set(productState);
			daStates.get(currentautomatonState).add(productState);
		}
		// now we can do vi for each of these in turn
		for (int daS = 0; daS < da.size(); daS++) {
			for (int daS2 = 0; daS2 < da.size(); daS2++) {
				if (daS != daS2) {
					if(daS == 1 && daS2 == 2)
					{
						System.out.println("Debug here");
					}
					if (da.getNumEdges(daS, daS2) > 0) {
						BitSet accDA = daBSs.get(daS2);
						// then you go and like do a thing for all the ones you'd like to ignore
						// so we basically ignore everything thats not daS or daS2
						BitSet daIgnore = new BitSet();
						for (int daIg = 0; daIg < da.size(); daIg++) {
							if (daIg != daS) {
								if (daIg != daS2) {
									daIgnore.or(daBSs.get(daIg));
								}
							}
						}

						ModelCheckerPartialSatResult res = computeValIter(maxIters, mainLog, mc, productMDP, accDA,
								prodCosts, discount, daIgnore);
						// create options
						// start with the initial states as in daS
						int initState = daBSs.get(daS).nextSetBit(0);

						ArrayList<XAIMdpOption> alloptions = new ArrayList<XAIMdpOption>();
						XAIMdpOption anoption = null;
						int optionNum = 0;
						while (initState != -1) {
							// okay this might change
							// but for now it is this
							// so like this is our first option
							// we'll trace this till we reach daS2
							int currentState = initState;
							Queue<Integer> tovisit = new LinkedList<Integer>();
							tovisit.add(currentState);

							while (!tovisit.isEmpty()) {
								currentState = tovisit.remove();
								res.strat.initialise(currentState);
								Object action = res.strat.getChoiceAction();
								if (currentState == initState) {
									if (anoption != null) {
//										if (!anoption.actions.contains(action)) {
//											// check if any of our other options have this
//											boolean uniqueoption = true;
//											for (XAIMdpOption option : alloptions) {
//												if (option.actions.contains(action)) {
//													uniqueoption = false;
//													anoption = option;
//													break;
//												}
//
//											}
//											if (uniqueoption) {
//												if (!alloptions.contains(anoption)) {
//													alloptions.add(anoption);
//												}
//												optionNum++;
//												anoption = new XAIMdpOption("" + optionNum);
//											}

//										}
										alloptions.add(anoption);
										optionNum++;
										anoption = new XAIMdpOption("" + optionNum);
										if(daSinkStates.get(daS2))
										{
											anoption.leadsToSinkState=true;
										}
									} else {
										anoption = new XAIMdpOption("" + optionNum);
										if(daSinkStates.get(daS2))
										{
											anoption.leadsToSinkState=true;
										}
									}
								}
								if (action != null) {
									boolean addState = true;
									// find action
									int numChoices = productMDP.getNumChoices(currentState);
									// find the action
									for (int nc = 0; nc < numChoices; nc++) {
										if (productMDP.getAction(currentState, nc).toString()
												.contentEquals(action.toString())) {
											Iterator<Entry<Integer, Double>> tranIter = productMDP
													.getTransitionsIterator(currentState, nc);
											while (tranIter.hasNext()) {
												Entry<Integer, Double> stateprob = tranIter.next();
												int nextstate = stateprob.getKey();
												if (!daIgnore.get(nextstate) && !accDA.get(nextstate)) {
													tovisit.add(nextstate);
//													addState = false;
												} else {
//													// hmmm we should get the labels for this state and the next one
//													Set<String> allLabels = productMDP.getLabels();
//													// for each label in the set get states that correspond to this
//													// label
//													for (String label : allLabels) {
//														BitSet labelStates = productMDP.getLabelStates(label);
//														if (labelStates.get(currentState)) {
//															System.out.println(currentState + ":" + label);
//														}
//														if (labelStates.get(nextstate)) {
//
//															System.out.println(
//																	currentState + ":" + label + ":" + nextstate);
//
//														}
//
//													}
													if (accDA.get(nextstate)) {
														List<String> apList = da.getAPList();
														BitSet s_labels = new BitSet();
														for (int k = 0; k < apList.size(); k++) {
															s_labels.set(k,
																	labelBS.get(Integer.parseInt(
																			da.getAPList().get(k).substring(1)))
																			.get(product.getModelState(nextstate)));
														}
														// Find corresponding successor in DA
														int q_2 = da.getEdgeDestByLabel(daS, s_labels);
														List<BitSet> allEdgeLabels = da.getEdgeLabels(daS, q_2);
														for (BitSet edgeLabel : allEdgeLabels) {
															if (edgeLabel.equals(s_labels)) {
																System.out
																		.println("The following labels are satisfied: "
																				+ s_labels.toString());
																anoption.labels.add(s_labels);
																break;
															}
														}
													}
													anoption.terminationStates
															.add(productMDP.getStatesList().get(nextstate));
												}
											}
											break;
										}
									}
									if (addState) {

										anoption.stateActionList.put(productMDP.getStatesList().get(currentState),
												action);
										if (!anoption.actions.contains(action))
											anoption.actions.add(action);
									}
								}
							}
							while (anoption.stateActionList.containsKey(productMDP.getStatesList().get(initState))) {
								initState = daBSs.get(daS).nextSetBit(initState + 1);
								if (initState == -1)
									break;

							}
						}
						// now lets look at all options
						// and combine ones with the same label as others
						ArrayList<XAIMdpOption> combinedOptions = new ArrayList<XAIMdpOption>();
						int[] matchedoptions = new int[alloptions.size()];
						Arrays.fill(matchedoptions, -1);
						for (int o1 = 0; o1 < alloptions.size(); o1++) {
							XAIMdpOption option1 = alloptions.get(o1);
							for (int o2 = o1 + 1; o2 < alloptions.size(); o2++) {
								XAIMdpOption option2 = alloptions.get(o2);
								if (option1.labels.containsAll(option2.labels)
										&& option2.labels.containsAll(option1.labels)) {
//									if (option1.actions.containsAll(option2.actions)
//											&& option2.actions.containsAll(option1.actions)) {
//										System.out.println("Combine " + option1.name + " " + option2.name);
										if (matchedoptions[o2] == -1) {
											matchedoptions[o2] = o1;
										} else {
											if (matchedoptions[o2] != matchedoptions[o1]) {
												System.out.println("Strange");
											}
										}
//									}
								}

							}

						}
						System.out.println(Arrays.toString(matchedoptions));
						// now we do the options stuff
						// like basically we go over the array start with the first non -1 one and
						// combine those two.
						for (int i = 0; i < matchedoptions.length; i++) {
							if (matchedoptions[i] != -1) {

								XAIMdpOption option1 = null;// = alloptions.get(i);
								XAIMdpOption option2 = null;// = alloptions.get(matchedoptions[i]);

								option1 = alloptions.get(i);
//									XAIMdpOption
								option2 = alloptions.get(matchedoptions[i]);

								option2.combine(option1);
//								System.out.println("Adding option "+i+" to "+matchedoptions[i]);

							}
						}
						for(int i = 0; i<matchedoptions.length; i++)
						{
							if(matchedoptions[i]==-1)
							{
								combinedOptions.add(alloptions.get(i));
							}
						}
						
						// end at daS2
						// and ignore daIg
						// int[] daStatesHere = daStates.get(daS).toArray(new
						// Integer[daStates.get(daS).size()]);
						XAIPathCreator optimalPolicyPaths = new XAIPathCreator();
						optimalPolicyPaths.createPathPolicy(daStates.get(daS), productMDP, res.strat,
								saveplace + "results/", "xai_" + daS + "_" + daS2 + "_options",
								(MDPRewardsSimple) prodCosts, accDA);
						optionsWorld.add(combinedOptions);

					}
				}
			}
		}
		System.out.println(optionsWorld.toString());
		return optionsWorld; 

	}

	public ModelCheckerPartialSatResult nviexposed(Prism prism, PrismLog mainLog, Model model, Expression expr,
			ExpressionReward rewExpr, BitSet statesOfInterest, ModulesFile modulesFile, MDPModelChecker mc,
			double discount) throws PrismException

	{
		int maxIters = 100000;
		LTLModelChecker mcLtl;
		StateValues probsProduct, probs, costsProduct, costs, rewsProduct, rews;
		MDPModelChecker mcProduct;
		LTLModelChecker.LTLProduct<MDP> product;
		MDP productMdp;
		DA<BitSet, ? extends AcceptanceOmega> da;
		Vector<BitSet> labelBS;

		String saveplace = System.getProperty("user.dir") + "/dotfiles/";
		// For LTL model checking routines
		mcLtl = new LTLModelChecker(prism);

		// Get LTL spec
		ExpressionReward exprRew = rewExpr;
		Expression ltl = expr;// exprRew.getExpression();
		// System.out.println("--------------------------------------------------------------");
		// //System.out.println("The flat MDP model has " + model.getNumStates()
		// + " states");
		System.out.println("The specification is " + ltl.toString());
		// System.out.println("Generating optimal policy...");
		// System.out.println(" ");

		// Build model costs
		RewardStruct costStruct = exprRew.getRewardStructByIndexObject(modulesFile, modulesFile.getConstantValues());
		mainLog.println("Building cost structure...");
		Rewards costsModel = mc.constructRewards(model, costStruct);

		// build DFA
		AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
		labelBS = new Vector<BitSet>();
		da = mcLtl.constructDAForLTLFormula(mc, model, ltl, labelBS, allowedAcceptance);
		HashMap<Expression, Integer> exprNumMap = new HashMap<Expression, Integer>();
		HashMap<Expression, Integer> actualExprNumMap = new HashMap<Expression, Integer>();
		tryingToGetExprConversionPrism(ltl, exprNumMap, actualExprNumMap);
		this.actualExprNumMap = actualExprNumMap;
		if (!(da.getAcceptance() instanceof AcceptanceReach) && !(da.getAcceptance() instanceof AcceptanceRabin)) {
			mainLog.println("\nAutomaton is not a DFA. Breaking.");
			// Dummy return vector
			return null;
		}
		// calculate distances to accepting states
		long time = System.currentTimeMillis();
		da.setDistancesToAcc();
		origda = da;

		da.printDot(mainLog);
		this.daSinkStates = this.getSinkStates(da);
		time = System.currentTimeMillis() - time;
		mainLog.println("\nAutomaton state distances to an accepting state: " + da.getDistsToAcc());
		mainLog.println("Time for DFA distance to acceptance metric calculation: " + time / 1000.0 + " seconds.");

		// build product
		int numStates = model.getNumStates();
		BitSet bsInit = new BitSet(numStates);
		for (int i = 0; i < numStates; i++) {
			bsInit.set(i, model.isInitialState(i));
		}
		product = mcLtl.constructProductModel(da, (MDP) model, labelBS, bsInit);

		this.origdaLabelBS = labelBS;
		// System.out.println("The product MDP has " +
		// product.getProductModel().getNumStates() + " states");

		// Find accepting states + compute reachability probabilities
		BitSet acc;
		// TODO: change this for safety specs
		if (product.getAcceptance() instanceof AcceptanceReach) {
			mainLog.println("\nSkipping accepting MEC computation since acceptance is defined via goal states...");
			acc = ((AcceptanceReach) product.getAcceptance()).getGoalStates();
		} else {
			mainLog.println("\nFinding accepting MECs...");
			acc = mcLtl.findAcceptingECStates(product.getProductModel(), product.getAcceptance());
		}

		time = System.currentTimeMillis();
		// Build progression rewards on product
		MDPRewards progRewards = product.liftProgressionFromAutomaton(da.getDistsToAcc());
		time = System.currentTimeMillis() - time;
		mainLog.println(
				"Time for lifting progression reward from automaton to product: " + time / 1000.0 + " seconds.");

		time = System.currentTimeMillis();
		// Build trimmed product costs
		MDPRewards prodCosts = ((MDPRewards) costsModel).liftFromModel(product);
		time = System.currentTimeMillis() - time;
		mainLog.println(
				"Time for lifting cost function from original model to product: " + time / 1000.0 + " seconds.");

		this.progRewards = (MDPRewardsSimple) progRewards;
		this.costRewards = (MDPRewardsSimple) prodCosts;

		BitSet progStates = mc.progressionTrim(product, (MDPRewardsSimple) progRewards, (MDPRewardsSimple) prodCosts);

		mcProduct = mc;

		if (product.getProductModel().getNumStates() > 10000) {
			mainLog.println("\nChanging product to MDPSparse...");
			productMdp = new MDPSparse((MDPSimple) product.getProductModel());
		} else {
			productMdp = product.getProductModel();
		}

		mainLog.println("\nComputing reachability probability, expected progression, and expected cost...");
		accStates = (BitSet) acc.clone();
		ModelCheckerPartialSatResult res = computeValIter(maxIters, mainLog, mc, productMdp, acc, progRewards,
				prodCosts, progStates, true, discount);
		this.productmdp = productMdp;
		probsProduct = StateValues.createFromDoubleArray(res.solnProb, productMdp);

		// Mapping probabilities in the original model
		probs = product.projectToOriginalModel(probsProduct);
		// Get final prob result
		double maxProb = probs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nMaximum probability to satisfy specification is " + maxProb);

		rewsProduct = StateValues.createFromDoubleArray(res.solnProg, productMdp);
		rews = product.projectToOriginalModel(rewsProduct);
		double maxRew = rews.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb
				+ ", the maximum expected cummulative reward to satisfy specification is " + maxRew);

		costsProduct = StateValues.createFromDoubleArray(res.solnCost, productMdp);
		costs = product.projectToOriginalModel(costsProduct);
		double minCost = costs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb + ", r = " + +maxRew
				+ " the minimum expected  cummulative cost to satisfy specification is " + minCost);
		// System.out.println("Probability to find objects: " + maxProb);
		// System.out.println("Expected progression reward: " + maxRew);
		// System.out.println("Expected time to execute task: " + minCost);
		// System.out.println("--------------------------------------------------------------");
		ArrayList<ArrayList<XAIMdpOption>> optionsWorld = doOptionsVI(da, product, maxIters, mainLog, mc, prodCosts, discount, saveplace, labelBS);
		
//		actualExprNumMap
		//now we want to link the actual expressions to the labels in options world okay 
		//okay 
		for (int i = 0; i<optionsWorld.size(); i++)
		{
			ArrayList<XAIMdpOption> currentOptionsList = optionsWorld.get(i);
			for(int j = 0; j<currentOptionsList.size(); j++)
			{
				//now we get the expressions 
				XAIMdpOption currentOption = currentOptionsList.get(j); 
				//now we get its first label 
				Iterator<BitSet> labelIter = currentOption.labels.iterator();
				while(labelIter.hasNext())
				{
					BitSet label = labelIter.next(); 
					String labelString = "";
					String notlabelString = "";
					//for each set bit set print out its expression 
					for(Expression labexpr:actualExprNumMap.keySet())
					{
						
						int labexprint = actualExprNumMap.get(labexpr);
						if(label.get(labexprint))
						{
							if(labelString!="")
							{
								labelString+=" & ";
							}
							labelString+=labexpr.toString();
						}
						else
						{
							if(notlabelString!="")
							{
								notlabelString+=" & ";
							}
							notlabelString+=labexpr.toString();
						}
					}
					System.out.println(label.toString()+" satifies "+labelString);

					System.out.println(label.toString()+" does not satisfy "+notlabelString);
					currentOption.satisfiesString=labelString; 
					currentOption.doesnotsatisfyString = notlabelString; 
				}
				
			}
		}
		optionsList = optionsWorld; 
		return res;

		// return new
		// AbstractMap.SimpleEntry<MDP,MDStrategy>(productMdp,(MDStrategy)res.strat);

	}

	public ModelCheckerPartialSatResult doPathNVI(Prism prism, PrismLog mainLog, XAIPathCreator pc, MDPModelChecker mc,
			double discount) throws PrismException

	{
		int maxIters = 10000;
		pc.pc.mdpCreator.createRewardStructures();
		MDPSimple productMdp = pc.pc.mdpCreator.mdp;
		BitSet acc = pc.pc.mdpCreator.accStates;
		MDPRewardsSimple progRewards = pc.pc.mdpCreator.expectedTaskCompletionRewards;
		MDPRewardsSimple prodCosts = pc.pc.mdpCreator.stateActionCostRewards;

		ModelCheckerPartialSatResult res = computeValIter(maxIters, mainLog, mc, productMdp, acc, progRewards,
				prodCosts, null, false, discount);

		// Get final prob result
		double maxProb = res.solnProb[productMdp.getFirstInitialState()];
		mainLog.println("\nMaximum probability to satisfy specification is " + maxProb);

		double maxRew = res.solnProg[productMdp.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb
				+ ", the maximum expected cummulative reward to satisfy specification is " + maxRew);

		double minCost = res.solnCost[productMdp.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb + ", r = " + +maxRew
				+ " the minimum expected  cummulative cost to satisfy specification is " + minCost);

		mainLog.println(Arrays.toString(res.solnProb));
		mainLog.println(Arrays.toString(res.solnProg));
		mainLog.println(Arrays.toString(res.solnCost));
		return res;

		// return new
		// AbstractMap.SimpleEntry<MDP,MDStrategy>(productMdp,(MDStrategy)res.strat);

	}

	public ModelCheckerPartialSatResult doPathOccupancyFreq(Prism prism, PrismLog mainLog, XAIPathCreator pc,
			MDPModelChecker mc, double discount) throws PrismException

	{
		int maxIters = 10000;
		MDPSimple productMdp = pc.pc.mdpCreator.mdp;
		BitSet acc = pc.pc.mdpCreator.accStates;

		ModelCheckerPartialSatResult res = computeOccupationFrequencyForPath(maxIters, mainLog, mc, productMdp, acc,
				discount);

		return res;

		// return new
		// AbstractMap.SimpleEntry<MDP,MDStrategy>(productMdp,(MDStrategy)res.strat);

	}

	/**
	 * Compute reachability probabilities using value iteration. Optionally, store
	 * optimal (memoryless) strategy info.
	 * 
	 * @param progStates
	 * @param mdp        The MDP
	 * @param no         Probability 0 states
	 * @param yes        Probability 1 states
	 * @param min        Min or max probabilities (true=min, false=max)
	 * @param init       Optionally, an initial solution vector (will be
	 *                   overwritten)
	 * @param known      Optionally, a set of states for which the exact answer is
	 *                   known
	 * @param strat      Storage for (memoryless) strategy choice indices (ignored
	 *                   if null) Note: if 'known' is specified (i.e. is non-null,
	 *                   'init' must also be given and is used for the exact values.
	 */
	protected ModelCheckerPartialSatResult computeValIter(int maxIters, PrismLog mainLog, MDPModelChecker mc,
			MDP trimProdMdp, BitSet target, MDPRewards progRewards, MDPRewards prodCosts, BitSet progStates,
			boolean saveVals, double discount) throws PrismException {
		ModelCheckerPartialSatResult res;
		int i, n, iters, numYes, numNo;
		double initValProb, initValRew, initValCost;
		// if (saveVals) {
		// stateActionProbValues = new HashMap<Integer, HashMap<Integer, Double>>();
		// stateActionProgValues = new HashMap<Integer, HashMap<Integer, Double>>();
		// stateActionCostValues = new HashMap<Integer, HashMap<Integer, Double>>();
		// }
		double solnProb[], soln2Prob[];
		double solnProg[], soln2Prog[];
		double solnCost[], soln2Cost[];
		double influenceValues[];
		boolean done;
		BitSet no, yes, unknown;
		long timerVI, timerProb0, timerProb1, timerGlobal;
		int strat[] = null;
		boolean min = false;

		timerGlobal = System.currentTimeMillis();

		// Check for deadlocks in non-target state (because breaks e.g. prob1)
		trimProdMdp.checkForDeadlocks(target);

		// Store num states
		n = trimProdMdp.getNumStates();

		if (progStates == null) {
			progStates = new BitSet(n);
			progStates.flip(0, n);
		}
		strat = new int[n];
		for (i = 0; i < n; i++) {
			strat[i] = target.get(i) ? -2 : -1;
		}
		// }

		// Precomputation
		timerProb0 = System.currentTimeMillis();
		// if (precomp && prob0) {
		no = mc.prob0(trimProdMdp, null, target, min, strat);
		// } else {
		// no = new BitSet();
		// }
		timerProb0 = System.currentTimeMillis() - timerProb0;
		timerProb1 = System.currentTimeMillis();
		// if (precomp && prob1) {
		yes = mc.prob1(trimProdMdp, null, target, min, strat);
		// } else {
		// yes = (BitSet) target.clone();
		// }
		timerProb1 = System.currentTimeMillis() - timerProb1;

		// Print results of precomputation
		numYes = yes.cardinality();
		numNo = no.cardinality();
		mainLog.println("target=" + target.cardinality() + ", yes=" + numYes + ", no=" + numNo + ", maybe="
				+ (n - (numYes + numNo)));

		if (min) {
			for (i = yes.nextSetBit(0); i >= 0; i = yes.nextSetBit(i + 1)) {
				if (!target.get(i))
					strat[i] = -2;
			}
		} else {
			for (i = no.nextSetBit(0); i >= 0; i = no.nextSetBit(i + 1)) {
				strat[i] = -2;
			}
		}
		// }

		// Start value iteration
		timerVI = System.currentTimeMillis();
		mainLog.println("Starting prioritised value iteration (" + (min ? "min" : "max") + ")...");

		// Create solution vector(s)
		solnProb = new double[n];
		// soln2Prob = new double[n];
		solnProg = new double[n];
		// soln2Prog = new double[n];
		solnCost = new double[n];
		// soln2Cost = new double[n];

		influenceValues = new double[n];
		// Initialise solution vectors to initVal
		// where initVal is 0.0 or 1.0, depending on whether we converge from
		// below/above.
		initValProb = 0.0;
		initValRew = 0.0;
		initValCost = 0.0;

		// (valIterDir == ValIterDir.BELOW) ? 0.0 : 1.0;

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(yes);
		unknown.andNot(no);
		for (i = 0; i < n; i++) {
			// solnProb[i] = soln2Prob[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 :
			// initValProb;
			// solnProg[i] = soln2Prog[i] = initValRew;
			// solnCost[i] = soln2Cost[i] = initValCost;
			solnProb[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : initValProb;
			solnProg[i] = initValRew;
			solnCost[i] = target.get(i) ? 0.0 : n * 1000;// initValCost;
		}

		// Start iterations
		iters = 0;
		done = false;

		int j;
		int numChoices;
		double currentProbVal, currentProgVal, currentCostVal;
		boolean sameProb, sameProg, sameCost;

		while (!done && iters < maxIters) {
			iters++;
			done = true;
			for (i = 0; i < n; i++) {

				boolean setIS = true;
				double maxIS = -1;
				double minIS = -1;
				// if (progStates.get(i)) {
				if (!target.get(i)) {
					// if (saveVals) {
					// if (!stateActionProbValues.containsKey(i))
					// stateActionProbValues.put(i, new HashMap<Integer, Double>());
					// if (!stateActionProgValues.containsKey(i))
					// stateActionProgValues.put(i, new HashMap<Integer, Double>());
					// if (!stateActionCostValues.containsKey(i))
					// stateActionCostValues.put(i, new HashMap<Integer, Double>());
					// }
					numChoices = trimProdMdp.getNumChoices(i);
					for (j = 0; j < numChoices; j++) {

						currentProbVal = trimProdMdp.mvMultJacSingle(i, j, solnProb);
						currentProgVal = trimProdMdp.mvMultRewSingle(i, j, solnProg, progRewards);
						currentCostVal = mvMultRewSingleDiscount(trimProdMdp, i, j, solnCost, prodCosts, discount);
						sameProb = PrismUtils.doublesAreClose(currentProbVal, solnProb[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						sameProg = PrismUtils.doublesAreClose(currentProgVal, solnProg[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						sameCost = PrismUtils.doublesAreClose(currentCostVal, solnCost[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						// if (saveVals) {
						//
						// stateActionProbValues.get(i).put(j, currentProbVal);
						// stateActionProgValues.get(i).put(j, currentProgVal);
						// stateActionCostValues.get(i).put(j, currentCostVal);
						// }

						if (setIS) {
							maxIS = currentCostVal;
							minIS = currentCostVal;
							setIS = false;
						}
						if (!setIS) {
							if (currentCostVal > maxIS)
								maxIS = currentCostVal;
							if (currentCostVal < minIS)
								minIS = currentCostVal;

						}
						if (!sameCost && currentCostVal < solnCost[i]) {
							done = false;
							solnProb[i] = currentProbVal;
							solnProg[i] = currentProgVal;
							solnCost[i] = currentCostVal;

							strat[i] = j;

						}
						// }
						// }
						// }
						// }
					}
				}
				influenceValues[i] = maxIS - minIS;
			}

		}

		// Finished value iteration
		timerVI = System.currentTimeMillis() - timerVI;
		mainLog.print("Prioritised value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timerVI / 1000.0 + " seconds.");

		timerGlobal = System.currentTimeMillis() - timerGlobal;
		mainLog.println("Overall policy calculation took  " + timerGlobal / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done /* && errorOnNonConverge */) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		res = new ModelCheckerPartialSatResult();

		res.strat = new MDStrategyArray(trimProdMdp, strat);

		// Return results
		res.solnProb = solnProb;
		res.solnProg = solnProg;
		res.solnCost = solnCost;
		res.numIters = iters;
		res.timeTaken = timerGlobal / 1000.0;
		res.lastSolnCost = influenceValues;
		return res;
	}

	/*
	 * Compute reachability probabilities using value iteration. Optionally, store
	 * optimal (memoryless) strategy info.
	 * 
	 * @param progStates
	 * 
	 * @param mdp The MDP
	 * 
	 * @param no Probability 0 states
	 * 
	 * @param yes Probability 1 states
	 * 
	 * @param min Min or max probabilities (true=min, false=max)
	 * 
	 * @param init Optionally, an initial solution vector (will be overwritten)
	 * 
	 * @param known Optionally, a set of states for which the exact answer is known
	 * 
	 * @param strat Storage for (memoryless) strategy choice indices (ignored if
	 * null) Note: if 'known' is specified (i.e. is non-null, 'init' must also be
	 * given and is used for the exact values.
	 */
	protected ModelCheckerPartialSatResult computeValIter(int maxIters, PrismLog mainLog, MDPModelChecker mc,
			MDP trimProdMdp, BitSet target, MDPRewards prodCosts, double discount, BitSet ignore)
			throws PrismException {
		ModelCheckerPartialSatResult res;
		int i, n, iters, numYes, numNo;
		double initValProb, initValRew, initValCost;

		double solnProb[], soln2Prob[];
		double solnProg[], soln2Prog[];
		double solnCost[], soln2Cost[];
		double influenceValues[];
		boolean done;
		BitSet no, yes, unknown;
		long timerVI, timerProb0, timerProb1, timerGlobal;
		int strat[] = null;
		boolean min = false;

		timerGlobal = System.currentTimeMillis();

		// Check for deadlocks in non-target state (because breaks e.g. prob1)
		trimProdMdp.checkForDeadlocks(target);

		// Store num states
		n = trimProdMdp.getNumStates();

		strat = new int[n];
		for (i = 0; i < n; i++) {
			strat[i] = target.get(i) ? -2 : -1;
		}
		// }

		// Precomputation
		timerProb0 = System.currentTimeMillis();
		// if (precomp && prob0) {
		no = mc.prob0(trimProdMdp, null, target, min, strat);
		// } else {
		// no = new BitSet();
		// }
		timerProb0 = System.currentTimeMillis() - timerProb0;
		timerProb1 = System.currentTimeMillis();
		// if (precomp && prob1) {
		yes = mc.prob1(trimProdMdp, null, target, min, strat);
		// } else {
		// yes = (BitSet) target.clone();
		// }
		timerProb1 = System.currentTimeMillis() - timerProb1;

		// Print results of precomputation
		numYes = yes.cardinality();
		numNo = no.cardinality();
		mainLog.println("target=" + target.cardinality() + ", yes=" + numYes + ", no=" + numNo + ", maybe="
				+ (n - (numYes + numNo)));

		if (min) {
			for (i = yes.nextSetBit(0); i >= 0; i = yes.nextSetBit(i + 1)) {
				if (!target.get(i))
					strat[i] = -2;
			}
		} else {
			for (i = no.nextSetBit(0); i >= 0; i = no.nextSetBit(i + 1)) {
				strat[i] = -2;
			}
		}
		// }

		// Start value iteration
		timerVI = System.currentTimeMillis();
		mainLog.println("Starting prioritised value iteration (" + (min ? "min" : "max") + ")...");

		// Create solution vector(s)
		solnProb = new double[n];
		// soln2Prob = new double[n];
		solnProg = new double[n];
		// soln2Prog = new double[n];
		solnCost = new double[n];
		// soln2Cost = new double[n];

		influenceValues = new double[n];
		// Initialise solution vectors to initVal
		// where initVal is 0.0 or 1.0, depending on whether we converge from
		// below/above.
		initValProb = 0.0;
		initValRew = 0.0;
		initValCost = 0.0;

		// (valIterDir == ValIterDir.BELOW) ? 0.0 : 1.0;

		// Determine set of states actually need to compute values for
		unknown = new BitSet();
		unknown.set(0, n);
		unknown.andNot(yes);
		unknown.andNot(no);
		for (i = 0; i < n; i++) {

			solnProb[i] = yes.get(i) ? 1.0 : no.get(i) ? 0.0 : initValProb;
			solnProg[i] = initValRew;
			solnCost[i] = target.get(i) ? 0.0 : n * 1000;// initValCost;
		}

		// Start iterations
		iters = 0;
		done = false;

		int j;
		int numChoices;
		double currentProbVal, currentProgVal, currentCostVal;
		boolean sameProb, sameProg, sameCost;

		while (!done && iters < maxIters) {
			iters++;
			done = true;
			for (i = 0; i < n; i++) {

				if (ignore.get(i))
					continue;
				boolean setIS = true;
				double maxIS = -1;
				double minIS = -1;
				if (!target.get(i)) {

					numChoices = trimProdMdp.getNumChoices(i);
					for (j = 0; j < numChoices; j++) {

						currentProbVal = trimProdMdp.mvMultJacSingle(i, j, solnProb);
//						currentProgVal = trimProdMdp.mvMultRewSingle(i, j, solnProg, progRewards);
						currentCostVal = mvMultRewSingleDiscount(trimProdMdp, i, j, solnCost, prodCosts, discount);
						sameProb = PrismUtils.doublesAreClose(currentProbVal, solnProb[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);

						sameCost = PrismUtils.doublesAreClose(currentCostVal, solnCost[i], termCritParam,
								termCrit == TermCrit.ABSOLUTE);
						// if (saveVals) {
						//
						// stateActionProbValues.get(i).put(j, currentProbVal);
						// stateActionProgValues.get(i).put(j, currentProgVal);
						// stateActionCostValues.get(i).put(j, currentCostVal);
						// }

						if (setIS) {
							maxIS = currentCostVal;
							minIS = currentCostVal;
							setIS = false;
						}
						if (!setIS) {
							if (currentCostVal > maxIS)
								maxIS = currentCostVal;
							if (currentCostVal < minIS)
								minIS = currentCostVal;

						}
						if (!sameCost && currentCostVal < solnCost[i]) {
							done = false;
							solnProb[i] = currentProbVal;
//							solnProg[i] = currentProgVal;
							solnCost[i] = currentCostVal;

							strat[i] = j;

						}
						// }
						// }
						// }
						// }
					}
				}
				influenceValues[i] = maxIS - minIS;
			}

		}

		// Finished value iteration
		timerVI = System.currentTimeMillis() - timerVI;
		mainLog.print("Prioritised value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timerVI / 1000.0 + " seconds.");

		timerGlobal = System.currentTimeMillis() - timerGlobal;
		mainLog.println("Overall policy calculation took  " + timerGlobal / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done /* && errorOnNonConverge */) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		res = new ModelCheckerPartialSatResult();

		res.strat = new MDStrategyArray(trimProdMdp, strat);

		// Return results
		res.solnProb = solnProb;
//		res.solnProg = solnProg;
		res.solnCost = solnCost;
		res.numIters = iters;
		res.timeTaken = timerGlobal / 1000.0;
		res.lastSolnCost = influenceValues;
		return res;
	}

	public double mvMultRewSingleDiscount(MDP mdp, int s, int i, double[] vect, MDPRewards mdpRewards,
			double discount) {
		double d, prob;
		int k;

		// Distribution distr
		Iterator<Entry<Integer, Double>> tranIter = mdp.getTransitionsIterator(s, i);// trans.get(s).get(i);
		// Compute sum for this distribution
		// TODO: use transition rewards when added to DTMCss
		// d = mcRewards.getTransitionReward(s);
		d = 0;
		// for (Map.Entry<Integer, Double> e : distr) {
		Entry<Integer, Double> e;
		while (tranIter.hasNext()) {
			e = tranIter.next();
			k = (Integer) e.getKey();
			prob = (Double) e.getValue();
			d += prob * vect[k];
		}
		d *= discount;
		d += mdpRewards.getTransitionReward(s, i);

		return d;
	}

	/**
	 * Compute reachability probabilities using value iteration. Optionally, store
	 * optimal (memoryless) strategy info.
	 * 
	 * @param progStates
	 * @param mdp        The MDP
	 * @param no         Probability 0 states
	 * @param yes        Probability 1 states
	 * @param min        Min or max probabilities (true=min, false=max)
	 * @param init       Optionally, an initial solution vector (will be
	 *                   overwritten)
	 * @param known      Optionally, a set of states for which the exact answer is
	 *                   known
	 * @param strat      Storage for (memoryless) strategy choice indices (ignored
	 *                   if null) Note: if 'known' is specified (i.e. is non-null,
	 *                   'init' must also be given and is used for the exact values.
	 */

	protected ModelCheckerPartialSatResult computeOccupationFrequencyForPath(int maxIters, PrismLog mainLog,
			MDPModelChecker mc, MDP trimProdMdp, BitSet target, double discount) throws PrismException {
		ModelCheckerPartialSatResult res;
		int i, n, iters;

		double solnProb[];

		boolean done;

		long timerVI, timerGlobal;
		int strat[] = null;
		boolean min = false;

		timerGlobal = System.currentTimeMillis();

		// Check for deadlocks in non-target state (because breaks e.g. prob1)
		trimProdMdp.checkForDeadlocks(target);

		// Store num states
		n = trimProdMdp.getNumStates();

		strat = new int[n];
		for (i = 0; i < n; i++) {
			strat[i] = target.get(i) ? -2 : -1;
		}

		// Start value iteration
		timerVI = System.currentTimeMillis();
		mainLog.println("Starting prioritised value iteration (" + (min ? "min" : "max") + ")...");

		// Create solution vector(s)
		solnProb = new double[n];

		for (i = 0; i < n; i++) {

			solnProb[i] = (trimProdMdp.isInitialState(i)) ? 1.0 : 0.0;

		}

		// Start iterations
		iters = 0;
		done = false;

		int j;
		int numChoices;
		double currentProbVal;
		boolean sameProb;

		BitSet selfLoopStates = new BitSet();
		while (!done && iters < maxIters) {
			iters++;
			done = true;
			for (i = 0; i < n; i++) {

				if (!selfLoopStates.get(i)) {

					currentProbVal = 0;
					for (int i_ = 0; i_ < n; i_++) {
						if (!target.get(i_)) {
							if (trimProdMdp.isSuccessor(i_, i)) {
								// get the number of choices from i_
								numChoices = trimProdMdp.getNumChoices(i_);
								for (j = 0; j < numChoices; j++) {
									Iterator<Entry<Integer, Double>> tranIter = trimProdMdp.getTransitionsIterator(i_,
											j);
									while (tranIter.hasNext()) {
										Entry<Integer, Double> sapair = tranIter.next();
										if (sapair.getKey() == i) {
											if (discount == 1.0) {
												if (sapair.getKey() == i_) {
													// a self loop
													// we need a discount factor
													// or just not add the cost here
													currentProbVal += n;
													selfLoopStates.set(i);

												} else
													currentProbVal += sapair.getValue() * solnProb[i_];
											} else
												currentProbVal += sapair.getValue() * solnProb[i_];
										}
									}

								}
							}
						}
					}
					// currentProbVal += //trimProdMdp.mvMultJacSingle(i, j, solnProb);
					currentProbVal *= discount;
					if (trimProdMdp.isInitialState(i))
						currentProbVal += 1;
					sameProb = PrismUtils.doublesAreClose(currentProbVal, solnProb[i], termCritParam,
							termCrit == TermCrit.ABSOLUTE);

					if (!sameProb) {
						done = false;
						solnProb[i] = currentProbVal;

					}
				}
			}
		}

		// Finished value iteration
		timerVI = System.currentTimeMillis() - timerVI;
		mainLog.print("Prioritised value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timerVI / 1000.0 + " seconds.");

		timerGlobal = System.currentTimeMillis() - timerGlobal;
		mainLog.println("Overall policy calculation took  " + timerGlobal / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done /* && errorOnNonConverge */) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		res = new ModelCheckerPartialSatResult();

		res.strat = new MDStrategyArray(trimProdMdp, strat);

		// Return results
		res.solnProb = solnProb;
		res.numIters = iters;
		res.timeTaken = timerGlobal / 1000.0;
		mainLog.println(Arrays.toString(solnProb));
		return res;
	}
}
