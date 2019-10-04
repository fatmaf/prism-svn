package demos;

import java.util.BitSet;
import java.util.HashMap;
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
import parser.ast.ExpressionReward;
import parser.ast.ModulesFile;
import parser.ast.RewardStruct;
import prism.Prism;
import prism.PrismException;
import prism.PrismLog;
import prism.PrismUtils;
import strat.MDStrategyArray;

public class NVIExposed
{

	public HashMap<Integer, HashMap<Integer, Double>> stateActionProbValues = new HashMap<Integer, HashMap<Integer, Double>>();
	public HashMap<Integer, HashMap<Integer, Double>> stateActionProgValues = new HashMap<Integer, HashMap<Integer, Double>>();
	public HashMap<Integer, HashMap<Integer, Double>> stateActionCostValues = new HashMap<Integer, HashMap<Integer, Double>>();

	MDP productmdp; 
	protected TermCrit termCrit = TermCrit.RELATIVE;
	// Parameter for iterative numerical method termination criteria
	protected double termCritParam = 1e-8;

	public ModelCheckerPartialSatResult nviexposed(Prism prism, PrismLog mainLog, Model model, Expression expr, ExpressionReward rewExpr, BitSet statesOfInterest,
			ModulesFile modulesFile, MDPModelChecker mc) throws PrismException

	{
		int maxIters = 10000;
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
		Expression ltl = expr;//exprRew.getExpression();
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

		if (!(da.getAcceptance() instanceof AcceptanceReach) && !(da.getAcceptance() instanceof AcceptanceRabin)) {
			mainLog.println("\nAutomaton is not a DFA. Breaking.");
			// Dummy return vector
			return null;
		}
		// calculate distances to accepting states
		long time = System.currentTimeMillis();
		da.setDistancesToAcc();
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

		// System.out.println("The product MDP has " +
		// product.getProductModel().getNumStates() + " states");

		// Find accepting states + compute reachability probabilities
		BitSet acc;
		//TODO: change this for safety specs 
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
		mainLog.println("Time for lifting progression reward from automaton to product: " + time / 1000.0 + " seconds.");

		time = System.currentTimeMillis();
		// Build trimmed product costs
		MDPRewards prodCosts = ((MDPRewards) costsModel).liftFromModel(product);
		time = System.currentTimeMillis() - time;
		mainLog.println("Time for lifting cost function from original model to product: " + time / 1000.0 + " seconds.");

		BitSet progStates = mc.progressionTrim(product, (MDPRewardsSimple) progRewards, (MDPRewardsSimple) prodCosts);

		mcProduct = mc;

		if (product.getProductModel().getNumStates() > 10000) {
			mainLog.println("\nChanging product to MDPSparse...");
			productMdp = new MDPSparse((MDPSimple) product.getProductModel());
		} else {
			productMdp = product.getProductModel();
		}

		mainLog.println("\nComputing reachability probability, expected progression, and expected cost...");
		ModelCheckerPartialSatResult res = computeNestedValIter(maxIters, mainLog, mc, productMdp, acc, progRewards, prodCosts, progStates);
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
		mainLog.println("\nFor p = " + maxProb + ", the maximum expected cummulative reward to satisfy specification is " + maxRew);

		costsProduct = StateValues.createFromDoubleArray(res.solnCost, productMdp);
		costs = product.projectToOriginalModel(costsProduct);
		double minCost = costs.getDoubleArray()[model.getFirstInitialState()];
		mainLog.println("\nFor p = " + maxProb + ", r = " + +maxRew + " the minimum expected  cummulative cost to satisfy specification is " + minCost);
		// System.out.println("Probability to find objects: " + maxProb);
		// System.out.println("Expected progression reward: " + maxRew);
		// System.out.println("Expected time to execute task: " + minCost);
		// System.out.println("--------------------------------------------------------------");
		return res;

		//		return new AbstractMap.SimpleEntry<MDP,MDStrategy>(productMdp,(MDStrategy)res.strat); 

	}

	/**
	 * Compute reachability probabilities using value iteration. Optionally, store
	 * optimal (memoryless) strategy info.
	 * 
	 * @param progStates
	 * @param mdp
	 *            The MDP
	 * @param no
	 *            Probability 0 states
	 * @param yes
	 *            Probability 1 states
	 * @param min
	 *            Min or max probabilities (true=min, false=max)
	 * @param init
	 *            Optionally, an initial solution vector (will be overwritten)
	 * @param known
	 *            Optionally, a set of states for which the exact answer is known
	 * @param strat
	 *            Storage for (memoryless) strategy choice indices (ignored if null)
	 *            Note: if 'known' is specified (i.e. is non-null, 'init' must also
	 *            be given and is used for the exact values.
	 */
	protected ModelCheckerPartialSatResult computeNestedValIter(int maxIters, PrismLog mainLog, MDPModelChecker mc, MDP trimProdMdp, BitSet target,
			MDPRewards progRewards, MDPRewards prodCosts, BitSet progStates) throws PrismException
	{
		ModelCheckerPartialSatResult res;
		int i, n, iters, numYes, numNo;
		double initValProb, initValRew, initValCost;
		stateActionProbValues = new HashMap<Integer, HashMap<Integer, Double>>();
		stateActionProgValues = new HashMap<Integer, HashMap<Integer, Double>>();
		stateActionCostValues = new HashMap<Integer, HashMap<Integer, Double>>();

		double solnProb[], soln2Prob[];
		double solnProg[], soln2Prog[];
		double solnCost[], soln2Cost[];
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

		// If required, export info about target states
		//			if (getExportTarget()) {
		//				BitSet bsInit = new BitSet(n);
		//				for (i = 0; i < n; i++) {
		//					bsInit.set(i, trimProdMdp.isInitialState(i));
		//				}
		//				List<BitSet> labels = Arrays.asList(bsInit, target);
		//				List<String> labelNames = Arrays.asList("init", "target");
		//				mainLog.println("\nExporting target states info to file \"" + getExportTargetFilename() + "\"...");
		//				PrismLog out = new PrismFileLog(getExportTargetFilename());
		//				exportLabels(trimProdMdp, labels, labelNames, Prism.EXPORT_PLAIN, out);
		//				out.close();
		//			}

		// If required, create/initialise strategy storage
		// Set choices to -1, denoting unknown
		// (except for target states, which are -2, denoting arbitrary)
		//			if (genStrat || exportAdv) {
		strat = new int[n];
		for (i = 0; i < n; i++) {
			strat[i] = target.get(i) ? -2 : -1;
		}
		//			}

		// Precomputation
		timerProb0 = System.currentTimeMillis();
		//			if (precomp && prob0) {
		no = mc.prob0(trimProdMdp, null, target, min, strat);
		//			} else {
		//				no = new BitSet();
		//			}
		timerProb0 = System.currentTimeMillis() - timerProb0;
		timerProb1 = System.currentTimeMillis();
		//			if (precomp && prob1) {
		yes = mc.prob1(trimProdMdp, null, target, min, strat);
		//			} else {
		//				yes = (BitSet) target.clone();
		//			}
		timerProb1 = System.currentTimeMillis() - timerProb1;

		// Print results of precomputation
		numYes = yes.cardinality();
		numNo = no.cardinality();
		mainLog.println("target=" + target.cardinality() + ", yes=" + numYes + ", no=" + numNo + ", maybe=" + (n - (numYes + numNo)));

		// If still required, store strategy for no/yes (0/1) states.
		// This is just for the cases max=0 and min=1, where arbitrary choices
		// suffice (denoted by -2)
		//			if (genStrat || exportAdv) {
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
		//			}

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
			solnCost[i] = initValCost;
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
				if (progStates.get(i)) {
					if (!stateActionProbValues.containsKey(i))
						stateActionProbValues.put(i, new HashMap<Integer, Double>());
					if (!stateActionProgValues.containsKey(i))
						stateActionProgValues.put(i, new HashMap<Integer, Double>());
					if (!stateActionCostValues.containsKey(i))
						stateActionCostValues.put(i, new HashMap<Integer, Double>());
					numChoices = trimProdMdp.getNumChoices(i);
					for (j = 0; j < numChoices; j++) {

						currentProbVal = trimProdMdp.mvMultJacSingle(i, j, solnProb);
						currentProgVal = trimProdMdp.mvMultRewSingle(i, j, solnProg, progRewards);
						currentCostVal = trimProdMdp.mvMultRewSingle(i, j, solnCost, prodCosts);
						sameProb = PrismUtils.doublesAreClose(currentProbVal, solnProb[i], termCritParam, termCrit == TermCrit.ABSOLUTE);
						sameProg = PrismUtils.doublesAreClose(currentProgVal, solnProg[i], termCritParam, termCrit == TermCrit.ABSOLUTE);
						sameCost = PrismUtils.doublesAreClose(currentCostVal, solnCost[i], termCritParam, termCrit == TermCrit.ABSOLUTE);
						stateActionProbValues.get(i).put(j, currentProbVal);
						stateActionProgValues.get(i).put(j, currentProgVal);
						stateActionCostValues.get(i).put(j, currentCostVal);
						if (!sameProb && currentProbVal > solnProb[i]) {
							done = false;
							solnProb[i] = currentProbVal;
							solnProg[i] = currentProgVal;
							solnCost[i] = currentCostVal;
							//								if (genStrat || exportAdv) {
							strat[i] = j;
							//								}
						} else {
							if (sameProb) {
								if (!sameProg && currentProgVal > solnProg[i]) {
									done = false;
									// solnProb[i] = currentProbVal;
									solnProg[i] = currentProgVal;
									solnCost[i] = currentCostVal;
									//										if (genStrat || exportAdv) {
									strat[i] = j;
									//										}
								} else {
									if (sameProg) {
										if (!sameCost && currentCostVal < solnCost[i]) {
											done = false;
											// solnProb[i] = currentProbVal;
											// solnProg[i] = currentProgVal;
											solnCost[i] = currentCostVal;
											//												if (genStrat || exportAdv) {
											strat[i] = j;
											//												}
										}
									}
								}
							}
						}
					}
				}
			}
			// Check termination
			// done = PrismUtils.doublesAreClose(solnProb, soln2Prob,
			// termCritParam, termCrit == TermCrit.ABSOLUTE);
			// done = done && PrismUtils.doublesAreClose(solnProg, soln2Prog,
			// termCritParam, termCrit == TermCrit.ABSOLUTE);
			// done = done && PrismUtils.doublesAreClose(solnCost, soln2Cost,
			// termCritParam, termCrit == TermCrit.ABSOLUTE);

			// Save previous iter
			// soln2Prob = solnProb.clone();
			// soln2Prog = solnProg.clone();
			// soln2Cost = solnCost.clone();
		}

		// Finished value iteration
		timerVI = System.currentTimeMillis() - timerVI;
		mainLog.print("Prioritised value iteration (" + (min ? "min" : "max") + ")");
		mainLog.println(" took " + iters + " iterations and " + timerVI / 1000.0 + " seconds.");

		timerGlobal = System.currentTimeMillis() - timerGlobal;
		mainLog.println("Overall policy calculation took  " + timerGlobal / 1000.0 + " seconds.");

		// Non-convergence is an error (usually)
		if (!done /*&& errorOnNonConverge*/) {
			String msg = "Iterative method did not converge within " + iters + " iterations.";
			msg += "\nConsider using a different numerical method or increasing the maximum number of iterations";
			throw new PrismException(msg);
		}

		res = new ModelCheckerPartialSatResult();
		// Store strategy
		//			if (genStrat) {
		res.strat = new MDStrategyArray(trimProdMdp, strat);
		//			}
		// Export adversary
		//			if (exportAdv) {
		//				// Prune strategy
		//				// restrictStrategyToReachableStates(trimProdMdp, strat);
		//				// Export
		//				PrismLog out = new PrismFileLog(exportAdvFilename);
		//				new DTMCFromMDPMemorylessAdversary(trimProdMdp, strat).exportToPrismExplicitTra(out);
		//				out.close();
		//			}

		// Return results
		res.solnProb = solnProb;
		res.solnProg = solnProg;
		res.solnCost = solnCost;
		res.numIters = iters;
		res.timeTaken = timerGlobal / 1000.0;
		return res;
	}

}
