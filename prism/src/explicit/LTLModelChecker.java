//==============================================================================
//	
//	Copyright (c) 2002-
//	Authors:
//	* Dave Parker <david.parker@comlab.ox.ac.uk> (University of Oxford)
//	* Joachim Klein <klein@tcs.inf.tu-dresden.de> (TU Dresden)
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

package explicit;

import java.awt.Point;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Stack;
import java.util.Vector;

import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.Expression;
import parser.ast.ExpressionBinaryOp;
import parser.ast.ExpressionLabel;
import parser.ast.ExpressionTemporal;
import parser.ast.ExpressionUnaryOp;
import parser.type.TypeBool;
import parser.type.TypePathBool;
import prism.ModelType;
import prism.PrismComponent;
import prism.PrismException;
import prism.PrismLangException;
import prism.PrismNotSupportedException;
import prism.PrismUtils;
import acceptance.AcceptanceBuchi;
import acceptance.AcceptanceGenRabin;
import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceStreett;
import acceptance.AcceptanceType;
import automata.DA;
import automata.LTL2DA;

import common.IterableStateSet;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MDPRewardsSimple;

/**
 * LTL model checking functionality
 */
public class LTLModelChecker extends PrismComponent {
	/** Make LTL product accessible as a Product */
	public class LTLProduct<M extends Model> extends Product<M> {
		private int daSize;
		private int invMap[];
		private AcceptanceOmega acceptance;

		public LTLProduct(M productModel, M originalModel, AcceptanceOmega acceptance, int daSize, int[] invMap) {
			super(productModel, originalModel);
			this.daSize = daSize;
			this.invMap = invMap;
			this.acceptance = acceptance;
		}

		@Override
		public int getModelState(int productState) {
			return invMap[productState] / daSize;
		}

		@Override
		public int getAutomatonState(int productState) {
			return invMap[productState] % daSize;
		}

		public AcceptanceOmega getAcceptance() {
			return acceptance;
		}

		public void setAcceptance(AcceptanceOmega acceptance) {
			this.acceptance = acceptance;
		}
	}

	/**
	 * Create a new LTLModelChecker, inherit basic state from parent (unless
	 * null).
	 */
	public LTLModelChecker(PrismComponent parent) {
		super(parent);
	}

	/**
	 * Returns {@code true} if expression {@code expr} is a formula that can be
	 * handled by LTLModelChecker for the given ModelType.
	 */
	public static boolean isSupportedLTLFormula(ModelType modelType, Expression expr) throws PrismLangException {
		if (!expr.isPathFormula(true)) {
			return false;
		}
		if (Expression.containsTemporalTimeBounds(expr)) {
			if (modelType.continuousTime()) {
				// Only support temporal bounds for discrete time models
				return false;
			}

			if (!expr.isSimplePathFormula()) {
				// Only support temporal bounds for simple path formulas
				return false;
			}
		}
		return true;
	}

	/**
	 * Extract maximal state formula from an LTL path formula, model check them
	 * (with passed in model checker) and replace them with ExpressionLabel
	 * objects L0, L1, etc. Expression passed in is modified directly, but the
	 * result is also returned. As an optimisation, expressions that results in
	 * true/false for all states are converted to an actual true/false, and
	 * duplicate results (or their negations) reuse the same label. BitSets
	 * giving the states which satisfy each label are put into the vector
	 * labelBS, which should be empty when this function is called.
	 */
	public Expression checkMaximalStateFormulas(StateModelChecker mc, Model model, Expression expr,
			Vector<BitSet> labelBS) throws PrismException {
		// A state formula
		if (expr.getType() instanceof TypeBool) {
			// Model check state formula for all states
			StateValues sv = mc.checkExpression(model, expr, null);
			BitSet bs = sv.getBitSet();
			// Detect special cases (true, false) for optimisation
			if (bs.isEmpty()) {
				return Expression.False();
			}
			if (bs.cardinality() == model.getNumStates()) {
				return Expression.True();
			}
			// See if we already have an identical result
			// (in which case, reuse it)
			int i = labelBS.indexOf(bs);
			if (i != -1) {
				sv.clear();
				return new ExpressionLabel("L" + i);
			}
			// Also, see if we already have the negation of this result
			// (in which case, reuse it)
			BitSet bsNeg = new BitSet(model.getNumStates());
			bsNeg.set(0, model.getNumStates());
			bsNeg.andNot(bs);
			i = labelBS.indexOf(bsNeg);
			if (i != -1) {
				sv.clear();
				return Expression.Not(new ExpressionLabel("L" + i));
			}
			// Otherwise, add result to list, return new label
			labelBS.add(bs);
			return new ExpressionLabel("L" + (labelBS.size() - 1));
		}
		// A path formula (recurse, modify, return)
		else if (expr.getType() instanceof TypePathBool) {
			if (expr instanceof ExpressionBinaryOp) {
				ExpressionBinaryOp exprBinOp = (ExpressionBinaryOp) expr;
				exprBinOp.setOperand1(checkMaximalStateFormulas(mc, model, exprBinOp.getOperand1(), labelBS));
				exprBinOp.setOperand2(checkMaximalStateFormulas(mc, model, exprBinOp.getOperand2(), labelBS));
			} else if (expr instanceof ExpressionUnaryOp) {
				ExpressionUnaryOp exprUnOp = (ExpressionUnaryOp) expr;
				exprUnOp.setOperand(checkMaximalStateFormulas(mc, model, exprUnOp.getOperand(), labelBS));
			} else if (expr instanceof ExpressionTemporal) {
				ExpressionTemporal exprTemp = (ExpressionTemporal) expr;
				if (exprTemp.getOperand1() != null) {
					exprTemp.setOperand1(checkMaximalStateFormulas(mc, model, exprTemp.getOperand1(), labelBS));
				}
				if (exprTemp.getOperand2() != null) {
					exprTemp.setOperand2(checkMaximalStateFormulas(mc, model, exprTemp.getOperand2(), labelBS));
				}
			}
		}
		return expr;
	}

	/**
	 * Construct a deterministic automaton (DA) for an LTL formula, having first
	 * extracted maximal state formulas and model checked them with the passed
	 * in model checker. The maximal state formulas are assigned labels (L0, L1,
	 * etc.) which become the atomic propositions in the resulting DA. BitSets
	 * giving the states which satisfy each label are put into the vector
	 * {@code labelBS}, which should be empty when this function is called.
	 *
	 * @param mc
	 *            a ProbModelChecker, used for checking maximal state formulas
	 * @param model
	 *            the model
	 * @param expr
	 *            a path expression, i.e. the LTL formula
	 * @param labelBS
	 *            empty vector to be filled with BitSets for subformulas
	 * @param allowedAcceptance
	 *            the allowed acceptance types
	 * @return the DA
	 */
	public DA<BitSet, ? extends AcceptanceOmega> constructDAForLTLFormula(ProbModelChecker mc, Model model,
			Expression expr, Vector<BitSet> labelBS, AcceptanceType... allowedAcceptance) throws PrismException {
		Expression ltl;
		DA<BitSet, ? extends AcceptanceOmega> da;
		long time;

		if (Expression.containsTemporalTimeBounds(expr)) {
			if (model.getModelType().continuousTime()) {
				throw new PrismException("Automaton construction for time-bounded operators not supported for "
						+ model.getModelType() + ".");
			}

			if (!expr.isSimplePathFormula()) {
				throw new PrismNotSupportedException("Time-bounded operators not supported in LTL: " + expr);
			}
		}

		// Model check maximal state formulas
		ltl = checkMaximalStateFormulas(mc, model, expr.deepCopy(), labelBS);

		// Convert LTL formula to deterministic automaton
		mainLog.println("\nBuilding deterministic automaton (for " + ltl + ")...");
		time = System.currentTimeMillis();
		LTL2DA ltl2da = new LTL2DA(this);
		da = ltl2da.convertLTLFormulaToDA(ltl, mc.getConstantValues(), allowedAcceptance);
		mainLog.println(da.getAutomataType() + " has " + da.size() + " states, "
				+ da.getAcceptance().getSizeStatistics() + ".");
		da.checkForCanonicalAPs(labelBS.size());
		time = System.currentTimeMillis() - time;
		mainLog.println("Time for " + da.getAutomataType() + " translation: " + time / 1000.0 + " seconds.");
		// If required, export DA
		if (settings.getExportPropAut()) {
			mainLog.println("Exporting " + da.getAutomataType() + " to file \"" + settings.getExportPropAutFilename()
					+ "\"...");
			PrintStream out = PrismUtils.newPrintStream(settings.getExportPropAutFilename());
			da.print(out, settings.getExportPropAutType());
			out.close();
		}

		return da;
	}

	/**
	 * Generate a deterministic automaton for the given LTL formula and
	 * construct the product of this automaton with a Markov chain.
	 *
	 * @param mc
	 *            a ProbModelChecker, used for checking maximal state formulas
	 * @param model
	 *            the model
	 * @param expr
	 *            a path expression
	 * @param statesOfInterest
	 *            the set of states for which values should be calculated (null
	 *            = all states)
	 * @param allowedAcceptance
	 *            the allowed acceptance types
	 * @return the product with the DA
	 */
	public LTLProduct<DTMC> constructProductMC(ProbModelChecker mc, DTMC model, Expression expr,
			BitSet statesOfInterest, AcceptanceType... allowedAcceptance) throws PrismException {
		// Convert LTL formula to automaton
		Vector<BitSet> labelBS = new Vector<BitSet>();
		DA<BitSet, ? extends AcceptanceOmega> da;
		da = constructDAForLTLFormula(mc, model, expr, labelBS, allowedAcceptance);

		// Build product of model and automaton
		mainLog.println("\nConstructing MC-" + da.getAutomataType() + " product...");
		LTLProduct<DTMC> product = constructProductModel(da, model, labelBS, statesOfInterest);
		mainLog.print("\n" + product.getProductModel().infoStringTable());

		return product;
	}

	/**
	 * Generate a deterministic automaton for the given LTL formula and
	 * construct the product of this automaton with an MDP.
	 *
	 * @param mc
	 *            a ProbModelChecker, used for checking maximal state formulas
	 * @param model
	 *            the model
	 * @param expr
	 *            a path expression
	 * @param statesOfInterest
	 *            the set of states for which values should be calculated (null
	 *            = all states)
	 * @param allowedAcceptance
	 *            the allowed acceptance conditions
	 * @return the product with the DA
	 * @throws PrismException
	 */
	public LTLProduct<MDP> constructProductMDP(ProbModelChecker mc, MDP model, Expression expr, BitSet statesOfInterest,
			AcceptanceType... allowedAcceptance) throws PrismException {
		// Convert LTL formula to automaton
		Vector<BitSet> labelBS = new Vector<BitSet>();
		DA<BitSet, ? extends AcceptanceOmega> da;
		da = constructDAForLTLFormula(mc, model, expr, labelBS, allowedAcceptance);

		// Build product of model and automaton
		mainLog.println("\nConstructing MDP-" + da.getAutomataType() + " product...");
		LTLProduct<MDP> product = constructProductModel(da, model, labelBS, statesOfInterest);
		mainLog.print("\n" + product.getProductModel().infoStringTable());

		return product;
	}

	/**
	 * Generate a deterministic automaton for the given LTL formula and
	 * construct the product of this automaton with an STPG.
	 *
	 * @param mc
	 *            a ProbModelChecker, used for checking maximal state formulas
	 * @param model
	 *            the model
	 * @param expr
	 *            a path expression
	 * @param statesOfInterest
	 *            the set of states for which values should be calculated (null
	 *            = all states)
	 * @param allowedAcceptance
	 *            the allowed acceptance conditions
	 * @return the product with the DA
	 * @throws PrismException
	 */
	public LTLProduct<STPG> constructProductSTPG(ProbModelChecker mc, STPG model, Expression expr,
			BitSet statesOfInterest, AcceptanceType... allowedAcceptance) throws PrismException {
		// Convert LTL formula to automaton
		Vector<BitSet> labelBS = new Vector<BitSet>();
		DA<BitSet, ? extends AcceptanceOmega> da;
		da = constructDAForLTLFormula(mc, model, expr, labelBS, allowedAcceptance);

		// Build product of model and automaton
		mainLog.println("\nConstructing STPG-" + da.getAutomataType() + " product...");
		LTLProduct<STPG> product = constructProductModel(da, model, labelBS, statesOfInterest);
		mainLog.print("\n" + product.getProductModel().infoStringTable());

		return product;
	}

	/**
	 * Generate a deterministic automaton for the given LTL formula and
	 * construct the product of this automaton with a model.
	 *
	 * @param mc
	 *            a ProbModelChecker, used for checking maximal state formulas
	 * @param model
	 *            the model
	 * @param expr
	 *            a path expression
	 * @param statesOfInterest
	 *            the set of states for which values should be calculated (null
	 *            = all states)
	 * @param allowedAcceptance
	 *            the allowed acceptance conditions
	 * @return the product with the DA
	 * @throws PrismException
	 */
	public <M extends Model> LTLProduct<M> constructProductModel(ProbModelChecker mc, M model, Expression expr,
			BitSet statesOfInterest, AcceptanceType... allowedAcceptance) throws PrismException {
		// Convert LTL formula to automaton
		Vector<BitSet> labelBS = new Vector<BitSet>();
		DA<BitSet, ? extends AcceptanceOmega> da;
		da = constructDAForLTLFormula(mc, model, expr, labelBS, allowedAcceptance);

		// Build product of model and automaton
		mainLog.println("\nConstructing " + model.getModelType() + "-" + da.getAutomataType() + " product...");
		LTLProduct<M> product = constructProductModel(da, model, labelBS, statesOfInterest);
		mainLog.print("\n" + product.getProductModel().infoStringTable());

		return product;
	}

	/**
	 * Construct the product of a DA and a model.
	 * 
	 * @param da
	 *            The DA
	 * @param model
	 *            The model
	 * @param labelBS
	 *            BitSets giving the set of states for each AP in the DA
	 * @param statesOfInterest
	 *            the set of states for which values should be calculated (null
	 *            = all states)
	 * @return The product model
	 */
	public <M extends Model> LTLProduct<M> constructProductModel(DA<BitSet, ? extends AcceptanceOmega> da, M model,
			Vector<BitSet> labelBS, BitSet statesOfInterest) throws PrismException {
		ModelType modelType = model.getModelType();
		int daSize = da.size();
		int numAPs = da.getAPList().size();
		int modelNumStates = model.getNumStates();
		int prodNumStates = modelNumStates * daSize;
		int s_1, s_2, q_1, q_2;
		BitSet s_labels = new BitSet(numAPs);
		List<State> prodStatesList = null, daStatesList = null;

		VarList newVarList = null;

		if (model.getVarList() != null) {
			VarList varList = model.getVarList();
			// Create a (new, unique) name for the variable that will represent
			// DA states
			String daVar = "_da";
			while (varList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}

			newVarList = (VarList) varList.clone();
			// NB: if DA only has one state, we add an extra dummy state
			Declaration decl = new Declaration(daVar,
					new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(da.size() - 1, 1))));
			newVarList.addVar(0, decl, 1, model.getConstantValues());
		}

		// Create a (simple, mutable) model of the appropriate type
		ModelSimple prodModel = null;
		switch (modelType) {
		case DTMC: {
			DTMCSimple dtmcProd = new DTMCSimple();
			dtmcProd.setVarList(newVarList);
			prodModel = dtmcProd;
			break;
		}
		case MDP: {
			MDPSimple mdpProd = new MDPSimple();
			mdpProd.setVarList(newVarList);
			prodModel = mdpProd;
			break;
		}
		case STPG: {
			STPGExplicit stpgProd = new STPGExplicit();
			stpgProd.setVarList(newVarList);
			prodModel = stpgProd;
			break;
		}
		default:
			throw new PrismNotSupportedException("Model construction not supported for " + modelType + "s");
		}

		// Encoding:
		// each state s' = <s, q> = s * daSize + q
		// s(s') = s' / daSize
		// q(s') = s' % daSize

		LinkedList<Point> queue = new LinkedList<Point>();
		int map[] = new int[prodNumStates];
		Arrays.fill(map, -1);

		if (model.getStatesList() != null) {
			prodStatesList = new ArrayList<State>();
			daStatesList = new ArrayList<State>(da.size());
			for (int i = 0; i < da.size(); i++) {
				daStatesList.add(new State(1).setValue(0, i));
			}
		}

		// We need results for all states of the original model in
		// statesOfInterest
		// We thus explore states of the product starting from these states.
		// These are designated as initial states of the product model
		// (a) to ensure reachability is done for these states; and
		// (b) to later identify the corresponding product state for the
		// original states
		// of interest
		for (int s_0 : new IterableStateSet(statesOfInterest, model.getNumStates())) {
			// Get BitSet representing APs (labels) satisfied by state s_0
			for (int k = 0; k < numAPs; k++) {
				s_labels.set(k, labelBS.get(Integer.parseInt(da.getAPList().get(k).substring(1))).get(s_0));
			}
			// Find corresponding initial state in DA
			int q_0 = da.getEdgeDestByLabel(da.getStartState(), s_labels);
			if (q_0 < 0) {
				throw new PrismException(
						"The deterministic automaton is not complete (state " + da.getStartState() + ")");
			}
			// Add (initial) state to product
			queue.add(new Point(s_0, q_0));
			switch (modelType) {
			case STPG:
				((STPGExplicit) prodModel).addState(((STPG) model).getPlayer(s_0));
				break;
			default:
				prodModel.addState();
				break;
			}
			prodModel.addInitialState(prodModel.getNumStates() - 1);
			map[s_0 * daSize + q_0] = prodModel.getNumStates() - 1;
			if (prodStatesList != null) {
				// Store state information for the product
				prodStatesList.add(new State(daStatesList.get(q_0), model.getStatesList().get(s_0)));
			}
		}

		// Product states
		BitSet visited = new BitSet(prodNumStates);
		while (!queue.isEmpty()) {
			Point p = queue.pop();
			s_1 = p.x;
			q_1 = p.y;
			visited.set(s_1 * daSize + q_1);

			// Go through transitions from state s_1 in original model
			int numChoices = (model instanceof NondetModel) ? ((NondetModel) model).getNumChoices(s_1) : 1;
			for (int j = 0; j < numChoices; j++) {
				Iterator<Map.Entry<Integer, Double>> iter;
				switch (modelType) {
				case DTMC:
					iter = ((DTMC) model).getTransitionsIterator(s_1);
					break;
				case MDP:
					iter = ((MDP) model).getTransitionsIterator(s_1, j);
					break;
				case STPG:
					iter = ((STPG) model).getTransitionsIterator(s_1, j);
					break;
				default:
					throw new PrismNotSupportedException("Product construction not implemented for " + modelType + "s");
				}
				Distribution prodDistr = null;
				if (modelType.nondeterministic()) {
					prodDistr = new Distribution();
				}
				while (iter.hasNext()) {
					Map.Entry<Integer, Double> e = iter.next();
					s_2 = e.getKey();
					double prob = e.getValue();
					// Get BitSet representing APs (labels) satisfied by
					// successor state s_2
					for (int k = 0; k < numAPs; k++) {
						s_labels.set(k, labelBS.get(Integer.parseInt(da.getAPList().get(k).substring(1))).get(s_2));
					}
					// Find corresponding successor in DA
					q_2 = da.getEdgeDestByLabel(q_1, s_labels);
					if (q_2 < 0) {
						throw new PrismException("The deterministic automaton is not complete (state " + q_1 + ")");
					}
					// Add state/transition to model
					if (!visited.get(s_2 * daSize + q_2) && map[s_2 * daSize + q_2] == -1) {
						queue.add(new Point(s_2, q_2));
						switch (modelType) {
						case STPG:
							((STPGExplicit) prodModel).addState(((STPG) model).getPlayer(s_2));
							break;
						default:
							prodModel.addState();
							break;
						}
						map[s_2 * daSize + q_2] = prodModel.getNumStates() - 1;
						if (prodStatesList != null) {
							// Store state information for the product
							prodStatesList.add(new State(daStatesList.get(q_2), model.getStatesList().get(s_2)));
						}
					}
					switch (modelType) {
					case DTMC:
						((DTMCSimple) prodModel).setProbability(map[s_1 * daSize + q_1], map[s_2 * daSize + q_2], prob);
						break;
					case MDP:
					case STPG:
						prodDistr.set(map[s_2 * daSize + q_2], prob);
						break;
					default:
						throw new PrismNotSupportedException(
								"Product construction not implemented for " + modelType + "s");
					}
				}
				switch (modelType) {
				case MDP:
					((MDPSimple) prodModel).addActionLabelledChoice(map[s_1 * daSize + q_1], prodDistr,
							((MDP) model).getAction(s_1, j));
					break;
				case STPG:
					((STPGExplicit) prodModel).addActionLabelledChoice(map[s_1 * daSize + q_1], prodDistr,
							((STPG) model).getAction(s_1, j));
					break;
				default:
					break;
				}
			}
		}

		// Build a mapping from state indices to states (s,q), encoded as (s *
		// daSize + q)
		int invMap[] = new int[prodModel.getNumStates()];
		for (int i = 0; i < map.length; i++) {
			if (map[i] != -1) {
				invMap[map[i]] = i;
			}
		}

		prodModel.findDeadlocks(false);

		if (prodStatesList != null) {
			prodModel.setStatesList(prodStatesList);
		}

		@SuppressWarnings("unchecked")
		LTLProduct<M> product = new LTLProduct<M>((M) prodModel, model, null, daSize, invMap);

		// generate acceptance for the product model by lifting
		product.setAcceptance(liftAcceptance(product, da.getAcceptance()));

		// lift the labels
		for (String label : model.getLabels()) {
			BitSet liftedLabel = product.liftFromModel(model.getLabelStates(label));
			prodModel.addLabel(label, liftedLabel);
		}

		return product;
	}

	// test constructmodel function with a weighted dfa
	public <M> Object[] constructWProductModel(DA<BitSet, ? extends AcceptanceOmega> da, M model,
			Vector<BitSet> labelBS, BitSet statesOfInterest) throws PrismException {
		
		int daSize = da.size();
		int numAPs = da.getAPList().size();
		int modelNumStates = ((MDP)model).getNumStates();
		int prodNumStates = modelNumStates * daSize;
		int s_1, s_2, q_1, q_2;
		BitSet s_labels = new BitSet(numAPs);
		List<State> prodStatesList = null, daStatesList = null;
		String saveplace = "/home/fatma/hubic/phD/work/code/mdpltl/prism-svn/prism/tests/fatma_tests/dotfiles/";

		VarList newVarList = null;

		if (((MDP)model).getVarList() != null) {
			VarList varList = ((MDP)model).getVarList();
			// Create a (new, unique) name for the variable that will represent
			// DA states
			String daVar = "_da";
			while (varList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
			}

			newVarList = (VarList) varList.clone();
			// NB: if DA only has one state, we add an extra dummy state
			Declaration decl = new Declaration(daVar,
					new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(da.size() - 1, 1))));
			newVarList.addVar(0, decl, 1, ((MDP)model).getConstantValues());
		}
		// create a list of da states and mdp states
		List<Point> all_s = new Vector<Point>();

		MDPSimple mdpProd = new MDPSimple();
		for (int q_i = 0; q_i < modelNumStates; q_i++) {
			for (int z_i = 0; z_i < da.size(); z_i++) {
				all_s.add(new Point(q_i, z_i));
				mainLog.println("adding (q,z) (" + q_i + "," + z_i + ")");
			}
		}

		// add states to product
		mdpProd.addStates(all_s.size());
		mainLog.println("number of states added = " + all_s.size() + " prodmdp states = " + mdpProd.numStates);
		BitSet qp_labels = new BitSet(numAPs);
		BitSet q_labels = new BitSet(numAPs);
		MDPSimple mdpProd_actionCopies = new MDPSimple(mdpProd);
		// for each AP there is a cost for skipping it
		// the powerset is 2^numAPs
		int ps = (int) Math.pow(2.0, (double) numAPs);
		double apCosts[] = new double[ps + 1]; // the first one will be 0 for
												// nothing
		// lets just set them all ourselves
		apCosts[0] = 0;
		for (int ap = 1; ap < ps + 1; ap++) {
			apCosts[ap] = ap + 10; // this is just so i can test stuff
		}
		// if only I cared about much else
		// mitti pao :P
		// create a thing for MDPRewards, add the states or whatever and for
		// each transition add a cost.
		// also we need to be making copies ish ... or do we ... cuz lots of
		// things will have their own stuff
		// look into this
		MDPRewardsSimple skipcosts = new MDPRewardsSimple(mdpProd.numStates);

		for (int i = 0; i < mdpProd.numStates; i++) {
			for (int j = i; j < mdpProd.numStates; j++) {
				int q = all_s.get(i).x;
				int z = all_s.get(i).y;
				int qp = all_s.get(j).x;
				int zp = all_s.get(j).y;

				boolean mdphasedge = ((MDP)model).isSuccessor(q, qp);

				List<BitSet> edges_da = da.getEdgeLabels(z, zp);
				int edges_da_num = edges_da.size();

				for (int k = 0; k < numAPs; k++) {
					qp_labels.set(k, labelBS.get(Integer.parseInt(da.getAPList().get(k).substring(1))).get(qp));
					q_labels.set(k, labelBS.get(Integer.parseInt(da.getAPList().get(k).substring(1))).get(q));
				}

				// finding the actions that get us from q to qp
				int numchoices = ((MDP) model).getNumChoices(q);
				int numchoicesqp = ((MDP) model).getNumChoices(qp);
				// for each choice get the state and action label
				// does this choice have this as a successor state
				List<Point> qtoqp = new ArrayList<Point>();
				List<Point> qptoq = new ArrayList<Point>();

				for (int c = 0; c < numchoices; c++) {
					Iterator<Map.Entry<Integer, Double>> iter = ((MDP) model).getTransitionsIterator(q, c);
					while (iter.hasNext()) {
						Map.Entry<Integer, Double> e = iter.next();
						if (qp == e.getKey()) {
							Point p = new Point();
							p.setLocation((double) c, e.getValue());

							qtoqp.add(p);
						}

					}
				}

				for (int c = 0; c < numchoicesqp; c++) {
					Iterator<Map.Entry<Integer, Double>> iter = ((MDP) model).getTransitionsIterator(qp, c);
					while (iter.hasNext()) {
						Map.Entry<Integer, Double> e = iter.next();
						if (q == e.getKey()) {
							Point p = new Point();
							p.setLocation((double) c, e.getValue());

							qptoq.add(p);
						}
					}
				}

				String info = "(d,d') e ->D (" + q + "," + qp + ") " + mdphasedge + " L(d')= " + qp_labels.toString()
						+ " (z,z') " + "(" + z + ", " + zp + ") z'=z " + (zp == z) + " t -> z' = d(z,t) = ";
				StringBuilder sb2 = new StringBuilder();
				for (int k = 0; k < edges_da.size(); k++) {

					sb2.append(edges_da.get(k).toString());
					sb2.append("\t");
				}
				info = info + sb2.toString();
				// mainLog.println(info);
				if (mdphasedge & edges_da_num > 0) {
					StringBuilder edsb = new StringBuilder();

					for (int ed = 0; ed < edges_da_num; ed++) {
						// for each action I have I can add an edge
						for (int c = 0; c < qtoqp.size(); c++) {
							Distribution probd = new Distribution();
							probd.add(j, qtoqp.get(c).getY());
							mdpProd.addActionLabelledChoice(i, probd,
									((MDP) model).getAction(q, (int) qtoqp.get(c).getX()));
							mdpProd_actionCopies.addActionLabelledChoice(i, probd,
									((MDP) model).getAction(q, (int) qtoqp.get(c).getX()).toString() + c + ed);
							// get the cost
							double cost = 0;
							boolean max_semantics = true;
							if (!qp_labels.equals(edges_da.get(ed))) {
								int ap1 = 0;
								long arr[] = qp_labels.toLongArray();
								if (arr.length > 0) {
									ap1 = (int) arr[0] + 1;
								}

								int ap2 = 0;
								long arr2[] = edges_da.get(ed).toLongArray();
								if (arr2.length > 0)
									ap2 = (int) arr2[0] + 1;
								if (max_semantics) {

									if (apCosts[ap1] < apCosts[ap2]) {
										cost = apCosts[ap2];
									} else
										cost = apCosts[ap1];
								}

								else {
									// sum
									cost = apCosts[ap1] + apCosts[ap2];
								}
							}
							skipcosts.addToTransitionReward(i, mdpProd_actionCopies.getNumChoices(i), cost);
							mainLog.println(
									"cost: " + qp_labels.toString() + "," + edges_da.get(ed).toString() + ":" + cost);

						}
						edsb.append(edges_da.get(ed).toString());
					}

					// add costs for this to mdp todo

					mainLog.println("(" + q + "," + z + ")--" + qp_labels.toString() + "," + edsb.toString() + " --> ("
							+ qp + "," + zp + ")");

				}
				if (i != j) {
					boolean mdphasedge_rev = ((MDP)model).isSuccessor(qp, q);
					List<BitSet> edges_da_rev = da.getEdgeLabels(zp, z);
					int edges_da_rev_num = edges_da_rev.size();

					if (mdphasedge_rev & edges_da_rev_num > 0) {
						StringBuilder edrsb = new StringBuilder();

						for (int ed = 0; ed < edges_da_rev_num; ed++) {
							edrsb.append(edges_da_rev.get(ed).toString());
							for (int c = 0; c < qptoq.size(); c++) {
								Distribution probd = new Distribution();
								probd.add(i, qptoq.get(c).getY());
								mdpProd.addActionLabelledChoice(j, probd,
										((MDP) model).getAction(qp, (int) qptoq.get(c).getX()));

								mdpProd_actionCopies.addActionLabelledChoice(j, probd,
										((MDP) model).getAction(qp, (int) qptoq.get(c).getX()).toString() + c + ed);
								// get the cost
								double cost = 0;
								boolean max_semantics = true;
								if (!q_labels.equals(edges_da_rev.get(ed))) {
									int ap1 = 0;
									long arr[] = q_labels.toLongArray();
									if (arr.length > 0) {
										ap1 = (int) arr[0] + 1;
									}

									int ap2 = 0;
									long arr2[] = edges_da_rev.get(ed).toLongArray();
									if (arr2.length > 0)
										ap2 = (int) arr2[0] + 1;
									if (max_semantics) {

										if (apCosts[ap1] < apCosts[ap2]) {
											cost = apCosts[ap2];
										} else
											cost = apCosts[ap1];
									}

									else {
										// sum
										cost = apCosts[ap1] + apCosts[ap2];
									}
								}
								skipcosts.addToTransitionReward(j, mdpProd_actionCopies.getNumChoices(j), cost);
								mainLog.println("cost: " + q_labels.toString() + "," + edges_da_rev.get(ed).toString()
										+ ":" + cost);
							}
						}
						mainLog.println("(" + qp + "," + zp + ")--" + q_labels.toString() + "," + edrsb.toString()
								+ " --> (" + q + "," + z + ")");
					}
					String info2 = "(d,d') e ->D (" + qp + "," + q + ") " + mdphasedge_rev + " L(d')= "
							+ q_labels.toString() + " (z,z') " + "(" + zp + ", " + z + ") z'=z " + (zp == z)
							+ " t -> z' = d(z,t) = ";
					StringBuilder sb3 = new StringBuilder();
					for (int k = 0; k < edges_da_rev.size(); k++) {

						sb3.append(edges_da_rev.get(k).toString());
						sb3.append("\t");
					}
					info2 = info2 + sb3.toString();

					// mainLog.println(info2);
				}

			}

		}
		int invMap[] = new int[mdpProd_actionCopies.getNumStates()];
		for (int i = 0; i < invMap.length; i++)
			invMap[i] = i;
		
		@SuppressWarnings("unchecked")
		LTLProduct<MDP> product = new LTLProduct<MDP>((MDP) mdpProd_actionCopies, (MDP) model,
				null, daSize, invMap);
		product.setAcceptance(liftAcceptance(product,da.getAcceptance()));
		
		mdpProd.exportToDotFile(saveplace + "prod_oo.dot");
		mdpProd_actionCopies.exportToDotFile(saveplace + "prod_ac.dot");
		//rew = new MDPRewardsSimple(skipcosts);
		
		//todo return an array of objects (product + costs) 

		// prodModel.findDeadlocks(false);
		//
		// if (prodStatesList != null) {
		// prodModel.setStatesList(prodStatesList);
		// }
		//
		// @SuppressWarnings("unchecked")
		// LTLProduct<M> product = new LTLProduct<M>((M) prodModel, model, null,
		// daSize, invMap);
		//
		// // generate acceptance for the product model by lifting
		// product.setAcceptance(liftAcceptance(product, da.getAcceptance()));
		//
		// // lift the labels
		// for (String label : model.getLabels()) {
		// BitSet liftedLabel =
		// product.liftFromModel(model.getLabelStates(label));
		// prodModel.addLabel(label, liftedLabel);
		// }
		//
		Object toret[] = new Object[2];
		toret[0]=product;
		toret[1]=skipcosts;
		//return product;
		return toret;
	}

	/**
	 * Find the set of states that belong to accepting BSCCs in a model wrt an
	 * acceptance condition.
	 * 
	 * @param model
	 *            The model
	 * @param acceptance
	 *            The acceptance condition
	 */
	public BitSet findAcceptingBSCCs(Model model, AcceptanceOmega acceptance) throws PrismException {
		// Compute bottom strongly connected components (BSCCs)
		SCCComputer sccComputer = SCCComputer.createSCCComputer(this, model);
		sccComputer.computeBSCCs();
		List<BitSet> bsccs = sccComputer.getBSCCs();

		BitSet result = new BitSet();

		for (BitSet bscc : bsccs) {
			if (acceptance.isBSCCAccepting(bscc)) {
				// this BSCC is accepting
				result.or(bscc);
			}
		}

		return result;
	}

	/**
	 * Compute the set of states in end components of the model that are
	 * accepting with regard to the acceptance condition.
	 * 
	 * @param model
	 *            the model
	 * @param acceptance
	 *            the acceptance condition
	 * @return BitSet with the set of states that are accepting
	 */
	public BitSet findAcceptingECStates(NondetModel model, AcceptanceOmega acceptance) throws PrismException {
		if (acceptance instanceof AcceptanceBuchi) {
			return findAcceptingECStatesForBuchi(model, (AcceptanceBuchi) acceptance);
		} else if (acceptance instanceof AcceptanceRabin) {
			return findAcceptingECStatesForRabin(model, (AcceptanceRabin) acceptance);
		} else if (acceptance instanceof AcceptanceStreett) {
			return findAcceptingECStatesForStreett(model, (AcceptanceStreett) acceptance);
		} else if (acceptance instanceof AcceptanceGenRabin) {
			return findAcceptingECStatesForGeneralizedRabin(model, (AcceptanceGenRabin) acceptance);
		}
		throw new PrismNotSupportedException("Computing end components for acceptance type '" + acceptance.getType()
				+ "' currently not supported (explicit engine).");
	}

	/**
	 * Find the set of states in accepting end components (ECs) in a
	 * nondeterministic model wrt a Büchi acceptance condition.
	 * 
	 * @param model
	 *            The model
	 * @param acceptance
	 *            The acceptance condition
	 */
	public BitSet findAcceptingECStatesForBuchi(NondetModel model, AcceptanceBuchi acceptance) throws PrismException {
		BitSet allAcceptingStates = new BitSet();

		if (acceptance.getAcceptingStates().isEmpty()) {
			return allAcceptingStates;
		}

		// Compute accepting maximum end components (MECs)
		ECComputer ecComputer = ECComputer.createECComputer(this, model);
		ecComputer.computeMECStates();
		List<BitSet> mecs = ecComputer.getMECStates();
		// Union of accepting MEC states
		for (BitSet mec : mecs) {
			if (mec.intersects(acceptance.getAcceptingStates())) {
				allAcceptingStates.or(mec);
			}
		}

		return allAcceptingStates;
	}

	/**
	 * Find the set of states in accepting end components (ECs) in a
	 * nondeterministic model wrt a Rabin acceptance condition.
	 * 
	 * @param model
	 *            The model
	 * @param acceptance
	 *            The acceptance condition
	 */
	public BitSet findAcceptingECStatesForRabin(NondetModel model, AcceptanceRabin acceptance) throws PrismException {
		BitSet allAcceptingStates = new BitSet();
		int numStates = model.getNumStates();

		// Go through the DRA acceptance pairs (L_i, K_i)
		for (int i = 0; i < acceptance.size(); i++) {
			// Find model states *not* satisfying L_i
			BitSet bitsetLi = acceptance.get(i).getL();
			BitSet statesLi_not = new BitSet();
			for (int s = 0; s < numStates; s++) {
				if (!bitsetLi.get(s)) {
					statesLi_not.set(s);
				}
			}
			// Skip pairs with empty !L_i
			if (statesLi_not.cardinality() == 0)
				continue;
			// Compute accepting maximum end components (MECs) in !L_i
			ECComputer ecComputer = ECComputer.createECComputer(this, model);
			ecComputer.computeMECStates(statesLi_not, acceptance.get(i).getK());
			List<BitSet> mecs = ecComputer.getMECStates();
			// Union MEC states
			for (BitSet mec : mecs) {
				allAcceptingStates.or(mec);
			}
		}

		return allAcceptingStates;
	}

	/**
	 * Find the set of states in accepting end components (ECs) in a
	 * nondeterministic model wrt a Streett acceptance condition.
	 * 
	 * @param model
	 *            The model
	 * @param acceptance
	 *            The Streett acceptance condition
	 */
	public BitSet findAcceptingECStatesForStreett(NondetModel model, AcceptanceStreett acceptance)
			throws PrismException {
		class ECandPairs {
			BitSet MEC;
			BitSet activePairs;
		}

		BitSet allAcceptingStates = new BitSet();
		BitSet allPairs = new BitSet();
		allPairs.set(0, acceptance.size());

		Stack<ECandPairs> todo = new Stack<ECandPairs>();
		ECComputer ecComputer = ECComputer.createECComputer(this, model);
		ecComputer.computeMECStates();
		for (BitSet mecs : ecComputer.getMECStates()) {
			ECandPairs ecp = new ECandPairs();
			ecp.MEC = mecs;
			ecp.activePairs = allPairs;
			todo.push(ecp);
		}

		while (!todo.empty()) {
			ECandPairs ecp = todo.pop();
			BitSet newActivePairs = (BitSet) ecp.activePairs.clone();
			BitSet restrict = null;

			// check for acceptance
			boolean allAccepting = true;
			for (int pair = ecp.activePairs.nextSetBit(0); pair != -1; pair = ecp.activePairs.nextSetBit(pair + 1)) {

				if (!acceptance.get(pair).isBSCCAccepting(ecp.MEC)) {
					// this pair is not accepting
					if (restrict == null) {
						restrict = (BitSet) ecp.MEC.clone();
					}
					restrict.andNot(acceptance.get(pair).getR());
					newActivePairs.clear(pair);
					allAccepting = false;
				}
			}

			if (allAccepting) {
				allAcceptingStates.or(ecp.MEC);
			} else if (restrict.isEmpty()) {
				// nothing to do
			} else {
				ecComputer = ECComputer.createECComputer(this, model);
				ecComputer.computeMECStates(restrict);
				for (BitSet mecs : ecComputer.getMECStates()) {
					ECandPairs newEcp = new ECandPairs();
					newEcp.MEC = mecs;
					newEcp.activePairs = newActivePairs;
					todo.push(newEcp);
				}
			}
		}

		return allAcceptingStates;
	}

	/**
	 * Find the set of states in accepting end components (ECs) in a
	 * nondeterministic model wrt a Generalized Rabin acceptance condition.
	 * 
	 * @param model
	 *            The model
	 * @param acceptance
	 *            The acceptance condition
	 */
	public BitSet findAcceptingECStatesForGeneralizedRabin(NondetModel model, AcceptanceGenRabin acceptance)
			throws PrismException {
		BitSet allAcceptingStates = new BitSet();
		int numStates = model.getNumStates();

		// Go through the GR acceptance pairs (L_i, K_i_1, ..., K_i_n)
		for (int i = 0; i < acceptance.size(); i++) {

			// Find model states *not* satisfying L_i
			BitSet bitsetLi = acceptance.get(i).getL();
			BitSet statesLi_not = new BitSet();
			for (int s = 0; s < numStates; s++) {
				if (!bitsetLi.get(s)) {
					statesLi_not.set(s);
				}
			}
			// Skip pairs with empty !L_i
			if (statesLi_not.cardinality() == 0)
				continue;
			// Compute maximum end components (MECs) in !L_i
			ECComputer ecComputer = ECComputer.createECComputer(this, model);
			ecComputer.computeMECStates(statesLi_not);
			List<BitSet> mecs = ecComputer.getMECStates();
			// Check which MECs contain a state from each K_i_j
			int n = acceptance.get(i).getNumK();
			for (BitSet mec : mecs) {
				boolean allj = true;
				for (int j = 0; j < n; j++) {
					if (!mec.intersects(acceptance.get(i).getK(j))) {
						allj = false;
						break;
					}
				}
				if (allj) {
					allAcceptingStates.or(mec);
				}
			}
		}

		return allAcceptingStates;
	}

	/**
	 * Lift the acceptance condition from the automaton to the product states.
	 */
	private AcceptanceOmega liftAcceptance(final LTLProduct<?> product, AcceptanceOmega acceptance) {
		// make a copy of the acceptance condition
		AcceptanceOmega lifted = acceptance.clone();

		// lift state sets
		lifted.lift(new AcceptanceOmega.LiftBitSet() {
			@Override
			public BitSet lift(BitSet states) {
				return product.liftFromAutomaton(states);
			}
		});

		return lifted;
	}

}
