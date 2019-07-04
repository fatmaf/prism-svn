package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import explicit.ModelCheckerResult;
import explicit.ProbModelChecker;
import explicit.LTLModelChecker.LTLProduct;
import parser.State;
import parser.VarList;
import parser.ast.Expression;
import parser.ast.ExpressionProb;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import parser.ast.RewardStruct;
import prism.ModelGenerator;
import prism.ModelInfo;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismLog;
import prism.ProductModelGenerator;
import simulator.ModulesFileModelGenerator;
import strat.MDStrategy;

public class SingleAgentLoader
{
	Prism prism;
	PrismLog mainLog;
	String modelFileName;
	String propertiesFileName;
	AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
	MDStrategy reachProbsSolution = null;
	List<State> solutionStateList;
	List<State> deadendStates;
	List<State> notDeadendStates;
	VarList solutionVarList;
	String resLoc;
	String agentLabel;
	public ProductModelGenerator prodModelGen;
	String latestSolutionInvoked;
	HashMap<Objectives, HashMap<State, Double>> partialSatSolution;
	//	HashMap<Objectives, Integer> rewStructNameIndex;
	HashMap<State, HashMap<Object, Integer>> stateActionIndices;
	boolean printMessages;
	ArrayList<Integer> sharedStateIndices;
	List<String> sharedStateVars;
	List<Double> distToAcc;
	BitSet daAccStates;

	private HashMap<Integer, Integer> prodModGenVarsToSolVars;
	private HashMap<State, State> prodModGenStateToSolState;
	private ModulesFile modulesFile;
	private ExpressionProb expr;
	private LTLModelChecker ltlMC;
	private PropertiesFile propertiesFile;
	private ModulesFileModelGenerator prismModelGen;

	private int daStateIndex = -1;
	private int maxStatesEstimate = -1;

	/*
	 * the prism component should be initialised 
	 */
	public SingleAgentLoader(Prism prismComponent, PrismLog log, String agentLabel, String modelfn, String propfn, String resLoc, List<String> sharedStateVars)
	{
		prism = prismComponent;
		mainLog = log;
		modelFileName = modelfn;
		propertiesFileName = propfn;
		this.resLoc = resLoc;
		this.agentLabel = agentLabel;
		this.sharedStateVars = sharedStateVars;
		printMessages = true;
		latestSolutionInvoked = null;
		if (printMessages) {
			mainLog.println("Single Agent Loader Initialised For " + agentLabel);
			mainLog.println("Remember to run the setup function");
			mainLog.println("Set printmessages off to turn off these messages");
		}
		this.stateActionIndices = new HashMap<State, HashMap<Object, Integer>>();
	}

	public void setPrintMessagesOn()
	{
		this.printMessages = true;
	}

	public void setPrintMessagesOff()
	{
		this.printMessages = false;
	}

	private void updateSharedStateIndices(VarList vl)
	{
		if (sharedStateVars != null) {
			//		if(this.sharedStateIndices == null)
			sharedStateIndices = new ArrayList<Integer>();
			for (int i = 0; i < this.sharedStateVars.size(); i++) {
				String ssN = sharedStateVars.get(i);
				int ssI = vl.getIndex(ssN);
				sharedStateIndices.add(ssI);
			}
		}
	}

	public MDStrategy solveUsingVI() throws PrismException
	{

		if (printMessages) {

			mainLog.println("Solving " + agentLabel + " using reach probs");
		}

		latestSolutionInvoked = "VI Reach Probs";
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

		solutionVarList = prodmdp.getVarList();
		solutionStateList = prodmdp.getStatesList();
		updateSharedStateIndices(solutionVarList);
		BitSet acc = tempda.getAccStates();

		ModelCheckerResult result = mc.computeReachProbs(prodmdp, acc, false);

		//		PrismLog out = new PrismFileLog(resLoc+"pmdp.dot"); 
		//		prodmdp.exportToDotFile(out, null, true);
		//		out.close();
		this.maxStatesEstimate = mdp.getNumStates();
		MDStrategy strat = (MDStrategy) (result.strat);
		this.reachProbsSolution = strat;
		return strat;
	}

	public int getMaxStatesEstimate()
	{
		return this.maxStatesEstimate;
	}

	public void setMaxStatesEstimate(int update)
	{
		this.maxStatesEstimate = update;
	}

	public HashMap<Objectives, HashMap<State, Double>> solveUsingPartialSatisfaction() throws PrismException
	{
		if (printMessages) {
			mainLog.println("Solving " + agentLabel + " using partial satisfaction");
		}
		latestSolutionInvoked = "Partial Sat";
		ModelGenerator modelGen = this.prismModelGen;
		ModelInfo modelInfo = this.modulesFile;

		prism.buildModel();
		MDP mdp = (MDP) prism.getBuiltModelExplicit();

		MDPModelChecker mc = new MDPModelChecker(prism);

		mc.setGenStrat(true);
		mc.setExportAdv(true);

		mc.setModulesFileAndPropertiesFile(modelInfo, propertiesFile, modelGen);
		boolean exportAdv = true;
		String savePlace = resLoc + agentLabel + "_partsat";

		ArrayList<VarList> varlist = new ArrayList<VarList>();

		PolicyCreator pc = new PolicyCreator();
		HashMap<String, HashMap<State, Double>> result = mc.checkPartialSatForBounds(mdp, expr.getExpression(), null, varlist, exportAdv, savePlace, pc);
		solutionVarList = varlist.get(0);
		updateSharedStateIndices(solutionVarList);

		partialSatSolution = new HashMap<Objectives, HashMap<State, Double>>();
		Objectives obj;
		for (String r : result.keySet()) {
			if (r.contentEquals("cost"))
				obj = Objectives.Cost;
			else if (r.contentEquals("prog"))
				obj = Objectives.Progression;
			else if (r.contentEquals("prob"))
				obj = Objectives.Probability;
			else
				throw new PrismException("Hain?");

			partialSatSolution.put(obj, result.get(r));

		}
		maxStatesEstimate = mdp.getNumStates();

		//		this.rewStructNameIndex = new HashMap<Objectives, Integer>();
		//		rewStructNameIndex.put(Objectives.Cost, 1);
		//		rewStructNameIndex.put(Objectives.Progression, 2);
		//		rewStructNameIndex.put(Objectives.Probability, 0);
		if (pc != null)
			pc.savePolicy(savePlace, "_partsat");
		return partialSatSolution;

	}

	public void setUp() throws PrismException, FileNotFoundException
	{
		if (printMessages) {
			mainLog.println("Setting up for " + agentLabel);
		}
		modulesFile = prism.parseModelFile(new File(modelFileName));
		prism.loadPRISMModel(modulesFile);

		propertiesFile = prism.parsePropertiesFile(modulesFile, new File(propertiesFileName));

		expr = (ExpressionProb) propertiesFile.getProperty(0);
		ltlMC = new LTLModelChecker(prism);

		prismModelGen = new ModulesFileModelGenerator(modulesFile, prism);

	}

	public void cleanUp()
	{
		modulesFile = null;
		propertiesFile = null;
		ltlMC = null;
		expr = null;
		prismModelGen = null;
	}

	public DA<BitSet, ? extends AcceptanceOmega> getSingleAgentModelGenReturnDA() throws FileNotFoundException, PrismException
	{

		List<Expression> labelExprs = new ArrayList<Expression>();

		DA<BitSet, ? extends AcceptanceOmega> da = ltlMC.constructExpressionDAForLTLFormula(expr.getExpression(), labelExprs, allowedAcceptance);
		da.setDistancesToAcc();
		daAccStates = da.getAccStates();
		PrismLog out = new PrismFileLog(resLoc + "da.dot");
		//printing the da 
		da.print(out, "dot");
		out.close();
		prodModelGen = new ProductModelGenerator(prismModelGen, da, labelExprs);
		prism.loadModelGenerator(prismModelGen);
		return da;

	}

	State getSolutionState(State prodModGenState)
	{
		if (prodModGenVarsToSolVars != null) {
			if (this.prodModGenStateToSolState == null)
				prodModGenStateToSolState = new HashMap<State, State>();
			if (prodModGenStateToSolState.containsKey(prodModGenState)) {
				return prodModGenStateToSolState.get(prodModGenState);
			} else {
				State solState = new State(solutionVarList.getNumVars());

				for (int i = 0; i < prodModGenState.varValues.length; i++) {
					solState.setValue(prodModGenVarsToSolVars.get(i), prodModGenState.varValues[i]);
				}
				prodModGenStateToSolState.put(prodModGenState, solState);
				return solState;
			}
		}
		return null;
	}

	public double getSolutionValue(State prodModGenState, Objectives objective) throws PrismException
	{
		if (partialSatSolution == null) {
			return getSolutionValueReachProbs(prodModGenState, objective);
		} else {
			return getSolutionValuePartialSat(prodModGenState, objective);
		}
	}

	public double getSolutionValuePartialSat(State prodModGenState, Objectives objective)
	{
		State solState = getSolutionState(prodModGenState);
		if (solState != null) {
			//			int costIndex = this.rewStructNameIndex.get(objective);
			HashMap<State, Double> solutionValues = this.partialSatSolution.get(objective);
			if (solutionValues.containsKey(solState))
				return solutionValues.get(solState);
		}
		return 0.0;
	}

	public double getSolutionValueReachProbs(State prodModGenState, Objectives objective) throws PrismException
	{
		if (objective != Objectives.Probability) {
			//			if(printMessages)
			mainLog.println("Invalid objective for reach probs VI");
			return 0.0;
		}
		State solState = getSolutionState(prodModGenState);
		if (solutionStateList != null) {
			if (solutionStateList.contains(solState)) {
				//TODO: fill in what to do here 
				//I've really forgotten 
				throw new PrismException("Not Implemented - getSolutionValueReachProbs");

			}
		}
		throw new PrismException("Error in getSolutionValueReachProbs");
		//		return 0.0; 
	}

	public boolean solutionProdModelVarListsAreSynced() throws PrismException
	{
		boolean synced = false;
		VarList prodModelGenVarList = null;
		if (prodModelGen != null) {
			prodModelGenVarList = prodModelGen.createVarList();
		}

		int solutionDAVarNum = findDANum(solutionVarList);
		int prodModelDAVarNum = findDANum(prodModelGenVarList);

		if (solutionDAVarNum != -1 && prodModelDAVarNum != -1) {
			if (solutionDAVarNum != prodModelDAVarNum) {
				updateSharedStateIndices(prodModelGenVarList);
				prodModGenVarsToSolVars = new HashMap<Integer, Integer>();
				for (int i = 0; i < solutionVarList.getNumVars(); i++) {
					String name = solutionVarList.getName(i);
					if (name.contains("da")) {
						if (name.equalsIgnoreCase("_da")) {
							name = "_da0";
						}
					}
					int prodModelGenVarIndex = prodModelGenVarList.getIndex(name);
					if (name.contentEquals("_da0"))
						this.daStateIndex = prodModelGenVarIndex;
					prodModGenVarsToSolVars.put(prodModelGenVarIndex, i);

				}
				synced = false;
			} else
				synced = true;
		} else {
			synced = true;
			//don't need to sync
		}

		if (printMessages) {
			String txt = "Solution ";
			if (latestSolutionInvoked != null) {
				txt += "(" + latestSolutionInvoked + ")";
				txt += " and Product Model ";
				if (!synced)
					txt += "NOT ";
				txt += "synced";
			} else {
				txt = "No solution used so far";
			}
			mainLog.println(txt);
		}

		return synced;
	}

	private int findDANum(VarList vl)
	{
		String daString = "_da";
		int daNum = -1;
		//assuming a single da 
		if (vl != null) {
			for (int i = 0; i < vl.getNumVars(); i++) {
				if (vl.getName(i).contains(daString)) {
					daNum = i;
					break;
				}
			}
		}
		return daNum;
	}

	public boolean isDeadend(State prodModGenState)
	{
		if (deadendStates == null) {
			deadendStates = new ArrayList<State>();

		}
		if (deadendStates.contains(prodModGenState))
			return true;
		if (notDeadendStates == null) {
			notDeadendStates = new ArrayList<State>();
		}
		if (notDeadendStates.contains(prodModGenState))
			return false;
		if (this.getSolutionValuePartialSat(prodModGenState, Objectives.Progression) == 0.0) {
			deadendStates.add(prodModGenState);
			return true;
		} else {
			notDeadendStates.add(prodModGenState);
		}
		return false;

	}

	public State getDAState(State s)
	{
		if (daStateIndex != -1) {
			return s.substate(daStateIndex, daStateIndex + 1);
		}
		return null;

	}

	public int getDAStateAsInt(State s)
	{
		State daState = getDAState(s);
		if (daState != null) {
			//assumption da state is only one variable 
			return (int) daState.varValues[0];
		}
		return -1;
	}

	public boolean hasSharedStates()
	{
		return sharedStateIndices != null;
	}

	public State getSharedState(State s)
	{

		if (hasSharedStates()) {
			State ss = new State(sharedStateIndices.size());
			for (int i = 0; i < this.sharedStateIndices.size(); i++) {
				int ssI = sharedStateIndices.get(i);
				Object ssV = s.varValues[ssI];
				ss.setValue(i, ssV);
			}
			return ss;
		}
		return null;
	}

	public State getPrivateState(State s) throws PrismException
	{

		int psSize = solutionVarList.getNumVars() - 1;
		if (hasSharedStates()) {
			psSize = psSize - sharedStateIndices.size();
		}
		State ps = new State(psSize);
		int psi = 0;
		boolean isPS;
		for (int i = 0; i < solutionVarList.getNumVars(); i++) {
			isPS = true;
			if (hasSharedStates()) {
				if (sharedStateIndices.contains(i))
					isPS = false;

			}
			if (i == this.daStateIndex)
				isPS = false;
			if (isPS) {
				Object psV = s.varValues[i];
				ps.setValue(psi, psV);
				psi++;
			}
		}
		if (psi != psSize) {
			throw new PrismException("Error in getting the private state?");
		}
		return ps;

	}

	public State createRobotState(State ps, State ss, State da) throws PrismException
	{
		Object[] psV = ps.varValues;
		Object[] ssV = null;
		if (ss != null)
			ssV = ss.varValues;
		Object[] daV = da.varValues;
		return createRobotState(psV, ssV, daV);
	}

	private State createRobotState(Object[] psV, Object[] ssV, Object[] daV) throws PrismException
	{
		int nsSize = solutionVarList.getNumVars();
		State ns = new State(nsSize);
		int psSize = psV.length;
		int ssSize = 0;
		if (ssV != null)
			ssSize = ssV.length;
		int daSize = daV.length;
		if (nsSize == (psSize + ssSize + daSize)) {
			int psI = 0;
			int ssI = 0;
			Object sVar;
			for (int i = 0; i < nsSize; i++) {

				if (sharedStateIndices != null && sharedStateIndices.contains(i)) {
					if (ssV != null) {
						sVar = ssV[ssI];
						ssI++;
						if (ssI > ssSize)
							throw new PrismException("Shared state size exceeding actual size??");
					} else
						sVar = "";
				} else {
					if (i == this.daStateIndex) {
						//assuming a single DA state
						sVar = daV[0];
					} else {
						sVar = psV[psI];
						psI++;
						if (psI > psSize) {
							throw new PrismException("Private state size exceeding actual size??");
						}

					}
				}
				ns.setValue(i, sVar);
			}
			return ns;
		}

		throw new PrismException("Problem with creating the robot state from joint state");
	}

	BitSet getStateLabels(State s) throws PrismException
	{
		BitSet trueLabels = new BitSet();
		prodModelGen.exploreState(s);
		for (int i = 0; i < prodModelGen.getNumLabelExprs(); i++) {
			if (prodModelGen.isExprTrue(i)) {
				trueLabels.set(i);
			}
		}
		return trueLabels;
	}

	HashMap<Object, Integer> getActionsForState(State s) throws PrismException
	{
		HashMap<Object, Integer> actionIndices = null;
		if (stateActionIndices.containsKey(s)) {
			actionIndices = stateActionIndices.get(s);

		} else {
			actionIndices = new HashMap<Object, Integer>();
			prodModelGen.exploreState(s);
			//get all the actions for this state 
			int numChoices = prodModelGen.getNumChoices();
			for (int i = 0; i < numChoices; i++) {
				Object action = prodModelGen.getChoiceAction(i);
				actionIndices.put(action, i);
			}
			if (numChoices == 0) {
				actionIndices.put("*", -1);
			}
			stateActionIndices.put(s, actionIndices);

		}
		return actionIndices;
	}

	ArrayList<Entry<State, Double>> getStateActionSuccessors(State s, Object a) throws PrismException
	{
		int choiceNum = getStateActionChoiceNum(s, a);
		ArrayList<Entry<State, Double>> succStates = new ArrayList<Entry<State, Double>>();

		if (choiceNum > -1) {
			int numTransitions = getNumTransitions(s, a, choiceNum);
			for (int i = 0; i < numTransitions; i++) {
				double prob = prodModelGen.getTransitionProbability(choiceNum, i);
				State succState = prodModelGen.computeTransitionTarget(choiceNum, i);
				succStates.add(new AbstractMap.SimpleEntry<State, Double>(succState, prob));
			}
		}
		//TODO: what to do if there are no sucessors ?? 
		//just putting this here 
		if (succStates.size() == 0)
			succStates.add(new AbstractMap.SimpleEntry<State, Double>(s, 1.0));
		return succStates;
	}

	int getStateActionChoiceNum(State s, Object a) throws PrismException
	{
		int choiceNum = -1;
		if (!stateActionIndices.containsKey(s)) {
			getActionsForState(s);
		}
		if (stateActionIndices.containsKey(s)) {
			if (stateActionIndices.get(s).containsKey(a)) {

				choiceNum = stateActionIndices.get(s).get(a);
			}

			else
				throw new PrismException("action not in state " + s.toString() + " " + a.toString());
		} else
			throw new PrismException("hmmm??? state not in state action indices");

		return choiceNum;
	}

	/*
	 * Gets the number of transitions 
	 * if c==-1 gets the choice number otherwise tries to look for choice number c
	 */

	int getNumTransitions(State s, Object a, int c) throws PrismException
	{
		int numTrans = -1;
		int choiceNum = c;
		if (c == -1)
			choiceNum = getStateActionChoiceNum(s, a);

		if (prodModelGen.getExploreState() != s)
			prodModelGen.exploreState(s);

		numTrans = prodModelGen.getNumTransitions(choiceNum);

		return numTrans;

	}

	double getStateReward(State s, String rew) throws PrismException
	{
		int rewInt = prodModelGen.getRewardStructIndex(rew);
		return getStateReward(s, rewInt);
	}

	double getStateReward(State s, int rew) throws PrismException
	{
		//		RewardStruct rewStruct = prodModelGen.getRewardStruct(rew); 
		return prodModelGen.getStateReward(rew, s);

	}

	double getStateActionReward(State s, Object a, int rew) throws PrismException
	{
		return prodModelGen.getStateActionReward(rew, s, a);
	}

	double getStateActionReward(State s, Object a, String rew) throws PrismException
	{
		int rewInd = prodModelGen.getRewardStructIndex(rew);
		return getStateActionReward(s, a, rewInd);
	}

}
