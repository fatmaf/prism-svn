package demos;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.Vector;

import explicit.MDP;
import explicit.MDPSimple;
import explicit.LTLModelChecker;
import explicit.LTLModelChecker.LTLProduct;
import parser.State;
import parser.VarList;
import prism.PrismLog;

public class SingleAgentNestedProductMDP
{
	/**
	 * 
	 */
	// private final STAPU stapu;
	ArrayList<DAInfo> daList;
	LTLProduct<MDP> finalProduct;
	HashMap<Integer, Integer> productStateToMDPState;
	BitSet combinedAcceptingStates;
	BitSet combinedStatesToAvoid;
	BitSet combinedEssentialStates;
	PrismLog mainLog;
	BitSet allAcceptingStatesCombined; // includes everything even the essential states
	int numMDPVars;

	//so each da has an associated index in the product 
	//the way it works is that its shifted by one for each multiplication 

	public SingleAgentNestedProductMDP(PrismLog log)
	{
		// this.stapu = stapu;
		mainLog = log;
		productStateToMDPState = new HashMap<Integer, Integer>();
	}

	public SingleAgentNestedProductMDP(PrismLog log, ArrayList<DAInfo> list, LTLProduct<MDP> product)
	{
		mainLog = log;
		productStateToMDPState = new HashMap<Integer, Integer>();
		daList = list;
		finalProduct = product;
		mainLog.println("Initializing Single Agent Nested Product MDP. " + "Make sure to update the state mappings");
		setBitSetsForAccEssentialBadStates();
	}

	public SingleAgentNestedProductMDP(SingleAgentNestedProductMDP np, LTLModelChecker LTLmc)
	{
		mainLog = np.mainLog;
		productStateToMDPState = new HashMap<Integer, Integer>();
		productStateToMDPState.putAll(np.productStateToMDPState);
		// np.productStateToMDPState.clone();
		daList = new ArrayList<DAInfo>();
		for (DAInfo dainfo : np.daList) {
			daList.add(new DAInfo(dainfo));
		}
		finalProduct = LTLmc.new LTLProduct<MDP>(np.finalProduct.getProductModel(), np.finalProduct.getOriginalModel(), np.finalProduct.getAcceptance(),
				np.finalProduct.getDASize(), np.finalProduct.getInvMap());
		setBitSetsForAccEssentialBadStates();
	}

	public boolean[] addRewardForTaskCompletion(int childState, int parentState)
	{
		boolean toreturn = false;
		boolean isAvoid = false;
		State cs = this.finalProduct.getProductModel().getStatesList().get(childState);
		State ps = this.finalProduct.getProductModel().getStatesList().get(parentState);
		int countTasks = 0;
		int countAcc = 0;
		for (int i = 0; i < this.daList.size(); i++) {
			if (!daList.get(i).isSafeExpr) {
				countTasks++;
				DAInfo dainfo = daList.get(i);
				int val = (int) cs.varValues[dainfo.associatedIndexInProduct];
				if (dainfo.daAccStates.get(val)) {
					countAcc++;
					if ((int) ps.varValues[dainfo.associatedIndexInProduct] != val) {
						toreturn = true;

					}
				}
			} else {
				DAInfo dainfo = daList.get(i);
				int val = (int) cs.varValues[dainfo.associatedIndexInProduct];
				if (dainfo.daAccStates.get(val)) {
					isAvoid = true;
				}
			}
		}
		boolean[] arrToRet = new boolean[] { toreturn, countTasks == countAcc, isAvoid };
		return arrToRet;
	}

	/**
	 * 
	 * @return bitset - all possible accepting states including essential states
	 */
	public BitSet getAllAcceptingStates()
	{
		int numStates = finalProduct.getProductModel().getNumStates();
		BitSet acceptingStates = new BitSet(numStates);
		// acceptingStates.set(0, numStates);
		for (int i = 0; i < daList.size(); i++) {
			if (!(daList.get(i).isSafeExpr)) {
				acceptingStates.or(daList.get(i).productAcceptingStates);
			}
		}
		return acceptingStates;
	}

	public BitSet getAndSetInitialStates(int state, boolean isMDPState, boolean ignoreSharedStateValues, List<String> sharedVars)
	{
		// State s1 = finalProduct.getProductModel().getStatesList().get(state);
		int numStates = finalProduct.getProductModel().getNumStates();
		BitSet initialStates = new BitSet(numStates);
		int mdpState = state;
		if (!isMDPState)
			mdpState = productStateToMDPState.get(state);
		((MDPSimple) finalProduct.getProductModel()).clearInitialStates();
		((MDPSimple) finalProduct.getProductModel()).addInitialState(state);
		if (!ignoreSharedStateValues) {
			Set<Integer> keySet = productStateToMDPState.keySet();
			for (int key : keySet) {
				if (productStateToMDPState.get(key) == mdpState && !combinedStatesToAvoid.get(key)) {
					initialStates.set(key);

				}
			}
		} else {
			//if we arent ignoring shared state variables we need to match the mdp bits that are not shared 
			State stateVar = finalProduct.getProductModel().getStatesList().get(state);
			VarList pVarList = finalProduct.getProductModel().getVarList();
			//create a list of the indices we want to match 
			Vector<Integer> statesToMatch = new Vector<Integer>();
			for (int i = stateVar.varValues.length - 1; i >= stateVar.varValues.length - numMDPVars; i--) {
				String varname = pVarList.getName(i);
				if (!sharedVars.contains(varname)) {
					statesToMatch.add(i);
				}
			}
			List<State> statesList = finalProduct.getProductModel().getStatesList();
			for (int i = 0; i < statesList.size(); i++) {
				if (StatesHelper.areEqual(stateVar, statesList.get(i), statesToMatch)) {
					initialStates.set(i);
				}
			}
		}

		//		mainLog.println("Initial States: " + initialStates.toString());
		//		mainLog.println("States To Avoid : " + combinedStatesToAvoid.toString());

		return initialStates;
	}

	private BitSet getFinalAcceptingStates()
	{
		int numStates = finalProduct.getProductModel().getNumStates();
		BitSet acceptingStates = new BitSet(numStates);
		acceptingStates.set(0, numStates);
		for (int i = 0; i < daList.size(); i++) {
			if (!(daList.get(i).isSafeExpr)) {
				acceptingStates.and(daList.get(i).productAcceptingStates);
			}
		}
		return acceptingStates;
	}

	private BitSet getFinalEssentialStates()
	{
		int numStates = finalProduct.getProductModel().getNumStates();
		BitSet finalEssentialStates = new BitSet(numStates);

		for (int i = 0; i < daList.size(); i++) {
			if (!(daList.get(i).isSafeExpr)) {
				finalEssentialStates.or(daList.get(i).essentialStates);
			}
		}
		return finalEssentialStates;
	}

	private BitSet getFinalStatesToAvoid()
	{
		int numStates = finalProduct.getProductModel().getNumStates();
		BitSet statesToAvoid = new BitSet(numStates);
		statesToAvoid.set(0, numStates, false);
		for (int i = 0; i < daList.size(); i++) {
			if ((daList.get(i).isSafeExpr)) {
				statesToAvoid.or(daList.get(i).productAcceptingStates);
			}
		}
		return statesToAvoid;
	}

	public BitSet getInitialStates(boolean ignoreSharedStateValues, List<String> sharedVars)
	{
		return getAndSetInitialStates(finalProduct.getProductModel().getFirstInitialState(), false, ignoreSharedStateValues, sharedVars);
	}

	public int getNumMDPVars()
	{
		return numMDPVars;
	}

	public void initializeProductToMDPStateMapping(MDP product)
	{
		for (int s = 0; s < product.getNumStates(); s++) {
			productStateToMDPState.put(s, s);
		}
	}

	public void setBitSetsForAccEssentialBadStates()
	{
		combinedAcceptingStates = getFinalAcceptingStates();
		combinedStatesToAvoid = getFinalStatesToAvoid();
		combinedEssentialStates = getFinalEssentialStates();

		// filter out bad states from acc and essential states
		combinedEssentialStates.andNot(combinedStatesToAvoid);
		combinedAcceptingStates.andNot(combinedStatesToAvoid);

		// make sure no accepting state is a switch state
		combinedEssentialStates.andNot(combinedAcceptingStates);
		//		mainLog.println("Accepting States: " + combinedAcceptingStates.toString());
		//		mainLog.println("Essential States: " + combinedEssentialStates.toString());
		//		mainLog.println("States To Avoid : " + combinedStatesToAvoid.toString());
	}

	public void setDAListAndFinalProduct(LTLProduct<MDP> product)
	{
		// daList = list;
		finalProduct = product;
		setBitSetsForAccEssentialBadStates();
	}

	public void setNumMDPVars(int n)
	{
		numMDPVars = n;
	}

	public void updateProductToMDPStateMapping(LTLProduct<MDP> product)
	{
		HashMap<Integer, Integer> tempProductStateToMDPState = new HashMap<Integer, Integer>();
		// System.out.println("s"+"-"+"oldstate"+"-"+"productStateToMDPState.get(oldstate)"+"-"+"s1.toString()");
		for (int s = 0; s < product.getProductModel().getNumStates(); s++) {
			int oldstate = product.getModelState(s);
			// State s1 = product.getProductModel().getStatesList().get(s);
			// System.out.println(s+"-"+oldstate+"-"+productStateToMDPState.get(oldstate)+"-"+s1.toString());
			tempProductStateToMDPState.put(s, productStateToMDPState.get(oldstate));

		}
		productStateToMDPState = tempProductStateToMDPState;
	}

}