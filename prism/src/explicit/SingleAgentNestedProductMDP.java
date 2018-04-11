package explicit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Set;

import explicit.LTLModelChecker.LTLProduct;
import parser.State;

class SingleAgentNestedProductMDP {
	/**
	 * 
	 */
	private final STAPU stapu;
	ArrayList<DAInfo> daList;
	LTLProduct<MDP> finalProduct;
	HashMap<Integer, Integer> productStateToMDPState;
	BitSet combinedAcceptingStates;
	BitSet combinedStatesToAvoid;
	BitSet combinedEssentialStates;

	public SingleAgentNestedProductMDP(STAPU stapu) {
		this.stapu = stapu;
		productStateToMDPState = new HashMap<Integer, Integer>();
	}

	public SingleAgentNestedProductMDP(STAPU stapu, ArrayList<DAInfo> list, LTLProduct<MDP> product) {
		this.stapu = stapu;
		productStateToMDPState = new HashMap<Integer, Integer>();
		daList = list;
		finalProduct = product;
		this.stapu.mainLogRef.println(
				"Initializing Single Agent Nested Product MDP. " + "Make sure to update the state mappings");
		setBitSetsForAccEssentialBadStates();
	}

	public BitSet getAndSetInitialStates(int state, boolean isMDPState) {
//		State s1 = finalProduct.getProductModel().getStatesList().get(state);
		int numStates = finalProduct.getProductModel().getNumStates();
		BitSet initialStates = new BitSet(numStates);
		int mdpState = state;
		if (!isMDPState)
			mdpState = productStateToMDPState.get(state);
		((MDPSimple) finalProduct.getProductModel()).clearInitialStates();
		((MDPSimple) finalProduct.getProductModel()).addInitialState(state);

		
		Set<Integer> keySet = productStateToMDPState.keySet();
		for (int key : keySet) {
			if (productStateToMDPState.get(key) == mdpState && !combinedStatesToAvoid.get(key)) {
				initialStates.set(key);
				
							}
		}
		this.stapu.mainLogRef.println("Initial States: " + initialStates.toString());
		this.stapu.mainLogRef.println("States To Avoid : " + combinedStatesToAvoid.toString());

		return initialStates;
	}

	private BitSet getFinalAcceptingStates() {
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

	private BitSet getFinalEssentialStates() {
		int numStates = finalProduct.getProductModel().getNumStates();
		BitSet finalEssentialStates = new BitSet(numStates);

		for (int i = 0; i < daList.size(); i++) {
			if (!(daList.get(i).isSafeExpr)) {
				finalEssentialStates.or(daList.get(i).essentialStates);
			}
		}
		return finalEssentialStates;
	}

	private BitSet getFinalStatesToAvoid() {
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

	public BitSet getInitialStates() {
		return getAndSetInitialStates(finalProduct.getProductModel().getFirstInitialState(), false);
	}

	public void initializeProductToMDPStateMapping(MDP product) {
		for (int s = 0; s < product.getNumStates(); s++) {
			productStateToMDPState.put(s, s);
		}
	}

	public void setBitSetsForAccEssentialBadStates() {
		combinedAcceptingStates = getFinalAcceptingStates();
		combinedStatesToAvoid = getFinalStatesToAvoid();
		combinedEssentialStates = getFinalEssentialStates();

		// filter out bad states from acc and essential states
		combinedEssentialStates.andNot(combinedStatesToAvoid);
		combinedAcceptingStates.andNot(combinedStatesToAvoid);

		// make sure no accepting state is a switch state
		combinedEssentialStates.andNot(combinedAcceptingStates);
		this.stapu.mainLogRef.println("Accepting States: " + combinedAcceptingStates.toString());
		this.stapu.mainLogRef.println("Essential States: " + combinedEssentialStates.toString());
		this.stapu.mainLogRef.println("States To Avoid : " + combinedStatesToAvoid.toString());
	}

	public void setDAListAndFinalProduct(ArrayList<DAInfo> list, LTLProduct<MDP> product) {
		daList = list;
		finalProduct = product;
		setBitSetsForAccEssentialBadStates();
	}
	

	public void updateProductToMDPStateMapping(LTLProduct<MDP> product) {
		HashMap<Integer, Integer> tempProductStateToMDPState = new HashMap<Integer, Integer>();
//		System.out.println("s"+"-"+"oldstate"+"-"+"productStateToMDPState.get(oldstate)"+"-"+"s1.toString()");
		for (int s = 0; s < product.getProductModel().getNumStates(); s++) {
			int oldstate = product.getModelState(s);
//			State s1 = product.getProductModel().getStatesList().get(s);
//			System.out.println(s+"-"+oldstate+"-"+productStateToMDPState.get(oldstate)+"-"+s1.toString());
			tempProductStateToMDPState.put(s, productStateToMDPState.get(oldstate));
		

			
		}
		productStateToMDPState = tempProductStateToMDPState;
	}

}