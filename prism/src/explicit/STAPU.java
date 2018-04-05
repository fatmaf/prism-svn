/**
 * 
 */
package explicit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceRabin;
import acceptance.AcceptanceReach;
import acceptance.AcceptanceType;
import automata.DA;
import explicit.LTLModelChecker.LTLProduct;
import explicit.rewards.MDPRewardsSimple;
import explicit.rewards.Rewards;
import parser.State;
import parser.VarList;
import parser.ast.Declaration;
import parser.ast.DeclarationInt;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionQuant;
import parser.ast.ExpressionReward;
import parser.ast.RewardStruct;
import prism.PrismComponent;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLangException;
import prism.PrismLog;

/**
 * @author fatma
 *
 */
public class STAPU extends ProbModelChecker {

	boolean run_tests = false;
	int BADVALUE = -2;
	String saveplace = "/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/temp/";
	// System.getProperty("user.dir") + "/tests/decomp_tests/temp/";
	boolean printHighlights = false;
	boolean printDetails = false;
	private boolean saveIntermediates = false;
	boolean printTestRelated = true;
	ArrayList<ArrayList<Entry<String, Double>>> testResults;
	int arrNum = 0;
	long timeout = 1000 * 3 * 60;
	long time = 0;
	
	public STAPU(PrismComponent parent) throws PrismException {
		super(parent);
		// TODO Auto-generated constructor stub
	}
	
	@Override
	protected StateValues checkExpressionFunc(Model model, ExpressionFunc expr, BitSet statesOfInterest)
			throws PrismException {
		switch (expr.getNameCode()) {
		case ExpressionFunc.STAPU: {
			mainLog.println("OMMG THIS WORKS");
			
			return doSTAPUU(model,expr,statesOfInterest); //return function name; 
	
		}
		default:
			return super.checkExpressionFunc(model, expr, statesOfInterest);
		}
	}
	
	
	/*
	 * Return a list of expressions 
	 */
	protected ArrayList<Expression> getLTLExpressions(ExpressionFunc expr) throws PrismException
	{
		int numOp = expr.getNumOperands(); 
		String exprString = "";
		ArrayList<Expression> ltlExpressions= new ArrayList<Expression>(numOp);
		for(int exprNum = 0; exprNum< numOp; exprNum++) {
			if(expr.getOperand(exprNum) instanceof ExpressionQuant)
			{
				ltlExpressions.add(((ExpressionQuant)expr.getOperand(exprNum)));
				exprString += ((ExpressionQuant)ltlExpressions.get(exprNum)).getExpression().toString();
			}
		}
		
		mainLog.println("LTL Mission: "+exprString);
		return ltlExpressions; 
	}
	
	class DAInfo{
		//BitSet acceptingStates; 
		BitSet productAcceptingStates; 
		BitSet essentialStates; 
		boolean isSafeExpr; 
		DA<BitSet, ? extends AcceptanceOmega> da; 
		Expression daExpr;
		Vector<BitSet> labelBS; 
		Rewards costsModel;
		
		public DAInfo(Expression expr)
		{
			daExpr = expr; 
			isSafeExpr = !Expression.isCoSafeLTLSyntactic(daExpr, true); 
			if(isSafeExpr)
			{
				daExpr = Expression.Not(daExpr); 
			}
			
		}
		public <M extends Model> LTLProduct<M> constructDAandProductModel(LTLModelChecker mcLTL, 
				ProbModelChecker mcProb,AcceptanceType[] accType,M model,
				BitSet statesOfInterest, boolean allStatesInDFA
				) throws PrismException
		{
			labelBS = new Vector<BitSet>();
			da = mcLTL.constructDAForLTLFormula(mcProb, model,daExpr, labelBS, accType);
			if (!(da.getAcceptance() instanceof AcceptanceReach))
			{
				mainLog.println("\nAutomaton is not a DFA... "); 
			}
			else
			{
				BitSet acceptingStates = ((AcceptanceReach)da.getAcceptance()).getGoalStates(); 
			}
			
			LTLProduct<M> product = mcLTL.constructProductModel(da, model, labelBS, statesOfInterest, allStatesInDFA);
			
			//update labelBS 
			Vector<BitSet> newLabelBS = new Vector<BitSet>();
			for(int bs = 0; bs<labelBS.size(); bs++)
			newLabelBS.add(product.liftFromAutomaton(labelBS.get(bs)));
			productAcceptingStates = ((AcceptanceReach)product.getAcceptance()).getGoalStates();
			
			//rewards 
			
			RewardStruct costStruct = ((ExpressionReward)daExpr).getRewardStructByIndexObject(modulesFile, modulesFile.getConstantValues());
			costsModel = constructRewards(model,costStruct); 
			
			return product;
		}
		
		public <M extends Model> void updateStateNumbers(LTLProduct<M> product)
		{
			int numStates = product.getProductModel().getNumStates(); 
			BitSet newProductAcceptingStates = new BitSet();
			BitSet newEssentialStates = new BitSet();
			Vector<BitSet> newLabelBS = new Vector<BitSet>();
			for(int bs = 0; bs < labelBS.size(); bs++)
				newLabelBS.add(new BitSet());
			for(int s = 0; s<numStates; s++)
			{
				int oldstate= product.getModelState(s); 
				if(productAcceptingStates.get(oldstate))
				{
					newProductAcceptingStates.set(s);
				}
				if (essentialStates.get(oldstate))
				{
					newEssentialStates.set(s);
				}
				for(int bs = 0; bs < labelBS.size(); bs++)
				{
					if(labelBS.get(bs).get(oldstate))
					{
						newLabelBS.get(bs).set(s);
					}
				}
			}
			productAcceptingStates = newProductAcceptingStates; 
			essentialStates = newEssentialStates; 
			labelBS = newLabelBS;
			
		}
		public BitSet getEssentialStates(MDP prod) {
			// check if an accepting state is connected to a non accepting state
			// basically I want a bitset of all the edges and and it with !(accepting
			// states), if its not null I know
			BitSet accs = productAcceptingStates;
			int numstates = prod.getNumStates();

			BitSet accsCopy = (BitSet) accs.clone();
			essentialStates = new BitSet(numstates);
			int setbit = -1;
			for (int s = 0; s < numstates; s++) {
				if (!accs.get(s)) // if not an accepting state
				{
					// if(prod.someSuccessorsInSet(s, accs)) //check if any accepting state is in
					// the succ set
					setbit = accsCopy.nextSetBit(0);

					while (setbit != -1) {
						if (prod.isSuccessor(s, setbit)) {
							essentialStates.set(setbit);
							accsCopy.clear(setbit);
						}

						setbit = accsCopy.nextSetBit(setbit + 1);
					}
				}
			}
			return essentialStates;

		}
		
	};
	
	protected ArrayList<DAInfo> initializeDAInfoFromLTLExpressions(ArrayList<Expression> exprs)
	{
		int numExprs = exprs.size();
		ArrayList<DAInfo> daInfoList = new ArrayList<DAInfo>(numExprs); 
		
		for(int daNum = 0; daNum < numExprs; daNum++)
		{
			Expression thisExpr = ((ExpressionQuant)exprs.get(daNum)).getExpression();
			DAInfo daInfo = new DAInfo(thisExpr); 			
			daInfoList.add( daInfo);
		}
		
		
		return daInfoList;
	}
	protected void saveMDP(MDP mdp, BitSet statesToMark,String name,boolean saveinsaveplace)
	{
		String temp = exportAdvFilename;
		temp = temp.replace("adv", ""); 
		temp = temp.replaceAll(".tra", "");
		String location = temp; 
		if (saveinsaveplace)
		{
			location = saveplace;
			temp = temp.substring(temp.lastIndexOf("/")+1,temp.length());
			location+=temp;
		}
		name = name.replace(" ", "_");
		PrismLog out = new PrismFileLog(location+name+".dot"); 
		mdp.exportToDotFile(out, statesToMark, true);
		out.close();
		
	}
	class SingleAgentNestedProductMDP{
		ArrayList<DAInfo> daList; 
		LTLProduct<MDP> finalProduct;
		HashMap<Integer,Integer> productStateToMDPState; 
		BitSet combinedAcceptingStates; 
		BitSet combinedStatesToAvoid; 
		BitSet combinedEssentialStates;
		
		public SingleAgentNestedProductMDP(ArrayList<DAInfo> list, LTLProduct<MDP> product)
		{
			productStateToMDPState = new HashMap<Integer,Integer>();
			daList = list; 
			finalProduct = product;
			mainLog.println("Initializing Single Agent Nested Product MDP. "
					+ "Make sure to update the state mappings");
			setBitSetsForAccEssentialBadStates();
		}
		public SingleAgentNestedProductMDP()
		{
			productStateToMDPState = new HashMap<Integer,Integer>(); 
		}
		public void setDAListAndFinalProduct(ArrayList<DAInfo> list, LTLProduct<MDP> product)
		{
			daList = list; 
			finalProduct = product;
			setBitSetsForAccEssentialBadStates();
		}
		public void initializeProductToMDPStateMapping(MDP product)
		{
			for(int s = 0; s<product.getNumStates(); s++)
			{
				productStateToMDPState.put(s, s);
			}
		}
		public void updateProductToMDPStateMapping(LTLProduct<MDP> product)
		{
			HashMap<Integer,Integer> tempProductStateToMDPState = new HashMap<Integer,Integer>(); 
			for(int s = 0; s<product.getProductModel().getNumStates(); s++)
			{
				int oldstate = product.getModelState(s);
				tempProductStateToMDPState.put(s, productStateToMDPState.get(oldstate));
			}
			productStateToMDPState = tempProductStateToMDPState; 
		}
		private BitSet getFinalAcceptingStates()
		{
			int numStates = finalProduct.getProductModel().getNumStates();
			BitSet acceptingStates = new BitSet(numStates);
			acceptingStates.set(0,numStates);
			for(int i = 0; i<daList.size(); i++)
			{
				if(!(daList.get(i).isSafeExpr))
				{
					acceptingStates.and(daList.get(i).productAcceptingStates);
				}
			}
			return acceptingStates;
		}
		private BitSet getFinalEssentialStates()
		{
			int numStates = finalProduct.getProductModel().getNumStates();
			BitSet finalEssentialStates = new BitSet(numStates);

			for(int i = 0; i<daList.size(); i++)
			{
				if(!(daList.get(i).isSafeExpr))
				{
					finalEssentialStates.or(daList.get(i).essentialStates);
				}
			}
			return finalEssentialStates;
		}
		private BitSet getFinalStatesToAvoid()
		{
			int numStates = finalProduct.getProductModel().getNumStates();
			BitSet statesToAvoid = new BitSet(numStates);
			statesToAvoid.set(0,numStates,false);
			for(int i = 0; i<daList.size(); i++)
			{
				if((daList.get(i).isSafeExpr))
				{
					statesToAvoid.or(daList.get(i).productAcceptingStates);
				}
			}
			return statesToAvoid;
		}
		public void setBitSetsForAccEssentialBadStates()
		{
			combinedAcceptingStates = getFinalAcceptingStates(); 
			combinedStatesToAvoid = getFinalStatesToAvoid(); 
			combinedEssentialStates = getFinalEssentialStates(); 
			
			//filter out bad states from acc and essentiall states 
			combinedEssentialStates.andNot(combinedStatesToAvoid);
			combinedAcceptingStates.andNot(combinedStatesToAvoid);
			
			//make sure no accepting state is a switch state 
			combinedEssentialStates.andNot(combinedAcceptingStates);
		}
		public BitSet getInitialStates()
		{
			return getInitialStates(finalProduct.getProductModel().getFirstInitialState(),false);
		}
		public BitSet getInitialStates(int state, boolean isMDPState)
		{
			int numStates = finalProduct.getProductModel().getNumStates();
			BitSet initialStates = new BitSet(numStates); 
			int mdpState = state; 
			if(!isMDPState)
			mdpState = productStateToMDPState.get(state);
			
			Set<Integer> keySet = productStateToMDPState.keySet(); 
			for( int key: keySet)
			{
				if (productStateToMDPState.get(key) == mdpState)
				{
					initialStates.set(key);
				}
			}
			
			return initialStates; 
		}
		
	};
	
	/*
	 * @param model - the mdp model 
	 * @param exprs - array list of expressions 
	 * @param statesOfInterest - states to care about - we care about everything so we don't really need this 
	 * 
	 */
	protected SingleAgentNestedProductMDP buildSingleAgentNestedProductMDP(Model model, ArrayList<DAInfo> daList, BitSet statesOfInterest) throws PrismException
	{
		//return the list of daInfo and the product mdp 
		SingleAgentNestedProductMDP res = new SingleAgentNestedProductMDP();
		res.initializeProductToMDPStateMapping((MDP)model);
		LTLProduct<MDP> product = null; 
		MDP productMDP = null; 
		LTLModelChecker mcLTL = new LTLModelChecker(this); //is this okay ? 
		AcceptanceType[] allowedAcceptance = {AcceptanceType.RABIN, AcceptanceType.REACH}; 
		
		int numStates = model.getNumStates(); 
		BitSet bsInit = new BitSet(numStates); 
		//all the states are states of interest 
		bsInit.set(0, numStates);
		productMDP = (MDP)model;
		for(int daNum = 0; daNum < daList.size(); daNum++)
		{
			DAInfo daInfo = daList.get(daNum); 
			product = daInfo.constructDAandProductModel(mcLTL,this,allowedAcceptance,productMDP,null,true);
			productMDP = product.getProductModel();
			daInfo.getEssentialStates(productMDP);
			saveMDP(productMDP,daInfo.productAcceptingStates,"pda_"+daNum,true);
			//update state numbers 
			for(int otherDAs = 0; otherDAs < daNum; otherDAs++)
			{
				daList.get(otherDAs).updateStateNumbers(product);
			}
			res.updateProductToMDPStateMapping(product);
		}
		res.setDAListAndFinalProduct(daList,product);
		return res;
	}
	
	protected void buildSequentialTeamMDP(ArrayList<SingleAgentNestedProductMDP> agentMDPs ) throws PrismLangException
	{
		
		int numRobots = agentMDPs.size(); 
		int numTeamStates = 0; 
		MDPSimple teamMDP = null;
		//I dont know how many rewards there are 
		//so lets just set like this unique rewards model thing ? yes okay 
		//its still an arraylist though 
		//woot 
		ArrayList<MDPRewardsSimple> teamRewardsList = null; 
		// number of states in team = sum of number of states in each mdp 
		for(int r = 0; r<numRobots; r++)
		{
			numTeamStates+=agentMDPs.get(r).finalProduct.getProductModel().getNumStates(); 
		}
		BitSet acceptingStates = new BitSet(numTeamStates);				//for the team mdp they are acc for r1 || acc for r2 || acc for r3 ... 
		BitSet statesToAvoid= new BitSet(numTeamStates);				//for the team mdp they are bad for r1 || bad for r2 || bad for r3 

		ArrayList<BitSet> initialStates = new ArrayList<BitSet>(); 		//for the team mdp they are different for each robot 
		ArrayList<BitSet> switchStates= new ArrayList<BitSet>(); 		//for the team mdp they are different for each robot 
		
		acceptingStates.set(0,numTeamStates);
		statesToAvoid.set(0,numTeamStates,false);
		
		VarList teamMDPVarList = null; 
		
		//the variable list - r,da0,da1...mdp
		//so we can just take the varlist for r1 and do the "needful" rukhsana :P 
		
		MDP productMDP = agentMDPs.get(0).finalProduct.getProductModel(); 
		if(productMDP.getVarList() != null)
		{
			teamMDPVarList = (VarList)(productMDP.getVarList()).clone(); 
			
			String daVar = "_da";			//TODO: come back to fix this 
			while (teamMDPVarList.getIndex(daVar) != -1) {
				daVar = "_" + daVar;
		}
			// NB: if DA only has one state, we add an extra dummy state
			DeclarationInt daint = new DeclarationInt(Expression.Int(0), Expression.Int(Math.max(numRobots - 1, 1)));
			Declaration decl = new Declaration(daVar, daint);
			teamMDPVarList.addVar(0, decl, 1, productMDP.getConstantValues());
		}
		
		teamMDP = new MDPSimple(); 
		teamRewardsList = new ArrayList<MDPRewardsSimple>();
		
		teamMDP.setVarList(teamMDPVarList);
		
		ArrayList<State> teamMDPStatesList = null, robotNumList = null; 
		
		//again making assumption that what works for r1 works for all of them 
		if(productMDP.getStatesList()!=null) {
			teamMDPStatesList = new ArrayList<State>(); 
			robotNumList = new ArrayList<State>(numRobots); 
			
			for(int i = 0; i< numRobots; i++) {
				robotNumList.add(new State(1).setValue(0, i)); 
			}
		}
		
		
		
		
		
		
	}
	
	protected StateValues doSTAPUU(Model model, ExpressionFunc expr, BitSet statesOfInterest) throws PrismException {
		
		//process ltl expressions 
		int numRobots = 3; 
		ArrayList<SingleAgentNestedProductMDP> singleAgentProductMDPs = 
				new ArrayList<SingleAgentNestedProductMDP>();
		ArrayList<Expression> ltlExpressions = getLTLExpressions(expr);
		ArrayList<DAInfo> daList = initializeDAInfoFromLTLExpressions(ltlExpressions);
		for(int i = 0; i<numRobots; i++) {
		SingleAgentNestedProductMDP nestedProduct = 
				buildSingleAgentNestedProductMDP(model,daList,statesOfInterest);
		singleAgentProductMDPs.add(nestedProduct);
		}
		
		
		//create team automaton from a set of MDP DA stuff 
		
		
		//solve 
		
		//add to joint policy 
		
		//repeat 
		
		
		
		return null; 
	}
	

}
