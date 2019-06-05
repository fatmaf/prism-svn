//package demos;
//
//import explicit.MDPSimple;
//
//public class THTS
//{
//
//	//
//	public THTS(MDPSimple[] mdps, long timeout)
//	{
//		THTSNode n0 = getRootNode(mdps); 
//		while (!n0.solved() && timePassed() < timeout)
//		{
//			visitDecisionNode(n0); 
//		}
//		return greedyAction(n0); 
//	}
//	
//	void visitDecisionNode(THTSNode nd)
//	{
//		if (nd.firstVisit())
//		{
//			initializeNode(nd); 
//		}
//		THTSNode[] chanceNodes = selectAction(nd); 
//		for (THTSNode nc : chanceNodes)
//		{
//			visitChanceNode(nc); 
//		}
//		backupDecisionNode(nd); 
//	}
//	void visitChanceNode(THTSNode nc)
//	{
//		THTSNode[] decisionNodes = selectOutcome(nc); 
//		for(THTSNode nd: decisionNodes )
//		{
//			visitDecisionNode(nd); 
//		}
//		backupDecisionNode(nc);
//	}
//}
