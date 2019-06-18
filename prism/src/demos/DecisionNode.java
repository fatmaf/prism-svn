package demos;

import java.util.ArrayList;
import java.util.HashMap;

import parser.State;
import prism.PrismException;

public class DecisionNode extends THTSNode
{
	HashMap<Object, ChanceNode> children;
	double transitionProbability;
	int numActions; 
	
	public DecisionNode(THTSNode parent, State s, double tprob,
			Bounds prob, Bounds prog, ArrayList<Bounds> cost)
	{
		this.setState(s);
		this.setProbValue(prob);
		this.setProg(prog);
		for (int i = 0; i < cost.size(); i++)
			setRew(cost.get(i), i);
		this.parent = parent;
		transitionProbability = tprob; 
		children = null; 
		numActions = 0; 
		solved = false; 

	}


	public Bounds getProbValueTimesTranProb()
	{
		return this.probValues.multiply(transitionProbability);
	}
	public Bounds getProgValueTimesTranProb()
	{
		return this.progValues.multiply(transitionProbability);
	}
	public Bounds getRewValueTimesTranProb(int i)
	{
		return getRew(i).multiply(transitionProbability);
	}
	public void addChild(Object a, ChanceNode child) throws PrismException
	{

		if (isLeafNode())
			children = new HashMap<Object,ChanceNode>();
			
		//check if this action exists in the children 
		if (!children.containsKey(a)) {
			children.put(a, child);
		}
		

	}



	ChanceNode getChild(Object a)
	{
		if(isLeafNode())
			return null;
		if (!children.containsKey(a))
			return null;
		return children.get(a);
	}
	HashMap<Object, ChanceNode> getChildren()
	{
		return children;
	}

	public boolean isLeafNode()
	{
		return children == null;
	}

	@Override
	public THTSNodeType nodeType()
	{
		return THTSNodeType.Decision;
	}
	public double getTranProb()
	{
		return transitionProbability;
	}
	public boolean allActionsAdded()
	{
		return children.size() == numActions; 
	}

}
