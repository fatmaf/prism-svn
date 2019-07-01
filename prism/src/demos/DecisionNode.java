package demos;

import java.util.ArrayList;
import java.util.HashMap;

import parser.State;
import prism.PrismException;

public class DecisionNode extends THTSNode
{
	HashMap<Object, ChanceNode> children;
	HashMap<THTSNode, Double> transitionProbability;
	int numActions;
	boolean isDeadend;
	boolean isGoal;

	public DecisionNode(THTSNode parent, State s, double tprob, Bounds prob, Bounds prog, ArrayList<Bounds> cost, boolean deadend, boolean goal)
	{
		this.setState(s);
		this.setProbValue(prob);
		this.setProg(prog);
		for (int i = 0; i < cost.size(); i++)
			setRew(cost.get(i), i);
		if (parent != null) {
			addParent(parent);
			if (transitionProbability == null) {
				transitionProbability = new HashMap<THTSNode, Double>();

			}

			transitionProbability.put(parent, tprob);
		}
		children = null;
		numActions = 0;
		solved = false;
		isDeadend = deadend;
		isGoal = goal;

		if (deadend | goal) {

			solved = true;
		}
		numVisits = 0;

	}

	public void addParent(THTSNode n, double tprob)
	{
		addParent(n);
		addTranProb(n, tprob);

	}

	public Bounds getProbValueTimesTranProb(THTSNode p)
	{
		if (p != null)
			return this.probValues.multiply(transitionProbability.get(p));
		else
			return this.probValues.multiply(1.0);
	}

	public Bounds getProgValueTimesTranProb(THTSNode p)
	{
		if (p != null)
			return this.progValues.multiply(transitionProbability.get(p));
		else
			return this.progValues.multiply(1.0);
	}

	public Bounds getRewValueTimesTranProb(int i, THTSNode p)
	{
		if (p != null)
			return getRew(i).multiply(transitionProbability.get(p));
		else
			return getRew(i).multiply(1.0);
	}

	public void addChild(Object a, ChanceNode child) throws PrismException
	{

		if (isLeafNode())
			children = new HashMap<Object, ChanceNode>();

		//check if this action exists in the children 
		if (!children.containsKey(a)) {
			children.put(a, child);
		}

	}

	ChanceNode getChild(Object a)
	{
		if (isLeafNode())
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

	public double getTranProb(THTSNode p)
	{
		return transitionProbability.get(p);
	}

	void addTranProb(THTSNode p, double tprob)
	{
		if (transitionProbability == null)
			transitionProbability = new HashMap<THTSNode, Double>();
		transitionProbability.put(p, tprob);
	}

	public boolean allActionsAdded()
	{
		return children.size() == numActions;
	}

	@Override
	public boolean equals(THTSNode n)
	{
		boolean equal = false;
		if (n instanceof DecisionNode) {

			//sate action parent 
			if (this.getState().compareTo(n.getState()) == 0) {

				equal = true;

			}
		}
		return equal;
	}

	@Override
	public String getShortName()
	{
		return this.getState().toString();
	}
}
