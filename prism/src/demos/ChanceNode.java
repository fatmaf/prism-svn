package demos;

import java.util.ArrayList;

import parser.State;

public class ChanceNode extends THTSNode
{
	Object action;
	ArrayList<DecisionNode> children;
	
	public ChanceNode(THTSNode parent, State s, Object a,
			Bounds prob, Bounds prog, ArrayList<Bounds> cost)
	{
		this.setState(s);
		this.setProbValue(prob);
		this.setProg(prog);
		for (int i = 0; i < cost.size(); i++)
			setRew(cost.get(i), i);
		this.parent = parent;
		action = a; 
		children = null; 
		solved = false; 

	}
	public ChanceNode(THTSNode parent, State s, Object a)
	{
		this.setState(s);
		this.parent = parent;
		action = a; 
		children = null; 
		solved = false; 

	}
	public void updateBounds(Bounds prob, Bounds prog, ArrayList<Bounds> cost)
	{
		this.setProbValue(prob);
		this.setProg(prog);
		for (int i = 0; i < cost.size(); i++)
			setRew(cost.get(i), i);
	}
	public void setChildren(ArrayList<DecisionNode> children)
	{
		this.children = children;
	}
	public ArrayList<DecisionNode> getChildren()
	{
		return this.children;
	}
	
	public Object getAction()
	{
		// TODO Auto-generated method stub
		return action;
	}

	@Override
	public THTSNodeType nodeType()
	{
		return THTSNodeType.Chance;
	}

	@Override
	public boolean isLeafNode()
	{
		// TODO Auto-generated method stub
		return children == null;
	}
	

}