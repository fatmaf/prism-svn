package demos;

import java.util.ArrayList;

import parser.State;
import prism.PrismException;

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
		addParent(parent);
		action = a; 
		children = null; 
		solved = false; 
		numVisits = 0;

	}
	public ChanceNode(THTSNode parent, State s, Object a)
	{
		this.setState(s);
		addParent(parent);
		action = a; 
		children = null; 
		solved = false; 

	}
	public double sumChildrenTProb() throws PrismException
	{
		double prob = 0; 
		if (children != null)
		{
			for(int i = 0; i< children.size(); i++)
			{
				double cprob = children.get(i).getTranProb(this); 
				prob = prob + cprob; 
			}
		}
		if(prob > 1)
			throw new PrismException("Children have incorrenct probabilities");
		return prob; 
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
	@Override
	public boolean equals(THTSNode n)
	{
		boolean equal = false; 
		if(n instanceof  ChanceNode)
		{
			
			//sate action parent 
			if(this.getState().compareTo(n.getState())==0)
			{
				if(this.getAction() == ((ChanceNode)n).getAction())
				{
					equal = true;
				}
			}
		}
		return equal;
	}
	
	@Override
	public String toString()
	{
		String str= "CN[s=" + s + ", p=" + probValues + ", pr=" + progValues + ", r=" + rewsValues + 
				", n=" + numVisits + ", solved=" + solved ;
		if (parents == null || parents.size() == 0) {
			str += ", abus=noone";
		} else {
			str += ", abus=";
			for (THTSNode abu : parents) {
				str += abu.getState() + " ";
			}
		}
			str+="a="+action;
			str+="]";
		
		return str; 
	}
	@Override
	public String getShortName()
	{
		return this.getState().toString()+this.getAction().toString();
	}
}