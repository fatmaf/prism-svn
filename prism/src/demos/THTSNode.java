package demos;

import java.util.ArrayList;
import java.util.HashMap;

import parser.State;
import prism.PrismException;

public abstract class THTSNode
{

	State s;
	Bounds probValues;
	Bounds progValues;
	HashMap<Integer, Bounds> rewsValues;
	THTSNode parent; 
	int numVisits = 0; 
	boolean solved; 

	public int visited()
	{
		return numVisits; 
	}
	public int increaseVisits()
	{
		numVisits++; 
		return numVisits; 
	}
	public State getState()
	{
		return s;
	}

	public void setState(State s)
	{
		this.s = s;
	}

	public Bounds getProbValue()
	{
		return probValues;
	}

	public void setProbValue(Bounds prob)
	{
		this.probValues = prob;
	}

	public Bounds getProg()
	{
		return progValues;
	}

	public void setProg(Bounds prog)
	{
		this.progValues = prog;
	}

	public HashMap<Integer, Bounds> getRews()
	{
		return rewsValues;
	}

	public void setRews(HashMap<Integer, Bounds> rews)
	{
		this.rewsValues = rews;
	}
	public void setRews()
	{
		initRews();
	}
	public Bounds getRew(int rewNum)
	{
		return rewsValues.get(rewNum);
	}

	public void setRew(Bounds b, int rewNum)
	{
		if(rewsValues == null)
			setRews();
		rewsValues.put(rewNum, b);
	}
	public void initRews()
	{
		rewsValues = new HashMap<Integer,Bounds>();
	}
	public boolean isRoot()
	{
		return this.parent == null;
	}
	public Bounds getObjectiveBounds(Objectives obj) throws PrismException
	{
		Bounds toret = null; 
		switch (obj)
		{
		case Probability:
			toret = getProbValue(); 
			break; 
		case Progression:
			toret = getProg(); 
			break; 
		case Cost:
			toret = getRew(0);
			break; 
		default:
			throw new PrismException("kya?"); 
			
			
		}
		return toret; 
	}
	public abstract THTSNodeType nodeType();
	public abstract boolean isLeafNode();

}