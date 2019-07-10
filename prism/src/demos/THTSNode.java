package demos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import parser.State;
import prism.PrismException;

public abstract class THTSNode
{

	State s;
	Bounds probValues;
	Bounds progValues;
	HashMap<Integer, Bounds> rewsValues;
	List<THTSNode> parents;
	int numVisits;
	boolean solved;

	public abstract boolean equals(THTSNode n);

	public void addParent(THTSNode n)
	{
		if (parents == null)
			parents = new ArrayList<THTSNode>();
		if (!parents.contains(n))
			parents.add(n);
	}


	public boolean isSolved()
	{
		return solved;
	}

	public void setSolved()
	{
		solved = true;
	}

	public void setUnsolved()
	{
		solved = false;
	}

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
		if (rewsValues == null)
			setRews();
		rewsValues.put(rewNum, b);
	}

	public void initRews()
	{
		rewsValues = new HashMap<Integer, Bounds>();
	}
	public int getMaxRews()
	{
		if (rewsValues == null)
			return 0; 
		else 
			return rewsValues.size();
	}

//	public boolean isRoot()
//	{
//		return this.parents == null;
//	}

	public Bounds getObjectiveBounds(Objectives obj) throws PrismException
	{
		Bounds toret = null;
		switch (obj) {
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

	public abstract String getShortName();

	@Override
	public String toString()
	{
		String str = "N[s=" + s + ", p=" + probValues + ", pr=" + progValues + ", r=" + rewsValues + ", n=" + numVisits + ", solved=" + solved;
		if (parents == null || parents.size() == 0) {
			str += ", abus=noone";
		} else {
			str += ", abus=";
			for (THTSNode abu : parents) {
				if (abu != null) {
					str += abu.getState() + " ";
				}
			}
		}
		str += "]";

		return str;
	}

}