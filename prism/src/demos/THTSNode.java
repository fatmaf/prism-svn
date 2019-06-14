package demos;

import java.util.ArrayList;
import java.util.HashMap;

import parser.State;

public abstract class THTSNode
{

	State s;
	Bounds prob;
	Bounds prog;
	HashMap<Integer, Bounds> rews;

	public State getState()
	{
		return s;
	}

	public void setState(State s)
	{
		this.s = s;
	}

	public Bounds getProb()
	{
		return prob;
	}

	public void setProb(Bounds prob)
	{
		this.prob = prob;
	}

	public Bounds getProg()
	{
		return prog;
	}

	public void setProg(Bounds prog)
	{
		this.prog = prog;
	}

	public HashMap<Integer, Bounds> getRews()
	{
		return rews;
	}

	public void setRews(HashMap<Integer, Bounds> rews)
	{
		this.rews = rews;
	}

	public Bounds getRew(int rewNum)
	{
		return rews.get(rewNum);
	}

	public void setRew(Bounds b, int rewNum)
	{
		rews.put(rewNum, b);
	}

}