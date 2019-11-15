package demos;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.AbstractMap;
import java.util.AbstractMap.SimpleEntry;
import parser.State;

public class XAIPolicyPath
{

	public class PathNode
	{
		//each path has a state and action
		//parent state 
		//list of successors with probabilities
		//we have a value for the state action 
		//we can also have values for other less optimal actions 
		XAIStateInformation root;
		ArrayList<XAIStateInformation> parent;
		HashMap<PathNode, Double> children;

		PathNode(XAIStateInformation s)
		{
			root = s;
			parent = new ArrayList<XAIStateInformation>();
			children = null;
		}

		PathNode(XAIStateInformation s, XAIStateInformation p)
		{
			root = s;
			parent = new ArrayList<XAIStateInformation>();
			parent.add(p);
			children = null;
		}

		void addChild(PathNode s, double prob)
		{
			if (children == null)
				children = new HashMap<PathNode, Double>();
			s.parent.add(this.root);
			children.put(s, prob);
		}
	}

	PathNode root;
	HashMap<String, PathNode> statesList;

	XAIPolicyPath()
	{

		root = null;

		statesList = new HashMap<String, PathNode>();
	}

	XAIPolicyPath(XAIStateInformation s)
	{
		statesList = new HashMap<String, PathNode>();
		this.setRoot(s);

	}

	public String createStatesListString(State s, Object a)
	{
		return s.toString(); //+ a.toString();
	}

	public PathNode addToStatesList(XAIStateInformation s)
	{
		PathNode sfound = null;
		State ss = s.getState();
		String sa = createStatesListString(ss, s.getChosenAction());
		if (!statesList.containsKey(sa)) {
			statesList.put(sa, new PathNode(s));
			sfound = statesList.get(sa);
		} else {
			sfound = statesList.get(sa);
			if (s.getChosenAction() != null) {
				if (sfound.root.getChosenAction() == null) {
					sfound.root.setChosenAction(s.getChosenAction());
				//get the path values of null and put them in this one 
				sfound.root.actionValues.put(s.getChosenAction(), s.actionValues.get(s.getChosenAction()));}
			}
			else
			{
			if(s.getChosenAction() == null && sfound.root.getChosenAction() == null)
			{
				sfound.root.actionValues.put(s.getChosenAction(), s.actionValues.get(s.getChosenAction()));	
			}
			}
			
		}
		return sfound;
	}

	public PathNode getState(State s, Object a)
	{
		String sa = createStatesListString(s, a);

		return statesList.get(sa);

	}

	public void setRoot(XAIStateInformation s)
	{
		root = addToStatesList(s);

	}

	public PathNode getNode(XAIStateInformation s)
	{
		return addToStatesList(s);
	}

	public void addChild(XAIStateInformation p, XAIStateInformation c, double prob)
	{
		PathNode parent = addToStatesList(p);
		parent.addChild(addToStatesList(c), prob);
	}

}
