package demos;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import parser.State;

public class XAIMdpOption {
	//an option has a policy 
	// termination condition 
	//this is interesting 
	//so if B(s) =1 we cant take the option 
	//but if B(s) < 1 we can take the option 
	// so basically an option will have like actions for all states 
	//and you can start anywhere - even in the middle or something 
	//i'll have to see how true this is 
	//but it makes sense 
	
	//initiation set 
	//this is just a set of states so a list for us ?
	
	public HashMap<State,Object> stateActionList; 
	public String name; 
	public Set<BitSet> labels; 
	public ArrayList<Object> actions; 
	public ArrayList<State> terminationStates; 
	public boolean leadsToSinkState = false; 
	public String satisfiesString =""; 
	public String doesnotsatisfyString=""; 
	public String violatesString=""; 
	public XAIMdpOption(String name)
	{
		stateActionList = new HashMap<State,Object>(); 
		this.name = name;
		actions = new ArrayList<Object>();
		terminationStates = new ArrayList<State>();
		labels = new HashSet<BitSet>();
	}
	public boolean combine(XAIMdpOption o2)
	{
		boolean combined = false; 
		//the actions and labels are the same 
//		if(actions.containsAll(o2.actions) && o2.actions.containsAll(actions))
//		{
//			if(labels.containsAll(o2.labels) && o2.labels.containsAll(labels))
//			{
				//now just combine the termination states and stateActionlist 
				stateActionList.putAll(o2.stateActionList);
				terminationStates.addAll(o2.terminationStates);
				actions.addAll(o2.actions);
				labels.addAll(o2.labels);
				combined = true; 
//			}
//		}
		return combined;
	}
	@Override
	public String toString() {
		return "XAIMdpOption [name=" + name + ", labels=" + labels
				+ ", actions=" + actions + "]";
	}
	

}
