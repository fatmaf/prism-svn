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
	public XAIMdpOption(String name)
	{
		stateActionList = new HashMap<State,Object>(); 
		this.name = name;
		actions = new ArrayList<Object>();
		terminationStates = new ArrayList<State>();
		labels = new HashSet<BitSet>();
	}
	
	

}
