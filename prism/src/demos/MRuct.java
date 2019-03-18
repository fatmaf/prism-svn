package demos;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import parser.State;
import prism.PrismException;
import prism.PrismLog;
import prism.ProductModelGenerator;

/**
 *  UCT for Multiple Robots 
 *  Following Bruno's code 
 *  and Planning with MDPs pg 112
 **/
public class MRuct {
	
	protected double termCritParam = 1e-8;
	public static final int UNK_STATE = -1;
	public static final int SINK_STATE = 0;
	public static final int ACC_STATE = 1;
	
	private PrismLog mainLog; 
	private ArrayList<ProductModelGenerator> prodModGens;
	private int  rollout_depth;  
	private int max_num_rollouts;
	private int current_rollout_num; 
	private int num_robots; 
	private List<Double> da_distToAcc;
	
	private String da_id_string = "_da0"; 
	private ArrayList<String> shared_state_names; 
	
	Random randomGen; 
	
	public MRuct(PrismLog log,ArrayList<ProductModelGenerator> robotProdModelGens, int max_rollouts, int rollout_depth, ArrayList<String> shared_state_names, List<Double> dadistToAcc)
	
	{
		num_robots = robotProdModelGens.size();
		prodModGens = robotProdModelGens; 
		this.rollout_depth = rollout_depth; 
		this.max_num_rollouts = max_rollouts; 
		mainLog = log; 
		current_rollout_num = 0;
		if(shared_state_names != null) {
		if(shared_state_names.size()>0)
			this.shared_state_names = shared_state_names; 
		else
			this.shared_state_names = null;
		}
		da_distToAcc = dadistToAcc;
		
	}
	public void search()
	{
		current_rollout_num = 0;
		//get the initial state 
		
		State temp_joint_state = new State(num_robots+1); 
		
		//creating the initial state 
		for(int i = 0; i<num_robots; i++)
		{
			try {
				State current_model_state = prodModGens.get(i).getInitialState();
				//collect non da values 
				//knowing that the da value is the last one 
				//TODO: shared state stuff 
				int da_state_index = prodModGens.get(i).getVarIndex(da_id_string); 
				
				temp_joint_state.setValue(i,current_model_state.substate(0, da_state_index));
				State da_state_in_joint_state = temp_joint_state.substate(num_robots, num_robots+1);
				if( da_state_in_joint_state.varValues[0] == null)
				{
					temp_joint_state.setValue(num_robots, current_model_state.substate(da_state_index, da_state_index+1)); 
				}
				else
				{
					//check if there's a change 
					State da_state_in_robot = current_model_state.substate(da_state_index, da_state_index+1); 
					if(!da_state_in_joint_state.equals(da_state_in_robot))
					{
						//if there are competing DA states 
						//lets take the one whose disttoacc is the smallest ?
						if(da_distToAcc.get((int)da_state_in_joint_state.varValues[0]) > da_distToAcc.get((int)da_state_in_robot.varValues[0]))
						{
							temp_joint_state.setValue(num_robots, da_state_in_robot); 
						}
								
						throw new PrismException("Robots are competing for a DA state. Not implemented"); 
					}
				}
				
				
			} catch (PrismException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} 
			
		}
		mainLog.println("Joint State:"+temp_joint_state.toString());
		
		
		
		while(current_rollout_num < max_num_rollouts)
		{
			
		}
	}
	
	
	
	
	
	

}
