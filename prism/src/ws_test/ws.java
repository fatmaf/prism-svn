package ws_test;

import java.util.ArrayList;
import java.util.HashMap;

public class ws {

	public int cost(String letter1, String letter2,boolean max)
	{
		int costtoret = 0; 
		if(letter1 != letter2)
		{
			HashMap<String,Integer> costs = new HashMap<String,Integer>(); 
			costs.put("carpet",1);
			costs.put("slippers",10);
			costs.put("space",0); 
			costs.put("init",0);
			if (max) 
			{
				costtoret = Math.max(costs.get(letter1),costs.get(letter2));
			}
			else
			{
				costtoret =costs.get(letter1)+costs.get(letter2);
			}
		}

		return costtoret;
	}
	public double dist(String[] robot_word, String[] logical_word,boolean max)
	{
			double disttoret = 0.0;
			if(robot_word.length == logical_word.length)
			{
				for(int i = 0; i<robot_word.length; i++)
				{
					disttoret= disttoret+ cost(robot_word[i],logical_word[i],max);
				}
			}
			else 
				disttoret=Double.POSITIVE_INFINITY;
			
					
			return disttoret;
	}
	
	public double distToSat(String[] robot_word, ArrayList<String[]> logical_words, boolean max)	
	{
		double currCost = 0.0;
		double minDist = Double.POSITIVE_INFINITY; 
		for(int i = 0; i< logical_words.size(); i++)
		{
			currCost = dist(robot_word,logical_words.get(i),max); 
			if(currCost<minDist)
			{
				minDist = currCost;
			}
		}
		//the robot word plus the list of all the logical worlds
		return minDist;	
	}

	public void test() {
		String[] labels ={"carpet","slippers","space","init"};

		for (int i = 0; i<labels.length; i++)
		{
			for(int j = 0; j<labels.length; j++){
				System.out.println(labels[i]+"-"+labels[j]+"(Sum: "+cost(labels[j],labels[i],false)+",Max: "+cost(labels[i],labels[j],true)+")");

			}
		}

	}

	public static void main(String[] args) {
		ws obj = new ws(); 
		obj.test();

	}

} 
