package demos;


import java.util.Comparator;

import demos.StateInformation.ValueLabel;

public class StateInformationRelativeCostComparator implements Comparator<StateInformation>
{
	public int compare(StateInformation s1, StateInformation s2)
	{
		ValueLabel vl = ValueLabel.cost;
		Object a1 = s1.getChosenAction(); 
		Object a2 = s2.getChosenAction(); 
		if(s1 != null) 
		{
			if(s2 !=null)
			{
				if(s1.actionValuesDifference!=null)
				{
					if(s2.actionValuesDifference!=null)
					{
						if(s1.actionValuesDifference.containsKey(a1))
						{
							if(s2.actionValuesDifference.containsKey(a2))
							{
								if(s1.actionValuesDifference.get(a1).containsKey(vl))
								{
									if(s2.actionValuesDifference.get(a2).containsKey(vl))
									{
										return (s1.actionValuesDifference.get(a1).get(vl)).compareTo(s2.actionValuesDifference.get(a2).get(vl));
									}
									
								}
							}
						}
					}
				}
			}
		}
		return -1;
	}
}
