package ws_test;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;
import java.util.Vector;

import prism.PrismLog;

public class wlabel {
	//public BitSet log; 
	public List<BitSet> phy;

	public wlabel(int apsize)
	{
		phy = new Vector<BitSet>();
		//log = (BitSet) act.clone(); 
		int size = apsize;//log.length();
		int powersetsize = (int)Math.pow(2.0, (double)size);
		for(int i = 0; i<powersetsize; i++)
		{
			BitSet tmp = BitSet.valueOf(new long[]{i});
			phy.add(tmp);
		}
		
	
	}
	
	
	public void print(PrismLog logp, BitSet log)
	{
		for(int i = 0; i<phy.size(); i++)
		{
			logp.println(log.toString()+ "," + phy.get(i).toString());
		}
	}
	
	public void print(PrintStream out, BitSet log)
	{
		for(int i = 0; i<phy.size(); i++)
		{
			out.println(log.toString()+ "," + phy.get(i).toString());
		}
	}
}
