package demos;

import java.util.concurrent.TimeUnit;

import cern.colt.Arrays;

public class CompareSTAPUSSINVI
{
	public double[] doSTAPU(String fn, int numRobots, int numFS, int numGoals, int numDoors)
	{
		STAPU stapu = new STAPU();
		double[] res = stapu.runGUISimpleTestsOne(fn, numRobots, numFS, numGoals, numDoors);
		//		System.out.println(res.toString());
		return res;
	}

	public double[] doSSI(String fn, int numRobots, int numDoors)
	{
		double[] res = new SSIAuctionNestedProduct().run(fn, numRobots, numDoors);
		//		System.out.println(res.toString());
		return res;
	}

	public void run()
	{
		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";
		numRobots = 2;
		numFS = 2;//5;//1;
		numGoals = 3;//6;//4;
		numDoors = 0;//2;

		fn = "g4_r2_t3_d0_fs2";
		long startTime = System.nanoTime();
		double[] ssiRes = doSSI(fn, numRobots, numDoors);
		long endTime = System.nanoTime();

		long durationSSI = (endTime - startTime);
		startTime = System.nanoTime();
		double[] stapuRes = doSTAPU(fn, numRobots, numFS, numGoals, numDoors);
		endTime = System.nanoTime();
		long durationStapu = (endTime - startTime);
		System.out.println("SSI Res: " + Arrays.toString(ssiRes));
		System.out.println("STAPU Res: " + Arrays.toString(stapuRes));
		System.out.println("SSI Res: " + durationSSI + " " + TimeUnit.MILLISECONDS.convert(durationSSI, TimeUnit.NANOSECONDS) + " ms");
		System.out.println("STAPU Res: " + durationStapu + " " + TimeUnit.MILLISECONDS.convert(durationStapu, TimeUnit.NANOSECONDS) + " ms");
	}

	public static void main(String[] args)
	{
		new CompareSTAPUSSINVI().run();

	}
}
