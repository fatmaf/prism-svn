package demos;

import java.util.concurrent.TimeUnit;

import cern.colt.Arrays;

public class CompareSTAPUSSINVI
{
	public double[] doSTAPU()
	{
		STAPU stapu = new STAPU();
		double[] res = stapu.runGUISimpleTestsOne();
		//		System.out.println(res.toString());
		return res;
	}

	public double[] doSSI()
	{
		double[] res = new SSIAuctionNestedProduct().run();
		//		System.out.println(res.toString());
		return res;
	}

	public void run()
	{
		long startTime = System.nanoTime();
		double[] ssiRes = doSSI();
		long endTime = System.nanoTime();

		long durationSSI = (endTime - startTime);
		startTime = System.nanoTime();
		double[] stapuRes = doSTAPU();
		endTime = System.nanoTime();
		long durationStapu = (endTime - startTime);
		System.out.println(Arrays.toString(ssiRes));
		System.out.println(Arrays.toString(stapuRes));
		System.out.println(durationSSI+" "+TimeUnit.MILLISECONDS.convert(durationSSI, TimeUnit.NANOSECONDS));
		System.out.println(durationStapu+" "+TimeUnit.MILLISECONDS.convert(durationStapu, TimeUnit.NANOSECONDS));
	}

	public static void main(String[] args)
	{
		new CompareSTAPUSSINVI().run();

	}
}
