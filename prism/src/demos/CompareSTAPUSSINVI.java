package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import cern.colt.Arrays;

public class CompareSTAPUSSINVI
{
	int rInd = 0;
	int gInd = 1;
	int dInd = 2;
	int fInd = 3;
	int ssiInd = 0;
	int stapuInd = 1;
	int probInd = 0;
	int taskInd = 1;
	int costInd = 2;
	int timeInd = 3;

	HashMap<String, HashMap<int[], float[][]>> results = new HashMap<String, HashMap<int[], float[][]>>();

	public void printResults(String contest)
	{
		File file = new File(contest + ".txt");
		FileOutputStream fileOutputStream = null;
		PrintStream outLog = null;
		try {
			fileOutputStream = new FileOutputStream(file);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (fileOutputStream != null)
			outLog = new PrintStream(fileOutputStream);
		else
			outLog = System.out;

		String header = "fn\tr\tg\td\tf\tl\tp\tt\tc\tt(s)\n";
		outLog.print(header);
		for (String fn : results.keySet()) {
			//String row = fn;
			HashMap<int[], float[][]> datahere = results.get(fn);
			for (int[] rgdf : datahere.keySet()) {
				float[][] valuesall = datahere.get(rgdf);

				int resInd = ssiInd;
				float[] values = valuesall[resInd];
				String row = fn + "\t" + rgdf[rInd] + "\t" + rgdf[gInd] + "\t" + rgdf[dInd] + "\t" + rgdf[fInd] + "\tssi\t" + values[probInd] + "\t"
						+ values[taskInd] + "\t" + values[costInd] + "\t" + values[timeInd];
				outLog.println(row);
				resInd = stapuInd;
				values = valuesall[stapuInd];
				row = fn + "\t" + rgdf[rInd] + "\t" + rgdf[gInd] + "\t" + rgdf[dInd] + "\t" + rgdf[fInd] + "\tstapu\t" + values[probInd] + "\t"
						+ values[taskInd] + "\t" + values[costInd] + "\t" + values[timeInd];
				outLog.println(row);

			}
		}

	}

	public void printResults()
	{

		PrintStream outLog = System.out;

		String header = "fn\tr\tg\td\tf\tl\tp\tt\tc\tt(s)\n";
		outLog.print(header);
		for (String fn : results.keySet()) {
			//String row = fn;
			HashMap<int[], float[][]> datahere = results.get(fn);
			for (int[] rgdf : datahere.keySet()) {
				float[][] valuesall = datahere.get(rgdf);

				int resInd = ssiInd;
				float[] values = valuesall[resInd];
				String row = fn + "\t" + rgdf[rInd] + "\t" + rgdf[gInd] + "\t" + rgdf[dInd] + "\t" + rgdf[fInd] + "\tssi\t" + values[probInd] + "\t"
						+ values[taskInd] + "\t" + values[costInd] + "\t" + values[timeInd];
				outLog.println(row);
				resInd = stapuInd;
				values = valuesall[stapuInd];
				row = fn + "\t" + rgdf[rInd] + "\t" + rgdf[gInd] + "\t" + rgdf[dInd] + "\t" + rgdf[fInd] + "\tstapu\t" + values[probInd] + "\t"
						+ values[taskInd] + "\t" + values[costInd] + "\t" + values[timeInd];
				outLog.println(row);

			}
		}

	}

	public double[] doSTAPU(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors)
	{
		STAPU stapu = new STAPU();
		double[] res = stapu.runGUISimpleTestsOne(dir, fn, numRobots, numFS, numGoals, numDoors);
		//		System.out.println(res.toString());
		return res;
	}

	public double[] doSSI(String dir, String fn, int numRobots, int numGoals, int numDoors)
	{
		double[] res = new SSIAuctionNestedProduct().run(dir, fn, numRobots, numGoals, numDoors);
		//		System.out.println(res.toString());
		return res;
	}

	public String doCompare(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors, float[][] resArr)
	{
		String resString = "\n" + fn;
		long startTime = System.nanoTime();
		double[] ssiRes = doSSI(dir, fn, numRobots, numGoals, numDoors);
		long endTime = System.nanoTime();

		long durationSSI = (endTime - startTime);
		startTime = System.nanoTime();
		resArr[ssiInd] = new float[4];
		for (int i = 0; i < 3; i++)
			resArr[ssiInd][i] = (float) ssiRes[i];
		resArr[ssiInd][3] = durationSSI;

		double[] stapuRes = doSTAPU(dir, fn, numRobots, numFS, numGoals, numDoors);
		endTime = System.nanoTime();
		long durationStapu = (endTime - startTime);
		resArr[stapuInd] = new float[4];
		for (int i = 0; i < 3; i++)
			resArr[stapuInd][i] = (float) stapuRes[i];
		resArr[stapuInd][3] = durationStapu;

		resString += "\nSSI Res: " + Arrays.toString(ssiRes);

		resString += "\nSTAPU Res: " + Arrays.toString(stapuRes);
		resString += "\nSSI Res: " + durationSSI + " " + TimeUnit.MILLISECONDS.convert(durationSSI, TimeUnit.NANOSECONDS) + " ms";
		resString += "\nSTAPU Res: " + durationStapu + " " + TimeUnit.MILLISECONDS.convert(durationStapu, TimeUnit.NANOSECONDS) + " ms";
		return resString;

	}

	public void runDoors()
	{
		//	String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSI/";
		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";

		String resString = "";
		try {
			numRobots = 2;
			numFS = 0;//5;//1;
			numGoals = 3;//6;//4;
			numDoors = 2;//2;
			fn = "andar_r2_g3_d2_fs0";

			if (!results.containsKey(fn))
				results.put(fn, new HashMap<int[], float[][]>());

			for (int r = 2; r <= numRobots; r++) {
				for (int g = 2; g <= numGoals; g += 2) {
					int[] rgdf = new int[] { r, g, numDoors, numFS };
					float[][] resArr = new float[2][4];
					resString += "\nR:" + r + "\tG:" + g;

					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr);
					results.get(fn).put(rgdf, resArr);

				}
			}

			numRobots = 5;
			numFS = 0;//5;//1;
			numGoals = 5;//6;//4;
			numDoors = 3;//2;
			fn = "andar_r5_g5_d3_fs0";

			if (!results.containsKey(fn))
				results.put(fn, new HashMap<int[], float[][]>());

			for (int r = 2; r <= numRobots; r++) {
				for (int g = 2; g <= numGoals; g += 2) {
					int[] rgdf = new int[] { r, g, numDoors, numFS };
					float[][] resArr = new float[2][4];
					resString += "\nR:" + r + "\tG:" + g;

					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr);
					results.get(fn).put(rgdf, resArr);

				}
			}

			numRobots = 8;
			numFS = 0;//5;//1;
			numGoals = 10;//6;//4;
			numDoors = 4;//2;
			fn = "andar_r8_g10_d4_fs0";

			if (!results.containsKey(fn))
				results.put(fn, new HashMap<int[], float[][]>());

			for (int r = 2; r <= numRobots; r++) {
				for (int g = 2; g <= numGoals; g += 2) {
					int[] rgdf = new int[] { r, g, numDoors, numFS };
					float[][] resArr = new float[2][4];
					resString += "\nR:" + r + "\tG:" + g;

					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr);
					results.get(fn).put(rgdf, resArr);

				}
			}
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");
			String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
			String resname = "doors";
			this.printResults(resSavePlace + resname);
			printResults();
		}
	}

	public void runRobots()
	{
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		//		String dir= "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSI/";
		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";

		String resString = "";
		//		//				doCompare(dir,fn,numRobots,numFS,numGoals,numDoors);
		//		//	
		//		//		numRobots = 2;
		//		//		numFS = 2;//5;//1;
		//		//		numGoals = 3;//6;//4;
		//		//		numDoors = 0;//2;
		//		//
		//		//		fn = "g4_r2_t3_d0_fs2";
		//		//		 
		//		//		resString+=doCompare(dir,fn,numRobots,numFS,numGoals,numDoors);
		//		//		
		//		////		numRobots = 2;
		//		////		numFS = 2;//5;//1;
		//		////		numGoals = 5;//6;//4;
		//		////		numDoors = 2;//2;
		//		////
		//		////		fn = "g5x10_r2_t5_d2_fs2";
		//		////		 
		//		////		resString+=doCompare(dir,fn,numRobots,numFS,numGoals,numDoors);
		//		////		
		//		////		numRobots = 2;
		//		////		numFS = 2;//5;//1;
		//		////		numGoals = 3;//6;//4;
		//		////		numDoors = 2;//2;
		//		////
		//		////		fn = "g5x10_r2_t5_d2_fs2";
		//		////		resString+="\nGoals=3";
		//		////		resString+=doCompare(dir,fn,numRobots,numFS,numGoals,numDoors);
		//		////		
		//		//		numRobots = 5;
		//		//		numFS = 0;//5;//1;
		//		//		numGoals = 6;//6;//4;
		//		//		numDoors = 0;//2;
		//		//		fn = "g9x5_r5_t5_d0_fs0";
		//		//
		//		//		for(int r = 2; r<=numRobots; r++)
		//		//		{
		//		//			for(int g = 2; g<=numGoals; g+=2) {
		//		//		resString+="\nR:"+r+"\tG:"+g;
		//		//		 
		//		//		resString+=doCompare(dir,fn,r,numFS,g,numDoors);
		//		//		}
		//		//		}
		numRobots = 10;
		numFS = 0;//5;//1;
		numGoals = 11;//6;//4;
		numDoors = 0;//2;
		fn = "g19x5_r10_t11_d0_fs0";

		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], float[][]>());

		for (int r = 2; r <= numRobots; r++) {
			for (int g = 3; g <= numGoals; g += 2) {
				int[] rgdf = new int[] { r, g, numDoors, numFS };
				float[][] resArr = new float[2][4];
				resString += "\nR:" + r + "\tG:" + g;

				resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr);
				results.get(fn).put(rgdf, resArr);

			}
		}
		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");
		String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
		String resname = "robots2";
		this.printResults(resSavePlace + resname);
	}

	public void run()
	{
		runRobots();
	}

	public static void main(String[] args)
	{
		new CompareSTAPUSSINVI().run();

	}
}
