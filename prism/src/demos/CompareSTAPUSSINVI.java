package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.concurrent.TimeUnit;
import java.util.stream.IntStream;

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

	HashMap<String, HashMap<int[], ArrayList<float[][]>>> results = new HashMap<String, HashMap<int[], ArrayList<float[][]>>>();

	public void printResults(String contest)
	{
		PrintStream outLog = System.out;
		File file = null;
		FileOutputStream fileOutputStream = null;
		if (contest != null) {
			file = new File(contest + ".txt");

			try {
				fileOutputStream = new FileOutputStream(file);
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			if (fileOutputStream != null)
				outLog = new PrintStream(fileOutputStream);
		}

		String header = "fn\tr\tg\td\tf\tl\tp\tt\tc\tt(s)\n";
		outLog.print(header);
		for (String fn : results.keySet()) {
			//String row = fn;
			HashMap<int[], ArrayList<float[][]>> datahere = results.get(fn);
			for (int[] rgdf : datahere.keySet()) {
				ArrayList<float[][]> valueslist = datahere.get(rgdf);
				for (float[][] valuesall : valueslist) {
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

	}

	public void printResults()
	{

		printResults(null);

	}

	public double[] doSTAPU(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors, boolean noreallocs, ArrayList<Integer> robotNumbers,
			ArrayList<Integer> goalNumbers)
	{
		STAPU stapu = new STAPU();
		double[] res = stapu.runGUISimpleTestsOne(dir, fn, numRobots, numFS, numGoals, numDoors, noreallocs, robotNumbers, goalNumbers);
		//		System.out.println(res.toString());
		return res;
	}

	public double[] doSSI(String dir, String fn, int numRobots, int numGoals, int numDoors, ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers)
	{
		double[] res = new SSIAuctionNestedProduct().run(dir, fn, numRobots, numGoals, numDoors, robotNumbers, goalNumbers);
		//		System.out.println(res.toString());
		return res;
	}

	public String doCompare(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors, float[][] resArr, ArrayList<Integer> robotNumbers,
			ArrayList<Integer> goalNumbers)
	{

		String resString = "\n" + fn;
		long startTime = System.nanoTime();
		if (robotNumbers != null && goalNumbers != null)
			System.out.println("R:" + numRobots + "-" + robotNumbers.toString() + " G:" + numGoals + " " + goalNumbers.toString());
		double[] ssiRes = doSSI(dir, fn, numRobots, numGoals, numDoors, robotNumbers, goalNumbers);
		long endTime = System.nanoTime();

		long durationSSI = (endTime - startTime);
		startTime = System.nanoTime();
		resArr[ssiInd] = new float[4];
		for (int i = 0; i < 3; i++)
			resArr[ssiInd][i] = (float) ssiRes[i];
		resArr[ssiInd][3] = durationSSI;
		if (robotNumbers != null && goalNumbers != null)
			System.out.println("R:" + numRobots + "-" + robotNumbers.toString() + " G:" + numGoals + " " + goalNumbers.toString());
		double[] stapuRes = doSTAPU(dir, fn, numRobots, numFS, numGoals, numDoors, false, robotNumbers, goalNumbers);
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

		String allfn = "";
		String resString = "";
		try {
			numRobots = 2;
			numFS = 0;//5;//1;
			numGoals = 3;//6;//4;
			numDoors = 2;//2;
			fn = "andar_r2_g3_d2_fs0";

			allfn += fn;
			if (!results.containsKey(fn))
				results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

			for (int r = 2; r <= numRobots; r++) {
				for (int g = 3; g <= numGoals; g += 2) {
					int[] rgdf = new int[] { r, g, numDoors, numFS };
					ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
					ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

					if (!results.get(fn).containsKey(rgdf))
						results.get(fn).put(rgdf, new ArrayList<float[][]>());
					float[][] resArr = new float[2][4];
					resString += "\nR:" + r + "\tG:" + g;

					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers);
					results.get(fn).get(rgdf).add(resArr);

				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
				String resname = "doors2_" + numDoors + "_robots_" + r + "_" + fn;
				this.printResults(resSavePlace + resname);
			}

			numRobots = 5;
			numFS = 0;//5;//1;
			numGoals = 5;//6;//4;
			numDoors = 3;//2;
			fn = "andar_r5_g5_d3_fs0";

			allfn += fn;
			if (!results.containsKey(fn))
				results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

			for (int r = 2; r <= numRobots; r++) {
				for (int g = 3; g <= numGoals; g += 2) {
					//					for (int t = 0; t < 5; t++) {
					int[] rgdf = new int[] { r, g, numDoors, numFS };
					if (!results.get(fn).containsKey(rgdf))
						results.get(fn).put(rgdf, new ArrayList<float[][]>());
					ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
					ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

					float[][] resArr = new float[2][4];
					resString += "\nR:" + r + "\tG:" + g;

					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers);
					results.get(fn).get(rgdf).add(resArr);

				}
				//				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
				String resname = "doors2_" + numDoors + "_robots_" + r + "_" + fn;
				this.printResults(resSavePlace + resname);
			}

			numRobots = 8;
			numFS = 0;//5;//1;
			numGoals = 10;//6;//4;
			numDoors = 4;//2;
			fn = "andar_r8_g10_d4_fs0";

			allfn += fn;
			if (!results.containsKey(fn))
				results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

			for (int r = 2; r <= numRobots; r++) {
				for (int g = 3; g <= numGoals; g += 2) {
					//					for (int t = 0; t < 5; t++) {
					int[] rgdf = new int[] { r, g, numDoors, numFS };
					ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
					ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

					if (!results.get(fn).containsKey(rgdf))
						results.get(fn).put(rgdf, new ArrayList<float[][]>());
					float[][] resArr = new float[2][4];
					resString += "\nR:" + r + "\tG:" + g;

					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers);
					results.get(fn).get(rgdf).add(resArr);

				}
				//				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
				String resname = "doors2_" + numDoors + "_robots_" + r + "_" + fn;
				this.printResults(resSavePlace + resname);
			}
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");
			String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
			String resname = "doors2_" + allfn;
			this.printResults(resSavePlace + resname);
			printResults();
		}
	}

	public void runRobots() throws Exception
	{
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		//		String dir= "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSI/";
		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";

		String resString = "";

		numRobots = 10;
		numFS = 0;//5;//1;
		numGoals = 11;//6;//4;
		numDoors = 0;//2;
		fn = "g19x5_r10_t11_d0_fs0";

		int maxGoals = numGoals;
		int maxRobots = numRobots;
		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		try {
			for (int r = 2; r <= maxRobots; r++) {
				//lets do this multiple times 
				//like 5 times 
				for (int t = 0; t < 5; t++) {
					for (int g = 3; g <= maxGoals; g += 2) {
						//						if(r <= 5 && g <=7)
						//							continue;
						ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
						ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

						int[] rgdf = new int[] { r, g, numDoors, numFS };
						if (!results.get(fn).containsKey(rgdf))
							results.get(fn).put(rgdf, new ArrayList<float[][]>());
						float[][] resArr = new float[2][4];
						resString += "\nR:" + r + "\tG:" + g;

						resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers);
						results.get(fn).get(rgdf).add(resArr);
					}
				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
				String resname = "robots2_" + r + "_" + fn;
				this.printResults(resSavePlace + resname);

			}
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");
			String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
			String resname = "robots2_" + fn;
			this.printResults(resSavePlace + resname);
			printResults();
		}

	}

	public void runFSGoalsOnly() throws Exception
	{
		//	String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSI/";
		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";

		String resString = "";

		numRobots = 10;
		numFS = 0;//5;//1;
		numGoals = 11;//6;//4;
		numDoors = 0;//2;
		fn = "g20x4_r10_g11_d0_fs10";

		int maxGoals = 7;//9;
		int maxRobots = 5;//numRobots;
		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		try {
			for (int r = 2; r <= maxRobots; r++) {
				//lets do this multiple times 
				//like 5 times 
				//				for (int t = 0; t < 5; t++) {
				for (int g = 3; g <= maxGoals; g += 2) {
					ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
					ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

					int[] rgdf = new int[] { r, g, numDoors, numFS };
					if (!results.get(fn).containsKey(rgdf))
						results.get(fn).put(rgdf, new ArrayList<float[][]>());
					float[][] resArr = new float[2][4];
					resString += "\nR:" + r + "\tG:" + g;

					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers);
					results.get(fn).get(rgdf).add(resArr);
				}
				//				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
				String resname = "fsgoalonly2_" + r + "_" + fn;
				this.printResults(resSavePlace + resname);

			}
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");
			String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
			String resname = "fsgoalsonly2_" + fn;
			this.printResults(resSavePlace + resname);
			printResults();
		}

	}

	public ArrayList<Integer> generateListOfRandomNumbers(int listSize, int maxR) throws Exception
	{
		//generating random numbers 
		Random rgen = new Random();

		ArrayList<Integer> listToRet = new ArrayList<Integer>();
		if (listSize > maxR)
			throw new Exception("Can not generate " + listSize + " unique numbers from a range of " + maxR);
		int randomNumber = rgen.nextInt(maxR);
		listToRet.add(randomNumber);
		while (listToRet.size() != listSize) {
			randomNumber = rgen.nextInt(maxR);
			if (!listToRet.contains(randomNumber))
				listToRet.add(randomNumber);
		}
		return listToRet;
	}

	public void run(String[] args)
	{

		try {
			String option = args[0];
			if (option.contains("robot"))
				runRobots();
			else if (option.contains("door"))
				runDoors();
			else if (option.contains("fsgoal"))
				runFSGoalsOnly();
			else
				System.out.println("invalid option, options are: robot, door,fsgoal");
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void main(String[] args)
	{

		new CompareSTAPUSSINVI().run(args);

	}
}
