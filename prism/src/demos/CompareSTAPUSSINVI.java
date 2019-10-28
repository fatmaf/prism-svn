package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.concurrent.TimeUnit;
import java.util.stream.IntStream;

import cern.colt.Arrays;
import prism.PrismFileLog;
import prism.PrismLog;

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
			ArrayList<Integer> goalNumbers, boolean reallocOnFirstRobotDeadend, PrismLog fileLog, String mlfn)
	{
		STAPU stapu = new STAPU();
		boolean excludeRobotInitStates = false;
		double[] res = stapu.runGUISimpleTestsOne(dir, fn, numRobots, numFS, numGoals, numDoors, noreallocs, robotNumbers, goalNumbers,
				reallocOnFirstRobotDeadend, fileLog, mlfn, excludeRobotInitStates);
		//		System.out.println(res.toString());
		return res;
	}

	public double[] doSSI(String dir, String fn, int numRobots, int numGoals, int numDoors, ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers,
			boolean reallocOnFirstRobotDeadend, PrismLog fileLog, String mainLogFile)
	{
		double[] res = new SSIAuctionNestedProduct().run(dir, fn, numRobots, numGoals, numDoors, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, fileLog,
				mainLogFile);
		//		System.out.println(res.toString());
		return res;
	}

	public float[] doSTAPUTime(String dir, String fn, int numRobots, int numGoals, int numFS, int numDoors, ArrayList<Integer> robotNumbers,
			ArrayList<Integer> goalNumbers, String logSuffix, boolean reallocOnFirstRobotDeadend, boolean nullML, float[][] resArr)
	{

		PrismLog stapuLog = new PrismFileLog(dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_stapu.txt");
		String mainLogFileName = dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_stapu_mainLog.txt";
		if (nullML)
			mainLogFileName = null;
		long startTime = System.currentTimeMillis();

		double[] stapuRes = doSTAPU(dir, fn, numRobots, numFS, numGoals, numDoors, false, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, stapuLog,
				mainLogFileName);

		long endTime = System.currentTimeMillis();
		long durationStapu = (endTime - startTime);
		resArr[stapuInd] = new float[4];
		for (int i = 0; i < 3; i++)
			resArr[stapuInd][i] = (float) stapuRes[i];
		resArr[stapuInd][3] = durationStapu;
		stapuLog.println("Final Values:(p,r,c,t) " + Arrays.toString(resArr[stapuInd]));
		stapuLog.close();
		return resArr[stapuInd];

	}

	public float[] doSSITime(String dir, String fn, int numRobots, int numGoals, int numFS, int numDoors, ArrayList<Integer> robotNumbers,
			ArrayList<Integer> goalNumbers, String logSuffix, boolean reallocOnFirstRobotDeadend, boolean nullML, float[][] resArr)

	{
		PrismLog ssiLog = new PrismFileLog(dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_ssi.txt");
		String mainLogFileName = dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_ssi_mainLog.txt";
		if (nullML)
			mainLogFileName = null;
		long startTime = System.currentTimeMillis();
		double[] ssiRes = doSSI(dir, fn, numRobots, numGoals, numDoors, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, ssiLog, mainLogFileName);

		long endTime = System.currentTimeMillis();

		long durationSSI = (endTime - startTime);

		resArr[ssiInd] = new float[4];
		for (int i = 0; i < 3; i++)
			resArr[ssiInd][i] = (float) ssiRes[i];
		resArr[ssiInd][3] = durationSSI;

		ssiLog.println("Final Values:(p,r,c,t) " + Arrays.toString(resArr[ssiInd]));
		ssiLog.close();
		return resArr[ssiInd];

	}

	public String doCompare(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors, float[][] resArr, ArrayList<Integer> robotNumbers,
			ArrayList<Integer> goalNumbers, boolean reallocOnFirstRobotDeadend, boolean nullML)
	{

		Path path = Paths.get(dir + "results/logs/");
		try {
			Files.createDirectories(path);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.out.println(e.getStackTrace().toString());
		}
		String resString = "\n" + fn;

		String logSuffix = "";
		if (robotNumbers != null && goalNumbers != null) {
			System.out.println("R:" + numRobots + "-" + robotNumbers.toString() + " G:" + numGoals + " " + goalNumbers.toString());
			logSuffix = "_R:" + numRobots + "-" + robotNumbers.toString().replace(" ", "") + "_G:" + numGoals + "-" + goalNumbers.toString().replace(" ", "");

		}

		if (robotNumbers != null && goalNumbers != null)
			System.out.println("R:" + numRobots + "-" + robotNumbers.toString() + " G:" + numGoals + " " + goalNumbers.toString());


		float[] ssiRes = doSSITime(dir, fn, numRobots, numGoals, numFS, numDoors, robotNumbers, goalNumbers, logSuffix, reallocOnFirstRobotDeadend, nullML,
				resArr);

		
		float[] stapuRes = doSTAPUTime(dir, fn, numRobots, numGoals, numFS, numDoors, robotNumbers, goalNumbers, logSuffix, reallocOnFirstRobotDeadend, nullML,
				resArr);

		resString += "\nSSI Res: " + Arrays.toString(ssiRes);

		resString += "\nSTAPU Res: " + Arrays.toString(stapuRes);

		if (resArr[ssiInd][0] > resArr[stapuInd][0]) {
			System.out.println("*********************************************************");
			System.out.println("Error: " + fn + logSuffix);
			System.out.println(resString);
			System.out.println("*********************************************************");
		}
		if (resArr[ssiInd][1] > resArr[stapuInd][1]) {
			System.out.println("*********************************************************");
			System.out.println("Error: " + fn + logSuffix);
			System.out.println(resString);
			System.out.println("*********************************************************");
		}
		return resString;

	}

	public void runDoors()
	{
		//g11x11_r13_g20_fs1_d8_fsgen0_d_50.prism

		boolean reallocOnFirstRobotDeadend = true;
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSIFS/";

		int numRobots = 13;
		int numFS = 1;
		int numGoals = 20;
		int numDoors = 0;
		String fn = "";

		String fnPrefix = "g11x11_r13_g20_fs1_d8_fsgen0_d_";
		String resString = "";

		int maxGoals = numGoals;
		int maxRobots = numRobots;
		int maxD = 8;//10;
		int minD = 1;
		int incD = 1;

		int numFilesPerFS = 10;

		int[] rarr = new int[] { 2, 3 };
		int[] garr = new int[] { 3, 4 };
		int r, g;
		int testNum = 1;
		int totalTests = rarr.length * garr.length * ((maxD - minD) / incD);
		for (int d = minD; d <= maxD; d += incD) {
			for (int i = 0; i < rarr.length; i++) {
				r = rarr[i];
				for (int j = 0; j < garr.length; j++) {
					g = garr[j];

					try {

						fn = fnPrefix + d;

						if (!results.containsKey(fn))
							results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

						ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
						ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

						int[] rgdf = new int[] { r, g, d + 1, numFS };
						if (!results.get(fn).containsKey(rgdf))
							results.get(fn).put(rgdf, new ArrayList<float[][]>());
						float[][] resArr = new float[2][4];
						resString += "\nR:" + r + "\tG:" + g;

						resString += doCompare(dir, fn, r, numFS, g, d + 1, resArr, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, true);
						results.get(fn).get(rgdf).add(resArr);

						System.out.println("Done " + fn + "\nTests " + testNum + " of " + totalTests);
						testNum++;
					}

					catch (Exception e) {
						e.printStackTrace();
					}
				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
				String resname = "doors_r" + r + "d" + d + "_" + fn;
				this.printResults(resSavePlace + resname);
			}
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");
			String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
			String resname = "doors_d" + d + "_" + fn;
			this.printResults(resSavePlace + resname);
		}
		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");
		String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
		String resname = "doors_" + fnPrefix;
		this.printResults(resSavePlace + resname);
	}

	public void singleTests() throws Exception
	{
		boolean reallocOnFirstRobotDeadend = true;
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/";
		//		dir = dir+"simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		dir = dir + "simpleTests/";//"compareSTAPUSSIFS/";

		//g20x4_r10_g10_fs10_fsgen0
		int numRobots = 10;
		int numFS = 1;
		int numGoals = 11;
		int numDoors = 0;
		String fn = "g20x4_r10_g10_fs80fs21_fsgen7_";//"g20x4_r10_g10_fs80fs21_fsgen3_"; //_R:2-[7,2]_G:6-[7,3,0,4,6]
		//this is an example where ssi exp t > stapu "g20x4_r10_g10_fs80fs11_fsgen5_";//"g20x4_r10_g10_fs80fs1_fsgen3_";//"g20x4_r10_g10_fs10_fsgen0_";//"g5_r2_t3_d2_fs1";
		
		numRobots = 10;
		numFS = 0;//5;//1;
		numGoals = 11;//6;//4;
		numDoors = 0;//2;
		fn = "g19x5_r10_t11_d0_fs0";
		
		String resString = "";
		int r = 4;//numRobots;
		int g = 3;//numGoals;

		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		ArrayList<Integer> robotNumbers = new ArrayList<Integer>();//generateListOfRandomNumbers(r, numRobots);
		ArrayList<Integer> goalNumbers = new ArrayList<Integer>(); //generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

		////		robotNumbers.add(2);
		////		robotNumbers.add(7);
		//		robotNumbers.add(7);
		//		robotNumbers.add(2);
		//		//		goalNumbers.add(5); 
		//		//		goalNumbers.add(4);
		//		//		goalNumbers.add(8);
		//		//		goalNumbers.add(1);
		//		//		goalNumbers.add(7);
		//		goalNumbers.add(7);
		//		goalNumbers.add(3);
		//		goalNumbers.add(0);
		//		goalNumbers.add(4);
		//		goalNumbers.add(6);
		//R:2-[8,5]_G:3-[1,0]
//		robotNumbers.add(8);
//		robotNumbers.add(5);
//		goalNumbers.add(1);
//		goalNumbers.add(0);
		
		robotNumbers.add(8); 
		robotNumbers.add(6);
		robotNumbers.add(0); 
		robotNumbers.add(7);
		
		goalNumbers.add(1);
		goalNumbers.add(3);
		

		int[] rgdf = new int[] { r, g, numDoors, numFS };
		if (!results.get(fn).containsKey(rgdf))
			results.get(fn).put(rgdf, new ArrayList<float[][]>());
		float[][] resArr = new float[2][4];
		resString += "\nR:" + r + "\tG:" + g;

//		resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, true);
//		results.get(fn).get(rgdf).add(resArr);
		reallocOnFirstRobotDeadend = true;
		resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, true);
		results.get(fn).get(rgdf).add(resArr);

		printResults();

	}

	public void runRobots() throws Exception
	{
		boolean reallocOnFirstRobotDeadend = true;
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

		int maxGoals = 9;
		int maxRobots = 9;
		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		try {
			for (int r = 2; r <= maxRobots; r += 2) {
				//lets do this multiple times 
				//like 5 times 
				for (int g = 3; g <= maxGoals; g += 2) {
					for (int t = 0; t < 10; t++) {

						//						if(r <= 5 && g <=7)
						//							continue;
						ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
						ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

						int[] rgdf = new int[] { r, g, numDoors, numFS };
						if (!results.get(fn).containsKey(rgdf))
							results.get(fn).put(rgdf, new ArrayList<float[][]>());
						float[][] resArr = new float[2][4];
						resString += "\nR:" + r + "\tG:" + g;

						resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, false);
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

	//	public void runFSGoalsOnly() throws Exception
	//
	//	{
	//		boolean reallocOnFirstRobotDeadend = false;
	//		//	String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
	//		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSI/";
	//		int numRobots = 2;
	//		int numFS = 1;
	//		int numGoals = 4;
	//		int numDoors = 2;
	//		String fn = "g5_r2_t3_d2_fs1";
	//
	//		String resString = "";
	//
	//		numRobots = 10;
	//		numFS = 0;//5;//1;
	//		numGoals = 11;//6;//4;
	//		numDoors = 0;//2;
	//		fn = "g20x4_r10_g11_d0_fs10";
	//
	//		int maxGoals = 7;//9;
	//		int maxRobots = 5;//numRobots;
	//		if (!results.containsKey(fn))
	//			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
	//		try {
	//			for (int r = 2; r <= maxRobots; r++) {
	//				//lets do this multiple times 
	//				//like 5 times 
	//				//				for (int t = 0; t < 5; t++) {
	//				for (int g = 3; g <= maxGoals; g += 2) {
	//					ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
	//					ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 
	//
	//					int[] rgdf = new int[] { r, g, numDoors, numFS };
	//					if (!results.get(fn).containsKey(rgdf))
	//						results.get(fn).put(rgdf, new ArrayList<float[][]>());
	//					float[][] resArr = new float[2][4];
	//					resString += "\nR:" + r + "\tG:" + g;
	//
	//					resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, false);
	//					results.get(fn).get(rgdf).add(resArr);
	//				}
	//				//				}
	//				System.out.println("***************************************************************");
	//				System.out.println(resString);
	//				System.out.println("***************************************************************");
	//				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
	//				String resname = "fsgoalonly2_" + r + "_" + fn;
	//				this.printResults(resSavePlace + resname);
	//
	//			}
	//		} catch (Exception e) {
	//			e.printStackTrace();
	//		} finally {
	//			System.out.println("***************************************************************");
	//			System.out.println(resString);
	//			System.out.println("***************************************************************");
	//			String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
	//			String resname = "fsgoalsonly2_" + fn;
	//			this.printResults(resSavePlace + resname);
	//			printResults();
	//		}
	//
	//	}

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
				runFSGR();
			//				runFSGoalsOnly();
			else if (option.contains("fs")) {

				runFS();

			} else if (option.contains("single"))
				singleTests();
			else
				System.out.println("invalid option, options are: robot, door,fsgoal");
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void runFS()
	{
		boolean reallocOnFirstRobotDeadend = true;
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSIFS/";

		int numRobots = 10;
		int numFS = 1;
		int numGoals = 10;
		int numDoors = 0;
		String fn = "";

		String resString = "";

		int maxGoals = numGoals;
		int maxRobots = numRobots;
		int maxFS = 80;//10;
		int minFS = 1;
		int incFS = 10;

		int numFilesPerFS = 10;

		int[] rarr = new int[] { 2, 4 };
		int[] garr = new int[] { 3, 6 };
		int r, g;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr.length; j++) {
				g = garr[j];
				try {

					for (int fs = minFS; fs <= maxFS; fs += incFS) {
						for (int fileForFS = 1; fileForFS <= numFilesPerFS; fileForFS++) {
							fn = "g20x4_r10_g10_fs" + maxFS + "fs" + fs + "_fsgen" + fileForFS + "_"; //40fs1_fsgen1_0"
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

							ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
							ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

							int[] rgdf = new int[] { r, g, numDoors, fs };
							if (!results.get(fn).containsKey(rgdf))
								results.get(fn).put(rgdf, new ArrayList<float[][]>());
							float[][] resArr = new float[2][4];
							resString += "\nR:" + r + "\tG:" + g;

							resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, false);
							results.get(fn).get(rgdf).add(resArr);

						}
						System.out.println("***************************************************************");
						System.out.println(resString);
						System.out.println("***************************************************************");
						String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
						String resname = "fsOnly_fs_r" + r + "g" + g + "fs" + fs + "_" + fn;
						this.printResults(resSavePlace + resname);
						//			}
						//				}
					}

				} catch (Exception e) {
					e.printStackTrace();
				} finally {
					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");
					String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
					String resname = "fsOnly_r" + r + "g" + g + "_" + fn;
					this.printResults(resSavePlace + resname);
					printResults();
				}
			}
		}
	}

	private void runFSGR()
	{
		boolean reallocOnFirstRobotDeadend = true;
		String dir = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSIFS/";

		int numRobots = 15;
		int numFS = 1;
		int numGoals = 17;
		int numDoors = 0;
		String fn = "";

		String fnPrefix = "g10x10_r15_g17_fs84_centergoals_fs";
		String resString = "";

		int maxGoals = numGoals;
		int maxRobots = numRobots;
		int maxFS = 108;//10;
		int minFS = 101;
		int incFS = 10;

		int numFilesPerFS = 10;

		boolean doList = false;

		int[] rarr = new int[] { 2, 4 };
		int[] garr = new int[] { 3, 6 };
		int r, g;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr.length; j++) {
				g = garr[j];
				try {

					for (int fs = minFS; fs <= maxFS; fs += incFS) {
						for (int fileForFS = 1; fileForFS <= numFilesPerFS; fileForFS++) {
							fn = fnPrefix + fs + "_fsgen" + fileForFS + "_r" + (fileForFS - 1) + "_t" + (fileForFS - 1);
							//							if(!fn.contains("g10x10_r15_g17_fs84_centergoals_fs1_fsgen2_r1_t1"))
							//								continue;
							if (doList) {
								//								System.out.println(fn);
								Path path = Paths.get(dir + fn + ".prop");
								if (!Files.exists(path)) {

									System.out.println(fn + " DOESNT EXIST");
								}
							}
							if (!doList) {
								if (!results.containsKey(fn))
									results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

								ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
								ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); //-1 cuz the last one is always a safety 

								int[] rgdf = new int[] { r, g, numDoors, fs };
								if (!results.get(fn).containsKey(rgdf))
									results.get(fn).put(rgdf, new ArrayList<float[][]>());
								float[][] resArr = new float[2][4];
								resString += "\nR:" + r + "\tG:" + g;

								resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers, reallocOnFirstRobotDeadend, false);
								results.get(fn).get(rgdf).add(resArr);
							}
						}
						if (!doList) {
							System.out.println("***************************************************************");
							System.out.println(resString);
							System.out.println("***************************************************************");
							String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
							String resname = "fsOnly_fs_r" + r + "g" + g + "fs" + fs + "_" + fn;
							this.printResults(resSavePlace + resname);
							//			}
							//				}
						}
					}

				} catch (Exception e) {
					System.out.println("ERROR");
					System.out.println(e.getStackTrace().toString());
					e.printStackTrace();
				} finally {
					if (!doList) {
						System.out.println("***************************************************************");
						System.out.println(resString);
						System.out.println("***************************************************************");
						String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
						String resname = "fsOnly_r" + r + "g" + g + "_" + fn;
						this.printResults(resSavePlace + resname);
						printResults();
					}
				}
			}
			if (!doList) {
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
				String resname = "fsOnly_r" + r + "_" + fn;
				this.printResults(resSavePlace + resname);
				printResults();
			}
		}
		if (!doList) {
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");
			String resSavePlace = "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
			String resname = "fsOnly_" + fnPrefix + minFS;
			this.printResults(resSavePlace + resname);
		}
	}

	public static void main(String[] args)
	{

		new CompareSTAPUSSINVI().run(args);

	}
}
