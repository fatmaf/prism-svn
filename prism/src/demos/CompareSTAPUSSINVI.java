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
import java.util.Scanner;
import java.util.concurrent.TimeUnit;
import java.util.stream.IntStream;

import cern.colt.Arrays;
import prism.PrismFileLog;
import prism.PrismLog;

public class CompareSTAPUSSINVI {
	int rInd = 0;
	int gInd = 1;
	int dInd = 2;
	int fInd = 3;
	int gridInd = 4;
	int ssiInd = 0;
	int stapuInd = 1;
	int probInd = 0;
	int taskInd = 1;
	int costInd = 2;
	int timeInd = 3;
	int firstSolTimeInd = 4;
	int allReplanningTimeInd = 5;

	boolean doDebug = false;
	boolean reallocSTAPUOnFirstDeadend = false;
	boolean doSeqSTAPUPolicy = true;
	boolean reallocSSIOnFirstDeadend = true;
	boolean stapuNoReallocs = false;
	boolean ssiNoReallocs = false;

	String resSavePlace = "/data/private/fxf603/code/prism-svn/prism/tests/resultsSummary/";// "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
	String testDirBaseLoc = "/data/private/fxf603/code/prism-svn/prism/tests/";// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/";

	ArrayList<String> errors = new ArrayList<String>();
	HashMap<String, HashMap<int[], ArrayList<float[][]>>> results = new HashMap<String, HashMap<int[], ArrayList<float[][]>>>();

	public void printResults(String contest, boolean hasGridData) {
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

		String header = "fn\tr\tg\td\tf";
		if (hasGridData)
			header = header + "\tgrid";
		header = header + "\tl\tp\tt\tc\ttT\tfirstSolT\tallReplanningT\n";
		outLog.print(header);
		for (String fn : results.keySet()) {
			// String row = fn;
			HashMap<int[], ArrayList<float[][]>> datahere = results.get(fn);
			for (int[] rgdf : datahere.keySet()) {
				ArrayList<float[][]> valueslist = datahere.get(rgdf);
				for (float[][] valuesall : valueslist) {
					int resInd = ssiInd;
					float[] values = valuesall[resInd];
					String row = fn + "\t" + rgdf[rInd] + "\t" + rgdf[gInd] + "\t" + rgdf[dInd] + "\t" + rgdf[fInd];
					if (hasGridData)
						row = row + "\t" + rgdf[gridInd];
					row = row + "\tssi\t" + values[probInd] + "\t" + values[taskInd] + "\t" + values[costInd] + "\t"
							+ values[timeInd];
					row = row + "\t" + values[firstSolTimeInd] + "\t" + values[allReplanningTimeInd];
					outLog.println(row);
					resInd = stapuInd;
					values = valuesall[stapuInd];
					row = fn + "\t" + rgdf[rInd] + "\t" + rgdf[gInd] + "\t" + rgdf[dInd] + "\t" + rgdf[fInd];
					if (hasGridData)
						row = row + "\t" + rgdf[gridInd];
					row = row + "\tstapu\t" + values[probInd] + "\t" + values[taskInd] + "\t" + values[costInd] + "\t"
							+ values[timeInd];
					row = row + "\t" + values[firstSolTimeInd] + "\t" + values[allReplanningTimeInd];
					outLog.println(row);
				}
			}
		}

	}

	public void printResults(boolean hasGridData) {

		printResults(null, hasGridData);

	}

	public long[] doSTAPU(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors,
			ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers, boolean reallocOnFirstRobotDeadend,
			PrismLog fileLog, String mlfn, double[] res) {
		STAPU stapu = new STAPU();
		boolean excludeRobotInitStates = false;
		stapu.debugSTAPU = doDebug;
		stapu.doSeqPolicyBuilding = this.doSeqSTAPUPolicy;
		stapu.noreallocations = this.stapuNoReallocs;
		double[] resHere = stapu.runGUISimpleTestsOne(dir, fn, numRobots, numFS, numGoals, numDoors, robotNumbers,
				goalNumbers, reallocOnFirstRobotDeadend, fileLog, mlfn, excludeRobotInitStates);
		// System.out.println(res.toString());
		for (int i = 0; i < resHere.length; i++)
			res[i] = resHere[i];
		long[] timeRes = new long[3];
		timeRes[0] = stapu.stapuTimeDuration;
		timeRes[1] = stapu.stapuFirstSolDuration;
		timeRes[2] = stapu.stapuAllReplanningDuration;
		return timeRes;
	}

	public long[] doSSI(String dir, String fn, int numRobots, int numGoals, int numDoors,
			ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers, boolean reallocOnFirstRobotDeadend,
			PrismLog fileLog, String mainLogFile, double[] res) {
		SSIAuctionNestedProduct ssi = new SSIAuctionNestedProduct();
		ssi.debugSSI = doDebug;
		ssi.doingReallocs = !this.ssiNoReallocs;
		double[] ssires = ssi.run(dir, fn, numRobots, numGoals, numDoors, robotNumbers, goalNumbers,
				reallocOnFirstRobotDeadend, fileLog, mainLogFile);
		// System.out.println(res.toString());
		for (int i = 0; i < ssires.length; i++)
			res[i] = ssires[i];

		long[] timeRes = new long[3];
		timeRes[0] = ssi.totalTimeDuration;
		timeRes[1] = ssi.firstSolDuration;
		timeRes[2] = ssi.allReplanningDuration;
		return timeRes;
	}

	public float[] doSTAPUTime(String dir, String fn, int numRobots, int numGoals, int numFS, int numDoors,
			ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers, String logSuffix, boolean nullML,
			float[][] resArr) {
		boolean reallocOnFirstRobotDeadend = this.reallocSTAPUOnFirstDeadend;
		PrismLog stapuLog = new PrismFileLog(
				dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_stapu.txt");
		String mainLogFileName = dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix
				+ "_stapu_mainLog.txt";
		if (nullML)
			mainLogFileName = null;
		long startTime = System.currentTimeMillis();

		double[] stapuRes = new double[3];
		long[] stapuTimeRes = doSTAPU(dir, fn, numRobots, numFS, numGoals, numDoors, robotNumbers, goalNumbers,
				reallocOnFirstRobotDeadend, stapuLog, mainLogFileName, stapuRes);
		long modifiedDurationStapu = stapuTimeRes[1];

		long endTime = System.currentTimeMillis();
		long durationStapu = (endTime - startTime);
		resArr[stapuInd] = new float[6];
		for (int i = 0; i < 3; i++)
			resArr[stapuInd][i] = (float) stapuRes[i];
		for (int i = 3; i < 6; i++)
			resArr[stapuInd][i] = (float) stapuTimeRes[i - 3];
		// resArr[stapuInd][3] = modifiedDurationStapu;//durationStapu;
		stapuLog.println("Final Values:(p,r,c,t,fst,rpt) " + Arrays.toString(resArr[stapuInd]));
		stapuLog.close();
		return resArr[stapuInd];

	}

	public float[] doSSITime(String dir, String fn, int numRobots, int numGoals, int numFS, int numDoors,
			ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers, String logSuffix, boolean nullML,
			float[][] resArr)

	{
		boolean reallocOnFirstRobotDeadend = this.reallocSSIOnFirstDeadend;
		PrismLog ssiLog = new PrismFileLog(
				dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_ssi.txt");
		String mainLogFileName = dir + "results/logs/" + fn + "_r" + numRobots + "_g" + numGoals + logSuffix
				+ "_ssi_mainLog.txt";
		if (nullML)
			mainLogFileName = null;
		long startTime = System.currentTimeMillis();
		double[] ssiRes = new double[3];
		long[] ssiTimeRes = doSSI(dir, fn, numRobots, numGoals, numDoors, robotNumbers, goalNumbers,
				reallocOnFirstRobotDeadend, ssiLog, mainLogFileName, ssiRes);

		long endTime = System.currentTimeMillis();

		long durationSSI = (endTime - startTime);

		resArr[ssiInd] = new float[6];
		for (int i = 0; i < 3; i++)
			resArr[ssiInd][i] = (float) ssiRes[i];
		for (int i = 3; i < 6; i++)
			resArr[ssiInd][i] = (float) ssiTimeRes[i - 3];
		// resArr[ssiInd][3] = durationSSIModified;//durationSSI;

		ssiLog.println("Final Values:(p,r,c,t,fst,rpt) " + Arrays.toString(resArr[ssiInd]));
		ssiLog.close();
		return resArr[ssiInd];

	}

	public String doCompare(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors,
			float[][] resArr, ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers, boolean nullML) {

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
			System.out.println(
					"R:" + numRobots + "-" + robotNumbers.toString() + " G:" + numGoals + " " + goalNumbers.toString());
			logSuffix = "_R:" + numRobots + "-" + robotNumbers.toString().replace(" ", "") + "_G:" + numGoals + "-"
					+ goalNumbers.toString().replace(" ", "");

		}

		if (robotNumbers != null && goalNumbers != null)
			System.out.println(
					"R:" + numRobots + "-" + robotNumbers.toString() + " G:" + numGoals + " " + goalNumbers.toString());

		float[] ssiRes = doSSITime(dir, fn, numRobots, numGoals, numFS, numDoors, robotNumbers, goalNumbers, logSuffix,
				nullML, resArr);

		float[] stapuRes = doSTAPUTime(dir, fn, numRobots, numGoals, numFS, numDoors, robotNumbers, goalNumbers,
				logSuffix, nullML, resArr);

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
		} else {
			if (resArr[stapuInd][2] > resArr[ssiInd][2]) {
				System.out.println("*********************************************************");
				System.out.println("Error: " + fn + logSuffix);
				System.out.println(resString);
				System.out.println("*********************************************************");
			}
		}
		return resString;

	}

	public void runDoors() {
		// g11x11_r13_g20_fs1_d8_fsgen0_d_50.prism
		boolean hasGridData = false;

		String dir = testDirBaseLoc + "compareSTAPUSSIFS/";// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSIFS/";

		int numRobots = 13;
		int numFS = 1;
		int numGoals = 20;
		int numDoors = 0;
		String fn = "";

		String fnPrefix = "g11x11_r13_g20_fs1_d8_fsgen0_d_";
		String resString = "";

		int maxGoals = numGoals;
		int maxRobots = numRobots;
		int maxD = 8;// 10;
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
						ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1 cuz the
																											// last one
																											// is always
																											// a safety

						int[] rgdf = new int[] { r, g, d + 1, numFS };
						if (!results.get(fn).containsKey(rgdf))
							results.get(fn).put(rgdf, new ArrayList<float[][]>());
						float[][] resArr = new float[2][6];
						resString += "\nR:" + r + "\tG:" + g;

						resString += doCompare(dir, fn, r, numFS, g, d + 1, resArr, robotNumbers, goalNumbers, true);
						results.get(fn).get(rgdf).add(resArr);

						System.out.println("Done " + fn + "\nTests " + testNum + " of " + totalTests);
						testNum++;

						String resname = "doors_r" + r + "d" + d + "_" + fn;
						this.printResults(resSavePlace + resname, hasGridData);
					}

					catch (Exception e) {
						e.printStackTrace();
					}
				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");

			}
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");

			String resname = "doors_d" + d + "_" + fn;
			this.printResults(resSavePlace + resname, hasGridData);
		}
		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");

		String resname = "doors_" + fnPrefix;
		this.printResults(resSavePlace + resname, hasGridData);
	}

	public void singleTests() throws Exception {
		doDebug = false;
		this.reallocSSIOnFirstDeadend = true;
		this.reallocSTAPUOnFirstDeadend = false;// true;

		this.doSeqSTAPUPolicy = true;// false;

		this.stapuNoReallocs = false;
		this.ssiNoReallocs = false;

		String dir = testDirBaseLoc;// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/";
		// dir =
		// dir+"simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
		// dir = dir + "strangehouse/";//"compareSTAPUSSIFS/";
		// dir = dir + "compareSTAPUSSIFS/";

		dir = dir + "gridIncTests/";
		// g20x4_r10_g10_fs10_fsgen0
		int numRobots = 10;
		int numFS = 31;
		int numGoals = 11;
		int numDoors = 0;
		String fn = "r10_g10_grid_9_fsp_0_0_";
		// "g20x4_r10_g10_fs80fs21_fsgen8_";//"g20x4_r10_g10_fs80fs31_fsgen9_";
		// fn = "g20x4_r10_g10_fs80fs11_fsgen6_";
		// fn = "strange_house_r10_g10_fs40_1_";
		// strange_house_r10_g10_fs20_4__R:4-[6,3,7,5]_G:3-[4,5]
		// strange_house_r10_g10_fs5_0__R:2-[1,7]_G:3-[6,3]
		// fn = "strange_house_r10_g10_fs5_0_";//
		// "strange_house_r10_g10_fs20_4_";//R:4-[6,3,7,5]_G:3-[4,5]
		// fn = "strange_house_r10_g10_fs5_7_";//Error:
		// strange_house_r10_g10_fs5_7__R:4-[8,6,9,7]_G:3-[5,6]
		// fn =
		// "strange_house_r10_g10_fs25_6_";//strange_house_r10_g10_fs25_6__R:4-[5,3,2,6]_G:3-[7,0]
		fn = "g20x4_r10_g10_fs80fs11_fsgen9_";// g20x4_r10_g10_fs80fs11_fsgen9__R:2-[0,3]_G:6-[5,6,7,3,8]
		fn = "g20x4_r10_g10_fs80fs51_fsgen10_"; // g20x4_r10_g10_fs80fs51_fsgen10__R:4-[9,6,7,8]_G:3-[4,8]
		fn = "r10_g10_grid_9_fsp_0_0_"; // R:2-[3, 1] G:3 [3, 0]
		boolean hasGridData = true;
		int gridV = 9;
		String resString = "";
		int r = 2;// numRobots;
		int g = 3;// 3;//numGoals;
		// R:4-[2, 4, 1, 7] G:3 [1, 6]
		// R:4-[5, 6, 9, 4] G:6 [0, 3, 8, 1, 5]
		// R:2-[8, 9] G:3 [0, 7]
		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		ArrayList<Integer> robotNumbers = new ArrayList<Integer>();// generateListOfRandomNumbers(r, numRobots);
		ArrayList<Integer> goalNumbers = new ArrayList<Integer>(); // generateListOfRandomNumbers(g - 1, numGoals - 1);
																	// //-1 cuz the last one is always a safety

		robotNumbers.add(3);
		robotNumbers.add(1);
		// robotNumbers.add(7);
		// robotNumbers.add(8);
		// robotNumbers.add(2);
		// robotNumbers.add(6);

		// goalNumbers.add(5);
		// goalNumbers.add(2);

		goalNumbers.add(3);
		goalNumbers.add(0);
		// goalNumbers.add(7);
		// goalNumbers.add(3);
		// goalNumbers.add(8);
		int[] rgdf;
		if (hasGridData)
			rgdf = new int[] { r, g, numDoors, numFS, gridV };
		else
			rgdf = new int[] { r, g, numDoors, numFS };
		if (!results.get(fn).containsKey(rgdf))
			results.get(fn).put(rgdf, new ArrayList<float[][]>());
		float[][] resArr = new float[2][6];
		resString += "\nR:" + r + "\tG:" + g;

		// resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers,
		// goalNumbers, reallocOnFirstRobotDeadend, true);
		// results.get(fn).get(rgdf).add(resArr);
		resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers, doDebug);
		results.get(fn).get(rgdf).add(resArr);

		printResults(hasGridData);

	}

	public void runRobotsVariations() throws Exception {
		boolean[] tf = new boolean[] { true, false };
		for (boolean flag : tf) {
			this.doSeqSTAPUPolicy = flag;
			for (boolean flag2 : tf) {
				this.reallocSSIOnFirstDeadend = flag2;
				for (boolean flag3 : tf) {
					this.reallocSTAPUOnFirstDeadend = flag3;
					String suffix = "_ssiR_" + this.reallocSSIOnFirstDeadend + "_stapuR_"
							+ this.reallocSTAPUOnFirstDeadend;
					if (flag) {
						suffix += "_seq";
					} else {
						suffix += "_hol";
					}
					runRobots(suffix);
				}
			}
		}

	}

	public void runRobots(String suffix) throws Exception {

		boolean hasGridData = false;
		String dir = testDirBaseLoc;// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";
		// String dir=
		// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/compareSTAPUSSI/";
		dir = dir + "simpleTests/";
		int numRobots = 2;
		int numFS = 1;
		int numGoals = 4;
		int numDoors = 2;
		String fn = "g5_r2_t3_d2_fs1";

		String resString = "";

		numRobots = 10;

		numFS = 0;// 5;//1;
		numGoals = 11;// 6;//4;
		numDoors = 0;// 2;
		fn = "g19x5_r10_t11_d0_fs0";

		int maxGoals = 9;
		int maxRobots = 9;
		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		try {

			// lets do this multiple times
			// like 5 times
			for (int g = 3; g <= maxGoals; g += 2) {
				for (int r = 2; r <= maxRobots; r += 2) {
					for (int t = 0; t < 10; t++) {

						// if(r <= 5 && g <=7)
						// continue;
						ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
						ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1 cuz the
																											// last one
																											// is always
																											// a safety

						int[] rgdf = new int[] { r, g, numDoors, numFS };
						if (!results.get(fn).containsKey(rgdf))
							results.get(fn).put(rgdf, new ArrayList<float[][]>());
						float[][] resArr = new float[2][6];
						resString += "\nR:" + r + "\tG:" + g;

						resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers,
								doDebug);
						results.get(fn).get(rgdf).add(resArr);

						String resname = "robots_g" + g + "_" + fn + suffix;
						this.printResults(resSavePlace + resname, hasGridData);
					}
				}
				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");

			}
		} catch (Exception e) {
			e.printStackTrace();
		} finally {
			System.out.println("***************************************************************");
			System.out.println(resString);
			System.out.println("***************************************************************");

			String resname = "robots_" + fn + suffix;
			this.printResults(resSavePlace + resname, hasGridData);
			printResults(hasGridData);
		}

	}

	public ArrayList<Integer> generateListOfRandomNumbers(int listSize, int maxR) throws Exception {
		// generating random numbers
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

	public void run(String[] args) {
		String[] options = new String[] { "robot", "door", "fsgoal", "fs", "single", "warehouse1", "warehouse2", "rvar",
				"wvar", "strangehouse", "grid" };

		this.doSeqSTAPUPolicy = true;// false;
		this.reallocSSIOnFirstDeadend = true;
		this.reallocSTAPUOnFirstDeadend = false;
		this.ssiNoReallocs = false;
		this.stapuNoReallocs = false;
		try {
			String option = args[0];
			if (option.contains(options[0]))
				runRobots("");
			else if (option.contains(options[1]))
				runDoors();
			else if (option.contains(options[2]))
				System.out.println("Not implemented");
			// runFSGR();
			// // runFSGoalsOnly();
			else if (option.contains(options[3])) {

				runFS();

			} else if (option.contains(options[4]))
				singleTests();
			else if (option.contains(options[5])) {

				runWarehouse(0, "");
			} else if (option.contains(options[6]))
				runWarehouse(1, "");
			else if (option.contains(options[7]))
				this.runRobotsVariations();
			else if (option.contains(options[8]))
				this.runWarehouseVariations();
			else if (option.contains(options[9])) {

				runStrangeHouse();

			} else if (option.contains(options[10])) {
				runGridStates("");
			} else
				System.out.println("invalid option, options are: " + Arrays.toString(options));
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void runFS() {
		boolean hasGridData = false;

		String dir = testDirBaseLoc + "compareSTAPUSSIFS/";

		int numRobots = 10;
		int numFS = 1;
		int numGoals = 10;
		int numDoors = 0;
		String fn = "";

		String resString = "";

		int maxGoals = numGoals;
		int maxRobots = numRobots;
		int maxFS = 80;// 10;
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
							fn = "g20x4_r10_g10_fs" + maxFS + "fs" + fs + "_fsgen" + fileForFS + "_"; // 40fs1_fsgen1_0"
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

							ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
							ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1 cuz
																												// the
																												// last
																												// one
																												// is
																												// always
																												// a
																												// safety

							int[] rgdf = new int[] { r, g, numDoors, fs };
							if (!results.get(fn).containsKey(rgdf))
								results.get(fn).put(rgdf, new ArrayList<float[][]>());
							float[][] resArr = new float[2][6];
							resString += "\nR:" + r + "\tG:" + g;

							resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers,
									doDebug);
							results.get(fn).get(rgdf).add(resArr);
							String resname = "fsOnly_fs_r" + r + "g" + g + "fs" + fs + "_" + fn;
							this.printResults(resSavePlace + resname, hasGridData);
						}
						System.out.println("***************************************************************");
						System.out.println(resString);
						System.out.println("***************************************************************");

						// }
						// }
					}

				} catch (Exception e) {
					e.printStackTrace();
				} finally {
					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");

					String resname = "fsOnly_r" + r + "g" + g + "_" + fn;
					this.printResults(resSavePlace + resname, hasGridData);
					printResults(hasGridData);
				}
			}
		}
	}

	private void runWarehouseVariations() {

		boolean[] tf = new boolean[] { true, false };
		for (boolean flag : tf) {
			this.doSeqSTAPUPolicy = flag;
			for (boolean flag2 : tf) {
				this.reallocSSIOnFirstDeadend = flag2;
				for (boolean flag3 : tf) {
					this.reallocSTAPUOnFirstDeadend = flag3;
					String suffix = "_ssiR_" + this.reallocSSIOnFirstDeadend + "_stapuR_"
							+ this.reallocSTAPUOnFirstDeadend;
					if (flag) {
						suffix += "_seq";
					} else {
						suffix += "_hol";
					}
					if (this.doSeqSTAPUPolicy == true && this.reallocSSIOnFirstDeadend == true
							&& this.reallocSTAPUOnFirstDeadend == true && flag == true)
						continue;

					runWarehouse(0, suffix);
					// runWarehouse(1,suffix);
				}
			}
		}

	}

	private void runWarehouse(int tnum, String fnSuffix) {
		String fnPrefix = "shelfDepot_r10_g10";
		int[] fsShelfDepot = new int[] { 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80 };
		int[] fsDepotShelf = new int[] { 0, 8, 16, 24, 32, 40, 48, 56, 64, 72, 80 };
		String fsBit = "fs_";
		// runWarehouse(fnPrefix,fsShelfDepot,fsBit);
		// runWarehouse("depotShelf_r10_g10",fsDepotShelf,fsBit);
		fsBit = "_fs";
		int[] fsShelfDepotOpenSides = new int[] { 0, 18, 36, 54, 72, 90 };
		int[] fsdepotShelfOpenSides = new int[] { 0, 9, 18, 27, 36, 45, 54, 63, 72, 81, 90 };
		fnPrefix = "depotShelfSideOpen_r10_g10";
		if (tnum == 0)
			runWarehouse(fnPrefix, fsShelfDepotOpenSides, fsBit, fnSuffix);
		fnPrefix = "shelfDepotSideOpen_r10_g10";
		if (tnum == 1)
			runWarehouse(fnPrefix, fsShelfDepotOpenSides, fsBit, fnSuffix);
	}

	private void runWarehouse(String fnPrefix, int[] fsShelfDepot, String fsBit, String fnSuffix) {
		boolean hasGridData = false;
		String dir = testDirBaseLoc + "warehouse/";
		String keepGoing = "";

		int numRobots = 10;
		int numGoals = 10;
		int numDoors = 0;
		String fn = "";

		String resString = "";

		int numFilesPerFS = 10;

		int[] rarr = new int[] { 2, 4 };
		int[][] garr = new int[][] { { 3, 6, 9 }, { 3, 6, 9 } };
		int r, g;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr[i].length; j++) {
				g = garr[i][j];
				try {

					for (int fs : fsShelfDepot) {
						for (int fileForFS = 0; fileForFS < numFilesPerFS; fileForFS++) {
							fn = fnPrefix + fsBit + fs + "_" + +fileForFS + "_"; // 40fs1_fsgen1_0"
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

							ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
							ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1 cuz
																												// the
																												// last
																												// one
																												// is
																												// always
																												// a
																												// safety

							int[] rgdf = new int[] { r, g, numDoors, fs };
							if (!results.get(fn).containsKey(rgdf))
								results.get(fn).put(rgdf, new ArrayList<float[][]>());
							float[][] resArr = new float[2][6];
							resString += "\nR:" + r + "\tG:" + g;

							resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers,
									doDebug);
							results.get(fn).get(rgdf).add(resArr);
							String resname = "w_fs_r" + r + "g" + g + "fs" + fs + "_" + fnPrefix + fnSuffix;
							this.printResults(resSavePlace + resname, hasGridData);

						}
						System.out.println("***************************************************************");
						System.out.println(resString);
						System.out.println("***************************************************************");

						// }
						// }

					}

				} catch (Exception e) {
					e.printStackTrace();
				} finally {
					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");

					String resname = "w_r" + r + "g" + g + "_" + fnPrefix + fnSuffix;
					this.printResults(resSavePlace + resname, hasGridData);
					// printResults();
				}
				System.out.println("Waiting to say go");
				Scanner in = new Scanner(System.in);
				keepGoing = in.nextLine();
				while (keepGoing.contains("no")) {
					keepGoing = in.nextLine();
					System.out.println("Paused at " + "w_r" + r + "g" + g + "_" + fnPrefix + fnSuffix);
				}
			}
		}

		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");

		String resname = "w_" + fnPrefix + fnSuffix;
		this.printResults(resSavePlace + resname, hasGridData);
		printResults(hasGridData);
	}

	private void runStrangeHouse() {
		boolean hasGridData = false;
		String dir = testDirBaseLoc + "strangehouse/";

		int numRobots = 10;
		int numGoals = 10;
		int numDoors = 0;
		String fn = "";

		String resString = "";

		int numFilesPerFS = 10;

		String fnPrefix = "sh_r10_g10_";// "strange_house_r10_g10_";
		String fsBit = "fs";
		String fnSuffix = "";
		int[] fsList = new int[] { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45 };
		int[] rarr = new int[] { 2, 4 };
		int[][] garr = new int[][] { { 3, 6 }, { 3, 6 } };
		int r, g;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr[i].length; j++) {
				g = garr[i][j];
				try {

					for (int fs : fsList) {
						for (int fileForFS = 0; fileForFS < numFilesPerFS; fileForFS++) {
							fn = fnPrefix + fsBit + fs + "_" + +fileForFS + "_";
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());

							ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
							ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1 cuz
																												// the
																												// last
																												// one
																												// is
																												// always
																												// a
																												// safety

							int[] rgdf = new int[] { r, g, numDoors, fs };
							if (!results.get(fn).containsKey(rgdf))
								results.get(fn).put(rgdf, new ArrayList<float[][]>());
							float[][] resArr = new float[2][6];
							resString += "\nR:" + r + "\tG:" + g;

							resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers,
									doDebug);
							results.get(fn).get(rgdf).add(resArr);
							String resname = "sh_fs_r" + r + "g" + g + "fs" + fs + "_" + fnPrefix + fnSuffix;
							this.printResults(resSavePlace + resname, hasGridData);
						}
						System.out.println("***************************************************************");
						System.out.println(resString);
						System.out.println("***************************************************************");

						// }
						// }

					}

				} catch (Exception e) {
					e.printStackTrace();
				} finally {
					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");

					String resname = "sh_r" + r + "g" + g + "_" + fnPrefix + fnSuffix;
					this.printResults(resSavePlace + resname, hasGridData);
					// printResults();
				}

			}
		}

		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");

		String resname = "sh_" + fnPrefix + fnSuffix;
		this.printResults(resSavePlace + resname, hasGridData);
		printResults(hasGridData);
	}

	private void runGridStates(String fnSuffix) {

		boolean hasGridData = true;
		String fnPrefix = "r10_g10_a1_grid_";// "r10_g10_grid_";
		String dir = testDirBaseLoc + "gridIncWithRepeats/";// "gridIncTests/";
		String keepGoing = "";

		int numRobots = 10;
		int numGoals = 10;
		int numDoors = 0;
		String fn = "";

		String fsBit = "_fsp_";
		String resString = "";

		int numFilesPerFS = 10;

		int[] gridIncs = new int[] { 5, 7, 9, 11, 13, 15, 17, 19, 21, 23/*, 25*/ };
		int[] fsPercentages = new int[] { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };
		int[] fsPercentagesToDo = new int[] { 0, /* 20, */40/* , 80 */, 100 };
		int[] rarr_grid9 = new int[] { 2, 4, 6, 8 };
		int[] garr_grid9 = new int[] { 3, 5, 7, 9 };
		int[] rarr = new int[] { 2, 4 };
		int[] garr = new int[] { 3, 5, 7 };
		int r, g;

		int roughNumFiles = 2 * rarr_grid9.length * garr_grid9.length * fsPercentages.length * numFilesPerFS;
		roughNumFiles += (gridIncs.length - 2) * rarr.length * garr.length * fsPercentagesToDo.length * numFilesPerFS;
		System.out.println("Running " + roughNumFiles + " tests ");
		int testFileNum = 1;
		// for grids 5 & 9 we do everything

		String errorString = "";
		for (int gridNum = 0; gridNum < gridIncs.length; gridNum++) {
			int gridVal = gridIncs[gridNum];
			int[] rnums;
			int[] gnums;
			int[] fsnums;
			if (gridVal < 10) {
				// do everything
				rnums = rarr_grid9;
				gnums = garr_grid9;
				fsnums = fsPercentages;
			} else {
				rnums = rarr;
				gnums = garr;
				fsnums = fsPercentagesToDo;
			}

			for (int rnum : rnums) {
				for (int gnum : gnums) {
					for (int fsnum : fsnums) {
						for (int filenum = 0; filenum < numFilesPerFS; filenum++) {

							fn = fnPrefix + gridVal + "_fsp_" + fsnum + "_" + filenum + "_";

							errorString = fn;
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
							try {
								ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(rnum, numRobots);

								ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(gnum - 1, numGoals - 1);

								int[] rgdf = new int[] { rnum, gnum, numDoors, fsnum, gridVal };
								if (!results.get(fn).containsKey(rgdf))
									results.get(fn).put(rgdf, new ArrayList<float[][]>());
								float[][] resArr = new float[2][6];
								resString += "\nR:" + rnum + "\tG:" + gnum + "\tGrid:" + gridVal;

								System.out.println("***************************************************************");
								System.out.println(testFileNum + " of " + roughNumFiles + " : "
										+ (((float) testFileNum) / ((float) roughNumFiles) * 100.0) + " % " + "\n" + fn
										+ "\nR:" + rnum + "\tG:" + gnum + "\tGrid:" + gridVal);
								System.out.println("***************************************************************");

								errorString += "\nr:" + rnum + " " + robotNumbers.toString() + "\tg:" + gnum + " "
										+ goalNumbers.toString();
								resString += doCompare(dir, fn, rnum, fsnum, gnum, numDoors, resArr, robotNumbers,
										goalNumbers, doDebug);

								results.get(fn).get(rgdf).add(resArr);
								String resname = fnPrefix + "_r" + rnum + "g" + gnum + "grid" + gridVal;

								this.printResults(resSavePlace + resname, hasGridData);
								testFileNum++;
							} catch (Exception e) {
								errors.add(errorString);
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}

						System.out.println("***************************************************************");
						System.out.println("Completed Set: " + rnum + "," + gnum + "," + fsnum + "," + gridVal
								+ "(r,g,fsnum,grid)");
						System.out.println(testFileNum + " of " + roughNumFiles + " : "
								+ (((float) testFileNum) / ((float) roughNumFiles) * 100.0) + " % done");

						System.out.println("***************************************************************");
						System.out.println(resString);
						System.out.println("***************************************************************");

					}
					System.out.println("***************************************************************");
					System.out.println("Completed Set: " + rnum + "," + gnum + "," + gridVal + "(r,g,grid)");
					System.out.println(testFileNum + " of " + roughNumFiles + " : "
							+ (((float) testFileNum) / ((float) roughNumFiles) * 100.0) + " % done");
					System.out.println("***************************************************************");

				}
				System.out.println("***************************************************************");
				System.out.println("Completed Set: " + rnum + "," + gridVal + "(r,grid)");
				System.out.println(testFileNum + " of " + roughNumFiles + " : "
						+ (((float) testFileNum) / ((float) roughNumFiles) * 100.0) + " % done");
				System.out.println("***************************************************************");

			}
			System.out.println("***************************************************************");
			System.out.println("Completed Set: " + gridVal + "(grid)");
			System.out.println(testFileNum + " of " + roughNumFiles + " : "
					+ (((float) testFileNum) / ((float) roughNumFiles) * 100.0) + " % done");
			System.out.println("***************************************************************");

		}

		if (errors.size() > 0) {
			System.out.println("Errors");
			for (int i = 0; i < errors.size(); i++) {
				System.out.println(errors.get(i));
			}
		}
	}

	public static void main(String[] args) {

		// System.out.print("runs");
		new CompareSTAPUSSINVI().run(args);

	}
}
