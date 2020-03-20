package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.Scanner;
import java.util.concurrent.TimeUnit;

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

	boolean donullml = true;
	String resSavePlace =  "/home/fatma/Data/PhD/code/stapussi_prelim/xkcdStyle/data/";
	String testDirBaseLoc =  "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/";

	String logsBaseLoc = "results/logs/";
	String logsModLoc = logsBaseLoc;
	ArrayList<String> errors = new ArrayList<String>();
	HashMap<String, HashMap<int[], ArrayList<float[][]>>> results = new HashMap<String, HashMap<int[], ArrayList<float[][]>>>();


	long alphastarttime = 0; 
	
	long alphadur = 0; 
	
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
		outLog.close();
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
				dir + logsModLoc + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_stapu.txt");
		String mainLogFileName = dir + logsModLoc + fn + "_r" + numRobots + "_g" + numGoals + logSuffix
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
				dir + logsModLoc + fn + "_r" + numRobots + "_g" + numGoals + logSuffix + "_ssi.txt");
		String mainLogFileName = dir + logsModLoc + fn + "_r" + numRobots + "_g" + numGoals + logSuffix
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

		return doCompare(dir, fn, numRobots, numFS, numGoals, numDoors, resArr, robotNumbers, goalNumbers, nullML,
				false, false);
	}

	public String doCompare(String dir, String fn, int numRobots, int numFS, int numGoals, int numDoors,
			float[][] resArr, ArrayList<Integer> robotNumbers, ArrayList<Integer> goalNumbers, boolean nullML,
			boolean justSTAPU, boolean justSSI) {

		Path path = Paths.get(dir + logsModLoc);
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
		float[] ssiRes = null;
		if (!justSTAPU)
			ssiRes = doSSITime(dir, fn, numRobots, numGoals, numFS, numDoors, robotNumbers, goalNumbers, logSuffix,
					nullML, resArr);
		float[] stapuRes = null;
		if (!justSSI)
			stapuRes = doSTAPUTime(dir, fn, numRobots, numGoals, numFS, numDoors, robotNumbers, goalNumbers, logSuffix,
					nullML, resArr);

		if (!justSTAPU)
			resString += "\nSSI Res: " + Arrays.toString(ssiRes);
		if (!justSSI)
			resString += "\nSTAPU Res: " + Arrays.toString(stapuRes);

		if (!justSTAPU && !justSSI) {
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
		}
		return resString;

	}

	public void singleTests(String scenario, String fntodo, String rnums, String gnums) throws Exception {

		doDebug = false;
		this.reallocSSIOnFirstDeadend = true;
		this.reallocSTAPUOnFirstDeadend = false;// true;

		this.doSeqSTAPUPolicy = true;// false;

		this.stapuNoReallocs = false;
		this.ssiNoReallocs = false;

		boolean justSTAPU = false;
		boolean justSSI = false;

		String dir = testDirBaseLoc;// "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/";

		dir = dir + scenario;

		int numRobots = 10;
		int numFS = 31;
		int numGoals = 11;
		int numDoors = 0;

		String fn = fntodo;
		boolean hasGridData = false;
		int gridV = 5;
		String resString = "";

		String[] rnumss = rnums.split(",");
		String[] gnumss = gnums.split(",");

		int r = rnumss.length;
		int g = gnumss.length + 1;
		boolean doRandomRG = false;
		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		ArrayList<Integer> robotNumbers;// = new ArrayList<Integer>();// generateListOfRandomNumbers(r, numRobots);
		ArrayList<Integer> goalNumbers;// = new ArrayList<Integer>(); // generateListOfRandomNumbers(g - 1, numGoals -
										// 1);
										// //-1 cuz the last one is always a safety
		if (doRandomRG) {
			robotNumbers = generateListOfRandomNumbers(r, numRobots);
			goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1);

			new ArrayList<Integer>(); //
		} else {
			robotNumbers = new ArrayList<Integer>();
			goalNumbers = new ArrayList<Integer>();
//			int[] rnums = new int[] {9, 5, 2, 7, 8, 1, 0, 3}; 
//			int[] gnums = new int[] {3, 6, 5, 1}; 
//			r = rnums.length; 
//			g = gnums.length; 
			for (String rnum : rnumss) {
				int rnumint = Integer.parseInt(rnum);
				robotNumbers.add(rnumint);
			}
			for (String gnum : gnumss) {
				int gnumint = Integer.parseInt(gnum);
				goalNumbers.add(gnumint);
			}
// 			robotNumbers.add(6);
//			robotNumbers.add(4);
//			robotNumbers.add(1);
//			robotNumbers.add(0);
//			goalNumbers.add(0);
//			goalNumbers.add(5);
//			goalNumbers.add(1);
//			goalNumbers.add(7);
//			goalNumbers.add(3);
//			goalNumbers.add(2);
//			goalNumbers.add(4);
//			goalNumbers.add(6);
		}
		int[] rgdf;
		if (hasGridData)
			rgdf = new int[] { r, g, numDoors, numFS, gridV };
		else
			rgdf = new int[] { r, g, numDoors, numFS };
		if (!results.get(fn).containsKey(rgdf))
			results.get(fn).put(rgdf, new ArrayList<float[][]>());
		float[][] resArr = new float[2][6];
		resString += "\nR:" + r + "\tG:" + g;

		resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers, true, justSTAPU,
				justSSI);
		results.get(fn).get(rgdf).add(resArr);

		printResults(hasGridData);
	}

	public void singleTests() throws Exception {
		doDebug = true;//false;
		this.reallocSSIOnFirstDeadend = true;
		this.reallocSTAPUOnFirstDeadend = false;// true;

		this.doSeqSTAPUPolicy = true;// false;

		this.stapuNoReallocs = false;
		this.ssiNoReallocs = false;

		boolean justSTAPU = false;//true;
		boolean justSSI = true;//false;

		String dir = testDirBaseLoc;//
//		single warehouse/ depotShelf_r10_g10_fs111_fsp_90.0_0_ 3,9,6,4,1,0,8,2 4,9,5,6

//		dir = dir + "gridIncWithRepeats/";// "gridIncTests/";
//		dir = dir + "warehouse/";
		dir = dir + "officedoors/";
		int numRobots = 10;
		int numFS = 31;
		int numGoals = 11;
		int numDoors = 3;

		String fn = "r10_g10_a1_grid_11_fsp_100_2_";
		fn = "sh2_r10_g10_a1_fs0_fsp_0_7_d_3_";
//		sh2_r10_g10_a1_fs0_fsp_0_7_d_3_
//		r:4 [8, 4, 1, 5] g:5 [8, 1, 0, 3]


		boolean hasGridData = false;// true;
		int gridV = 5;
		String resString = "";
		int r = 4; // numRobots;
		int g = 7;// 9;// 3;//numGoals;
		boolean doRandomRG = false;
		if (!results.containsKey(fn))
			results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
		ArrayList<Integer> robotNumbers;// = new ArrayList<Integer>();// generateListOfRandomNumbers(r, numRobots);
		ArrayList<Integer> goalNumbers;// = new ArrayList<Integer>(); // generateListOfRandomNumbers(g - 1, numGoals -
										// 1);
										// //-1 cuz the last one is always a safety
		if (doRandomRG) {
			robotNumbers = generateListOfRandomNumbers(r, numRobots);
			goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1);

			new ArrayList<Integer>(); //
		} else {
			robotNumbers = new ArrayList<Integer>();
			goalNumbers = new ArrayList<Integer>();
			int[] rnums = new int[] {8, 4, 1, 5};
			int[] gnums = new int[] { 8, 1, 0, 3};
			r = rnums.length;
			g = gnums.length + 1;
			for (int rnum : rnums)
				robotNumbers.add(rnum);
			for (int gnum : gnums)
				goalNumbers.add(gnum);

		}
		int[] rgdf;
		if (hasGridData)
			rgdf = new int[] { r, g, numDoors, numFS, gridV };
		else
			rgdf = new int[] { r, g, numDoors, numFS };
		if (!results.get(fn).containsKey(rgdf))
			results.get(fn).put(rgdf, new ArrayList<float[][]>());
		float[][] resArr = new float[2][6];
		resString += "\nR:" + r + "\tG:" + g;

		resString += doCompare(dir, fn, r, numFS, g, numDoors, resArr, robotNumbers, goalNumbers, true, justSTAPU,
				justSSI);
		results.get(fn).get(rgdf).add(resArr);

		printResults(hasGridData);

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
				"wvar", "strangehouse", "grid", "warehousefree", "gfixed", "warehousedoors","smallwhdoors" };

		this.doSeqSTAPUPolicy = true;// false;
		this.reallocSSIOnFirstDeadend = true;
		this.reallocSTAPUOnFirstDeadend = false;
		this.ssiNoReallocs = false;
		this.stapuNoReallocs = false;
		alphastarttime = System.currentTimeMillis();
		System.out.println("Running STAPU SSI comparision scripts!");
		try {
			String option = args[0];

//			if (option.contains(options[0]))
//				System.out.println("Not implemented");
//				runRobots("");
//			else if (option.contains(options[1]))
//				System.out.println("Not implemented");
//				runDoors();
//			else if (option.contains(options[2]))
//				System.out.println("Not implemented");
//			// runFSGR();
//			// // runFSGoalsOnly();
//			else if (option.contains(options[3])) {
//
////				runFS();
//				System.out.println("Not implemented");
//
//			} else 
			if (option.contains(options[4])) {
				if (args.length > 1) {
					String scen = args[1];
					String fn = args[2];
					String rnums = args[3];
					String gnums = args[4];
					singleTests(scen, fn, rnums, gnums);
				} else {
					singleTests();
				}
			} else if (option.contains(options[5])) {
				String grfs = "all";
				if (args.length > 1) {
					grfs = args[1];
				}
				runWarehouse(0, "", grfs);
			} else if (option.contains(options[6])) {
				String grfs = "all";
				if (args.length > 1) {
					grfs = args[1];
				}
				runWarehouse(1, "", grfs);
			} else if (option.contains(options[7]))
				System.out.println("Not implemented");
//				this.runRobotsVariations();
			else if (option.contains(options[8]))
				System.out.println("Not implemented");
//				this.runWarehouseVariations();
			else if (option.contains(options[9])) {
				String torun = "all";
				if (args.length > 1) {
					torun = args[1];
				}
				if (torun.contains("doors"))
					runStrangeHouseDoors();
				else
					runStrangeHouse(torun);

			} else if (option.contains(options[10])) {
				String gridVal = "all";
				if (args.length > 1) {
					gridVal = args[1];
				}
				runGridStates(gridVal);
			} else if (option.contains(options[11])) {
				String grfs = "all";
				if (args.length > 1) {
					grfs = args[1];
				}
				runWarehouse(2, "", grfs);
			} else if (option.contains(options[12])) {
				String gridVal = "2";
				if (args.length > 1) {
					gridVal = args[1];
				}
				runGrid11Fixed(gridVal);
			} else if (option.contains(options[13])) {
				
				runWarehouseDoors(1, "");
			}else if (option.contains(options[14])) {
				
				runWarehouseDoors(3, "");
			} 
			else {
				System.out.println("invalid option, options are: " + Arrays.toString(options));
				System.out.println("Format for " + options[9] + ": " + options[9] + " ," + options[9] + " r,"
						+ options[9] + " fs," + options[9] + " g");
				System.out.println("Format for " + options[5] + "and " + options[6] + ": " + options[6] + " ,"
						+ options[6] + " seq,r,9,g,5,fs,10.0," + options[6] + " r," + options[6] + " fs," + options[6]
						+ " g");
				System.out.println("Format for " + options[4] + ": " + options[4] + " ," + options[4]
						+ " scenariodir fn r1,r2,..rn g1,g2,..gn");
			}
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private void runWarehouse(int tnum, String fnSuffix, String grfs) {
		String fnPrefix = "shelfDepot_r10_g10_";

		int[] fsShelfDepot;
		String[] fsPercentageStrings;
		if (grfs.contains("seq")) {
			// theformat for seq is
			// seq,r,10,g,10,fs,10
			if (tnum == 0) {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111, 123 };
				fsPercentageStrings = new String[] { "0.0", "11.0", "20.0", "30.0", "41.0", "50.0", "60.0", "71.0",
						"80.0", "90.0", "100" };
			} else if (tnum == 1) {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111 };
				fsPercentageStrings = new String[] { "0.0", "11.0", "20.0", "30.0", "41.0", "50.0", "60.0", "71.0",
						"80.0", "90.0" };
			} else {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111, 123 };
				fsPercentageStrings = new String[] { "0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100" };
			}

		} else if (grfs.contains("fs")) {
			if (tnum == 0) {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111, 123 };
				fsPercentageStrings = new String[] { "0.0", "11.0", "20.0", "30.0", "41.0", "50.0", "60.0", "71.0",
						"80.0", "90.0", "100" };
			} else if (tnum == 1) {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111 };
				fsPercentageStrings = new String[] { "0.0", "11.0", "20.0", "30.0", "41.0", "50.0", "60.0", "71.0",
						"80.0", "90.0" };
			} else {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111, 123 };
				fsPercentageStrings = new String[] { "0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100" };
			}

		} else if (grfs.contains("r")) {
			if (tnum == 0) {
				fsShelfDepot = new int[] { 0, 62, 111 };
				fsPercentageStrings = new String[] { "0.0", "50.0", "90.0" };
			} else if (tnum == 1) {
				fsShelfDepot = new int[] { 0, 62, 111 };
				fsPercentageStrings = new String[] { "0.0", "50.0", "90.0" };
			} else {
				fsShelfDepot = new int[] { 0, 62, 111 };
				fsPercentageStrings = new String[] { "0", "50", "90" };
			}

		} else if (grfs.contains("g")) {
			if (tnum == 0) {
				fsShelfDepot = new int[] { /* 0, 62, */ 111 };
				fsPercentageStrings = new String[] { /* "0.0", "50.0", */ "90.0" };
			} else if (tnum == 1) {
				fsShelfDepot = new int[] { /* 0, 62, */ 111 };
				fsPercentageStrings = new String[] { /* "0.0", "50.0", */ "90.0" };
			} else {
				fsShelfDepot = new int[] { 0/* , 62, 111 */ };
				fsPercentageStrings = new String[] { "0"/* , "50", */ /* "90" */ };
			}
		} else {
			if (tnum == 0) {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111, 123 };
				fsPercentageStrings = new String[] { "0.0", "11.0", "20.0", "30.0", "41.0", "50.0", "60.0", "71.0",
						"80.0", "90.0", "100" };
			} else if (tnum == 1) {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111 };
				fsPercentageStrings = new String[] { "0.0", "11.0", "20.0", "30.0", "41.0", "50.0", "60.0", "71.0",
						"80.0", "90.0" };
			} else {
				fsShelfDepot = new int[] { 0, 13, 25, 37, 50, 62, 74, 87, 99, 111, 123 };
				fsPercentageStrings = new String[] { "0", "10", "20", "30", "40", "50", "60", "70", "80", "90", "100" };
			}
		}
		String fsBit = "fs";

		if (tnum == 0)
			runWarehouse(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix, grfs);
		fnPrefix = "depotShelf_r10_g10_";
		if (tnum == 1)
			runWarehouse(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix, grfs);
		fnPrefix = "whfree_r10_g10_a1_";
		if (tnum == 2)
			runWarehouse(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix, grfs);
	}

	private void fileText(String fn, int maxFiles, int testNum) {
		alphadur = System.currentTimeMillis()-alphastarttime; 
		long avgTime = alphadur/testNum; 
		long expTime = avgTime* (maxFiles-testNum);
		if(expTime < 0)
			expTime = expTime*-1;
		System.out.println("***************************************************************");
		System.out.println("Elapsed Time "+TimeUnit.MINUTES.convert(alphadur, TimeUnit.MILLISECONDS)+" min "); 
		System.out.println("Expected End Time for remaining tests "+TimeUnit.MINUTES.convert(expTime, TimeUnit.MILLISECONDS)); 
		System.out.println("Average test time "+TimeUnit.MINUTES.convert(avgTime, TimeUnit.MILLISECONDS)); 
		System.out.println(testNum + " / " + maxFiles + ": " + (((float) testNum / (float) maxFiles) * 100.0));
		System.out.println(fn);
		System.out.println("***************************************************************");

	}


	private void runWarehouse(String fnPrefix, int[] fsShelfDepot, String[] fspStrings, String fsBit, String fnSuffix,
			String grfs) {
		boolean hasGridData = false;
		String dir = testDirBaseLoc + "warehouse/";

		int numRobots = 10;
		int numGoals = 11;
		int numDoors = 0;
		String fn = "";

		String resString = "";

		String currentErrorfn = "";
		int numFilesPerFS = 10;
		int fileForFSstart = 0;
		String errorString = "";
		int[] rarr = null;
		int[] garr = null;
		if (grfs.contains("seq")) {
			// theformat for seq is
			// seq,r,10,g,10,fs,10
			String[] stuff = grfs.split(",");
			logsModLoc = logsBaseLoc + "seq/";
			// r at loc 1
			// rval at loc 2
			// g at loc 3
			// gval at loc 4
			// fs at loc 5 s
			// fsval at loc 6
			// just checking
			String rval = "4";
			String gval = "5";
			String fsval = "50";
			String fspString = "50.0";
			int fsvalint = Integer.parseInt(fsval);
			if (stuff[0].contentEquals("seq")) {
				if (stuff[1].contentEquals("r")) {
//					int gridInt = Integer.parseInt(gridValString);
					rval = stuff[2];
					if (stuff[3].contentEquals("g")) {
						gval = stuff[4];
						if (stuff[5].contentEquals("fs")) {
							fspString = stuff[6];
							if (stuff.length > 7) {
								if (stuff[7].contentEquals("fn")) {
									fileForFSstart = Integer.parseInt(stuff[8]);
									System.out.println(fileForFSstart);
								}
							}
						}
					}
				}
			}
			int rvalint = Integer.parseInt(rval);
			int gvalint = Integer.parseInt(gval);

			for (int i = 0; i < fsShelfDepot.length; i++) {
				if (fspStrings[i].contentEquals(fspString)) {
					fsvalint = fsShelfDepot[i];
					break;
				}
			}

			rarr = new int[] { rvalint };
			garr = new int[] { gvalint };
			fsShelfDepot = new int[] { fsvalint };
			fspStrings = new String[] { fspString };
			fnSuffix = "seq_r" + rval + "g" + gval + "fsp" + fspString;

		} else if (grfs.contains("fs")) {
			logsModLoc = logsBaseLoc + "fs/";
			rarr = new int[] { 4 };
			garr = new int[] { 5 };
			fnSuffix = "allfs";
		} else if (grfs.contains("r")) {
			logsModLoc = logsBaseLoc + "r/";
			rarr = new int[] { /* 2, 4, 6, */ 8 };
			garr = new int[] { 5 };
			fnSuffix = "allr";
		} else if (grfs.contains("g")) {
			logsModLoc = logsBaseLoc + "g/";
			rarr = new int[] { 4 };
			garr = new int[] { /* 3, 5, 7, */9 };
			fnSuffix = "allg";
		} else {
			logsModLoc = logsBaseLoc + "all/";

			rarr = new int[] { 2, 4, 6, 8, 10 };
			garr = new int[] { 3, 5, 7, 9, 11 };
		}

		int r, g;
		int maxFiles = rarr.length * garr.length * fsShelfDepot.length * numFilesPerFS;
		int testFileNum = 1;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr.length; j++) {
				g = garr[j];

				for (int fsNum = 0; fsNum < fsShelfDepot.length; fsNum++) {

					int fs = fsShelfDepot[fsNum];

					for (int fileForFS = fileForFSstart; fileForFS < numFilesPerFS; fileForFS++) {

						fn = fnPrefix + fsBit + fs + "_fsp_" + fspStrings[fsNum] + "_" + fileForFS + "_";

						errorString = fn;
						currentErrorfn = fn;
						fileText(fn, maxFiles, testFileNum);
						if (!results.containsKey(fn))
							results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
						try {
							ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
							ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1 cuz
																												// the
																												// last
																												// one
																												// is
																												// always
																												// safety
							int[] rgdf = new int[] { r, g, numDoors, fs };
							if (!results.get(fn).containsKey(rgdf))
								results.get(fn).put(rgdf, new ArrayList<float[][]>());
							float[][] resArr = new float[2][6];
							resString += "\nR:" + r + "\tG:" + g;

							errorString += "\nr:" + r + " " + robotNumbers.toString() + " g:" + g + " "
									+ goalNumbers.toString();

							resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers,
									donullml);
							results.get(fn).get(rgdf).add(resArr);
							String resname = "wh_r" + r + "g" + g + "fs" + fs + "_" + fnPrefix + fnSuffix;
							this.printResults(resSavePlace + resname, hasGridData);
							testFileNum++;
						} catch (Exception e) {
							System.out.print("Error");
							System.out.println(errorString);
							errors.add(errorString);
							if (results.containsKey(currentErrorfn))
								results.remove(currentErrorfn);
							writeErrors(resSavePlace + "wh_r" + r + "g" + g + "fs" + fs + "_" + fnPrefix + fnSuffix
									+ "_errors.txt");
							e.printStackTrace();

						}
					}
					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");

					// }
					// }

				}

				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				fileText("", maxFiles, testFileNum);
				if (this.results.size() > 0) {
					String resname = "wh_r" + r + "g" + g + "_" + fnPrefix + fnSuffix;
					this.printResults(resSavePlace + resname, hasGridData);
					// printResults();
				}
			}
		}

		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");
		String resname = "wh_" + fnPrefix + fnSuffix;
		if (this.results.size() > 0) {

			this.printResults(resSavePlace + resname, hasGridData);
			printResults(hasGridData);
		}
		if (errors.size() > 0)

		{
			writeErrors(resSavePlace + resname + "_errors.txt");
			System.out.println("Errors");
			for (int i = 0; i < errors.size(); i++) {
				System.out.println(errors.get(i));
			}
		}
	}

	private void runWarehouseDoors(int tnum, String fnSuffix) {
		String fnPrefix = "shelfDepot_r10_g10_a1_";

		int[] fsShelfDepot;
		String[] fsPercentageStrings;

		fsShelfDepot = new int[] { /* 0, 62, */ 110 };
		fsPercentageStrings = new String[] { /* "0", "50", */ "90" };

		String fsBit = "fs";

		if (tnum == 0)
			runWarehouseDoors(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix);
		fnPrefix = "depotShelf_r10_g10_a1_";
		if (tnum == 1)
			runWarehouseDoors(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix);
		fnPrefix = "whfree_r10_g10_a1_";
		if (tnum == 2)
			runWarehouseDoors(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix);
		if(tnum==3) {
			fsShelfDepot = new int[] { /* 0, 62, */ 79 };
			fnPrefix = "smallerShelfDepot_r10_g10_a1_";
			runWarehouseDoors(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix);
	}
		if(tnum==4) {
			fsShelfDepot = new int[] { /* 0, 62, */ 79 };
			fnPrefix = "smallerDepotShelf_r10_g10_a1_";
			runWarehouseDoors(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix);
	}
		
		if(tnum==5) {
			fsShelfDepot = new int[] { /* 0, 62, */ 79 };
			fnPrefix = "smallerWarehouseFree_r10_g10_a1_";
			runWarehouseDoors(fnPrefix, fsShelfDepot, fsPercentageStrings, fsBit, fnSuffix);
	}
		}

	private void runWarehouseDoors(String fnPrefix, int[] fsShelfDepot, String[] fspStrings, String fsBit,
			String fnSuffix) {
		boolean hasGridData = false;
		String dir = testDirBaseLoc + "warehousedoors/";

		if(fnPrefix.contains("smaller"))
			dir = testDirBaseLoc + "smallerwhdoors/";
		int numRobots = 10;
		int numGoals = 11;
		int[] doorsarray = new int[] { 1, 2, 3 ,4,5,6};
		String fn = "";

		String resString = "";

		String currentErrorfn = "";
		int numFilesPerFS = 10;
		int fileForFSstart = 0;
		String errorString = "";
		int[] rarr = new int[] { 4 };
		int[] garr = new int[] { 5 };

		int r, g;
		int maxFiles = rarr.length * garr.length * fsShelfDepot.length * numFilesPerFS * doorsarray.length;
		int testFileNum = 1;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr.length; j++) {
				g = garr[j];
				for (int numDoors : doorsarray) {
				for (int fsNum = 0; fsNum < fsShelfDepot.length; fsNum++) {

					int fs = fsShelfDepot[fsNum];

					for (int fileForFS = fileForFSstart; fileForFS < numFilesPerFS; fileForFS++) {

						
							fn = fnPrefix + fsBit + fs + "_fsp_" + fspStrings[fsNum] + "_" + fileForFS + "_d_"
									+ numDoors + "_";

							errorString = fn;
							currentErrorfn = fn;
							fileText(fn, maxFiles, testFileNum);
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
							try {
								ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
								ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1
																													// cuz
																													// the
																													// last
																													// one
																													// is
																													// always
																													// safety
								int[] rgdf = new int[] { r, g, numDoors, fs };
								if (!results.get(fn).containsKey(rgdf))
									results.get(fn).put(rgdf, new ArrayList<float[][]>());
								float[][] resArr = new float[2][6];
								resString += "\nR:" + r + "\tG:" + g;

								errorString += "\nr:" + r + " " + robotNumbers.toString() + " g:" + g + " "
										+ goalNumbers.toString();

								resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers,
										donullml);
								results.get(fn).get(rgdf).add(resArr);
								String resname = "wh_r" + r + "g" + g + "fs" + fs + "_d_" + numDoors + "_" + "_"
										+ fnPrefix + fnSuffix;
								this.printResults(resSavePlace + resname, hasGridData);
								testFileNum++;
							} catch (Exception e) {
								System.out.print("Error");
								System.out.println(errorString);
								errors.add(errorString);
								if (results.containsKey(currentErrorfn))
									results.remove(currentErrorfn);
								writeErrors(resSavePlace + "wh_r" + r + "g" + g + "fs"  + "_" + fs+ "_d_" + numDoors
										+ "_" + fnPrefix + fnSuffix + "_errors.txt");
								e.printStackTrace();

							}
						}
					}
					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");

					// }
					// }

				}

				System.out.println("***************************************************************");
				System.out.println(resString);
				System.out.println("***************************************************************");
				fileText("", maxFiles, testFileNum);
				if (this.results.size() > 0) {
					String resname = "wh_r" + r + "g" + g + "_" + fnPrefix + fnSuffix;
					this.printResults(resSavePlace + resname, hasGridData);
					// printResults();
				}
			}
		}

		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");
		String resname = "wh_" + fnPrefix + fnSuffix;
		if (this.results.size() > 0) {

			this.printResults(resSavePlace + resname, hasGridData);
			printResults(hasGridData);
		}
		if (errors.size() > 0)

		{
			writeErrors(resSavePlace + resname + "_errors.txt");
			System.out.println("Errors");
			for (int i = 0; i < errors.size(); i++) {
				System.out.println(errors.get(i));
			}
		}
	}

	private void runStrangeHouseDoors() {
		boolean hasGridData = false;
		String dir = testDirBaseLoc + "officedoors/";

		String fnPrefix = "sh2_r10_g10_a1_";
		String fsBit = "fs";
		String fspBit = "_fsp_";
		int numRobots = 10;
		int numGoals = 11;
		int[] doorsarray = new int[] { 1, 2, 3/* , 4,5 */ };

		String fn = "";

		String resString = "";

		String fnSuffix = "";

		int numFilesPerFS = 10;
		int fileForFSstart = 0;
		String errorString = "";
		String errorfn = "";
		int[] rarr = null;
		int[] garr = null;
		int[] allfs = null;
		int[] allfsp = null;
		rarr = new int[] { 4 };
		garr = new int[] { 5 };
		allfs = new int[] { 0/* , 34 */, 60 };
		allfsp = new int[] { 0/* , 50 */, 90 };
		fnSuffix = "doors";

		int r, g;
		int maxFiles = rarr.length * garr.length * allfs.length * numFilesPerFS * doorsarray.length;
		int testFileNum = 1;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr.length; j++) {
				g = garr[j];

				for (int fsi = 0; fsi < allfs.length; fsi++) {
					int fs = allfs[fsi];
					int fsp = allfsp[fsi];

					for (int fileForFS = fileForFSstart; fileForFS < numFilesPerFS; fileForFS++) {
						for (int numDoors : doorsarray) {
							fn = fnPrefix + fsBit + fs + fspBit + fsp + "_" + +fileForFS + "_d_" + numDoors + "_";

							errorfn = fn;
							errorString = fn;
							fileText(fn, maxFiles, testFileNum);
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
							try {
								ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(r, numRobots);
								ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(g - 1, numGoals - 1); // -1
																													// cuz
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
								errorString += "\nr:" + r + " " + robotNumbers.toString() + " g:" + g + " "
										+ goalNumbers.toString();

								resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers,
										donullml);
								results.get(fn).get(rgdf).add(resArr);
								String resname = "sh_r" + r + "g" + g + "fs" + fs + "_d" + numDoors + "_" + fnPrefix
										+ fnSuffix;
								this.printResults(resSavePlace + resname, hasGridData);
								testFileNum++;
							} catch (Exception e) {
								errorString += "\n" + getStackTraceString(e);
								errors.add(errorString);
								writeErrors(resSavePlace + "sh_r" + r + "g" + g + "fs" + fs + "_d" + numDoors + "_"
										+ fnPrefix + fnSuffix + "_errors.txt");
								if (results.containsKey(errorfn))
									// remove this from the results
									results.remove(errorfn);
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}
					}

					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");
					fileText("", maxFiles, testFileNum);
				}

				String resname = "sh_r" + r + "g" + g + "_" + fnPrefix + fnSuffix;
				this.printResults(resSavePlace + resname, hasGridData);
				fileText("", maxFiles, testFileNum);
			}
		}

		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");
		fileText("", maxFiles, testFileNum);

		String resname = "sh_doors" + fnPrefix + fnSuffix;
		this.printResults(resSavePlace + resname, hasGridData);
		printResults(hasGridData);
		if (errors.size() > 0)

		{
			writeErrors(resSavePlace + resname + "_errors.txt");
			System.out.println("Errors");
			for (int i = 0; i < errors.size(); i++) {
				System.out.println(errors.get(i));
			}
		}
	}

	private void runStrangeHouse(String grfs) {
		boolean hasGridData = false;
		String dir = testDirBaseLoc + "strangehouse/";

		String fnPrefix = "sh2_r10_g10_";
		String fsBit = "fs";
		String fspBit = "_fsp_";
		int numRobots = 10;
		int numGoals = 11;
		int numDoors = 0;
		String fn = "";

		String resString = "";

		String fnSuffix = "";

		int numFilesPerFS = 10;
		int fileForFSstart = 0;
		String errorString = "";
		String errorfn = "";
		int[] rarr = null;
		int[] garr = null;
		int[] allfs = null;
		int[] allfsp = null;
		if (grfs.contains("fs")) {
			logsModLoc = logsBaseLoc + "fs/";
			rarr = new int[] { 4 };
			garr = new int[] { 5 };
			allfs = new int[] { 0, 7, 14, 20, 27, 34, 40, 47, 53, 60, 66 };
			allfsp = new int[] { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };
			fnSuffix = "allfs";
		} else if (grfs.contains("r")) {
			logsModLoc = logsBaseLoc + "r/";
			rarr = new int[] { 2, 4, 6, 8 };
			garr = new int[] { 5 };
			allfs = new int[] { 0, 34, 60 };
			allfsp = new int[] { 0, 50, 90 };
			fnSuffix = "allr";
		} else if (grfs.contains("g")) {
			logsModLoc = logsBaseLoc + "g/";
			rarr = new int[] { 4 };
			garr = new int[] { 3, 5, 7, 9 };
			allfs = new int[] { 60 };// { 0, 34, 60 };
			allfsp = new int[] { 90 };// { 0, 50, 90 };
			fnSuffix = "allg";
		} else {
			logsModLoc = logsBaseLoc + "all/";
			rarr = new int[] { 2, 4, 6, 8, 10 };
			garr = new int[] { 3, 5, 7, 9, 11 };
			allfs = new int[] { 0, 7, 14, 20, 27, 34, 40, 47, 53, 60, 66 };
			allfsp = new int[] { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };
		}

		int r, g;
		int maxFiles = rarr.length * garr.length * allfs.length * numFilesPerFS;
		int testFileNum = 1;
		for (int i = 0; i < rarr.length; i++) {
			r = rarr[i];
			for (int j = 0; j < garr.length; j++) {
				g = garr[j];

				for (int fsi = 0; fsi < allfs.length; fsi++) {
					int fs = allfs[fsi];
					int fsp = allfsp[fsi];

					for (int fileForFS = fileForFSstart; fileForFS < numFilesPerFS; fileForFS++) {
						fn = fnPrefix + fsBit + fs + fspBit + fsp + "_" + +fileForFS + "_";
						errorfn = fn;
						errorString = fn;
						fileText(fn, maxFiles, testFileNum);
						if (!results.containsKey(fn))
							results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
						try {
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
							errorString += "\nr:" + r + " " + robotNumbers.toString() + " g:" + g + " "
									+ goalNumbers.toString();

							resString += doCompare(dir, fn, r, fs, g, numDoors, resArr, robotNumbers, goalNumbers,
									donullml);
							results.get(fn).get(rgdf).add(resArr);
							String resname = "sh_fs_r" + r + "g" + g + "fs" + fs + "_" + fnPrefix + fnSuffix;
							this.printResults(resSavePlace + resname, hasGridData);
							testFileNum++;
						} catch (Exception e) {
							errorString += "\n" + getStackTraceString(e);
							errors.add(errorString);
							writeErrors(resSavePlace + "sh_fs_r" + r + "g" + g + "fs" + fs + "_" + fnPrefix + fnSuffix
									+ "_errors.txt");
							if (results.containsKey(errorfn))
								// remove this from the results
								results.remove(errorfn);
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}

					System.out.println("***************************************************************");
					System.out.println(resString);
					System.out.println("***************************************************************");
					fileText("", maxFiles, testFileNum);
				}

				String resname = "sh_r" + r + "g" + g + "_" + fnPrefix + fnSuffix;
				this.printResults(resSavePlace + resname, hasGridData);
				fileText("", maxFiles, testFileNum);
			}
		}

		System.out.println("***************************************************************");
		System.out.println(resString);
		System.out.println("***************************************************************");
		fileText("", maxFiles, testFileNum);

		String resname = "sh_" + fnPrefix + fnSuffix;
		this.printResults(resSavePlace + resname, hasGridData);
		printResults(hasGridData);
		if (errors.size() > 0)

		{
			writeErrors(resSavePlace + resname + "_errors.txt");
			System.out.println("Errors");
			for (int i = 0; i < errors.size(); i++) {
				System.out.println(errors.get(i));
			}
		}
	}

	private void runGridStates(String gridValString) {

		boolean hasGridData = true;
		String fnPrefix = "r10_g10_a1_grid_";// {/"r10_g10_a2_grid_11_li_rg_","r10_g10_a2_grid_11_ri_lg_"};//
												// "r10_g10_grid_";
		String dir = testDirBaseLoc + "gridIncWithRepeats/";// "gridIncTests/";

		String extraText = "";
		int numRobots = 10;
		int numGoals = 11;
		int numDoors = 0;
		String fn = "";

		boolean dorobotsonly = false;
		boolean dogoalsonly = false;
		int numFilesPerFS = 10;

		int[] gridIncs = null;

		int[] fsPercentages = new int[] { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };

		int[] fsPercentagesToDo = new int[] { 0, /* 20, */50/* , 80 */, 100 };
		int[] rarr_grid9 = new int[] { 2, 4 };
		int[] garr_grid9 = new int[] { 3, 5, 7 };
		int[] rarr = new int[] { 2, 4, 6, 8 };
		int[] garr = new int[] { 3, 5, 7, 9 };
		int roughNumFiles = 0;
		int[] rnums = null;
		int[] gnums = null;
		int[] fsnums = null;

		boolean doall = false;
		if (gridValString.contains("all")) {
			logsModLoc = logsBaseLoc + "all/";
			gridIncs = new int[] { 5, 8, 11, 14, 17, 20, 23 };
			doall = true;
			rnums = new int[] { 4 };
			gnums = new int[] { 5 };
			fsnums = new int[] { 90 };
			extraText = "r4_g5_fs90_allgrid";
			roughNumFiles = rnums.length * gnums.length * fsnums.length * numFilesPerFS * gridIncs.length;
			System.out.println("Running " + roughNumFiles + " tests ");
//			roughNumFiles = 2 * rarr_grid9.length * garr_grid9.length * fsPercentages.length * numFilesPerFS;
//			roughNumFiles += (gridIncs.length - 2) * rarr.length * garr.length * fsPercentagesToDo.length
//					* numFilesPerFS;
//			System.out.println("Running " + roughNumFiles + " tests ");
		} else {

			if ((gridValString.contains("r"))) {
				logsModLoc = logsBaseLoc + "r/";
				dorobotsonly = true;
				gridValString = gridValString.replace("r", "");
			} else {
				if (gridValString.contains("g")) {
					logsModLoc = logsBaseLoc + "g/";
					dogoalsonly = true;
					gridValString = gridValString.replace("g", "");
				} else {
					logsModLoc = logsBaseLoc + "fs/";
				}
			}
			int gridInt = Integer.parseInt(gridValString);
			gridIncs = new int[] { gridInt };
			logsModLoc = logsModLoc + gridInt + "/";
		}
		int fnnumStart = 0;

		int testFileNum = 1;
		// for grids 5 & 9 we do everything

		String errorString = "";
		String errorfn = "";
		String resString;
		for (int gridNum = 0; gridNum < gridIncs.length; gridNum++) {

			int gridVal = gridIncs[gridNum];

			if (gridVal == 11 && !doall) {

				if (dorobotsonly) {
					rnums = new int[] { 2, 4, 6, 8/* , 10 */ };
//						rnums = new int[] { 10 };
					gnums = new int[] { 5 };
					fsnums = new int[] { 0, 50, 90 };
//						fsnums = new int[] { 100 };
					fnnumStart = 0;
					extraText = "rall_g5_fs0_50_90";
//						extraText = "rall_g5_fs0_50_100_r10_fs100_5onwards";
				} else if (dogoalsonly) {
					rnums = new int[] { 4 };
					gnums = new int[] { 9 /* 3, 5, 7,9/* /* , 11 */ };
					fsnums = new int[] { 90 };// { 0, 50, 90 };
					extraText = "r4_gall_fs90";
				} else {
					rnums = new int[] { 4 };
					gnums = new int[] { 5 };
					fsnums = fsPercentages;
					extraText = "r4_g5_fsall";
				} // fnPrefix = "grid_r4_g5_fsall_";
			} else {
				if (doall) {
					rnums = new int[] { 4 };
					gnums = new int[] { 5 };
					fsnums = new int[] { 90 };
					extraText = "r4_g5_fs90_allgrid";

				} else {
					rnums = rarr;
					gnums = garr;
					fsnums = fsPercentagesToDo;
				}
			}

			if (!gridValString.contains("all")) {
				roughNumFiles = rnums.length * gnums.length * fsnums.length * numFilesPerFS;
				System.out.println("Running " + roughNumFiles + " tests ");
			}

			for (int rnum : rnums) {
				for (int gnum : gnums) {

					for (int fsnum : fsnums) {

						for (int filenum = fnnumStart; filenum < numFilesPerFS; filenum++) {

							fn = fnPrefix + gridVal + "_fsp_" + fsnum + "_" + filenum + "_";
							resString = "";
							errorString = fn;
							errorfn = fn;
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

								fileText(fn + "\nR:" + rnum + "\tG:" + gnum + "\tGrid:" + gridVal, roughNumFiles,
										testFileNum);

								errorString += "\nr:" + rnum + " " + robotNumbers.toString() + "\tg:" + gnum + " "
										+ goalNumbers.toString();
								resString += doCompare(dir, fn, rnum, fsnum, gnum, numDoors, resArr, robotNumbers,
										goalNumbers, donullml);

								results.get(fn).get(rgdf).add(resArr);
								String resname = fnPrefix + extraText + "_r" + rnum + "g" + gnum + "grid" + gridVal;

								this.printResults(resSavePlace + resname, hasGridData);
								testFileNum++;
							} catch (Exception e) {
								errorString += "\n" + getStackTraceString(e);
								errors.add(errorString);
								writeErrors(resSavePlace + fn + Arrays.toString(rnums) + Arrays.toString(gnums)
										+ Arrays.toString(fsnums) + extraText + "_errors.txt");
								if (results.containsKey(errorfn))
									// remove this from the results
									results.remove(errorfn);
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}
						fileText("Completed Set: " + rnum + "," + gnum + "," + fsnum + "," + gridVal
								+ "(r,g,fsnum,grid)", roughNumFiles, testFileNum);

					}
					fileText("Completed Set: " + rnum + "," + gnum + "," + gridVal + "(r,g,grid)", roughNumFiles,
							testFileNum);

				}
				fileText("Completed Set: " + rnum + "," + gridVal + "(r,grid)", roughNumFiles, testFileNum);

			}
			fileText("Completed Set: " + gridVal + "(grid)", roughNumFiles, testFileNum);

		}

		if (errors.size() > 0)

		{
			writeErrors(resSavePlace + fn + Arrays.toString(rnums) + Arrays.toString(gnums) + Arrays.toString(fsnums)
					+ extraText + "_errors.txt");
			System.out.println("Errors");
			for (int i = 0; i < errors.size(); i++) {
				System.out.println(errors.get(i));
			}
		}
	}

	private void runGrid11Fixed(String gridValString) {

		boolean hasGridData = false;
		// String fnPrefix = "r10_g10_a2_grid_";// "r10_g10_grid_";
		String dir = testDirBaseLoc + "grid11fixed/";// "gridIncTests/";
		String[] fnPrefixes = new String[] { "r10_g10_a2_grid_11_li_rg_", "r10_g10_a2_grid_11_ri_lg_" };
		String extraText = "";
		int numRobots = 10;
		int numGoals = 11;
		int numDoors = 0;
		String fn = "";
//		r10_g10_a2_grid_11_li_rg_  r10_g10_a2_grid_11_li_rg_fs97_fsp_80_9
		boolean dorobotsonly = false;
		boolean dogoalsonly = false;
		int numFilesPerFS = 10;

		int[] fsPercentages = new int[] { 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };

		int[] fsValues = new int[] { 0, 13, 25, 37, 49, 61, 73, 85, 97, 109, 121 };
		int[] fsPercentagesToDo = new int[] { 0, /* 20, */50/* , 80 */, 90 };
		int[] fsValuesToDo = new int[] { 0, 61, 109 };

		int[] rarr = new int[] { 2, 4, 6, 8 };
		int[] garr = new int[] { 3, 5, 7, 9 };
		int roughNumFiles = 0;
		int fnnumStart = 0;
		boolean dofsonly = false;

		if ((gridValString.contains("r"))) {
			logsModLoc = logsBaseLoc + "r/";
			dorobotsonly = true;
			gridValString = gridValString.replace("r", "");
		} else if (gridValString.contains("g")) {
			logsModLoc = logsBaseLoc + "g/";
			dogoalsonly = true;
			gridValString = gridValString.replace("g", "");

		} else if (gridValString.contains("fs")) {
			logsModLoc = logsBaseLoc + "fs/";
			gridValString = gridValString.replace("fs", "");
			dofsonly = true;
		} else {
			logsModLoc = logsBaseLoc + "all/";
		}
		int gridInt = Integer.parseInt(gridValString);
		if (gridInt == 0) {
			fnPrefixes = new String[] { "r10_g10_a2_grid_11_li_rg_" };
		} else if (gridInt == 1) {
			fnPrefixes = new String[] { "r10_g10_a2_grid_11_ri_lg_" };
		}

		int[] rnums = null;
		int[] gnums = null;
		int[] fsnums = null;
		int[] fsvals = null;
		int testFileNum = 1;

		String errorString = "";
		String errorfn = "";
		String resString;

		if (dorobotsonly) {
			rnums = new int[] { /* 2, 4, 6, */8/* , 10 */ };
//			rnums = new int[] { 10 };
			gnums = new int[] { 5 };
			fsnums = new int[] { /* 0, */50, 90 };
			fsvals = new int[] { /* 0, */ 61, 109 };
			extraText = "rall_g5_fs_50_90";
		} else if (dogoalsonly) {
			rnums = new int[] { 4 };
			gnums = new int[] { 9/* 3, 5, 7, 9, 11 */ };
			fsnums = new int[] { 50 };// { 0, 50, 90 };
			fsvals = new int[] { 61 };// { 0, 61, 109 };
			fnnumStart = 5;

			extraText = "r4_gall_fs50_some";
			if (fnnumStart != 0)
				extraText += "_" + fnnumStart;
		} else if (dofsonly) {
			rnums = new int[] { 4 };
			gnums = new int[] { 5 };
			fsnums = fsPercentages;
			fsvals = fsValues;
			extraText = "r4_g5_fsall";
		} // fnPrefix = "grid_r4_g5_fsall_";
		else {
			rnums = rarr;
			gnums = garr;
			fsnums = fsPercentagesToDo;
			fsvals = fsValuesToDo;
		}

		roughNumFiles = rnums.length * gnums.length * fsnums.length * numFilesPerFS;
		System.out.println("Running " + roughNumFiles + " tests ");
		for (String fnPrefix : fnPrefixes) {
			logsModLoc = logsModLoc + fnPrefix + "/";
			for (int rnum : rnums) {
				for (int gnum : gnums) {

					for (int fsnumid = 0; fsnumid < fsnums.length; fsnumid++) {
						int fsnum = fsnums[fsnumid];
						int fsv = fsvals[fsnumid];
						for (int filenum = fnnumStart; filenum < numFilesPerFS; filenum++) {
//						10_g10_a2_grid_11_li_rg_fs97_fsp_80_9
							fn = fnPrefix + "fs" + fsv + "_fsp_" + fsnum + "_" + filenum + "_";
							resString = "";
							errorString = fn;
							errorfn = fn;
							if (!results.containsKey(fn))
								results.put(fn, new HashMap<int[], ArrayList<float[][]>>());
							try {
								ArrayList<Integer> robotNumbers = generateListOfRandomNumbers(rnum, numRobots);

								ArrayList<Integer> goalNumbers = generateListOfRandomNumbers(gnum - 1, numGoals - 1);

								int[] rgdf = new int[] { rnum, gnum, numDoors, fsnum };
								if (!results.get(fn).containsKey(rgdf))
									results.get(fn).put(rgdf, new ArrayList<float[][]>());
								float[][] resArr = new float[2][6];
								resString += "\nR:" + rnum + "\tG:" + gnum;

								fileText(fn + "\nR:" + rnum + "\tG:" + gnum, roughNumFiles, testFileNum);

								errorString += "\nr:" + rnum + " " + robotNumbers.toString() + "\tg:" + gnum + " "
										+ goalNumbers.toString();
								resString += doCompare(dir, fn, rnum, fsnum, gnum, numDoors, resArr, robotNumbers,
										goalNumbers, donullml);

								results.get(fn).get(rgdf).add(resArr);
								String resname = fnPrefix + extraText + "_r" + rnum + "g" + gnum;

								this.printResults(resSavePlace + resname, hasGridData);
								testFileNum++;
							} catch (Exception e) {
								errorString += "\n" + getStackTraceString(e);
								errors.add(errorString);
								writeErrors(resSavePlace + fn + Arrays.toString(rnums) + Arrays.toString(gnums)
										+ Arrays.toString(fsnums) + extraText + "_errors.txt");
								if (results.containsKey(errorfn))
									// remove this from the results
									results.remove(errorfn);
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}
						if (errors.size() > 0)

						{
							writeErrors(resSavePlace + fn + Arrays.toString(rnums) + Arrays.toString(gnums)
									+ Arrays.toString(fsnums) + extraText + "_errors.txt");
							System.out.println("Errors");
							for (int i = 0; i < errors.size(); i++) {
								System.out.println(errors.get(i));
							}
						}
						fileText("Completed Set: " + rnum + "," + gnum + "," + fsnum + "," + "(r,g,fsnum)",
								roughNumFiles, testFileNum);

					}
					fileText("Completed Set: " + rnum + "," + gnum + "," + "(r,g)", roughNumFiles, testFileNum);

				}
				fileText("Completed Set: " + rnum + "," + "(r)", roughNumFiles, testFileNum);

			}

		}
	}

	public String getStackTraceString(Exception e) {
		StringWriter sw = new StringWriter();
		e.printStackTrace(new PrintWriter(sw));
		return sw.toString();
	}

	public void writeErrors(String errorFN) {
		PrintStream outLog = System.out;
		File file = null;
		FileOutputStream fileOutputStream = null;
		if (errorFN != null) {
			file = new File(errorFN + ".txt");

			try {
				fileOutputStream = new FileOutputStream(file);
			} catch (FileNotFoundException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			if (fileOutputStream != null)
				outLog = new PrintStream(fileOutputStream);
		}
		for (int i = 0; i < errors.size(); i++) {
			outLog.println(errors.get(i));
		}
		outLog.close();

	}

	public static void main(String[] args) {

		new CompareSTAPUSSINVI().run(args);

	}
}
