package demos;

import java.util.Date;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import prism.PrismFileLog;
import prism.PrismLog;

/*
 * A class to store results and any timing information we might want 
 * Hopefully something we can reuse
 */
public class ResultsTiming
{

	private HashMap<Integer, Long> productCreation = new HashMap<Integer, Long>();
	private HashMap<Integer, Long> reallocations = new HashMap<Integer, Long>();
	private HashMap<Integer, Long> jointPolicyCreation = new HashMap<Integer, Long>();
	private HashMap<Integer, Long> nestedProductStates = new HashMap<Integer, Long>();
	private HashMap<Integer, Long> nestedProductTimes = new HashMap<Integer, Long>();
	private HashMap<Integer, Long> modelLoadingTimes = new HashMap<Integer, Long>();
	private HashMap<Integer, Long> singleAgentSolutions = new HashMap<Integer, Long>();
	private int numRobots = 0;
	private int numTasks = 0;
	private int numDoors = 0;
	private int numReallocStates = 0;
	private int teamMDPStates = 0;
	private int teamMDPTransitions = 0;
	private int numReallocInCode = 0;
	private int numProd = 0;
	private long totalComputationTime = 0;
	private long totalTeamMDPCreationTime = 0;
	private long teamMDPCreationTimeMinusNestedProduct = 0;
	private long allNestedProductCreationTime = 0;
	private long allReallocationsTime = 0;
	private long totalmodelloadingtime = 0;
	private int numFS = 0;
	private String res_trial_name;


	private long global_start_time = 0;
	private long local_start_time = 0;
	private long scope_start_time = 0;


	public long timeout = 100*60 * 1000;
	public PrismLog mainLog;
	public PrismLog resLog;
	String time_identifiers = "+-+";
	private int numModel = 0;
	String saveLoc;
	boolean thts = false;

	String prefix; 
	HashMap<String,Double>values; 
	public enum varIDs {

		numrobots, numtasks, numreallocstates, teammdpstates, teammdptransitions, numreallocationsincode,
		totalcomputationtime, totalteammdpcreationtime, allnestedproductcreationtime, allreallocationstime, 
		productcreation, reallocations, jointpolicycreation, nestedproductstates, nestedproducttimes, numdoors, 
		modelloadingtimes, totalmodelloadingtime, teammdptimeonly, singleagentsolutiontimes,
		jointmodelgentime,failstates


	}

	public ResultsTiming(PrismLog mainLog, String res_trial_name, String saveLoc, boolean thts,String prefix)
	{
		this.mainLog = mainLog;
		this.res_trial_name = res_trial_name;
		this.saveLoc = saveLoc;
		this.thts = thts;
		this.prefix = prefix; 
	}

	public void writeResults()

	{

		// the basics
		String comma = ",";
		String json_text = "{";
		json_text += createJsonStyleString("Time", new Date().toString()) + comma;
		json_text += createJsonStyleString("trial", res_trial_name) + comma;
		json_text += createJsonStyleString("robots", numRobots) + comma;
		json_text += createJsonStyleString("tasks", numTasks) + comma;
		json_text += createJsonStyleString("doors", numDoors) + comma;
		json_text += createJsonStyleString("fs", numFS) + comma;
		if (!thts)
			json_text += createJsonStyleString("ReallocationStates", numReallocStates) + comma;
		if (!thts)
			json_text += createJsonStyleString("productCreation", productCreation, true) + comma;
		if (!thts)
			json_text += createJsonStyleString("Reallocations", reallocations, true) + comma;

		json_text += createJsonStyleString("JointPolicyCreation", jointPolicyCreation, true) + comma;
		if (!thts)
			json_text += createJsonStyleString("nestedProductStates", nestedProductStates, false) + comma;
		else
			json_text += createJsonStyleString("singleAgentSolutions", nestedProductStates, false) + comma;
		if (!thts)
			json_text += createJsonStyleString("nestedProductTimes", nestedProductTimes, true) + comma;
		else
			json_text += createJsonStyleString("singleAgentSolutionTimes", nestedProductTimes, true) + comma;
		
		if (!thts) {
			json_text += createJsonStyleString("teamMDPStates", teamMDPStates) + comma;
			json_text += createJsonStyleString("teamMDPTransitions", teamMDPTransitions) + comma;
			json_text += createJsonStyleString("NumReallocations", numReallocInCode) + comma;
			json_text += createJsonStyleString("Team MDP Time", totalTeamMDPCreationTime) + comma;
		}
		if (!thts)
			json_text += createJsonStyleString("Nested Product Time", allNestedProductCreationTime) + comma;
		else
			json_text += createJsonStyleString("All Single Agent Solutions", allNestedProductCreationTime) + comma;
		if (!thts)
			json_text += createJsonStyleString("All Reallocations Time", allReallocationsTime) + comma;
		json_text += createJsonStyleString("Values",values)+comma;
		json_text += createJsonStyleString("Total Time", System.nanoTime() - global_start_time);
		json_text += "}";
		if (!thts)
			resLog = new PrismFileLog(saveLoc + prefix + res_trial_name + "_r" + numRobots + "_t" + numTasks + "_d" + numDoors + "_fs" + numFS + ".json");
		else
			resLog = new PrismFileLog(
					saveLoc + prefix+ res_trial_name + "_r" + numRobots + "_t" + numTasks + "_d" + numDoors + "_fs" + numFS + "_thts.json");

		resLog.print(json_text);
		resLog.close();
	}

	void saveJointPolicy(JointPolicyBuilder jointPolicyBuilder)
	{
		StatesHelper.saveMDP(jointPolicyBuilder.jointMDP, null, "", prefix +  "_r" + numRobots + "_t" + numTasks + "_d" + numDoors + "_fs" + numFS + "jointPolicy", true);
//		StatesHelper.saveMDPstatra(jointPolicyBuilder.jointMDP, "", prefix +  "_r" + numRobots + "_t" + numTasks + "_d" + numDoors + "_fs" + numFS+ "jointPolicy", true);
	}
	
	public String createJsonStyleString(String varname, HashMap<String, Double> varvalues)
	{
		String comma = ",";
		String text = "n/a";
		if (varvalues != null) {
			text = '"' + varname + '"' + ":[ ";
			for (String key : varvalues.keySet()) {
				text += "{" + createJsonStyleString(key, ""+varvalues.get(key)) + "}" + comma;
				
			}
			text += "]";
		}
		return text;
	}
	public void saveValues(HashMap<String,Double> v)
	{
		values = v; 
	}
	public String createJsonStyleString(String varname, HashMap<Integer, Long> varvalues, boolean dosum)
	{
		long sum = 0;
		String comma = ",";
		String text = "n/a";
		if (varvalues != null) {
			text = '"' + varname + '"' + ":[ ";
			for (int key : varvalues.keySet()) {
				text += "{" + createJsonStyleString(key, varvalues.get(key)) + "}" + comma;
				if (dosum)
					sum += varvalues.get(key);
			}
			if (dosum)
				text += "{" + createJsonStyleString("sum", sum) + "}";
			else
				text = text.substring(0, text.length() - 1);
			text += "]";
		}
		return text;
	}

	public String createJsonStyleString(String varname, String varvalue)
	{
		return '"' + varname + '"' + ':' + '"' + varvalue + '"';
	}

	public String createJsonStyleString(int varname, long varvalue)
	{
		return ("" + '"' + varname + '"' + ':' + '"' + varvalue + '"');
	}

	public String createJsonStyleString(String varname, long varvalue)
	{
		return '"' + varname + '"' + ':' + '"' + varvalue + '"';
	}

	public String createJsonStyleString(String varname, int varvalue)
	{
		return '"' + varname + '"' + ':' + '"' + varvalue + '"';
	}

	private boolean hasTimedOut(long startTime, long endTime, String text, varIDs varid)
	{
		long time = endTime - startTime;

		printTime(time, text, varid);
		if (time > timeout) {
			mainLog.println("Timed Out");
			return true;
		} else
			return false;
	}

	private void recordTime(long startTime, long endTime, String text, varIDs varid)
	{
		long time = endTime - startTime;

		printTime(time, text, varid);

	}

	public void recordInits(int value, String text, varIDs varid)
	{
		mainLog.println(time_identifiers + text + ":" + value + " " + time_identifiers);
		if (varid != null)
			saveData(value, varid);
	}

	public void setGlobalStartTime()
	{
		this.global_start_time = System.nanoTime();
	}

	public void setLocalStartTime()
	{
		this.local_start_time = System.nanoTime();
	}

	public void setScopeStartTime()
	{
		this.scope_start_time = System.nanoTime();
	}

	public void recordTime(String text, varIDs varid, boolean scope_time)
	{
		long endTime = System.nanoTime();
		if (scope_time)
			recordTime(scope_start_time, endTime, text, varid);
		else
			recordTime(local_start_time, endTime, text, varid);
	}

	public void recordTime(long startTime, String text, varIDs varid)
	{
		long endTime = System.nanoTime();
		recordTime(startTime, endTime, text, varid);

	}

	public boolean hasTimedOut(long startTime, String text, varIDs varid)
	{
		long endTime = System.nanoTime();
		return hasTimedOut(startTime, endTime, text, varid);

	}

	private void printTime(long time, String text, varIDs varid)
	{

		if (text == "")
			text = "Time";
		if ((time > 1)) {
			mainLog.println(time_identifiers + text + ": " + time + "ns (" + TimeUnit.SECONDS.convert(time, TimeUnit.NANOSECONDS) + " seconds /"
					+ TimeUnit.MINUTES.convert(time, TimeUnit.NANOSECONDS) + " mins)" + time_identifiers);
		}

		if (varid != null) {
			saveData(time, varid);
		}
	}

	public void saveData(int num, varIDs varid)
	{
		switch (varid) {
		case failstates:

			numFS = num;

			break;
		case numrobots:
			numRobots = num;
			break;
		case numtasks:
			numTasks = num;
			break;
		case numreallocstates:
			numReallocStates = num;
			break;
		case teammdpstates:
			teamMDPStates = num;
		case teammdptransitions:
			teamMDPTransitions = num;
			break;
		case numreallocationsincode:
			numReallocInCode++;
			break;
		case nestedproductstates:
			nestedProductStates.put(numProd, (long) num);
			numProd++;
			break;
		case numdoors:
			numDoors = num;
			break;
		default:
			mainLog.println("ERORR - " + varid.toString() + " not implemented for Save Data");
			break;

		}
	}

	public void saveData(long time, varIDs varid)
	{
		switch (varid) {
		case numrobots:
			numRobots = (int) time;
			break;
		case numtasks:
			numTasks = (int) time;
			break;
		case numreallocstates:
			numReallocStates = (int) time;
			break;
		case teammdpstates:
			teamMDPStates = (int) time;
		case teammdptransitions:
			teamMDPTransitions = (int) time;
			break;
		case numreallocationsincode:
			numReallocInCode++;
			break;
		case totalcomputationtime:
			totalComputationTime = time;
			break;
		case totalteammdpcreationtime:
			totalTeamMDPCreationTime = time;
			break;
		case allnestedproductcreationtime:
			allNestedProductCreationTime = time;
			break;
		case allreallocationstime:
			allReallocationsTime = time;
			break;
		case productcreation:
			productCreation.put(numProd, time);
			// numProd++;
			break;
		case jointpolicycreation:
			jointPolicyCreation.put(numReallocInCode, time);
			break;
		case nestedproducttimes:
			nestedProductTimes.put(numProd, time);
			break;
		case nestedproductstates:
			nestedProductStates.put(numProd, time);
			numProd++;
			break;
		case reallocations:
			reallocations.put(numReallocInCode, time);
			break;
		case modelloadingtimes:
			modelLoadingTimes.put(numModel, time);
			numModel++;
			break;
		case totalmodelloadingtime:
			totalmodelloadingtime = time;
			break;
		case teammdptimeonly:
		case jointmodelgentime:
			this.teamMDPCreationTimeMinusNestedProduct = time;
			break;
		case singleagentsolutiontimes:
			this.singleAgentSolutions.put(numProd, time);
			break;
		default:
			mainLog.println("ERORR - " + varid.toString() + " not implemented for Save Data");
			break;

		}
	}

	public static void main(String[] args)
	{
		// TODO Auto-generated method stub

	}

	public void recordValues(int value, String text, varIDs varid)
	{
		mainLog.println(time_identifiers + text + ":" + value + " " + time_identifiers);
		if (varid != null)
			saveData(value, varid);
	}

}
