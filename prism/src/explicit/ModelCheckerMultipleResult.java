package explicit;
import java.util.ArrayList;

import strat.MDStrategyArray;

/**
* Class storing some info/data from a call to a model checking or
* numerical computation method in the explicit engine. 
*/
public class ModelCheckerMultipleResult {

	// Solution vectors
	public ArrayList<double[]> solns = null;
	// Solution vectors from previous iteration
	public ArrayList<double[]> lastSolns = null;
	// Iterations performed
	public int numIters = 0;
	// Total time taken (secs)
	public double timeTaken = 0.0;
	// Time taken for any precomputation (secs)
	public double timePre = 0.0;
	// Time taken for Prob0-type precomputation (secs)
	public double timeProb0 = 0.0;
	// Strategy
	public MDStrategyArray strat = null;

	/**
	 * Clear all stored data, including setting of array pointers to null
	 * (which may be helpful for garbage collection purposes).
	 */
	public void clear()
	{
		solns = lastSolns = null;
		numIters = 0;
		timeTaken = timePre = timeProb0 = 0.0;
	}
	

}
