//==============================================================================
//
//Copyright (c) 2002-
//Authors:
//* Dave Parker <david.parker@comlab.ox.ac.uk> (University of Oxford)
//
//------------------------------------------------------------------------------
//
//This file is part of PRISM.
//
//PRISM is free software; you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation; either version 2 of the License, or
//(at your option) any later version.
//
//PRISM is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with PRISM; if not, write to the Free Software Foundation,
//Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//==============================================================================

package explicit;

import strat.Strategy;

/**
 * Class storing some info/data from a call to a model checking or numerical
 * computation method in the explicit engine.
 */
public class ModelCheckerPartialSatResultVar {
	// Solution vector for probs
	public double[] solnProb = null;
	// Solution vector for probs from previous iteration
	public double[] lastSolnProb = null;
	//// Solution vector for prog rewards
	// public double[] solnProg = null;
	//// Solution vector for prog rewards from previous iteration
	// public double[] lastSolnProg = null;
	//// Solution vector for costs
	// public double[] solnCost = null;
	//// Solution vector for costs from previous iteration
	// public double[] lastSolnCost = null;
	public int numRew = 0;
	public double[][] solns = null;
	public double[][] lastSolns = null;
	// Iterations performed
	public int numIters = 0;
	// Total time taken (secs)
	public double timeTaken = 0.0;
	// Time taken for any precomputation (secs)
	public double timePre = 0.0;
	// Time taken for Prob0-type precomputation (secs)
	public double timeProb0 = 0.0;
	// Strategy
	public Strategy strat = null;

	/**
	 * Clear all stored data, including setting of array pointers to null (which
	 * may be helpful for garbage collection purposes).
	 */
	public void clear() {
		solnProb = lastSolnProb = null;
		for (int i = 0; i < numRew; i++) {
			solns[i] = null;
			lastSolns[i] = null;
		}
		solns = null;
		lastSolns = null;
		numIters = 0;
		timeTaken = timePre = timeProb0 = 0.0;
	}

	private void initSolns(int numRewards) {
		if (numRewards == 0) {
			// default
			numRew = 2;
		} else
			numRew = numRewards;
		solns = new double[numRew][];
		lastSolns = new double[numRew][];
	}

	public void reset(int numRewards) {
		clear();
		initSolns(numRewards);

	}

	public ModelCheckerPartialSatResultVar(int numRewards) {
		initSolns(numRewards);
	}
}