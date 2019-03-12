//==============================================================================
//	
//	Copyright (c) 2017-
//	Authors:
//	* Dave Parker <d.a.parker@cs.bham.ac.uk> (University of Birmingham)
//	
//------------------------------------------------------------------------------
//	
//	This file is part of PRISM.
//	
//	PRISM is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.
//	
//	PRISM is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//	
//	You should have received a copy of the GNU General Public License
//	along with PRISM; if not, write to the Free Software Foundation,
//	Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//	
//==============================================================================

package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.BitSet;

import explicit.MDP;
import explicit.MDPModelChecker;
import parser.ast.Expression;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismDevNullLog;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.Result;

/**
 * An example class demonstrating how to control PRISM programmatically,
 * through the functions exposed by the class prism.Prism.
 * 
 * This shows how to load a model from a file, then obtain direct access to
 * the model (and MDP) and analyse it via the explicit model checking engine. 
 * 
 * See the README for how to link this to PRISM.
*/
public class BRTDPandMCTSWithExplicitEngine
{
	public static void main(String[] args)
	{
		new BRTDPandMCTSWithExplicitEngine().run();
	}

	public void run()
	{
		try {
			
			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/mcts/bounded/";
			//tests/mcts/bounded/clocked_coffee.prism tests/mcts/bounded/clocked_coffee.prop -mcts
			saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/decomp_tests/";
			String filename = "clocked_coffee";
			filename="no_door_example";
			// Create a log for PRISM output (hidden or stdout)
			//PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);
			prism.initialise();

			// Parse and load a PRISM model from a file
			ModulesFile modulesFile = prism.parseModelFile(new File(saveplace+filename+".prism"));
			prism.loadPRISMModel(modulesFile);

			// Parse and load a properties model for the model
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace+filename+".prop"));

			// Get PRISM to build the model and then extract it
			prism.setEngine(Prism.EXPLICIT);
//			prism.buildModel();
//			MDP mdp = (MDP) prism.getBuiltModelExplicit(); 
//			mdp.exportToDotFile(saveplace+filename+"mdp.dot");
			
			BRTDPModelChecker brtdpmc = new BRTDPModelChecker(prism,modulesFile,propertiesFile);
			Expression expr = propertiesFile.getProperty(0); 
			
			Result resultBRTDP = brtdpmc.check(expr);
			System.out.println(resultBRTDP.getResult());
			
						
			MCTSModelChecker mctsmc = new MCTSModelChecker(prism,modulesFile,propertiesFile);
			Result resultMCTS = mctsmc.check(expr); 
			System.out.println(resultMCTS.getResult());
			
			
			// Close down PRISM
			prism.closeDown();

		} catch (FileNotFoundException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		} catch (PrismException e) {
			System.out.println("Error: " + e.getMessage());
			System.exit(1);
		}
	}
}