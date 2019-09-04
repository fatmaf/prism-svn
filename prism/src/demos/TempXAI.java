package demos;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Vector;

import acceptance.AcceptanceOmega;
import acceptance.AcceptanceType;
import automata.DA;
import common.IterableStateSet;
import explicit.LTLModelChecker;
import explicit.MDP;
import explicit.MDPModelChecker;
import parser.ast.ASTElement;
import parser.ast.Expression;
import parser.ast.ExpressionFunc;
import parser.ast.ExpressionProb;
import parser.ast.ModulesFile;
import parser.ast.PropertiesFile;
import prism.Prism;
import prism.PrismException;
import prism.PrismFileLog;
import prism.PrismLog;
import prism.PrismSettings;
import simulator.ModulesFileModelGenerator;

public class TempXAI
{

	//main function 
	public static void main(String[] args)
	{
		new TempXAI().run();
	}

	//run function 
	public void run()
	{

		try {

			String saveplace = "/home/fatma/Data/PhD/code/prism_ws/prism-svn/prism/tests/wkspace/simpleTests/";//"/home/fatma/Data/phD/work/code/mdpltl/prism-svn/prism/tests/decomp_tests/";
			String filename = "g7x3_r2_t3_d0_fs1";//"robot";
			// Create a log for PRISM output (hidden or stdout)
			//PrismLog mainLog = new PrismDevNullLog();
			PrismLog mainLog = new PrismFileLog("stdout");

			// Initialise PRISM engine 
			Prism prism = new Prism(mainLog);

			prism.initialise();
			prism.setEngine(Prism.EXPLICIT);

			//load the mdp for a single model 

			String modelFileName = saveplace + filename + "0.prism";
			ModulesFile modulesFile = prism.parseModelFile(new File(modelFileName));
			prism.loadPRISMModel(modulesFile);
			PropertiesFile propertiesFile = prism.parsePropertiesFile(modulesFile, new File(saveplace + filename + ".props"));
			prism.buildModel();
			MDP mdp = (MDP) prism.getBuiltModelExplicit();

			MDPModelChecker mc = new MDPModelChecker(prism);
			mc.setModulesFileAndPropertiesFile(modulesFile, propertiesFile, new ModulesFileModelGenerator(modulesFile, prism));

			//get the property for a single model 
			for (int i = 0; i < propertiesFile.getNumProperties(); i++)
				mainLog.println(propertiesFile.getProperty(i));
			ExpressionProb exprProb= (ExpressionProb) propertiesFile.getProperty(0);
			Expression expr = exprProb.getExpression();
			//convert property to da 
			LTLModelChecker mcLTL = new LTLModelChecker(prism);
			AcceptanceType[] allowedAcceptance = { AcceptanceType.RABIN, AcceptanceType.REACH };
			Vector<BitSet> labelBS = new Vector<BitSet>();
			DA<BitSet, ? extends AcceptanceOmega> da = mcLTL.constructDAForLTLFormula(mc, mdp, expr, labelBS, allowedAcceptance);
			mainLog.println(da.size());
			int numAPs = da.getAPList().size();
			BitSet s_labels = new BitSet(numAPs);
			mainLog.println(da.getAPList().toString());
			mainLog.println(expr.toString());
			ArrayList<BitSet> statelabels = new ArrayList<BitSet>();
			for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
				s_labels = new BitSet(numAPs);
				// Get BitSet representing APs (labels) satisfied by state s_0
				for (int k = 0; k < numAPs; k++) {
					s_labels.set(k, labelBS.get(Integer.parseInt(da.getAPList().get(k).substring(1))).get(s_0));
				}
				// Find corresponding initial state in DA
				int q_0 = da.getEdgeDestByLabel(da.getStartState(), s_labels);
				mainLog.println(mdp.getStatesList().get(s_0)+" "+s_0+","+q_0+":"+s_labels.toString()); 
				statelabels.add(s_labels);
			}
			HashMap<Object,ArrayList<BitSet>> actionLabels = new HashMap<Object,ArrayList<BitSet>>();
			for (int s_0 : new IterableStateSet(mdp.getNumStates())) {
				//go over all these states again 
				//assign labels to actions as those for the next state 
				//but what about probabilistic ones ? 
				int numChoices = mdp.getNumChoices(s_0); 
				for(int i = 0; i<numChoices; i++)
				{
					Object action = mdp.getAction(s_0, i);
					if (!actionLabels.containsKey(action))
					{
						actionLabels.put(action, new ArrayList<BitSet>());
					}
					Iterator<Entry<Integer, Double>> trIter = mdp.getTransitionsIterator(s_0, i);
					while(trIter.hasNext())
					{
						Entry<Integer, Double> nextSP = trIter.next(); 
						int ns = nextSP.getKey(); 
						actionLabels.get(action).add(statelabels.get(ns));
						
					}
				}
			}
			for(Object a: actionLabels.keySet())
			{
				mainLog.println(a.toString()+" "+ actionLabels.get(a).toString());
			}
			//map da stuff to actions 
			//so essentially form bins 
		} catch (PrismException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
