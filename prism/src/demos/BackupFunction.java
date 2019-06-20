package demos;

import prism.PrismException;

public interface BackupFunction
{
	public void backup(ChanceNode n) throws PrismException; 
	public void backup(DecisionNode n) throws PrismException;

	double residual(DecisionNode n, boolean upperBound,float epsilon) throws PrismException;
}
