package demos;

import prism.PrismException;

public interface BackupFunction {
	public void backup(ChanceNode n) throws PrismException;

	public void backup(DecisionNode n) throws PrismException;

	public  void backup(THTSNode n) throws PrismException;

	double residual(DecisionNode n, boolean upperBound, float epsilon) throws PrismException;

	double residual(ChanceNode n, boolean upperBound, float epsilon) throws PrismException;

	public double residual(THTSNode n, boolean upperBound, float epsilon) throws PrismException;
}
