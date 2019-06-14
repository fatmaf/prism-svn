package demos;

public class Bounds{
	double upper; 
	double lower; 
	public Bounds()
	{
		upper = 0; 
		lower = 0; 
	}
	public Bounds(double u, double l)
	{
		upper = u; 
		lower = l; 
	}
	public double getUpper()
	{
		return upper;
	}
	public void setUpper(double upper)
	{
		this.upper = upper;
	}
	public double getLower()
	{
		return lower;
	}
	public void setLower(double lower)
	{
		this.lower = lower;
	}
	
}