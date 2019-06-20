package demos;

public class Bounds{
	double upper; 
	double lower; 
	public Bounds()
	{
		upper = 0; 
		lower = 0; 
	}
	public Bounds add(Bounds b)
	{
		return new Bounds(upper+b.getUpper(),lower+b.getLower());
	}
	public Bounds add(double b)
	{
		return new Bounds(upper+b,lower+b);
	}
	public Bounds multiply(double m)
	{
		return new Bounds(upper*m, lower*m);
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
	public double diff()
	{
		return upper-lower; 
	}
	public double subtractUpper(double m)
	{
		return upper - m; 
	}
	public double subractLower(double m)
	{
		return lower - m; 
	}
	public double subtractUpper(Bounds m)
	{
		return subtractUpper(m.getUpper());
	}
	public double subtractLower(Bounds m)
	{
		return subractLower(m.getLower());
	}
	@Override
	public String toString()
	{
		return "[" + upper + "," + lower + "]";
	}
	
	
}