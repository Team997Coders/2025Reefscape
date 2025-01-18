package swervelib;

/** Add your docs here. */
public class Util {
  private Util() {}


  /**
   * @param x the value to be mapped
   * @param a the beginning of the input range
   * @param b the end of the input range
   * @param c the beginning of the output range
   * @param d the end of the input range
   * @return value x within [a,b] mapped to [c,d]
   * return = (X-A)/(B-A) * (D-C) + C
   */
  public static final double mapDouble(double x, double a, double b, double c, double d) {
    return (x - a) / (b - a) * (d - c) + c;
  }
}
