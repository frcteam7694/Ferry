package frc.robot.util;

// Garbage collector fuckery
public class IntReference {

  private int val;

  public IntReference(int val) {
    this.val = val;
  }

  public int val() {
    return val;
  }

  public void val(int val) {
    this.val = val;
  }
}
