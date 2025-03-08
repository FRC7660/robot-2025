// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.json.JSONArray;
import org.json.JSONObject;

public class VisionReader extends SubsystemBase {
  /** Creates a new VisionReader. */
  public VisionReader() {}

  @Override
  public void periodic() {
    
    String fromTable = SmartDashboard.getString("detecTable", "[]");

    // System.out.println(fromTable);

    JSONArray top = new JSONArray(fromTable);
    int x = top.length();

    // String pageName = obj.getJSONObject("pageInfo").getString("pageName");
    JSONObject obj = top.getJSONObject(0);
    int tag_id = obj.getInt("tag_id");
    JSONArray pose_R = obj.getJSONArray("pose_R"); // Extract pose_R
    JSONArray pose_t = obj.getJSONArray("pose_t"); // Extract pose_t

    
    System.out.println(x);
    if (x > 0) {
      JSONObject dict = (JSONObject) top.get(0);
      System.out.println(dict);
      System.out.println("PRINT IN BETWEEN");
      // Extracting rotation matrix values
      // roll
      double r00 = pose_R.getJSONArray(0).getDouble(0);
      // double r01 = pose_R.getJSONArray(0).getDouble(1);
      // double r02 = pose_R.getJSONArray(0).getDouble(2);
      // pitch
      double r10 = pose_R.getJSONArray(1).getDouble(0);
      // double r11 = pose_R.getJSONArray(1).getDouble(1);
      // double r12 = pose_R.getJSONArray(1).getDouble(2);
      // yaw
      double r20 = pose_R.getJSONArray(2).getDouble(0);
      double r21 = pose_R.getJSONArray(2).getDouble(1);
      double r22 = pose_R.getJSONArray(2).getDouble(2);

      // Extracting translation vector values
      double tx = pose_t.getJSONArray(0).getDouble(0);
      double ty = pose_t.getJSONArray(1).getDouble(0);
      double tz = pose_t.getJSONArray(2).getDouble(0);

      // Convert rotation matrix to roll, pitch, yaw
      double roll = Math.atan2(r21, r22); // Rotation around X-axis
      double pitch = Math.asin(-r20); // Rotation around Y-axis
      double yaw = Math.atan2(r10, r00); // Rotation around Z-axis

      // Create Rotation3d object
      Rotation3d rotation = new Rotation3d(roll, pitch, yaw);
      // Create Translation3d object
      Translation3d translation = new Translation3d(tx, ty, tz);

      // Construct Pose3d
      Pose3d pose = new Pose3d(translation, rotation);

      // System.out.println("The print works");
      // Print Pose3d
      System.out.println("Constructed Pose3d: " + pose);
      System.out.println(r00);
    }

    // String pageName = obj.getJSONObject("pageInfo").getString("pageName");
    // String pageName = obj.getString("tag_family");
    // JSONObject jsonThing = top.getJSONObject(0);
    // // System.out.println(jsonThing);

    // JSONArray pose_R = jsonThing.getJSONArray("pose_R");
    // JSONArray pose_t = jsonThing.getJSONArray("pose_t");
    // int tag_id = jsonThing.getInt("tag_id");
    // //Int tag_Id = new Int(jsonThing.getInt("tag_id"));
    // System.out.println(pose_R);

  }
}
