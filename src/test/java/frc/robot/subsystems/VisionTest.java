package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.jupiter.api.Test;

public class VisionTest {
  private static String jblob =
      """
                                            [
   {
      "tag_id":7,
      "pose_R":[
         [
            0.995131548323372,
            0.09842680200605644,
            -0.005036484725413422
         ],
         [
            -0.09845150094317151,
            0.9951297070238668,
            -0.004916112349186687
         ],
         [
            0.004528078352397911,
            0.005388027974472105,
            0.9999752325237764
         ]
      ],
      "pose_t":[
         [
            -0.26829730244886396
         ],
         [
            -0.1540926605595976
         ],
         [
            0.11786400035698297
         ]
      ]
   }
        ]
            """;

  @Test
  public void test1() {
    // JSONObject obj = new JSONObject(jblob);
    JSONArray top = new JSONArray(jblob);
    // String pageName = obj.getJSONObject("pageInfo").getString("pageName");
    JSONObject obj = top.getJSONObject(0);

    int tag_id = obj.getInt("tag_id");
    JSONArray pose_R = obj.getJSONArray("pose_R"); // Extract pose_R
    JSONArray pose_t = obj.getJSONArray("pose_t"); // Extract pose_t

    // Print values
    // System.out.println("Tag ID: " + tag_id);
    // System.out.println("Rotation Matrix: " + pose_R);
    // System.out.println("Translation Vector: " + pose_t);

    // Extracting rotation matrix values
    // roll
    double r00 = pose_R.getJSONArray(0).getDouble(0);
    double r01 = pose_R.getJSONArray(0).getDouble(1);
    double r02 = pose_R.getJSONArray(0).getDouble(2);
    // pitch
    double r10 = pose_R.getJSONArray(1).getDouble(0);
    double r11 = pose_R.getJSONArray(1).getDouble(1);
    double r12 = pose_R.getJSONArray(1).getDouble(2);
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

    // Print Pose3d
    System.out.println("Constructed Pose3d: " + roll);

    // JSONObject jsonThing = top.getJSONObject(0);
    // System.out.println(jsonThing);

    // JSONArray pose_R = jsonThing.getJSONArray("pose_R");
    // double[] row1 = (double[])pose_R.get(0);
    // double x1 = row1[0];

    // JSONArray pose_t = jsonThing.getJSONArray("pose_t");
    // int tag_id = jsonThing.getInt("tag_id");
    // //Int tag_Id = new Int(jsonThing.getInt("tag_id"));

    // System.out.println(pose_R);

    // //Assertions.assertEquals("24t", pageName);
    // // this would fail
    // // Assertions.assertEquals(obj.getString("tag_id"), "19a");

    // // Double pose_err = obj.getDouble("pose_err");
    // // Assertions.assertEquals(obj.getDouble("pose_err"), 19.23);

  }
}
