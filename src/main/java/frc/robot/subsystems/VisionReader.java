// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.HashMap;

// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;
import org.json.JSONObject;
import org.json.JSONArray;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;





public class VisionReader extends SubsystemBase {
  /** Creates a new VisionReader. */
public VisionReader() {
    
  }

 

  @Override
  public void periodic() {
   

    String fromTable = SmartDashboard.getString("detecTable", "[]");

    System.out.println(fromTable);

    JSONArray top = new JSONArray(fromTable);
    int x = top.length();
    System.out.println(x);
    if(x>0){
      JSONObject dict = (JSONObject)top.get(0);
      System.out.println(dict);
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
