package frc.robot.lib;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;



/** Add your docs here. */
public class OrangePi5Vision {
    PhotonCamera m_noteCam;
    PhotonCamera m_aprilCam;
    PhotonPipelineResult resultsNote;
    PhotonPipelineResult resultsApril;
    public static double m_noteYaw;
    public static boolean m_noteVisible;
    public static double m_noteArea;
    public static NoteData m_noteData = new NoteData();
    public static double m_aprilYaw;
    public static double m_aprilArea;

    List<PhotonTrackedTarget> m_aprilTargets;
    PhotonTrackedTarget m_aprilTarget;
    public OrangePi5Vision(){
       
    //   m_aprilCam = new PhotonCamera("N300");
    //   m_aprilCam.setPipelineIndex(0);
    //   m_aprilCam.setDriverMode(false);

       m_noteCam = new PhotonCamera("NexiGo");
       m_noteCam.setPipelineIndex(0);
       m_noteCam.setDriverMode(false);

    }
    public PhotonPipelineResult getNoteResults(){
        return m_noteCam.getLatestResult();
    }
    
    public NoteData getNoteData(){
        if(hasNoteTargets()){
            PhotonPipelineResult results = getNoteResults();
            if(results.hasTargets()){
                m_noteVisible = true;
                m_noteArea = results.getBestTarget().getArea();
                m_noteYaw = results.getBestTarget().getYaw();
                m_noteData.distance = m_noteArea;
                m_noteData.isNote = true;
                m_noteData.yaw = m_noteYaw;
            }else {
                m_noteVisible = false;
                m_noteArea = 0;
                m_noteYaw = 0;
                m_noteData.distance = 0;
                m_noteData.isNote = false;
                m_noteData.yaw = 0;
            }
        }else {
            m_noteVisible = false;
            m_noteArea = 0;
            m_noteYaw = 0;
            m_noteData.distance = 0;
            m_noteData.isNote = false;
            m_noteData.yaw = 0;
        }
        return m_noteData;
    }
    public boolean hasNoteTargets(){
        return getNoteResults().hasTargets();
    }

    public PhotonPipelineResult getAprilResults(){
        return m_aprilCam.getLatestResult();
    }
    public boolean hasAprilTargets(){
        return getAprilResults().hasTargets();
    }

    // /**
    //  * 
    //  * @return The Yaw in degrees with Positive right
    //  */
    // private double getNoteData(){
    //     resultsNote = m_noteCam.getLatestResult();
        
    //     if(resultsNote.hasTargets()){
    //         return -resultsNote.getBestTarget().getYaw();
    //     }else {
    //         return 0;
    //     }
    // }

    // private double getCameraAprilYaw(){
    //     resultsApril = m_aprilCam.getLatestResult();

    //     m_aprilTarget = resultsApril.getBestTarget();
    //     if(resultsApril.hasTargets()){
    //     //   if(GD.G_Alliance == Alliance.Blue && m_aprilTarget.getFiducialId() == 11){
    //     //         return -r  esultsApril.getBestTarget().getYaw();
    //     //     }else if(GD.G_Alliance == Alliance.Blue && m_aprilTarget.getFiducialId() == 16){
    //     //         m_aprilArea = resultsApril.getBestTarget().getArea();
    //     //         return -resultsApril.getBestTarget().getYaw();
    //     //     }else {
    //     //         return 0;
    //     //     }
            
    //     }else {
    //         return 180.0;
    //     }
    // }
    public double getNoteYaw(){
        return m_noteYaw;
    }
    public double getAprilYaw(){
        return m_aprilYaw;
    }
    // public void findNote(){
    //    getNoteData();
    // }
    // public void findApril(){
    //     m_aprilYaw = getCameraAprilYaw();
    // }
    public double getAprilArea(){
        return m_aprilArea;
    }
}