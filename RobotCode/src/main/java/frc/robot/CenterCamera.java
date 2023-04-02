package frc.robot;

public class CenterCamera {
    
    public BaseCamera camera;

    /* Singleton infrastructure */
    private static CenterCamera instance;
    public static CenterCamera getInstance() {
        if (instance == null) {
            instance = new CenterCamera();
        }
        return instance;
    }

    public CenterCamera(){
        camera = new BaseCamera("");
    }

    public void update(){

        camera.update();

    }

}