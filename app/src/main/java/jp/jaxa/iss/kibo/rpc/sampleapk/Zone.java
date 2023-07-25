package jp.jaxa.iss.kibo.rpc.sampleapk;

abstract class Zone {
    protected float x_min;
    protected float y_min;
    protected float z_min;
    protected float x_max;
    protected float y_max;
    protected float z_max;

    public Zone(float x_min, float y_min, float z_min, float x_max, float y_max, float z_max){
        this.x_min = x_min;
        this.y_min = y_min;
        this.z_min = z_min;
        this.x_max = x_max;
        this.y_max = y_max;
        this.z_max = z_max;
    }

    public boolean contains(float x, float y, float z){

        if(x >= x_min && x<= x_max && y >= y_min && y <= y_max && z >+ z_min && z <= z_max) return true;

        return false;
    }
}