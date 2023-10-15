import processing.video.*;


Capture cam;

void setup() {
   size(500,500);
   frameRate(30);
   noStroke();
   smooth();

   String[] cameras = Capture.list();

  if (cameras.length == 0) {
    println("There are no cameras available for capture.");
    exit();
  } else {
    println("Available cameras:");
    for (int i = 0; i < cameras.length; i++) {
      println(cameras[i]);
    }

    // The camera can be initialized directly using an 
    // element from the array returned by list():
    //cam = new Capture(this, cameras[3]); //built in mac cam "isight"
    cam = new Capture(this, 1280, 960, "USB-camera"); //externe camera Lex, linker USB
    cam.start();
  }
}
void draw(){
 if (cam.available() == true) {
    cam.read();
  }
  image(cam, 0,0,width, height);
}
