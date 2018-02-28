
import g4p_controls.*;

float GRAVITY=9.8;  // m/s2
float real_time=0;
//parameter
float gyro_uncertainty;  //degree
float carAcceleration;
float carspeed;
float carx,cary;  //pixel
float l;  //meter
float theta;  //degree
float theta_dot;
float sum_theta;
float mass;  //kg
float fs;    //hz
float mForce;
float pixelPerMeter;
float errspeed;
float errspeed_dot;
float totaloffset_x;
float groundslope;   //degree
//
float kp,kd,ki;
float desiredCarSpeed;  //m/s
float errspeed_kp,errspeed_kd;
float virtualthetaforspeed;
float offsetx_kp;
float leftV;

PrintWriter output;

boolean isControl=false;
void setup(){
  size(800,800);
  setting();
  setGUI();
 //   output = createWriter("softsimu_anglesensor.csv"); 

}
void setting(){
  l=0.5;  //meter == height/4;
  pixelPerMeter = 500;//(height/4.)/l;
  //convert meter to pixel
  gyro_uncertainty=0.1;  //degree
  carx=width/2;
  cary=height/2;
  carspeed=0;
  theta=0;
  theta_dot=0;
  mass=1;
  fs = 100;
  mForce = mass * GRAVITY;
  //
  kp=35;
  kd=5;
  ki=0;
  desiredCarSpeed=0;
  errspeed_kp=0.1;
  errspeed_kd=0.;
  offsetx_kp=0.01;
  virtualthetaforspeed=0;
  totaloffset_x=0;
  groundslope=0;
}
void draw(){
  
    physical_simu();
  
    //carx = (carx+1) % width;
    drawPendulum();
    
    drawParameter();
    
    if (isControl){
      control();
    }
    else{
      carAcceleration=0;
    }
    
    real_time += 1/fs;
    
}

void control(){
  //speed control
  //after close enough, then enable offsetx control
  float clip_offsetx=0, ctrled_desiredCarSpeed;
  if (desiredCarSpeed<0.0001 && desiredCarSpeed>-0.0001){
    if (totaloffset_x>100){
      clip_offsetx=0;
      ctrled_desiredCarSpeed=-0.5;
    }
    else if (totaloffset_x <-100){
      clip_offsetx=0;
      ctrled_desiredCarSpeed=0.5;
    }
    else{
      clip_offsetx=totaloffset_x;
      ctrled_desiredCarSpeed=desiredCarSpeed;
    }
  }
  else{
    ctrled_desiredCarSpeed=desiredCarSpeed;
  }
  //
  errspeed_dot = ((ctrled_desiredCarSpeed - carspeed)-errspeed)*fs;  
  errspeed = ctrled_desiredCarSpeed - carspeed;
  float clip_errspeed;
  if (errspeed>1){
    clip_errspeed=1;
  }
  else if (errspeed<-1){
    clip_errspeed=-1;
  }
  else{
    clip_errspeed=errspeed;
  }
  
  
  if (desiredCarSpeed<0.00001 && desiredCarSpeed>-0.00001){  //desiredCarSpeed is 0 case
    virtualthetaforspeed = errspeed_kp * clip_errspeed - offsetx_kp * clip_offsetx;  
  }
  else{
    virtualthetaforspeed = errspeed_kp * clip_errspeed ;  
  }
  if (virtualthetaforspeed>5/360.*TWO_PI ){
    virtualthetaforspeed = 5/360.*TWO_PI;
  }
  else if ( virtualthetaforspeed<-5/360.*TWO_PI){
    virtualthetaforspeed = -5/360.*TWO_PI;
  }
  
  
  float speedCompensate = errspeed_kd * errspeed_dot ;

  //theta =0 control
  float signedTheta ;
  if (theta  < 180 && theta >0){
    signedTheta = theta ;
  }
  else if (theta <0 && theta > -180){
    signedTheta = theta;
  }
  else if (theta >=180) {
    signedTheta =  (theta % 360) - 360;
  }
  else {//if (theta <=-180){
    signedTheta = 360+(theta % 360);
  }
  
  theta = signedTheta;
  
  sum_theta += theta;
  
  //if (abs(theta)<1){
  //  theta=0;
  //}
  
  //adaptive speed control
  //disable speed tunning when theta over 45 degree
  if ((theta-groundslope) >=10 || (theta-groundslope) <= -10){
    virtualthetaforspeed=0;
  }
  
  
  //signedTheta/360.*TWO_PI must be dominant term, virtualthetaforspeed is less than 0.1% weight
  carAcceleration = kp*((signedTheta-groundslope)/360.*TWO_PI - virtualthetaforspeed) + kd*theta_dot +ki*sum_theta;// + speedCompensate;
  //
  /*
  //emulate motor-driver
  float incr_v = carAcceleration * accele_kp;
  
  if (abs(leftV)<turnOnThresh){
    leftV = incr_v>0 ? turnOnThresh+incr_v : -turnOnThresh+incr_v;
  }
  else if (abs(leftV+incr_v)<turnOnThresh){    
    leftV = 0;
  }
  else{
    leftV += incr_v;
  }
  */
  
}

void physical_simu(){
  
  float carTangentialAcceleration = (carAcceleration) * cos((theta-groundslope)/360.*TWO_PI)  ; //m/s2
  float GRAVITYTangentialAcceleration = GRAVITY * sin((theta-groundslope)/360.*TWO_PI) ;  //m/s2
  //
  float TangentialAcceleration = GRAVITYTangentialAcceleration - carTangentialAcceleration;  //m/s2
  float theta_dot_dot = TangentialAcceleration / l ;
  float delta_theta = theta_dot*(1./fs) + 0.5 * theta_dot_dot * (1./fs) * (1./fs);  //radian
  theta_dot += theta_dot_dot /fs;
  
  //
  theta += delta_theta*360./TWO_PI ;
  theta += gyro_uncertainty*((float)random(101)-50.)/100.;
  //
  carspeed += carAcceleration/fs;
  if (carspeed>1){
    carspeed=1;
  }
  else if (carspeed<-1){
    carspeed=-1;
  }
  float delta_carpos = carspeed/fs + 0.5 * carAcceleration / (fs*fs);
  
  totaloffset_x += delta_carpos;
  
  carx += delta_carpos * pixelPerMeter;
  if (carx>width){
    carx = (carx % width);
  }
  else if (carx<0) {
    carx = ((carx+width)); //<>// //<>//
  }
  
  //
  cary = height/2 - (carx-width/2)  * sin((groundslope)/360.*TWO_PI);
  /*
    output.print(theta);  
    output.print(',');
    output.print(theta_dot_dot);  
    output.print(',');
    output.print(carAcceleration);  
    output.print(',');
    output.println(theta_dot);  
  */
}

void drawParameter(){
  textSize(15);
  text("carspeed m/s", 10, 30); 
  text("theta", 120, 30); 
  text("theta_dot", 180, 30); 
  text("real_time(s)", 280, 30); 
  text("carAcceleration(cm/s2)", 380, 30); 
  text("x offset(m)", 570, 30); 

  fill(0, 102, 153);
  text(carspeed, 10, 60);
  text(theta, 110, 60);
  text(theta_dot, 190, 60); 
  text(real_time, 280, 60); 
  text(carAcceleration*100, 400, 60); 
  text(totaloffset_x, 580, 60); 
}
//
GButton start,stop,desiredCarSpeed_btn,fs_btn,kp_btn,kd_btn,speedkp_btn,gyrouncer_btn,l_btn;
GTextField desiredCarSpeed_txf,fs_txf,kp_txf,kd_txf,speedkp_txf,gyrouncer_txf,l_txf;
GButton pixelPerMeter_btn,groundslope_btn,offsetx_kp_btn,ki_btn;
GTextField pixelPerMeter_txf,groundslope_txf,offsetx_kp_txf,ki_txf;
void setGUI(){
    start = new GButton(this, 670, 10, 100, 40, "startControl");
    stop = new GButton(this, 670, 60, 100, 40, "stopControl");
    desiredCarSpeed_btn =new GButton(this, 10, height-90, 100, 30, "desiredCarSpeed m/s"); 
    fs_btn =new GButton(this, 120, height-90, 100, 30, "sampleing(hz)"); 
    kp_btn =new GButton(this, 230, height-90, 100, 30, "balence kp"); 
    kd_btn =new GButton(this, 340, height-90, 100, 30, "balence kd"); 
    speedkp_btn =new GButton(this, 450, height-90, 100, 30, "speed kp"); 
    gyrouncer_btn =new GButton(this, 560, height-90, 100, 30, "gyro noise"); 
    l_btn =new GButton(this, 670, height-90, 100, 30, "Length"); 
    pixelPerMeter_btn =new GButton(this, 10, height-180, 100, 30, "pixelPerMeter"); 
    groundslope_btn =new GButton(this, 120, height-180, 100, 30, "ground slop"); 
    offsetx_kp_btn =new GButton(this, 230, height-180, 100, 30, "offsetx kp"); 
    ki_btn =new GButton(this, 340, height-180, 100, 30, "balence ki"); 
    //
    desiredCarSpeed_txf = new GTextField(this, 10, height-50, 100, 20);
    desiredCarSpeed_txf.tag = "desiredCarSpeed";
    desiredCarSpeed_txf.setPromptText(str(desiredCarSpeed));

    fs_txf = new GTextField(this, 120, height-50, 100, 20);
    fs_txf.tag = "fs";
    fs_txf.setText(str(fs));

    kp_txf = new GTextField(this, 230, height-50, 100, 20);
    kp_txf.tag = "kp";
    kp_txf.setText(str(kp));

    kd_txf = new GTextField(this, 340, height-50, 100, 20);
    kd_txf.tag = "kd";
    kd_txf.setText(str(kd));

    speedkp_txf = new GTextField(this, 450, height-50, 100, 20);
    speedkp_txf.tag = "speedkp";
    speedkp_txf.setText(str(errspeed_kp));

    gyrouncer_txf = new GTextField(this, 560, height-50, 100, 20);
    gyrouncer_txf.tag = "gyronoise";
    gyrouncer_txf.setText(str(gyro_uncertainty));

    l_txf = new GTextField(this, 670, height-50, 100, 20);
    l_txf.tag = "l";
    l_txf.setText(str(l));

    pixelPerMeter_txf = new GTextField(this, 10, height-140, 100, 20);
    pixelPerMeter_txf.tag = "pixelPerMeter_txf";
    pixelPerMeter_txf.setText(str(pixelPerMeter));

    groundslope_txf = new GTextField(this, 120, height-140, 100, 20);
    groundslope_txf.tag = "groundslope_txf";
    groundslope_txf.setText(str(groundslope));

    offsetx_kp_txf = new GTextField(this, 230, height-140, 100, 20);
    offsetx_kp_txf.tag = "offsetx_kp_txf";
    offsetx_kp_txf.setText(str(offsetx_kp));

    ki_txf = new GTextField(this, 340, height-140, 100, 20);
    ki_txf.tag = "ki_txf";
    ki_txf.setText(str(ki));

}
void drawPendulum(){
  //
  background(200);
  //
  stroke(200,100,100);
  line(carx,0,carx,height);
  stroke(200,100,200);
  line(0,height/2,width,height/2);
  line(0,height/2+width/2 * sin((groundslope)/360.*TWO_PI),width,height/2-width/2 * sin((groundslope)/360.*TWO_PI));
  text("screen="+width/pixelPerMeter+"m", 10, height/2-20); 

  //
  fill(100,100,200);
  rect(carx-20, cary-10, 40, 20);
  float mx = carx+l*pixelPerMeter*sin((theta-groundslope)/360.*TWO_PI);
  float my = cary-l*pixelPerMeter*cos((theta-groundslope)/360.*TWO_PI);
  stroke(0);
  line(carx, cary, mx, my);
  fill(100,200,100);
  ellipse(mx, my, 10, 10);
}

void handleButtonEvents(GButton button, GEvent event) {
   if(button == start && event == GEvent.CLICKED){
       // code for button click event
       isControl =true;
    }
    else if(button == stop && event == GEvent.CLICKED){
       // code for button click event
       isControl =false;
    }
    else if(button == desiredCarSpeed_btn && event == GEvent.CLICKED){
       // code for button click event
       desiredCarSpeed = float(desiredCarSpeed_txf.getText());
    }
    else if(button == fs_btn && event == GEvent.CLICKED){
       // code for button click event
       fs = float(fs_txf.getText());
    }
    else if(button == kp_btn && event == GEvent.CLICKED){
       // code for button click event
       kp = float(kp_txf.getText());
    }
    else if(button == kd_btn && event == GEvent.CLICKED){
       // code for button click event
       kd = float(kd_txf.getText());
    }
    else if(button == speedkp_btn && event == GEvent.CLICKED){
       // code for button click event
       errspeed_kp = float(speedkp_txf.getText());
    }
    else if(button == gyrouncer_btn && event == GEvent.CLICKED){
       // code for button click event
       gyro_uncertainty = float(gyrouncer_txf.getText());
    }
    else if(button == l_btn && event == GEvent.CLICKED){
       // code for button click event
       l = float(l_txf.getText());
       //pixelPerMeter = (height/4.)/l;
    }
    else if(button == pixelPerMeter_btn && event == GEvent.CLICKED){
       // code for button click event
       pixelPerMeter = float(pixelPerMeter_txf.getText());
    }
    else if(button == groundslope_btn && event == GEvent.CLICKED){
       // code for button click event
       groundslope = float(groundslope_txf.getText());
    }
    else if(button == offsetx_kp_btn && event == GEvent.CLICKED){
       // code for button click event
       offsetx_kp = float(offsetx_kp_txf.getText());
    }
    else if(button == ki_btn && event == GEvent.CLICKED){
       // code for button click event
       ki = float(ki_txf.getText());
    }
    
}

public void handleTextEvents(GEditableTextControl textControl, GEvent event) { 
  
}