let l1,l2,l3; //limb object declaration
let l1_len = 100 , l2_len = 70,l3_len = 40;  //limb lengths
let a = 0 , b = 0 , c =0 ;  //joint angles and gamma
let P0;  //2DOF point obtained from gamma
let rot_base_speed = 0.5;
let faux_origin; // faux origin for computations
let path = []; //3x2 array that contains 2 points on a path
let tracker = [false,false,false]; //tracking array to track progress on path  note: that last path should be return to home


function setup() {
  createCanvas(450, 450);
  
  //set angle mode
  angleMode(DEGREES);
  
  //instantiate limbs
  l1 = new limb(width/4,height/3,l1_len);
  l2 = new limb(l1.end_x , l1.end_y ,l2_len);
  l3 = new limb(l2.end_x , l2.end_y , l3_len);
  
  faux_origin = createVector(l1.x,l1.y);
  
  //set path queue (x,y,gamma)
  path.push(createVector(100,30,0));  //home point
  path.push(createVector(120,60,-90));    //intermediate point
  path.push(createVector(100,100,0)); //desired end point
  
  //home arm at path[0] or els animation will not work
  home();
  //set base speed
  rot_base_speed = 0.1;
  
   
}

function draw() {
  background(220);
  
  //label important points and set origin to lower left
  label_diagram(l1.x,l1.y,l3.end_x,l3.end_y);
  
  //three point path
  if(!tracker[0]){
    tracker[0] = move_angles(path[0],path[1]);
  }else if(!tracker[1]){
    tracker[1] = move_angles(path[1],path[2]);
  }else if(!tracker[2]){
    //return to home
    tracker[2] = move_angles(path[2],path[0]);
    
    //reset arm to home
    //note: homing arm after each cycle gets rid of cumulative error
    if(tracker[2])home();
    
  }
  
  
  //update robot manipulator
  show_angles(createVector(l1.x,l1.y),createVector(l2.x,l2.y));
  l1.extend = true;
  l1.display(a);
  
  
  l2.x = l1.end_x;
  l2.y = l1.end_y;
  l2.extend = true;
  l2.display(a+b);
  
  l3.x = l2.end_x;
  l3.y = l2.end_y;
  l3.display(a+b+c);
  
}

class limb{
  
  constructor(x,y , L_len){
    this.x = x;
    this.y = y;
    this.end_x = this.x + (this.L_len * cos(random(0,180)));
    this.end_y = this.y + (this.L_len * sin(random(0,180)));
    this.ext_x = this.x + ((this.L_len + 70) * cos(0));
    this.ext_y = this.y + ((this.L_len + 70) * sin(0));
    
    this.extend = false;
    this.slope = (this.end_y - this.y ) / (this.end_x - this.x);
    this.L_len = L_len;
  }
  
  display(angle){
    
    //update end point
    this.end_x = this.x + (this.L_len * cos(angle));
    this.end_y = this.y + (this.L_len * sin(angle));
    
    //extend Limb for angle visualisation
    this.ext_x = this.x + ((this.L_len + this.L_len * 0.5) * cos(angle));
    this.ext_y = this.y + ((this.L_len + this.L_len * 0.5) * sin(angle));
    
    this.slope = (this.end_y - this.y ) / (this.end_x - this.x);
    
    
    strokeWeight(2);
    
    //draw limb
    circle(this.x,this.y, 10);
    strokeWeight(5);
    line(this.x , this.y ,this.end_x,this.end_y);
    
    if(this.extend === true){
      strokeWeight(0.5);
      line(this.x,this.y,this.ext_x,this.ext_y);  
    }
    
    
  }
}


//function to calculate inverse kinematics of 2DOF portion
function calculate_ik_2DOF(dest_x,dest_y){
  
  var L1_sqrd = l1_len * l1_len;
  var L2_sqrd = l2_len * l2_len;
  var r_sqrd = (dest_x * dest_x) + (dest_y * dest_y);
  var r = sqrt(r_sqrd);
  
  //calculating for beta
  var beta = 180 - acos((L1_sqrd + L2_sqrd  - r_sqrd )/(2 * l1_len * l2_len));
  
  //calculating for alpha
  var alpha = atan(dest_y/dest_x) + acos((r_sqrd + L1_sqrd - L2_sqrd)/(2 * l1_len * r));
  
  //note that beta is always defined as negative;
  return createVector(alpha,-beta);
}

//function to calculate inverse kinematics of 3DOF portion
function calculate_ik_3DOF(dest_x,dest_y,gamma){
  var x0 = dest_x - (l3_len*cos(gamma));
  var y0 = dest_y - (l3_len*sin(gamma));
  
  P0 = createVector(x0,y0);
  //console.log(P0);
  //after getting simplified version calculate 2DOF version
  var buff = calculate_ik_2DOF(x0,y0);
  var alpha = buff.x;
  var beta = buff.y;
  
  //note that beta is always defined as negative 
  var theta = gamma - alpha - beta;
  //console.log(alpha,beta,theta,gamma);
  return createVector(alpha,beta,theta);
}

//function to facilitate incrementing of angles
function move_angles(src,dest){
  
  
  //get angles from inverse kinematics
  var p1_angles = calculate_ik_3DOF(src.x,src.y,src.z);
  var p2_angles = calculate_ik_3DOF(dest.x,dest.y,dest.z);
  
  //calculate change in angle of alpha, beta and theta
  var d_alpha = p2_angles.x - p1_angles.x;
  var d_beta = p2_angles.y - p1_angles.y;
  var d_theta = p2_angles.z - p1_angles.z;
  
  var s_alpha = 0;
  var s_beta = 0;
  var s_theta = 0;
  
  //progress status
  var done = false;
  
  //incrementing based on next position's beta & alpha angles
  if(d_beta > 0 && d_alpha > 0){
    if(b < p2_angles.y ){
        s_alpha = rot_base_speed;
        s_beta = (s_alpha * d_beta)/d_alpha;
        s_theta = (s_alpha * d_theta)/d_alpha;
    }else done = true;
  }else if(d_beta < 0 && d_alpha > 0){
    if(b > p2_angles.y ){
        s_alpha = rot_base_speed;
        s_beta = (s_alpha * d_beta)/d_alpha;
        s_theta = (s_alpha * d_theta)/d_alpha;
    }else done = true;
  }else if(d_beta < 0 && d_alpha < 0){
    if(b > p2_angles.y ){
        s_alpha -= rot_base_speed;
        s_beta = (s_alpha * d_beta)/d_alpha;
        s_theta = (s_alpha * d_theta)/d_alpha;
    }else done = true;
  }else if(d_beta > 0 && d_alpha < 0){
    if(b < p2_angles.y ){
        s_alpha -= rot_base_speed;
        s_beta = (s_alpha * d_beta)/d_alpha;
        s_theta = (s_alpha * d_theta)/d_alpha;
    }else done = true;
  }
    
  
  //increment angles
  a = a + s_alpha;
  b = b + s_beta;
  c = c + s_theta;
  
  //return true if reached right angle
  return done;
}

//initializing arm angles (to path[0]) and resetting path tracker status
function home(){
  var buff = calculate_ik_3DOF(path[0].x,path[0].y,path[0].z);
  a=buff.x;
  b=buff.y;
  c=buff.z;
  tracker = [false,false,false];
}

function show_angles(joint1,joint2){
  
  strokeWeight(1);
  
  //change this to change arc size
  var arc_sz = 50;
  
  //arc visualizing alpha
  arc(joint1.x , joint1.y ,arc_sz,arc_sz, 0,a);
  
  
  var joint1_end = createVector(joint1.x + l1_len *(cos(a)) , joint1.y + l1_len * (sin(a)));
  
  var joint2_end = createVector(joint1_end.x + l2_len*cos(a+b), joint1_end.y + l2_len*sin(a+b)); 
  
  //arc visualizing beta note: added some filtering since glitches
  if( b > 1 ) arc(joint1_end.x,joint1_end.y,arc_sz,arc_sz,a,a+b );
  else if(b < -1)arc(joint1_end.x,joint1_end.y,arc_sz,arc_sz,a+b,a);
  
  //arc visualizing theta 
  if( c > 1 ) arc(joint2_end.x,joint2_end.y,arc_sz,arc_sz,a+b,a+b+c );
  else if(c < -1)arc(joint2_end.x,joint2_end.y,arc_sz,arc_sz,a+b+c,a+b);
  
  
}

function label_diagram(origin_x,origin_y,end_x,end_y){
  
  //label faux origin
  text('(0,0)', origin_x - 30 ,height - origin_y + 10);
  
  //label faux end effector relative to faux origin
  var end_eff_x = round(end_x - origin_x);
  var end_eff_y = round(end_y - origin_y);
  text( '(' + end_eff_x +','+ end_eff_y + ')' , end_x ,height- end_y); 
  
  //show Forward kinematics
  textSize(16);
  text('Inverse Kinematics',0,15);
  textSize(12);
  
  
  var r_sqrd = (P0.y *P0.y) + ( P0.x * P0.x);
  var r = sqrt(r_sqrd);
  
  text('Alpha =  atan( ' + round(P0.y) +' / ' + round(P0.x) + ' ) acos( ('+ round(r_sqrd )+' + L1^2 - L2^2)  / 2 * '+ round(r) + ' * L1) = '+ round(a),0,50);
  text('Beta   =  180 - acos( (L1^2 + L2^2 - ' + round(r_sqrd) + ') / 2 * L1 * L2) = '+ round(b) , 0, 65); 
  text('Theta   =  gamma - ' + round(a) + ' - ' + round(b) +  ' = '+ round(c) , 0, 80); 
  
  
  
  //display variables on upper left
  text('L1 length: '+ l1_len + '\nL2 length: '+ l2_len + '\nL3 length: ' + l3_len ,0,100);
  
  
  //label angles
  text('a = ' + round(a),origin_x + 30,height - origin_y + 10);
  text('b = ' + round(b), origin_x + (l1_len * cos(a)) + 30 ,  height - (origin_y + l1_len * sin(a)) + 10);
  text('c = ' + round(c), origin_x + (l1_len*cos(a) + l2_len*cos(a+b)) + 30, height - (origin_y + (l1_len*sin(a) + l2_len*sin(a+b))) + 10);
  
  
  text('Inputs:\nP1(' + path[0].x + ','+ path[0].y+')  gamma: '+path[0].z +'\nP2(' + path[1].x + ',' + path[1].y + ') gamma: ' + path[1].z + '\nP3(' + path[2].x + ',' + path[2].y + ') gamma: ' + path[2].z,0,height - 100);
  
  
  
  //set origin to lower left
  translate(0,height);
  
  //invert y axis (y increases upwards)
  scale(1,-1);
  
  
  //draw path points
  point(faux_origin.x+path[0].x,faux_origin.y+path[0].y);
  point(faux_origin.x+path[1].x,faux_origin.y+path[1].y);
  point(faux_origin.x+path[2].x,faux_origin.y+path[2].y);
}


