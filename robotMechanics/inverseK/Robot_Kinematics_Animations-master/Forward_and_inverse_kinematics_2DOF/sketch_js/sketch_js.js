let l1,l2,l3; //limb object declaration
let l1_len = 85 , l2_len = 84;
let a = 0 , b = 0;
let faux_origin; // faux origin for computations
let path = []; //2x3 array that contains 3 points on a path



function setup() {
  createCanvas(600, 600);
  
  //set angle mode
  angleMode(DEGREES);
  
  //instantiate limbs
  l1 = new limb(width/4,height/3,l1_len);
  l2 = new limb(l1.end_x , l1.end_y ,l2_len);
  
  faux_origin = createVector(l1.x,l1.y);
  
  //set path queue
  path.push(createVector(0,0));  //start at homepoint
  path.push(createVector(-0,20)); //desired end 
  
   
}

function draw() {
  background(220);
  
  //label important points and set origin to lower left
  label_diagram(l1.x,l1.y,l2.end_x,l2.end_y,a,b,l1.L_len,l2.L_len);
  
  //path should always starts at (0,0)
  inc = move_angles(path[0],path[1],a,b,l1.L_len,l2.L_len);
  a = inc.x;
  b = inc.y;
  
  //update robot manipulator
  show_angles(l1.x,l1.y,l1.L_len,a,b);
  l1.extend = true;
  l1.display(a);
  
  
  l2.x = l1.end_x;
  l2.y = l1.end_y;
  
  l2.display(a+b);
  
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
    this.ext_x = this.x + ((this.L_len + 70) * cos(angle));
    this.ext_y = this.y + ((this.L_len + 70) * sin(angle));
    
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


//function to calculate inverse kinematics
function calculate_ik(L1_len,L2_len,dest_x,dest_y){
  
  var L1_sqrd = L1_len * L1_len;
  var L2_sqrd = L2_len * L2_len;
  var r = sqrt( (dest_x * dest_x) + (dest_y * dest_y));
  var r_sqrd = r * r;
  
  //calculating for beta
  var beta = 180 - acos((L1_sqrd + L2_sqrd  - r_sqrd )/(2 * L1_len * L2_len));
  
  //calculating for alpha
  var alpha = atan(dest_y/dest_x) + acos((r_sqrd + L1_sqrd - L2_sqrd)/(2 * L1_len * r));
  
  //note that beta is always defined as negative;
  return createVector(alpha,-beta);
}


//function to facilitate incrementing of angles
// makes use of 
function move_angles(src,dest,alpha,beta,L1,L2){
  var p1_angles = calculate_ik(L1,L2,src.x,src.y);
  var p2_angles = calculate_ik(L1,L2,dest.x,dest.y);
  
  //calculate change in angle of alpha
  var d_alpha = p2_angles.x - p1_angles.x;
  var d_beta = p2_angles.y - p1_angles.y;
  
  /*
  var s_alpha = 0.5;
  var s_beta = (s_alpha * d_beta)/d_alpha;
  */
  var s_alpha = 0;
  var s_beta = 0;
  
  
  if(beta > p2_angles.y){
    s_alpha = 0.5;
    s_beta = (s_alpha * d_beta)/d_alpha;
  }
  
  
  
  return createVector( alpha + s_alpha , beta + s_beta);
}


function show_angles(start_x,start_y, L_len,alpha,beta){
  
  strokeWeight(1);
  
  //change this to change arc size
  var arc_sz = 50;
  
  //arc visualizing alpha
  arc(start_x , start_y ,arc_sz,arc_sz, 0,alpha);
  
  
  var end = createVector(start_x + L_len *(cos(alpha)) , start_y + L_len * (sin(alpha)));
  
  //arc visualizing beta note: added some filtering since glitches
  if( beta > 1 ) arc(end.x,end.y,arc_sz,arc_sz, alpha ,alpha + beta );
  else if(beta < -1)arc(end.x,end.y,arc_sz,arc_sz, (alpha + beta) , alpha);
  
}

function label_diagram(origin_x,origin_y,end_x,end_y,alpha,beta,L1_len , L2_len){
  
  //label faux origin
  text('(0,0)', origin_x - 30 ,height - origin_y + 10);
  
  //label faux end effector relative to faux origin
  var end_eff_x = round(end_x - origin_x);
  var end_eff_y = round(end_y - origin_y);
  text( '(' + end_eff_x +','+ end_eff_y + ')' , end_x ,height- end_y); 
  
  //show Forward kinematics
  textSize(16);
  text('Forward Kinematics',0,15);
  textSize(12);
  
  //display variables on upper left
  text('Alpha: ' + round(alpha) + '\nBeta: ' + round(beta) +'\nL1 length: '+ L1_len + '\nL2 length: '+ L2_len  ,0,40);
  
  //display calculations lower right
  var x_calc = 'x = L1 cos('+ round(alpha)+') + L2 cos('+round(alpha)+' + '+ round(beta) + ') = ' + end_eff_x;
  var y_calc = 'y = L1 sin('+ round(alpha)+') + L2 sin('+round(alpha)+' + '+ round(beta) + ') = ' + end_eff_y;
  text(x_calc,150,40);
  text(y_calc,150,60);
  
  //label angles
  text('a = ' + round(alpha),origin_x + 30,height - origin_y + 10);
  
  if(beta > 0) text('b = ' + round(beta), origin_x + (L1_len * cos(alpha)) + 30 ,  height - (origin_y + L1_len * sin(alpha)));
  else text('b = ' + round(beta), origin_x + (L1_len * cos(alpha)) -50 ,  height - (origin_y + L1_len * sin(alpha)) - 20);
  
  
  //Show inverse kinematics
  
  textSize(16);
  text('Inverse Kinematics',0,height - 80);
  textSize(12);
  
  var r_sqrd = (end_eff_y * end_eff_y) + ( end_eff_x * end_eff_x);
  var r = sqrt(r_sqrd);
  
  text('a = atan( ' + end_eff_y +' / ' + end_eff_x + ' ) acos( ('+ r_sqrd +' + L1^2 - L2^2)  / 2 * '+ round(r) + ' * L1) = '+ round(alpha),0,height - 50);
  text('b = 180 - acos( (L1^2 + L2^2 - ' + r_sqrd + ') / 2 * L1 * L2) = '+ round(beta) , 0,height - 30); 
  
  
  //set origin to lower left
  translate(0,height);
  
  //invert y axis (y increases upwards)
  scale(1,-1);
  
}
