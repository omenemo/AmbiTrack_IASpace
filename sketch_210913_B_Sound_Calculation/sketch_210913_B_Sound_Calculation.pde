/**
 Sketch to calculatie projected vector location for use in ambisonic system
 */

import oscP5.*;
import netP5.*;
import java.util.Collections; 

OscP5 oscP5;
NetAddress myRemoteLocation;

PVector listener = new PVector(0, 0, 0);
PVector rb403 = new PVector(0, 0, 0);
PVector rb502 = new PVector(0, 0, 0);
PVector monolith = new PVector(0, 0, 0);

//elevation and Y-intersection
float m;
float b;

// Hardcoded Boundaries to check against
float lBorX = (-60);
float rBorX = (60);
float tBorY = (-100);
float bBorY = (100);

// variables for intersection coordinates
float tDesX;
float bDesX;
float lDesY;
float rDesY;

PVector tDes = new PVector(0, 0, 0);
PVector bDes = new PVector(0, 0, 0);
PVector lDes = new PVector(0, 0, 0);
PVector rDes = new PVector(0, 0, 0);


// Data for magnitude calculation and comparison
PVector testVector1 = new PVector(0, 0, 0);
PVector testVector2 = new PVector(0, 0, 0);
PVector lVec = new PVector(0, 0, 0);
PVector cVec = new PVector(0, 0, 0);
float dotProd;
ArrayList <PVector> Des = new ArrayList <PVector> ();

// Final calculated values
PVector destination = new PVector(0, 0, 0);
float distance = (0);


void setup() {
  size(800, 480);
  //frameRate();

  /* start oscP5, listening for incoming messages at port 54321 */
  oscP5 = new OscP5(this, 54321);

  // filter osc stream for «rigidbody» objects, send it to position function
  oscP5.plug(this, "position_trck", "/rigidbody");
  // OSC test function
  oscP5.plug(this, "test", "/test");
  
  myRemoteLocation = new NetAddress("127.0.0.1",12000);
}

void draw() {
  background(0);

  // Translate to center
  translate(400, 240); 

  // Draw Room model
  stroke(255, 255, 255);
  strokeWeight(1);
  rect(0, 0, 120, 200);

  // draw tracking
  fill(255, 40, 0); 
  stroke(255, 50, 0);
  strokeWeight(1);
  rectMode(CENTER);
  rect(listener.x, listener.y, 20, 20); // object
  stroke(0, 200, 200);
  fill(0);
  circle(monolith.x, monolith.y, 20); // user
  stroke(255, 255, 255);
  line(monolith.x, monolith.y, listener.x, listener.y);

  //Function for calculating the intersection points
  vectorCalc();
  fill(0, 0, 0);
  stroke(255, 255, 255);
  line(listener.x, listener.y, destination.x, destination.y);
  // displaying 4 possible intersection Points
  //circle(tDesX, tBorY, 20);
  //circle(bDesX, bBorY, 20);
  //circle(lBorX, lDesY, 20);
  //circle(rBorX, rDesY, 20);
  stroke(255, 50, 0);
  
  position_send();
  amp_send();
}

void position_sim () {
  float step = 50;
  listener.x += random(-step, step);
  listener.y += random(-step, step);

  listener.x = constrain(listener.x, -50, 50);
  listener.y = constrain(listener.y, -90, 90);

  monolith.x += random(-30, 30);
  monolith.y += random(-30, 30);

  monolith.x = constrain(monolith.x, -50, 50);
  monolith.y = constrain(monolith.y, -90, 90);
  
}

// function to parse and sort the plugged information
void position_trck (int id, String msg_type, float x, float y, float z) {
  // checking Rigid Body id and saving it to its PVector
  if (id == 909) {
    listener.x = (x*-10);
    listener.y = (y*10);
    //println("x," + x + "y," + y + "z," + z);
  }
  
  if (id == 402) {
    monolith.x = (x*-10);
    monolith.y = (y*10);
  }
}

void position_send() {
  OscMessage myMessage = new OscMessage("soundDest");
  
  myMessage.add(destination.x/-10);
  myMessage.add(destination.y/10);/* add an int to the osc message */
  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 
}

void amp_send() {
  OscMessage myMessage = new OscMessage("soundAmp");
 
  myMessage.add(distance);
  /* send the message */
  oscP5.send(myMessage, myRemoteLocation); 
}

void vectorCalc () {

  //calculate elevation
  m = ((listener.y - monolith.y) / (listener.x - monolith.x));

  // calculate Y axis intersection
  b = (listener.y - m * listener.x);

  Des.clear();
  // check against boders
  tDesX = ((tBorY - b) / m); // left boder collsion point
  tDes.set(tDesX, tBorY);
  Des.add(tDes);

  bDesX = ((bBorY - b) / m); // left boder collsion point
  bDes.set(bDesX, bBorY);
  Des.add(bDes);

  lDesY = (m * lBorX + b); // left boder collsion point
  lDes.set(lBorX, lDesY);
  Des.add(lDes);

  rDesY = (m * rBorX + b); // left boder collsion point
  rDes.set(rBorX, rDesY);
  Des.add(rDes);
  

  float minDist = (999999);
  
  for (int i = 0; i < Des.size(); i++) { // for loop to calculate 4 dot products
    testVector1 = PVector.sub(listener, monolith); //generate pvector from two positions
    testVector2 = PVector.sub(Des.get(i), monolith); // take intersection point number (i) from array, calculate vector
    dotProd = PVector.dot(testVector1, testVector2); // calculate current dot product

    if (dotProd > 0  && dotProd < minDist) {
      //checking calculated current dot product paralellity, Parallell if dot product = 1
      destination.set(Des.get(i));
      minDist = (dotProd);
    }
  }
  
  distance = mag(testVector1.x, testVector1.y);
  println("distance: " + distance);
  
  fill(255, 0, 0);
  circle(destination.x, destination.y, 40);
}
