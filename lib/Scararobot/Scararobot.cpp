#include "Scararobot.h"

Scararobot::Scararobot(int l1, int l2, AccelStepper *M1, AccelStepper *M2, AccelStepper *M3, AccelStepper *M4, Servo *servo)
{
    _L1 = l1;
    _L2 = l2;

    _M1 = M1;
    _M2 = M2;
    _M3 = M3;
    _M4 = M4;

    _servo = servo;

    bras.addStepper(*_M1);
    steppers_control.addStepper(*_M2);
    bras.addStepper(*_M3);
    steppers_control.addStepper(*_M4);

    reglerSensMoteur(-1);

    _M1->setCurrentPosition(0);
    _M2->setCurrentPosition(0);
    _M3->setCurrentPosition(0);
    _M4->setCurrentPosition(0);
}

Scararobot::~Scararobot()
{
    delete _chessboard;
    delete _M1;
    delete _M2;
    delete _M3;
    delete _M4;
    delete _servo;
}

void Scararobot::loadChessboard(Point (*chessboard)[CHS_N]){
    _chessboard = chessboard;
}

// void Scararobot::displayChessboard() {
//     if (_chessboard) {
//         for (int i = 0; i < 8; ++i) {
//             for (int j = 0; j < 8; ++j) {
//                 Serial.print("chessboard["); Serial.print(i); Serial.print("]["); Serial.print(j); Serial.print("] = {");
//                 Serial.print(_chessboard[i][j].x); Serial.print(", "); Serial.print(_chessboard[i][j].y); Serial.println("}");
//             }
//         }
//     } else {
//         Serial.println("Chessboard not loaded");
//     }
// }

void Scararobot::rotationBras(Angles angles){//En absolu
    static long steps[2];

    steps[0] = sens *(long)(angles.theta1 * RAD_TO_STEP); //Moteur M1
    steps[1] = sens *(long)(angles.theta2 * RAD_TO_STEP2); //Moteur M3
    bras.moveTo(steps);
}

void Scararobot::hauteurBras(long step){
    static long steps[1];

    steps[0] = (step);
    steppers_control.moveTo(steps);
}

void Scararobot::setStepParTourBrasMoteurs(long stepParTour, long stepParTour2){
    _step_par_tour = stepParTour;
    _step_par_tour2 = stepParTour2;
    
    RAD_TO_STEP = (_step_par_tour/(TWO_PI_F));
    RAD_TO_STEP2 = (_step_par_tour2/(TWO_PI_F));
    STEP_TO_RAD = (TWO_PI_F/_step_par_tour);
    STEP_TO_RAD2 = (TWO_PI_F/_step_par_tour2);
}

bool Scararobot::run(){
    return bras.run() || steppers_control.run();
}

void Scararobot::runSpeedToPosition(){while(run());}

Angles Scararobot::getAnglesActuelle(){
    static Angles angles;
    angles.theta1 = (float)(sens *_M1->currentPosition()) * STEP_TO_RAD;
    angles.theta2 = (float)(sens *_M3->currentPosition()) * STEP_TO_RAD2;
    return angles;
}

long Scararobot::getHauteurActuelle(){
    return _M2->currentPosition();
}

Angles Scararobot::calculAngles(Point p) {
  static Angles angles;
  // resolution numérique
  angles.theta2 = acos((float(sq(p.x) + sq(p.y) - sq(this->_L1) - sq(this->_L2))) / ((this->_L1 * this->_L2) * 2.f));
  if (p.x < 0 && p.y < 0) {
    angles.theta2 = (-1.) * angles.theta2;
  }

  angles.theta1 = atan((float(p.x)) / (float(p.y))) - atan(((float(this->_L2)) * sin(angles.theta2)) / ((float(this->_L1)) + (float(this->_L2)) * cos(angles.theta2)));

  // Angles adjustment depending in which quadrant the final tool coordinate x,y is
  angles.theta2 *= -1.f;
  
  if (p.x >= 0 && p.y >= 0) {       // 1st quadrant
    angles.theta1 = HALF_PI_F - angles.theta1;
  }
  if (p.x < 0 && p.y > 0) {         // 2nd quadrant
    angles.theta1 = HALF_PI_F - angles.theta1;
  }
  /*
  if (x < 0 && y < 0) {         // 3rd quadrant
    angles.theta1 = 270 - angles.theta1;
    phi = 270 - angles.theta1 - angles.theta2;
    phi = (-1) * phi;
  }
  if (x > 0 && y < 0) {         // 4th quadrant
    angles.theta1 = -90 - angles.theta1;
  }
  if (x < 0 && y == 0) {
    angles.theta1 = 270 + angles.theta1;
  }
  */
  // angles.theta1 = int(angles.theta1 * 16000./(360.));
  // angles.theta2 = int(angles.theta2 * 51200./(360.));
  return angles;
}

void Scararobot::goToXy(Point consigne) {
  static Angles angles;
  
  angles = calculAngles(consigne);

  Serial.print("theta1 = "); Serial.println(angles.theta1);
  Serial.print("theta2 = "); Serial.println(angles.theta2);

  if (angles.theta1 >= 0 && angles.theta1 <= PI_F && angles.theta2 >= (-_495DEG_TO_RAD) && angles.theta2 <= _495DEG_TO_RAD) {
    this->rotationBras(angles);
    // Serial.println("position possible");
  }
  else {
    Serial.println("position impossible");
  }
}