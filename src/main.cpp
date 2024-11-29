#include <Arduino.h>
#include <Scararobot.h>

// branchement des moteurs
#define STEP_X_PIN 2
#define STEP_Y_PIN 3
#define STEP_Z_PIN 4
#define STEP_A_PIN 12
#define DIR_X_PIN 5
#define DIR_Y_PIN 6
#define DIR_Z_PIN 7
#define DIR_A_PIN 13
#define ENABLE_PIN 8 // nécessaire pour le shield

// caractéristiques du bras
int L1=234; //20 23.5 25   (205 + 214) - (79/2) = 419
int L2=144; //14.5  = 223 - 79
long step_par_tour = 16000; // nombre de pas par tour pour le moteurs bras M1
long step_par_tour2 = 54000; // nombre de pas par tour pour le moteurs bras M3

AccelStepper M1(1, STEP_Y_PIN, DIR_Y_PIN); // moteur rotation robot sur lui même (theta1) : vitesse: 1000; step: 16 000 pour un tour
AccelStepper M2(1, STEP_Z_PIN, DIR_Z_PIN); // moteur monté/descente : vitesse:  600; step: 4000 --> 1000 = 4cm
AccelStepper M3(1, STEP_X_PIN, DIR_X_PIN); // moteur tour du 2e bras (theta2) : vitesse: 10 000; step: 12 800 pour un quart de tour soit pour un tour 51 200
AccelStepper M4(1, STEP_A_PIN, DIR_A_PIN); // moteur rotation pince sur elle même (phi) : vitesse ...; step ...

Servo myservo;

MultiStepper steppersControl;
// sens direct : négatif / sens horaire : positif / pour M2 : positif:monte et négatif:descend

Scararobot robot(L1, L2, &M1, &M2, &M3, &M4, &myservo);

String state_servo;
String state_hauteur = "Haut";

Point liste_mouvement[SIZE_MOUV];
unsigned char index_mouvement = 0;
void addMouvement(Point p) {
  liste_mouvement[index_mouvement] = p;
  index_mouvement = (index_mouvement + 1) % SIZE_MOUV;
}

// Point poubelle = {L1 + L2, 0};
// Point CHS_A8 = {13.3, 12.7};
// Point CHS_A1 = {16, 31};
// Point CHS_B8 = {CHS_A8.x - CASE, CHS_A8.y};
// Point CHS_C8 = {CHS_A8.x - 2*CASE, CHS_A8.y};

// chessboard[0][0] = {13.30 - 0 * CASE, 12.70 + (7 - 0) * CASE} = {13.30, 32.30}
// chessboard[0][1] = {13.30 - 0 * CASE, 12.70 + (7 - 1) * CASE} = {13.30, 29.50}
// chessboard[0][2] = {13.30 - 0 * CASE, 12.70 + (7 - 2) * CASE} = {13.30, 26.70}
// chessboard[0][3] = {13.30 - 0 * CASE, 12.70 + (7 - 3) * CASE} = {13.30, 23.90}
// chessboard[0][4] = {13.30 - 0 * CASE, 12.70 + (7 - 4) * CASE} = {13.30, 21.10}
// chessboard[0][5] = {13.30 - 0 * CASE, 12.70 + (7 - 5) * CASE} = {13.30, 18.30}
// chessboard[0][6] = {13.30 - 0 * CASE, 12.70 + (7 - 6) * CASE} = {13.30, 15.50}
// chessboard[0][7] = {13.30 - 0 * CASE, 12.70 + (7 - 7) * CASE} = {13.30, 12.70}
// chessboard[1][0] = {13.30 - 1 * CASE, 12.70 + (7 - 0) * CASE} = {10.50, 32.30}
// chessboard[1][1] = {13.30 - 1 * CASE, 12.70 + (7 - 1) * CASE} = {10.50, 29.50}
// chessboard[1][2] = {13.30 - 1 * CASE, 12.70 + (7 - 2) * CASE} = {10.50, 26.70}
// chessboard[1][3] = {13.30 - 1 * CASE, 12.70 + (7 - 3) * CASE} = {10.50, 23.90}
// chessboard[1][4] = {13.30 - 1 * CASE, 12.70 + (7 - 4) * CASE} = {10.50, 21.10}
// chessboard[1][5] = {13.30 - 1 * CASE, 12.70 + (7 - 5) * CASE} = {10.50, 18.30}
// chessboard[1][6] = {13.30 - 1 * CASE, 12.70 + (7 - 6) * CASE} = {10.50, 15.50}
// chessboard[1][7] = {13.30 - 1 * CASE, 12.70 + (7 - 7) * CASE} = {10.50, 12.70}
// chessboard[2][0] = {13.30 - 2 * CASE, 12.70 + (7 - 0) * CASE} = {7.70, 32.30}
// chessboard[2][1] = {13.30 - 2 * CASE, 12.70 + (7 - 1) * CASE} = {7.70, 29.50}
// chessboard[2][2] = {13.30 - 2 * CASE, 12.70 + (7 - 2) * CASE} = {7.70, 26.70}
// chessboard[2][3] = {13.30 - 2 * CASE, 12.70 + (7 - 3) * CASE} = {7.70, 23.90}
// chessboard[2][4] = {13.30 - 2 * CASE, 12.70 + (7 - 4) * CASE} = {7.70, 21.10}
// chessboard[2][5] = {13.30 - 2 * CASE, 12.70 + (7 - 5) * CASE} = {7.70, 18.30}
// chessboard[2][6] = {13.30 - 2 * CASE, 12.70 + (7 - 6) * CASE} = {7.70, 15.50}
// chessboard[2][7] = {13.30 - 2 * CASE, 12.70 + (7 - 7) * CASE} = {7.70, 12.70}
// chessboard[3][0] = {13.30 - 3 * CASE, 12.70 + (7 - 0) * CASE} = {4.90, 32.30}
// chessboard[3][1] = {13.30 - 3 * CASE, 12.70 + (7 - 1) * CASE} = {4.90, 29.50}
// chessboard[3][2] = {13.30 - 3 * CASE, 12.70 + (7 - 2) * CASE} = {4.90, 26.70}
// chessboard[3][3] = {13.30 - 3 * CASE, 12.70 + (7 - 3) * CASE} = {4.90, 23.90}
// chessboard[3][4] = {13.30 - 3 * CASE, 12.70 + (7 - 4) * CASE} = {4.90, 21.10}
// chessboard[3][5] = {13.30 - 3 * CASE, 12.70 + (7 - 5) * CASE} = {4.90, 18.30}
// chessboard[3][6] = {13.30 - 3 * CASE, 12.70 + (7 - 6) * CASE} = {4.90, 15.50}
// chessboard[3][7] = {13.30 - 3 * CASE, 12.70 + (7 - 7) * CASE} = {4.90, 12.70}
// chessboard[4][0] = {13.30 - 4 * CASE, 12.70 + (7 - 0) * CASE} = {2.10, 32.30}
// chessboard[4][1] = {13.30 - 4 * CASE, 12.70 + (7 - 1) * CASE} = {2.10, 29.50}
// chessboard[4][2] = {13.30 - 4 * CASE, 12.70 + (7 - 2) * CASE} = {2.10, 26.70}
// chessboard[4][3] = {13.30 - 4 * CASE, 12.70 + (7 - 3) * CASE} = {2.10, 23.90}
// chessboard[4][4] = {13.30 - 4 * CASE, 12.70 + (7 - 4) * CASE} = {2.10, 21.10}
// chessboard[4][5] = {13.30 - 4 * CASE, 12.70 + (7 - 5) * CASE} = {2.10, 18.30}
// chessboard[4][6] = {13.30 - 4 * CASE, 12.70 + (7 - 6) * CASE} = {2.10, 15.50}
// chessboard[4][7] = {13.30 - 4 * CASE, 12.70 + (7 - 7) * CASE} = {2.10, 12.70}
// chessboard[5][0] = {13.30 - 5 * CASE, 12.70 + (7 - 0) * CASE} = {-0.70, 32.30}
// chessboard[5][1] = {13.30 - 5 * CASE, 12.70 + (7 - 1) * CASE} = {-0.70, 29.50}
// chessboard[5][2] = {13.30 - 5 * CASE, 12.70 + (7 - 2) * CASE} = {-0.70, 26.70}
// chessboard[5][3] = {13.30 - 5 * CASE, 12.70 + (7 - 3) * CASE} = {-0.70, 23.90}
// chessboard[5][4] = {13.30 - 5 * CASE, 12.70 + (7 - 4) * CASE} = {-0.70, 21.10}
// chessboard[5][5] = {13.30 - 5 * CASE, 12.70 + (7 - 5) * CASE} = {-0.70, 18.30}
// chessboard[5][6] = {13.30 - 5 * CASE, 12.70 + (7 - 6) * CASE} = {-0.70, 15.50}
// chessboard[5][7] = {13.30 - 5 * CASE, 12.70 + (7 - 7) * CASE} = {-0.70, 12.70}
// chessboard[6][0] = {13.30 - 6 * CASE, 12.70 + (7 - 0) * CASE} = {-3.50, 32.30}
// chessboard[6][1] = {13.30 - 6 * CASE, 12.70 + (7 - 1) * CASE} = {-3.50, 29.50}
// chessboard[6][2] = {13.30 - 6 * CASE, 12.70 + (7 - 2) * CASE} = {-3.50, 26.70}
// chessboard[6][3] = {13.30 - 6 * CASE, 12.70 + (7 - 3) * CASE} = {-3.50, 23.90}
// chessboard[6][4] = {13.30 - 6 * CASE, 12.70 + (7 - 4) * CASE} = {-3.50, 21.10}
// chessboard[6][5] = {13.30 - 6 * CASE, 12.70 + (7 - 5) * CASE} = {-3.50, 18.30}
// chessboard[6][6] = {13.30 - 6 * CASE, 12.70 + (7 - 6) * CASE} = {-3.50, 15.50}
// chessboard[6][7] = {13.30 - 6 * CASE, 12.70 + (7 - 7) * CASE} = {-3.50, 12.70}
// chessboard[7][0] = {13.30 - 7 * CASE, 12.70 + (7 - 0) * CASE} = {-6.30, 32.30}
// chessboard[7][1] = {13.30 - 7 * CASE, 12.70 + (7 - 1) * CASE} = {-6.30, 29.50}
// chessboard[7][2] = {13.30 - 7 * CASE, 12.70 + (7 - 2) * CASE} = {-6.30, 26.70}
// chessboard[7][3] = {13.30 - 7 * CASE, 12.70 + (7 - 3) * CASE} = {-6.30, 23.90}
// chessboard[7][4] = {13.30 - 7 * CASE, 12.70 + (7 - 4) * CASE} = {-6.30, 21.10}
// chessboard[7][5] = {13.30 - 7 * CASE, 12.70 + (7 - 5) * CASE} = {-6.30, 18.30}
// chessboard[7][6] = {13.30 - 7 * CASE, 12.70 + (7 - 6) * CASE} = {-6.30, 15.50}
// chessboard[7][7] = {13.30 - 7 * CASE, 12.70 + (7 - 7) * CASE} = {-6.30, 12.70}

// Point chessboard[CHS_N][CHS_N] = {
//   {{13.30, 32.30}, {13.30, 29.50}, {13.30, 26.70}, {13.30, 23.90}, {13.30, 21.10}, {13.30, 18.30}, {13.30, 15.50}, {13.30, 12.70}},
//   {{10.50, 32.30}, {10.50, 29.50}, {10.50, 26.70}, {10.50, 23.90}, {10.50, 21.10}, {10.50, 18.30}, {10.50, 15.50}, {10.50, 12.70}},
//   {{7.70, 32.30}, {7.70, 29.50}, {7.70, 26.70}, {7.70, 23.90}, {7.70, 21.10}, {7.70, 18.30}, {7.70, 15.50}, {7.70, 12.70}},
//   {{4.90, 32.30}, {4.90, 29.50}, {4.90, 26.70}, {4.90, 23.90}, {4.90, 21.10}, {4.90, 18.30}, {4.90, 15.50}, {4.90, 12.70}},
//   {{2.10, 32.30}, {2.10, 29.50}, {2.10, 26.70}, {2.10, 23.90}, {2.10, 21.10}, {2.10, 18.30}, {2.10, 15.50}, {2.10, 12.70}},
//   {{-0.70, 32.30}, {-0.70, 29.50}, {-0.70, 26.70}, {-0.70, 23.90}, {-0.70, 21.10}, {-0.70, 18.30}, {-0.70, 15.50}, {-0.70, 12.70}},
//   {{-3.50, 32.30}, {-3.50, 29.50}, {-3.50, 26.70}, {-3.50, 23.90}, {-3.50, 21.10}, {-3.50, 18.30}, {-3.50, 15.50}, {-3.50, 12.70}},
//   {{-6.30, 32.30}, {-6.30, 29.50}, {-6.30, 26.70}, {-6.30, 23.90}, {-6.30, 21.10}, {-6.30, 18.30}, {-6.30, 15.50}, {-6.30, 12.70}}
// };

Point chessboard[CHS_N][CHS_N] = {//En mm
  {{133, 323}, {133, 295}, {133, 267}, {133, 239}, {133, 211}, {133, 183}, {133, 155}, {133, 127}},
  {{105, 323}, {105, 295}, {105, 267}, {105, 239}, {105, 211}, {105, 183}, {105, 155}, {105, 127}},
  {{77, 323}, {77, 295}, {77, 267}, {77, 239}, {77, 211}, {77, 183}, {77, 155}, {77, 127}},
  {{49, 323}, {49, 295}, {49, 267}, {49, 239}, {49, 211}, {49, 183}, {49, 155}, {49, 127}},
  {{21, 323}, {21, 295}, {21, 267}, {21, 239}, {21, 211}, {21, 183}, {21, 155}, {21, 127}},
  {{-7, 323}, {-7, 295}, {-7, 267}, {-7, 239}, {-7, 211}, {-7, 183}, {-7, 155}, {-7, 127}},
  {{-35, 323}, {-35, 295}, {-35, 267}, {-35, 239}, {-35, 211}, {-35, 183}, {-35, 155}, {-35, 127}},
  {{-63, 323}, {-63, 295}, {-63, 267}, {-63, 239}, {-63, 211}, {-63, 183}, {-63, 155}, {-63, 127}}
};


void RxManage();
void initialize_servo();

void ouvrePince();
void fermePince();


void setup() {
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);

    Serial.begin(9600);
    Serial.println("setup");

    // Set les vitesses max des moteurs
    M1.setMaxSpeed(2000); // 2000
    M2.setMaxSpeed(600); // 600
    M3.setMaxSpeed(7000); // 10000
    M4.setMaxSpeed(100); // 100//Ne sert à rien

    myservo.attach(12);// attaches the servo on pin 12 to the servo object
    myservo.write(96);

    robot.loadChessboard(chessboard);
    robot.setStepParTourBrasMoteurs(step_par_tour, step_par_tour2);

    // robot.rotationBras({0,- 4* PI/2});
    

    // for (int i = 0; i < CHS_N; i++)
    // {
    //   for (int j = 0; j < 3; j++)
    //   {
    //     chessboard[i][j].x = CHS_A8.x - i * CASE;
    //     chessboard[i][j].y = CHS_A8.y + ((CHS_N-1)-j) * CASE;
    //     // Serial.print("chessboard[");
    //     // Serial.print(i);
    //     // Serial.print("][");
    //     // Serial.print(j);
    //     // Serial.print("] = {");
    //     // Serial.print(CHS_A8.x);
    //     // Serial.print(" - ");
    //     // Serial.print(i);
    //     // Serial.print(" * CASE, ");
    //     // Serial.print(CHS_A8.y);
    //     // Serial.print(" + (");
    //     // Serial.print(CHS_N-1);
    //     // Serial.print(" - ");
    //     // Serial.print(j);
    //     // Serial.print(") * CASE} = {");
    //     // Serial.print(chessboard[i][j].x);
    //     // Serial.print(", ");
    //     // Serial.print(chessboard[i][j].y);
    //     // Serial.println("}");
    //   }
    // }
    

    // poubelle.x = L1 + L2;
    // poubelle.y = 0;

    /*
    A1 = 16, 31
    H1 = -2.5, 32.3
    H8 = -5, 12
    A8 = 13.3 , 12

    P1 = 10.5, 25
    P2 = 2.5, 25.7
    P3 = 1.1, 16.8
    P4 = 8.7, 16.9
    */

    // P0.x = L1 + L2;
    // P0.y = 0;

    // P11.x = 17;
    // P11.y = 31;

    // P12.x = 14.2;
    // P12.y = 31;

    // P81.x = -2.5;
    // P81.y = 32.3;

    // P88.x = -5;
    // P88.y = 12;

    // P18.x = 13.3;
    // P18.y = 12;

    // P33.x = 10.5;
    // P33.y = 25;

    // P63.x = 2.5;
    // P63.y = 25.7;

    // P66.x = 0.8;
    // P66.y = 16.6;

    // P36.x = 8.7;
    // P36.y = 16.9;
    // descendBras2();
    // delay(100);
    // monteBras2();
    // delay(500);
    addMouvement(chessboard[0][0]);
    addMouvement(chessboard[0][7]);
    // addMouvement(chessboard[7][7]);
    // addMouvement(chessboard[7][0]);
    // for (int i = 0; i < CHS_N; i+=2)
    // {
    //   for (int j = 0; j < CHS_N; j++) 
    //   {
    //     addMouvement(chessboard[i][j]);
    //   }
    // }
    
    // robot.goToXy(poubelle);
    // robot.goToXy(chessboard[0][0]);
    // Serial.print("index_mouvement = ");
    // Serial.println(index_mouvement);

    robot.hauteurBras(-200);
    robot.runSpeedToPosition();
    delay(100);
    robot.hauteurBras(0);
    robot.runSpeedToPosition();
}

void loop() {
  if(!robot.run()){
    RxManage();
  }
  
    // for (int x = 14; x >= -2; x -= 4) {          // De 14 à -2 avec un pas de -4
    //     for (int y = 30; y >= 5; y -= 5) {        // De 30 à 5 avec un pas de -5
    //         robot.goToXy(x, y);
    //         delay(500);
    //         descendBras2();
    //         delay(500);
    //         monteBras2();
    //         delay(500);
    //     }
    // }
}


void RxManage(){
  static Angles angles;
  static signed char FIFO_lecture=0;
  if (Serial.available()) {
    char data = Serial.read();
    Serial.println(data);
    if(data == 'a'){
      Serial.print("liste_mouvement[");
      Serial.print(FIFO_lecture);
      Serial.print("] = {");
      Serial.print(liste_mouvement[FIFO_lecture].x);
      Serial.print(", ");
      Serial.print(liste_mouvement[FIFO_lecture].y);
      Serial.println("}");
      robot.goToXy(liste_mouvement[FIFO_lecture]);
      FIFO_lecture=(FIFO_lecture+1)%(index_mouvement+1);
      if(!FIFO_lecture){
        robot.goToXy({L1 + L2, 0});
      }
    }
    else if(data == 'z'){
      FIFO_lecture = (FIFO_lecture-1)<0?index_mouvement:FIFO_lecture-1;
      Serial.print("liste_mouvement[");
      Serial.print(FIFO_lecture);
      Serial.print("] = {");
      Serial.print(liste_mouvement[FIFO_lecture].x);
      Serial.print(", ");
      Serial.print(liste_mouvement[FIFO_lecture].y);
      Serial.println("}");
      robot.goToXy(liste_mouvement[FIFO_lecture]);
    }
    else if (data == 'p'){
      //Augmenter L1
      L1+=1;
      robot.setLongueurBras(L1, L2);
      Serial.print("L1 = ");
      Serial.println(L1);
    }
    else if (data == 'm'){
      //Diminuer L1
      L1-=1;
      robot.setLongueurBras(L1, L2);
      Serial.print("L1 = ");
      Serial.println(L1);
    }
    else if (data == 'o'){
      //Augmenter L2
      L2+=1;
      robot.setLongueurBras(L1, L2);
      Serial.print("L2 = ");
      Serial.println(L2);
    }
    else if (data == 'l'){
      //Diminuer L2
      L2-=1;
      robot.setLongueurBras(L1, L2);
      Serial.print("L2 = ");
      Serial.println(L2);
    }
    else if(data == 't'){
      //Rotation relative de 90° de M3
      angles = robot.getAnglesActuelle();
      robot.rotationBras({angles.theta1, angles.theta2 + (PI_F/2.f)});
      Serial.print("theta2 = ");
      Serial.println(angles.theta1);
    }
    else if(data == 'g'){
      //Rotation relative de -90° de M3
      angles = robot.getAnglesActuelle();
      robot.rotationBras({angles.theta1, angles.theta2 - (PI_F/2.f)});
      Serial.print("theta2 = ");
      Serial.println(angles.theta2);
    }
     else if(data == 'u'){
      step_par_tour2 += 10;
      robot.setStepParTourBrasMoteurs(step_par_tour, step_par_tour2);
      Serial.print("step_par_tour2 = ");
      Serial.println(step_par_tour2);
    }
    else if(data == 'j'){
      step_par_tour2 -= 10;
      robot.setStepParTourBrasMoteurs(step_par_tour, step_par_tour2);
      Serial.print("step_par_tour2 = ");
      Serial.println(step_par_tour2);
    }
    else if(data == 'r'){
      //Rotation relative de 90° de M1
      angles = robot.getAnglesActuelle();
      robot.rotationBras({angles.theta1 + (PI_F/2.f), angles.theta2});
      Serial.print("theta1 = ");
      Serial.println(angles.theta1);
    }
    else if(data == 'f'){
      //Rotation relative de -90° de M1
      angles = robot.getAnglesActuelle();
      robot.rotationBras({angles.theta1 - (PI_F/2.f), angles.theta2});
      Serial.print("theta1 = ");
      Serial.println(angles.theta1);
    }
  }
}
//*/
void initialize_servo() {
  myservo.write(86);                 
  delay(1100);                   // le servo tourne jusqu'à la butée (la pince se ferme)
  myservo.write(106);
  delay(1000);                // la pince s'ouvre
  myservo.write(96);
  state_servo = "Open";       
}

void ouvrePince() {
    myservo.write(180);
    delay(500);
}

void fermePince() {
    myservo.write(30);
    delay(500);
}

// void retourDepart() {
//     steps[0] = 0;
//     steps[1] = 0;
//     steps[2] = 0;
//     steps[3] = 0;
//     steppersControl.moveTo(steps);
//     steppersControl.runSpeedToPosition();
// }

// Angles calculAngles(Point p) {
//   static Angles angles;
//   // resolution numérique
//   angles.theta2 = acos((sq(p.x) + sq(p.y) - sq(L1) - sq(L2)) / (2 * L1 * L2));
//   if (p.x < 0 && p.y < 0) {
//     angles.theta2 = (-1.) * angles.theta2;
//   }

//   angles.theta1 = atan(p.x / p.y) - atan((L2 * sin(angles.theta2)) / (L1 + L2 * cos(angles.theta2)));

//   angles.theta2 = (-1.) * angles.theta2 * RAD_TO_DEG;
//   angles.theta1 = angles.theta1 * RAD_TO_DEG;

//   // Angles adjustment depending in which quadrant the final tool coordinate x,y is
  
//   if (p.x >= 0 && p.y >= 0) {       // 1st quadrant
//     angles.theta1 = 90. - angles.theta1;
//   }
//   if (p.x < 0 && p.y > 0) {         // 2nd quadrant
//     angles.theta1 = 90. - angles.theta1;
//   }
//   /*
//   if (x < 0 && y < 0) {         // 3rd quadrant
//     angles.theta1 = 270 - angles.theta1;
//     phi = 270 - angles.theta1 - angles.theta2;
//     phi = (-1) * phi;
//   }
//   if (x > 0 && y < 0) {         // 4th quadrant
//     angles.theta1 = -90 - angles.theta1;
//   }
//   if (x < 0 && y == 0) {
//     angles.theta1 = 270 + angles.theta1;
//   }
//   */
//   // angles.theta1 = int(angles.theta1 * 16000./(360.));
//   // angles.theta2 = int(angles.theta2 * 51200./(360.));
//   return angles;
// }

// void robot.goToXy(Point consigne, Angles &angles_actuelle) { // y est toujours négatif
//   static Angles angles_obj;
  
//   angles_obj = calculAngles(consigne);

//   Serial.print("theta1 = "); Serial.println(angles_obj.theta1);
//   Serial.print("theta2 = "); Serial.println(angles_obj.theta2);

//   if (angles_obj.theta1 >= 0 && angles_obj.theta1 <= 180 && angles_obj.theta2 >= -495 && angles_obj.theta2 <= 495) {
//     theta1 = angles_obj.theta1;
//     theta2 = angles_obj.theta2;

//     // rotationMoteurs(angles_obj);

//     // M1.setCurrentPosition(0);
//     // M3.setCurrentPosition(0);

//     // angles_actuelle.theta1 = M1.currentPosition() * STEP_TO_DEG;
//     // angles_actuelle.theta2 = M3.currentPosition() * STEP_TO_DEG2;
//     Serial.print("Angles actuelle : "); Serial.print(angles_actuelle.theta1); Serial.print(" : "); Serial.println(angles_actuelle.theta2);

//     // Serial.println("position possible");
//   }
//   else {
//     Serial.println("position impossible");
//   }
// }

/*
Point pos_to_coor(int n, int m) {
  Point P;
  int i;
  int j;
  if (n <= 4) {
    if (m <= 4) { // quartier 1
      i = 3-n;
      j = 3-m;
  	  P.x = P1.x + i*l;
      P.y = P1.y + j*l;
    }
    else { // quartier 4
      i = 3-n;
      j = 6-m;
  	  P.x = P4.x + i*l;
      P.y = P4.y + j*l;
    }
  }
  else {
    if (m <= 4) { // quartier 2
      i = 6-n;
      j = 3-m;
  	  P.x = P2.x + i*l;
      P.y = P2.y + j*l;
    }
    else { // quartier 3
      i = 6-n;
      j = 6-m;
  	  P.x = P3.x + i*l;
      P.y = P3.y + j*l;
    }
  }
  return P;
}
*/

// int case_occupe = 0;

// void move_piece_from_to(Point pos1, Point pos2, Angles &angles_actuelle, int case_occupe) {
//   if (case_occupe == 1) {
//     ouvrePince();
//     delay(500);
//     robot.goToXy(pos2, angles_actuelle);
//     delay(500);
//     descendBras();
//     delay(500);
//     fermePince();
//     delay(500);
//     monteBras();
//     delay(500);
//     robot.goToXy({L1+L2, 0}, angles_actuelle);
//     delay(500);
//     ouvrePince();
//     delay(500);
//     robot.goToXy(pos1, angles_actuelle);
//     delay(500);
//     descendBras();
//     delay(500);
//     fermePince();
//     delay(500);
//     monteBras();
//     delay(500);
//     robot.goToXy(pos2, angles_actuelle);
//     delay(500);
//     descendBras();
//     delay(500);
//     ouvrePince();
//     delay(500);
//     monteBras0();
//     delay(500);
//     retourDepart();   
//   }
//   else {
//     ouvrePince();
//     delay(500);
//     robot.goToXy(pos1, angles_actuelle);
//     delay(500);
//     descendBras();
//     delay(500);
//     fermePince();
//     delay(500);
//     monteBras();
//     delay(500);
//     robot.goToXy(pos2, angles_actuelle);
//     delay(500);
//     descendBras();
//     delay(500);
//     ouvrePince();
//     delay(500);
//     monteBras0();
//     delay(500);
//     retourDepart();
//   }
// }

/*
void go_to_NN(int N1) {
  convert_coor(N1,33);
  Point PX = pos_to_coor(N11, N12);
  robot.goToXy(PX);
}
*/

/*
void all(int N1, int N2, int case_occupe_) {
  convert_coor(N1, N2);
  Point PA = pos_to_coor(N11, N12);
  Point PB = pos_to_coor(N21, N22);

  case_occupe = case_occupe_;

  move_piece_from_to({PA,PB});
}
*/