#ifndef SCARAROBOT_H
#define SCARAROBOT_H

#include <math.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

//RAD_TO_DEG existe déjà dans Arduino.h

#define PI_F            (3.1415926535897932384626433832795f) // PI en float
#define HALF_PI_F       (1.5707963267948966192313216916398f)
#define TWO_PI_F        (6.283185307179586476925286766559f)
#define _495DEG_TO_RAD  (8.6393797973719314057722693040186f)

#define SIZE_MOUV 65 // taille du tableau de mouvements

#define CHS_N 8 // nombre de colonnes/lignes de l'échiquier

#define CHS_A 0
#define CHS_B 1
#define CHS_C 2
#define CHS_D 3
#define CHS_E 4
#define CHS_F 5
#define CHS_G 6
#define CHS_H 7

#define CHS_1 0
#define CHS_2 1
#define CHS_3 2
#define CHS_4 3
#define CHS_5 4
#define CHS_6 5
#define CHS_7 6
#define CHS_8 7

#define CASE 2.8f // côté d'une case

typedef struct {
  int x;
  int y;
}Point;// En mm

typedef struct {
  float theta1;
  float theta2;
}Angles;//En radians

class Scararobot
{
public:
    Scararobot(int l1, int l2, AccelStepper *M1, AccelStepper *M2, AccelStepper *M3, AccelStepper *M4, Servo *servo);
    ~Scararobot();

    void loadChessboard(Point (*chessboard)[CHS_N]);
    // void displayChessboard();

    void rotationBras(Angles angles);

    //Position absolu en hauteur du bras. Positif monte et négatif descend
    void hauteurBras(long step);

    //En radian, absolu
    Angles getAnglesActuelle();
    //En step, absolu
    long getHauteurActuelle();

    
    void setLongueurBras(int l1, int l2){_L1 = l1; _L2 = l2;}
    //nb de step par tour pour les moteurs bras depandant des engrenages
    void setStepParTourBrasMoteurs(long stepParTour, long stepParTour2);

    void reglerSensMoteur(long sens){this->sens = (sens<0)?(-1l):(1l);}

    //Fonction à appeler dans la loop pour que le robot fonctionne. Renvoie true si un des moteurs est en mouvement
    bool run();

    void runSpeedToPosition();

    void goToXy(Point consigne);

    float STEP_TO_RAD = (TWO_PI_F/_step_par_tour);
    float STEP_TO_RAD2 = (TWO_PI_F/_step_par_tour2);
    long RAD_TO_STEP = (_step_par_tour/(TWO_PI_F));
    long RAD_TO_STEP2 = (_step_par_tour2/(TWO_PI_F));

private:
    AccelStepper *_M1, *_M2, *_M3, *_M4;
    Servo *_servo;
    MultiStepper bras, steppers_control;

    Point (*_chessboard)[CHS_N];// Point chessboard[CHS_N][CHS_N]

    // caractéristiques du bras
    long sens = -1; // sens direct : négatif / sens horaire : positif
    int _L1, _L2; //en mm

    long _step_par_tour = 16000; // nombre de pas par tour pour le moteurs bras M1
    long _step_par_tour2 = 54000; // nombre de pas par tour pour le moteurs bras M3

    //Donne des angles en radian
    Angles calculAngles(Point p);
};

#endif // SCARAROBOT_H