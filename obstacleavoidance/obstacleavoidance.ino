#include <FuzzyRule.h>
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>
#include <Encoder.h>
#include <NewPing.h>


#define LM1 8
#define LM2 9
#define RM1 4
#define RM2 5
#define EN1 6
#define EN2 7
#define ENCODERFACTOR 0.1
#define AXLE_LENGTH 10

#define T1  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define E1  11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define T2  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define E2  11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define T3  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define E3  11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1 (T1, E1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2 (T2, E2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar3 (T3, E3, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


Encoder leftEnc(2, 3);
Encoder rightEnc (18, 19);

long long prev_position_l = 0, curr_position_l = 0;
long long prev_position_r = 0, curr_position_r = 0;

float target_x = 400 , target_y = 400;
float curr_x = 0 , curr_y = 0;
float curr_theta = 0, prev_theta = 0;

float diff_angle = 100;
float dist_from_target = 200;


int dist1, dist2, dist3, dist4; // distances of all 4 ultrasonic sensors
int robotAngle = 0; // current Angle of the robot

void moveRobot(int targetVL, int targetVR)
{

  analogWrite(EN1, targetVL);
  analogWrite(EN2, targetVR);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
}

void getAllDistances()
{

  unsigned int uS1 = sonar1.ping();
  delay(33);
  unsigned int uS2 = sonar2.ping();
  delay(33);
  unsigned int uS3 = sonar3.ping();
  delay(33);

  dist1 = uS1 / US_ROUNDTRIP_CM;
  dist2 = uS2 / US_ROUNDTRIP_CM;
  dist3 = uS3 / US_ROUNDTRIP_CM;
}

void updateOdometry()
{
  curr_position_l = leftEnc.read();
  curr_position_r = rightEnc.read();

  float diff_l = curr_position_l - prev_position_l;
  float diff_r = curr_position_r - prev_position_r;

  float cos_current = cos(curr_theta);
  float sin_current = sin(curr_theta);

  if (diff_l == diff_r)
  {
    /* Moving in a straight line */
    curr_x += diff_l * cos_current;
    curr_y += diff_l * sin_current;
  }
  else
  {
    /* Moving in an arc */
    float expr1 = AXLE_LENGTH * (diff_r + diff_l) / (2.0 * (diff_r - diff_l));

    float right_minus_left = diff_r - diff_l;

    curr_x += expr1 * (sin(right_minus_left / AXLE_LENGTH + curr_theta) - sin_current);

    curr_y -= expr1 * (cos(right_minus_left / AXLE_LENGTH + curr_theta) - cos_current);

    /* Calculate new orientation */
    curr_theta += right_minus_left / AXLE_LENGTH;

    /* Keep in the range -PI to +PI */
    while (curr_theta > PI)
      curr_theta -= (2.0 * PI);
    while (curr_theta < -PI)
      curr_theta += (2.0 * PI);

    prev_position_l = curr_position_l;
    prev_position_r = curr_position_r;

  }

  float target_theta = atan2 (( target_y - curr_y), (target_x - curr_x));
  float diff_angle = target_theta - curr_theta;

  while (diff_angle > PI)
    diff_angle -= (2.0 * PI);
  while (diff_angle < -PI)
    diff_angle += (2.0 * PI);


}


// Step 1 -  Instantiating an object library
Fuzzy* oafuzzy = new Fuzzy();

void setup()
{

  Serial.begin(9600);

  pinMode(LM1, OUTPUT); // left motor input1
  pinMode(LM2, OUTPUT); // left motor input2
  pinMode(RM1, OUTPUT); // right motor input1
  pinMode(RM2, OUTPUT); // right motor input2
  pinMode(EN1, OUTPUT); // enable 1
  pinMode(EN2, OUTPUT); // enable 2

  //Creating a FuzzyInput distance
  FuzzyInput* odistance = new FuzzyInput(1);

  // Creating the FuzzySet to compond FuzzyInput distance
  FuzzySet* near = new FuzzySet(0, 0, 10, 15);
  odistance->addFuzzySet(near);
  FuzzySet* medium = new FuzzySet(0, 15, 15, 20);
  odistance->addFuzzySet(medium);
  FuzzySet* far = new FuzzySet(15, 20, 300, 300);
  odistance->addFuzzySet(far);

  // Creating the FuzzySet to compond FuzzyInput angle
  FuzzyInput* oangle = new FuzzyInput(2);

  FuzzySet* negbig = new FuzzySet(-180, -180, -50, -30);
  oangle->addFuzzySet(negbig);
  FuzzySet* negsmall = new FuzzySet(-45, -25, -25,  -5);
  oangle->addFuzzySet(negsmall);
  FuzzySet* zero = new FuzzySet(-7, 0, 0, 7);
  oangle->addFuzzySet(zero);
  FuzzySet* possmall = new FuzzySet(5, 25, 25, 45);
  oangle->addFuzzySet(possmall);
  FuzzySet* posbig = new FuzzySet(30, 50, 180, 180);
  oangle->addFuzzySet(posbig);

  oafuzzy->addFuzzyInput(odistance);
  oafuzzy->addFuzzyInput(oangle); // Add FuzzyInput to Fuzzy object

  // Creating FuzzyOutput velocity
  FuzzyOutput* vl = new FuzzyOutput(1);
  FuzzyOutput* vr = new FuzzyOutput(2);

  // Creating FuzzySet to compond FuzzyOutput vl (Left Wheel Velocity)

  FuzzySet* Lnegfast = new FuzzySet(-250, -200, -200, -150);
  FuzzySet* Lnegmed = new FuzzySet(-180, -140, -140, -100);
  FuzzySet* Lnegslow = new FuzzySet(-120, -90, -90, -60);
  FuzzySet* Lzero = new FuzzySet(-90, 0, 0, 90);
  FuzzySet* Lposslow = new FuzzySet(60, 90, 90, 120);
  FuzzySet* Lposmed = new FuzzySet(100, 140, 140, 180);
  FuzzySet* Lposfast = new FuzzySet(150, 200, 200, 250);
  
  
  FuzzySet* Rnegfast = new FuzzySet(-250, -200, -200, -150);
  FuzzySet* Rnegmed = new FuzzySet(-180, -140, -140, -100);
  FuzzySet* Rnegslow = new FuzzySet(-120, -90, -90, -60);
  FuzzySet* Rzero = new FuzzySet(-90, 0, 0, 90);
  FuzzySet* Rposslow = new FuzzySet(60, 90, 90, 120);
  FuzzySet* Rposmed = new FuzzySet(100, 140, 140, 180);
  FuzzySet* Rposfast = new FuzzySet(150, 200, 200, 250);

  // Creating FuzzySet to compond FuzzyOutput vr (Right Wheel Velocity)

  vl->addFuzzySet(Lnegfast);
  vl->addFuzzySet(Lnegmed);
  vl->addFuzzySet(Lnegslow);
  vl->addFuzzySet(Lzero);
  vl->addFuzzySet(Lposslow);
  vl->addFuzzySet(Lposmed);
  vl->addFuzzySet(Lposfast);

  vr->addFuzzySet(Rnegfast);
  vr->addFuzzySet(Rnegmed);
  vr->addFuzzySet(Rnegslow);
  vr->addFuzzySet(Rzero);
  vr->addFuzzySet(Rposslow);
  vr->addFuzzySet(Rposmed);
  vr->addFuzzySet(Rposfast);


  oafuzzy->addFuzzyOutput(vl);
  oafuzzy->addFuzzyOutput(vr);


  FuzzyRuleAntecedent*  PBN = new FuzzyRuleAntecedent();
  PBN->joinWithAND(posbig, near);
  FuzzyRuleAntecedent*  PBM = new FuzzyRuleAntecedent();
  PBM->joinWithAND(posbig, medium);
  FuzzyRuleAntecedent*  PBF = new FuzzyRuleAntecedent();
  PBF->joinWithAND(posbig, far);
  FuzzyRuleAntecedent*  PSN = new FuzzyRuleAntecedent();
  PSN->joinWithAND(posbig, near);
  FuzzyRuleAntecedent*  PSM = new FuzzyRuleAntecedent();
  PSM->joinWithAND(posbig, medium);
  FuzzyRuleAntecedent*  PSF = new FuzzyRuleAntecedent();
  PSF->joinWithAND(posbig, far);
  FuzzyRuleAntecedent*  ZN = new FuzzyRuleAntecedent();
  ZN->joinWithAND(posbig, near);
  FuzzyRuleAntecedent*  ZM = new FuzzyRuleAntecedent();
  ZM->joinWithAND(posbig, medium);
  FuzzyRuleAntecedent*  ZF = new FuzzyRuleAntecedent();
  ZF->joinWithAND(posbig, far);
  FuzzyRuleAntecedent*  NSN = new FuzzyRuleAntecedent();
  NSN->joinWithAND(posbig, near);
  FuzzyRuleAntecedent*  NSM = new FuzzyRuleAntecedent();
  NSM->joinWithAND(posbig, medium);
  FuzzyRuleAntecedent*  NSF = new FuzzyRuleAntecedent();
  NSF->joinWithAND(posbig, far);
  FuzzyRuleAntecedent*  NBN = new FuzzyRuleAntecedent();
  NBN->joinWithAND(posbig, near);
  FuzzyRuleAntecedent*  NBM = new FuzzyRuleAntecedent();
  NBM->joinWithAND(posbig, medium);
  FuzzyRuleAntecedent*  NBF = new FuzzyRuleAntecedent();
  NBF->joinWithAND(posbig, far);

  FuzzyRuleAntecedent*  PBFF = new FuzzyRuleAntecedent();
  PBFF->joinWithAND(PBF, far);
  FuzzyRuleAntecedent*  PBFM = new FuzzyRuleAntecedent();
  PBFM->joinWithAND(PBF, medium);
  FuzzyRuleAntecedent*  PBFN = new FuzzyRuleAntecedent();
  PBFN->joinWithAND(PBF, near);
  FuzzyRuleAntecedent*  PBMF = new FuzzyRuleAntecedent();
  PBMF->joinWithAND(PBM, far);
  FuzzyRuleAntecedent*  PBMM = new FuzzyRuleAntecedent();
  PBMM->joinWithAND(PBM, medium);
  FuzzyRuleAntecedent*  PBMN = new FuzzyRuleAntecedent();
  PBMN->joinWithAND(PBM, near);
  FuzzyRuleAntecedent*  PBNF = new FuzzyRuleAntecedent();
  PBNF->joinWithAND(PBN, far);
  FuzzyRuleAntecedent*  PBNM = new FuzzyRuleAntecedent();
  PBNM->joinWithAND(PBN, medium);
  FuzzyRuleAntecedent*  PBNN = new FuzzyRuleAntecedent();
  PBNN->joinWithAND(PBN, near);

  FuzzyRuleAntecedent*  PSFF = new FuzzyRuleAntecedent();
  PSFF->joinWithAND(PSF, far);
  FuzzyRuleAntecedent*  PSFM = new FuzzyRuleAntecedent();
  PSFM->joinWithAND(PSF, medium);
  FuzzyRuleAntecedent*  PSFN = new FuzzyRuleAntecedent();
  PSFN->joinWithAND(PSF, near);
  FuzzyRuleAntecedent*  PSMF = new FuzzyRuleAntecedent();
  PSMF->joinWithAND(PSM, far);
  FuzzyRuleAntecedent*  PSMM = new FuzzyRuleAntecedent();
  PSMM->joinWithAND(PSM, medium);
  FuzzyRuleAntecedent*  PSMN = new FuzzyRuleAntecedent();
  PSMN->joinWithAND(PSM, near);
  FuzzyRuleAntecedent*  PSNF = new FuzzyRuleAntecedent();
  PSNF->joinWithAND(PSN, far);
  FuzzyRuleAntecedent*  PSNM = new FuzzyRuleAntecedent();
  PSNM->joinWithAND(PSN, medium);
  FuzzyRuleAntecedent*  PSNN = new FuzzyRuleAntecedent();
  PSNN->joinWithAND(PSN, near);

  FuzzyRuleAntecedent*  ZFF = new FuzzyRuleAntecedent();
  ZFF->joinWithAND(ZF, far);
  FuzzyRuleAntecedent*  ZFM = new FuzzyRuleAntecedent();
  ZFM->joinWithAND(ZF, medium);
  FuzzyRuleAntecedent*  ZFN = new FuzzyRuleAntecedent();
  ZFN->joinWithAND(ZF, near);
  FuzzyRuleAntecedent*  ZMF = new FuzzyRuleAntecedent();
  ZMF->joinWithAND(ZM, far);
  FuzzyRuleAntecedent*  ZMM = new FuzzyRuleAntecedent();
  ZMM->joinWithAND(ZM, medium);
  FuzzyRuleAntecedent*  ZMN = new FuzzyRuleAntecedent();
  ZMN->joinWithAND(ZM, near);
  FuzzyRuleAntecedent*  ZNF = new FuzzyRuleAntecedent();
  ZNF->joinWithAND(ZN, far);
  FuzzyRuleAntecedent*  ZNM = new FuzzyRuleAntecedent();
  ZNM->joinWithAND(ZN, medium);
  FuzzyRuleAntecedent*  ZNN = new FuzzyRuleAntecedent();
  ZNN->joinWithAND(ZN, near);

  FuzzyRuleAntecedent*  NSFF = new FuzzyRuleAntecedent();
  NSFF->joinWithAND(NSF, far);
  FuzzyRuleAntecedent*  NSFM = new FuzzyRuleAntecedent();
  NSFM->joinWithAND(NSF, medium);
  FuzzyRuleAntecedent*  NSFN = new FuzzyRuleAntecedent();
  NSFN->joinWithAND(NSF, near);
  FuzzyRuleAntecedent*  NSMF = new FuzzyRuleAntecedent();
  NSMF->joinWithAND(NSM, far);
  FuzzyRuleAntecedent*  NSMM = new FuzzyRuleAntecedent();
  NSMM->joinWithAND(NSM, medium);
  FuzzyRuleAntecedent*  NSMN = new FuzzyRuleAntecedent();
  NSMN->joinWithAND(NSM, near);
  FuzzyRuleAntecedent*  NSNF = new FuzzyRuleAntecedent();
  NSNF->joinWithAND(NSN, far);
  FuzzyRuleAntecedent*  NSNM = new FuzzyRuleAntecedent();
  NSNM->joinWithAND(NSN, medium);
  FuzzyRuleAntecedent*  NSNN = new FuzzyRuleAntecedent();
  NSNN->joinWithAND(NSN, near);


  FuzzyRuleAntecedent*  NBFF = new FuzzyRuleAntecedent();
  NBFF->joinWithAND(NBF, far);
  FuzzyRuleAntecedent*  NBFM = new FuzzyRuleAntecedent();
  NBFM->joinWithAND(NBF, medium);
  FuzzyRuleAntecedent*  NBFN = new FuzzyRuleAntecedent();
  NBFN->joinWithAND(NBF, near);
  FuzzyRuleAntecedent*  NBMF = new FuzzyRuleAntecedent();
  NBMF->joinWithAND(NBM, far);
  FuzzyRuleAntecedent*  NBMM = new FuzzyRuleAntecedent();
  NBMM->joinWithAND(NBM, medium);
  FuzzyRuleAntecedent*  NBMN = new FuzzyRuleAntecedent();
  NBMN->joinWithAND(NBM, near);
  FuzzyRuleAntecedent*  NBNF = new FuzzyRuleAntecedent();
  NBNF->joinWithAND(NBN, far);
  FuzzyRuleAntecedent*  NBNM = new FuzzyRuleAntecedent();
  NBNM->joinWithAND(NBN, medium);
  FuzzyRuleAntecedent*  NBNN = new FuzzyRuleAntecedent();
  NBNN->joinWithAND(NBN, near);

  //Negative Big

  FuzzyRuleAntecedent*  NBFFF = new FuzzyRuleAntecedent();
  NBFFF->joinWithAND(NBFF, far);
  FuzzyRuleAntecedent*  NBFFM = new FuzzyRuleAntecedent();
  NBFFM->joinWithAND(NBFF, medium);
  FuzzyRuleAntecedent*  NBFFN = new FuzzyRuleAntecedent();
  NBFFN->joinWithAND(NBFF, near);
  FuzzyRuleAntecedent*  NBMMM = new FuzzyRuleAntecedent();
  NBMMM->joinWithAND(NBMM, medium);
  FuzzyRuleAntecedent*  NBMMN = new FuzzyRuleAntecedent();
  NBMMN->joinWithAND(NBMM, near);
  FuzzyRuleAntecedent*  NBMMF = new FuzzyRuleAntecedent();
  NBMMF->joinWithAND(NBMM, far);
  FuzzyRuleAntecedent*  NBNNF = new FuzzyRuleAntecedent();
  NBNNF->joinWithAND(NBNN, far);
  FuzzyRuleAntecedent*  NBNNM = new FuzzyRuleAntecedent();
  NBNNM->joinWithAND(NBNN, medium);
  FuzzyRuleAntecedent*  NBNNN = new FuzzyRuleAntecedent();
  NBNNN->joinWithAND(NBNN, near);
  FuzzyRuleAntecedent*  NBFMF = new FuzzyRuleAntecedent();
  NBFMF->joinWithAND(NBFM, far);
  FuzzyRuleAntecedent*  NBFMM = new FuzzyRuleAntecedent();
  NBFMM->joinWithAND(NBFM, medium);
  FuzzyRuleAntecedent*  NBFMN = new FuzzyRuleAntecedent();
  NBFMN->joinWithAND(NBFM, near);
  FuzzyRuleAntecedent*  NBFNF = new FuzzyRuleAntecedent();
  NBFNF->joinWithAND(NBFN, far);
  FuzzyRuleAntecedent*  NBFNM = new FuzzyRuleAntecedent();
  NBFNM->joinWithAND(NBFN, medium);
  FuzzyRuleAntecedent*  NBFNN = new FuzzyRuleAntecedent();
  NBFNN->joinWithAND(NBFN, near);
  FuzzyRuleAntecedent*  NBMFM = new FuzzyRuleAntecedent();
  NBMFM->joinWithAND(NBMF, medium);
  FuzzyRuleAntecedent*  NBMFN = new FuzzyRuleAntecedent();
  NBMFN->joinWithAND(NBMF, near);
  FuzzyRuleAntecedent*  NBMFF = new FuzzyRuleAntecedent();
  NBMFF->joinWithAND(NBMF, far);
  FuzzyRuleAntecedent*  NBMNF = new FuzzyRuleAntecedent();
  NBMNF->joinWithAND(NBMN, far);
  FuzzyRuleAntecedent*  NBMNM = new FuzzyRuleAntecedent();
  NBMNM->joinWithAND(NBMN, medium);
  FuzzyRuleAntecedent*  NBMNN = new FuzzyRuleAntecedent();
  NBMNN->joinWithAND(NBMN, near);
  FuzzyRuleAntecedent*  NBNMF = new FuzzyRuleAntecedent();
  NBNMF->joinWithAND(NBNM, far);
  FuzzyRuleAntecedent*  NBNMM = new FuzzyRuleAntecedent();
  NBNMM->joinWithAND(NBNM, medium);
  FuzzyRuleAntecedent*  NBNMN = new FuzzyRuleAntecedent();
  NBNMN->joinWithAND(NBNM, near);
  FuzzyRuleAntecedent*  NBNFF = new FuzzyRuleAntecedent();
  NBNFF->joinWithAND(NBNF, far);
  FuzzyRuleAntecedent*  NBNFM = new FuzzyRuleAntecedent();
  NBNFM->joinWithAND(NBNF, medium);
  FuzzyRuleAntecedent*  NBNFN = new FuzzyRuleAntecedent();
  NBNFN->joinWithAND(NBNF, near);


  // Negative Small

  FuzzyRuleAntecedent*  NSFFF = new FuzzyRuleAntecedent();
  NSFFF->joinWithAND(NSFF, far);
  FuzzyRuleAntecedent*  NSFFM = new FuzzyRuleAntecedent();
  NSFFM->joinWithAND(NSFF, medium);
  FuzzyRuleAntecedent*  NSFFN = new FuzzyRuleAntecedent();
  NSFFN->joinWithAND(NSFF, near);
  FuzzyRuleAntecedent*  NSMMM = new FuzzyRuleAntecedent();
  NSMMM->joinWithAND(NSMM, medium);
  FuzzyRuleAntecedent*  NSMMN = new FuzzyRuleAntecedent();
  NSMMN->joinWithAND(NSMM, near);
  FuzzyRuleAntecedent*  NSMMF = new FuzzyRuleAntecedent();
  NSMMF->joinWithAND(NSMM, far);
  FuzzyRuleAntecedent*  NSNNF = new FuzzyRuleAntecedent();
  NSNNF->joinWithAND(NSNN, far);
  FuzzyRuleAntecedent*  NSNNM = new FuzzyRuleAntecedent();
  NSNNM->joinWithAND(NSNN, medium);
  FuzzyRuleAntecedent*  NSNNN = new FuzzyRuleAntecedent();
  NSNNN->joinWithAND(NSNN, near);
  FuzzyRuleAntecedent*  NSFMF = new FuzzyRuleAntecedent();
  NSFMF->joinWithAND(NSFM, far);
  FuzzyRuleAntecedent*  NSFMM = new FuzzyRuleAntecedent();
  NSFMM->joinWithAND(NSFM, medium);
  FuzzyRuleAntecedent*  NSFMN = new FuzzyRuleAntecedent();
  NSFMN->joinWithAND(NSFM, near);
  FuzzyRuleAntecedent*  NSFNF = new FuzzyRuleAntecedent();
  NSFNF->joinWithAND(NSFN, far);
  FuzzyRuleAntecedent*  NSFNM = new FuzzyRuleAntecedent();
  NSFNM->joinWithAND(NSFN, medium);
  FuzzyRuleAntecedent*  NSFNN = new FuzzyRuleAntecedent();
  NSFNN->joinWithAND(NSFN, near);
  FuzzyRuleAntecedent*  NSMFM = new FuzzyRuleAntecedent();
  NSMFM->joinWithAND(NSMF, medium);
  FuzzyRuleAntecedent*  NSMFN = new FuzzyRuleAntecedent();
  NSMFN->joinWithAND(NSMF, near);
  FuzzyRuleAntecedent*  NSMFF = new FuzzyRuleAntecedent();
  NSMFF->joinWithAND(NSMF, far);
  FuzzyRuleAntecedent*  NSMNF = new FuzzyRuleAntecedent();
  NSMNF->joinWithAND(NSMN, far);
  FuzzyRuleAntecedent*  NSMNM = new FuzzyRuleAntecedent();
  NSMNM->joinWithAND(NSMN, medium);
  FuzzyRuleAntecedent*  NSMNN = new FuzzyRuleAntecedent();
  NSMNN->joinWithAND(NSMN, near);
  FuzzyRuleAntecedent*  NSNMF = new FuzzyRuleAntecedent();
  NSNMF->joinWithAND(NSNM, far);
  FuzzyRuleAntecedent*  NSNMM = new FuzzyRuleAntecedent();
  NSNMM->joinWithAND(NSNM, medium);
  FuzzyRuleAntecedent*  NSNMN = new FuzzyRuleAntecedent();
  NSNMN->joinWithAND(NSNM, near);
  FuzzyRuleAntecedent*  NSNFF = new FuzzyRuleAntecedent();
  NSNFF->joinWithAND(NSNF, far);
  FuzzyRuleAntecedent*  NSNFM = new FuzzyRuleAntecedent();
  NSNFM->joinWithAND(NSNF, medium);
  FuzzyRuleAntecedent*  NSNFN = new FuzzyRuleAntecedent();
  NSNFN->joinWithAND(NSNF, near);

  //  Zero

  FuzzyRuleAntecedent*  ZFFF = new FuzzyRuleAntecedent();
  ZFFF->joinWithAND(ZFF, far);
  FuzzyRuleAntecedent*  ZFFM = new FuzzyRuleAntecedent();
  ZFFM->joinWithAND(ZFF, medium);
  FuzzyRuleAntecedent*  ZFFN = new FuzzyRuleAntecedent();
  ZFFN->joinWithAND(ZFF, near);
  FuzzyRuleAntecedent*  ZMMM = new FuzzyRuleAntecedent();
  ZMMM->joinWithAND(ZMM, medium);
  FuzzyRuleAntecedent*  ZMMN = new FuzzyRuleAntecedent();
  ZMMN->joinWithAND(ZMM, near);
  FuzzyRuleAntecedent*  ZMMF = new FuzzyRuleAntecedent();
  ZMMF->joinWithAND(ZMM, far);
  FuzzyRuleAntecedent*  ZNNF = new FuzzyRuleAntecedent();
  ZNNF->joinWithAND(ZNN, far);
  FuzzyRuleAntecedent*  ZNNM = new FuzzyRuleAntecedent();
  ZNNM->joinWithAND(ZNN, medium);
  FuzzyRuleAntecedent*  ZNNN = new FuzzyRuleAntecedent();
  ZNNN->joinWithAND(ZNN, near);
  FuzzyRuleAntecedent*  ZFMF = new FuzzyRuleAntecedent();
  ZFMF->joinWithAND(ZFM, far);
  FuzzyRuleAntecedent*  ZFMM = new FuzzyRuleAntecedent();
  ZFMM->joinWithAND(ZFM, medium);
  FuzzyRuleAntecedent*  ZFMN = new FuzzyRuleAntecedent();
  ZFMN->joinWithAND(ZFM, near);
  FuzzyRuleAntecedent*  ZFNF = new FuzzyRuleAntecedent();
  ZFNF->joinWithAND(ZFN, far);
  FuzzyRuleAntecedent*  ZFNM = new FuzzyRuleAntecedent();
  ZFNM->joinWithAND(ZFN, medium);
  FuzzyRuleAntecedent*  ZFNN = new FuzzyRuleAntecedent();
  ZFNN->joinWithAND(ZFN, near);
  FuzzyRuleAntecedent*  ZMFM = new FuzzyRuleAntecedent();
  ZMFM->joinWithAND(ZMF, medium);
  FuzzyRuleAntecedent*  ZMFN = new FuzzyRuleAntecedent();
  ZMFN->joinWithAND(ZMF, near);
  FuzzyRuleAntecedent*  ZMFF = new FuzzyRuleAntecedent();
  ZMFF->joinWithAND(ZMF, far);
  FuzzyRuleAntecedent*  ZMNF = new FuzzyRuleAntecedent();
  ZMNF->joinWithAND(ZMN, far);
  FuzzyRuleAntecedent*  ZMNM = new FuzzyRuleAntecedent();
  ZMNM->joinWithAND(ZMN, medium);
  FuzzyRuleAntecedent*  ZMNN = new FuzzyRuleAntecedent();
  ZMNN->joinWithAND(ZMN, near);
  FuzzyRuleAntecedent*  ZNMF = new FuzzyRuleAntecedent();
  ZNMF->joinWithAND(ZNM, far);
  FuzzyRuleAntecedent*  ZNMM = new FuzzyRuleAntecedent();
  ZNMM->joinWithAND(ZNM, medium);
  FuzzyRuleAntecedent*  ZNMN = new FuzzyRuleAntecedent();
  ZNMN->joinWithAND(ZNM, near);
  FuzzyRuleAntecedent*  ZNFF = new FuzzyRuleAntecedent();
  ZNFF->joinWithAND(ZNF, far);
  FuzzyRuleAntecedent*  ZNFM = new FuzzyRuleAntecedent();
  ZNFM->joinWithAND(ZNF, medium);
  FuzzyRuleAntecedent*  ZNFN = new FuzzyRuleAntecedent();
  ZNFN->joinWithAND(ZNF, near);

  // Positive Small

  FuzzyRuleAntecedent*  PSFFF = new FuzzyRuleAntecedent();
  PSFFF->joinWithAND(PSFF, far);
  FuzzyRuleAntecedent*  PSFFM = new FuzzyRuleAntecedent();
  PSFFM->joinWithAND(PSFF, medium);
  FuzzyRuleAntecedent*  PSFFN = new FuzzyRuleAntecedent();
  PSFFN->joinWithAND(PSFF, near);
  FuzzyRuleAntecedent*  PSMMM = new FuzzyRuleAntecedent();
  PSMMM->joinWithAND(PSMM, medium);
  FuzzyRuleAntecedent*  PSMMN = new FuzzyRuleAntecedent();
  PSMMN->joinWithAND(PSMM, near);
  FuzzyRuleAntecedent*  PSMMF = new FuzzyRuleAntecedent();
  PSMMF->joinWithAND(PSMM, far);
  FuzzyRuleAntecedent*  PSNNF = new FuzzyRuleAntecedent();
  PSNNF->joinWithAND(PSNN, far);
  FuzzyRuleAntecedent*  PSNNM = new FuzzyRuleAntecedent();
  PSNNM->joinWithAND(PSNN, medium);
  FuzzyRuleAntecedent*  PSNNN = new FuzzyRuleAntecedent();
  PSNNN->joinWithAND(PSNN, near);
  FuzzyRuleAntecedent*  PSFMF = new FuzzyRuleAntecedent();
  PSFMF->joinWithAND(PSFM, far);
  FuzzyRuleAntecedent*  PSFMM = new FuzzyRuleAntecedent();
  PSFMM->joinWithAND(PSFM, medium);
  FuzzyRuleAntecedent*  PSFMN = new FuzzyRuleAntecedent();
  PSFMN->joinWithAND(PSFM, near);
  FuzzyRuleAntecedent*  PSFNF = new FuzzyRuleAntecedent();
  PSFNF->joinWithAND(PSFN, far);
  FuzzyRuleAntecedent*  PSFNM = new FuzzyRuleAntecedent();
  PSFNM->joinWithAND(PSFN, medium);
  FuzzyRuleAntecedent*  PSFNN = new FuzzyRuleAntecedent();
  PSFNN->joinWithAND(PSFN, near);
  FuzzyRuleAntecedent*  PSMFM = new FuzzyRuleAntecedent();
  PSMFM->joinWithAND(PSMF, medium);
  FuzzyRuleAntecedent*  PSMFN = new FuzzyRuleAntecedent();
  PSMFN->joinWithAND(PSMF, near);
  FuzzyRuleAntecedent*  PSMFF = new FuzzyRuleAntecedent();
  PSMFF->joinWithAND(PSMF, far);
  FuzzyRuleAntecedent*  PSMNF = new FuzzyRuleAntecedent();
  PSMNF->joinWithAND(PSMN, far);
  FuzzyRuleAntecedent*  PSMNM = new FuzzyRuleAntecedent();
  PSMNM->joinWithAND(PSMN, medium);
  FuzzyRuleAntecedent*  PSMNN = new FuzzyRuleAntecedent();
  PSMNN->joinWithAND(PSMN, near);
  FuzzyRuleAntecedent*  PSNMF = new FuzzyRuleAntecedent();
  PSNMF->joinWithAND(PSNM, far);
  FuzzyRuleAntecedent*  PSNMM = new FuzzyRuleAntecedent();
  PSNMM->joinWithAND(PSNM, medium);
  FuzzyRuleAntecedent*  PSNMN = new FuzzyRuleAntecedent();
  PSNMN->joinWithAND(PSNM, near);
  FuzzyRuleAntecedent*  PSNFF = new FuzzyRuleAntecedent();
  PSNFF->joinWithAND(PSNF, far);
  FuzzyRuleAntecedent*  PSNFM = new FuzzyRuleAntecedent();
  PSNFM->joinWithAND(PSNF, medium);
  FuzzyRuleAntecedent*  PSNFN = new FuzzyRuleAntecedent();
  PSNFN->joinWithAND(PSNF, near);

  // Positive Big

  FuzzyRuleAntecedent*  PBFFF = new FuzzyRuleAntecedent();
  PBFFF->joinWithAND(PBFF, far);
  FuzzyRuleAntecedent*  PBFFM = new FuzzyRuleAntecedent();
  PBFFM->joinWithAND(PBFF, medium);
  FuzzyRuleAntecedent*  PBFFN = new FuzzyRuleAntecedent();
  PBFFN->joinWithAND(PBFF, near);
  FuzzyRuleAntecedent*  PBMMM = new FuzzyRuleAntecedent();
  PBMMM->joinWithAND(PBMM, medium);
  FuzzyRuleAntecedent*  PBMMN = new FuzzyRuleAntecedent();
  PBMMN->joinWithAND(PBMM, near);
  FuzzyRuleAntecedent*  PBMMF = new FuzzyRuleAntecedent();
  PBMMF->joinWithAND(PBMM, far);
  FuzzyRuleAntecedent*  PBNNF = new FuzzyRuleAntecedent();
  PBNNF->joinWithAND(PBNN, far);
  FuzzyRuleAntecedent*  PBNNM = new FuzzyRuleAntecedent();
  PBNNM->joinWithAND(PBNN, medium);
  FuzzyRuleAntecedent*  PBNNN = new FuzzyRuleAntecedent();
  PBNNN->joinWithAND(PBNN, near);
  FuzzyRuleAntecedent*  PBFMF = new FuzzyRuleAntecedent();
  PBFMF->joinWithAND(PBFM, far);
  FuzzyRuleAntecedent*  PBFMM = new FuzzyRuleAntecedent();
  PBFMM->joinWithAND(PBFM, medium);
  FuzzyRuleAntecedent*  PBFMN = new FuzzyRuleAntecedent();
  PBFMN->joinWithAND(PBFM, near);
  FuzzyRuleAntecedent*  PBFNF = new FuzzyRuleAntecedent();
  PBFNF->joinWithAND(PBFN, far);
  FuzzyRuleAntecedent*  PBFNM = new FuzzyRuleAntecedent();
  PBFNM->joinWithAND(PBFN, medium);
  FuzzyRuleAntecedent*  PBFNN = new FuzzyRuleAntecedent();
  PBFNN->joinWithAND(PBFN, near);
  FuzzyRuleAntecedent*  PBMFM = new FuzzyRuleAntecedent();
  PBMFM->joinWithAND(PBMF, medium);
  FuzzyRuleAntecedent*  PBMFN = new FuzzyRuleAntecedent();
  PBMFN->joinWithAND(PBMF, near);
  FuzzyRuleAntecedent*  PBMFF = new FuzzyRuleAntecedent();
  PBMFF->joinWithAND(PBMF, far);
  FuzzyRuleAntecedent*  PBMNF = new FuzzyRuleAntecedent();
  PBMNF->joinWithAND(PBMN, far);
  FuzzyRuleAntecedent*  PBMNM = new FuzzyRuleAntecedent();
  PBMNM->joinWithAND(PBMN, medium);
  FuzzyRuleAntecedent*  PBMNN = new FuzzyRuleAntecedent();
  PBMNN->joinWithAND(PBMN, near);
  FuzzyRuleAntecedent*  PBNMF = new FuzzyRuleAntecedent();
  PBNMF->joinWithAND(PBNM, far);
  FuzzyRuleAntecedent*  PBNMM = new FuzzyRuleAntecedent();
  PBNMM->joinWithAND(PBNM, medium);
  FuzzyRuleAntecedent*  PBNMN = new FuzzyRuleAntecedent();
  PBNMN->joinWithAND(PBNM, near);
  FuzzyRuleAntecedent*  PBNFF = new FuzzyRuleAntecedent();
  PBNFF->joinWithAND(PBNF, far);
  FuzzyRuleAntecedent*  PBNFM = new FuzzyRuleAntecedent();
  PBNFM->joinWithAND(PBNF, medium);
  FuzzyRuleAntecedent*  PBNFN = new FuzzyRuleAntecedent();
  PBNFN->joinWithAND(PBNF, near);




  FuzzyRuleConsequent* PFPF = new FuzzyRuleConsequent();
  PFPF->addOutput(Lposfast);
  PFPF->addOutput(Rposfast);
  FuzzyRuleConsequent* PFPM = new FuzzyRuleConsequent();
  PFPM->addOutput(Lposfast);
  PFPM->addOutput(Rposmed);
  FuzzyRuleConsequent* PFPS = new FuzzyRuleConsequent();
  PFPS->addOutput(Lposfast);
  PFPS->addOutput(Rposslow);
  FuzzyRuleConsequent* PFZ = new FuzzyRuleConsequent();
  PFZ->addOutput(Lposfast);
  PFZ->addOutput(Rzero);
  FuzzyRuleConsequent* PFNS = new FuzzyRuleConsequent();
  PFNS->addOutput(Lposfast);
  PFNS->addOutput(Rnegslow);
  FuzzyRuleConsequent* PFNM = new FuzzyRuleConsequent();
  PFNM->addOutput(Lposfast);
  PFNM->addOutput(Rnegslow);
  FuzzyRuleConsequent* PFNF = new FuzzyRuleConsequent();
  PFNF->addOutput(Lposfast);
  PFNF->addOutput(Rnegfast);


  FuzzyRuleConsequent* PMPF = new FuzzyRuleConsequent();
  PMPF->addOutput(Lposmed);
  PMPF->addOutput(Rposfast);
  FuzzyRuleConsequent* PMPM = new FuzzyRuleConsequent();
  PMPM->addOutput(Lposmed);
  PMPM->addOutput(Rposmed);
  FuzzyRuleConsequent* PMPS = new FuzzyRuleConsequent();
  PMPS->addOutput(Lposmed);
  PMPS->addOutput(Rposslow);
  FuzzyRuleConsequent* PMZ = new FuzzyRuleConsequent();
  PMZ->addOutput(Lposmed);
  PMZ->addOutput(Rzero);
  FuzzyRuleConsequent* PMNS = new FuzzyRuleConsequent();
  PMNS->addOutput(Lposmed);
  PMNS->addOutput(Rnegslow);
  FuzzyRuleConsequent* PMNM = new FuzzyRuleConsequent();
  PMNM->addOutput(Lposmed);
  PMNM->addOutput(Rnegslow);
  FuzzyRuleConsequent* PMNF = new FuzzyRuleConsequent();
  PMNF->addOutput(Lposmed);
  PMNF->addOutput(Rnegfast);


  FuzzyRuleConsequent* PSPF = new FuzzyRuleConsequent();
  PSPF->addOutput(Lposmed);
  PSPF->addOutput(Rposfast);
  FuzzyRuleConsequent* PSPM = new FuzzyRuleConsequent();
  PSPM->addOutput(Lposmed);
  PSPM->addOutput(Rposmed);
  FuzzyRuleConsequent* PSPS = new FuzzyRuleConsequent();
  PSPS->addOutput(Lposmed);
  PSPS->addOutput(Rposslow);
  FuzzyRuleConsequent* PSZ = new FuzzyRuleConsequent();
  PSZ->addOutput(Lposmed);
  PSZ->addOutput(Rzero);
  FuzzyRuleConsequent* PSNS = new FuzzyRuleConsequent();
  PSNS->addOutput(Lposmed);
  PSNS->addOutput(Rnegslow);
  FuzzyRuleConsequent* PSNMC = new FuzzyRuleConsequent();
  PSNMC->addOutput(Lposmed);
  PSNMC->addOutput(Rnegslow);
  FuzzyRuleConsequent* PSNFC = new FuzzyRuleConsequent();
  PSNFC->addOutput(Lposmed);
  PSNFC->addOutput(Rnegfast);



  FuzzyRuleConsequent* ZPF = new FuzzyRuleConsequent();
  ZPF->addOutput(Lzero);
  ZPF->addOutput(Rposfast);
  FuzzyRuleConsequent* ZPM = new FuzzyRuleConsequent();
  ZPM->addOutput(Lzero);
  ZPM->addOutput(Rposmed);
  FuzzyRuleConsequent* ZPS = new FuzzyRuleConsequent();
  ZPS->addOutput(Lzero);
  ZPS->addOutput(Rposslow);
  FuzzyRuleConsequent* ZZ = new FuzzyRuleConsequent();
  ZZ->addOutput(Lzero);
  ZZ->addOutput(Rzero);
  FuzzyRuleConsequent* ZNS = new FuzzyRuleConsequent();
  ZNS->addOutput(Lzero);
  ZNS->addOutput(Rnegslow);
  FuzzyRuleConsequent* ZNMC = new FuzzyRuleConsequent();
  ZNMC->addOutput(Lzero);
  ZNMC->addOutput(Rnegslow);
  FuzzyRuleConsequent* ZNFC = new FuzzyRuleConsequent();
  ZNFC->addOutput(Lzero);
  ZNFC->addOutput(Rnegfast);


  FuzzyRuleConsequent* NSPF = new FuzzyRuleConsequent();
  NSPF->addOutput(Lnegslow);
  NSPF->addOutput(Rposfast);
  FuzzyRuleConsequent* NSPM = new FuzzyRuleConsequent();
  NSPM->addOutput(Lnegslow);
  NSPM->addOutput(Rposmed);
  FuzzyRuleConsequent* NSPL = new FuzzyRuleConsequent();
  NSPL->addOutput(Lnegslow);
  NSPL->addOutput(Rposslow);
  FuzzyRuleConsequent* NSZ = new FuzzyRuleConsequent();
  NSZ->addOutput(Lnegslow);
  NSZ->addOutput(Rzero);
  FuzzyRuleConsequent* NSNS = new FuzzyRuleConsequent();
  NSNS->addOutput(Lnegslow);
  NSNS->addOutput(Rnegslow);
  FuzzyRuleConsequent* NSNMC = new FuzzyRuleConsequent();
  NSNMC->addOutput(Lnegslow);
  NSNMC->addOutput(Rnegslow);
  FuzzyRuleConsequent* NSNFC = new FuzzyRuleConsequent();
  NSNFC->addOutput(Lnegslow);
  NSNFC->addOutput(Rnegfast);


  FuzzyRuleConsequent* NMPF = new FuzzyRuleConsequent();
  NMPF->addOutput(Lnegmed);
  NMPF->addOutput(Rposfast);
  FuzzyRuleConsequent* NMPM = new FuzzyRuleConsequent();
  NMPM->addOutput(Lnegmed);
  NMPM->addOutput(Rposmed);
  FuzzyRuleConsequent* NMPS = new FuzzyRuleConsequent();
  NMPS->addOutput(Lnegmed);
  NMPS->addOutput(Rposslow);
  FuzzyRuleConsequent* NMZ = new FuzzyRuleConsequent();
  NMZ->addOutput(Lnegmed);
  NMZ->addOutput(Rzero);
  FuzzyRuleConsequent* NMNS = new FuzzyRuleConsequent();
  NMNS->addOutput(Lnegmed);
  NMNS->addOutput(Rnegslow);
  FuzzyRuleConsequent* NMNM = new FuzzyRuleConsequent();
  NMNM->addOutput(Lnegmed);
  NMNM->addOutput(Rnegslow);
  FuzzyRuleConsequent* NMNF = new FuzzyRuleConsequent();
  NMNF->addOutput(Lnegmed);
  NMNF->addOutput(Rnegfast);


  FuzzyRuleConsequent* NFPF = new FuzzyRuleConsequent();
  NFPF->addOutput(Lnegfast);
  NFPF->addOutput(Rposfast);
  FuzzyRuleConsequent* NFPM = new FuzzyRuleConsequent();
  NFPM->addOutput(Lnegfast);
  NFPM->addOutput(Rposmed);
  FuzzyRuleConsequent* NFPS = new FuzzyRuleConsequent();
  NFPS->addOutput(Lnegfast);
  NFPS->addOutput(Rposslow);
  FuzzyRuleConsequent* NFZ = new FuzzyRuleConsequent();
  NFZ->addOutput(Lposfast);
  NFZ->addOutput(Rzero);
  FuzzyRuleConsequent* NFNS = new FuzzyRuleConsequent();
  NFNS->addOutput(Lnegfast);
  NFNS->addOutput(Rnegslow);
  FuzzyRuleConsequent* NFNM = new FuzzyRuleConsequent();
  NFNM->addOutput(Lnegfast);
  NFNM->addOutput(Rnegslow);
  FuzzyRuleConsequent* NFNF = new FuzzyRuleConsequent();
  NFNF->addOutput(Lnegfast);
  NFNF->addOutput(Rnegfast);



  FuzzyRule* fuzzyRule01 = new FuzzyRule(1,  PBFFF, PMPM    );
  FuzzyRule* fuzzyRule02 = new FuzzyRule(2,  PSFFF, PMPM    );
  FuzzyRule* fuzzyRule03 = new FuzzyRule(3,  ZFFF,  PMPM    );
  FuzzyRule* fuzzyRule04 = new FuzzyRule(4,  NSFFF, PMPM    );
  FuzzyRule* fuzzyRule05 = new FuzzyRule(5,  NBFFF, PMPM    );
  FuzzyRule* fuzzyRule06 = new FuzzyRule(6,  PBFMF, NFPF    );
  FuzzyRule* fuzzyRule07 = new FuzzyRule(7,  PSFMF, NSPF    );
  FuzzyRule* fuzzyRule08 = new FuzzyRule(8,  ZFMF,   NSPF   );
  FuzzyRule* fuzzyRule09 = new FuzzyRule(9,  NSFMF,  PFNS   );
  FuzzyRule* fuzzyRule10 = new FuzzyRule(10, NBFMF,  PFNM   );
  FuzzyRule* fuzzyRule11 = new FuzzyRule(11, PBFNF,  NFPF   );
  FuzzyRule* fuzzyRule12 = new FuzzyRule(12, PSFNF,  NFPF   );
  FuzzyRule* fuzzyRule13 = new FuzzyRule(13, ZFNF,   NFPF   );
  FuzzyRule* fuzzyRule14 = new FuzzyRule(14, NSFNF,  PFNF   );
  FuzzyRule* fuzzyRule15 = new FuzzyRule(15, NBFNF,  PFNF   );
  FuzzyRule* fuzzyRule16 = new FuzzyRule(16, PBMFF,  PMPM   );
  FuzzyRule* fuzzyRule17 = new FuzzyRule(17, PSMFF,  PSPS   );
  FuzzyRule* fuzzyRule18 = new FuzzyRule(18, ZMFF,   PMPM   );
  FuzzyRule* fuzzyRule19 = new FuzzyRule(19, NSMFF,  PMPM   );
  FuzzyRule* fuzzyRule20 = new FuzzyRule(20, NBMFF,  PMPM   );
  FuzzyRule* fuzzyRule21 = new FuzzyRule(21, PBNFF,  PMPM   );
  FuzzyRule* fuzzyRule22 = new FuzzyRule(22, PSNFF,  PSPS   );
  FuzzyRule* fuzzyRule23 = new FuzzyRule(23, ZNFF,   PMPM   );
  FuzzyRule* fuzzyRule24 = new FuzzyRule(24, NSNFF,  PMPM   );
  FuzzyRule* fuzzyRule25 = new FuzzyRule(25, NBNFF,  PMPM   );
  FuzzyRule* fuzzyRule26 = new FuzzyRule(26, PBFFM,  PMPM   );
  FuzzyRule* fuzzyRule27 = new FuzzyRule(27, PSFFM,  PMPM   );
  FuzzyRule* fuzzyRule28 = new FuzzyRule(28, ZFFM,   PMPM   );
  FuzzyRule* fuzzyRule29 = new FuzzyRule(29, NSFFM,  PSPS   );
  FuzzyRule* fuzzyRule30 = new FuzzyRule(30, NBFFM,  PMPM   );
  FuzzyRule* fuzzyRule31 = new FuzzyRule(31, PBFFN,  PMPM   );
  FuzzyRule* fuzzyRule32 = new FuzzyRule(32, PSFFN,  PMPM   );
  FuzzyRule* fuzzyRule33 = new FuzzyRule(33, ZFFN,   PMPM   );
  FuzzyRule* fuzzyRule34 = new FuzzyRule(34, NSFFN,  PSPS   );
  FuzzyRule* fuzzyRule35 = new FuzzyRule(35, NBFFN,  PMPM   );
  FuzzyRule* fuzzyRule36 = new FuzzyRule(36,  PBFMM, PMPM   );
  FuzzyRule* fuzzyRule37 = new FuzzyRule(37,  PSFMM, PMPM   );
  FuzzyRule* fuzzyRule38 = new FuzzyRule(38,  ZFMM,  PMPM   );
  FuzzyRule* fuzzyRule39 = new FuzzyRule(39,  NSFMM, PMPM   );
  FuzzyRule* fuzzyRule40 = new FuzzyRule(40,  NBFMM, PMPM   );
  FuzzyRule* fuzzyRule41 = new FuzzyRule(41,  PBFNM, NFPF   );
  FuzzyRule* fuzzyRule42 = new FuzzyRule(42,  PSFNM, NFPF   );
  FuzzyRule* fuzzyRule43 = new FuzzyRule(43,  ZFNM,  NMPM   );
  FuzzyRule* fuzzyRule44 = new FuzzyRule(44,  NSFNM, PMNM   );
  FuzzyRule* fuzzyRule45 = new FuzzyRule(45,  NBFNM, PFNF   );
  FuzzyRule* fuzzyRule46 = new FuzzyRule(46,  PBFMN, PMPM   );
  FuzzyRule* fuzzyRule47 = new FuzzyRule(47,  PSFMN, PMPM   );
  FuzzyRule* fuzzyRule48 = new FuzzyRule(48,  ZFMN,  ZPM    );
  FuzzyRule* fuzzyRule49 = new FuzzyRule(49,  NSFMN, ZPS    );
  FuzzyRule* fuzzyRule50 = new FuzzyRule(50,  NBFMN, PSPS   );
  FuzzyRule* fuzzyRule51 = new FuzzyRule(51,  PBFNN, NFPF   );
  FuzzyRule* fuzzyRule52 = new FuzzyRule(52,  PSFNN, NFPF   );
  FuzzyRule* fuzzyRule53 = new FuzzyRule(53,  ZFNN,  NFPF   );
  FuzzyRule* fuzzyRule54 = new FuzzyRule(54,  NSFNN, NFPF   );
  FuzzyRule* fuzzyRule55 = new FuzzyRule(55,  NBFNN, NFPF   );
  FuzzyRule* fuzzyRule56 = new FuzzyRule(56,  PBMFM,  PMPM   );
  FuzzyRule* fuzzyRule57 = new FuzzyRule(57,  PSMFM,  PMPM   );
  FuzzyRule* fuzzyRule58 = new FuzzyRule(58,  ZMFM,   PMPM   );
  FuzzyRule* fuzzyRule59 = new FuzzyRule(59,  NSMFM,  PMPM   );
  FuzzyRule* fuzzyRule60 = new FuzzyRule(60,  NBMFM,  PMPM   );
  FuzzyRule* fuzzyRule61 = new FuzzyRule(61,  PBMFN,  PMPM   );
  FuzzyRule* fuzzyRule62 = new FuzzyRule(62,  PSMFN,  PMPM   );
  FuzzyRule* fuzzyRule63 = new FuzzyRule(63,  ZMFN,   PMPM   );
  FuzzyRule* fuzzyRule64 = new FuzzyRule(64,  NSMFN,  PMPM   );
  FuzzyRule* fuzzyRule65 = new FuzzyRule(65,  NBMFN,  PMPM   );
  FuzzyRule* fuzzyRule66 = new FuzzyRule(66, PBMMF,   PMPM   );
  FuzzyRule* fuzzyRule67 = new FuzzyRule(67, PSMMF,   PMPM   );
  FuzzyRule* fuzzyRule68 = new FuzzyRule(68, ZMMF,    PMPM   );
  FuzzyRule* fuzzyRule69 = new FuzzyRule(69, NSMMF,   PMPM   );
  FuzzyRule* fuzzyRule70 = new FuzzyRule(70, NBMMF,   PMPM   );
  FuzzyRule* fuzzyRule71 = new FuzzyRule(71,  PBMMM,  PMPM   );
  FuzzyRule* fuzzyRule72 = new FuzzyRule(72,  PSMMM,  PMPM   );
  FuzzyRule* fuzzyRule73 = new FuzzyRule(73,  ZMMM,   PMPM   );
  FuzzyRule* fuzzyRule74 = new FuzzyRule(74,  NSMMM,  PMPM   );
  FuzzyRule* fuzzyRule75 = new FuzzyRule(75,  NBMMM,  PMPM   );
  FuzzyRule* fuzzyRule76 = new FuzzyRule(76,  PBMMN,  PMPM   );
  FuzzyRule* fuzzyRule77 = new FuzzyRule(77,  PSMMN,  PMPM   );
  FuzzyRule* fuzzyRule78 = new FuzzyRule(78,  ZMMN,   PMPM   );
  FuzzyRule* fuzzyRule79 = new FuzzyRule(79,  NSMMN,  PSPS   );
  FuzzyRule* fuzzyRule80 = new FuzzyRule(80,  NBMMN,  PSPS   );
  FuzzyRule* fuzzyRule81 = new FuzzyRule(81,  PBMNF,  PFNF   );
  FuzzyRule* fuzzyRule82 = new FuzzyRule(82,  PSMNF,  PFNF   );
  FuzzyRule* fuzzyRule83 = new FuzzyRule(83,  ZMNF,   PMNM   );
  FuzzyRule* fuzzyRule84 = new FuzzyRule(84,  NSMNF,  NMPM   );
  FuzzyRule* fuzzyRule85 = new FuzzyRule(85,  NBMNF,  NFPF   );
  FuzzyRule* fuzzyRule86 = new FuzzyRule(86,  PBMNM,  PMPM   );
  FuzzyRule* fuzzyRule87 = new FuzzyRule(87,  PSMNM,  NMPM   );
  FuzzyRule* fuzzyRule88 = new FuzzyRule(88,  ZMNM,   NMPM   );
  FuzzyRule* fuzzyRule89 = new FuzzyRule(89,  NSMNM,  PMNM   );
  FuzzyRule* fuzzyRule90 = new FuzzyRule(90,  NBMNM,  PMPM   );
  FuzzyRule* fuzzyRule91 = new FuzzyRule(91,  PBMNN,  NFPF   );
  FuzzyRule* fuzzyRule92 = new FuzzyRule(92,  PSMNN,  NFPF   );
  FuzzyRule* fuzzyRule93 = new FuzzyRule(93,  ZMNN,   PMNM   );
  FuzzyRule* fuzzyRule94 = new FuzzyRule(94,  NSMNN,  NMPM   );
  FuzzyRule* fuzzyRule95 = new FuzzyRule(95,  NBMNN,  NFPF   );
  FuzzyRule* fuzzyRule96 = new FuzzyRule(96,  PBNFM,  PMPM   );
  FuzzyRule* fuzzyRule97 = new FuzzyRule(97,  PSNFM,  PMPM   );
  FuzzyRule* fuzzyRule98 = new FuzzyRule(98,  ZNFM,   PMPM   );
  FuzzyRule* fuzzyRule99 = new FuzzyRule(99,  NSNFM,  PSPS   );
  FuzzyRule* fuzzyRule100 = new FuzzyRule(100, NBNFM,  PMPM   );
  FuzzyRule* fuzzyRule101 = new FuzzyRule(101, PBNFN, PMPM   );
  FuzzyRule* fuzzyRule102 = new FuzzyRule(102, PSNFN, PMPM   );
  FuzzyRule* fuzzyRule103 = new FuzzyRule(103, ZNFN,  PMPM   );
  FuzzyRule* fuzzyRule104 = new FuzzyRule(104, NSNFN, PMPM   );
  FuzzyRule* fuzzyRule105 = new FuzzyRule(105, NBNFN, PMPM   );
  FuzzyRule* fuzzyRule106 = new FuzzyRule(106, PBNMF, PSPS   );
  FuzzyRule* fuzzyRule107 = new FuzzyRule(107, PSNMF, PSZ    );
  FuzzyRule* fuzzyRule108 = new FuzzyRule(108, ZNMF,  PMZ    );
  FuzzyRule* fuzzyRule109 = new FuzzyRule(109, NSNMF, PMPM   );
  FuzzyRule* fuzzyRule110 = new FuzzyRule(110, NBNMF, PMPM   );
  FuzzyRule* fuzzyRule111 = new FuzzyRule(111, PBNMM, PSPS   );
  FuzzyRule* fuzzyRule112 = new FuzzyRule(112, PSNMM, PSZ    );
  FuzzyRule* fuzzyRule113 = new FuzzyRule(113, ZNMM,  PMZ    );
  FuzzyRule* fuzzyRule114 = new FuzzyRule(114, NSNMM, PMPM   );
  FuzzyRule* fuzzyRule115 = new FuzzyRule(115, NBNMM, PMPM   );
  FuzzyRule* fuzzyRule116 = new FuzzyRule(116, PBNMN, PSPS   );
  FuzzyRule* fuzzyRule117 = new FuzzyRule(117, PSNMN, PSPS   );
  FuzzyRule* fuzzyRule118 = new FuzzyRule(118, ZNMN,  PSPS   );
  FuzzyRule* fuzzyRule119 = new FuzzyRule(119, NSNMN, PSPS   );
  FuzzyRule* fuzzyRule120 = new FuzzyRule(120, NBNMN, PSPS   );
  FuzzyRule* fuzzyRule121 = new FuzzyRule(121, PBNNF, PFNF   );
  FuzzyRule* fuzzyRule122 = new FuzzyRule(122, PSNNF, PFNF   );
  FuzzyRule* fuzzyRule123 = new FuzzyRule(123, ZNNF,  PFNF   );
  FuzzyRule* fuzzyRule124 = new FuzzyRule(124, NSNNF, PFNF   );
  FuzzyRule* fuzzyRule125 = new FuzzyRule(125, NBNNF, PFNF   );
  FuzzyRule* fuzzyRule126 = new FuzzyRule(126, PBNNM, PFNF   );
  FuzzyRule* fuzzyRule127 = new FuzzyRule(127, PSNNM, PFNF   );
  FuzzyRule* fuzzyRule128 = new FuzzyRule(128, ZNNM,  PFNF   );
  FuzzyRule* fuzzyRule129 = new FuzzyRule(129, NSNNM, PFNF   );
  FuzzyRule* fuzzyRule130 = new FuzzyRule(130, NBNNM, PFNF   );
  FuzzyRule* fuzzyRule131 = new FuzzyRule(131, PBNNN, NMNM   );
  FuzzyRule* fuzzyRule132 = new FuzzyRule(132, PSNNN, NMNM   );
  FuzzyRule* fuzzyRule133 = new FuzzyRule(133, ZNNN,  NMNM   );
  FuzzyRule* fuzzyRule134 = new FuzzyRule(134, NSNNN, NMNM   );
  FuzzyRule* fuzzyRule135 = new FuzzyRule(135, NBNNN, NMNM   );


  oafuzzy->addFuzzyRule(fuzzyRule01);
  oafuzzy->addFuzzyRule(fuzzyRule02);
  oafuzzy->addFuzzyRule(fuzzyRule03);
  oafuzzy->addFuzzyRule(fuzzyRule04);
  oafuzzy->addFuzzyRule(fuzzyRule05);
  oafuzzy->addFuzzyRule(fuzzyRule06);
  oafuzzy->addFuzzyRule(fuzzyRule07);
  oafuzzy->addFuzzyRule(fuzzyRule08);
  oafuzzy->addFuzzyRule(fuzzyRule09);
  oafuzzy->addFuzzyRule(fuzzyRule10);
  oafuzzy->addFuzzyRule(fuzzyRule11);
  oafuzzy->addFuzzyRule(fuzzyRule12);
  oafuzzy->addFuzzyRule(fuzzyRule13);
  oafuzzy->addFuzzyRule(fuzzyRule14);
  oafuzzy->addFuzzyRule(fuzzyRule15);
  oafuzzy->addFuzzyRule(fuzzyRule16);
  oafuzzy->addFuzzyRule(fuzzyRule17);
  oafuzzy->addFuzzyRule(fuzzyRule18);
  oafuzzy->addFuzzyRule(fuzzyRule19);
  oafuzzy->addFuzzyRule(fuzzyRule20);
  oafuzzy->addFuzzyRule(fuzzyRule21);
  oafuzzy->addFuzzyRule(fuzzyRule22);
  oafuzzy->addFuzzyRule(fuzzyRule23);
  oafuzzy->addFuzzyRule(fuzzyRule24);
  oafuzzy->addFuzzyRule(fuzzyRule25);
  oafuzzy->addFuzzyRule(fuzzyRule26);
  oafuzzy->addFuzzyRule(fuzzyRule27);
  oafuzzy->addFuzzyRule(fuzzyRule28);
  oafuzzy->addFuzzyRule(fuzzyRule29);
  oafuzzy->addFuzzyRule(fuzzyRule30);
  oafuzzy->addFuzzyRule(fuzzyRule31);
  oafuzzy->addFuzzyRule(fuzzyRule32);
  oafuzzy->addFuzzyRule(fuzzyRule33);
  oafuzzy->addFuzzyRule(fuzzyRule34);
  oafuzzy->addFuzzyRule(fuzzyRule35);
  oafuzzy->addFuzzyRule(fuzzyRule36);
  oafuzzy->addFuzzyRule(fuzzyRule37);
  oafuzzy->addFuzzyRule(fuzzyRule38);
  oafuzzy->addFuzzyRule(fuzzyRule39);
  oafuzzy->addFuzzyRule(fuzzyRule40);
  oafuzzy->addFuzzyRule(fuzzyRule41);
  oafuzzy->addFuzzyRule(fuzzyRule42);
  oafuzzy->addFuzzyRule(fuzzyRule43);
  oafuzzy->addFuzzyRule(fuzzyRule44);
  oafuzzy->addFuzzyRule(fuzzyRule45);
  oafuzzy->addFuzzyRule(fuzzyRule46);
  oafuzzy->addFuzzyRule(fuzzyRule47);
  oafuzzy->addFuzzyRule(fuzzyRule48);
  oafuzzy->addFuzzyRule(fuzzyRule49);
  oafuzzy->addFuzzyRule(fuzzyRule50);
  oafuzzy->addFuzzyRule(fuzzyRule51);
  oafuzzy->addFuzzyRule(fuzzyRule52);
  oafuzzy->addFuzzyRule(fuzzyRule53);
  oafuzzy->addFuzzyRule(fuzzyRule54);
  oafuzzy->addFuzzyRule(fuzzyRule55);
  oafuzzy->addFuzzyRule(fuzzyRule56);
  oafuzzy->addFuzzyRule(fuzzyRule57);
  oafuzzy->addFuzzyRule(fuzzyRule58);
  oafuzzy->addFuzzyRule(fuzzyRule59);
  oafuzzy->addFuzzyRule(fuzzyRule60);
  oafuzzy->addFuzzyRule(fuzzyRule61);
  oafuzzy->addFuzzyRule(fuzzyRule62);
  oafuzzy->addFuzzyRule(fuzzyRule63);
  oafuzzy->addFuzzyRule(fuzzyRule64);
  oafuzzy->addFuzzyRule(fuzzyRule65);
  oafuzzy->addFuzzyRule(fuzzyRule66);
  oafuzzy->addFuzzyRule(fuzzyRule67);
  oafuzzy->addFuzzyRule(fuzzyRule68);
  oafuzzy->addFuzzyRule(fuzzyRule69);
  oafuzzy->addFuzzyRule(fuzzyRule70);
  oafuzzy->addFuzzyRule(fuzzyRule71);
  oafuzzy->addFuzzyRule(fuzzyRule72);
  oafuzzy->addFuzzyRule(fuzzyRule73);
  oafuzzy->addFuzzyRule(fuzzyRule74);
  oafuzzy->addFuzzyRule(fuzzyRule75);
  oafuzzy->addFuzzyRule(fuzzyRule76);
  oafuzzy->addFuzzyRule(fuzzyRule77);
  oafuzzy->addFuzzyRule(fuzzyRule78);
  oafuzzy->addFuzzyRule(fuzzyRule79);
  oafuzzy->addFuzzyRule(fuzzyRule80);
  oafuzzy->addFuzzyRule(fuzzyRule81);
  oafuzzy->addFuzzyRule(fuzzyRule82);
  oafuzzy->addFuzzyRule(fuzzyRule83);
  oafuzzy->addFuzzyRule(fuzzyRule84);
  oafuzzy->addFuzzyRule(fuzzyRule85);
  oafuzzy->addFuzzyRule(fuzzyRule86);
  oafuzzy->addFuzzyRule(fuzzyRule87);
  oafuzzy->addFuzzyRule(fuzzyRule88);
  oafuzzy->addFuzzyRule(fuzzyRule89);
  oafuzzy->addFuzzyRule(fuzzyRule90);
  oafuzzy->addFuzzyRule(fuzzyRule91);
  oafuzzy->addFuzzyRule(fuzzyRule92);
  oafuzzy->addFuzzyRule(fuzzyRule93);
  oafuzzy->addFuzzyRule(fuzzyRule94);
  oafuzzy->addFuzzyRule(fuzzyRule95);
  oafuzzy->addFuzzyRule(fuzzyRule96);
  oafuzzy->addFuzzyRule(fuzzyRule97);
  oafuzzy->addFuzzyRule(fuzzyRule98);
  oafuzzy->addFuzzyRule(fuzzyRule99);
  oafuzzy->addFuzzyRule(fuzzyRule100);
  oafuzzy->addFuzzyRule(fuzzyRule101);
  oafuzzy->addFuzzyRule(fuzzyRule102);
  oafuzzy->addFuzzyRule(fuzzyRule103);
  oafuzzy->addFuzzyRule(fuzzyRule104);
  oafuzzy->addFuzzyRule(fuzzyRule105);
  oafuzzy->addFuzzyRule(fuzzyRule106);
  oafuzzy->addFuzzyRule(fuzzyRule107);
  oafuzzy->addFuzzyRule(fuzzyRule108);
  oafuzzy->addFuzzyRule(fuzzyRule109);
  oafuzzy->addFuzzyRule(fuzzyRule110);
  oafuzzy->addFuzzyRule(fuzzyRule111);
  oafuzzy->addFuzzyRule(fuzzyRule112);
  oafuzzy->addFuzzyRule(fuzzyRule113);
  oafuzzy->addFuzzyRule(fuzzyRule114);
  oafuzzy->addFuzzyRule(fuzzyRule115);
  oafuzzy->addFuzzyRule(fuzzyRule116);
  oafuzzy->addFuzzyRule(fuzzyRule117);
  oafuzzy->addFuzzyRule(fuzzyRule118);
  oafuzzy->addFuzzyRule(fuzzyRule119);
  oafuzzy->addFuzzyRule(fuzzyRule120);
  oafuzzy->addFuzzyRule(fuzzyRule121);
  oafuzzy->addFuzzyRule(fuzzyRule122);
  oafuzzy->addFuzzyRule(fuzzyRule123);
  oafuzzy->addFuzzyRule(fuzzyRule124);
  oafuzzy->addFuzzyRule(fuzzyRule125);
  oafuzzy->addFuzzyRule(fuzzyRule126);
  oafuzzy->addFuzzyRule(fuzzyRule127);
  oafuzzy->addFuzzyRule(fuzzyRule128);
  oafuzzy->addFuzzyRule(fuzzyRule129);
  oafuzzy->addFuzzyRule(fuzzyRule130);
  oafuzzy->addFuzzyRule(fuzzyRule131);
  oafuzzy->addFuzzyRule(fuzzyRule132);
  oafuzzy->addFuzzyRule(fuzzyRule133);
  oafuzzy->addFuzzyRule(fuzzyRule134);
  oafuzzy->addFuzzyRule(fuzzyRule135);


}

void loop()
{
  getAllDistances();  //to be used in pt 2 of code
  updateOdometry();

  oafuzzy->setInput(1, dist1);
  oafuzzy->setInput(2, dist2);
  oafuzzy->setInput(3, dist3);
  oafuzzy->setInput(4, diff_angle); //todo: ADD ANGLE DIFFERENCE

  oafuzzy->fuzzify();

  float targetvl = oafuzzy->defuzzify(1);
  float targetvr = oafuzzy->defuzzify(2);

  Serial.print("Vl: \t");
  Serial.print(targetvl);
  Serial.print("\t Vr: \t");
  Serial.println(targetvr);

  moveRobot(targetvl, targetvr);
}
