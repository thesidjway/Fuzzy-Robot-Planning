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

#define LM1 6
#define LM2 13
#define RM1 11
#define RM2 5
#define EN1 8
#define EN2 12
#define ENCODERFACTOR 0.0156
#define AXLE_LENGTH 7.0

Encoder leftEnc(2, 3);
Encoder rightEnc (18, 19);

long long prev_position_l = 0, curr_position_l = 0;
long long prev_position_r = 0, curr_position_r = 0;

float target_x = -400, target_y = 1;
float curr_x = 0 , curr_y = 0;
float curr_theta = 0, prev_theta = 0;

float diff_angle = 100;
float dist_from_target = 200;


int dist1, dist2, dist3; // distances of all 3 ultrasonic sensors
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
  int temp_dist1, temp_dist2, temp_dist3, temp_dist4; // temporary variables for storing data, so that all distances get written simultaneously.
  /*Insert distance data collection code here*/
  dist1 = temp_dist1;
  dist2 = temp_dist2;
  dist3 = temp_dist3;
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
  Serial.print ("curr_x");
  Serial.print (curr_x);
  Serial.print ("curr_y");
  Serial.print (curr_y);
  float target_theta = atan2 (( target_y - curr_y), (target_x - curr_x));
  dist_from_target = sqrt (pow( target_y - curr_y, 2) + pow(target_x - curr_x, 2));
  diff_angle = target_theta - curr_theta;
  while (diff_angle > PI)
    diff_angle -= (2.0 * PI);
  while (diff_angle < -PI)
    diff_angle += (2.0 * PI);
}


// Step 1 -  Instantiating an object library
Fuzzy* fuzzy = new Fuzzy();

FuzzySet* verysmall = new FuzzySet(0, 0, 0, 125);
FuzzySet* small = new FuzzySet(0, 125, 125, 250);
FuzzySet* medium = new FuzzySet(125, 250, 250, 375);
FuzzySet* big = new FuzzySet(250, 375, 375, 500);
FuzzySet* verybig = new FuzzySet(375, 500, 500, 500);


FuzzySet* negbig = new FuzzySet(-180, -180, -180, -120);
FuzzySet* negmed = new FuzzySet(-180, -120, -120, -60);
FuzzySet* negsmall = new FuzzySet(-120, -60, -60, 0);
FuzzySet* zero = new FuzzySet(-60, 0, 0, 60);
FuzzySet* possmall = new FuzzySet(0, 60, 60, 120);
FuzzySet* posmed = new FuzzySet(60, 120, 120, 180);
FuzzySet* posbig = new FuzzySet(120, 180, 180, 180);

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
  FuzzyInput* distance = new FuzzyInput(1);
  distance->addFuzzySet(verysmall);
  distance->addFuzzySet(small);
  distance->addFuzzySet(medium);
  distance->addFuzzySet(big);
  distance->addFuzzySet(verybig);


  FuzzyInput* angle = new FuzzyInput(2);
  angle->addFuzzySet(negbig);
  angle->addFuzzySet(negmed);
  angle->addFuzzySet(negsmall);
  angle->addFuzzySet(zero);
  angle->addFuzzySet(possmall);
  angle->addFuzzySet(posmed);
  angle->addFuzzySet(posbig);







  fuzzy->addFuzzyInput(distance); // Add FuzzyInput to Fuzzy object
  fuzzy->addFuzzyInput(angle);

  // Passo 3 - Creating FuzzyOutput velocity
  FuzzyOutput* vl = new FuzzyOutput(1);
  FuzzyOutput* vr = new FuzzyOutput(2);

  // Creating FuzzySet to compond FuzzyOutput vl (Left Wheel Velocity)
  FuzzySet* Lveryslow = new FuzzySet(0, 0, 0, 60);
  FuzzySet* Lslow = new FuzzySet(100, 130, 130, 160);
  FuzzySet* Lmid = new FuzzySet(130, 160, 160, 190);
  FuzzySet* Lfast = new FuzzySet(160, 190, 190, 220);
  FuzzySet* Lveryfast = new FuzzySet(190, 220, 220, 250);
  
  FuzzySet* Rveryslow = new FuzzySet(0, 0, 0, 60);
  FuzzySet* Rslow = new FuzzySet(100, 130, 130, 160);
  FuzzySet* Rmid = new FuzzySet(130, 160, 160, 190);
  FuzzySet* Rfast = new FuzzySet(160, 190, 190, 220);
  FuzzySet* Rveryfast = new FuzzySet(190, 220, 220, 250);

  // Creating FuzzySet to compond FuzzyOutput vr (Right Wheel Velocity)

  vl->addFuzzySet(Lveryslow);
  vl->addFuzzySet(Lslow);
  vl->addFuzzySet(Lmid);
  vl->addFuzzySet(Lfast);
  vl->addFuzzySet(Lveryfast);


  vr->addFuzzySet(Rveryslow);
  vr->addFuzzySet(Rslow);
  vr->addFuzzySet(Rmid);
  vr->addFuzzySet(Rfast);
  vr->addFuzzySet(Rveryfast);


  fuzzy->addFuzzyOutput(vl);
  fuzzy->addFuzzyOutput(vr);


  FuzzyRuleAntecedent*  VSNB = new FuzzyRuleAntecedent();
  VSNB->joinWithAND(verysmall, negbig);
  FuzzyRuleAntecedent*  SNB = new FuzzyRuleAntecedent();
  SNB->joinWithAND(small, negbig);
  FuzzyRuleAntecedent*  MNB = new FuzzyRuleAntecedent();
  MNB->joinWithAND(medium, negbig);
  FuzzyRuleAntecedent*  BNB = new FuzzyRuleAntecedent();
  BNB->joinWithAND(big, negbig);
  FuzzyRuleAntecedent*  VBNB = new FuzzyRuleAntecedent();
  VBNB->joinWithAND(verybig, negbig);

  FuzzyRuleAntecedent*  VSNM = new FuzzyRuleAntecedent();
  VSNM->joinWithAND(verysmall, negmed);
  FuzzyRuleAntecedent*  SNM = new FuzzyRuleAntecedent();
  SNM->joinWithAND(small, negmed);
  FuzzyRuleAntecedent*  MNM = new FuzzyRuleAntecedent();
  MNM->joinWithAND(medium, negmed);
  FuzzyRuleAntecedent*  BNM = new FuzzyRuleAntecedent();
  BNM->joinWithAND(big, negmed);
  FuzzyRuleAntecedent*  VBNM = new FuzzyRuleAntecedent();
  VBNM->joinWithAND(verybig, negmed);

  FuzzyRuleAntecedent*  VSNS = new FuzzyRuleAntecedent();
  VSNS->joinWithAND(verysmall, negsmall);
  FuzzyRuleAntecedent*  SNS = new FuzzyRuleAntecedent();
  SNS->joinWithAND(small, negsmall);
  FuzzyRuleAntecedent*  MNS = new FuzzyRuleAntecedent();
  MNS->joinWithAND(medium, negsmall);
  FuzzyRuleAntecedent*  BNS = new FuzzyRuleAntecedent();
  BNS->joinWithAND(big, negsmall);
  FuzzyRuleAntecedent*  VBNS = new FuzzyRuleAntecedent();
  VBNS->joinWithAND(verybig, negsmall);

  FuzzyRuleAntecedent*  VSZ = new FuzzyRuleAntecedent();
  VSZ->joinWithAND(verysmall, zero);
  FuzzyRuleAntecedent*  SZ = new FuzzyRuleAntecedent();
  SZ->joinWithAND(small, zero);
  FuzzyRuleAntecedent*  MZ = new FuzzyRuleAntecedent();
  MZ->joinWithAND(medium, zero);
  FuzzyRuleAntecedent*  BZ = new FuzzyRuleAntecedent();
  BZ->joinWithAND(big, zero);
  FuzzyRuleAntecedent*  VBZ = new FuzzyRuleAntecedent();
  VBZ->joinWithAND(verybig, zero);

  FuzzyRuleAntecedent*  VSPS = new FuzzyRuleAntecedent();
  VSPS->joinWithAND(verysmall, possmall);
  FuzzyRuleAntecedent*  SPS = new FuzzyRuleAntecedent();
  SPS->joinWithAND(small, possmall);
  FuzzyRuleAntecedent*  MPS = new FuzzyRuleAntecedent();
  MPS->joinWithAND(medium, possmall);
  FuzzyRuleAntecedent*  BPS = new FuzzyRuleAntecedent();
  BPS->joinWithAND(big, possmall);
  FuzzyRuleAntecedent*  VBPS = new FuzzyRuleAntecedent();
  VBPS->joinWithAND(verybig, possmall);

  FuzzyRuleAntecedent*  VSPM = new FuzzyRuleAntecedent();
  VSPM->joinWithAND(verysmall, posmed);
  FuzzyRuleAntecedent*  SPM = new FuzzyRuleAntecedent();
  SPM->joinWithAND(small, posmed);
  FuzzyRuleAntecedent*  MPM = new FuzzyRuleAntecedent();
  MPM->joinWithAND(medium, posmed);
  FuzzyRuleAntecedent*  BPM = new FuzzyRuleAntecedent();
  BPM->joinWithAND(big, posmed);
  FuzzyRuleAntecedent*  VBPM = new FuzzyRuleAntecedent();
  VBPM->joinWithAND(verybig, posmed);

  FuzzyRuleAntecedent*  VSPB = new FuzzyRuleAntecedent();
  VSPB->joinWithAND(verysmall, posbig);
  FuzzyRuleAntecedent*  SPB = new FuzzyRuleAntecedent();
  SPB->joinWithAND(small, posbig);
  FuzzyRuleAntecedent*  MPB = new FuzzyRuleAntecedent();
  MPB->joinWithAND(medium, posbig);
  FuzzyRuleAntecedent*  BPB = new FuzzyRuleAntecedent();
  BPB->joinWithAND(big, posbig);
  FuzzyRuleAntecedent*  VBPB = new FuzzyRuleAntecedent();
  VBPB->joinWithAND(verybig, posbig);


  FuzzyRuleConsequent* VFVS = new FuzzyRuleConsequent();
  VFVS->addOutput(Lveryfast);
  VFVS->addOutput(Rveryslow);
  FuzzyRuleConsequent* FVS = new FuzzyRuleConsequent();
  FVS->addOutput(Lfast);
  FVS->addOutput(Rveryslow);
  FuzzyRuleConsequent* MVS = new FuzzyRuleConsequent();
  MVS->addOutput(Lmid);
  MVS->addOutput(Rveryslow);
  FuzzyRuleConsequent* SVS = new FuzzyRuleConsequent();
  SVS->addOutput(Lslow);
  SVS->addOutput(Rveryslow);

  FuzzyRuleConsequent* SS = new FuzzyRuleConsequent();
  SS->addOutput(Lslow);
  SS->addOutput(Rslow);
  FuzzyRuleConsequent* MM = new FuzzyRuleConsequent();
  MM->addOutput(Lmid);
  MM->addOutput(Rmid);
  FuzzyRuleConsequent* FF = new FuzzyRuleConsequent();
  FF->addOutput(Lfast);
  FF->addOutput(Rfast);
  FuzzyRuleConsequent* VFVF = new FuzzyRuleConsequent();
  VFVF->addOutput(Lveryfast);
  VFVF->addOutput(Rveryfast);


  FuzzyRuleConsequent* VSVF = new FuzzyRuleConsequent();
  VSVF->addOutput(Lveryslow);
  VSVF->addOutput(Rveryfast);
  FuzzyRuleConsequent* VSF = new FuzzyRuleConsequent();
  VSF->addOutput(Lveryslow);
  VSF->addOutput(Rfast);
  FuzzyRuleConsequent* VSM = new FuzzyRuleConsequent();
  VSM->addOutput(Lveryslow);
  VSM->addOutput(Rmid);
  FuzzyRuleConsequent* VSS = new FuzzyRuleConsequent();
  VSS->addOutput(Lveryslow);
  VSS->addOutput(Rslow);

  FuzzyRule* fuzzyRule01 = new FuzzyRule(1,  VSNB, FVS   );
  FuzzyRule* fuzzyRule02 = new FuzzyRule(2,  SNB,  VFVS  );
  FuzzyRule* fuzzyRule03 = new FuzzyRule(3,  MNB,  VFVS  );
  FuzzyRule* fuzzyRule04 = new FuzzyRule(4,  BNB,  VFVS  );
  FuzzyRule* fuzzyRule05 = new FuzzyRule(5,  VBNB, VFVS  );
  FuzzyRule* fuzzyRule06 = new FuzzyRule(6,  VSNM, MVS   );
  FuzzyRule* fuzzyRule07 = new FuzzyRule(7,  SNM,  FVS   );
  FuzzyRule* fuzzyRule08 = new FuzzyRule(8,  MNM,  VFVS  );
  FuzzyRule* fuzzyRule09 = new FuzzyRule(9,  BNM,  VFVS  );
  FuzzyRule* fuzzyRule10 = new FuzzyRule(10, VBNM, VFVS  );
  FuzzyRule* fuzzyRule11 = new FuzzyRule(11, VSNS, SVS   );
  FuzzyRule* fuzzyRule12 = new FuzzyRule(12, SNS,  MVS   );
  FuzzyRule* fuzzyRule13 = new FuzzyRule(13, MNS,  FVS   );
  FuzzyRule* fuzzyRule14 = new FuzzyRule(14, BNS,  VFVS  );
  FuzzyRule* fuzzyRule15 = new FuzzyRule(15, VBNS, VFVS  );
  FuzzyRule* fuzzyRule16 = new FuzzyRule(16, VSZ,  SS    );
  FuzzyRule* fuzzyRule17 = new FuzzyRule(17, SZ,   SS    );
  FuzzyRule* fuzzyRule18 = new FuzzyRule(18, MZ,   MM    );
  FuzzyRule* fuzzyRule19 = new FuzzyRule(19, BZ,   FF    );
  FuzzyRule* fuzzyRule20 = new FuzzyRule(20, VBZ,  VFVF  );
  FuzzyRule* fuzzyRule21 = new FuzzyRule(21, VSPS, VSS   );
  FuzzyRule* fuzzyRule22 = new FuzzyRule(22, SPS,  VSM   );
  FuzzyRule* fuzzyRule23 = new FuzzyRule(23, MPS,  VSF   );
  FuzzyRule* fuzzyRule24 = new FuzzyRule(24, BPS,  VSVF  );
  FuzzyRule* fuzzyRule25 = new FuzzyRule(25, VBPS, VSVF  );
  FuzzyRule* fuzzyRule26 = new FuzzyRule(26, VSPM, VSS   );
  FuzzyRule* fuzzyRule27 = new FuzzyRule(27, SPM,  VSF   );
  FuzzyRule* fuzzyRule28 = new FuzzyRule(28, MPM,  VSVF  );
  FuzzyRule* fuzzyRule29 = new FuzzyRule(29, BPM,  VSVF  );
  FuzzyRule* fuzzyRule30 = new FuzzyRule(30, VBPM, VSVF  );
  FuzzyRule* fuzzyRule31 = new FuzzyRule(31, VSPB, VSM   );
  FuzzyRule* fuzzyRule32 = new FuzzyRule(32, SPB,  VSVF  );
  FuzzyRule* fuzzyRule33 = new FuzzyRule(33, MPB,  VSVF  );
  FuzzyRule* fuzzyRule34 = new FuzzyRule(34, BPB,  VSVF  ); //VSVF
  FuzzyRule* fuzzyRule35 = new FuzzyRule(35, VBPB, VSVF  );

  fuzzy->addFuzzyRule(fuzzyRule01);
  fuzzy->addFuzzyRule(fuzzyRule02);
  fuzzy->addFuzzyRule(fuzzyRule03);
  fuzzy->addFuzzyRule(fuzzyRule04);
  fuzzy->addFuzzyRule(fuzzyRule05);
  fuzzy->addFuzzyRule(fuzzyRule06);
  fuzzy->addFuzzyRule(fuzzyRule07);
  fuzzy->addFuzzyRule(fuzzyRule08);
  fuzzy->addFuzzyRule(fuzzyRule09);
  fuzzy->addFuzzyRule(fuzzyRule10);
  fuzzy->addFuzzyRule(fuzzyRule11);
  fuzzy->addFuzzyRule(fuzzyRule12);
  fuzzy->addFuzzyRule(fuzzyRule13);
  fuzzy->addFuzzyRule(fuzzyRule14);
  fuzzy->addFuzzyRule(fuzzyRule15);
  fuzzy->addFuzzyRule(fuzzyRule16);
  fuzzy->addFuzzyRule(fuzzyRule17);
  fuzzy->addFuzzyRule(fuzzyRule18);
  fuzzy->addFuzzyRule(fuzzyRule19);
  fuzzy->addFuzzyRule(fuzzyRule20);
  fuzzy->addFuzzyRule(fuzzyRule21);
  fuzzy->addFuzzyRule(fuzzyRule22);
  fuzzy->addFuzzyRule(fuzzyRule23);
  fuzzy->addFuzzyRule(fuzzyRule24);
  fuzzy->addFuzzyRule(fuzzyRule25);
  fuzzy->addFuzzyRule(fuzzyRule26);
  fuzzy->addFuzzyRule(fuzzyRule27);
  fuzzy->addFuzzyRule(fuzzyRule28);
  fuzzy->addFuzzyRule(fuzzyRule29);
  fuzzy->addFuzzyRule(fuzzyRule30);
  fuzzy->addFuzzyRule(fuzzyRule31);
  fuzzy->addFuzzyRule(fuzzyRule32);
  fuzzy->addFuzzyRule(fuzzyRule33);
  fuzzy->addFuzzyRule(fuzzyRule34);
  fuzzy->addFuzzyRule(fuzzyRule35);


}

void loop()
{
  //getAllDistances();  //to be used in pt 2 of code
  updateOdometry();
  Serial.print("Dist: ");
  Serial.print (dist_from_target);
  Serial.print(" Diff Angle: ");
  Serial.println(diff_angle * 180 / PI);

  fuzzy->setInput(1, dist_from_target); //todo: ADD DISTANCE FROM TARGET
  fuzzy->setInput(2, diff_angle * 180 / PI); //todo: ADD ANGLE DIFFERENCE
  
  fuzzy->fuzzify();

  Serial.print("distance pertinence: ");
  Serial.print(verybig->getPertinence());
  Serial.print(", ");
  Serial.print(big->getPertinence());
  Serial.print(", ");
  Serial.print(medium->getPertinence());
  Serial.print(", ");
  Serial.print(small->getPertinence());
  Serial.print(", ");
  Serial.println(verysmall->getPertinence());
  
  Serial.print("angle pertinence: ");
  Serial.print(negbig->getPertinence());
  Serial.print(", ");
  Serial.print(negmed->getPertinence());
  Serial.print(", ");
  Serial.print(negsmall->getPertinence());
  Serial.print(", ");
  Serial.print(zero->getPertinence());
  Serial.print(", ");
  Serial.print(possmall->getPertinence());
  Serial.print(", ");
  Serial.print(posmed->getPertinence());
  Serial.print(", ");
  Serial.println(posbig->getPertinence());
  
  Serial.println(fuzzy->isFiredRule(34));
  Serial.println(fuzzy->isFiredRule(17));

  

  float targetvl = fuzzy->defuzzify(1);
  float targetvr = fuzzy->defuzzify(2);

  Serial.print("Vl: \t");
  Serial.print(targetvl);
  Serial.print("\t Vr: \t");
  Serial.println(targetvr);

  moveRobot(targetvl, targetvr);

  delay(100);
}

