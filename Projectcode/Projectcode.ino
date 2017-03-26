#define LM1 2
#define LM2 3
#define RM1 4
#define RM2 5
#define EN1 6
#define EN2 7

#include <FuzzyRule.h>
#include <FuzzyComposition.h>
#include <Fuzzy.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzyOutput.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzySet.h>
#include <FuzzyRuleAntecedent.h>

int dist1, dist2, dist3, dist4; // distances of all 4 ultrasonic sensors
int robotAngle = 0; // current Angle of the robot

int getRobotAngle()
{
  //ADD ANGLE DIFFERENCE CALCULATION CODE HERE
  return robotAngle;
}


void getAllDistances()
{
  int temp_dist1, temp_dist2, temp_dist3, temp_dist4; // temporary variables for storing data, so that all distances get written simultaneously.
  /*Insert distance data collection code here*/
  dist1 = temp_dist1;
  dist2 = temp_dist2;
  dist3 = temp_dist3;
  dist4 = temp_dist4;
}

void updateOdometry()
{
  //ADD ODOMETRY CODE HERE
}


// Step 1 -  Instantiating an object library
Fuzzy* fuzzy = new Fuzzy();

void setup() 
{
  
  Serial.begin(9600);

  pinMode(LM1, OUTPUT); // left motor input1
  pinMode(LM2, OUTPUT); // left motor input2
  pinMode(RM1, OUTPUT); // right motor input1
  pinMode(RM2, OUTPUT); // right motor input2
  pinMode(EN1, OUTPUT); // enable 1
  pinMode(EN2, OUTPUT); // enable 2

  // Step 2 - Creating a FuzzyInput distance
  FuzzyInput* distance = new FuzzyInput(1);// With its ID in param

  // Creating the FuzzySet to compond FuzzyInput distance
  FuzzySet* verysmall = new FuzzySet(0, 0, 0, 125);
  distance->addFuzzySet(verysmall);
  FuzzySet* small = new FuzzySet(0, 125, 125, 250);
  distance->addFuzzySet(small);
  FuzzySet* medium = new FuzzySet(125, 250, 250, 375);
  distance->addFuzzySet(medium);
  FuzzySet* big = new FuzzySet(250, 375, 375, 500);
  distance->addFuzzySet(big);
  FuzzySet* verybig = new FuzzySet(375, 500, 500, 500);
  distance->addFuzzySet(verybig);

  // Creating the FuzzySet to compond FuzzyInput angle
  FuzzyInput* angle = new FuzzyInput(2);

  FuzzySet* negbig = new FuzzySet(-180, -180, -180, -120);
  angle->addFuzzySet(negbig);
  FuzzySet* negmed = new FuzzySet(-180, -120, -120, -60);
  angle->addFuzzySet(negmed);
  FuzzySet* negsmall = new FuzzySet(-120, -60, -60, 0);
  angle->addFuzzySet(negsmall);
  FuzzySet* zero = new FuzzySet(-60, 0, 0, 60);
  angle->addFuzzySet(zero);
  FuzzySet* possmall = new FuzzySet(0, 60, 60, 120);
  angle->addFuzzySet(possmall);
  FuzzySet* posmed = new FuzzySet(60, 120, 120, 180);
  angle->addFuzzySet(posmed);
  FuzzySet* posbig = new FuzzySet(120, 180, 180, 180);
  angle->addFuzzySet(posbig);


  fuzzy->addFuzzyInput(distance); // Add FuzzyInput to Fuzzy object

  // Passo 3 - Creating FuzzyOutput velocity
  FuzzyOutput* vl = new FuzzyOutput(1);
  FuzzyOutput* vr = new FuzzyOutput(2);

  // Creating FuzzySet to compond FuzzyOutput vl (Left Wheel Velocity)
  FuzzySet* veryslow = new FuzzySet(0, 0, 0, 0);
  FuzzySet* slow = new FuzzySet(100, 130, 130, 160);
  FuzzySet* mid = new FuzzySet(130, 160, 160, 190);
  FuzzySet* fast = new FuzzySet(160, 190, 190, 220);
  FuzzySet* veryfast = new FuzzySet(190, 220, 220, 250);

  // Creating FuzzySet to compond FuzzyOutput vr (Right Wheel Velocity)

  vl->addFuzzySet(veryslow);
  vl->addFuzzySet(slow);
  vl->addFuzzySet(mid);
  vl->addFuzzySet(fast);
  vl->addFuzzySet(veryfast);


  vr->addFuzzySet(veryslow);
  vr->addFuzzySet(slow);
  vr->addFuzzySet(mid);
  vr->addFuzzySet(fast);
  vr->addFuzzySet(veryfast);


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
  VFVS->addOutput(veryfast);
  VFVS->addOutput(veryslow);
  FuzzyRuleConsequent* FVS = new FuzzyRuleConsequent();
  FVS->addOutput(fast);
  FVS->addOutput(veryslow);
  FuzzyRuleConsequent* MVS = new FuzzyRuleConsequent();
  MVS->addOutput(mid);
  MVS->addOutput(veryslow);
  FuzzyRuleConsequent* SVS = new FuzzyRuleConsequent();
  SVS->addOutput(slow);
  SVS->addOutput(veryslow);

  FuzzyRuleConsequent* SS = new FuzzyRuleConsequent();
  SS->addOutput(slow);
  SS->addOutput(slow);
  FuzzyRuleConsequent* MM = new FuzzyRuleConsequent();
  MM->addOutput(mid);
  MM->addOutput(mid);
  FuzzyRuleConsequent* FF = new FuzzyRuleConsequent();
  FF->addOutput(fast);
  FF->addOutput(fast);
  FuzzyRuleConsequent* VFVF = new FuzzyRuleConsequent();
  VFVF->addOutput(veryfast);
  VFVF->addOutput(veryfast);


  FuzzyRuleConsequent* VSVF = new FuzzyRuleConsequent();
  VSVF->addOutput(veryslow);
  VSVF->addOutput(veryfast);
  FuzzyRuleConsequent* VSF = new FuzzyRuleConsequent();
  VSF->addOutput(veryslow);
  VSF->addOutput(fast);
  FuzzyRuleConsequent* VSM = new FuzzyRuleConsequent();
  VSM->addOutput(veryslow);
  VSM->addOutput(mid);
  FuzzyRuleConsequent* VSS = new FuzzyRuleConsequent();
  VSS->addOutput(veryslow);
  VSS->addOutput(slow);

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
  FuzzyRule* fuzzyRule34 = new FuzzyRule(34, BPB,  VSVF  );
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
  getAllDistances();
  getRobotAngle();
  updateOdometry();
  
  
  
  fuzzy->setInput(1, 10); //todo: ADD DISTANCE FROM TARGET
  fuzzy->setInput(2, 20); //todo: ADD ANGLE DIFFERENCE
  
  fuzzy->fuzzify();

  float targetvl = fuzzy->defuzzify(1);
  float targetvr = fuzzy->defuzzify(2);

  Serial.print("Vl: \t");
  Serial.print(targetvl);
  Serial.print("\t Vr: \t");
  Serial.println(targetvr);

  delay(100);
}
