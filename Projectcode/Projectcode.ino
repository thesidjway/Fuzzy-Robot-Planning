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
  return robotAngle;
}


void getAllDistances()
{
  int temp_dist1, temp_dist2, temp_dist3, temp_dist4; // temporary variables for storing data, so that all distances get written simultaneously.
  /*Insert SONAR data collection code here*/
  dist1 = temp_dist1;
  dist2 = temp_dist2;
  dist3 = temp_dist3;
  dist4 = temp_dist4;
}

void updateOdometry()
{

}


// Step 1 -  Instantiating an object library
Fuzzy* fuzzy = new Fuzzy();

void setup() {
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
  FuzzySet* still = new FuzzySet(0, 0, 0, 0);
  FuzzySet* slow = new FuzzySet(100, 130, 130, 160);
  FuzzySet* mid = new FuzzySet(130, 160, 160, 190);
  FuzzySet* fast = new FuzzySet(160, 190, 190, 220);
  FuzzySet* veryfast = new FuzzySet(190, 220, 220, 250);

  // Creating FuzzySet to compond FuzzyOutput vr (Right Wheel Velocity)

  vl->addFuzzySet(still);
  vl->addFuzzySet(slow);
  vl->addFuzzySet(mid);
  vl->addFuzzySet(fast);
  vl->addFuzzySet(veryfast);


  vr->addFuzzySet(still);
  vr->addFuzzySet(slow);
  vr->addFuzzySet(mid);
  vr->addFuzzySet(fast);
  vr->addFuzzySet(veryfast);


  fuzzy->addFuzzyOutput(vl); // Add FuzzyOutput to Fuzzy object
  fuzzy->addFuzzyOutput(vr); // Add FuzzyOutput to Fuzzy object
/*
  //Passo 4 - Assembly the Fuzzy rules
  // FuzzyRule "IF distance = samll THEN velocity = slow"
  FuzzyRuleAntecedent* ifDistanceSmall = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifDistanceSmall->joinSingle(small); // Adding corresponding FuzzySet to Antecedent object
  FuzzyRuleConsequent* thenVelocitySlow = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenVelocitySlow->addOutput(slow);// Adding corresponding FuzzySet to Consequent object
  // Instantiating a FuzzyRule object
  FuzzyRule* fuzzyRule01 = new FuzzyRule(1, ifDistanceSmall, thenVelocitySlow); // Passing the Antecedent and the Consequent of expression

  fuzzy->addFuzzyRule(fuzzyRule01); // Adding FuzzyRule to Fuzzy object

  // FuzzyRule "IF distance = safe THEN velocity = normal"
  FuzzyRuleAntecedent* ifDistanceSafe = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifDistanceSafe->joinSingle(safe); // Adding corresponding FuzzySet to Antecedent object
  FuzzyRuleConsequent* thenVelocityAverage = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenVelocityAverage->addOutput(average); // Adding corresponding FuzzySet to Consequent object
  // Instantiating a FuzzyRule object
  FuzzyRule* fuzzyRule02 = new FuzzyRule(2, ifDistanceSafe, thenVelocityAverage); // Passing the Antecedent and the Consequent of expression

  fuzzy->addFuzzyRule(fuzzyRule02); // Adding FuzzyRule to Fuzzy object

  // FuzzyRule "IF distance = big THEN velocity = fast"
  FuzzyRuleAntecedent* ifDistanceBig = new FuzzyRuleAntecedent(); // Instantiating an Antecedent to expression
  ifDistanceBig->joinSingle(big); // Adding corresponding FuzzySet to Antecedent object
  FuzzyRuleConsequent* thenVelocityFast = new FuzzyRuleConsequent(); // Instantiating a Consequent to expression
  thenVelocityFast->addOutput(fast);// Adding corresponding FuzzySet to Consequent object
  // Instantiating a FuzzyRule object
  FuzzyRule* fuzzyRule03 = new FuzzyRule(3, ifDistanceBig, thenVelocityFast); // Passing the Antecedent and the Consequent of expression

  fuzzy->addFuzzyRule(fuzzyRule03); // Adding FuzzyRule to Fuzzy object
*/
}

void loop() {
  getAllDistances();

  // Step 5 - Report inputs value, passing its ID and value
  // fuzzy->setInput(1, dist);
  // Step 6 - Exe the fuzzification
  fuzzy->fuzzify();
  // Step 7 - Exe the desfuzzyficação for each output, passing its ID
  float output = fuzzy->defuzzify(1);

  Serial.println(output);
  //setRobotSpeed(output);

  delay(100);
}
