// libs
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <array>

#include <patternData.h>

#define LENGTH 150
// TODO: make compatible with smaller storage size + make class for dtwGestures
// attribues
Adafruit_MPU6050 mpu;

const short THRESHOLD = 10000;

uint16_t recInterval = 0;
boolean recFlag = false;

//----------------------- plotter -----------------------//

void plotData(sensors_event_t a, sensors_event_t g)
{
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("GyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.print(g.gyro.z);
  Serial.print(", ");

  Serial.print("Top:");
  Serial.print(9.81);
  Serial.print(",");
  Serial.print("Bottom:");
  Serial.print(-9.81);
  Serial.print(",");

  Serial.println("");
}

void recData(sensors_event_t a, sensors_event_t g)
{
  if (!recFlag && Serial.parseInt() == 1)
  {
    recFlag = true;
    Serial.println("AccelX,AccelY,AccelZ");
  }

  if (recFlag && recInterval < 1000)
  {
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.println(a.acceleration.z);

    if (++recInterval >= 150)
    {
      recFlag = false;
      recInterval = 0;
      Serial.println("----------------------------------------------------------");
    }
  }
}

//----------------------- actions -----------------------//
void perfomAction1()
{
  digitalWrite(25, HIGH);
  delay(100);
  digitalWrite(25, LOW);
  Serial.println("\033[1;32m Gesture1 recognized: Square counter-clockwise\033[0;39m");
}

void perfomAction2()
{
  digitalWrite(25, HIGH);
  delay(100);
  digitalWrite(25, LOW);
  delay(50);
  digitalWrite(25, HIGH);
  delay(100);
  digitalWrite(25, LOW);
  Serial.println("\033[1;32m Gesture2 recognized: Circle clockwise\033[0;39m");
}

//----------------------- dtw methodes -----------------------// TODO: optimze datatypes
float costMatrix[LENGTH][LENGTH];

float dtwRecXGyro[] = {};
float dtwRecYGyro[] = {};
float dtwRecZGyro[] = {};
float dtwRecXAcc[] = {};
float dtwRecYAcc[] = {};
float dtwRecZAcc[] = {};
float dtwRecGesture[][LENGTH] = {*dtwRecXGyro, *dtwRecYGyro, *dtwRecZGyro, *dtwRecXAcc, *dtwRecYAcc, *dtwRecZAcc};

unsigned char dtwRecCount = 0;

/**
 * @brief writes the current state of the sensor in a array
 *
 * @param a accelerometer input
 * @param g gyroscope input
 */
void recDTWData(sensors_event_t a, sensors_event_t g)
{
  if (!recFlag && Serial.parseInt() == 2)
  {
    recFlag = true;
    dtwRecCount = 0;
    Serial.println("Started recognition recording");
  }

  if (recFlag && recInterval < 1000)
  {
    dtwRecXGyro[dtwRecCount] = g.gyro.x;
    dtwRecYGyro[dtwRecCount] = g.gyro.y;
    dtwRecZGyro[dtwRecCount] = g.gyro.z;
    dtwRecXAcc[dtwRecCount] = a.acceleration.x;
    dtwRecYAcc[dtwRecCount] = a.acceleration.y;
    dtwRecZAcc[dtwRecCount] = a.acceleration.z;
    dtwRecCount++;

    if (++recInterval >= LENGTH)
    {
      recFlag = false;
      recInterval = 0;
      Serial.println("  ->End recognition recording");
    }
  }
}

void costMatrixInitialize()
{
  for (unsigned char row = 0; row < LENGTH; row++)
    for (unsigned char column = 0; column < LENGTH; column++)
      costMatrix[row][column] = 0;
  costMatrix[0][0] = 0;
}

/**
 * @brief gives the values of the cost matrix to the left, up und up-left
 * @param i = x
 * @param j = y
 * @return std::array<float, 3> containing the neighbour values
 */
std::array<float, 3> minChilds(int i, int j)
{
  std::array<float, 3> tmpChilds = {10000.0, 10000.0, 10000.0};
  if (i - 1 >= 0 && j - 1 >= 0) // match
    tmpChilds[0] = costMatrix[i - 1][j - 1];
  if (i - 1 >= 0) // insertion
    tmpChilds[1] = costMatrix[i - 1][j];
  if (j - 1 >= 0) // deletion
    tmpChilds[2] = costMatrix[i][j - 1];
  return tmpChilds;
}

/**
 * @brief returns the minimum of minChilds of current position
 * @param i = x
 * @param j = y
 * @return float = smallest of the neighbours
 */
float getMin(int i, int j)
{ // min -> match, insertion, deletion
  std::array<float, 3> minChild = minChilds(i, j);
  float min = 10000;
  if (min > minChild[0])
  {
    min = minChild[0];
  }
  if (min > minChild[1])
  {
    min = minChild[1];
  }
  if (min > minChild[2])
  {
    min = minChild[2];
  }
  return min;
}

/**
 * @brief calcualtes the cost for the current point
 *
 * @param i = x
 * @param j = y
 * @param pattern
 * @param gesture
 * @return float value of the current cost matrix cell
 */
float calculateCost(int i, int j, float pattern[], float gesture[])
{ // dtw algorithm
  return std::abs(pattern[i] - gesture[j]) + getMin(i, j);
}

/**
 * @brief calcualtes the cost matrix for a given pattern and current gesture
 *
 * @param tempPattern = pattern
 * @param seqPattern = gesture
 */
void calcualteCostMatrix(float samplePattern[], float gesturePattern[]) // TODO: LOCAL CONSTRAINTS
{
  for (int i = 1; i < LENGTH; i++)
    for (int j = 1; j < LENGTH; j++)
      costMatrix[i][j] = calculateCost(i, j, samplePattern, gesturePattern);
}

/**
 * @brief calculate shortest distance between point/diagonale
 * @param i = y
 * @param j = x
 * @return float = area size
 */
float calculateSpaceToDia(int i, int j)
{
  float tmpResult = 0;
  while (i >= 0 && j >= 0)
  {
    std::array<float, 3> tmpChilds = minChilds(i, j);
    float tmpMin = getMin(i, j);

    float tmpCoord = 0.5 * (i + j);
    float tmpDistance = sqrt(pow(tmpCoord - i, 2) + pow(tmpCoord - j, 2));

    tmpResult += abs(tmpDistance);

    if (tmpMin != tmpChilds[1] || tmpChilds[0] == tmpChilds[1])
      --j;
    if (tmpMin != tmpChilds[2] || tmpChilds[0] == tmpChilds[2])
      --i;
  }
  return tmpResult;
}

void printCostMatrix()
{
  Serial.println("  Cost matrix print out");
  for (byte row = 0; row < LENGTH; row++)
  {
    for (byte column = 0; column < LENGTH; column++)
    {
      Serial.print(costMatrix[row][column]);
      Serial.print("\t");
    }
    Serial.println("");
  }
  Serial.println("    ->Print done");
}

/**
 * @brief iterrating of all gyro/acc axies and adding up the similarity based on the area
 * @param pattern one of the patterns to match
 * @return short = similarity
 */
short dtwAnalysis(float pattern[][LENGTH])
{
  short similarity = 0;
  for (int i = 0; i < sizeof(pattern[0]) / sizeof(pattern[0][1]); i++)
  {
    costMatrixInitialize();
    calcualteCostMatrix(pattern[i], dtwRecGesture[i]);
    similarity += calculateSpaceToDia(LENGTH - 1, LENGTH - 1);
  }
  return similarity;
}

/**
 * @brief calculating dtwAnalysis for each pattern with the current recorded gesture
 * @return short* containing the similarity
 */
short *detectPattern()
{
  short similaritys[NUM_PATTERN];
  for (int i = 0; i < NUM_PATTERN; i++)
  {
    similaritys[i] = dtwAnalysis(patterns[i]);
    Serial.print("\033[1m  =>Similarity: ");
    Serial.println(similaritys[i]);
    Serial.println("\033[0;39m");
  }
  return similaritys;
}

/**
 * @brief picking the best similarity from the array -> perfomAction
 */
void recognizeGesture()
{
  short similaritys[] = {*detectPattern()};
  if (similaritys[0] < THRESHOLD && similaritys[0] < similaritys[1])
    perfomAction1();
  else if (similaritys[1] < THRESHOLD && similaritys[1] < similaritys[0])
    perfomAction2();
  else
    Serial.println("  \033[1;31mWrong gesture\033[0;39m");
}

//----------------------- main -----------------------//
void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");

  pinMode(25, OUTPUT);

  Serial.println("Harry Potter Magic Wand ━━☆ﾟ");
  Serial.println("Press 2 for gesture recognition");
  Serial.println("should be automated in the future\n");

  delay(100);
}

void loop()
{
  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  //--- DTW ---//
  recDTWData(acc, gyro);
  if (dtwRecCount >= LENGTH) // start recognition
  {
    Serial.println("\nStart dtw recognition");

    recognizeGesture();

    dtwRecCount = 0;
    Serial.println("  ->Dtw finished");
    Serial.println("-------------------------------------------------");
  }

  delay(10);
}
