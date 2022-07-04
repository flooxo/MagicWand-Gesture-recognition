// libs
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <array>

#define LENGTH 150

// attribues
Adafruit_MPU6050 mpu;

float threshold = 10;
byte sqrPos = 0;
byte showPos = 0;

uint16_t recInterval = 0;
boolean recFlag = false;

int length(float array[])
{
  return sizeof(array) / sizeof(array[0]);
}

// pattern data
float accXPatternSquare[] = {
    3.49,
    3.26,
    3.27,
    3.3,
    3.3,
    3.3,
    3.3,
    3.31,
    3.31,
    3.39,
    3.39,
    3.38,
    3.35,
    3.33,
    3.25,
    3.28,
    3.31,
    3.36,
    3.41,
    3.42,
    3.46,
    3.42,
    3.45,
    3.45,
    3.47,
    3.49,
    3.51,
    3.49,
    3.52,
    3.53,
    3.54,
    3.52,
    3.51,
    3.45,
    3.4,
    3.36,
    3.33,
    3.3,
    3.2,
    3.28,
    3.23,
    3.19,
    3.26,
    3.24,
    3.19,
    3.24,
    3.3,
    3.3,
    3.28,
    3.23,
    3.12,
    3.11,
    3.09,
    3.03,
    2.85,
    2.65,
    2.45,
    2.22,
    2.02,
    1.85,
    1.76,
    1.68,
    1.52,
    1.31,
    1.01,
    0.77,
    1.05,
    1.54,
    1.94,
    2.27,
    2.54,
    2.58,
    2.57,
    2.62,
    2.47,
    2.42,
    2.13,
    1.85,
    1.7,
    1.7,
    1.69,
    1.82,
    1.79,
    1.71,
    1.58,
    1.47,
    1.46,
    1.36,
    1.08,
    0.82,
    0.58,
    0.34,
    0.21,
    -0.06,
    -0.17,
    -0.48,
    -0.64,
    -0.74,
    -0.69,
    -0.6,
    -0.5,
    -0.3,
    -0.01,
    0.26,
    0.55,
    0.7,
    0.79,
    0.9,
    0.94,
    1.06,
    1.1,
    1.03,
    0.97,
    0.87,
    0.78,
    0.62,
    0.64,
    0.5,
    0.49,
    0.53,
    0.56,
    0.79,
    1.02,
    1.24,
    1.48,
    1.62,
    1.76,
    1.73,
    1.71,
    1.68,
    1.65,
    1.53,
    1.47,
    1.38,
    1.21,
    1.11,
    1.05,
    0.91,
    0.76,
    0.76,
    0.86,
    0.93,
    0.99,
    1.07,
    1.18,
    1.25,
    1.36,
    1.54,
    1.82,
    2.04,
};
float accYPatternSquare[] = {
    -0.38,
    -0.36,
    -0.67,
    -0.67,
    -0.54,
    -0.42,
    -0.13,
    -0.23,
    -0.38,
    -0.63,
    -0.88,
    -1.13,
    -1.14,
    -0.86,
    -0.78,
    -0.62,
    -0.24,
    0.03,
    0.29,
    0.54,
    0.72,
    0.87,
    0.97,
    1.11,
    1.44,
    1.52,
    1.49,
    1.14,
    0.72,
    0.4,
    0.16,
    0.07,
    -0.06,
    -0.11,
    -0.08,
    -0.06,
    -0.14,
    -0.23,
    -0.17,
    -0.17,
    -0.42,
    -0.52,
    -0.51,
    -0.54,
    -0.99,
    -1.22,
    -1.55,
    -1.82,
    -1.93,
    -2.34,
    -2.7,
    -2.97,
    -2.98,
    -3.05,
    -2.87,
    -2.36,
    -1.78,
    -1.51,
    -1.19,
    -0.94,
    -0.84,
    -0.74,
    -0.55,
    -0.08,
    0.03,
    0.11,
    0.18,
    -0.48,
    -1.87,
    -1.92,
    -1.26,
    -1,
    -1.1,
    -1.96,
    -2.37,
    -2.05,
    -1.56,
    -1.2,
    -0.6,
    -0.6,
    -1.19,
    -1.72,
    -1.59,
    -1.57,
    -1.79,
    -1.98,
    -2.32,
    -2.59,
    -3.04,
    -3.14,
    -2.89,
    -2.74,
    -2.53,
    -2.7,
    -2.87,
    -3.07,
    -3.08,
    -3.01,
    -2.79,
    -2.55,
    -2.28,
    -2.26,
    -2.63,
    -3.27,
    -3.14,
    -2.96,
    -2.73,
    -2,
    -1.39,
    -1.03,
    -0.22,
    0.47,
    0.54,
    1.36,
    2.22,
    2.54,
    1.94,
    1.48,
    1.49,
    1.8,
    1.65,
    1.38,
    0.86,
    0.44,
    0.09,
    -0.27,
    -0.72,
    -0.93,
    -1.15,
    -1.3,
    -1.23,
    -1.33,
    -1.69,
    -1.32,
    -1.59,
    -1.93,
    -1.45,
    -1.06,
    -1.16,
    -0.8,
    -0.47,
    -1.17,
    -1.47,
    -1.01,
    -0.47,
    -0.7,
    -1.02,
    -1.31,
    -1.47,
    -1.45,
};
float accZPatternSquare[] = {
    8.98,
    8.5,
    8.69,
    8.84,
    8.83,
    8.73,
    8.68,
    8.6,
    8.74,
    8.83,
    8.94,
    8.84,
    8.89,
    8.93,
    8.71,
    8.69,
    8.82,
    8.86,
    8.94,
    9.1,
    9.2,
    9.22,
    9.02,
    8.87,
    8.72,
    8.56,
    8.3,
    8.25,
    8.59,
    8.74,
    8.74,
    8.59,
    8.57,
    8.61,
    8.64,
    8.64,
    8.74,
    8.84,
    8.8,
    8.71,
    8.71,
    8.65,
    8.58,
    8.51,
    8.29,
    8.37,
    8.52,
    8.82,
    9.05,
    9.11,
    9.01,
    8.92,
    8.86,
    8.62,
    8.27,
    8.03,
    7.81,
    7.5,
    7.19,
    7.05,
    7,
    6.93,
    6.68,
    6.19,
    5.47,
    4.82,
    4.67,
    5.11,
    5.89,
    7.09,
    8.03,
    8.69,
    9.21,
    9.84,
    10.85,
    11.82,
    12.13,
    11.81,
    11.34,
    10.92,
    10.85,
    11.18,
    11.59,
    11.59,
    11.24,
    11.09,
    10.87,
    10.58,
    10.14,
    9.84,
    9.62,
    9.38,
    9.11,
    9.1,
    9.23,
    9.23,
    8.82,
    8.72,
    8.45,
    8.28,
    8.25,
    8.21,
    8.26,
    8.67,
    9.06,
    9.31,
    9.37,
    9.08,
    8.63,
    8.36,
    8.35,
    8.23,
    8.61,
    9.05,
    9.94,
    10.73,
    10.7,
    10.84,
    11.22,
    11.63,
    11.88,
    12.18,
    12.49,
    12.69,
    13.15,
    13.26,
    13,
    12.67,
    12.28,
    12.02,
    11.63,
    11.21,
    10.94,
    10.51,
    9.87,
    9.12,
    8.53,
    7.57,
    6.35,
    5.37,
    4.81,
    4.97,
    5.34,
    5.72,
    6.03,
    6.41,
    6.36,
    6.45,
    6.56,
    7.09,
};

//----------------------- plotter -----------------------//

// void plotData(sensors_event_t a, sensors_event_t g)
// {
//   Serial.print("AccelX:");
//   Serial.print(a.acceleration.x);
//   Serial.print(",");
//   Serial.print("AccelY:");
//   Serial.print(a.acceleration.y);
//   Serial.print(",");
//   Serial.print("AccelZ:");
//   Serial.print(a.acceleration.z);
//   Serial.print(", ");
//   Serial.print("GyroX:");
//   Serial.print(g.gyro.x);
//   Serial.print(",");
//   Serial.print("GyroY:");
//   Serial.print(g.gyro.y);
//   Serial.print(",");
//   Serial.print("GyroZ:");
//   Serial.print(g.gyro.z);
//   Serial.print(", ");

//   Serial.print("Top:");
//   Serial.print(9.81);
//   Serial.print(",");
//   Serial.print("Bottom:");
//   Serial.print(-9.81);
//   Serial.print(",");

//   Serial.println("");
// }

void perfomAction1()
{
  digitalWrite(25, HIGH);
  // Serial.println("Gesture1 recognized: Square");
}

// void detectPattern(sensors_event_t a, sensors_event_t g)
// {
//   if (isPatternMatching())                   // see next data
//     if (sqrPos >= length(accXPatternSquare)) // end of data
//     {
//       perfomAction1();
//       sqrPos = 0;
//     }
//     else
//       sqrPos++;
//   else // begin again
//     sqrPos = 0;
// }

// void showPattern(sensors_event_t a, float accXPattern[], float accYPattern[], float accZPattern[])
// {
//   Serial.print("AccelX:");
//   Serial.print(a.acceleration.x);
//   Serial.print(",");
//   Serial.print("AccelY:");
//   Serial.print(a.acceleration.y);
//   Serial.print(",");
//   Serial.print("AccelZ:");
//   Serial.print(a.acceleration.z);
//   Serial.print(",");

//   Serial.print("AccelXTop:");
//   Serial.print(accXPattern[showPos] + threshold);
//   Serial.print(",");
//   Serial.print("AccelYTop:");
//   Serial.print(accYPattern[showPos] + threshold);
//   Serial.print(",");
//   Serial.print("AccelZTop:");
//   Serial.print(accZPattern[showPos] + threshold);
//   Serial.print(",");

//   Serial.print("AccelXBottom:");
//   Serial.print(accXPattern[showPos] - threshold);
//   Serial.print(",");
//   Serial.print("AccelYBottom:");
//   Serial.print(accYPattern[showPos] - threshold);
//   Serial.print(",");
//   Serial.print("AccelZBottom:");
//   Serial.print(accZPattern[showPos] - threshold);
//   Serial.print(",");

//   Serial.println("");

//   showPos++;

//   if (showPos >= sizeof(accXPatternSquare) / sizeof(accXPatternSquare[0]))
//   {
//     showPos = 0;
//   }
// }

// void recData(sensors_event_t a, sensors_event_t g)
// {
//   if (!recFlag && Serial.parseInt() == 1)
//   {
//     recFlag = true;
//     Serial.println("AccelX,AccelY,AccelZ");
//   }

//   if (recFlag && recInterval < 1000)
//   {
//     Serial.print(a.acceleration.x);
//     Serial.print(",");
//     Serial.print(a.acceleration.y);
//     Serial.print(",");
//     Serial.println(a.acceleration.z);

//     if (++recInterval >= 150)
//     {
//       recFlag = false;
//       recInterval = 0;
//       Serial.println("----------------------------------------------------------");
//     }
//   }
// }

//----------------------- dtw methodes -----------------------// TODO: optimze datatypes
float costMatrix[LENGTH][LENGTH];
float dtwRecY[LENGTH];
float dtwRecZ[LENGTH];
short dtwRecCount = 0;
int similarity = 0;

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
    dtwRecY[dtwRecCount] = a.acceleration.y;
    dtwRecZ[dtwRecCount] = a.acceleration.z;
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
  Serial.println("    Cost matrix initialize");
  for (byte row = 0; row < LENGTH; row++)
  {
    for (byte column = 0; column < LENGTH; column++)
    {
      costMatrix[row][column] = 0;
    }
  }
  costMatrix[0][0] = 0;
  Serial.println("      ->Initialize done");
}

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

float calculateCost(int i, int j, float *pattern, float *matrix)
{ // dtw algorithm
  return std::abs(pattern[i] - matrix[j]) + getMin(i, j);
}

void calcualteCostMatrix(float tempPattern[], float seqPattern[]) // TODO: LOCAL CONSTRAINTS
{
  Serial.println("    Starting cost matrix calculation");
  for (int i = 1; i < LENGTH; i++)
    for (int j = 1; j < LENGTH; j++)
    {
      float cost =
          costMatrix[i][j] = calculateCost(i, j, tempPattern, seqPattern);
    }
  Serial.println("      ->Calculation done");
}

float calculateSpaceToDia(int i, int j)
{ // calculate shortest distance between point/diagonale
  float tmpResult = 0;
  Serial.println("    Started area calcualtion");
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
  Serial.println("      ->Calcualtion done");
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

void dwtAnalysis()
{
  similarity = 0;
  Serial.println("  y-axis: ");
  costMatrixInitialize();
  calcualteCostMatrix(accYPatternSquare, dtwRecY); // FIXME: first rows/colums wrong calc
  // printCostMatrix();
  similarity += calculateSpaceToDia(LENGTH - 1, LENGTH - 1);

  Serial.println("  z-axis: ");
  costMatrixInitialize();
  calcualteCostMatrix(accZPatternSquare, dtwRecZ);
  // printCostMatrix();
  similarity += calculateSpaceToDia(LENGTH - 1, LENGTH - 1);
}

// boolean isPatternMatching()
// {
//   return smiilarity <= 10000;
// }

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

  delay(100);
}

void loop()
{
  sensors_event_t acc, gyro, temp;
  mpu.getEvent(&acc, &gyro, &temp);

  // plotDatarecData(acc, gyro);

  // recData(acc, gyro);
  // plotData(acc, gyro);
  //  showPattern(acc, accXPatternDown, accYPatternDown, accZPatternDown);

  // detectPattern(acc, gyro);

  //--- DTW ---//
  recDTWData(acc, gyro);
  if (dtwRecCount >= LENGTH) // start recognition
  {
    Serial.println("\nStart dtw recognition");
    dwtAnalysis();
    Serial.print("\n  Similarity: ");
    Serial.println(similarity);
    if (similarity < 10000)
    { // TODO: adapt threshold
      Serial.println("  Gesture recognized!");
      perfomAction1();
    }
    else
    {
      Serial.println("  Wrong gesture");
    }
    dtwRecCount = 0;
    Serial.println("  ->Dtw finished");
  }

  delay(10);
}