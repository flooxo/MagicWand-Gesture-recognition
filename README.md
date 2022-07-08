# MagicWand-Gesture-recognition

### Real working magic wand
Using an Arduino and a gyroscope/accelerometer as input for gesture recognition, commands can be executed in an almost magical way.
It is encased in a 3D-printed housing that looks like Harry Potter's wand.
<br />
### Functionality:
The accelerometer data is used in the Dynamic Time Warp algorithm to calculate the similarity compared to a previously recorded pattern.
If the pattern matches (=below a certain threshold), an action is executed. In my case I wanted to control my LED strips, so I send an IR signal
when the gesture is detected.
<br />
The serial output is not needed for functionality, but for debugging. It is set up to work with Putty/VSCode and has some nice colors to make debugging easier. (https://xdevs.com/guide/color_serial/)

![image](https://user-images.githubusercontent.com/93255373/177857505-fcd9639e-0442-45ab-9a4b-7e79443cd54a.png)
<br />
*accelerometer data of one axis; green = sample patter, red = wrong pattern, other = similar to sample pattern*
<br /><br />
***How is a gesture recognized?***
<br />
As a preparation, the acceleration data from a certain period are stored in an array, the so-called sample pattern. In a second array the current motion data are stored.
Now a similarity is to be calculated from these two data. This works by means of Dynamic Time Warping (DTW).
DTW tries to create a best possible match of the two time dependent sequences by means of matching, deletion and insertion of data points.
For this there is a so-called cost matrix. This contains calculated values, which are then always connected with the smallest neighbor and thus ideally form a diagonal. The area between the calculated path and the diagonal can be used to determine the similarity. Under a certain threshold it is accepted as a gesture
<br /><br />
![image](https://user-images.githubusercontent.com/93255373/177857310-efb15bd7-fd3d-4c15-999d-4c3c1cd2dfac.png)
<br />
*cost matrix visualization; data from pic above*
<br />
The green diagonal would be if the gesture would match at every point
<br />&nbsp;&nbsp;&nbsp;&nbsp;-> area = 0 => similarity = 100%.<br />
The blue path is a gesture that is very similar to the sample pattern, which you can see because it is very close to the diagonal
<br />&nbsp;&nbsp;&nbsp;&nbsp;-> area ≈ 950 -> similarity = ~95%<br /> 
The red path is the inverse of the sample pattern, which is also clearly visible because it deviates strongly from the diagonal
<br />&nbsp;&nbsp;&nbsp;&nbsp;-> area ≈ 8500 -> similarity = ~20%<br />
In purple is the area that is calculated to get the similarity.

### Components:
* ESP32 (DevKitC V4)
* (ATTiny85/Arduino Pro Micro)
* MPU6050 gyroscope
* IR diode
* cables

![image](https://user-images.githubusercontent.com/93255373/177861540-97b264a6-fc60-43e1-ac3d-29d83a6cb6f2.png) <br />
_connection sketch_

## Roadmap/ToDos:
- [x] Recognize simple gesture
- [x] Recognize multiple gestures
- [ ] Local constraints in the DTW cost matrix to aboard the calculation if outside of range
- [ ] Auto-recognition of gestures
- [ ] Add 3D files
- [ ] ArduinoProMicro/ATTiny85 compatible
- [ ] Substract gyro out of acc data!!!!
- [ ] Serial progress bar for VSCode/Putty (when reccording or calcualting)

## Plans:
![20220707_204939](https://user-images.githubusercontent.com/93255373/177851997-8b7767d9-c346-4ddb-b3e3-dd23a55e031e.jpg)
![20220707_205002](https://user-images.githubusercontent.com/93255373/177852004-b94bef55-0f3d-45ce-a192-484314a0ddd3.jpg)
*Prototommy (Prototype 2nd Gen)*
<br /><br />
![image](https://user-images.githubusercontent.com/93255373/177853735-520d2834-567e-47ed-a77f-de22d1c1876e.png)
*3D case plan*

### Miscellaneous:
Actually it was planned not to use an ESP32 but originally an ATTiny85. However, this is for the development and debugging rather less suitable. The advantage would be that this is much smaller and can be operated longer with a battery.
As a compromise I still have an Arduino Pro Micro lying around, maybe the time will find that I can use this, but the memory must be adapted, since this is much smaller than that of the ESP32.

- - - -
###### all information without guarantee of correctness, is only my hobby on the side; for questions, suggestions or improvements please contact me
