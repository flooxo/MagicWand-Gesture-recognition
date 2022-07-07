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
The serial output is not needed for functionality, but for debugging. It is set up to work with Putty and has some nice colors to make debugging easier.

### Components:
* ESP32 (DevKitC V4)
* MPU6050 gyroscope
* IR diode
* cables

## Roadmap/ToDos:
- [x] Recognize simple gesture
- [x] Recognize multiple gestures
- [ ] Local constraints in the DTW cost matrix to aboard the calculation if outside of range
- [ ] Auto-recognition of gestures
- [ ] Add 3D files
