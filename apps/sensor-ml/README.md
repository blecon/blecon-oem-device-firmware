 # ML Inference application

 This application shows how an Edge Impulse-generated inference model can be integrated within a Blecon device.

 The Edge Impulse [continuous motion recognition](https://studio.edgeimpulse.com/public/84984/latest) demo is integrated and all inference results are logged.

 Additionally, inference results will also be displayed using the device's LEDs if the device's button has been pressed within a configurable period:
 * Orange: Wave
 * Red: Up/Down motion
 * Green Snake

 Results are reported within `log` messages in the `ml-sensor` namespace.
 