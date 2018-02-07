# Sodaq_LSM303AGR

Arduino library for the ST LSM303AGR.

## Usage

Method|Description
------|------
**Sodaq_LSM303AGR(TwoWire& wire = Wire, uint8_t accelAddress = SODAQ_LSM303AGR_ACCEL_ADDRESS, uint8_t magAddress = SODAQ_LSM303AGR_MAG_ADDRESS)**|Constructor with optional Wire instance and address parameters.
**getTemperatureDelta()**|Returns the temperature delta as read from the sensor (8-bit int).
**enableAccelerometer(AccelerometerMode mode = NormalMode, AccelerometerODR odr = HrNormalLowPower25Hz, Axes axes = XYZ, Scale scale = Scale2g, bool isTemperatureOn = true)**|Enables the accelerometer sensor with the default values or the given data rate, axes and scale.
**disableAccelerometer()**|Disables the accelerometer sensor and puts it in the lowest power consumption state possible.
**rebootAccelerometer()**|Resets all the registers of the accelerometre sensor.
**disableMagnetometer()**|Disables the magnetometer sensor and puts it in the lowest power consumption state possible.
**enableInterrupt1(uint8_t axesEvents, float threshold, uint8_t duration, InterruptMode interruptMode = MovementRecognition)**|Enables the accelerometer interrupt on INT1 pin using the given triggger criteria. The axesEvents can be multiple ORed AxesEvents.
**enableInterrupt2(uint8_t axesEvents, float threshold, uint8_t duration, InterruptMode interruptMode = MovementRecognition)**|Enables the accelerometer interrupt on INT2 pin using the given triggger criteria. The axesEvents can be multiple ORed AxesEvents.
**getX()**|Returns a measurement of the X axis of the accelerometer.
**getY()**|Returns a measurement of the Y axis of the accelerometer.
**getZ()**|Returns a measurement of the Z axis of the accelerometer.

## Contributing

1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -am 'Add some feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Submit a pull request


## License

Copyright (c) 2018 SODAQ. All rights reserved.

This file is part of Sodaq_LSM303AGR.

Sodaq_LSM303AGR is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of
the License, or(at your option) any later version.

Sodaq_LSM303AGR is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with Sodaq_LSM303AGR.  If not, see
<http://www.gnu.org/licenses/>.
