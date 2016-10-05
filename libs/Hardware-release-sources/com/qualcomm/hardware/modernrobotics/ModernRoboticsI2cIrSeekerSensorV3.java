/*
 * Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.qualcomm.hardware.modernrobotics;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;
import java.util.concurrent.locks.Lock;

/**
 * Modern Robotics I2C IR Seeker Sensor
 */
public class ModernRoboticsI2cIrSeekerSensorV3 extends I2cControllerPortDeviceImpl implements IrSeekerSensor, I2cController.I2cPortReadyCallback {

  //------------------------------------------------------------------------------------------------
  // Constants
  //------------------------------------------------------------------------------------------------

  public static final I2cAddr DEFAULT_I2C_ADDRESS = I2cAddr.create8bit(0x38);
  public static final int ADDRESS_MEM_START = 0x04; // beginning of useful data
  public static final int MEM_LENGTH = 0x0c; // 12 bytes of useful data.

  public static final int OFFSET_1200HZ_HEADING_DATA = 0x00 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_1200HZ_SIGNAL_STRENGTH = 0x01 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_600HZ_HEADING_DATA = 0x02 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_600HZ_SIGNAL_STRENGTH = 0x03 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;

  // lsb:msb
  public static final int OFFSET_1200HZ_LEFT_SIDE_RAW_DATA = 0x04 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_1200HZ_RIGHT_SIDE_RAW_DATA = 0x06 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_600HZ_LEFT_SIDE_RAW_DATA = 0x08 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_600HZ_RIGHT_SIDE_RAW_DATA = 0x0a + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;

  public static final byte SENSOR_COUNT = 2;
  public static final double MAX_SENSOR_STRENGTH = 256.0;
  public static final byte INVALID_ANGLE = 0;

  public static final double DEFAULT_SIGNAL_DETECTED_THRESHOLD = 1.0 / MAX_SENSOR_STRENGTH;

  //------------------------------------------------------------------------------------------------
  // State
  //------------------------------------------------------------------------------------------------

  private volatile I2cAddr i2cAddr = DEFAULT_I2C_ADDRESS; // this can be changed by the user.
  private Mode         mode;
  private byte[]       readCacheBuffer;
  private Lock         readCacheLock;
  private double       signalDetectedThreshold = DEFAULT_SIGNAL_DETECTED_THRESHOLD;

  //------------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------------

  public ModernRoboticsI2cIrSeekerSensorV3(I2cController module, int physicalPort) {
    super(module, physicalPort);

    this.mode = Mode.MODE_1200HZ;

    finishConstruction();
  }

  @Override
  protected void controllerNowArmedOrPretending() {
    this.readCacheBuffer = this.controller.getI2cReadCache(physicalPort);
    this.readCacheLock = this.controller.getI2cReadCacheLock(physicalPort);

    this.controller.enableI2cReadMode(physicalPort, i2cAddr, ADDRESS_MEM_START, MEM_LENGTH);
    this.controller.setI2cPortActionFlag(physicalPort);
    this.controller.writeI2cCacheToController(physicalPort);

    this.controller.registerForI2cPortReadyCallback(this, physicalPort);
    }

  //------------------------------------------------------------------------------------------------
  // Operations
  //------------------------------------------------------------------------------------------------

  @Override
  public String toString() {
    if (signalDetected()) {
      return String.format("IR Seeker: %3.0f%% signal at %6.1f degrees", getStrength() * 100.0, getAngle());
      }
    else {
      return "IR Seeker:  --% signal at  ---.- degrees";
      }
    }

  @Override
  public synchronized void setSignalDetectedThreshold(double threshold) {
    signalDetectedThreshold = threshold;
  }

  @Override
  public double getSignalDetectedThreshold() {
    return signalDetectedThreshold;
  }

  @Override
  public synchronized void setMode(Mode mode) {
    this.mode = mode;
  }

  @Override
  public Mode getMode() {
    return mode;
  }

  @Override
  public boolean signalDetected() {
    return (getStrength() > signalDetectedThreshold);
  }

  @Override
  public synchronized double getAngle() {

    double angle = 0;
    int headingOffset = (mode == Mode.MODE_1200HZ) ? OFFSET_1200HZ_HEADING_DATA : OFFSET_600HZ_HEADING_DATA;

    try {
      readCacheLock.lock();
      angle = readCacheBuffer[headingOffset];
    } finally {
      readCacheLock.unlock();
    }

    return angle;
  }

  @Override
  public synchronized double getStrength() {

    double strength = 0;
    int strengthOffset = (mode == Mode.MODE_1200HZ) ? OFFSET_1200HZ_SIGNAL_STRENGTH : OFFSET_600HZ_SIGNAL_STRENGTH;

    try {
      readCacheLock.lock();
      strength = TypeConversion.unsignedByteToDouble(readCacheBuffer[strengthOffset]) / MAX_SENSOR_STRENGTH;
    } finally {
      readCacheLock.unlock();
    }

    return strength;
  }

  // Returns right and left raw values, scaled from -1 to 1.
  @Override
  public synchronized IrSeekerIndividualSensor[] getIndividualSensors() {
    IrSeekerIndividualSensor sensors[] = new IrSeekerIndividualSensor[SENSOR_COUNT];

    // we don't know the angle of these sensors so we will give a bad estimate; -1 for left, +1 for right

    try {
      readCacheLock.lock();

      int leftSideRawOffset = (mode == Mode.MODE_1200HZ) ? OFFSET_1200HZ_LEFT_SIDE_RAW_DATA : OFFSET_600HZ_LEFT_SIDE_RAW_DATA;
      byte[] rawLeftValues = new byte[2];
      System.arraycopy(readCacheBuffer, leftSideRawOffset, rawLeftValues, 0, rawLeftValues.length);
      double strengthLeft = TypeConversion.byteArrayToShort(rawLeftValues, ByteOrder.LITTLE_ENDIAN) / MAX_SENSOR_STRENGTH;
      sensors[0] = new IrSeekerIndividualSensor(-1, strengthLeft);

      int rightSideRawOffset = (mode == Mode.MODE_1200HZ) ? OFFSET_1200HZ_RIGHT_SIDE_RAW_DATA : OFFSET_600HZ_RIGHT_SIDE_RAW_DATA;
      byte[] rawRightValues = new byte[2];
      System.arraycopy(readCacheBuffer, rightSideRawOffset, rawRightValues, 0, rawRightValues.length);
      double strengthRight = TypeConversion.byteArrayToShort(rawRightValues, ByteOrder.LITTLE_ENDIAN) / MAX_SENSOR_STRENGTH;
      sensors[1] = new IrSeekerIndividualSensor(1, strengthRight);
    } finally {
      readCacheLock.unlock();
    }

    return sensors;
  }

  /*
   * Callback method, will be called by the Device Interface Module when the port is ready, assuming we
   * registered that call
   */
  public void portIsReady(int port) {
    controller.setI2cPortActionFlag(port);
    controller.readI2cCacheFromController(port);
    controller.writeI2cPortFlagOnlyToController(port);
  }

  @Override public Manufacturer getManufacturer() {
    return Manufacturer.ModernRobotics;
  }

  @Override
  public String getDeviceName() {
    return "Modern Robotics I2C IR Seeker Sensor";
  }

  @Override
  public String getConnectionInfo() {
    return controller.getConnectionInfo() + "; I2C port " + physicalPort;
  }

  @Override
  public int getVersion() {
    return 3;
  }

  @Override
  public void resetDeviceConfigurationForOpMode() {
  }

  @Override
  public void close() {

  }

  @Override
  public synchronized void setI2cAddress(I2cAddr newAddress) {
    ModernRoboticsUsbDeviceInterfaceModule.throwIfModernRoboticsI2cAddressIsInvalid(newAddress);
    RobotLog.i(getDeviceName() + ", just changed the I2C address. Original address: " + i2cAddr + ", new address: " + newAddress);

    i2cAddr = newAddress;

    controller.enableI2cReadMode(physicalPort, i2cAddr, ADDRESS_MEM_START, MEM_LENGTH);
    controller.setI2cPortActionFlag(physicalPort);
    controller.writeI2cCacheToController(physicalPort);

    controller.registerForI2cPortReadyCallback(this, physicalPort);
  }

  @Override
  public I2cAddr getI2cAddress() {
    return i2cAddr;
  }
}
