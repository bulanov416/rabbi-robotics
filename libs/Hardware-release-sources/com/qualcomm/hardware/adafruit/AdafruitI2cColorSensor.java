/*
 * Copyright (c) 2015 Qualcomm Technologies Inc
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

package com.qualcomm.hardware.adafruit;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDeviceImpl;

import java.util.concurrent.locks.Lock;

/**
 * @deprecated This should be retired in favor of AMSColorSensorImpl
 */
@Deprecated
public class AdafruitI2cColorSensor extends I2cControllerPortDeviceImpl implements ColorSensor, I2cController.I2cPortReadyCallback {

  //------------------------------------------------------------------------------------------------
  // State
  //------------------------------------------------------------------------------------------------

  public static final int I2C_ADDRESS_TCS34725 = 0x29;
  public static final int TCS34725_COMMAND_BIT = 0x80;
  public static final int TCS34725_ID = 0x12;

  public static final int ADDRESS_TCS34725_ENABLE = 0x00;
  public static final int TCS34725_ENABLE_AIEN = 0x10;  //Enable RGB Interrupt
  public static final int TCS34725_ENABLE_AEN = 0x02;   //RGBC Enable - this bit high activates the ADC
  public static final int TCS34725_ENABLE_PON = 0x01;   //Power on - this bit high activates the internal oscillator

  public static final int TCS34725_CDATAL = 0x14;   //Clear channel data register
  public static final int TCS34725_RDATAL = 0x16;   //Red channel data register for low bit
  public static final int TCS34725_GDATAL = 0x18;   //Green Channel data register for low bit
  public static final int TCS34725_BDATAL = 0x1A;   //Blue channel data register for low bit

  public static final int OFFSET_ALPHA_LOW_BYTE = ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_ALPHA_HIGH_BYTE = 0x01 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_RED_LOW_BYTE = 0x02 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_RED_HIGH_BYTE = 0x03 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_GREEN_LOW_BYTE = 0x04 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_GREEN_HIGH_BYTE = 0x05 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_BLUE_LOW_BYTE = 0x06 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;
  public static final int OFFSET_BLUE_HIGH_BYTE = 0x07 + ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER;

  private       byte[] readBuffer;
  private       Lock readLock;
  private       byte[] writeBuffer;
  private       Lock writeLock;

  private boolean enableWrite = false;
  private boolean enableRead = false;

  //------------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------------

  public AdafruitI2cColorSensor(I2cController module, int physicalPort) {
    super(module, physicalPort);

    this.enableWrite = true;

    finishConstruction();
  }

  @Override
  protected void controllerNowArmedOrPretending() {

    this.readBuffer  = controller.getI2cReadCache(physicalPort);
    this.readLock    = controller.getI2cReadCacheLock(physicalPort);
    this.writeBuffer = controller.getI2cWriteCache(physicalPort);
    this.writeLock   = controller.getI2cWriteCacheLock(physicalPort);

    controller.registerForI2cPortReadyCallback(this, physicalPort);
  }

  //------------------------------------------------------------------------------------------------
  // ColorSensor
  //------------------------------------------------------------------------------------------------

  @Override
  public String toString() {
    return String.format("argb: %d", argb());
  }

  @Override
  public int red() {
    return getColor(OFFSET_RED_HIGH_BYTE, OFFSET_RED_LOW_BYTE);
  }

  @Override
  public int green() {
    return getColor(OFFSET_GREEN_HIGH_BYTE, OFFSET_GREEN_LOW_BYTE);
  }

  @Override
  public int blue() {
    return getColor(OFFSET_BLUE_HIGH_BYTE, OFFSET_BLUE_LOW_BYTE);
  }

  @Override
  public int alpha() {
    return getColor(OFFSET_ALPHA_HIGH_BYTE, OFFSET_ALPHA_LOW_BYTE);
  }

  private int getColor(int HIGH_OFFSET, int LOW_OFFSET) {
    int color;
    try {
      readLock.lock();
      color = (readBuffer[HIGH_OFFSET] << 8) | (readBuffer[LOW_OFFSET] & 0xFF);
    } finally {
      readLock.unlock();
    }
    return color;
  }

  @Override
  public int argb() {
    return Color.argb(alpha(), red(), green(), blue());
  }

  @Override
  public void enableLed(boolean enable) {
    throw new UnsupportedOperationException("enableLed is not implemented.");
  }

  @Override
  public Manufacturer getManufacturer() {
    return Manufacturer.Adafruit;
  }

  @Override
  public String getDeviceName() {
    return "Adafruit I2C Color Sensor";
  }

  @Override
  public String getConnectionInfo() {
    return controller.getConnectionInfo() + "; I2C port: " + physicalPort;
  }

  @Override
  public int getVersion() {
    return 1;
  }

  @Override
  public void resetDeviceConfigurationForOpMode() {
    // do nothing
  }

  @Override
  public void close() {
    // do nothing
  }

  @Override
  public void portIsReady(int port) {

    if (enableWrite) {
      enableWrite();
      enableWrite = false;
      enableRead = true;
    } else if (enableRead) {
      enableRead();
      enableRead = false;
    }
    controller.readI2cCacheFromController(physicalPort);
    controller.setI2cPortActionFlag(physicalPort);
    controller.writeI2cPortFlagOnlyToController(physicalPort);
  }

  private void enableRead() {
    // get latest data
    controller.enableI2cReadMode(physicalPort, I2cAddr.create7bit(I2C_ADDRESS_TCS34725), TCS34725_CDATAL | TCS34725_COMMAND_BIT, 8);
    controller.writeI2cCacheToController(physicalPort);

  }

  private void enableWrite() {
    controller.enableI2cWriteMode(physicalPort, I2cAddr.create7bit(I2C_ADDRESS_TCS34725), ADDRESS_TCS34725_ENABLE | TCS34725_COMMAND_BIT, 1);
    try {
      writeLock.lock();
      //Set the register to power on, and enable RGB reading mode
      writeBuffer[ModernRoboticsUsbDeviceInterfaceModule.OFFSET_I2C_PORT_MEMORY_BUFFER] = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
    } finally {
      writeLock.unlock();
    }
    controller.setI2cPortActionFlag(physicalPort);
    controller.writeI2cCacheToController(physicalPort);
  }

  @Override
  public void setI2cAddress(I2cAddr newAddress) {
    throw new UnsupportedOperationException("setI2cAddress is not supported.");
  }

  @Override
  public I2cAddr getI2cAddress() {
    return I2cAddr.create7bit(I2C_ADDRESS_TCS34725);
  }
}
