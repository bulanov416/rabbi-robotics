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

package com.qualcomm.hardware.hitechnic;

import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LegacyModulePortDeviceImpl;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

public class HiTechnicNxtGyroSensor extends LegacyModulePortDeviceImpl implements GyroSensor, AnalogSensor {

  //------------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------------

  public HiTechnicNxtGyroSensor(LegacyModule legacyModule, int physicalPort) {
    super(legacyModule, physicalPort);
    finishConstruction();
  }

  @Override
  protected void moduleNowArmedOrPretending() {
    module.enableAnalogReadMode(physicalPort);
    }

  //------------------------------------------------------------------------------------------------
  // Operations
  //------------------------------------------------------------------------------------------------

  @Override
  public String toString() {
    return String.format("Gyro: %3.1f", getRotationFraction());
  }

/**
   * Method not supported by hardware.
   * @return nothing
   * @throws UnsupportedOperationException
   */
  @Override
  public void calibrate() {
    // nothing to do
  }

  /**
   * Method not supported by hardware.
   * @return nothing
   * @throws UnsupportedOperationException
   */
  @Override
  public boolean isCalibrating() {
    return false;
  }

  @Override
  public double getRotationFraction() {
    return readRawVoltage() / getMaxVoltage();
  }

  @Override
  public double readRawVoltage()  {
    return module.readAnalogVoltage(physicalPort);
  }

  public double getMaxVoltage() {
    // The sensor itself is a 5v sensor, reporting analog values from 0v to 5v. However, depending
    // on the level conversion hardware that might be between us and the sensor, that may get shifted
    // to a different range. We'll assume that we only ever shift *down* in range, not up, so we
    // can take the min of the sensor's natural level and what the input controller can do.
    final double sensorMaxVoltage = 5.0;
    return Math.min(sensorMaxVoltage, module.getMaxAnalogInputVoltage());
  }

  /**
   * Method not supported by hardware.
   * @return nothing
   * @throws UnsupportedOperationException
   */
  @Override
  public int getHeading() {
    notSupported();
    return 0;
  }

  /**
   * Method not supported by hardware.
   * @return nothing
   * @throws UnsupportedOperationException
   */
  @Override
  public int rawX() {
    notSupported();
    return 0;
  }

  /**
   * Method not supported by hardware.
   * @return nothing
   * @throws UnsupportedOperationException
   */
  @Override
  public int rawY() {
    notSupported();
    return 0;
  }

  /**
   * Method not supported by hardware.
   * @return nothing
   * @throws UnsupportedOperationException
   */
  @Override
  public int rawZ() {
    notSupported();
    return 0;
  }

  /**
   * Method not supported by hardware.
   * @return nothing
   * @throws UnsupportedOperationException
   */
  @Override
  public void resetZAxisIntegrator() {
    // nothing to do
  }

  @Override
  public String status() {
    return String.format("NXT Gyro Sensor, connected via device %s, port %d",
        module.getSerialNumber().toString(), physicalPort);
  }

  @Override public Manufacturer getManufacturer() {
    return Manufacturer.HiTechnic;
  }

  @Override
  public String getDeviceName() {
    return "NXT Gyro Sensor";
  }

  @Override
  public String getConnectionInfo() {
    return module.getConnectionInfo() + "; port " + physicalPort;
  }

  @Override
  public int getVersion() {
    return 1;
  }

  @Override
  public void resetDeviceConfigurationForOpMode() {
    // nothing to do
  }

  @Override
  public void close() {
    // take no action
  }

  protected void notSupported() {
    throw new UnsupportedOperationException("This method is not supported for " + getDeviceName());
  }

}
