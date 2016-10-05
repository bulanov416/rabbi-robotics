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
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Touch Sensor
 */
public class ModernRoboticsDigitalTouchSensor implements TouchSensor {

  private DigitalChannelController module = null;
  private int physicalPort = -1;

  /**
   * Constructor
   *
   * @param module Device Interface Module this device is attached to
   * @param physicalPort physical port on the Device Interface Module
   */
  public ModernRoboticsDigitalTouchSensor(DigitalChannelController module, int physicalPort) {
    this.module = module;
    this.physicalPort = physicalPort;
  }

  @Override
  public String toString() {
    return String.format("Touch Sensor: %1.2f", getValue());
  }

  @Override
  public double getValue() {
    return isPressed() ? 1.0 : 0.0;
  }

  @Override
  public boolean isPressed() {
    return module.getDigitalChannelState(physicalPort);
  }

  @Override public Manufacturer getManufacturer() {
    return Manufacturer.ModernRobotics;
  }

  @Override
  public String getDeviceName() {
    return "Modern Robotics Digital Touch Sensor";
  }

  @Override
  public String getConnectionInfo() {
    return module.getConnectionInfo()+ "; digital port " + physicalPort;
  }

  @Override
  public int getVersion() {
    return 1;
  }

  @Override
  public void resetDeviceConfigurationForOpMode() {
  }

  @Override
  public void close() {

  }
}
