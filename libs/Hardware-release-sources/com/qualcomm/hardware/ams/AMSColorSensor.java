/*
Copyright (c) 2016 Robert Atkinson & Steve Geffner

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the names of Robert Atkinson nor Steve Geffner nor the names of their contributors
may be used to endorse or promote products derived from this software without specific
prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.qualcomm.hardware.ams;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * AMSColorSensor is an extension of ColorSensor interface that provides additional functionality
 * supported by a family of color sensor chips from AMS. Notable uses of chips in this family
 * include the <a href="http://adafru.it/1334">AdaFruit color sensor</a> and the Lynx color sensor.
 *
 * @see <a href="http://ams.com/eng/Support/Demoboards/Light-Sensors/(show)/145298">AMS Color Sensors</a>
 */
public interface AMSColorSensor extends ColorSensor
    {
    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    /**
     * Initialize the sensor using the indicated set of parameters.
     * @param parameters the parameters with which to initialize the device
     * @return whether initialization was successful or not
     */
    boolean initialize(Parameters parameters);

    /**
     * Initializes the sensor using the previously attempted set of parameters.
     * @return whether initialization was successful or not
     */
    boolean initialize();

    /**
     * Returns the parameters which which initialization was last attempted, if any
     * @return the parameters which which initialization was last attempted, if any
     */
    Parameters getParameters();

    /**
     * Instances of Parameters contain data indicating how the
     * sensor is to be initialized.
     *
     * @see #initialize(Parameters)
     */
    class Parameters
        {
        /** the device id expected to be reported by the color sensor chip */
        public final int deviceId;

        /** the address at which the sensor resides on the I2C bus. */
        public I2cAddr i2cAddr;

        /** the integration time to use */
        public IntegrationTime integrationTime = IntegrationTime.MS_24;

        /** the gain level to use */
        public Gain gain = Gain.GAIN_4;

        /** set of registers to read in background, if supported by underlying I2cDeviceSynch */
        public I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(IREG_READ_FIRST, IREG_READ_LAST-IREG_READ_FIRST+1, I2cDeviceSynch.ReadMode.REPEAT);

        /** debugging aid: enable logging for this device? */
        public boolean loggingEnabled = false;

        /** debugging aid: the logging tag to use when logging */
        public String loggingTag = "AMSColorSensor";

        public Parameters(I2cAddr i2cAddr, int deviceId)
            {
            this.i2cAddr = i2cAddr;
            this.deviceId    = deviceId;
            }
        public static Parameters createForAdaFruit()
            {
            return new Parameters(I2cAddr.create7bit(AMS_TCS34725_ADDRESS), AMS_TCS34725_ID);
            }
        public static Parameters createForLynx()
            {
            return new Parameters(I2cAddr.create7bit(AMS_TMD37821_ADDRESS), AMS_TMD37821_ID);
            }
        }

    //----------------------------------------------------------------------------------------------
    // Status inquiry
    //----------------------------------------------------------------------------------------------

    /**
     * Returns the flavor of the AMS color sensor as reported by the chip itself
     * @return the flavor of the AMS color sensor as reported by the chip itself
     */
    byte getDeviceID();

    //----------------------------------------------------------------------------------------------
    // Low level reading and writing
    //----------------------------------------------------------------------------------------------

    /**
     * Low level: read the byte starting at the indicated register
     *
     * @param register the location from which to read the data
     * @return the data that was read
     */
    byte read8(Register register);

    /**
     * Low level: read data starting at the indicated register
     *
     * @param register the location from which to read the data
     * @param cb       the number of bytes to read
     * @return the data that was read
     */
    byte[] read(Register register, int cb);

    /**
     * Low level: read two bytes of data starting at the indicated register
     * and return the results as an unsigned integer
     *
     * @param reg the location from which to read the data; should be an integer register.
     * @return the data that was read
     */
    int readUnsignedShort(Register reg);

    /**
     * Low level: write a byte to the indicated register
     *
     * @param register the location at which to write the data
     * @param bVal     the data to write
     */
    void write8(Register register, int bVal);

    /**
     * Low level: write data starting at the indicated register
     *
     * @param register the location at which to write the data
     * @param data     the data to write
     */
    void write(Register register, byte[] data);

    /**
     * Low level: write two bytes of data starting at the indicated register
     *
     * @param ireg  the location into which to write the data; should be an integer register.
     * @param value the integer to
     */
    void writeShort(Register ireg, int value);

    //------------------------------------------------------------------------------------------
    // Constants
    //------------------------------------------------------------------------------------------

    /**
     * REGISTER provides symbolic names for interesting device registers
     */
    enum Register
        {
            ENABLE(0x00),
            ATIME(0x01),
            CONFIGURATION(0x0D),
            CONTROL(0x0F),
            DEVICE_ID(0x12),
            STATUS(0x13),
            CLEAR(0x14),   //clear color value: 0x14 = low byte, 0x15 = high byte
            RED(0x16),     //red color value: 0x16 = low byte, 0x17 = high byte
            GREEN(0x18),   //etc.
            BLUE(0x1A);

        public final byte byteVal;
        Register(int i) { this.byteVal = (byte) i; }
        }

    enum Gain
        {
            GAIN_1(0x00),
            GAIN_4(0x01),
            GAIN_16(0x02),
            GAIN_64(0x03);

        public final byte byteVal;
        Gain(int i) { this.byteVal = (byte) i; }
        }

    enum IntegrationTime
        {
            MS_2_4(0xFF),
            MS_24(0xF6),
            MS_50(0xEB),
            MS_101(0xD5),
            MS_154(0xC0),
            MS_700(0x00);

        public final byte byteVal;
        IntegrationTime(int i) { this.byteVal = (byte) i; }
        }

    /*
        ADDRESS NAME        R/W     FUNCTION                            RESET VALUE
        −−     COMMAND     W       Specifies register address              0x00
        0x00    ENABLE      R/W     Enables states and interrupts           0x00
        0x01    ATIME       R/W     RGBC time                               0xFF
        0x03    WTIME       R/W     Wait time                               0xFF
        0x04    AILTL       R/W     Clear interrupt low threshold low byte  0x00
        0x05    AILTH       R/W     Clear interrupt low threshold high byte 0x00
        0x06    AIHTL       R/W     Clear interrupt high threshold low byte 0x00
        0x07    AIHTH       R/W     Clear interrupt high threshold high byte 0x00
        0x0C    PERS        R/W     Interrupt persistence filter            0x00
        0x0D    CONFIG      R/W     Configuration                           0x00
        0x0F    CONTROL     R/W     Control                                 0x00
        0x12    ID          R       Device ID                                ID
        0x13    STATUS      R       Device status                           0x00
        0x14    CDATAL      R       Clear data low byte                     0x00
        0x15    CDATAH      R       Clear data high byte                    0x00
        0x16    RDATAL      R       Red data low byte                       0x00
        0x17    RDATAH      R       Red data high byte                      0x00
        0x18    GDATAL      R       Green data low byte                     0x00
        0x19    GDATAH      R       Green data high byte                    0x00
        0x1A    BDATAL      R       Blue data low byte                      0x00
        0x1B    BDATAH      R       Blue data high byte                     0x00
        */
    //----------------------------------------------------------------------------------------------

    // The 7-bit I2C address of this device
    int AMS_TCS34725_ADDRESS = 0x29;
    int AMS_TMD37821_ADDRESS = 0x39;

    byte AMS_TCS34725_ID = 0x44;
    byte AMS_TMD37821_ID = 0x60;
    byte AMS_TMD37823_ID = 0x69;

    int AMS_COLOR_COMMAND_BIT = 0x80;

    int AMS_COLOR_ENABLE = 0x00;
    int AMS_COLOR_ENABLE_PIEN = 0x20;        /* Proximity interrupt enable */
    int AMS_COLOR_ENABLE_AIEN = 0x10;        /* RGBC Interrupt Enable */
    int AMS_COLOR_ENABLE_WEN = 0x08;         /* Wait enable - Writing 1 activates the wait timer */
    int AMS_COLOR_ENABLE_PEN = 0x04;         /* Proximity enable */
    int AMS_COLOR_ENABLE_AEN = 0x02;         /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
    int AMS_COLOR_ENABLE_PON = 0x01;         /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
    int AMS_COLOR_ATIME = 0x01;              /* Integration time */
    int AMS_COLOR_WTIME = 0x03;              /* Wait time = if AMS_COLOR_ENABLE_WEN is asserted; */
    int AMS_COLOR_WTIME_2_4MS = 0xFF;        /* WLONG0 = 2.4ms   WLONG1 = 0.029s */
    int AMS_COLOR_WTIME_204MS = 0xAB;        /* WLONG0 = 204ms   WLONG1 = 2.45s  */
    int AMS_COLOR_WTIME_614MS = 0x00;        /* WLONG0 = 614ms   WLONG1 = 7.4s   */
    int AMS_COLOR_AILTL = 0x04;              /* Clear channel lower interrupt threshold */
    int AMS_COLOR_AILTH = 0x05;
    int AMS_COLOR_AIHTL = 0x06;              /* Clear channel upper interrupt threshold */
    int AMS_COLOR_AIHTH = 0x07;
    int AMS_COLOR_PERS = 0x0C;               /* Persistence register - basic SW filtering mechanism for interrupts */
    int AMS_COLOR_PERS_NONE = 0b0000;        /* Every RGBC cycle generates an interrupt                                */
    int AMS_COLOR_PERS_1_CYCLE = 0b0001;     /* 1 clean channel value outside threshold range generates an interrupt   */
    int AMS_COLOR_PERS_2_CYCLE = 0b0010;     /* 2 clean channel values outside threshold range generates an interrupt  */
    int AMS_COLOR_PERS_3_CYCLE = 0b0011;     /* 3 clean channel values outside threshold range generates an interrupt  */
    int AMS_COLOR_PERS_5_CYCLE = 0b0100;     /* 5 clean channel values outside threshold range generates an interrupt  */
    int AMS_COLOR_PERS_10_CYCLE = 0b0101;    /* 10 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_15_CYCLE = 0b0110;    /* 15 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_20_CYCLE = 0b0111;    /* 20 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_25_CYCLE = 0b1000;    /* 25 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_30_CYCLE = 0b1001;    /* 30 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_35_CYCLE = 0b1010;    /* 35 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_40_CYCLE = 0b1011;    /* 40 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_45_CYCLE = 0b1100;    /* 45 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_50_CYCLE = 0b1101;    /* 50 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_55_CYCLE = 0b1110;    /* 55 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_PERS_60_CYCLE = 0b1111;    /* 60 clean channel values outside threshold range generates an interrupt */
    int AMS_COLOR_CONFIG = 0x0D;
    int AMS_COLOR_CONFIG_NORMAL = 0x00;      /* normal wait times */
    int AMS_COLOR_CONFIG_WLONG = 0x02;       /* Extended wait time = 12x normal wait times via AMS_COLOR_WTIME */
    int AMS_COLOR_CONTROL = 0x0F;            /* Set the gain level for the sensor */
    int AMS_COLOR_GAIN_1 = 0x00;             /* normal gain */
    int AMS_COLOR_GAIN_4 = 0x01;             /* 4x gain */
    int AMS_COLOR_GAIN_16 = 0x02;            /* 16x gain */
    int AMS_COLOR_GAIN_60 = 0x03;            /* 60x gain */
    int AMS_COLOR_ID = 0x12;                 /* 0x44 = TCS34721/AMS_COLOR, 0x4D = TCS34723/TCS34727 */
    int AMS_COLOR_STATUS = 0x13;
    int AMS_COLOR_STATUS_AINT = 0x10;        /* RGBC Clean channel interrupt */
    int AMS_COLOR_STATUS_AVALID = 0x01;      /* Indicates that the RGBC channels have completed an integration cycle */
    int AMS_COLOR_CDATAL = 0x14;             /* Clear channel data */
    int AMS_COLOR_CDATAH = 0x15;
    int AMS_COLOR_RDATAL = 0x16;             /* Red channel data */
    int AMS_COLOR_RDATAH = 0x17;
    int AMS_COLOR_GDATAL = 0x18;             /* Green channel data */
    int AMS_COLOR_GDATAH = 0x19;
    int AMS_COLOR_BDATAL = 0x1A;             /* Blue channel data */
    int AMS_COLOR_BDATAH = 0x1B;

    // Registers we used to read-ahead, if supported
    int IREG_READ_FIRST = AMS_COLOR_CDATAL;
    int IREG_READ_LAST  = AMS_COLOR_BDATAH;
    }

