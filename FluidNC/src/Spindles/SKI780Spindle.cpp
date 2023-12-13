// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    SKI780Spindle.cpp

    This is for the new SKI780 VFD based spindle via RS485 Modbus.

                         WARNING!!!!
    VFDs are very dangerous. They have high voltages and are very powerful
    Remove power before changing bits.

    The documentation is okay once you get how it works, but unfortunately
    incomplete... See H2ASpindle.md for the remainder of the docs that I
    managed to piece together.
*/

#include "SKI780Spindle.h"

namespace Spindles {
    SKI780::SKI780() : VFD() {}

    void SKI780::direction_command(SpindleState mode, ModbusCommand& data) {
        data.tx_length = 6;
        data.rx_length = 6;

        data.msg[1] = 0x06;  // WRITE
        data.msg[2] = 0x20;  // Command ID 0x2000
        data.msg[3] = 0x00;
        data.msg[4] = 0x00;
        data.msg[5] = (mode == SpindleState::Ccw) ? 0x02 : (mode == SpindleState::Cw ? 0x01 : 0x06);
    }

    void SKI780::set_speed_command(uint32_t dev_speed, ModbusCommand& data) {
        // NOTE: SKI780 inverters are a-symmetrical. You set the speed in 1/100 
        // percentages, and you get the speed in RPM. So, we need to convert
        // the RPM using maxRPM to a percentage. See MD document for details.
        //
        // For the SKI780 VFD, the speed is not readable - you can only set hz unlike many
        // other VFDs where it is given in Hz times some scale factor.
        // To set the speed, you create a percentage value * 100 so 100.00% = 10000  
        data.tx_length = 6;
        data.rx_length = 6;
        uint32_t speed_acc = (uint32_t)dev_speed * 10000UL;
        speed_acc /=  (uint32_t)_maxFrequency;
        speed_acc *= 10000UL;
        uint16_t speed = (uint32_t)(speed_acc / 6000UL);     
        if (speed < 0) {
            speed = 0;
        }
        if (speed > 10000) {
            speed = 10000;
        }
#ifdef DEBUG_VFD
        log_debug("SKI780 speed: " << dev_speed << " Max freq: " << _maxFrequency << " (X100 Intermediate : " << speed_acc);
        log_debug("SKI780 rpm: " << dev_speed << " percentage of max: " << speed << " % (x100)");
#endif
        data.msg[1] = 0x06;  // WRITE
        data.msg[2] = 0x10;  // Command ID 0x1000
        data.msg[3] = 0x00;
        data.msg[4] = speed >> 8;
        data.msg[5] = speed & 0xFF;
    }

    VFD::response_parser SKI780::initialization_sequence(int index, ModbusCommand& data) {
        if (index == -1) {
            data.tx_length = 6;
            data.rx_length = 6;

            // Send: 01 03 F00E 0001
            data.msg[1] = 0x03;  // READ
            data.msg[2] = 0xF0;  // P0.14 = Get Min Hz
            data.msg[3] = 0x0e;
            data.msg[4] = 0x00;  // Read 1 value
            data.msg[5] = 0x01;

            //  Recv: 01 03 00 02 7D 00
            //                    -- -- = 32000 (val #1) 320.00 Hz
            return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                auto ski780           = static_cast<SKI780*>(vfd);
                ski780->_minFrequency = (uint16_t(response[4]) << 8) | uint16_t(response[5]);
#ifdef DEBUG_VFD
                log_debug("SKI780 allows minimum frequency of:" << ski780->_minFrequency << " Hz (x100)");
#endif
                return true;
            };
        } else if (index == -2) {
            data.tx_length = 6;
            data.rx_length = 6;

            // Send: 01 03 B005 0002
            data.msg[1] = 0x03;  // READ
            data.msg[2] = 0xF0;  // P0.10 = Get Max Hz
            data.msg[3] = 0x0a;
            data.msg[4] = 0x00;  // Read 1 value
            data.msg[5] = 0x01;

            //  Recv: 01 03 00 02 7D 00
            //                    -- -- = 32000 (val #1) 320.00 Hz
            return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
                auto ski780           = static_cast<SKI780*>(vfd);
                ski780->_maxFrequency = (uint16_t(response[4]) << 8) | uint16_t(response[5]);

                // frequency is in Hz * 100, so RPM is frequency * 60 / 100 = frequency * 6 /10
                // E.g. for 400 Hz, we have frequency = 40000, so 40000 * 0.6 = 24000 RPM

                if (vfd->_speeds.size() == 0) {
                    // Convert from frequency in deciHz to RPM (*60/10)
                    SpindleSpeed maxRPM = (ski780->_maxFrequency * 600) / 1000;
                    SpindleSpeed minRPM = (ski780->_minFrequency * 600) / 1000;

                    vfd->shelfSpeeds(minRPM, maxRPM);
                }

                vfd->setupSpeeds((ski780->_maxFrequency *600) /1000);
                vfd->_slop = std::max(ski780->_maxFrequency / 40, 1);

                //                vfd->_min_rpm = uint32_t(vfd->_max_rpm) * uint32_t(ski780->_minFrequency) /
                //                                uint32_t(yl620->_maxFrequency);  //   1000 * 24000 / 4000 =   6000 RPM.

#ifdef DEBUG_VFD
                log_debug("SKI780 allows maximum frequency " << ski780->_maxFrequency << " Hz (x100)");
#endif

                return true;
            };
        } else {
            return nullptr;
        }
    }

    VFD::response_parser SKI780::get_current_speed(ModbusCommand& data) {
        data.tx_length = 6;
        data.rx_length = 6;

        // Send: 01 03 700C 0002
        data.msg[1] = 0x03;  // READ
        data.msg[2] = 0x10;  // 10.01 = Get speed - Actually use 10 07
        data.msg[3] = 0x01;
        data.msg[4] = 0x00;  // Read 1 value
        data.msg[5] = 0x01;

        //  Recv: 01 03 0002 095D 0000
        //                   ---- = 2397 (val #1)
        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
            uint32_t Hz = (uint32_t(response[4]) << 8) | uint32_t(response[5]);
           if (response[1] != 0x03) {
#ifdef DEBUG_VFD_ALL
                log_debug("SKI780: bad read response type: " << response[1]);
#endif 
                return false;
            }

            // We expect a result length of 2 bytes
            if (response[2] != 0 || response[3] != 2) {
#ifdef DEBUG_VFD_ALL
                log_debug("SKI780: bad read response length: " << response[2] << " " << response[3]);
#endif 
                return false;
            }

#ifdef DEBUG_VFD
            log_debug("SKI780 at frequency " << Hz << " Hz (x100)");
#endif      
        //   rpm = Hz*60 /100
        //   
            Hz *= 600;
            Hz /= 1000UL;
            vfd->_sync_dev_speed = Hz;
#ifdef DEBUG_VFD
             log_debug("SKI780 at speed " << vfd->_sync_dev_speed << " RPM");
#endif
            return true;
        };
    }

    VFD::response_parser SKI780::get_current_direction(ModbusCommand& data) {
        data.tx_length = 6;
        data.rx_length = 6;

        // Send: 01 03 30 00 00 01
        data.msg[1] = 0x03;  // READ
        data.msg[2] = 0x30;  // Command group ID
        data.msg[3] = 0x00;
        data.msg[4] = 0x00;  // Message ID
        data.msg[5] = 0x01;

        // Receive: 01 03 00 02 00 02
        //                      ----- status

        // TODO: What are we going to do with this? Update vfd state?
        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
            if (response[1] != 0x03) {
                return false;
            }

            // We expect a result length of 2 bytes
            if (response[2] != 0 || response[3] != 2) {
                return false;
            }

            uint16_t status = (uint16_t(response[4]) << 8) | uint16_t(response[5]);

            //TODO: Check what to do with the inform ation we have now.
            switch (status) {
                case 1:
                    log_debug("SKI780: Running direction CW");
                    break;
                case 2:
                    log_debug("SKI780: Runing direction CCW");
                    break;
                case 3:
                    log_debug("SKI780: Spindle not running");
                    break;
                default:
                    log_debug("SKI780: Spindle status unknown: " << status);
                    break; 
            }
            return true;
        };   
    }

    VFD::response_parser SKI780::get_status_ok(ModbusCommand& data) {
        data.tx_length = 6;
        data.rx_length = 6;

        data.msg[1] = 0x03;  // READ
        data.msg[2] = 0x80;  // Register address, high byte (current fault number)
        data.msg[3] = 0x00;  // Register address, low byte (current fault number)
        data.msg[4] = 0x00;  // Number of elements, high byte
        data.msg[5] = 0x01;  // Number of elements, low byte (1 element)

        /*
        Contents of register 0x08000
        Bit 0-15: current fault number, 0 = no fault, 1~18 = fault number
        */

        return [](const uint8_t* response, Spindles::VFD* vfd) -> bool {
            uint16_t currentFaultNumber = 0;

            if (response[1] != 0x03) {
                return false;
            }

            // We expect a result length of 2 bytes
            if (response[2] != 0 || response[3] != 2) {
                return false;
            }

            currentFaultNumber = (uint16_t(response[4]) << 8) | uint16_t(response[5]);
#ifdef DEBUG_VFD
            log_debug("SKI780 current error " << currentFaultNumber);
#endif
            if (currentFaultNumber != 0) {
                log_debug("VFD: Got fault number: " << currentFaultNumber);
                return false;
            }

            return true;
        };
    }

    // Configuration registration
    namespace {
        SpindleFactory::InstanceBuilder<SKI780> registration("SKI780");
    }
}
