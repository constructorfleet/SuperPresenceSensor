#include "Arduino.h"
#include "MR24HPB1.h"

namespace MR24HPB1 {
    uint16_t getCRC16(uint8_t *Frame, uint8_t Len) {
        unsigned char luc_CRCHi = 0xFF;
        unsigned char luc_CRCLo = 0xFF;
        int li_Index = 0;
        while (Len--)
        {
            li_Index = luc_CRCLo ^ *(Frame++);
            luc_CRCLo = (unsigned char)(luc_CRCHi ^ cuc_CRCHi[li_Index]);
            luc_CRCHi = cuc_CRCLo[li_Index];
        }
        return (uint16_t)(luc_CRCLo << 8 | luc_CRCHi);
    }

    Radar::Radar(HardwareSerial &serialPort) : serial(serialPort) {
        serial = serialPort;
    }

    void Radar::setup(uint8_t *presencePin, uint8_t *motionPin) {
        if (presencePin != nullptr) {
            pinMode(*presencePin, INPUT_PULLUP);
        }
        if (motionPin != nullptr) {
            pinMode(*motionPin, INPUT_PULLUP);
        }
        serial.begin(8600);
    }

    void Radar::loop() {
        while (serial.available() > 0) {
            DataFrameBuilder* dataFrame = parseFrame();
            if (dataFrame != nullptr) {
                process(dataFrame->dataFrame);
                return;
            }
        }
    }

    void Radar::readSettingss() {
        send(Command::READ, 0x04, 0x0C);
        send(Command::READ, 0x04, 0x10);
    }

    DataFrameBuilder* Radar::parseFrame() {
        if (serial.read() != HEADER) {
            return nullptr;
        }
        uint8_t dataLength = serial.read() + (serial.read() << 8);
        DataFrameBuilder dataFrame;
        for (int i = 0; i < dataLength - 2; i++)
        {
          dataFrame.bytes[i] = serial.read();
        }

        return &dataFrame;
    }

    void Radar::registerOccupancyCallback(OccupancyCallback callback) {
        _occupancyCallback = callback;
    }

    void Radar::registerMotionCallback(MotionCallback callback) {
        _motionCallback = callback;
    }

    void Radar::registerDirectionCallback(DirectionCallback callback) {
        _directionCallback = callback;
    }

    void Radar::send(uint8_t functionCode, uint8_t address1, uint8_t address2, uint8_t* data, uint8_t length) {
        uint8_t frameLength = length + 8;
        uint8_t frame[frameLength];
        frame[0] = HEADER;
        frame[1] = lowByte(frameLength - 1);
        frame[2] = highByte(frameLength - 1);
        frame[3] = functionCode;
        frame[4] = address1;
        frame[5] = address2;
        for(int i = 0; i < length; i++) {
            frame[6+i] = data[i];
        }
        uint16_t crc = getCRC16(frame, 6 + length);
        frame[length + 6] = highByte(crc);
        frame[length + 7] = lowByte(crc);
        serial.write(frame, frameLength);
    }

    void Radar::process(DataFrame dataFrame) {
        uint16_t commandAddress = (dataFrame.address1 << 8) | dataFrame.address2;
        switch (dataFrame.commandFunction) {
            case Command::READ:
                break;
            case Command::WRITE:
                break;
            case Command::PASSIVE_REPORT:
                switch (commandAddress) {
                    case 0x040C: // Threshold
                        setThreshold(dataFrame.data[0]);
                        break;
                    case 0x0410: // Scene
                        setScene(static_cast<Scene::Name>(dataFrame.data[0]));
                        break;
                    case 0x0412: // Forced unoccupied
                        break;
                }
                break;
            case Command::ACTIVE_REPORT:
                switch (commandAddress) {
                    case 0x0305: // Environment Status
                        setOccupancy(static_cast<Occupancy::State>(dataFrame.data[0]));
                        setMotion(static_cast<Motion::State>(dataFrame.data[1]));
                        break;
                    case 0x0306: // Body Parameters
                        break;
                    case 0x0307: // Direction
                        setDirection(static_cast<Direction::State>(dataFrame.data[2]));
                        break;
                }
                break;
        }
    }

    void Radar::setScene(Scene::Name scene) {
        _activeScene = scene;
    }

    void Radar::setThreshold(uint8_t threshold) {
        _threshold = threshold;
    }

    void Radar::setOccupancy(Occupancy::State occupancy) {
        if (_occupancyCallback != NULL && occupancy != _occupancyState) {
            _occupancyCallback(occupancy);
        }
        _occupancyState = occupancy;
    }

    void Radar::setMotion(Motion::State motion) {
        if (_motionCallback != NULL && motion != _motionState) {
            _motionCallback(motion);
        }
        _motionState = motion;
    }

    void Radar::setDirection(Direction::State direction) {
        if (_directionCallback != NULL && direction != _directionState) {
            _directionCallback(direction);
        }
        _directionState = direction;
    }

    void Radar::configureScene(Scene::Name scene) {
        uint8_t data = static_cast<uint8_t>(scene);
        send(Command::WRITE, 0x04, 0x10, &data, 1);
    }

    void Radar::configureThreshold(uint8_t threshold) {
        send(Command::WRITE, 0x04, 0x0C, &threshold, 1);
    }
};