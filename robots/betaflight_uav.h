#include <FlightController.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include "uav.h"

namespace BoBRobotics {
namespace Robots
{

namespace BetaFlight {

    std::string currentArmFlags;
    float currentVoltage = 0.0;
    float currentAmpDraw = 0.0;

	struct arm_flag {
		int err_num;
		std::string msg;
		void set_arm_flag(int e_num, std::string m, std::vector < arm_flag > &arm_flags) {
			this->err_num = e_num;
			this->msg = m;
			arm_flags.push_back(*this);
		}
	};

	std::vector < arm_flag > arm_flags;

	void setup_arm_flags() {

		arm_flag new_flag;

		arm_flags.clear();

		new_flag.set_arm_flag(1, std::string("Gyro not detected"), arm_flags);
		new_flag.set_arm_flag(2, std::string("Fail safe active"), arm_flags);
		new_flag.set_arm_flag(3, std::string("RX disconnected"), arm_flags);
		new_flag.set_arm_flag(4, std::string("RX connected, Arm switch on, turn off Arming"), arm_flags);
		new_flag.set_arm_flag(5, std::string("Failsafe switch active"), arm_flags);
		new_flag.set_arm_flag(6, std::string("Runaway takeoff prevention"), arm_flags);
		new_flag.set_arm_flag(7, std::string("Throttle too high"), arm_flags);
		new_flag.set_arm_flag(8, std::string("Craft not level"), arm_flags);
		new_flag.set_arm_flag(9, std::string("Arming too soon after power up"), arm_flags);
		new_flag.set_arm_flag(10, std::string("Prearm not active"), arm_flags);
		new_flag.set_arm_flag(11, std::string("Sytem load too high"), arm_flags);
		new_flag.set_arm_flag(12, std::string("Sensor calibration still ongoing"), arm_flags);
		new_flag.set_arm_flag(13, std::string("CLI active"), arm_flags);
		new_flag.set_arm_flag(14, std::string("Config menu active - over OSD other display"), arm_flags);
		new_flag.set_arm_flag(15, std::string("OSD menu active"), arm_flags);
		new_flag.set_arm_flag(16, std::string("A Black Sheep Telemetry device (TBS Core Pro for example) disarmed and is preventing arming"), arm_flags);
		new_flag.set_arm_flag(17, std::string("MSP connection is active, probably via Betaflight Configurator"), arm_flags);
		new_flag.set_arm_flag(18, std::string("Arm switch is in an unsafe position"), arm_flags);

	}

	struct MyIdent : public msp::Request {
		msp::ID id() const { return msp::ID::MSP_STATUS_EX; }

		msp::ByteVector raw_data;

		void decode(const msp::ByteVector &data) {
		    raw_data = data;
		}

	};
	struct Analog : public msp::Request {
		msp::ID id() const { return msp::ID::MSP_ANALOG; }

		float	vbat;           // Volt
		float	powerMeterSum;  // Ah
		size_t	rssi;  // Received Signal Strength Indication [0; 1023]
		float	amperage;       // Ampere

		void decode(const std::vector<uint8_t> &data) {
		    vbat          = data[0]/10.0f;
		    //powerMeterSum = data[1]/1000.0f;
		    //rssi          = data[3];
		    amperage      = data[5]/10.0f;
		}
	};

	struct Callbacks {
		void onIdent(const MyIdent &ident) {
		    //std::cout << "Arming Flag: ";
		    currentArmFlags.clear();
			int flag_data = *((int*) (&ident.raw_data[17]));
			for (auto af : arm_flags) {
				if (flag_data & (int) pow(2, af.err_num - 1)){
				currentArmFlags.append(af.msg);
				    currentArmFlags.append(std::string(" (Error number ") + std::to_string(af.err_num) + std::string("), "));
					//std::cout <<  af.msg << " (Error number " << af.err_num << "), ";
				}
			}
			//std::cout << std::endl;

		}
		void onAnalog(const Analog &anog){
			//std::cout << "Battery voltage level: " << anog.vbat << " V." << std::endl;
			currentVoltage = anog.vbat;
			//std::cout << "Power Meter Summery: " << anog.powerMeterSum << std::endl;
			//std::cout << "Received Signal Strength Indication: " << anog.rssi << std::endl;
			//std::cout << "Current level: " << anog.amperage << " mA." << std::endl;
			currentAmpDraw = anog.amperage;
		}
	};
}

class betaflight_uav : public UAV {

	public:

		betaflight_uav(std::string device, int baud) :
			m_Device(device),
			m_Baud(baud)
		{

			BetaFlight::setup_arm_flags();

			m_Fcu = new fcu::FlightController(m_Device, m_Baud);
			m_Fcu->initialise();

		}

		void subscribe() {
			m_Fcu->subscribe(&BetaFlight::Callbacks::onIdent, &m_Cbs, 0.1);
			m_Fcu->subscribe(&BetaFlight::Callbacks::onAnalog, &m_Cbs, 0.5);
		}

		~betaflight_uav() {
			if (m_Fcu) {
				delete m_Fcu;
			}
		}

		virtual void takeOff() override {}
        virtual void land() override {}
        virtual void setRoll(float right) override
        {
            right = std::min(1.0f, std::max(-1.0f, right));
            M_RC_values[0] = 1500 + right*M_ControlScale;
        }
        virtual void setPitch(float pitch) override
        {
            pitch = std::min(1.0f, std::max(-1.0f, pitch));
            M_RC_values[1] = 1500 + pitch*M_ControlScale;
        }
        virtual void setVerticalSpeed(float up) override
        {
            up = std::min(1.0f, std::max(-1.0f, up));
            M_RC_values[2] = 1500 + up*M_ThrottleScale;
        }
        virtual void setYawSpeed(float right) override
        {
            right = std::min(1.0f, std::max(-1.0f, right));
            M_RC_values[3] = 1500 + right*M_YawScale;
            std::cout << M_RC_values[3] << std::endl;
        }

        std::string getArmStateAsString() {
            return BetaFlight::currentArmFlags.c_str();
        }

        float getVoltage() {
            return BetaFlight::currentVoltage;
        }

        float getAmpDraw() {
            return BetaFlight::currentAmpDraw;
        }

        // accessors for M_ControlScale
        void setControlScale(float scale) {
            this->M_ControlScale = scale;
        }

        float getControlScale() {
            return this->M_ControlScale;
        }

        void sendCommands() {
            std::vector<uint16_t> auxs;
            m_Fcu->setRc(M_RC_values[0],M_RC_values[1],M_RC_values[3],M_RC_values[2],M_RC_values[4],M_RC_values[5],M_RC_values[6],M_RC_values[7], auxs);
        }

        void armDrone() {
            // hold throttle low to arm
            //M_RC_values[2] = 1100;
            M_RC_values[5] = 2000;
        }

        void disarmDrone() {
            M_RC_values[5] = 1000;
            // reset other values
            M_RC_values[0] = 1500;
            M_RC_values[1] = 1500;
            M_RC_values[2] = 1040;
            M_RC_values[3] = 1500;
        }

	private:
		std::string m_Device;
		int m_Baud;
		fcu::FlightController* m_Fcu = NULL;
		BetaFlight::Callbacks m_Cbs;
		uint16_t M_RC_values[8] = {1500,1500,1040,1500,2000,1000,1500,1500};
		float M_ControlScale = 180.0f;
    float M_ThrottleScale = 100.0f;
    float M_YawScale = 250.0f;



};


}
}
