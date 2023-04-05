#ifndef __TIC_DRIVER__
#define __TIC_DRIVER__

#include "Serial.hpp"
#include <ros/ros.h>

namespace ss
{
	class TicDriver
	{
		static constexpr double reset_timeout = 120.0;
		
	public:
		TicDriver();
		~TicDriver();

		int init(Serial *port, int initial_position);
		int setPosition(int32_t position);
		int getPosition(int32_t *position);

	private:
		bool isInitialized;

		Serial *_port;

		enum class TicCommands : uint8_t
		{
			SetTargetPosition = 0xE0,
			SetTargetVelocity = 0xE3,
			HaltAndSetPosition = 0xEC,
			HaltAndHold = 0x89,
			GoHome = 0x97,
			ResetCommandTimeout = 0x8C,
			DeEnergize = 0x86,
			Energize = 0x85,
			ExitSafeStart = 0x83,
			EnterSafeStart = 0x8F,
			Reset = 0xB0,
			ClearDriverError = 0x8A,
			SetMaxSpeed = 0xE6,
			SetStartingSpeed = 0xE5,
			SetMaxAcceleration = 0xEA,
			SetMaxDeceleration = 0xE9,
			SetStepMode = 0x94,
			SetCurrentLimit = 0x91,
			SetDecayMode = 0x92,
			SetAGCOption = 0x98,
			GetVariable = 0xA1,
			GetVariableAndClearErrors = 0xA2,
			GetSettings = 0xA8,
			SetSetting = 0x13,
			Reinitialize = 0x10,
			StartBootloader = 0xFF
		};

		int setPort(Serial *port);
		int getVariable(uint8_t offset, uint8_t *buf, uint32_t length);
	};
};

#endif

