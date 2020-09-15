/*
 * A sample custom extension to the Sensapex Micro Manipulator control library's C++ API.
 * Demonstrates a way to use the interface library published under LGPL license
 * without a requirement to publish the proprietary extension (due it's inline code on
 * the application side)
 *
 * Copyright (c) 2012, Sensapex Oy
 * All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

#ifndef CUSTOMUMCTL_H
#define CUSTOMUMCTL_H

#include <umanipulatorctl.h>

#ifndef PORT
#ifdef WIN32
#include <windows.h>
#define PORT "com1"
#define sleep(t) Sleep((t)*1000)
#define msSleep(t) Sleep((t))
#else
#define PORT "/dev/ttyUSB0"
#include <unistd.h>
#define msSleep(t) usleep((t)*1000)
#endif
#endif

// Some of the command and register values are defined
// in common.h of the micromanipulator source codes.
// A permission is given to copy them into an extension
// class like this inheriting the original class protected
// by the LGPL lisence

#define INCREMENT_REG         0x05
#define STEP_LENGHT_REG       0x1F

#define	NORMAL_DRIVE_MODE_CMD 18
#define	SNAIL_DRIVE_MODE_CMD  19

#define DEF_STEP_LENGTH_REG_VALUE 10

// Inherit and extend the base class.

class CustomUManipulatorCtl : public UManipulatorCtl
{
public:
	CustomUManipulatorCtl()
	{
		statusPollErrorCountLimit = 5;
		statusPollPeriod  = 100;
		commandDelay      = 300;
		originalIncrement = 2;  // Speed 1 in Control Unit UI
		originalStepSize  = 10; // 100nm in device settings UI
		inSnailMode = false;
	};

	// This extensions uses mode selection commands and write step length parameter which
	// may yield to long latencies in the response, thus a long timeout has to be used
	bool open(const char *port) { return UManipulatorCtl::open(port, 600); }

	// Attempt to restore the original settings on close by
	// overdriving these two methods. Read values at the beginning
	virtual bool select(const int dev)
	{
		if(!UManipulatorCtl::select(dev))
			return false;
		msSleep(commandDelay);
		originalIncrement = read(INCREMENT_REG);
		msSleep(commandDelay);
		originalStepSize = read(STEP_LENGHT_REG);
		return true;
	}

	// and write back on close (but filter out erronous values)
	virtual void close()
	{
		if(originalIncrement > 0)
			write(INCREMENT_REG, originalIncrement);
		if(originalStepSize < 6 || originalStepSize > 16)
			originalStepSize = DEF_STEP_LENGTH_REG_VALUE;
		write(STEP_LENGHT_REG, originalStepSize);
		msSleep(commandDelay);
		if(inSnailMode)
			cmd(NORMAL_DRIVE_MODE_CMD);
		UManipulatorCtl::close();
	}

	// Poll manipulator status to detect when previous drive is ready.
	bool waitBusy()
	{
		int errorCount = 0;
		umanipulatorctl_status_t statusValue;
		do
		{
			msSleep(statusPollPeriod);
			statusValue = status();
			if(errorStatus(statusValue))
			{
				errorCount++;
				if(errorCount >= statusPollErrorCountLimit)
					return false;
			}
			else
				errorCount = 0;
		} while(busyStatus(statusValue) || errorStatus(statusValue));
		return true;
	}

	// Tune device settigs so that the memory pos drive is using requested speed
	bool setMemoryDriveSpeed(int value)
	{
		if(value < 1)
			return false;

		// With zero argument value write step length to default
		int targetStep = DEF_STEP_LENGTH_REG_VALUE;
		if(value < 1)
		{
			if(!write(INCREMENT_REG, 2))
				return false;
			// Mode change causes write to HW and thus needs some quard time
			if(!cmd(NORMAL_DRIVE_MODE_CMD))
				return false;
			msSleep(commandDelay);
		}
		else if(value >= 1000)
		{
			// For higher speeds is reasonable to use normal mode
			// and speed 1 instead of snail mode.
			// Increment register value affects also memory pos drive speed.
			if(originalIncrement != 2 && !write(INCREMENT_REG, 2))
				return false;
			// In theory pulse freq is about 30KHz, 1/2 scaling at speed 1
			// causes 15K steps. Register value 1 (10nm) is minimum and
			// should provide physical movement of 150 um/s (or nm/ms)
			targetStep = value/150;
			// Functionality at longer target step sizes not tested.
			// This is same as 200nm at higher speeds, which should still work
			if(targetStep > 40)
				targetStep = 40;
			// Causes writes to HW DACs and thus needs some quard time prior next message
			if(!cmd(NORMAL_DRIVE_MODE_CMD))
				return false;
			msSleep(commandDelay);
		}
		else // Slow speed 0 < value < 1000 requested, activate snail mode
		{
			if(originalIncrement != 1 && !write(INCREMENT_REG, 1))
				return false;
			// In theory 30K/s pulses and due 1/8 scaling due snail mode,
			// effectively 3750 target size steps/s = 37500 nm/s i.e.
			// ~38 um/s for register value 1
			targetStep = value/38;
			if(targetStep < 1)
				targetStep = 1;
			if(!cmd(SNAIL_DRIVE_MODE_CMD))
				return false;
			msSleep(commandDelay);
		}
		// This parameter is stored into flash and thus write is relatively slow operation
		if(!write(STEP_LENGHT_REG, targetStep))
			return false;
		msSleep(commandDelay);
		return true;
	}

	// Getting occasional timeouts is quite normal when polling the status with
	// high speed, tolerate some amount or errors before giving up
	bool setStatusPollErrorCountLimit(int value)
	{
		if(value < 1)
			return false;
		statusPollErrorCountLimit = value;
		return true;
	}

	// Note that very high polling freq affects the movement
	// and may cause malfunction in the manipulator's microcontroller
	bool setStatusPollPeriod(int value)
	{
		if(value < 2 || value > 1000)
			return false;
		statusPollPeriod = value;
		return true;
	}

	bool setCommandDelay(int value)
	{
		if(value < 100 || value > 1000)
			return false;
		commandDelay = value;
		return true;
	}

private:
	int statusPollErrorCountLimit, statusPollPeriod;
	int originalIncrement, originalStepSize, commandDelay;
	bool inSnailMode;
};

#endif // CUSTOMUMCTL_H
