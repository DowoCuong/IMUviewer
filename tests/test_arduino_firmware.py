"""Execute the actual Arduino sketches with deterministic GPIO/clock/serial mocks.

These checks exercise parsing and scheduling, not motor mechanics or AVR IRQ latency.
The normal AVR build separately checks board headers, linking and memory limits.
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
ARDUINO = r'''
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <string>
#include <sstream>
#include <vector>
#include <cstdio>
#define F(x) x
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define RISING 3
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)
#define max(a,b) ((a)>(b)?(a):(b))
#define _BV(x) (1U<<(x))
#define PG5 5
#define PE3 3
using std::isfinite;
static uint32_t fakeUs;
static uint8_t SREG=128, PING=0, PINE=0;
static bool microsSawMasked=false;
struct Pulse { uint8_t pin; uint32_t time; };
static std::vector<Pulse> pulses;
unsigned long micros() { if (!(SREG&128)) microsSawMasked=true; return fakeUs; }
unsigned long millis() { return fakeUs/1000; }
void delayMicroseconds(unsigned int us) { fakeUs+=us; }
void noInterrupts() { SREG &= 127; }
void interrupts() { SREG |= 128; }
void pinMode(uint8_t, int) {}
void attachInterrupt(int, void(*)(), int) {}
int digitalPinToInterrupt(int pin) { return pin; }
void digitalWrite(uint8_t pin, uint8_t value) {
    if ((pin==2 || pin==3) && value==HIGH) pulses.push_back({pin,fakeUs});
}
struct MockSerial {
    std::string input, output;
    size_t readIndex=0;
    void begin(int) {}
    int available() { return int(input.size()-readIndex); }
    int read() { return (uint8_t)input[readIndex++]; }
    template<class T> void print(T value) { std::ostringstream s; s<<value; output+=s.str(); }
    void print(float value, int) { print(value); }
    template<class T> void println(T value) { print(value); output+='\n'; }
} Serial;
#define CHECK(x) do { if(!(x)) { std::fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
'''
STEPPER = r'''
#pragma once
class AccelStepper {
    long position=0, target=0;
public:
    enum { DRIVER=1 };
    AccelStepper(int, int, int) {}
    void setPinsInverted(bool, bool, bool) {}
    void setMinPulseWidth(unsigned) {}
    void setMaxSpeed(float) {}
    void setAcceleration(float) {}
    long currentPosition() const { return position; }
    void setCurrentPosition(long p) { position=target=p; }
    void moveTo(long p) { target=p; }
    void stop() { target=position; }
    bool run() { if(position<target) ++position; else if(position>target) --position; return position!=target; }
};
'''
UNO = r'''
int main(int argc, char **argv) {
    CHECK(sizeof(unsigned long)==4);  // AVR clock arithmetic also uses 32-bit long.
    CHECK(argc==2);
    std::string test=argv[1];
    setup(); Serial.output.clear();
    if(test=="overflow") {
        Serial.input=std::string(48,'x')+"M 90 0\n";
        while(Serial.available()) loop();
        CHECK(motionState==IDLE);
        Serial.input+="M 1 0\n";
        while(Serial.available()) loop();
        CHECK(motionState==MOVE_ONCE);
    } else if(test=="nul") {
        Serial.input=std::string("M 90 0\0garbage\n",15);
        while(Serial.available()) loop();
        CHECK(motionState==IDLE);
    } else if(test=="silence") {
        Serial.input="M 1 0"; loop();
        CHECK(motionState==IDLE);
        fakeUs+=51000; loop(); CHECK(motionState==MOVE_ONCE);
    } else if(test=="bad_angles") {
        for(const char *s:{"M nan 0", "C inf 1", "M 36001 0", "M 1 2 extra", "C 0 0"}) {
            char line[48]; std::strcpy(line,s); handleLine(line); CHECK(motionState==IDLE);
        }
    } else if(test=="pulse_limit") {
        startCycle(90,90);
        fakeUs=cycleStartUs+cyclePeriodUs/4;
        for(int i=0;i<30;++i) serviceCycle(); // An I/O stall creates a target backlog.
        for(int i=0;i<100;++i) { fakeUs+=2000; serviceCycle(); }
        uint32_t previous[2]={0,0};
        CHECK(pulses.size()>=100); // Must resume stepping, not suppress all pulses.
        for(const auto &p:pulses) {
            unsigned axis=p.pin-2;
            if(previous[axis]) CHECK(uint32_t(p.time-previous[axis])>=2000);
            previous[axis]=p.time;
        }
    } else if(test=="stop_latched") {
        startCycle(90,90);
        fakeUs=cycleStartUs+cyclePeriodUs/4;
        stopMotion();
        fakeUs=cycleStartUs+cyclePeriodUs+cyclePeriodUs/4;
        // A late service pass must latch the origin even if S arrives again.
        stopMotion();
        cyclePosition[0]=cycleOrigin[0]; cyclePosition[1]=cycleOrigin[1];
        serviceCycle();
        CHECK(motionState==IDLE);
    } else if(test=="stop_return") {
        startCycle(5,-5);
        for(int i=0;i<200;++i) { fakeUs+=2000; serviceCycle(); }
        stopMotion();
        // Skip the exact boundary: origin must remain the target until reached.
        fakeUs=cycleStartUs+cyclePeriodUs+cyclePeriodUs/4;
        for(int i=0;i<300 && motionState!=IDLE;++i) { serviceCycle(); fakeUs+=2000; }
        CHECK(motionState==IDLE);
        CHECK(cyclePosition[0]==cycleOrigin[0] && cyclePosition[1]==cycleOrigin[1]);
    } else if(test=="clock_wrap") {
        fakeUs=0xffff0000UL;
        startCycle(90,90);
        const uint32_t period=cyclePeriodUs;
        // Cross micros() rollover and then run beyond a full 2^32 us lifetime.
        uint64_t elapsed=0;
        uint32_t previous=fakeUs;
        for(int i=0;i<4400;++i) {
            fakeUs+=1000000;
            uint32_t before=fakeUs;
            elapsed+=uint32_t(before-previous); previous=before;
            serviceCycle();
            CHECK(uint32_t(before-cycleStartUs)==elapsed%period);
        }
    } else if(test=="serial_budget") {
        startCycle(90,90); fakeUs=cycleStartUs+cyclePeriodUs/4;
        Serial.input=std::string(400,'x'); loop();
        CHECK(Serial.available()>0); CHECK(!pulses.empty());
    } else return 2;
    return 0;
}
'''
MEGA = r'''
int main(int argc, char **argv) {
    CHECK(sizeof(unsigned long)==4); CHECK(argc==2);
    std::string test=argv[1];
    setup();
    CHECK(Serial.output.find("ENCODER,2,2500,100\n")!=std::string::npos);
    Serial.output.clear();
    if(test=="snapshot") {
        encoder1Count=-5001; encoder2Count=123456; fakeUs=0xfffffff0UL;
        SREG=0x80; printData(); CHECK(SREG==0x80 && microsSawMasked);
        CHECK(Serial.output=="ENC,0,4294967280,-5001,123456\n");
        Serial.output.clear(); fakeUs+=10000; printData();
        CHECK(Serial.output=="ENC,1,9984,-5001,123456\n");
        SREG=0; long a,b; unsigned long t; copyCounts(a,b,t); CHECK(SREG==0);
    } else if(test=="direction") {
        PING=0; PINE=0; onEncoder1A(); onEncoder2A();
        CHECK(encoder1Count==1 && encoder2Count==1);
        PING=_BV(PG5); PINE=_BV(PE3); onEncoder1A(); onEncoder2A();
        CHECK(encoder1Count==0 && encoder2Count==0);
        CHECK(std::fabs(angle0To360(-1)-359.856f)<0.001f);
    } else if(test=="zero") {
        encoder1Count=10; encoder2Count=20;
        Serial.input="Z1\n"; readCommands();
        CHECK(encoder1Count==0 && encoder2Count==20);
        CHECK(Serial.output=="ZERO,1\n");
    } else if(test=="overflow") {
        encoder1Count=10; encoder2Count=20;
        Serial.input=std::string(8,'x')+"Z\n"; readCommands();
        CHECK(encoder1Count==10 && encoder2Count==20);
    } else if(test=="nul") {
        encoder1Count=10; encoder2Count=20;
        Serial.input=std::string("Z\0bad\n",6); readCommands();
        CHECK(encoder1Count==10 && encoder2Count==20);
    } else if(test=="serial_budget") {
        fakeUs=10000; Serial.input=std::string(400,'x'); loop();
        CHECK(Serial.available()>0);
        CHECK(Serial.output.find("ENC,0,10000,")!=std::string::npos);
    } else return 2;
    return 0;
}
'''


def compile_sketch(directory, name, body):
    directory.mkdir()
    (directory/'Arduino.h').write_text(ARDUINO)
    (directory/'AccelStepper.h').write_text(STEPPER)
    source = ROOT/'firmware/arduino_motion'/name/(name+'.ino')
    (directory/'unit.cpp').write_text('#include "'+source.as_posix()+'"\n'+body)
    exe = directory/'unit.exe'
    env = os.environ.copy()
    # MSVC uses the same 32-bit long width as AVR; host GCC on Linux does not.
    vcvars = next(Path('C:/Program Files/Microsoft Visual Studio').glob(
        '*/Community/VC/Auxiliary/Build/vcvars64.bat'), None)
    if os.name == 'nt' and vcvars:
        script = directory/'compile.cmd'
        script.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\n'
            'cl /nologo /EHsc /std:c++17 /I"'+str(directory)+'" unit.cpp /Fe"'+str(exe)+'"\n')
        windows = env.get('SystemRoot', 'C:/Windows')
        env['PATH'] = windows+'/System32;'+windows
        command = ['cmd', '/c', str(script)]
    elif shutil.which('g++') and os.name == 'nt':
        command = ['g++', '-std=c++17', '-I'+str(directory), 'unit.cpp', '-o', str(exe)]
    else:
        raise unittest.SkipTest('Requires a native C++ compiler with 32-bit long')
    result = subprocess.run(command, cwd=directory, env=env, capture_output=True,
                            text=True, timeout=60)
    if result.returncode:
        raise AssertionError(result.stdout+result.stderr)
    return exe


class ArduinoFirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        root = Path(cls.temporary.name)
        cls.uno = compile_sketch(root/'uno', 'uno_dual_tb6600', UNO)
        cls.mega = compile_sketch(root/'mega', 'mega_dual_encoder', MEGA)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def check_case(self, executable, name):
        result = subprocess.run([str(executable), name], capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


def add_case(board, name):
    def test(self):
        self.check_case(getattr(self, board), name)
    setattr(ArduinoFirmwareTests, 'test_'+board+'_'+name, test)


for case in ('overflow', 'nul', 'silence', 'bad_angles', 'pulse_limit', 'stop_latched',
             'stop_return', 'clock_wrap', 'serial_budget'):
    add_case('uno', case)
for case in ('snapshot', 'direction', 'zero', 'overflow', 'nul', 'serial_budget'):
    add_case('mega', case)


if __name__ == '__main__':
    unittest.main()
