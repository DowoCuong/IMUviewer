"""Compile the bundled Mega/Uno sketches with an installed Arduino AVR core.

No upload or serial port access. Produces HEX plus toolchain/source checksums.
Uses the core's normal AVR compiler, archive, link and objcopy options.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    packages = Path(os.environ.get('LOCALAPPDATA', ''))/'Arduino15/packages/arduino'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core', type=Path, default=packages/'hardware/avr/1.8.8')
    parser.add_argument('--compiler', type=Path, default=packages/'tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin')
    parser.add_argument('--accelstepper', type=Path,
        default=Path.home()/'OneDrive/Documents/Arduino/libraries/AccelStepper/src')
    args = parser.parse_args()
    core = args.core/'cores/arduino'
    if not (core/'Arduino.h').exists() or not (args.compiler/'avr-g++.exe').exists():
        parser.error('Install Arduino AVR Boards or pass --core and --compiler')
    report = {'core': str(args.core), 'compiler': str(args.compiler), 'uploaded': False, 'boards': []}
    def run(command):
        result = subprocess.run([str(x) for x in command], capture_output=True, text=True,
                                encoding='utf-8', errors='replace', timeout=90)
        if result.returncode:
            raise RuntimeError(' '.join(map(str, command))+'\n'+result.stdout+result.stderr)
        return result.stdout
    for name, mcu, variant, board, flash_limit in (
            ('mega_dual_encoder', 'atmega2560', 'mega', 'AVR_MEGA2560', 253952),
            ('uno_dual_tb6600', 'atmega328p', 'standard', 'AVR_UNO', 32256)):
        source = ROOT/'firmware/arduino_motion'/name/(name+'.ino')
        build = ROOT/'build/arduino-bench'/name
        build.mkdir(parents=True, exist_ok=True)
        defines = ['-mmcu='+mcu, '-DF_CPU=16000000L', '-DARDUINO=10819', '-DARDUINO_'+board, '-DARDUINO_ARCH_AVR']
        includes = ['-I'+str(core), '-I'+str(args.core/'variants'/variant)]
        if name.startswith('uno'):
            if not (args.accelstepper/'AccelStepper.h').exists():
                parser.error('Install AccelStepper or pass --accelstepper')
            includes.append('-I'+str(args.accelstepper))
        cpp_flags = ['-std=gnu++11', '-fpermissive', '-fno-exceptions', '-fno-threadsafe-statics']
        common = ['-c', '-Os', '-flto', '-ffunction-sections', '-fdata-sections']+defines+includes
        core_objects = []
        for path in sorted(core.iterdir()):
            if path.suffix not in ('.cpp', '.c', '.S'):
                continue
            obj = build/(path.name+'.o')
            compiler = 'avr-g++.exe' if path.suffix == '.cpp' else 'avr-gcc.exe'
            flags = cpp_flags if path.suffix == '.cpp' else (['-std=gnu11'] if path.suffix == '.c' else ['-x', 'assembler-with-cpp'])
            run([args.compiler/compiler]+common+flags+[path, '-o', obj])
            core_objects.append(obj)
        archive = build/'core.a'
        archive.unlink(missing_ok=True)
        run([args.compiler/'avr-gcc-ar.exe', 'rcs', archive]+core_objects)
        sketch_obj = build/'sketch.o'
        run([args.compiler/'avr-g++.exe']+common+cpp_flags+['-x', 'c++', source, '-o', sketch_obj])
        objects = [sketch_obj]
        if name.startswith('uno'):
            obj = build/'AccelStepper.o'
            run([args.compiler/'avr-g++.exe']+common+cpp_flags+[args.accelstepper/'AccelStepper.cpp', '-o', obj])
            objects.append(obj)
        elf = build/(name+'.elf')
        run([args.compiler/'avr-gcc.exe', '-Os', '-flto', '-fuse-linker-plugin', '-mmcu='+mcu,
             '-Wl,--gc-sections', '-o', elf]+objects+[archive, '-lm'])
        size = run([args.compiler/'avr-size.exe', elf]).strip()
        size_fields = size.splitlines()[-1].split()
        flash_bytes = int(size_fields[0])+int(size_fields[1])
        ram_bytes = int(size_fields[1])+int(size_fields[2])
        if flash_bytes > flash_limit or ram_bytes > (8192 if mcu=='atmega2560' else 2048):
            raise RuntimeError('Sketch exceeds board memory: '+size)
        output = source.with_suffix('.hex')
        run([args.compiler/'avr-objcopy.exe', '-O', 'ihex', '-R', '.eeprom', elf, output])
        report['boards'].append({'board': board, 'source': str(source), 'hex': str(output),
            'flash_bytes': flash_bytes, 'ram_bytes': ram_bytes,
            'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
            'hex_sha256': hashlib.sha256(output.read_bytes()).hexdigest()})
        print(name, 'PASS', flash_bytes, 'flash bytes;', ram_bytes, 'RAM bytes', flush=True)
    destination = ROOT/'firmware/arduino_motion/build-manifest.json'
    destination.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
