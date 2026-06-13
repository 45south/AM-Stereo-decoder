# AM Stereo Decoder

A Windows application that decodes AM stereo broadcasts received through SDRuno's IQ OUT facility.

## Features

- **C-QUAM** decoding with PLL carrier recovery and stereo matrix
- **Kahn-Hazeltine ISB** decoding via sideband separation
- Win32 GUI with real-time level meters and STEREO indicator
- Variable bandwidth filter: DX (3 kHz) / Narrow (5 kHz) / Normal (9 kHz) / Wide (15 kHz) / Custom
- Gain slider (0.1–10.0)
- Fixed whistle filter at 9 kHz and 10 kHz
- Variable Het Notch filter (0.5–8.5 kHz, 100 Hz steps)
- Mono switch, Always on Top, Anti-fade (Kahn mode)
- Click suppression on retune
- Settings saved to `amstereo.ini`

## Requirements

- Windows 10/11
- SDRuno with SDRplay RSP hardware
- VB-Audio Virtual Cable (or equivalent VAC)
- MSYS2/MinGW-w64 (to build from source)

## Build

```
gcc -Wall -Wextra -O2 -std=c11 -D_USE_MATH_DEFINES -o amstereo.exe amstereo.c -lwinmm -lcomctl32 -lgdi32 -lm -mwindows
```

Or with the supplied Makefile:

```
make
```

## SDRuno Setup

1. Set VRX mode to **IQ OUT**
2. Set VRX output device (Sett → OUT) to **CABLE Input (VB-Audio Virtual Cable)**
3. Tune VRX to the AM carrier frequency
4. Press Play

In AM Stereo Decoder, set **In** to **CABLE Output (VB-Audio Virtual Cable)**, set **Out** to your headphones or speakers, then press **Start**.

## Compatibility

Works with any SDR software that can output an IQ stream to a Windows audio device, including SDR#, HDSDR, and SDR Console.

## Version History

### v1.0
- Initial release

## Author

Dave Headland ([@45south](https://github.com/45south))  
Developed with Claude (Anthropic)

## Licence

MIT
