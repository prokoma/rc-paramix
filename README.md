# RC Paramotor Mixer

Code for an external mixer module that takes two input channels (direction and brake) and outputs signal for two servos (left and right hand). Contains calibration mode to set stick centers and rates and offset servos against each other. This module is required for transmitters with limited or no ability to mix channels.

It's been a bit of a journey...

There are two functionally identical versions -- for ATTiny85 (Digispark) and for RP2040 (Waveshare RP2040-Zero).

The Digispark version didn't work well, because it doesn't have an external crystal and uses internal RC oscillator. That causes servo jitter, which could wear out the servos prematurely. I didn't do enough research beforehand and thought that the oscillator would be precise enough even without the crystal, but it isn't.

I've eliminated RP2040 before, because of its non-5V-tolerant pins. But by measurement I've discovered that both of my receivers (Futaba R617FS and FrSKY TFR6-A) output only 3.3V signal and the servos clearly don't mind that, so RP2040 was now a viable option.

I recently used a Waveshare RP2040-Zero board for a different project and really liked it for its small size, low price and versatility, so it was a natural option for this project.
