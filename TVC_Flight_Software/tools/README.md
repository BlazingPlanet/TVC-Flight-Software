# Flight log decoder

## Capture a dump
1. PuTTY -> Session -> Logging -> All session output
2. Log file: C:\dev\capture.log, "Always overwrite it"
3. Session -> Serial, 921600 -> Open
4. Reset the board, press 'd' within 3 s
5. Wait for ---FLASH DUMP END---, then close PuTTY

## Decode
    cd C:\dev\Circuit-Testing\06_stmport\tools
    python decode_flight_log.py C:\dev\capture.log

## Example Decode for Flight 3 on the Chip:
    python decode_flight_log.py C:\dev\capture.log --save-replay flight1.gif

## Options - Can use any combination of these after the decode_flight_log.py call
    --replay            3D orientation animation
    --save-replay F     write the animation to a file (implies --replay)
    --csv flight.csv    also write CSV
    --flight 2          decode an earlier flight (default: most recent)
    --no-plots          summary only
    --decimate 5        replay frame skip (default 10 = ~20 fps)
    --no-trim           plot the whole log, not just the flight window
    --pre-boost 10      seconds of pad time to keep before BOOST (default 10)
    --post-descent 10   seconds to keep after DESCENT (default 10)

## Plot trimming
Logging starts at PAD, not at launch, so most of a log is pad time. A five
minute hold followed by fifteen seconds of flight squeezes the entire powered
portion into a few pixels.

By default the plots and replay are trimmed to **10 s before BOOST through
10 s after DESCENT**. The printed summary and any CSV always cover the whole
flight -- only the plots are cut, since flag percentages over a trimmed window
would be misleading.

    --no-trim                       see everything
    --pre-boost 30                  more pad lead-in
    --post-descent 60               follow the descent further

If the log contains no BOOST state, nothing is trimmed. A vehicle that never
launched has its explanation in the part that would have been cut.

The plots are also interactive: the magnifying glass in the toolbar zooms to a
dragged box, and the home button resets. Good for hunting a specific moment
without re-running with different trim values.

## Saving the replay
    python decode_flight_log.py C:\dev\capture.log --save-replay flight1.gif
    python decode_flight_log.py C:\dev\capture.log --save-replay flight1.mp4

**.gif** uses pillow, which ships with matplotlib -- always works.
**.mp4** needs ffmpeg installed and on PATH. Smaller files, better quality.

Saving is slow: every frame is a full 3D re-render. Trimming keeps this
manageable, so avoid combining `--save-replay` with `--no-trim` on a log with a
long pad hold. Raise `--decimate` for a faster, choppier render.

## Checks worth making every flight
- Tick dt should read ~5.00 ms mean
- PULSE_Y / PULSE_Z flags should never appear -- if they do,
  a servo calibration constant is wrong
- OVERRUN flags indicate the loop missed its deadline
- NO_TRUST should be set for essentially the whole burn. That is correct
  behavior, not a fault -- see the boost-phase drift section in TUNING.md
- EJECT_BAK set means apogee detection failed and the backup timer fired.
  Investigate before flying again
- Compare cmd_y/cmd_z against pulse_y/pulse_z. Sustained divergence through
  boost means the slew limiter is binding, which means the gains are asking
  for more than the servos can deliver
