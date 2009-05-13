byte ALL = -1 # Only legal with PLAY_STOP
byte BACKINGUP = 1
byte NEEDS_UNPLUGGING = 2
byte NEEDS_PLUGGING = 3

byte sound # Selects which sound to play (see above)

byte PLAY_STOP = 0 # Stop this sound from playing
byte PLAY_ONCE = 1 # Play the sound once
byte PLAY_START = 2 # Play the sound in a loop until a stop request occurs

byte command # Indicates what to do with the sound
