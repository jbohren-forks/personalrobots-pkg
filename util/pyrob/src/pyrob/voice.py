import os

def say( mytex ):
    os.system( 'echo "' + mytex + '" | festival --tts' )

