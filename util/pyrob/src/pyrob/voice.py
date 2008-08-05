import os

def say( self, mytex ):
    os.system( 'echo "' + mytex + '" | festival --tts' )

