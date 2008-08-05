import os

class Voice( ):
    def __init__(self):
        self.salutations = [ "hello", "top of the morning"]
        pass

    def say_text( self, mytex ):
        os.system( 'echo "' + mytex + '" | festival --tts' )

