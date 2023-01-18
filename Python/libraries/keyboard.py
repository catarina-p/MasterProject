from pynput import keyboard

########
### Key commands
########

class Key():
    def __init__(self):                
        self.keyPressLatching = None
        self._keyReleaseLatching = None
        self.keyPress = None
        self._start_keyboard_listener()

    def _on_press(self, key):
        try:
            self.keyPressLatching = key.char
            self.keyPress = key.char
            
        except AttributeError:
            self.keyPressLatching = key
            self.keyPress = key
            

    def _on_release(self, key):
        try:
            self._keyReleaseLatching = key.char

            if self._keyReleaseLatching == self.keyPress: 
                self.keyPress = None

        except AttributeError:
            self._keyReleaseLatching = key

            if self._keyReleaseLatching == self.keyPress: 
                self.keyPress = None


    def _start_keyboard_listener(self):  
        listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        listener.start()
        print("keyboard listener started")