import threading
import pygame
import subprocess
import os
import time

class QuadrupedAudio:
    def __init__(self, mac_address):
        self.mac_address = mac_address
        self.connected = False
        
        # Initialize mixer - use a failsafe here
        try:
            pygame.mixer.init()
        except Exception as e:
            print(f"Critial Mixer Error: {e}")

        # Attempt connection but don't let it crash the whole robot
        self.connected = self._connect()

    def _connect(self):
        try:
            # We add a timeout so the robot doesn't sit there forever 
            # trying to find a speaker that isn't on.
            result = subprocess.run(
                ["bluetoothctl", "connect", self.mac_address], 
                capture_output=True, text=True, timeout=5 
            )
            if "Connection successful" in result.stdout or "already connected" in result.stdout:
                return True
            return False
        except Exception:
            return False

    def play(self, file_path): 
        if not self.connected:
            print("Audio ignored: Speaker not connected.")
            return

        if os.path.exists(file_path):
            def _play_thread():
                try:
                    sound = pygame.mixer.Sound(file_path)
                    sound.play()
                    while pygame.mixer.get_busy():
                        time.sleep(0.1)
                except Exception as e:
                    print(f"Playback failed: {e}")
            
            # Fire and forget audio playback asynchronously
            threading.Thread(target=_play_thread, daemon=True).start()