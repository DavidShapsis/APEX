import threading
import subprocess
import os
import time

# Inside the Docker container there is no host audio socket or D-Bus. The
# container shares the host network, so audio goes to the host's PipeWire over
# its local pulse TCP port instead (enabled on the host in
# ~/.config/pipewire/pipewire-pulse.conf.d/tcp.conf -- see HARDWARE.md). Set
# before pygame loads SDL; an explicit value in the environment still wins.
os.environ.setdefault("PULSE_SERVER", "tcp:127.0.0.1:4713")
os.environ.setdefault("SDL_AUDIODRIVER", "pulseaudio")

import pygame


class QuadrupedAudio:
    def __init__(self, mac_address):
        """Raises if the mixer will not start or the speaker is not the host's
        default output, so boot reports Audio FAIL instead of a false OK.

        The Bluetooth link itself is the host's job: the speaker is paired and
        trusted there, so bluez reconnects it on its own. bluetoothctl does not
        exist in the container, and could not reach the host's bluez anyway.
        """
        self.mac_address = mac_address
        pygame.mixer.init()
        sink = self._default_sink()
        # PipeWire names a Bluetooth sink bluez_output.<MAC with _>.<n>.
        if mac_address.replace(":", "_").upper() not in sink.upper():
            pygame.mixer.quit()
            raise RuntimeError(
                f"speaker {mac_address} is not the default output "
                f"(default is {sink or 'none'})")
        self.connected = True

    @staticmethod
    def _default_sink():
        try:
            return subprocess.run(["pactl", "get-default-sink"],
                                  capture_output=True, text=True,
                                  timeout=3).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return ""

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
