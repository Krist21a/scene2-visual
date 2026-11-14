"""Continuous capture of the official simulator window, before policy launch."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time


class Recording:
    def __init__(self, destination):
        self.destination = Path(destination)
        self.process = None
        self.stream = None
        self.receipt = {}

    def start(self):
        env = os.environ.copy()
        env.setdefault('DISPLAY', ':0')
        if not env.get('XAUTHORITY'):
            for path in (Path(f'/run/user/{os.getuid()}/gdm/Xauthority'), Path.home()/'.Xauthority'):
                if path.is_file():
                    env['XAUTHORITY'] = str(path); break
        ids = subprocess.check_output(['xdotool', 'search', '--name', '^MuJoCo : Challenge Cup Scene 2$'],
                                      env=env, text=True).split()
        if len(ids) != 1:
            raise RuntimeError('Recording requires exactly one Scene2 viewer window')
        self.stream = self.destination.with_suffix('.video.log').open('x')
        command = ['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'warning', '-n',
                   '-f', 'x11grab', '-framerate', '10', '-window_id', ids[0], '-i', env['DISPLAY'],
                   '-vf', 'scale=1280:-2', '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast',
                   '-crf', '23', '-pix_fmt', 'yuv420p', str(self.destination)]
        self.receipt = dict(window_id=ids[0], started_wall_s=time.time(), frame_rate=10,
                            file=self.destination.name, continuous=True, speed=1)
        self.process = subprocess.Popen(command, env=env, stdout=self.stream, stderr=subprocess.STDOUT)
        time.sleep(1.)
        if self.process.poll() is not None:
            self.stream.close()
            raise RuntimeError('Scene2 recording could not start; inspect '+str(self.stream.name))
        return self

    def stop(self):
        if self.process is None:
            return
        premature = self.process.poll() is not None
        if not premature:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self.process.terminate(); self.process.wait(timeout=5)
                premature = True
        self.stream.close()
        self.receipt.update(stopped_wall_s=time.time(), premature_exit=premature,
                            encoder_returncode=self.process.returncode)
        if self.destination.exists():
            digest = hashlib.sha256()
            with self.destination.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(block)
            self.receipt['sha256'] = digest.hexdigest()
            probe = subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration,size',
                                    '-of', 'json', str(self.destination)], capture_output=True, text=True)
            if probe.returncode == 0:
                self.receipt['probe'] = json.loads(probe.stdout)
        self.destination.with_suffix('.video.json').write_text(json.dumps(self.receipt, indent=2)+'\n')
