import subprocess
import sys

r = subprocess.run([
    'git',
    'credential-store',
    'get'
], input='url=https://github.com\nprotocol=https\nhost=github.com\n', capture_output=True, text=True, cwd='.')

for line in r.stdout.strip().split('\n'):
    if '=' in line:
        k, v = line.split('=', 1)
        if k == 'password':
            print(v.strip())
            sys.exit(0)

sys.exit(1)