#!/usr/bin/env python3

import argparse
import os
import sys
from typing import Tuple
import subprocess
import random
import time
from scipy import stats
import numpy as np
from math import *
from lockfile import LockFile
import csv
from collections import *
import requests

root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def create_host(my_token):
    id = random.randint(0, 10000)
    r = requests.post('https://api.hetzner.cloud/v1/servers',
                      json={"name": "raic-%d" % id,
                            "server_type": "cx11",
                            "location": "nbg1",
                            "start_after_create": True,
                            "image": "ubuntu-18.04",
                            "ssh_keys": ["pav@WPumb"]},
                      headers={'Authorization': 'Bearer %s' % my_token})
    j = r.json()
    print('Host response: ', j)
    return j['server']['public_net']['ipv4']['ip'], str(j['server']['id'])

def prepare_server(tmp_dir, host, c1, c2, my_token, server_id):
    tmp_file = os.path.join(tmp_dir, str(random.randint(1, 100000000)) + '.sh')
    cmd = """
apt install python3-pip cmake make gcc g++ --yes
pip3 install lockfile scipy numpy requests

mkdir repo
cd repo
git init
git pull https://somewater:Adelaida664@bitbucket.org/somewater/raic2018.git master
git fetch https://somewater:Adelaida664@bitbucket.org/somewater/raic2018.git %s %s

echo "load localrunner"
wget http://russianaicup.ru/s/1546002449703/assets/local-runner/codeball2018-linux.tar.gz
tar -xvf codeball2018-linux.tar.gz
mv codeball2018-linux localrunner
rm codeball2018-linux.tar.gz
cd

git clone repo %s
cd %s
git checkout %s
cd
git clone repo %s
cd %s
git checkout %s
cd

tmux new-session -c . -s "raic" -d \; send-keys 'cd && ./repo/testsystem/testsystem.py --p1=%s --p2=%s --server_result ; curl -XDELETE -H "Content-Type: application/json" -H "Authorization: Bearer %s" https://api.hetzner.cloud/v1/servers/%s' C-m \;
    """.strip() % (c1, c2, c1, c1, c1, c2, c2, c2, c1, c2, my_token, server_id)
    print(cmd)
    with open(tmp_file, 'w') as f:
        f.write(cmd)
    os.system('scp -o "StrictHostKeyChecking no" %s root@%s:~/setup.sh' % (tmp_file, host))
    os.system('ssh -o "StrictHostKeyChecking no" root@%s "chmod +x setup.sh && ./setup.sh"' % host)
    os.unlink(tmp_file)

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='Game testing system CLI')
    argparser.add_argument('--tmp', default='/var/tmp/raic2018')
    argparser.add_argument('--host', default='')
    argparser.add_argument('--server_id', default='')
    argparser.add_argument('--c1', required=True)
    argparser.add_argument('--c2', required=True)
    args = argparser.parse_args()
    print('ARGS:', args)
    my_token = 'C5iJ0HYtZD6cIHVy15tbmaHVsovO7bcp7Isb0gc7u9vZNJ578bBlo6E2arm07Uur'

    if args.host and args.server_id:
        host, server_id = args.host, args.server_id
    else:
        print('Create host automatically')
        host, server_id = create_host(my_token)
        print('Host %s server_id=%s created' % (host, server_id))

    prepare_server(args.tmp, host, args.c1, args.c2, my_token, server_id)

