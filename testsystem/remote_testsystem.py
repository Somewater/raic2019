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

def create_host(my_token, server_type, git_branch):
    id = random.randint(0, 10000)
    r = requests.post('https://api.hetzner.cloud/v1/servers',
                      json={"name": "raic-%d" % id,
                            "server_type": server_type,
                            "location": "nbg1",
                            "start_after_create": True,
                            "image": "ubuntu-18.04",
                            "ssh_keys": ["pav@WPumb"]},
                      headers={'Authorization': 'Bearer %s' % my_token})
    j = r.json()
    print('Host response: ', j)
    return j['server']['public_net']['ipv4']['ip'], str(j['server']['id'])

def prepare_server(tmp_dir, host, my_token, server_id, commit_pairs_list, git_branch, retry: int = 0, auto_kill = False):
    run_flags = ''
    if retry:
        run_flags += '--retry %d ' % int(retry)
    tmp_file = os.path.join(tmp_dir, str(random.randint(1, 100000000)) + '.sh')
    cmd = """
apt update
apt install python3-pip cmake make gcc g++ --yes
sleep 10
pip3 install lockfile scipy numpy requests
sleep 10

mkdir repo
cd repo
git init
git pull https://somewater:Adelaida664@bitbucket.org/somewater/raic2018.git %s

echo "load localrunner"
wget http://russianaicup.ru/s/1546002449703/assets/local-runner/codeball2018-linux.tar.gz
tar -xvf codeball2018-linux.tar.gz
mv codeball2018-linux localrunner
rm codeball2018-linux.tar.gz
    """ % git_branch
    for c1, c2 in commit_pairs_list:
        for c in [c1, c2]:
            cmd += ("\ngit fetch https://somewater:Adelaida664@bitbucket.org/somewater/raic2018.git %s\n" % c)
    cmd += "\ncd\n"

    for c1, c2 in commit_pairs_list:
        cmd += ("""
git clone repo %s
cd %s
git checkout %s
cd
git clone repo %s
cd %s
git checkout %s
cd
        """ % (c1,c1,c1,c2,c2,c2))

    for i, (c1, c2) in enumerate(commit_pairs_list):
        is_first = i == 0
        is_last = i == len(commit_pairs_list) - 1
        if is_first:
            cmd += """\ntmux new-session -c . -s "raic" -d\n"""
        else:
            cmd += """\ntmux new-window\n"""

        if is_first and is_last and auto_kill:
            cmd += ("""
tmux send-keys 'cd && ./repo/testsystem/testsystem.py --p1=%s --p2=%s --server_result %s ; curl -XDELETE -H "Content-Type: application/json" -H "Authorization: Bearer %s" https://api.hetzner.cloud/v1/servers/%s' C-m \;
            """ % (c1, c2, run_flags, my_token, server_id))
        else:
            cmd += ("""
tmux send-keys 'cd && ./repo/testsystem/testsystem.py --p1=%s --p2=%s --server_result %s' C-m \;
echo "Started %s VS %s"
sleep 5
            """ % (c1, c2, run_flags, c1, c2))
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
    argparser.add_argument('--retry', default='')
    argparser.add_argument('--commits', required=True)
    argparser.add_argument('--server_type', default='cx11')
    argparser.add_argument('--git_branch', default='master')
    argparser.add_argument('--auto_kill', action='store_true')
    args = argparser.parse_args()
    print('ARGS:', args)
    my_token = 'C5iJ0HYtZD6cIHVy15tbmaHVsovO7bcp7Isb0gc7u9vZNJ578bBlo6E2arm07Uur'

    if not os.path.exists(args.tmp):
        os.mkdir(args.tmp)
    if args.host and args.server_id:
        host, server_id = args.host, args.server_id
    else:
        print('Create host automatically')
        host, server_id = create_host(my_token, args.server_type, args.git_branch)
        print('Host --host=%s --server_id=%s created' % (host, server_id))
        time.sleep(10)

    commits = []
    commit = None
    for c in args.commits.split(','):
        if commit is None:
            commit = c
        else:
            commits.append((commit, c))
            commit = None
    prepare_server(args.tmp, host, my_token, server_id, commits, args.git_branch, args.retry, args.auto_kill)

