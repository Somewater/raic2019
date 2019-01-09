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

root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

def is_alive(process):
    return process.poll() is None

def prepare_player(player_root):
    os.system('cd %s/packages/c++17 && cmake . >/dev/null 2>&1 && make >/dev/null 2>&1' % player_root)

def run_processes_using_subprocess(localrunner_cmd, player1_cmd, player2_cmd = None, verbose: bool = True):
    localrunner_cmd = localrunner_cmd.replace('  ', ' ').split()
    player1_cmd = player1_cmd.split()
    if player2_cmd:
        player2_cmd = player2_cmd.split()

    localrunner = subprocess.Popen(localrunner_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    player1 = subprocess.Popen(player1_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    player2 = None
    if player2_cmd:
        player2 = subprocess.Popen(player2_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    processes = [('localrunner', localrunner), ('player1', player1)]
    if player2:
        processes.append(('player2', player2))
    time.sleep(0.1)
    start_time = time.time()
    next_check_time = start_time
    while True:
        pcompleted = None
        for name, process  in processes:
            if not is_alive(process):
                pcompleted = name
                break
        if pcompleted:
            print('Run completed, process %s stopped' % pcompleted)
            break

        for name, process  in processes:
            while True:
                if verbose and process.stdout:
                    line = process.stdout.readline().decode('UTF-8').rstrip()
                    if line:
                        print('[%s] %s' % (name, line))
                    else:
                        break
                if process.stderr:
                    line = process.stderr.readline().decode('UTF-8').rstrip()
                    if line:
                        print('[%s] %s' % (name, line), file=sys.stderr)
                    else:
                        break
        if time.time() - next_check_time > 60:
            print('Game duration %d seconds' % (time.time() - start_time))
            next_check_time = time.time()
        time.sleep(1)

def run_processes_using_system(localrunner_cmd, player1_cmd, player2_cmd = None, verbose: bool = True):
    if not verbose:
        devnull = ' >/dev/null 2>&1'
        localrunner_cmd += devnull
        player1_cmd += devnull
        if player2_cmd:
            player2_cmd += devnull
    if player2_cmd:
        cmd = " (sleep 2 && %s) & (sleep 2 && %s) & (%s )" % (player1_cmd, player2_cmd, localrunner_cmd)
    else:
        cmd = "(sleep 2 && %s) & (%s)" % (player1_cmd, localrunner_cmd)
    #print(cmd)
    os.system(cmd)

def run_game(tmp_filepath, player1_root, player2_root = None, verbose: bool = True) -> Tuple[int, int]:
    uid = random.randint(0, 1000000)
    result_filepath = os.path.join(tmp_filepath, 'result_%d.txt' % uid)
    player1_port = 30000 + random.randint(1, 10000)
    player2_port = 30000 + random.randint(1, 10000)
    noshow = True
    until_first_goal = True
    flags = ''
    if noshow:
        flags += '--noshow '
    if until_first_goal:
        flags += '--until-first-goal '
    localrunner_cmd = '%s/localrunner/codeball2018 --results-file=%s %s --p1=tcp-%d' % \
                      (root, result_filepath, flags, player1_port)
    localrunner_cmd = localrunner_cmd.replace('  ', ' ')
    if player2_root:
        localrunner_cmd += ' --p2=tcp-%d' % player2_port
    player1_cmd = '%s/packages/c++17/MyStrategy localhost %d 0000000000000000' % (player1_root, player1_port)
    player2_cmd = '%s/packages/c++17/MyStrategy localhost %d 0000000000000000' % (player2_root, player2_port) if player2_root else None

    run_processes_using_system(localrunner_cmd, player1_cmd, player2_cmd, verbose)

    if os.path.exists(result_filepath):
        scores = []
        with open(result_filepath) as f:
            for l in f:
                if ':' in l:
                    score = int(l.split(':')[1])
                    scores.append(score)
                    if len(scores) == 2:
                        break
        if len(scores) == 2:
            return (scores[0], scores[1])
        else:
            print('Unexpected result file format in %s:\n' % result_filepath)
            with open(result_filepath) as f:
                for l in f:
                    print('>', l.strip())
        os.unlink(result_filepath)
    else:
        print('Result file %s not found' % result_filepath)

def confident(a, alpha=0.99):
    mean = np.mean(a)
    stddev = np.std(a, ddof=1)
    t_bounds = stats.t.interval(alpha, len(a) - 1)
    ci = [mean + critval * stddev / sqrt(len(a)) for critval in t_bounds]
    return (ci[0], ci[1])

def get_commit_name(repo_filepath):
    if not repo_filepath:
        return
    process = subprocess.Popen('git log -1 --pretty=%B'.split(), stdout=subprocess.PIPE, cwd=repo_filepath)
    message = ''
    for l in process.stdout.readlines():
        l = l.decode('UTF-8').strip()
        if len(l) > 0:
            if message:
                message += ' '
            message += l
    return message

def show_stat(all_results_filepath):
    c = defaultdict(list)
    with open(all_results_filepath) as f:
        for l in f:
            n1, c1, r1, n2, c2, r2 = l.split('\t')
            if n1 > n2:
                n2, c2, r2, n1, c1, r1 = (n1, c1, r1, n2, c2, r2)
            score = 0
            if r1 > r2:
                score = 1
            else:
                score = -1
            c[(n1, n2)].append(score)
    h = []
    for key, vs in c.items():
        r1, r2 = confident(vs)
        n1, n2 = key
        v = ' '
        if r1 < 0 and r2 < 0:
            v = '-'
            h.append((n2, n1))
        if r1 > 0 and r2 > 0:
            v = '+'
            h.append((n1, n2))
        # print(v, key, (r1, r2), len(vs))
    less_to_better = defaultdict(set)
    for better, less in h:
        less_to_better[less].add(better)
    names = list(set([n for n1, n2 in h for n in [n1, n2]]))
    changed = True
    while changed:
        changed = False
        for i1, n1 in enumerate(names):
            for i2, n2 in enumerate(names):
                if n1 != n2:
                    if n1 in less_to_better[n2] and i1 > i2:
                        names[i1] = n2
                        names[i2] = n1
                        changed = True
                        break
            if changed:
                break
    print('From best to worst:')
    for name in names:
        print(' ', name)
    return c

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='Game testing system CLI')
    argparser.add_argument('--tmp', default='/var/tmp/raic2018')
    argparser.add_argument('--all_results', default='~/all_results.txt')
    argparser.add_argument('--p1', default='.')
    argparser.add_argument('--p2', default='')
    argparser.add_argument('--n1', default='')
    argparser.add_argument('--n2', default='')
    argparser.add_argument('--verbose', action='store_true')
    argparser.add_argument('--retry', default='100')
    argparser.add_argument('--stat_exit', action='store_true')
    argparser.add_argument('--stat', action='store_true')
    args = argparser.parse_args()
    print('ARGS:', args)

    if args.stat:
        show_stat(args.all_results)
        sys.exit(0)

    lock = LockFile(os.path.join(args.tmp, 'lock'))
    all_results_filepath = args.all_results.replace('~', os.getenv('HOME')).rstrip('/')

    player1_root = args.p1.replace('~', os.getenv('HOME')).rstrip('/')
    if player1_root == '.':
        player1_root = root
    player2_root = args.p2.replace('~', os.getenv('HOME')).rstrip('/')
    if player2_root == '.':
        player2_root = root
    payer1_commit = get_commit_name(player1_root)
    payer2_commit = get_commit_name(player2_root)
    player1_name = args.n1 or payer1_commit or player1_root
    player2_name = args.n2 or payer2_commit or player2_root or '<helper>'
    print('Players: %s (%s) VS %s (%s)' % (player1_name, payer1_commit, player2_name, payer2_commit))

    prepare_player(player1_root)
    if player2_root and player2_root != player1_root:
        prepare_player(player2_root)
    print('Player startegies compiled')

    if not os.path.exists(args.tmp):
        os.mkdir(args.tmp)

    scores = []
    for i in range(int(args.retry)):
        result = run_game(args.tmp, player1_root, player2_root, args.verbose)
        if result:
            r1, r2 = result
            with lock:
                with open(all_results_filepath, 'a') as f:
                    w = csv.writer(f, delimiter='\t')
                    cols = [player1_name, payer1_commit or '', r1, player2_name, payer2_commit or '', r2]
                    w.writerow([str(c) for c in cols])
            score = 0
            if r1 > r2:
                score = 1
            elif r1 < r2:
                score = -1
            scores.append(score)
            r1, r2 = confident([0] + scores)
            print('%d. Run result: %s, score: %f, confidence: [%f : %f]' % (i, repr(result), score, r1, r2))
            if r1 > 0:
                print('p1 is better')
                if args.stat_exit:
                    sys.exit(0)
            elif r2 < 0:
                print('p2 is better')
                if args.stat_exit:
                    sys.exit(0)
        else:
            print('%d. Run result: %s' % (i, repr(result)))
