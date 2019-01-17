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
import http.server
import socketserver
import requests
import threading
import math

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

def run_game(result_filepath, player1_root, player2_root = None, verbose: bool = True) -> Tuple[int, int]:
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

def show_stat(all_results_filepath, alpha):
    c = defaultdict(list)
    with open(all_results_filepath) as f:
        for l in f:
            n1, c1, r1, n2, c2, r2 = l.split('\t')
            if n1 > n2:
                n2, c2, r2, n1, c1, r1 = (n1, c1, r1, n2, c2, r2)
            score = 0
            if r1 > r2:
                score = 1
            elif r1 < r2:
                score = -1
            c[(n1, n2)].append(score)
    h = []
    for key, vs in c.items():
        r1, r2 = confident(vs, alpha)
        n1, n2 = key
        v = ' '
        if r1 < 0 and r2 < 0:
            v = '-'
            h.append((n2, n1))
        if r1 > 0 and r2 > 0:
            v = '+'
            h.append((n1, n2))
        print(v, key, (r1, r2), len(vs))
    less_to_better = defaultdict(set)
    for better, less in h:
        less_to_better[less].add(better)
    names = list(set([n for n1, n2 in h for n in [n1, n2]]))
    changed = True
    while changed:
        changed = False
        for i1, n1 in enumerate(names):
            for i2, n2 in enumerate(names):
                if i1 != i2:
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

def server_daemon(all_results_filepath, lock):
    class HandlerMy(http.server.BaseHTTPRequestHandler):
        def handle_http(self, status_code):
            self.send_response(status_code)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

        def respond(self, text):
            self.handle_http(200)
            self.wfile.write(bytes(text, 'UTF-8'))

        def do_GET(self):
            self.respond('OK')

        def do_POST(self):
            text = self.rfile.read(int(self.headers['Content-Length'])).decode('UTF-8')
            print('RESULT:', text)
            lines_written = 0
            with lock:
                with open(all_results_filepath, 'a') as f:
                    for line in text.strip().splitlines():
                        f.write('%s\n' % line.strip())
                        lines_written += 1
            self.respond('%d lines written' % lines_written)


    with socketserver.TCPServer(("0.0.0.0", 14526), HandlerMy) as httpd:
        httpd.serve_forever()

def read_result(result_filepath):
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

class DummyThread:
    def join(self):
        pass

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='Game testing system CLI')
    argparser.add_argument('--tmp', default='/var/tmp/raic2018')
    argparser.add_argument('--all_results', default='~/all_results.txt')
    argparser.add_argument('--p1', default='.')
    argparser.add_argument('--p2', default='')
    argparser.add_argument('--n1', default='')
    argparser.add_argument('--n2', default='')
    argparser.add_argument('--alpha', default='0.99')
    argparser.add_argument('--verbose', action='store_true')
    argparser.add_argument('--retry', default='1000')
    argparser.add_argument('--stat_exit', action='store_true')
    argparser.add_argument('--stat', action='store_true')
    argparser.add_argument('--server_daemon', action='store_true')
    argparser.add_argument('--server_result', action='store_true')
    argparser.add_argument('--parallel', default='1')
    args = argparser.parse_args()
    print('ARGS:', args)

    lock = LockFile(os.path.join(args.tmp, 'lock'))
    all_results_filepath = args.all_results.replace('~', os.getenv('HOME')).rstrip('/')
    alpha = float(args.alpha)

    if args.server_daemon:
        server_daemon(all_results_filepath + '_server.txt', lock)
        sys.exit(0)
    elif args.stat:
        show_stat(all_results_filepath, alpha)
        sys.exit(0)

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

    thread_count = int(args.parallel) if args.parallel else 1
    retry_count = int(args.retry)
    retry_batch_count = math.ceil(retry_count / thread_count)
    scores = []
    count = 0
    i = 0
    for _ in range(retry_batch_count):
        game_threads = []
        for _ in range(thread_count):
            count += 1
            if count > retry_count:
                break
            result_filepath = os.path.join(args.tmp, 'result_%d.txt' % random.randint(0, 1000000))
            if thread_count > 1:
                t = threading.Thread(target=run_game, args=(result_filepath, player1_root, player2_root, args.verbose))
                t.start()
            else:
                run_game(result_filepath, player1_root, player2_root, args.verbose)
                t = DummyThread()
            game_threads.append((t, result_filepath))

        for t, result_filepath in game_threads:
            t.join()
            result = read_result(result_filepath)
            if result:
                r1, r2 = result
                cols = [player1_name, payer1_commit or '', r1, player2_name, payer2_commit or '', r2]
                cols = [str(c) for c in cols]
                if args.server_result:
                    requests.post('http://atlantor.ru:14526', data="\t".join(cols))
                else:
                    with lock:
                        with open(all_results_filepath, 'a') as f:
                            w = csv.writer(f, delimiter='\t')
                            w.writerow(cols)
                score = 0
                if r1 > r2:
                    score = 1
                elif r1 < r2:
                    score = -1
                scores.append(score)
                r1, r2 = confident([0] + scores, alpha)
                print('%d. Run result: %s, score: %f, confidence: [%f : %f]' % (i, repr(result), score, r1, r2))
                i += 1
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
                i += 1
