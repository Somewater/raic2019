#!/usr/bin/env python3
import os
from datetime import datetime
import zipfile

SkippedFiles = {'RemoteProcessClient.py', 'Runner.py'}
root = os.path.dirname(os.path.realpath(__file__))
output_dir = os.path.join(root, 'my_startegy')
src_dir = os.path.join(root, 'packages', 'python3')
if not os.path.exists(output_dir):
    os.mkdir(output_dir)

file_num = 1
def generate_next_filepath():
    global file_num
    fn = os.path.join(output_dir, 'startegy_%d.zip' % file_num)
    file_num += 1
    return fn

output_archive = generate_next_filepath()
while os.path.exists(output_archive):
    output_archive = generate_next_filepath()

zip = zipfile.ZipFile(output_archive, 'w', zipfile.ZIP_DEFLATED)
for file in os.listdir(src_dir):
    if file.endswith('.py') and file not in SkippedFiles:
        zip.write(os.path.join(src_dir, file), file)
        print('\tadd %s' % file)
zip.close()

print('%s created' % (output_archive))
