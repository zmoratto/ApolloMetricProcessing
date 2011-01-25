#!/usr/bin/env python

"""Convert all TIFF files output by reconstruct into JPEG files viewable
in a web browser for debugging."""

import os
import stat
import re
import tempfile

BASE_DIR = '%s/a15' % os.environ['HOME']
RESULTS_DIR = '%s/results/a15/sub16/albedo' % os.environ['HOME']

COLORMAP = '%s/projects/VisionWorkbench/build/x86_64_linux_gcc4.1/bin/colormap' % os.environ['HOME']

def getTime(f):
    try:
        s = os.stat(f)
    except OSError:
        return 0
    return s[stat.ST_MTIME]

def dosys(cmd, stopOnErr=False):
    print cmd
    ret = os.system(cmd)
    if stopOnErr and ret != 0:
        raise Exception('command exited with non-zero return value %d' % ret)
    return ret

class JobQueue(object):
    def __init__(self, numProcessors=1, deleteTmpFiles=True):
        self.numProcessors = numProcessors
        self.deleteTmpFiles = deleteTmpFiles
        self.jobs = []

    def addJob(self, step, job):
        self.jobs.append((step, job))

    def run(self):
        maxStep = max(*[step for step, job in self.jobs])
        for currentStep in xrange(0, maxStep+1):
            print '\n*** JobQueue: running step %d of %d ***\n' % (currentStep+1, maxStep+1)
            jobsForStep = [job for jobStep, job in self.jobs
                           if jobStep == currentStep]
            fd, jobsFileName = tempfile.mkstemp('jobQueueStep%d.txt' % currentStep)
            os.close(fd)
            jobsFile = file(jobsFileName, 'w')
            for job in jobsForStep:
                jobsFile.write('%s\n' % job)
            jobsFile.close()
            dosys('cat %s | xargs -t -P%d -I__cmd__ bash -c __cmd__' % (jobsFileName, self.numProcessors),
                  stopOnErr=True)
            if self.deleteTmpFiles:
                dosys('rm -f %s' % jobsFileName)

jobQueueG = None

def convert(tif, opts):
    stem, suffix = os.path.splitext(tif)
    out = stem + '.jpg'
    if getTime(tif) > getTime(out) or opts.force:
        if re.search('dem', tif, re.IGNORECASE):
            # file is a 16-bit TIFF DEM -- colormap to tif then convert to jpg
            tmp = stem + '_colormap.tif'
            jobQueueG.addJob(0, '%s %s -o %s' % (COLORMAP, tif, tmp))
            jobQueueG.addJob(1, 'convert %s %s' % (tmp, out))
            jobQueueG.addJob(2, 'rm -f %s' % tmp)
            return 1
        else:
            # file is an 8-bit TIFF -- contrast stretch and convert to jpeg
            jobQueueG.addJob(0, 'convert -contrast-stretch 0x0 %s %s' % (tif, out))
            return 1
    return 0

def clean(tif):
    stem, suffix = os.path.splitext(tif)
    out = stem + '.jpg'
    if os.path.exists(out):
        dosys('rm -f %s' % out)

def doit(opts):
    global jobQueueG
    jobQueueG = JobQueue(opts.numProcessors)

    tiffsName = '/tmp/convertAllTiffs.txt'
    dosys('find %s -name *.tif > %s' % (RESULTS_DIR, tiffsName))
    tiffsFile = file(tiffsName, 'r')
    tiffs = [line[:-1] for line in tiffsFile]
    tiffsFile.close()
    tiffs.sort()

    if opts.clean:
        for t in tiffs:
            clean(t)
    else:
        numConverted = 0
        for t in tiffs:
            numConverted += convert(t, opts)
        jobQueueG.run()
        print '%d of %d images were up to date' % (len(tiffs) - numConverted, len(tiffs))

def main():
    import optparse
    parser = optparse.OptionParser('usage: convertAll.py')
    parser.add_option('-f', '--force',
                      action='store_true', default=False,
                      help='Convert all files, even if they are up to date')
    parser.add_option('-P', '--numProcessors',
                      type='int', default=1,
                      help='Specify N > 1 to run convert on multiple processors')
    parser.add_option('-c', '--clean',
                      action='store_true', default=False,
                      help='Remove all convert-generated jpegs')
    opts, args = parser.parse_args()
    doit(opts)

if __name__ == '__main__':
    main()
