#!/usr/bin/env python

import os
import sys
import optparse

VERBOSE = True

def read_hexfile(fname):
    if VERBOSE:
        print('Reading from {0}'.format(fname))
    res = bytearray()
    with open(fname, 'r') as read_file:
        for line in read_file:
            l=line.split(" ");
            for i in range(1, 9):
                #print(l[i])
                b = bytearray.fromhex(l[i])
                r = b[::-1]
                res += r
    if VERBOSE:
        print('  {0} bytes read'.format(len(res)))
    return res

def write_binfile(fname, data):
    if VERBOSE:
        print('Writing to {0}'.format(fname))
    with open(fname, 'wb') as outf:
        outf.write(data)
        outf.close()
    if VERBOSE:
        print('  {0} bytes written'.format(len(data)))

def main(input, output):
    data = read_hexfile(input)
    write_binfile(output, data)

if __name__=="__main__":
    args = sys.argv[1:]
    if len(args) == 2:
        main(args[0], args[1])
