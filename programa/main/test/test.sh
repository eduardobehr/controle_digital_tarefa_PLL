#! /bin/sh

SOURCE=test_interpreter_x64.c
TARGET=test.elf

cd $(dirname $0)                        \
&& gcc -Wall -g -o $TARGET $SOURCE      \
&& ./$TARGET