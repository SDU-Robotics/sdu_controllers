#!/usr/bin/env bash

SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
  SOURCE=$(readlink "$SOURCE")
  [[ $SOURCE != /* ]] && SOURCE=$DIR/$SOURCE # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )


for entry in $(find $DIR/src -print | grep -e ".*\.[ch]pp")
do
  clang-format -i $entry
done

for entry in $(find $DIR/include -print | grep -e ".*\.[ch]pp")
do
  clang-format -i $entry
done

for entry in $(find $DIR/examples -print | grep -e ".*\.[ch]pp")
do
  clang-format -i $entry
done

for entry in $(find $DIR/tests -print | grep -e ".*\.[ch]pp")
do
  clang-format -i $entry
done
