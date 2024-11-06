#! /bin/bash

cd ../..

for ((i=0; i<1000; i++)) do
  if ! newt test @apache-mynewt-core/kernel/os/selftest
  then
    exit 1
  fi
done
