#!/bin/bash
pid=$(ps -elf|grep athletics_perform.py|grep -v grep|grep -v sh |awk '{print $4}')
kill -9 $pid


