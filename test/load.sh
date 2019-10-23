#!/bin/bash

rm /media/lucas/BA4E-0EA5/*
ls SD_test_gcodes/ | sort -n -r | xargs -n 1 bash -c 'cp SD_test_gcodes/$0 /media/lucas/BA4E-0EA5/'
