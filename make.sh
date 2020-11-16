gcc fb2ami.c -o bitbang -O3 -march=armv7 -I/opt/vc/include -L/opt/vc/lib -lbcm_host
taskset 0x8 sudo ./bitbang
