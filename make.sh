gcc fb2ami.c ps_protocol.c -o fb2ami -O3 -march=armv7 -I/opt/vc/include -L/opt/vc/lib -lbcm_host
taskset 0x8 sudo ./fb2ami
