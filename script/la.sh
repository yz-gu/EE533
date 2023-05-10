#!/bin/bash

file_name=readfile.txt
sudo rm $file_name
sudo touch $file_name
sudo chmod 777 $file_name
# printf "%8s   %8s   %3s   %3s   %16s   %16s   %16s   %5s   %5s   %5s   %5s\n\n" "sl_match" "match" "tmp" "pc" "rddata" "rdata" "wdata" "rctrl" "wctrl" "raddr" "waddr" >> $file_name
printf "%5s   %4s   %5s   %5s   %2s   %3s   %16s   %16s   %16s   %5s   %5s   %5s   %5s\n\n" "IDins" "wea" "match" "state" "we" "pc" "rddata" "rdata" "wdata" "rctrl" "wctrl" "raddr" "waddr" >> $file_name
# address_0=0x2000218
# address_1=0x200021c
address_0=0x2000220
address_1=0x2000224
address_2=0x2000228
address_3=0x200022c
address_4=0x2000230
address_5=0x2000234
address_6=0x2000238
address_7=0x200023c

for i in {0..255}
do
    cmd=$(printf "0x80%02x0000" $i)
    regwrite 0x2000200 $cmd

    temp_file_0=$(sudo mktemp)
    sudo chmod 777 $temp_file_0
    regread $address_0 >> $temp_file_0
    word_0=$(cat $temp_file_0 | cut -c32-39)

    temp_file_1=$(sudo mktemp)
    sudo chmod 777 $temp_file_1
    regread $address_1 >> $temp_file_1
    word_1=$(cat $temp_file_1 | cut -c32-39)

    temp_file_2=$(sudo mktemp)
    sudo chmod 777 $temp_file_2
    regread $address_2 >> $temp_file_2
    word_2=$(cat $temp_file_2 | cut -c32-39)

    temp_file_3=$(sudo mktemp)
    sudo chmod 777 $temp_file_3
    regread $address_3 >> $temp_file_3
    word_3=$(cat $temp_file_3 | cut -c32-39)

    temp_file_4=$(sudo mktemp)
    sudo chmod 777 $temp_file_4
    regread $address_4 >> $temp_file_4
    word_4=$(cat $temp_file_4 | cut -c32-39)

    temp_file_5=$(sudo mktemp)
    sudo chmod 777 $temp_file_5
    regread $address_5 >> $temp_file_5
    word_5=$(cat $temp_file_5 | cut -c32-39)

    temp_file_6=$(sudo mktemp)
    sudo chmod 777 $temp_file_6
    regread $address_6 >> $temp_file_6
    word_6=$(cat $temp_file_6 | cut -c32-39)

    temp_file_7=$(sudo mktemp)
    sudo chmod 777 $temp_file_7
    regread $address_7 >> $temp_file_7
    word_7=$(cat $temp_file_7 | cut -c32-39)

    printf "%5s   %4s   %5s   %5s   %2s   %3s   %16s   %16s   %16s   %5s   %5s   %5s   %5s\n" "${word_7:0:1}" "${word_7:1:1}" "${word_7:2:1}" "${word_7:3:1}" "${word_7:4:1}" "${word_7:5:3}" "$word_6$word_5" "$word_4$word_3" "$word_2$word_1" "${word_0:0:2}" "${word_0:2:2}" "${word_0:4:2}" "${word_0:6:2}" >> $file_name

    sudo rm $temp_file_0
    sudo rm $temp_file_1
    sudo rm $temp_file_2
    sudo rm $temp_file_3
    sudo rm $temp_file_4
    sudo rm $temp_file_5
    sudo rm $temp_file_6
    sudo rm $temp_file_7

done