# set -x
sudo /usr/local/netfpga/lib/scripts/cpci_reprogram/cpci_reprogram.pl --all

input="/proj/USCEE533/exp/lab7-gyz/tbdata/tbreport.log"
IFS=$'\n' read -d '' -ra lines < "$input"
for i in {0..70}
do
	if [ "${lines[$i]}" = "Physical Lan/Link Mapping:" ]; then
		for j in 5 9 13 17
		do
			read -ra arr <<<"${lines[$((i+j))]}"
			sudo /sbin/ifconfig nf2c${arr[3]: -1} ${arr[2]}/24 up
		done
	fi
done

nf_download /usr/local/netfpga/bitfiles/reference_nic.bit
