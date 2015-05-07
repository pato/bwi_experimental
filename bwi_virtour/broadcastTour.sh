hostname=$(hostname)
link="http://nixons-head.csres.utexas.edu/tours"
echo "Tour started on $hostname\nCheck it out at: $link" | mail -s "[bwi_virtour] Tour started on $hostname" plankenau@gmail.com
