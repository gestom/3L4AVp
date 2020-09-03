make sure the .gnu name of file matches the data like here multipersonone_two.gnu

cat sensor_fusion-8-stdout.log.filtered | awk '{print $1 " " $2 " "  $3 " " $4 " "  $5 " " $6 " " $7 " " $8 "" $9}'> multipersonone_two.txt

to create .txt with first columns for sensor fusion

in process.sh change the name of the files at ->  compute *name* 

run process.sh

