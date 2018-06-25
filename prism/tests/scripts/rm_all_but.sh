pwd=$PWD
echo $1
find . ! -name ${1} -type f -exec rm -f {} +
