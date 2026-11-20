SSHPASS="systemcore"
ping -c 3 10.1.0.2
echo "ping successful"
sshpass -p "systemcore" scp rileys_virus.sh systemcore@10.1.0.2:/home/systemcore/
echo "copy successful"
sshpass -e ssh systemcore@10.1.0.2 "./home/systemcore/rileys_virus.sh"  
echo "ssh successful"
