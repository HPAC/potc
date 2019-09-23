 (( speaker-test -t sine -f 333 > /dev/null)& pid=$! ; sleep 0.1s ; kill -9 $pid)
