#!/bin/sh

lock_atag() {
	python3 get_pic1.py nowait
	python3 find_at6.py nowait


	while [ "`cat st_argv_for_arm.txt`" != "20 0 0 0" ]
	do
	  echo "============="

	  if [ "$st1" != "20 0 0 0" ]; then
	    python3 pub5.py "$(cat st_argv_for_arm.txt)"
	    python3 get_pic1.py nowait
	    python3 find_at6.py nowait    
	  fi

	done
}

lock_atag
python pub5.py "20 0 0 -40"
lock_atag

