#!/bin/bash
#export ROS_DOMAIN_ID=100          # ← add this line

# Check if an argument was provided for the number of agents
if [ -z "$1" ]; then
  echo "No argument provided. Please provide a value for 'n_agent'."
  exit 1
fi

# Read the argument and echo it
n_agent=$1
echo "**********************The number of requested agents is: $n_agent****************************"

# Loop to launch each PX4 instance in a new `terminator` tab
for (( i=1; i<=n_agent; i++ )); do
  echo "Starting PX4 with instance ID: $i in a new terminal tab"
  j=$(( -3*i + 1 ))
  # Open a new terminator tab and run the PX4 command with the current instance ID
  terminator --new-tab -x bash -c "
    export ROS_DOMAIN_ID=100     

    cd ~/uav/PX4-Autopilot || { echo 'Directory not found'; exec bash; }
    PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="$i,$j" PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=uav ./build/px4_sitl_default/bin/px4 -i $i || echo 'Failed to start PX4 with instance ID: $i'
    exec bash" &

  # Optional delay to prevent overlapping launches 
  sleep 10
done

echo "All PX4 instances launched (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)."
