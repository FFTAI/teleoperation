export HYDRA_FULL_ERROR=1

robot_type=$1
task=$2
camera=$3
# machine_id=$(hostname | cut -d '-' -f 2)
machine_id=$(hostname)

shift 3
echo "-----------------------------------"
echo "Running Data Collection..."
echo "Task: ${task}"
echo "Machine ID: ${machine_id}"
echo "Robot Type: ${robot_type}"
echo "Camera: ${camera}"
echo "-----------------------------------"
if [[ ! "$robot_type" =~ ^(gr1t1|gr1t2|gr2t2)$ ]]; then
    echo "Invalid robot type. Please use gr1t1, gr1t2, or gr2t2."
    exit 1
fi

python -m teleoperation --config-name daq \
    robot=${robot_type}_legacy \
    robot.instance.namespace="gr/daq-${machine_id}" \
    task_name=factory_$(date +%m_%d)_"${task}" \
    camera="${camera}" \
    hand=fourier_dexpilot_dhx \
    gravity_compensation=false \
    robot.visualize=true \
    ${@}
