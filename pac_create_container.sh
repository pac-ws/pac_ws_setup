ORIG_INPUT_PARAMS="$@"
params="$(getopt -o d:n: -l directory: -l ns: -l name: --name "$(basename "$0")" -- "$@")"

if [ $? -ne 0 ]
then
  echo "Parameter error"
  print_usage
fi

print_usage() {
  printf "bash $0 [-d|--directory <workspace directory>]\n"
}

eval set -- "$params"
unset params

IMAGE_BASE_NAME=agarwalsaurav/pac
IMAGE_TAG=latest
# Check if the OS architecture is arm64
if [ "$(uname -m)" == "aarch64" ]; then
  IMAGE_TAG=arm64
fi
IMAGE_NAME="${IMAGE_BASE_NAME}:${IMAGE_TAG}"
docker pull ${IMAGE_NAME}
WS_DIR="/data/pac_ws"

while true; do
  case ${1} in
    -d|--directory) WS_DIR=("${2}");shift 2;;
    -n|--name) CONTAINER_NAME=("${2}");shift 2;;
    --ns) ROS_NAMESPACE=("${2}");shift 2;;
    --) shift;break;;
    *) print_usage
      exit 1 ;;
  esac
done

CONTAINER_CC_WS="/workspace"

if [ -z ${WS_DIR} ]; then
  VOLUME_OPTION=""
else
  VOLUME_OPTION="-v ${WS_DIR}:${CONTAINER_CC_WS}:rw"
fi



if [ -z ${CONTAINER_NAME} ]; then
  CONTAINER_NAME="pac-${HOSTNAME}"
fi

docker run -it \
  --name=${CONTAINER_NAME} \
  ${CONTAINER_OPTIONS} \
  --net=host \
  --privileged \
  --ipc=host \
  --restart=always \
  --pid=host \
  --env=ROS_NAMESPACE=${ROS_NAMESPACE} \
  ${VOLUME_OPTION} \
  --workdir=${CONTAINER_CC_WS} \
  ${IMAGE_NAME} \
  bash
